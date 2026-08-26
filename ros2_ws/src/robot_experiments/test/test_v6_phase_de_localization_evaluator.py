import json
import math
from types import SimpleNamespace

import pytest

import robot_experiments.v6_localization_causal_evaluator as evaluator
from robot_experiments.v6_localization_causal_evaluator import (
    EvaluationError,
    PHASE_DE_EVENT_SCHEMA,
    PHASE_DE_GT_SCHEMA,
    evaluate_phase_de_episode,
    evaluate_phase_de_pair,
    evaluate_phase_de_pair_files,
    extract_phase_de_ground_truth,
)


def _event(phase, arm, seed, event, stamp_s, **values):
    return {
        "schema": PHASE_DE_EVENT_SCHEMA,
        "run_id": f"phase-{phase.lower()}-{arm.lower()}-{seed}",
        "phase": phase,
        "arm": arm,
        "seed": seed,
        "event": event,
        "stamp_s": stamp_s,
        **values,
    }


def _gt(phase, arm, seed, stamp_s, *, x, region_id="east"):
    return {
        "schema": PHASE_DE_GT_SCHEMA,
        "run_id": f"phase-{phase.lower()}-{arm.lower()}-{seed}",
        "phase": phase,
        "arm": arm,
        "seed": seed,
        "event": "ground_truth_pose",
        "stamp_s": stamp_s,
        "x": x,
        "y": 0.0,
        "yaw_deg": 0.0,
        "region_id": region_id,
    }


def _episode(phase, arm, seed=9001, *, ready_s=2.0, recover_s=None):
    events = [_event(phase, arm, seed, "episode_start", 0.0)]
    if phase == "D":
        if arm == "S1":
            events.append(
                _event(phase, arm, seed, "prior_write", 0.5, source="supervisor", count=1)
            )
            source = "supervisor"
        else:
            source = "runner"
        events.append(
            _event(phase, arm, seed, "initialpose", 0.6, source=source, count=1)
        )
        events.extend(
            [
                _event(phase, arm, seed, "localization_ready", ready_s, ready=True),
                _event(
                    phase,
                    arm,
                    seed,
                    "supervisor_diagnostic",
                    ready_s,
                    mode="startup" if arm == "S1" else "shadow",
                    state="READY",
                    reason="candidate_accepted" if arm == "S1" else "amcl_ready",
                    result="applied" if arm == "S1" else "observed",
                    reset_attempts=0,
                ),
                _event(phase, arm, seed, "goal_dispatched", 4.0, leg_id="G1_G2"),
                _event(
                    phase, arm, seed, "goal_result", 7.0,
                    leg_id="G1_G2", state="SUCCEEDED"
                ),
            ]
        )
    else:
        events.extend(
            [
                _event(
                    phase, arm, seed, "initialpose", 0.4,
                    source="runner", count=1
                ),
                _event(phase, arm, seed, "localization_ready", ready_s, ready=True),
                _event(
                    phase, arm, seed, "supervisor_diagnostic", 1.1,
                    mode="active" if arm == "R1" else "shadow",
                    state="NORMAL", reason="amcl_healthy", result="observed",
                    reset_attempts=0,
                ),
                _event(phase, arm, seed, "goal_dispatched", 1.2, leg_id="G2"),
                _event(
                    phase, arm, seed, "goal_result", 1.8,
                    leg_id="G2", state="SUCCEEDED"
                ),
                _event(phase, arm, seed, "pause_requested", 1.9),
                _event(
                    phase, arm, seed, "cmd_vel_sim", 2.0, zero=True,
                    linear_x=0.0, linear_y=0.0, angular_z=0.0,
                ),
                _event(
                    phase, arm, seed, "pause_confirmed", 2.1,
                    cmd_vel_sim_zero=True, stationary=True,
                ),
                _event(
                    phase, arm, seed, "particle_cloud", 2.15,
                    count=1, particle_count=2000, post_fault=True,
                ),
                _event(
                    phase,
                    arm,
                    seed,
                    "fault_injected",
                    2.2,
                    fault_id="F2",
                    kind="amcl_global_localization_particle_spread",
                    service="/reinitialize_global_localization",
                    service_request_count=1,
                    service_response_observed=True,
                    first_post_fault_amcl_pose_observed=True,
                    outcome="FAULT_DISCRIMINATIVE",
                ),
            ]
        )
        if arm == "R1":
            events.extend(
                [
                    _event(
                        phase, arm, seed, "manual_rescue_requested", 2.7,
                        count=1,
                        purpose="ENGINEERING_EXPLICIT_MANUAL_RECOVERY_ONLY",
                    ),
                    _event(
                        phase,
                        arm,
                        seed,
                        "prior_write",
                        2.8,
                        source="supervisor",
                        count=1,
                    ),
                    _event(
                        phase,
                        arm,
                        seed,
                        "initialpose",
                        2.9,
                        source="supervisor",
                        count=2,
                    ),
                ]
            )
        if recover_s is not None:
            events.extend(
                [
                    _event(
                        phase, arm, seed, "localization_recovered", recover_s,
                        success=True,
                    ),
                    _event(
                        phase, arm, seed, "supervisor_diagnostic", recover_s,
                        mode="active" if arm == "R1" else "shadow",
                        state="RECOVERED", reason="candidate_accepted",
                        result="applied" if arm == "R1" else "observed",
                        reset_attempts=1 if arm == "R1" else 0,
                    ),
                ]
            )
            for index, leg_id in enumerate(("G3", "G4", "G5", "G1")):
                dispatch_s = recover_s + 0.1 + 0.5 * index
                events.extend(
                    [
                        _event(
                            phase, arm, seed, "goal_dispatched", dispatch_s,
                            leg_id=leg_id,
                        ),
                        _event(
                            phase, arm, seed, "goal_result", dispatch_s + 0.4,
                            leg_id=leg_id, state="SUCCEEDED",
                        ),
                    ]
                )
    for stamp, x in ((1.0, 0.0), (2.0, 0.1), (3.0, 0.2), (4.0, 0.3)):
        events.extend(
            [
                _event(
                    phase,
                    arm,
                    seed,
                    "estimated_pose",
                    stamp,
                    source="amcl",
                    x=x + 0.1,
                    y=0.0,
                    yaw_deg=5.0,
                ),
                _event(
                    phase,
                    arm,
                    seed,
                    "odom_pose",
                    stamp,
                    x=x,
                    y=0.0,
                    yaw_deg=0.0,
                ),
                _event(
                    phase,
                    arm,
                    seed,
                    "module1_diagnostic",
                    stamp,
                    values={
                        "region_id": "west" if stamp < 3.0 else "east",
                        "entropy": 0.4,
                        "reliability": 0.8,
                        "ood_probability": 0.1,
                        "dominant_mass": 0.75,
                        "dominant_covariance_m2": [0.4, 0.05, 0.05, 0.2],
                    },
                ),
            ]
        )
    events.append(
        _event(
            phase,
            arm,
            seed,
            "episode_end",
            8.0,
            state="SUCCEEDED",
            stop_reason="route_complete",
            collision=False,
            terminal_zero_confirmed=True,
            completed_leg_ids=(
                ["G1_G2"] if phase == "D"
                else ["G2", "G3", "G4", "G5", "G1"]
            ),
            fault_service_request_count=1 if phase == "E" else 0,
            nomotion_request_count=1 if phase == "E" and arm == "R0" else 0,
            manual_rescue_count=1 if phase == "E" and arm == "R1" else 0,
            supervisor_initialpose_count=(
                1 if phase == "E" and arm == "R1" else 0
            ),
        )
    )
    truth = [_gt(phase, arm, seed, stamp, x=x) for stamp, x in ((1.0, 0.0), (2.0, 0.1), (3.0, 0.2), (4.0, 0.3))]
    return events, truth


def test_phase_d_pair_reports_seed_provenance_gt_region_and_module1_raw_metrics():
    s0, s0_gt = _episode("D", "S0", ready_s=3.0)
    s1, s1_gt = _episode("D", "S1", ready_s=2.0)
    result = evaluate_phase_de_pair(s0, s1, s0_gt, s1_gt)

    assert result["phase"] == "D"
    assert result["arms"] == ["S0", "S1"]
    assert result["formal_gate"] is False
    assert result["baseline"]["initialpose"]["by_source"] == {"runner": 1}
    assert result["experimental"]["initialpose"]["by_source"] == {"supervisor": 1}
    assert result["experimental"]["prior_write_count"] == 1
    assert result["baseline"]["region"] == {
        "matched_sample_count": 4,
        "correct_sample_count": 2,
        "first_correct_region_id": "east",
        "first_correct_region_time_s": 3.0,
        "wrong_region_duration_s": 2.0,
        "wrong_region_distance_m": pytest.approx(0.2),
    }
    assert result["baseline"]["localization"]["gt_position_error_m"]["p95"] == pytest.approx(0.1)
    assert result["baseline"]["localization"]["gt_yaw_error_deg"]["p95"] == 5.0
    assert result["experimental"]["module1"]["dominant_mass"]["median"] == 0.75
    assert result["experimental"]["module1"]["dominant_covariance_m2"]["trace"]["median"] == pytest.approx(0.6)
    paired = result["paired_metrics"]["localization.time_to_ready_s"]
    assert paired == {"baseline": 3.0, "experimental": 2.0, "experimental_minus_baseline": -1.0}
    assert result["experimental"]["route"]["success"] is True


def test_phase_e_pair_reports_fault_pause_recovery_and_supervisor_diagnostics():
    r0, r0_gt = _episode("E", "R0", ready_s=1.0, recover_s=5.0)
    r1, r1_gt = _episode("E", "R1", ready_s=1.0, recover_s=4.0)
    result = evaluate_phase_de_pair(r0, r1, r0_gt, r1_gt)

    assert result["fault"] == "F2"
    assert result["baseline"]["fault_kind"] == (
        "amcl_global_localization_particle_spread"
    )
    assert result["experimental"]["timestamps_s"]["seed"] == 2.2
    assert result["baseline"]["localization"]["time_to_recover_s"] == pytest.approx(2.8)
    assert result["experimental"]["localization"]["time_to_recover_s"] == pytest.approx(1.8)
    assert result["experimental"]["pause"] == {
        "latency_s": pytest.approx(0.2),
        "cmd_vel_sim_zero_confirmed": True,
        "stationary_confirmed": True,
    }
    assert result["experimental"]["timestamps_s"]["prior_write"] == 2.8
    assert result["experimental"]["supervisor_diagnostics"][-1]["result"] == "applied"
    assert result["baseline"]["recovery_requests"] == {
        "fault_service": 1,
        "nomotion": 1,
        "manual": 0,
    }
    assert result["experimental"]["recovery_requests"] == {
        "fault_service": 1,
        "nomotion": 0,
        "manual": 1,
    }
    assert result["paired_metrics"]["localization.time_to_recover_s"][
        "experimental_minus_baseline"
    ] == pytest.approx(-1.0)


def test_phase_e_invalid_fault_stops_without_rescue_or_g3_and_stays_raw_only():
    runtime, truth = _episode("E", "R1", ready_s=1.0, recover_s=4.0)
    removed = {
        "manual_rescue_requested",
        "prior_write",
        "localization_recovered",
    }
    runtime = [
        row for row in runtime
        if row["event"] not in removed
        and not (row["event"] == "initialpose" and row.get("source") == "supervisor")
        and not (row["event"] in {"goal_dispatched", "goal_result"}
                 and row.get("leg_id") in {"G3", "G4", "G5", "G1"})
    ]
    runtime = [
        dict(
            row,
            outcome="INVALID_FAULT_NOT_DISCRIMINATIVE",
            supervisor_lost_observed=False,
        )
        if row["event"] == "fault_injected"
        else dict(
            row,
            state="STOP",
            stop_reason="INVALID_FAULT_NOT_DISCRIMINATIVE",
            completed_leg_ids=["G2"],
            manual_rescue_count=0,
            supervisor_initialpose_count=0,
        )
        if row["event"] == "episode_end"
        else row
        for row in runtime
    ]
    result = evaluate_phase_de_episode(runtime, truth)

    assert result["fault_outcome"] == "INVALID_FAULT_NOT_DISCRIMINATIVE"
    assert result["recovery_requests"]["manual"] == 0
    assert result["initialpose"]["by_source"] == {"runner": 1}
    assert result["route"]["success"] is False
    assert result["formal_gate"] is False
    assert result["evaluation_kind"] == "ENGINEERING_RAW_METRICS_ONLY"


def test_phase_e_evaluator_requires_completed_g2_before_fault_and_fault_before_g3():
    runtime, truth = _episode("E", "R0", ready_s=1.0, recover_s=5.0)
    bad = [
        dict(row, stamp_s=1.7) if row["event"] == "fault_injected" else row
        for row in runtime
    ]
    with pytest.raises(EvaluationError, match="completed G2 before fault"):
        evaluate_phase_de_episode(bad, truth)

    bad = [
        dict(row, stamp_s=2.1)
        if row["event"] == "goal_dispatched" and row.get("leg_id") == "G3"
        else row
        for row in runtime
    ]
    with pytest.raises(EvaluationError, match="fault before G3"):
        evaluate_phase_de_episode(bad, truth)


def test_runtime_ground_truth_firewall_and_pair_identity_are_fail_closed():
    s0, s0_gt = _episode("D", "S0")
    s1, s1_gt = _episode("D", "S1")
    contaminated = [dict(row) for row in s0]
    contaminated[0]["topic"] = "/ground_truth/odom"
    with pytest.raises(EvaluationError, match="Ground Truth firewall"):
        evaluate_phase_de_episode(contaminated, s0_gt)

    mismatched = [dict(row, seed=9002) for row in s1]
    mismatched_gt = [dict(row, seed=9002) for row in s1_gt]
    with pytest.raises(EvaluationError, match="same seed"):
        evaluate_phase_de_pair(s0, mismatched, s0_gt, mismatched_gt)


def test_pair_file_loader_keeps_runtime_and_passive_gt_schemas_separate(tmp_path):
    s0, s0_gt = _episode("D", "S0")
    s1, s1_gt = _episode("D", "S1")
    paths = []
    for name, rows in (("s0", s0), ("s1", s1), ("s0_gt", s0_gt), ("s1_gt", s1_gt)):
        path = tmp_path / f"{name}.jsonl"
        path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
        paths.append(path)
    result = evaluate_phase_de_pair_files(*paths)
    assert result["evaluation_kind"] == "PAIRED_ENGINEERING_RAW_METRICS_ONLY"

    bad_gt = tmp_path / "bad_gt.jsonl"
    bad_gt.write_text(json.dumps(s0[0]) + "\n", encoding="utf-8")
    with pytest.raises(EvaluationError, match="schema must be"):
        evaluate_phase_de_pair_files(paths[0], paths[1], bad_gt, paths[3])


def _odom(stamp_s, *, x=0.25, y=0.25, yaw_deg=0.0):
    seconds = math.floor(stamp_s)
    nanoseconds = round((stamp_s - seconds) * 1.0e9)
    half_yaw = math.radians(yaw_deg) / 2.0
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds)
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=x, y=y),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(half_yaw),
                    w=math.cos(half_yaw),
                ),
            )
        ),
    )


def test_extract_gt_aligns_header_time_and_selects_latest_post_reset_epoch():
    runtime, _ = _episode("D", "S1")
    # Both epochs cover the estimate stamps.  The later epoch is the accepted
    # post-reset stream and must win deterministically.
    messages = [
        *[_odom(stamp, x=-0.25) for stamp in (1.0, 2.0, 3.0, 4.0)],
        *[
            _odom(stamp, x=0.25, yaw_deg=90.0)
            for stamp in (0.98, 2.02, 3.0, 4.08)
        ],
    ]
    rows = extract_phase_de_ground_truth(
        runtime, messages, synchronization_tolerance_s=0.10
    )

    assert [row["stamp_s"] for row in rows] == pytest.approx([0.98, 2.02, 3.0, 4.08])
    assert {row["schema"] for row in rows} == {PHASE_DE_GT_SCHEMA}
    assert {row["event"] for row in rows} == {"ground_truth_pose"}
    assert {row["run_id"] for row in rows} == {"phase-d-s1-9001"}
    assert {row["phase"] for row in rows} == {"D"}
    assert {row["arm"] for row in rows} == {"S1"}
    assert {row["seed"] for row in rows} == {9001}
    assert {row["x"] for row in rows} == {0.25}
    assert {row["region_id"] for row in rows} == {"east_corridor"}
    assert [row["yaw_deg"] for row in rows] == pytest.approx([90.0] * 4)


def test_extract_gt_rejects_incomplete_stamp_alignment_and_non_enrolled_pose():
    runtime, _ = _episode("E", "R0")
    with pytest.raises(EvaluationError, match="does not cover every estimated_pose"):
        extract_phase_de_ground_truth(
            runtime,
            [_odom(1.0), _odom(2.0)],
            synchronization_tolerance_s=0.01,
        )

    with pytest.raises(EvaluationError, match="non-enrolled state"):
        extract_phase_de_ground_truth(
            runtime,
            [_odom(stamp, x=7.25, y=7.25) for stamp in (1.0, 2.0, 3.0, 4.0)],
        )


def test_extract_gt_cli_uses_offline_reader_and_writes_minimal_jsonl(
    tmp_path, monkeypatch
):
    runtime, _ = _episode("D", "S0")
    runtime_path = tmp_path / "runtime.jsonl"
    runtime_path.write_text(
        "".join(json.dumps(row) + "\n" for row in runtime), encoding="utf-8"
    )
    seen = []

    def fake_reader(path):
        seen.append(path)
        return iter(_odom(stamp) for stamp in (1.0, 2.0, 3.0, 4.0))

    monkeypatch.setattr(evaluator, "_iter_ground_truth_odometry", fake_reader)
    output = tmp_path / "gt.jsonl"
    assert evaluator.main(
        [
            "extract-gt",
            "--bag",
            str(tmp_path / "episode.mcap"),
            "--episode-jsonl",
            str(runtime_path),
            "--output",
            str(output),
        ]
    ) == 0
    assert seen == [str(tmp_path / "episode.mcap")]
    rows = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
    assert len(rows) == 4
    assert set(rows[0]) == {
        "schema",
        "run_id",
        "phase",
        "arm",
        "seed",
        "event",
        "stamp_s",
        "x",
        "y",
        "yaw_deg",
        "region_id",
    }


def test_evaluate_cli_accepts_reversed_phase_pair_and_calls_pair_evaluator(tmp_path):
    s0, s0_gt = _episode("D", "S0", ready_s=3.0)
    s1, s1_gt = _episode("D", "S1", ready_s=2.0)
    paths = {}
    for name, rows in (("s0", s0), ("s1", s1), ("s0_gt", s0_gt), ("s1_gt", s1_gt)):
        path = tmp_path / f"{name}.jsonl"
        path.write_text(
            "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
        )
        paths[name] = path
    output = tmp_path / "paired.json"
    assert evaluator.main(
        [
            "evaluate",
            "--runtime-jsonl",
            str(paths["s1"]),
            "--gt-jsonl",
            str(paths["s1_gt"]),
            "--runtime-jsonl",
            str(paths["s0"]),
            "--gt-jsonl",
            str(paths["s0_gt"]),
            "--output",
            str(output),
        ]
    ) == 0
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["phase"] == "D"
    assert result["arms"] == ["S0", "S1"]
    assert result["paired_metrics"]["localization.time_to_ready_s"] == {
        "baseline": 3.0,
        "experimental": 2.0,
        "experimental_minus_baseline": -1.0,
    }


def test_evaluate_cli_rejects_ground_truth_contamination_in_runtime(tmp_path, capsys):
    runtime, truth = _episode("E", "R0", recover_s=5.0)
    runtime[0] = dict(runtime[0], topic="/ground_truth/odom")
    runtime_path = tmp_path / "runtime.jsonl"
    gt_path = tmp_path / "gt.jsonl"
    runtime_path.write_text(
        "".join(json.dumps(row) + "\n" for row in runtime), encoding="utf-8"
    )
    gt_path.write_text(
        "".join(json.dumps(row) + "\n" for row in truth), encoding="utf-8"
    )
    assert evaluator.main(
        [
            "evaluate",
            "--runtime-jsonl",
            str(runtime_path),
            "--gt-jsonl",
            str(gt_path),
            "--output",
            str(tmp_path / "out.json"),
        ]
    ) == 2
    assert "Ground Truth firewall" in capsys.readouterr().err
