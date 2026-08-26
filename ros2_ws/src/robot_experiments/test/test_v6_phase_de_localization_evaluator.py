import json

import pytest

from robot_experiments.v6_localization_causal_evaluator import (
    EvaluationError,
    PHASE_DE_EVENT_SCHEMA,
    PHASE_DE_GT_SCHEMA,
    evaluate_phase_de_episode,
    evaluate_phase_de_pair,
    evaluate_phase_de_pair_files,
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
    else:
        events.extend(
            [
                _event(
                    phase,
                    arm,
                    seed,
                    "fault_injected",
                    2.0,
                    fault_id="F2",
                    arc_length_m=1.0,
                    x=-2.2,
                    y=-2.95,
                    yaw_deg=-42.0,
                ),
                _event(phase, arm, seed, "pause_requested", 2.1),
                _event(
                    phase,
                    arm,
                    seed,
                    "cmd_vel_sim",
                    2.2,
                    zero=True,
                    linear_x=0.0,
                    linear_y=0.0,
                    angular_z=0.0,
                ),
                _event(
                    phase,
                    arm,
                    seed,
                    "pause_confirmed",
                    2.3,
                    cmd_vel_sim_zero=True,
                    stationary=True,
                ),
            ]
        )
        if arm == "R1":
            events.extend(
                [
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
                        count=1,
                    ),
                ]
            )
        if recover_s is not None:
            events.append(
                _event(
                    phase,
                    arm,
                    seed,
                    "localization_recovered",
                    recover_s,
                    success=True,
                )
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
                mode="startup" if arm == "S1" else "active" if arm == "R1" else "shadow",
                state="READY" if phase == "D" else "RECOVERED",
                reason="candidate_accepted" if arm in {"S1", "R1"} else "amcl_ready",
                result="applied" if arm in {"S1", "R1"} else "observed",
                reset_attempts=0,
            ),
            _event(phase, arm, seed, "goal_dispatched", 4.0, leg_id="G1_G2"),
            _event(
                phase, arm, seed, "goal_result", 7.0, leg_id="G1_G2", state="SUCCEEDED"
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
            completed_leg_ids=["G1_G2"],
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
    assert result["experimental"]["timestamps_s"]["seed"] == 2.0
    assert result["baseline"]["localization"]["time_to_recover_s"] == 3.0
    assert result["experimental"]["localization"]["time_to_recover_s"] == 2.0
    assert result["experimental"]["pause"] == {
        "latency_s": pytest.approx(0.2),
        "cmd_vel_sim_zero_confirmed": True,
        "stationary_confirmed": True,
    }
    assert result["experimental"]["timestamps_s"]["prior_write"] == 2.8
    assert result["experimental"]["supervisor_diagnostics"][0]["result"] == "applied"
    assert result["paired_metrics"]["localization.time_to_recover_s"][
        "experimental_minus_baseline"
    ] == -1.0


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
