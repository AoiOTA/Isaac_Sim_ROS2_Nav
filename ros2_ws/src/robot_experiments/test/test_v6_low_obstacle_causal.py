import json
from pathlib import Path

import pytest

from robot_experiments.v6_low_obstacle_causal import (
    DISPATCHER_TOPICS,
    EXPECTED_ORDER,
    build_plan,
    cli,
    evaluate,
    load_manifest,
)


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "v6_kujiale_low_obstacle_causal.yaml"


def _paths(arm):
    paths = {
        "M0": [[0.45, -5.35], [0.50, -0.2], [0.80, 4.80]],
        "M1": [[0.45, -5.35], [0.53, -0.2], [0.80, 4.80]],
        "M2": [[0.45, -5.35], [-0.55, -0.2], [0.80, 4.80]],
        "M3": [[0.45, -5.35], [-0.80, -0.2], [0.80, 4.80]],
    }
    return paths[arm]


def _evidence(manifest, run, *, stale=False, omit=None, m3_same_as_m2=False):
    arm = manifest.arms[run.arm]
    plan = _paths(run.arm)
    if m3_same_as_m2 and run.arm == "M3":
        plan = _paths("M2")
    typed = [] if run.arm == "M0" else [{"x": 1.02, "y": 2.01, "accepted": True}]
    clearance = {"M0": 0.20, "M1": 0.21, "M2": 0.46, "M3": 0.52}[run.arm]
    row = {
        "run_id": run.run_id,
        "repeat": run.repeat,
        "arm": run.arm,
        "identity": dict(manifest.identity),
        "module2_uds_connected": run.arm != "M0",
        "reset": {
            "calls": 1,
            "events": 1,
            "goal_publications": 1,
            "localization_contract": "same_estimated_autonomy",
        },
        "freshness": {
            "max_typed_obstacle_age_sec": 0.8 if stale else 0.1,
            "stopped_before_dispatch": stale,
            "layer_zero_write": stale,
            "critic_not_applied": stale,
        },
        "synchronized_samples": [{
            "stamp_ns": 123000000,
            "frame_id": "map",
            "scan_point_count": 50,
            "scan_hits_in_obstacle_footprints": 0,
            "rgbd_obstacle_footprints": [{"center": [1.0, 2.0]}],
            "typed_obstacles": typed,
        }],
        "obstacle_validation": typed,
        "layer": {
            "mode": arm.obstacle_layer_mode,
            "global": {"status_count": 2, "cells": 10 if run.arm in {"M2", "M3"} else 0, "max_cost": 190},
            "local": {"status_count": 2, "cells": 4 if run.arm in {"M2", "M3"} else 0, "max_cost": 150},
        },
        "critic": {
            "mode": arm.critic_mode,
            "applied": False,
            "reason": "offline_reconstructed" if run.arm == "M3" else "disabled_or_shadow",
            "offline_reconstructed_scores": [0.4, 0.8] if run.arm == "M3" else [],
        },
        "planning_prior": [] if run.arm == "M0" else [{"stamp_ns": 123000000}],
        "costmaps": {"global": {"recorded": True}, "local": {"recorded": True}},
        "plan": plan,
        "optimal_trajectory": plan,
        "odom": plan,
        "cmd_vel": [{"linear_x": 0.2, "angular_z": 0.0}],
        "passive": {
            "ground_truth_odom": plan,
            "minimum_clearance_m": clearance,
            "collision": False,
            "success": True,
        },
    }
    if omit:
        row.pop(omit)
    return row


def _write_evidence(tmp_path, manifest, **kwargs):
    for run in manifest.runs:
        row = _evidence(manifest, run, **kwargs)
        (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")


def test_manifest_freezes_exact_twelve_counterbalanced_runs_and_identity():
    manifest = load_manifest(CONFIG)
    assert len(manifest.runs) == 12
    assert tuple(run.arm for run in manifest.runs) == EXPECTED_ORDER
    assert len({run.run_id for run in manifest.runs}) == 12
    assert {run.repeat for run in manifest.runs} == {1, 2, 3}
    assert manifest.identity["seed"] == 8601
    assert manifest.identity["start"]["id"] == "G1"
    assert manifest.identity["goal"]["id"] == "G2"
    assert manifest.identity["timeout_sec"] == 180.0
    assert manifest.identity["graph_backend"] == "gvg"
    assert manifest.identity["direct_rgbd_costmap_enabled"] is False


def test_m0_disables_uds_but_preserves_same_localization_contract():
    manifest = load_manifest(CONFIG)
    m0 = manifest.arms["M0"]
    assert m0.module2_uds_enabled is False
    assert m0.integration_bridge_enabled is False
    assert m0.integration_process_required is True
    assert m0.localization_contract == "same_estimated_autonomy"
    assert manifest.localization_contract["startup_profile"] == "estimated_autonomy"
    assert manifest.localization_contract["preserve_when_module2_disabled"] is True


def test_dispatcher_firewall_and_passive_evidence_are_separate():
    manifest = load_manifest(CONFIG)
    assert not [topic for topic in DISPATCHER_TOPICS if topic.startswith("/ground_truth/")]
    plan = build_plan(manifest)
    for run in plan["runs"]:
        assert not [topic for topic in run["dispatcher_topics"] if topic.startswith("/ground_truth/")]
        assert "/ground_truth/odom" in run["passive_evaluator_topics"]


def test_arm_sequence_exercises_off_shadow_on_and_critic_on():
    manifest = load_manifest(CONFIG)
    assert [manifest.arms[name].obstacle_layer_mode for name in EXPECTED_ORDER] == [
        "off", "shadow", "active", "active",
        "active", "active", "shadow", "off",
        "shadow", "active", "off", "active",
    ]
    assert [manifest.arms[name].critic_mode for name in EXPECTED_ORDER].count("active") == 3


def test_evaluator_pairs_scan_invisibility_and_typed_spatial_match(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    summary = evaluate(manifest, tmp_path)
    assert all(result.synchronized_frames == 1 for result in summary.runs)
    assert all(result.scan_invisible_rgbd_pairs == 1 for result in summary.runs)
    assert all(
        result.typed_spatial_matches == result.typed_spatial_total == 1
        for result in summary.runs if result.arm != "M0"
    )


def test_offline_causal_evaluator_passes_isolation_clearance_and_m3_score(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    summary = evaluate(manifest, tmp_path)
    assert summary.verdict == "PASS_ENGINEERING_CAUSAL"
    assert all(pair.hausdorff_m <= 0.15 for pair in summary.m1_vs_m0)
    assert all(pair.length_delta_fraction <= 0.05 for pair in summary.m1_vs_m0)
    assert min(pair.clearance_gain_m for pair in summary.m2_vs_m1) >= 0.20
    assert min(pair.clearance_gain_m for pair in summary.m3_vs_m1) >= 0.20
    assert all(result.critic_participation == "offline_reconstructed" for result in summary.runs if result.arm == "M3")
    assert len(summary.visualization_inputs) == 3


def test_m3_offline_score_without_trajectory_separation_is_ambiguous(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest, m3_same_as_m2=True)
    summary = evaluate(manifest, tmp_path)
    assert summary.verdict == "AMBIGUOUS"
    assert "M3_critic_has_no_trajectory_separation" in summary.reasons


def test_ttl_expiry_is_stop_fail_open_and_never_causal_pass(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[2]
    (tmp_path / f"{run.run_id}.json").write_text(
        json.dumps(_evidence(manifest, run, stale=True)), encoding="utf-8"
    )
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "STOP_FAIL_OPEN"
    assert summary.verdict == "INVALID"


def test_missing_evidence_is_invalid(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[0]
    (tmp_path / f"{run.run_id}.json").write_text(
        json.dumps(_evidence(manifest, run, omit="costmaps")), encoding="utf-8"
    )
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert "missing evidence: costmaps" in result.reasons[0]


def test_run_without_external_adapters_reports_not_run(capsys):
    code = cli(["run", "--config", str(CONFIG)])
    output = json.loads(capsys.readouterr().out)
    assert code == 2
    assert output["state"] == "NOT_RUN"
    assert output["qualification"] == "ENGINEERING_CAUSAL_NOT_RUN"


@pytest.mark.parametrize("command", ["manifest", "plan"])
def test_non_runtime_cli_commands_emit_json(command, capsys):
    assert cli([command, "--config", str(CONFIG)]) == 0
    assert json.loads(capsys.readouterr().out)
