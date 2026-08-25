import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
from dataclasses import replace
from types import SimpleNamespace

import pytest

import robot_experiments.v6_low_obstacle_causal as causal

from robot_experiments.v6_low_obstacle_causal import (
    AdapterTemplates,
    DISPATCHER_TOPICS,
    EXPECTED_ORDER,
    RecordedMessage,
    build_recorded_evidence,
    build_plan,
    cli,
    evaluate,
    exact_adapter_templates,
    load_manifest,
    run_campaign,
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
    typed = [] if run.arm == "M0" else [{
        "x": 1.02, "y": 2.01, "accepted": True,
        "observed_spatial_error_m": 0.022360679774997918,
    }]
    clearance = {"M0": 0.20, "M1": 0.21, "M2": 0.46, "M3": 0.52}[run.arm]
    row = {
        "run_id": run.run_id,
        "repeat": run.repeat,
        "arm": run.arm,
        "identity": dict(manifest.identity),
        "module2_uds_connected": run.arm != "M0",
        "module2_health": {
            "message_count": 0 if run.arm == "M0" else 2,
            "healthy_count": 0 if run.arm == "M0" else 2,
            "trusted_write_count": 2 if run.arm in {"M2", "M3"} else 0,
            "observation_valid_count": 0 if run.arm == "M0" else 2,
            "candidate_cadence_hz": 0.0 if run.arm == "M0" else 5.0,
            "scope": "low_obstacle_only",
        },
        "isolation": {
            "module1_amcl_prior_enabled": False,
            "cognitive_place_graph_enabled": False,
            "dynamic_actors_enabled": False,
            "unexpected_topic_counts": {
                "/bio_nav/module2/edge_priors": 0,
                "/bio_nav/module2/cognitive_place_graph": 0,
            },
        },
        "reset": {
            "calls": 1,
            "events": 1,
            "goal_publications": 1,
            "localization_contract": "same_estimated_autonomy",
        },
        "freshness": {
            "ttl_clear_applicability": (
                "required_active_write" if run.arm in {"M2", "M3"}
                else "not_applicable_inactive"
            ),
            "ttl_source_sequence": 7 if run.arm in {"M2", "M3"} else None,
            "ttl_expiry_stamp_ns": 700000000 if run.arm in {"M2", "M3"} else None,
            "max_typed_obstacle_age_sec": 0.8 if stale else 0.1,
            "stale_applied_count": 1 if stale else 0,
            "stopped_before_dispatch": not stale if run.arm in {"M2", "M3"} else None,
            "layer_zero_write": True if run.arm in {"M2", "M3"} else None,
            "critic_not_applied": None,
            "ttl_expiry_observed": True if run.arm in {"M2", "M3"} else None,
            "ttl_expiry_zero_write": True if run.arm in {"M2", "M3"} else None,
            "ttl_expiry_critic_not_applied": None,
            "critic_ttl_status": (
                "N/A_NO_CONTROLLER_SCORING" if run.arm == "M3" else None
            ),
            "critic_post_expiry_applied": False if run.arm == "M3" else None,
            "critic_stale_active_probe": "NOT_RUN" if run.arm == "M3" else None,
        },
        "synchronized_samples": [{
            "stamp_ns": 123000000,
            "frame_id": "map",
            "scan_valid": True,
            "scan_point_count": 50,
            "scan_hits_in_obstacle_footprints": 0,
            "depth_observation_valid": True,
            "depth_observation_reason": "observed",
            "depth_point_count": 20,
            "depth_hits_in_obstacle_bounds": 4,
            "rgbd_obstacle_footprints": [{
                "center": [1.0, 2.0], "source": "projected_depth_points", "point_count": 4,
            }],
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
            "applied": run.arm == "M3",
            "reason": "cost_delta_applied=true;obstacle_applied=true" if run.arm == "M3" else "disabled_or_shadow",
            "cost_delta_nonzero_count": 1 if run.arm == "M3" else 0,
            "near_obstacle_speed_mps": {"M0": 0.4, "M1": 0.4, "M2": 0.3, "M3": 0.2}[run.arm],
            "offline_reconstructed_scores": [],
        },
        "planning_prior": [] if run.arm == "M0" else [{"stamp_ns": 123000000}],
        "costmaps": {"global": {"recorded": True}, "local": {"recorded": True}},
        "plan": plan,
        "optimal_trajectory": plan,
        "odom": plan,
        "cmd_vel": [{"linear_x": 0.2, "angular_z": 0.0}],
        "action": {
            "state": "SUCCEEDED",
            "stop_reason": "",
            "completed_leg_ids": ["G2"],
            "terminal_zero_confirmed": True,
        },
        "route": {"goal_results": [], "progress_messages": 2, "completion_messages": 1},
        "passive": {
            "ground_truth_odom": plan,
            "minimum_clearance_m": clearance,
            "collision": False,
            "success": True,
        },
        "navigation_metrics": {
            "recorded_duration_sec": 20.0,
            "global_plan_updates": 2,
            "local_trajectory_updates": 10,
            "nonzero_command_count": 1,
            "near_obstacle_speed_mps": 0.2,
            "dynamic_risk_exposure": "not_applicable_dynamic_actors_off",
            "false_positive_deadlock": False,
            "stale_residual_applied_count": 0,
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
    assert manifest.identity["low_obstacles_enabled"] is True
    assert manifest.identity["dynamic_actors_enabled"] is False
    assert manifest.identity["module1_amcl_prior_enabled"] is False
    assert manifest.identity["cognitive_place_graph_enabled"] is False
    assert manifest.identity["scene_asset"].endswith("kujiale_0026_A_to_B_door_open.usd")


def test_m0_disables_uds_but_preserves_same_localization_contract():
    manifest = load_manifest(CONFIG)
    m0 = manifest.arms["M0"]
    assert m0.module2_uds_enabled is False
    assert m0.integration_bridge_enabled is False
    assert m0.integration_process_required is False
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


def test_offline_causal_evaluator_passes_isolation_clearance_and_m3_local_effect(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    summary = evaluate(manifest, tmp_path)
    assert summary.verdict == "PASS_ENGINEERING_CAUSAL"
    assert all(pair.hausdorff_m <= 0.15 for pair in summary.m1_vs_m0)
    assert all(pair.length_delta_fraction <= 0.05 for pair in summary.m1_vs_m0)
    assert min(pair.clearance_gain_m for pair in summary.m2_vs_m1) >= 0.20
    assert min(pair.clearance_gain_m for pair in summary.m3_vs_m1) >= 0.20
    assert all(result.critic_participation == "online_applied" for result in summary.runs if result.arm == "M3")
    assert all(pair.trajectory_source == "local_trajectory" for pair in summary.m3_vs_m2)
    assert all(pair.near_obstacle_speed_delta_mps < 0.0 for pair in summary.m3_vs_m2)
    assert len(summary.visualization_inputs) == 3


def test_evaluator_consumes_campaign_nested_run_output_layout(tmp_path):
    manifest = load_manifest(CONFIG)
    for run in manifest.runs:
        run_dir = tmp_path / run.run_id
        run_dir.mkdir()
        (run_dir / f"{run.run_id}.json").write_text(
            json.dumps(_evidence(manifest, run)), encoding="utf-8"
        )
    summary = evaluate(manifest, tmp_path)
    assert summary.verdict == "PASS_ENGINEERING_CAUSAL"
    assert all(
        Path(result.evidence_file).parent.name == result.run_id
        for result in summary.runs
    )


def test_ttl_clear_is_explicitly_not_applicable_for_m0_and_m1(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    summary = evaluate(manifest, tmp_path)
    assert all(result.verdict == "VALID" for result in summary.runs if result.arm in {"M0", "M1"})
    for run in manifest.runs:
        if run.arm in {"M0", "M1"}:
            row = json.loads((tmp_path / f"{run.run_id}.json").read_text())
            assert row["freshness"]["ttl_clear_applicability"] == "not_applicable_inactive"


def test_m3_nominal_ttl_accepts_no_post_expiry_controller_scoring(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    summary = evaluate(manifest, tmp_path)
    results = [result for result in summary.runs if result.arm == "M3"]
    assert all(result.verdict == "VALID" for result in results)
    assert all(
        result.critic_ttl_status == "N/A_NO_CONTROLLER_SCORING"
        and result.critic_post_expiry_applied is False
        and result.critic_stale_active_probe == "NOT_RUN"
        for result in results
    )
    for run in manifest.runs:
        if run.arm == "M3":
            freshness = json.loads(
                (tmp_path / f"{run.run_id}.json").read_text(encoding="utf-8")
            )["freshness"]
            assert freshness["ttl_expiry_zero_write"] is True
            assert freshness["critic_ttl_status"] == "N/A_NO_CONTROLLER_SCORING"
            assert freshness["critic_post_expiry_applied"] is False
            assert freshness["critic_stale_active_probe"] == "NOT_RUN"


def test_m3_post_expiry_critic_application_is_invalid(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M3")
    row = _evidence(manifest, run)
    row["freshness"].update({
        "stale_applied_count": 1,
        "critic_ttl_status": "FAIL_POST_EXPIRY_APPLIED",
        "critic_post_expiry_applied": True,
    })
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert result.reasons == ("stale_input_applied_after_expiry",)


def test_m3_post_expiry_stale_rejected_callback_is_valid(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M3")
    row = _evidence(manifest, run)
    row["freshness"].update({
        "ttl_expiry_critic_not_applied": True,
        "critic_ttl_status": "STALE_REJECTED",
        "critic_post_expiry_applied": False,
    })
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "VALID"


def test_m3_missing_global_or_local_ttl_clear_is_invalid(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M3")
    row = _evidence(manifest, run)
    row["freshness"].update({
        "ttl_expiry_observed": False,
        "ttl_expiry_zero_write": False,
        "layer_zero_write": False,
    })
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert "active arm lacks clean TTL-expiry evidence" in result.reasons


def test_m3_local_trajectory_without_separation_is_ambiguous(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest, m3_same_as_m2=True)
    summary = evaluate(manifest, tmp_path)
    assert summary.verdict == "AMBIGUOUS"
    assert "M3_critic_has_no_trajectory_separation" in summary.reasons


@pytest.mark.parametrize(
    "reason",
    [
        "cost_delta_applied=false;zero_cost_delta;obstacle_applied=false",
        "obstacle_applied=true",
    ],
)
def test_zero_or_unproven_online_delta_is_not_reported_applied(tmp_path, reason):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M3")
    row = _evidence(manifest, run)
    row["critic"].update({"applied": True, "reason": reason, "cost_delta_nonzero_count": 0})
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.critic_participation == "none"


@pytest.mark.parametrize(
    "reason",
    [
        "cost_delta_applied=true;obstacle_applied=true",
        (
            "cost_delta_applied=true;obstacle_applied=false;"
            "obstacle_suppressed=zero_cost_delta;context_applied=true"
        ),
    ],
)
def test_positive_online_delta_is_reported_applied(tmp_path, reason):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M3")
    row = _evidence(manifest, run)
    row["critic"].update({
        "applied": True,
        "reason": reason,
        "cost_delta_nonzero_count": 1,
    })
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.critic_participation == "online_applied"


def test_post_expiry_application_is_invalid_and_never_causal_pass(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[2]
    (tmp_path / f"{run.run_id}.json").write_text(
        json.dumps(_evidence(manifest, run, stale=True)), encoding="utf-8"
    )
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert result.reasons == ("stale_input_applied_after_expiry",)
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


@pytest.mark.parametrize(
    ("mutation", "expected"),
    [
        ({"passive": {"collision": True}}, "STOP_COLLISION"),
        ({"action": {"terminal_zero_confirmed": False}}, "STOP_TERMINAL_ZERO"),
    ],
)
def test_collision_and_missing_terminal_zero_are_retained_stop_results(tmp_path, mutation, expected):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[0]
    row = _evidence(manifest, run)
    for section, values in mutation.items():
        row[section].update(values)
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == expected
    assert summary.verdict == "INVALID"


def test_recorder_reduces_synthetic_messages_to_required_real_fields():
    manifest = load_manifest(CONFIG)
    run = manifest.runs[3]
    stamp = 2_000_000_000
    records = [
        RecordedMessage("/camera/front/depth/image_raw", stamp, {
            "header": {"stamp": stamp, "frame_id": "camera_optical"},
            "width": 8, "height": 8, "step": 32, "encoding": "32FC1",
            "is_bigendian": False, "data": struct.pack("<64f", *([1.0] * 64)),
        }),
        RecordedMessage("/camera/front/camera_info", stamp, {
            "header": {"stamp": stamp, "frame_id": "camera_optical"},
            "width": 8, "height": 8,
            "k": [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0],
        }),
        RecordedMessage("/tf_static", stamp, {"transforms": [{
            "header": {"stamp": stamp, "frame_id": "map"},
            "child_frame_id": "camera_optical",
            "transform": {
                "translation": {"x": -0.45, "y": -0.35, "z": -0.92},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }]}),
        RecordedMessage("/scan", stamp, {
            "header": {"stamp": stamp},
            "scan_point_count": 80,
            "scan_hits_in_obstacle_footprints": 0,
        }),
        RecordedMessage("/ground_truth/odom", stamp, {
            "header": {"stamp": stamp},
            "pose": {"pose": {"position": {"x": -1.15, "y": -1.2}}},
        }),
        RecordedMessage("/odom", stamp, {
            "header": {"stamp": stamp},
            "pose": {"pose": {"position": {"x": 0.0, "y": 0.0}}},
        }),
        RecordedMessage("/bio_nav/module2/cognitive_obstacles", stamp, {
            "header": {"stamp": stamp},
            "module2_healthy": True,
            "trusted_write": True,
            "observation_valid": True,
            "source_age": {"sec": 0, "nanosec": 20_000_000},
            "validation_stamp": {"sec": 2, "nanosec": 0},
            "validation_ttl": {"sec": 0, "nanosec": 500_000_000},
            "sequence": 7,
            "obstacles": [{
                "id": "v6_low_box_solo",
                "pose_xy_m": [-1.15, -0.35],
                "radius_m": 0.2,
                "confidence": 0.9,
            }],
        }),
        RecordedMessage("/bio_nav/module2/cognitive_obstacles", stamp + 100_000_000, {
            "header": {"stamp": stamp + 100_000_000},
            "module2_healthy": True,
            "trusted_write": True,
            "observation_valid": True,
            "source_age": {"sec": 0, "nanosec": 20_000_000},
            "validation_stamp": {"sec": 2, "nanosec": 100_000_000},
            "validation_ttl": {"sec": 0, "nanosec": 500_000_000},
            "sequence": 8,
            "obstacles": [{
                "id": "v6_low_box_solo", "pose_xy_m": [-0.45, -0.35],
                "radius_m": 0.2, "confidence": 0.9,
            }],
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", stamp, {
            "consumer": "/global_costmap:layer", "raised_cell_count": 7,
            "active_cell_count": 7, "maximum_cost": 60, "applied": True,
            "message_age_ms": 20.0, "source_sequence": 7,
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", stamp, {
            "consumer": "/local_costmap:layer", "raised_cell_count": 5,
            "active_cell_count": 5, "maximum_cost": 55, "applied": True,
            "message_age_ms": 20.0, "source_sequence": 7,
        }),
        RecordedMessage("/bio_nav/cognitive_risk_critic/status", stamp, {
            "applied": True, "fallback_reason": "cost_delta_applied=true;obstacle_applied=true",
            "message_age_ms": 20.0, "source_sequence": 7,
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", stamp + 700_000_000, {
            "consumer": "/global_costmap:layer", "raised_cell_count": 0,
            "active_cell_count": 0, "maximum_cost": 0, "maximum_cost_increase": 0,
            "applied": False, "message_age_ms": 700.0, "source_sequence": 8,
            "fallback_reason": "rejection_reason=validation_stale",
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", stamp + 700_000_000, {
            "consumer": "/local_costmap:layer", "raised_cell_count": 0,
            "active_cell_count": 0, "maximum_cost": 0, "maximum_cost_increase": 0,
            "applied": False, "message_age_ms": 700.0, "source_sequence": 8,
            "fallback_reason": "rejection_reason=validation_stale",
        }),
        RecordedMessage("/plan", stamp, {"poses": [[0.45, -5.35], [0.8, 4.8]]}),
        RecordedMessage("/optimal_trajectory", stamp, {"poses": [[0.0, 0.0], [-0.5, 0.2]]}),
        RecordedMessage("/cmd_vel", stamp, {"linear": {"x": 0.2}, "angular": {"z": 0.1}}),
        RecordedMessage("/global_costmap/costmap", stamp, {}),
        RecordedMessage("/local_costmap/costmap", stamp, {}),
        RecordedMessage("/bio_nav/module2/planning_prior", stamp, {"healthy": True}),
        RecordedMessage("/simulation/collision", stamp, {"data": False}),
    ]
    episode_result = {
        "state": "SUCCEEDED",
        "reset_calls": 1,
        "reset_events": 1,
        "goal_publications": 1,
        "completed_leg_ids": ["G2"],
        "route_progress_messages": 3,
        "route_completion_messages": 1,
        "route_goal_results": [],
        "terminal_zero_confirmed": True,
    }
    evidence = build_recorded_evidence(manifest, run, records, episode_result)
    assert evidence["module2_health"]["trusted_write_count"] == 2
    assert evidence["module2_health"]["candidate_cadence_hz"] == pytest.approx(10.0)
    assert evidence["layer"]["global"]["cells"] == 7
    assert evidence["layer"]["local"]["cells"] == 5
    assert evidence["critic"]["cost_delta_nonzero_count"] == 1
    assert evidence["freshness"]["ttl_expiry_observed"] is True
    assert evidence["freshness"]["ttl_expiry_zero_write"] is True
    assert evidence["freshness"]["ttl_expiry_critic_not_applied"] is None
    assert evidence["freshness"]["critic_ttl_status"] == "N/A_NO_CONTROLLER_SCORING"
    assert evidence["freshness"]["critic_post_expiry_applied"] is False
    assert evidence["freshness"]["critic_stale_active_probe"] == "NOT_RUN"
    assert evidence["synchronized_samples"][0]["scan_point_count"] == 80
    assert evidence["synchronized_samples"][0]["depth_observation_valid"] is True
    assert evidence["synchronized_samples"][0]["rgbd_obstacle_footprints"][0]["source"] == "projected_depth_points"
    assert evidence["synchronized_samples"][0]["typed_obstacles"][0]["observed_spatial_error_m"] is not None
    assert evidence["passive"]["success"] is True


def _m3_ttl_lifecycle_records(*, critic_reason=None, critic_applied=False, include_local=True):
    stamp = 2_000_000_000
    expiry_status_stamp = 2_600_000_000
    records = [
        RecordedMessage("/ground_truth/odom", stamp, {
            "header": {"stamp": stamp},
            "pose": {"pose": {"position": {"x": 0.0, "y": 0.0}}},
        }),
        RecordedMessage("/bio_nav/module2/cognitive_obstacles", stamp, {
            "header": {"stamp": stamp},
            "validation_stamp": {"sec": 2, "nanosec": 0},
            "validation_ttl": {"sec": 0, "nanosec": 500_000_000},
            "sequence": 7,
            "obstacles": [],
        }),
    ]
    for scope in (("global", "local") if include_local else ("global",)):
        records.append(RecordedMessage(
            "/bio_nav/cognitive_obstacle_layer/status", expiry_status_stamp, {
                "header": {"stamp": expiry_status_stamp},
                "consumer": f"/{scope}_costmap:layer",
                "raised_cell_count": 0,
                "active_cell_count": 0,
                "maximum_cost_increase": 0,
                "applied": False,
                "source_sequence": 7,
                "fallback_reason": "rejection_reason=validation_stale",
            },
        ))
    if critic_reason is not None:
        records.append(RecordedMessage(
            "/bio_nav/cognitive_risk_critic/status", expiry_status_stamp, {
                "header": {"stamp": expiry_status_stamp},
                "applied": critic_applied,
                "source_sequence": 7,
                "fallback_reason": critic_reason,
            },
        ))
    return records


@pytest.mark.parametrize(
    ("critic_reason", "critic_applied"),
    [
        ("cost_delta_applied=true;obstacle_applied=true", False),
        ("obstacle_rejected=validation_stale", True),
    ],
)
def test_recorder_marks_any_post_expiry_critic_application_invalid(
    critic_reason, critic_applied
):
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M3")
    evidence = build_recorded_evidence(
        manifest,
        run,
        _m3_ttl_lifecycle_records(
            critic_reason=critic_reason,
            critic_applied=critic_applied,
        ),
        {},
    )
    assert evidence["freshness"]["critic_ttl_status"] == "FAIL_POST_EXPIRY_APPLIED"
    assert evidence["freshness"]["critic_post_expiry_applied"] is True
    assert evidence["freshness"]["stale_applied_count"] == 1


def test_recorder_accepts_explicit_post_expiry_stale_critic_rejection():
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M3")
    evidence = build_recorded_evidence(
        manifest,
        run,
        _m3_ttl_lifecycle_records(
            critic_reason="obstacle_rejected=validation_stale",
        ),
        {},
    )
    assert evidence["freshness"]["critic_ttl_status"] == "STALE_REJECTED"
    assert evidence["freshness"]["critic_post_expiry_applied"] is False
    assert evidence["freshness"]["ttl_expiry_critic_not_applied"] is True


def test_recorder_requires_both_costmap_layers_to_clear_after_expiry():
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M3")
    evidence = build_recorded_evidence(
        manifest,
        run,
        _m3_ttl_lifecycle_records(include_local=False),
        {},
    )
    assert evidence["freshness"]["ttl_expiry_zero_write"] is False
    assert evidence["freshness"]["ttl_expiry_observed"] is False
    assert evidence["freshness"]["critic_ttl_status"] == "N/A_NO_CONTROLLER_SCORING"


def _depth_projection_inputs(depth_m=1.0, *, encoding="32FC1", with_tf=True):
    stamp = 3_000_000_000
    if encoding == "32FC1":
        data = struct.pack("<64f", *([depth_m] * 64))
        step = 32
        bigendian = False
    else:
        data = struct.pack(">64H", *([int(depth_m * 1000.0)] * 64))
        step = 16
        bigendian = True
    depth = RecordedMessage("/camera/front/depth/image_raw", stamp, {
        "header": {"stamp": stamp, "frame_id": "camera_optical"},
        "width": 8, "height": 8, "step": step, "encoding": encoding,
        "is_bigendian": bigendian, "data": data,
    })
    info = RecordedMessage("/camera/front/camera_info", stamp, {
        "header": {"stamp": stamp, "frame_id": "camera_optical"},
        "width": 8, "height": 8,
        "k": [100.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 1.0],
    })
    transforms = []
    if with_tf:
        transforms.append(RecordedMessage("/tf_static", stamp, {"transforms": [{
            "header": {"stamp": stamp, "frame_id": "map"},
            "child_frame_id": "camera_optical",
            "transform": {
                "translation": {"x": -0.45, "y": -0.35, "z": -0.92},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }]}))
    return depth, info, transforms


def test_depth_decoder_honors_row_padding_and_big_endian_millimeters():
    padded = b"".join(
        struct.pack("<2f", *row) + b"PAD!"
        for row in ((1.0, 2.0), (3.0, 4.0))
    )
    pixels, error = causal._decode_depth_pixels({
        "width": 2, "height": 2, "step": 12, "encoding": "32FC1",
        "is_bigendian": False, "data": padded,
    }, stride=1, maximum_depth_m=8.0)
    assert error is None
    assert [value for _, _, value in pixels] == [1.0, 2.0, 3.0, 4.0]
    pixels, error = causal._decode_depth_pixels({
        "width": 2, "height": 1, "step": 4, "encoding": "16UC1",
        "is_bigendian": True, "data": struct.pack(">2H", 500, 1250),
    }, stride=1, maximum_depth_m=8.0)
    assert error is None
    assert [value for _, _, value in pixels] == pytest.approx([0.5, 1.25])


@pytest.mark.parametrize(
    ("depth_m", "encoding", "with_tf", "expected_valid", "expected_hit"),
    [
        (1.0, "32FC1", True, True, True),
        (1.0, "16UC1", True, True, True),
        (2.0, "32FC1", True, True, False),
        (1.0, "32FC1", False, False, False),
    ],
)
def test_depth_footprint_requires_real_projected_points(
    depth_m, encoding, with_tf, expected_valid, expected_hit
):
    manifest = load_manifest(CONFIG)
    depth, info, transforms = _depth_projection_inputs(
        depth_m, encoding=encoding, with_tf=with_tf
    )
    result = causal._project_depth_obstacle(
        depth, info, [], transforms,
        causal._load_frozen_obstacle(manifest), manifest.criteria,
    )
    assert result["valid"] is expected_valid
    assert bool(result["footprints"]) is expected_hit
    if expected_hit:
        assert result["footprints"][0]["source"] == "projected_depth_points"
        assert result["footprints"][0]["point_count"] >= 3


def test_invalid_depth_or_future_tf_never_fabricates_a_hit():
    manifest = load_manifest(CONFIG)
    depth, info, _ = _depth_projection_inputs()
    invalid_depth = RecordedMessage(depth.topic, depth.stamp_ns, {
        **depth.message, "encoding": "8UC1",
    })
    invalid = causal._project_depth_obstacle(
        invalid_depth, info, [], [],
        causal._load_frozen_obstacle(manifest), manifest.criteria,
    )
    assert invalid["valid"] is False
    assert invalid["footprints"] == []
    future_tf = RecordedMessage("/tf", depth.stamp_ns + 1, {"transforms": [{
        "header": {"stamp": depth.stamp_ns + 1, "frame_id": "map"},
        "child_frame_id": "camera_optical",
        "transform": {
            "translation": {"x": -0.45, "y": -0.35, "z": -0.92},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }]})
    no_future = causal._project_depth_obstacle(
        depth, info, [future_tf], [],
        causal._load_frozen_obstacle(manifest), manifest.criteria,
    )
    assert no_future["valid"] is False
    assert no_future["footprints"] == []


def test_recorded_tf_chain_and_inverse_are_composed_at_depth_stamp():
    stamp = 4_000_000_000
    transforms = RecordedMessage("/tf_static", stamp, {"transforms": [
        {
            "header": {"stamp": stamp, "frame_id": "map"},
            "child_frame_id": "base_link",
            "transform": {
                "translation": {"x": 1.0, "y": 2.0, "z": 0.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
        {
            "header": {"stamp": stamp, "frame_id": "base_link"},
            "child_frame_id": "camera_optical",
            "transform": {
                "translation": {"x": 0.5, "y": 0.0, "z": 0.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        },
    ]})
    map_from_camera = causal._lookup_recorded_transform(
        [], [transforms], target_frame="map", source_frame="camera_optical",
        stamp_ns=stamp,
    )
    assert map_from_camera is not None
    assert causal._apply_transform(map_from_camera, (0.0, 0.0, 0.0)) == pytest.approx(
        (1.5, 2.0, 0.0)
    )
    camera_from_map = causal._lookup_recorded_transform(
        [], [transforms], target_frame="camera_optical", source_frame="map",
        stamp_ns=stamp,
    )
    assert camera_from_map is not None
    assert causal._apply_transform(camera_from_map, (1.5, 2.0, 0.0)) == pytest.approx(
        (0.0, 0.0, 0.0)
    )


def test_scan_invisibility_requires_an_independent_valid_scan(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[0]
    row = _evidence(manifest, run)
    row["synchronized_samples"][0]["scan_valid"] = False
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.scan_invisible_rgbd_pairs == 0


def test_plan_dispatch_requires_all_three_adapters_and_constructs_commands(tmp_path):
    manifest = load_manifest(CONFIG)
    without = build_plan(manifest, pilot=True, output_root=tmp_path)
    assert without["dispatch"] is False
    adapters = AdapterTemplates(
        "/opt/scene --asset {scene_asset} --seed {seed}",
        "/opt/stack --arm {arm} --profile {integration_startup_profile} --scope {active_effect_scope}",
        "/opt/episode --trace {episode_jsonl} --run {run_id}",
    )
    plan = build_plan(manifest, adapters=adapters, pilot=True, output_root=tmp_path)
    assert plan["dispatch"] is True
    assert [row["arm"] for row in plan["runs"]] == ["M0", "M1", "M2", "M3"]
    assert plan["runs"][2]["commands"]["stack"] == (
        "/opt/stack", "--arm", "M2", "--profile", "module2_causal_obstacle_active",
        "--scope", "obstacle_only",
    )
    assert plan["runs"][1]["setup"]["integration_startup_profile"] == "estimated_shadow"
    assert plan["runs"][0]["setup"]["integration_startup_profile"] == "off"
    assert plan["runs"][0]["setup"]["module1_amcl_prior_enabled"] is False
    assert plan["runs"][0]["setup"]["cognitive_place_graph_enabled"] is False
    exact = build_plan(
        manifest,
        adapters=exact_adapter_templates(manifest),
        pilot=True,
        output_root=tmp_path,
    )
    assert exact["runs"][0]["commands"]["scene"][0].endswith(
        "run_v6_r5_phase_b_kujiale.sh"
    )
    assert exact["runs"][3]["commands"]["episode"][1] == "dispatch-episode"
    assert exact["runs"][2]["commands"]["producer_stop"][1] == "stop-producer"


def test_copy_installed_manifest_resolves_phase_f_assets_without_cwd(
    tmp_path, monkeypatch
):
    root = PACKAGE.parents[2]
    share = tmp_path / "prefix/share/robot_experiments"
    (share / "config").mkdir(parents=True)
    installed_config = share / "config/v6_kujiale_low_obstacle_causal.yaml"
    shutil.copy2(CONFIG, installed_config)
    resources = (
        "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        "data/maps/occupancy/v6_kujiale_isaacgen_v1.pgm",
        "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml",
        "isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen.yaml",
        "isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen_manifest.yaml",
        "ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson",
        "ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml",
    )
    for relative in resources:
        destination = share / "phase_f_assets" / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(root / relative, destination)
    monkeypatch.delenv("BIO_NAV_MODULE3_ROOT", raising=False)
    monkeypatch.setattr(causal, "_installed_package_share", lambda: share)
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    manifest = load_manifest(installed_config)
    assert manifest.module3_root is None
    for key in (
        "occupancy_map", "spawn_manifest", "route_graph", "obstacle_config",
        "obstacle_manifest", "navigation_overlay",
    ):
        resolved = Path(manifest.identity[key])
        assert resolved.is_file()
        assert share in resolved.parents
    plan = build_plan(manifest, pilot=True, output_root=tmp_path / "plan")
    assert Path(plan["runs"][0]["setup"]["navigation_overlay"]).is_file()


def _run_campaign_with_fake_processes(tmp_path, monkeypatch, *, clear):
    manifest = load_manifest(CONFIG)
    run = next(row for row in manifest.runs if row.arm == "M2")
    manifest = replace(manifest, runs=(run,))
    events = []

    def fake_start(name, command, log_path, *, env=None):
        events.append(("start", name))
        return SimpleNamespace(name=name)

    def fake_stop(process, timeout_sec):
        events.append(("stop", process.name))
        return {"name": process.name, "returncode": 0}

    def fake_run(command, **kwargs):
        events.append(("run", command[0]))
        return SimpleNamespace(returncode=0)

    def fake_sleep(duration):
        events.append(("sleep", duration))

    def fake_record(*args, **kwargs):
        events.append(("record_evidence", None))
        return {"freshness": {
            "ttl_expiry_observed": clear,
            "ttl_expiry_zero_write": clear,
            "ttl_expiry_critic_not_applied": None,
        }}

    monkeypatch.setattr(causal, "_start_process", fake_start)
    monkeypatch.setattr(causal, "_stop_process", fake_stop)
    monkeypatch.setattr(causal.subprocess, "run", fake_run)
    monkeypatch.setattr(causal.time, "sleep", fake_sleep)
    monkeypatch.setattr(causal, "record_evidence_from_bag", fake_record)
    adapters = AdapterTemplates("/scene", "/stack", "/episode", "/producer-stop")
    summary = run_campaign(
        manifest, adapters, tmp_path / "campaign", pilot=False,
        shutdown_timeout_sec=0.1,
    )
    return summary, events


def test_campaign_stops_producer_then_records_ttl_clear_before_stack_shutdown(
    tmp_path, monkeypatch
):
    summary, events = _run_campaign_with_fake_processes(
        tmp_path, monkeypatch, clear=True
    )
    assert summary["runs"][0]["state"] == "EPISODE_FINISHED"
    assert events == [
        ("start", "scene"), ("start", "stack"), ("start", "recorder"),
        ("run", "/episode"), ("run", "/producer-stop"),
        ("sleep", pytest.approx(1.5)),
        ("stop", "stack"), ("stop", "recorder"), ("stop", "scene"),
        ("record_evidence", None),
    ]


def test_campaign_missing_post_ttl_clear_is_a_failure(tmp_path, monkeypatch):
    summary, _ = _run_campaign_with_fake_processes(tmp_path, monkeypatch, clear=False)
    assert summary["runs"][0]["state"] == "TTL_CLEAR_FAILED"
    assert summary["state"] == "FINISHED_WITH_FAILURES"


def test_pilot_evaluator_uses_only_one_four_arm_repeat(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    summary = evaluate(manifest, tmp_path, pilot=True)
    assert summary.verdict == "PASS_ENGINEERING_PILOT"
    assert [result.arm for result in summary.runs] == ["M0", "M1", "M2", "M3"]
    assert len(summary.m3_vs_m2) == 1


def test_exact_stack_adapter_maps_profiles_and_keeps_phase_f_isolation():
    root = PACKAGE.parents[2]
    stack = (root / "scripts/run_v6_low_obstacle_phase_f_stack.sh").read_text(
        encoding="utf-8"
    )
    wrapper = (root / "scripts/run_v6_kujiale_low_obstacles.sh").read_text(
        encoding="utf-8"
    )
    assert '[[ "${arm}" != "M0" ]]' in stack
    assert "startup_profile=\"estimated_shadow\"" in stack
    assert 'startup_profile="module2_causal_obstacle_active"' in stack
    assert "--active-effect-scope obstacle_only" in stack
    assert "run_v6_module2_causal_obstacle_server.sh" in stack
    assert "edge_prior" not in stack
    assert "cognitive_place_graph" not in stack
    assert "initialpose" not in stack
    assert '[[ "${1:-}" == "stop-producer" ]]' in stack
    assert 'wait "${module3_pid}"' in stack
    assert 'integration_bridge.pid' in stack
    assert 'module2_server.pid' in stack
    assert "run_ros_profile gvg fail_closed auto M3 mixed final" in wrapper
    assert "cognitive_graph_mode:=\"${graph_mode}\"" in wrapper
    assert "run_v6_r5_phase_b_kujiale.sh\" isaac" in wrapper


def _run_low_obstacle_wrapper(tmp_path, *arguments):
    root = PACKAGE.parents[2]
    project = tmp_path / "project"
    scripts = project / "scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copy2(root / "scripts/run_v6_kujiale_low_obstacles.sh", scripts)
    (scripts / "lib/common.sh").write_text(
        """#!/usr/bin/env bash
export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
die() { printf '%s\\n' "$*" >&2; exit 1; }
require_file() { [[ -f "$1" ]] || die "missing: $1"; }
""",
        encoding="utf-8",
    )
    run_ros = scripts / "run_ros.sh"
    run_ros.write_text(
        "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n", encoding="utf-8"
    )
    run_ros.chmod(0o755)
    scenario = (
        project
        / "ros2_ws/src/robot_experiments/config/v6_kujiale_low_obstacles_static.yaml"
    )
    scenario.parent.mkdir(parents=True)
    scenario.touch()
    env = os.environ.copy()
    env.pop("V6_COGNITIVE_PROFILE", None)
    result = subprocess.run(
        [str(scripts / "run_v6_kujiale_low_obstacles.sh"), *arguments],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )
    return project, result.stdout.splitlines()


@pytest.mark.parametrize("arm", ["M0", "M1", "M2", "M3"])
def test_phase_f_mixed_argv_uses_occupancy_map_without_posegraph(tmp_path, arm):
    project, argv = _run_low_obstacle_wrapper(tmp_path, "ros", arm)
    assert argv == [
        "navigation",
        "odometry_mode:=mixed",
        "localization_profile:=kujiale",
        "nav2_profile:=v6_low_obstacle_isolation",
        f"cognitive_profile:={arm}",
        "cognitive_graph_mode:=gvg",
        "initial_pose_source:=auto",
        "activation_startup_policy:=fail_closed",
        "structure_tf_source:=isaac",
        "localization_map_contract:=occupancy_only",
        "localization_owner:=amcl",
        f"spawn_poses_file:={project}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml",
        f"map_file:={project}/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        f"route_graph_file:={project}/ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson",
        "interactive:=false",
        "use_rviz:=false",
        "use_teleop:=false",
        "ekf_profile:=wheel_imu",
        f"imu_calibration_params_file:={project}/ros2_ws/src/robot_odometry/config/imu_calibration.yaml",
        "lidar_odometry_backend:=off",
        "lidar_odometry_validated:=false",
    ]
    assert not any(argument.startswith("posegraph_file:=") for argument in argv)


def test_legacy_shadow_argv_keeps_estimated_rf2o_posegraph_bundle(tmp_path):
    project, argv = _run_low_obstacle_wrapper(tmp_path, "shadow")
    assert argv == [
        "navigation",
        "odometry_mode:=estimated",
        "localization_profile:=kujiale",
        "nav2_profile:=v6_low_obstacle_isolation",
        "cognitive_profile:=M1",
        "cognitive_graph_mode:=gvg",
        "initial_pose_source:=rviz",
        "activation_startup_policy:=wait_for_seed",
        "structure_tf_source:=isaac",
        "localization_map_contract:=posegraph_bundle",
        "localization_owner:=auto",
        f"spawn_poses_file:={project}/isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml",
        f"posegraph_file:={project}/data/maps/posegraphs/v6_kujiale_isaacgen_v1",
        f"map_file:={project}/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        f"route_graph_file:={project}/ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson",
        "interactive:=false",
        "use_rviz:=false",
        "use_teleop:=false",
        "ekf_profile:=wheel_imu",
        "lidar_odometry_backend:=rf2o",
        "lidar_odometry_validated:=false",
    ]


@pytest.mark.parametrize("command", ["manifest", "plan"])
def test_non_runtime_cli_commands_emit_json(command, capsys):
    assert cli([command, "--config", str(CONFIG)]) == 0
    assert json.loads(capsys.readouterr().out)
