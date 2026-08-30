import json
import fcntl
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import struct
import subprocess
import sys
import textwrap
import threading
import time
from dataclasses import replace
from types import SimpleNamespace

import pytest
import yaml

import robot_experiments.v6_low_obstacle_causal as causal
import robot_experiments.v6_formal as v6_formal

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
TEST_LIVE_MAP_VERSION = "4f8c2a1d" * 8


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
    baseline_failed = run.arm in {"M0", "M1"}
    plan = _paths(run.arm)
    if m3_same_as_m2 and run.arm == "M3":
        plan = _paths("M2")
    typed = [] if run.arm == "M0" else [{
        "id": "v6_low_box_solo", "x": -0.43, "y": -0.34,
        "radius_m": 0.20, "accepted": True,
        "observed_spatial_error_m": 0.022360679774997918,
        "trusted_write": run.arm in {"M2", "M3"},
        "validation_mode": causal.VALIDATION_STATIC_DEPTH_REVALIDATED,
        "validation_sensor_mask": causal.VALIDATION_SENSOR_DEPTH,
        "rejection_mask": (
            causal.SHADOW_REJECTION_UNTRUSTED if run.arm == "M1" else 0
        ),
        "motion_class": causal.MOTION_STATIC,
        "static_confirmed": True,
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
                causal.NOMINAL_TTL_STATUS if run.arm in {"M2", "M3"}
                else "not_applicable_inactive"
            ),
            "external_active_controller_probe_required": run.arm in {"M2", "M3"},
            "external_active_controller_probe_status": (
                "NOT_EVALUATED_BY_NOMINAL_RUN"
                if run.arm in {"M2", "M3"} else None
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
                causal.NOMINAL_TTL_STATUS if run.arm == "M3" else None
            ),
            "nominal_post_route_critic_observation": (
                "N/A_NO_CONTROLLER_SCORING" if run.arm == "M3" else None
            ),
            "critic_post_expiry_applied": False if run.arm == "M3" else None,
            "critic_stale_active_probe": "NOT_RUN" if run.arm == "M3" else None,
        },
        "sensor_counts": {
            "scan_message_count": 1,
            "depth_message_count": 1,
            "camera_info_message_count": 1,
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
                "id": "v6_low_box_solo",
                "center": [-0.45, -0.35, 0.08],
                "size": [0.30, 0.30, 0.16],
                "rectangle": [-0.60, -0.50, -0.30, -0.20],
                "source": causal.PHYSICAL_DEPTH_FOOTPRINT_SOURCE,
                "point_count": 4,
                "hit_count": 4,
            }],
            "typed_obstacles": typed,
        }],
        "obstacle_validation": typed,
        "shadow_obstacle_candidate": {
            "message_count": 2 if run.arm == "M1" else 0,
            "nonempty_message_count": 2 if run.arm == "M1" else 0,
            "static_depth_revalidated_geometry_count": 2 if run.arm == "M1" else 0,
            "trusted_write_count": 0,
            "shadow_rejection_count": 2 if run.arm == "M1" else 0,
            "invalid_geometry_count": 0,
        },
        "layer": {
            "mode": arm.obstacle_layer_mode,
            "global": {
                "status_count": 2, "cells": 10 if run.arm in {"M2", "M3"} else 0,
                "active_cells": 10 if run.arm in {"M2", "M3"} else 0,
                "applied_count": 2 if run.arm in {"M2", "M3"} else 0,
                "max_cost_increase": 10 if run.arm in {"M2", "M3"} else 0,
                "max_cost": 190,
            },
            "local": {
                "status_count": 2, "cells": 4 if run.arm in {"M2", "M3"} else 0,
                "active_cells": 4 if run.arm in {"M2", "M3"} else 0,
                "applied_count": 2 if run.arm in {"M2", "M3"} else 0,
                "max_cost_increase": 10 if run.arm in {"M2", "M3"} else 0,
                "max_cost": 150,
            },
        },
        "critic": {
            "mode": arm.critic_mode,
            "applied": run.arm == "M3",
            "reason": "cost_delta_applied=true;obstacle_applied=true" if run.arm == "M3" else "disabled_or_shadow",
            "cost_delta_nonzero_count": 1 if run.arm == "M3" else 0,
            "near_obstacle_speed_mps": {"M0": 0.4, "M1": 0.4, "M2": 0.3, "M3": 0.2}[run.arm],
            "offline_reconstructed_scores": [],
        },
        "planning_prior": [] if run.arm == "M0" else [{
            "stamp_ns": 123000000,
            "module2_healthy": True,
            "observation_valid": True,
            "trusted_write": run.arm in {"M2", "M3"},
        }],
        "costmaps": {"global": {"recorded": True}, "local": {"recorded": True}},
        "plan": plan,
        "optimal_trajectory": plan,
        "odom": plan,
        "cmd_vel": [{"linear_x": 0.2, "angular_z": 0.0}],
        "action": {
            "state": "STOP" if baseline_failed else "SUCCEEDED",
            "stop_reason": "collision_monitor_stop" if baseline_failed else "",
            "completed_leg_ids": ["G2"],
            "terminal_zero_confirmed": True,
        },
        "route": {"goal_results": [], "progress_messages": 2, "completion_messages": 1},
        "passive": {
            "ground_truth_odom": plan,
            "minimum_clearance_m": clearance,
            "collision": baseline_failed,
            "success": not baseline_failed,
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
    assert manifest.identity["route_backend"] == "gvg"
    assert manifest.identity["route_prior_enabled"] is False
    assert manifest.identity["graph_backend"] == "gvg"
    assert manifest.identity["direct_rgbd_costmap_enabled"] is False
    assert manifest.identity["low_obstacles_enabled"] is True
    assert manifest.identity["dynamic_actors_enabled"] is False
    assert manifest.identity["module1_amcl_prior_enabled"] is False
    assert manifest.identity["cognitive_place_graph_enabled"] is False
    assert manifest.identity["scene_asset"].endswith("kujiale_0026_A_to_B_door_open.usd")
    assert manifest.criteria["source_recall_min"] == 0.80
    assert manifest.criteria["candidate_precision_min"] == 0.50
    assert manifest.criteria["candidate_radius_max_m"] == 0.35
    assert manifest.criteria["selection_policy"] == (
        "simplest_valid_arm_with_observed_net_benefit"
    )
    assert manifest.criteria["m1_m0_path_similarity_diagnostic_only"] is True
    assert manifest.criteria["active_clearance_gain_diagnostic_only"] is True
    assert manifest.criteria[
        "m3_m2_trajectory_separation_diagnostic_only"
    ] is True
    assert manifest.criteria["selected_arm_active_ttl_required"] is True
    assert manifest.criteria["depth_obstacle_bounds_tolerance_m"] <= 0.02
    assert manifest.criteria["depth_min_height_above_floor_m"] >= 0.02


@pytest.mark.parametrize(
    ("key", "value", "reason"),
    [
        ("route_backend", "primary", "identity.route_backend"),
        ("route_prior_enabled", True, "identity.route_prior_enabled"),
    ],
)
def test_manifest_rejects_non_gvg_or_enabled_route_prior(
    tmp_path, monkeypatch, key, value, reason
):
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    raw["identity"][key] = value
    broken = tmp_path / "broken.yaml"
    broken.write_text(yaml.safe_dump(raw), encoding="utf-8")
    monkeypatch.setenv("BIO_NAV_MODULE3_ROOT", str(PACKAGE.parents[2]))
    with pytest.raises(causal.CausalContractError, match=reason):
        load_manifest(broken)


def test_plan_and_episode_runtime_share_gvg_without_route_prior(tmp_path):
    manifest = load_manifest(CONFIG)
    plan = build_plan(manifest, pilot=True, output_root=tmp_path)
    for row in plan["runs"]:
        assert row["setup"]["route_backend"] == "gvg"
        assert row["setup"]["route_prior_enabled"] is False
        run = next(item for item in manifest.runs if item.run_id == row["run_id"])
        runtime = causal._phase_f_runtime(
            manifest, run, dynamic_actors_enabled=False
        )
        assert runtime["route_backend"] == "gvg"
        assert runtime["route_prior_enabled"] is False
        assert runtime["obstacle_layer_mode"] == manifest.arms[run.arm].obstacle_layer_mode
        assert runtime["critic_mode"] == manifest.arms[run.arm].critic_mode


@pytest.mark.parametrize(
    ("key", "value", "reason"),
    [
        ("route_backend", "primary", "identity.route_backend"),
        ("route_prior_enabled", True, "identity.route_prior_enabled"),
    ],
)
def test_plan_rejects_inconsistent_route_identity(key, value, reason):
    manifest = load_manifest(CONFIG)
    broken = replace(manifest, identity={**manifest.identity, key: value})
    with pytest.raises(causal.CausalContractError, match=reason):
        build_plan(broken, pilot=True)


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
    assert all(
        result.scan_invisible_rgbd_pairs == (1 if result.arm in {"M1", "M2", "M3"} else 0)
        for result in summary.runs
    )
    assert all(
        result.typed_spatial_matches == result.typed_spatial_total == 1
        for result in summary.runs if result.arm in {"M1", "M2", "M3"}
    )
    assert all(
        result.typed_spatial_matches == result.typed_spatial_total == 0
        for result in summary.runs if result.arm == "M0"
    )


def test_one_visible_source_assigns_at_most_one_candidate_true_positive():
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M2")
    sample = _evidence(manifest, run)["synchronized_samples"][0]
    sample["typed_obstacles"].extend([
        {
            **sample["typed_obstacles"][0],
            "id": "duplicate-box-candidate",
            "x": -0.44,
            "y": -0.36,
            "observed_spatial_error_m": math.dist((-0.44, -0.36), (-0.45, -0.35)),
        },
        {
            **sample["typed_obstacles"][0],
            "id": "furniture-false-positive",
            "x": 1.0,
            "y": 2.0,
            "observed_spatial_error_m": math.dist((1.0, 2.0), (-0.45, -0.35)),
        },
    ])
    metrics = causal._scan_and_spatial_metrics(
        [sample],
        float(manifest.criteria["typed_spatial_match_tolerance_m"]),
        physical_obstacle=causal._load_frozen_obstacle(manifest),
    )
    assert metrics["source_visible_count"] == 1
    assert metrics["source_matched_count"] == 1
    assert metrics["candidate_true_positive_count"] == 1
    assert metrics["candidate_false_positive_count"] == 2
    assert metrics["candidate_precision"] == pytest.approx(1.0 / 3.0)
    assert len(metrics["best_center_errors_m"]) == 1


@pytest.mark.parametrize(
    ("fault", "reason"),
    [
        ("recall", "source recall below engineering threshold"),
        ("precision", "candidate precision below engineering threshold"),
        ("radius", "candidate radius exceeds engineering threshold"),
    ],
)
def test_active_spatial_semantics_gates_fail_closed(tmp_path, fault, reason):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M2")
    row = _evidence(manifest, run)
    sample = row["synchronized_samples"][0]
    if fault == "recall":
        missed = json.loads(json.dumps(sample))
        missed["stamp_ns"] += 1
        missed["typed_obstacles"] = []
        row["synchronized_samples"].append(missed)
    elif fault == "precision":
        sample["typed_obstacles"].extend([
            {
                **sample["typed_obstacles"][0],
                "id": f"false-positive-{index}",
                "x": 1.0 + index,
                "y": 2.0,
                "observed_spatial_error_m": math.dist(
                    (1.0 + index, 2.0), (-0.45, -0.35)
                ),
            }
            for index in range(2)
        ])
    else:
        sample["typed_obstacles"][0]["radius_m"] = 0.36
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert reason in result.reasons[0]


def test_m1_reports_but_does_not_gate_candidate_precision(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M1")
    row = _evidence(manifest, run)
    sample = row["synchronized_samples"][0]
    sample["typed_obstacles"].extend([
        {
            **sample["typed_obstacles"][0],
            "id": f"shadow-false-positive-{index}",
            "x": 1.0 + index,
            "y": 2.0,
            "observed_spatial_error_m": math.dist(
                (1.0 + index, 2.0), (-0.45, -0.35)
            ),
        }
        for index in range(2)
    ])
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "VALID"
    assert result.candidate_precision == pytest.approx(1.0 / 3.0)


def test_offline_causal_evaluator_passes_isolation_clearance_and_m3_local_effect(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    summary = evaluate(manifest, tmp_path)
    assert summary.verdict == "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
    assert summary.formal_qualification is False
    assert summary.phase_f_complete is False
    assert summary.selected_arm_active_ttl_status == "PENDING"
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
    assert summary.verdict == "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
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


def test_nominal_active_arms_defer_ttl_to_separate_controller_probe(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    summary = evaluate(manifest, tmp_path)
    results = [result for result in summary.runs if result.arm in {"M2", "M3"}]
    assert all(result.verdict == "VALID" for result in results)
    assert all(
        result.critic_ttl_status == causal.NOMINAL_TTL_STATUS
        for result in results if result.arm == "M3"
    )
    for run in manifest.runs:
        if run.arm in {"M2", "M3"}:
            freshness = json.loads(
                (tmp_path / f"{run.run_id}.json").read_text(encoding="utf-8")
            )["freshness"]
            assert freshness["ttl_clear_applicability"] == causal.NOMINAL_TTL_STATUS
            assert freshness["external_active_controller_probe_required"] is True
            assert freshness["external_active_controller_probe_status"] == (
                "NOT_EVALUATED_BY_NOMINAL_RUN"
            )


def test_nominal_post_route_ttl_observation_cannot_replace_active_probe(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M3")
    row = _evidence(manifest, run)
    row["freshness"].update({
        "stale_applied_count": 1,
        "nominal_post_route_critic_observation": "FAIL_POST_EXPIRY_APPLIED",
        "critic_post_expiry_applied": True,
    })
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "VALID"
    assert result.critic_ttl_status == causal.NOMINAL_TTL_STATUS


def test_m3_post_expiry_stale_rejected_callback_is_valid(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M3")
    row = _evidence(manifest, run)
    row["freshness"].update({
        "ttl_expiry_critic_not_applied": True,
        "nominal_post_route_critic_observation": "STALE_REJECTED",
        "critic_post_expiry_applied": False,
    })
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "VALID"


def test_nominal_missing_post_route_callbacks_is_explicit_na(tmp_path):
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
    assert result.verdict == "VALID"
    assert result.critic_ttl_status == causal.NOMINAL_TTL_STATUS


def test_m3_separation_target_is_diagnostic_when_other_net_benefit_exists(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest, m3_same_as_m2=True)
    summary = evaluate(manifest, tmp_path)
    assert summary.verdict == "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
    assert summary.selected_arm == "M3"
    assert "M3_trajectory_separation_below_diagnostic_target" in summary.reasons


def test_pilot_selects_simpler_m2_when_m3_has_no_incremental_net_benefit(
    tmp_path,
):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    runs = {run.arm: run for run in manifest.runs if run.repeat == 1}

    m2 = _evidence(manifest, runs["M2"])
    m2["passive"]["minimum_clearance_m"] = 0.25
    (tmp_path / f'{runs["M2"].run_id}.json').write_text(
        json.dumps(m2), encoding="utf-8"
    )

    m3 = _evidence(manifest, runs["M3"], m3_same_as_m2=True)
    m3["passive"]["minimum_clearance_m"] = 0.25
    m3["critic"]["near_obstacle_speed_mps"] = 0.3
    (tmp_path / f'{runs["M3"].run_id}.json').write_text(
        json.dumps(m3), encoding="utf-8"
    )

    summary = evaluate(manifest, tmp_path, pilot=True)

    assert summary.verdict == "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
    assert summary.selected_arm == "M2"
    assert summary.selection_outcome == (
        "NO_INCREMENTAL_BENEFIT_KEEP_M2_CRITIC_OFF"
    )
    assert "NO_INCREMENTAL_BENEFIT_KEEP_M2_CRITIC_OFF" in summary.reasons
    assert "M2_median_clearance_gain_below_diagnostic_target" in summary.reasons
    assert "M3_trajectory_separation_below_diagnostic_target" in summary.reasons
    assert summary.m2_vs_m1[0].clearance_gain_m == pytest.approx(0.04)
    assert summary.m3_vs_m2[0].clearance_gain_m == 0.0
    assert summary.m3_vs_m2[0].hausdorff_m == 0.0
    assert manifest.criteria["source_recall_min"] == 0.80
    assert manifest.criteria["candidate_precision_min"] == 0.50
    assert manifest.criteria["candidate_radius_max_m"] == 0.35


def test_invalid_m3_does_not_block_valid_m2_selection(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(
        item for item in manifest.runs if item.repeat == 1 and item.arm == "M3"
    )
    row = _evidence(manifest, run)
    row["module2_uds_connected"] = False
    (tmp_path / f"{run.run_id}.json").write_text(
        json.dumps(row), encoding="utf-8"
    )

    summary = evaluate(manifest, tmp_path, pilot=True)
    result = next(item for item in summary.runs if item.arm == "M3")

    assert result.verdict == "INVALID"
    assert summary.verdict == "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
    assert summary.selected_arm == "M2"
    assert summary.selection_outcome == "M3_NOT_ADMITTED_EVIDENCE_INSUFFICIENT"
    assert "M3_NOT_ADMITTED_EVIDENCE_INSUFFICIENT" in summary.reasons
    assert "NO_INCREMENTAL_BENEFIT_KEEP_M2_CRITIC_OFF" not in summary.reasons
    assert summary.formal_qualification is False
    assert summary.phase_f_complete is False
    assert summary.selected_arm_active_ttl_status == "PENDING"


def test_path_and_clearance_diagnostic_targets_do_not_block_m2_selection(
    tmp_path,
):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    runs = {run.arm: run for run in manifest.runs if run.repeat == 1}

    m1 = _evidence(manifest, runs["M1"])
    m1["plan"] = [[0.45, -5.35], [2.0, -0.2], [0.80, 4.80]]
    (tmp_path / f'{runs["M1"].run_id}.json').write_text(
        json.dumps(m1), encoding="utf-8"
    )

    m2 = _evidence(manifest, runs["M2"])
    m2["plan"] = m1["plan"]
    m2["optimal_trajectory"] = m1["plan"]
    m2["passive"]["minimum_clearance_m"] = 0.19
    (tmp_path / f'{runs["M2"].run_id}.json').write_text(
        json.dumps(m2), encoding="utf-8"
    )

    m3 = _evidence(manifest, runs["M3"])
    m3["module2_uds_connected"] = False
    (tmp_path / f'{runs["M3"].run_id}.json').write_text(
        json.dumps(m3), encoding="utf-8"
    )

    summary = evaluate(manifest, tmp_path, pilot=True)

    assert summary.selected_arm == "M2"
    assert summary.verdict == "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
    assert "M1_vs_M0_path_similarity_below_diagnostic_target" in summary.reasons
    assert "M2_median_clearance_gain_below_diagnostic_target" in summary.reasons


def test_invalid_m2_cannot_be_selected_or_rescued_by_m3(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(
        item for item in manifest.runs if item.repeat == 1 and item.arm == "M2"
    )
    row = _evidence(manifest, run)
    row["synchronized_samples"][0]["typed_obstacles"][0]["radius_m"] = 0.36
    (tmp_path / f"{run.run_id}.json").write_text(
        json.dumps(row), encoding="utf-8"
    )

    summary = evaluate(manifest, tmp_path, pilot=True)

    assert summary.verdict == "INVALID"
    assert summary.selected_arm is None
    assert summary.selection_outcome == "NOT_SELECTED_INVALID_EVIDENCE"
    assert summary.selected_arm_active_ttl_status == "NOT_APPLICABLE_NO_SELECTION"
    assert any(
        "M2 candidate radius exceeds engineering threshold" in reason
        for reason in summary.reasons
    )


@pytest.mark.parametrize("scope", ["global", "local"])
def test_m2_requires_obstacle_layer_applied_status(tmp_path, scope):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(
        item for item in manifest.runs if item.repeat == 1 and item.arm == "M2"
    )
    row = _evidence(manifest, run)
    row["layer"][scope]["applied_count"] = 0
    (tmp_path / f"{run.run_id}.json").write_text(
        json.dumps(row), encoding="utf-8"
    )

    summary = evaluate(manifest, tmp_path, pilot=True)
    result = next(item for item in summary.runs if item.arm == "M2")

    assert result.verdict == "INVALID"
    assert "active obstacle layer lacks global/local applied status" in result.reasons
    assert summary.selected_arm is None


def test_evaluate_cli_zero_is_pilot_candidate_only(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    pilot_output = tmp_path / "pilot-summary.json"
    full_output = tmp_path / "full-summary.json"

    assert cli([
        "evaluate", "--config", str(CONFIG),
        "--evidence-dir", str(tmp_path), "--pilot",
        "--output", str(pilot_output),
    ]) == 0
    assert cli([
        "evaluate", "--config", str(CONFIG),
        "--evidence-dir", str(tmp_path),
        "--output", str(full_output),
    ]) == 2
    for output in (pilot_output, full_output):
        payload = json.loads(output.read_text(encoding="utf-8"))
        assert payload["verdict"] == (
            "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
        )
        assert payload["formal_qualification"] is False
        assert payload["phase_f_complete"] is False
        assert payload["selected_arm_active_ttl_status"] == "PENDING"


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


def test_nominal_post_route_stale_counter_is_not_ttl_qualification(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[2]
    (tmp_path / f"{run.run_id}.json").write_text(
        json.dumps(_evidence(manifest, run, stale=True)), encoding="utf-8"
    )
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "VALID"
    assert json.loads(
        (tmp_path / f"{run.run_id}.json").read_text(encoding="utf-8")
    )["freshness"]["external_active_controller_probe_required"] is True


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


def test_unavailable_invalid_facts_are_null_not_false_or_zero(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[0]
    (tmp_path / f"{run.run_id}.json").unlink()

    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)

    assert result.verdict == "INVALID"
    assert result.collision is None
    assert result.success is None
    assert result.terminal_zero_confirmed is None
    assert result.source_recall is None
    assert result.critic_applied is None
    assert result.critic_participation == "unavailable"


@pytest.mark.parametrize("arm_name", ["M1", "M2"])
def test_prevalidation_failure_preserves_raw_collision_and_action_facts(
    tmp_path, arm_name
):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == arm_name)
    row = _evidence(manifest, run)
    row["module2_uds_connected"] = False
    row["passive"].update({"collision": True, "success": False})
    row["action"].update({
        "state": "STOP",
        "terminal_zero_confirmed": True,
    })
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")

    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)

    assert result.verdict == "INVALID"
    assert result.collision is True
    assert result.success is False
    assert result.action_state == "STOP"
    assert result.terminal_zero_confirmed is True
    assert result.source_visible_count == 1
    assert result.source_matched_count == 1
    assert result.source_recall == 1.0
    assert summary.verdict == "INVALID"


def test_invalid_m3_preserves_raw_critic_application_facts(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M3")
    row = _evidence(manifest, run)
    row["module2_uds_connected"] = False
    row["critic"].update({
        "applied": True,
        "reason": "cost_delta_applied=true;obstacle_applied=true",
        "status_count": 645,
        "applied_count": 645,
    })
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")

    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)

    assert result.verdict == "INVALID"
    assert result.critic_applied is True
    assert result.critic_status_count == 645
    assert result.critic_applied_count == 645
    assert result.critic_participation == "online_applied"


def test_late_invalid_run_preserves_successfully_computed_source_metrics(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M2")
    row = _evidence(manifest, run)
    row["freshness"]["ttl_clear_applicability"] = "invalid"
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")

    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)

    assert result.verdict == "INVALID"
    assert result.source_visible_count == 1
    assert result.source_matched_count == 1
    assert result.source_recall == 1.0
    assert result.candidate_true_positive_count == 1
    assert result.candidate_false_positive_count == 0
    assert result.candidate_precision == 1.0


@pytest.mark.parametrize("arm_name", ["M0", "M1"])
@pytest.mark.parametrize("outcome", ["collision", "navigation_failed"])
def test_baseline_collision_and_navigation_failure_are_valid_causal_rows(
    tmp_path, arm_name, outcome
):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == arm_name)
    row = _evidence(manifest, run)
    row["passive"]["success"] = False
    if outcome == "collision":
        row["passive"]["collision"] = True
    else:
        row["passive"]["collision"] = False
        row["action"].update({"state": "FAILED", "stop_reason": "navigation_failed"})
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "VALID"
    assert result.success is False
    assert result.reasons == (("collision",) if outcome == "collision" else ("navigation_failed",))
    assert summary.verdict != "INVALID"
    assert summary.m1_vs_m0


def test_missing_terminal_zero_is_invalid_evidence(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[0]
    row = _evidence(manifest, run)
    row["action"]["terminal_zero_confirmed"] = False
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert result.reasons == ("terminal_zero_not_confirmed",)
    assert summary.verdict == "INVALID"


def test_active_collision_is_valid_evidence_but_safety_fails_campaign(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M2")
    row = _evidence(manifest, run)
    row["passive"].update({"collision": True, "success": False})
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "VALID"
    assert result.collision is True
    assert summary.verdict == "FAIL"
    assert "M2_collision_safety_stop" in summary.reasons


def test_invalid_source_row_keeps_pair_diagnostics_and_reports_all_blockers(
    tmp_path,
):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    runs = {run.arm: run for run in manifest.runs if run.repeat == 1}

    m1 = _evidence(manifest, runs["M1"])
    m1["plan"] = [[0.45, -5.35], [2.0, -0.2], [0.80, 4.80]]
    (tmp_path / f'{runs["M1"].run_id}.json').write_text(
        json.dumps(m1), encoding="utf-8"
    )

    straight = [[0.45, -5.35], [0.625, -0.275], [0.80, 4.80]]
    m2 = _evidence(manifest, runs["M2"])
    missed = json.loads(json.dumps(m2["synchronized_samples"][0]))
    missed["stamp_ns"] += 1
    missed["typed_obstacles"] = []
    m2["synchronized_samples"].append(missed)
    m2["plan"] = straight
    m2["optimal_trajectory"] = straight
    m2["passive"]["minimum_clearance_m"] = 0.25
    (tmp_path / f'{runs["M2"].run_id}.json').write_text(
        json.dumps(m2), encoding="utf-8"
    )

    m3 = _evidence(manifest, runs["M3"])
    m3["optimal_trajectory"] = straight
    m3["passive"]["minimum_clearance_m"] = 0.25
    m3["critic"]["near_obstacle_speed_mps"] = 0.3
    (tmp_path / f'{runs["M3"].run_id}.json').write_text(
        json.dumps(m3), encoding="utf-8"
    )

    summary = evaluate(manifest, tmp_path, pilot=True)

    assert summary.verdict == "INVALID"
    assert any(
        "M2 source recall below engineering threshold" in reason
        for reason in summary.reasons
    )
    assert "M1_vs_M0_path_similarity_below_diagnostic_target" in summary.reasons
    assert "M2_median_clearance_gain_below_diagnostic_target" in summary.reasons
    assert "M2_reroute_direction_inconsistent" in summary.reasons
    assert "M3_trajectory_separation_below_diagnostic_target" in summary.reasons
    assert "M3_NO_INCREMENTAL_BENEFIT_DIAGNOSTIC" in summary.reasons
    assert len(summary.reasons) == len(set(summary.reasons))
    assert summary.selected_arm is None
    assert summary.selection_outcome == "NOT_SELECTED_INVALID_EVIDENCE"
    assert len(summary.m2_vs_m1) == 1
    assert len(summary.m3_vs_m2) == 1
    assert summary.m2_vs_m1[0].diagnostic_when_invalid is True
    assert summary.m3_vs_m2[0].diagnostic_when_invalid is True
    assert summary.m1_vs_m0[0].diagnostic_when_invalid is False
    assert summary.m3_vs_m2[0].hausdorff_m == 0.0
    assert manifest.criteria["source_recall_min"] == 0.80
    assert manifest.criteria["active_clearance_gain_min_m"] == 0.20
    assert manifest.criteria["m3_m2_trajectory_separation_min_m"] == 0.05


def test_m1_missing_uds_is_invalid_evidence(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M1")
    row = _evidence(manifest, run)
    row["module2_uds_connected"] = False
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert result.reasons == ("M1 requires Module2 UDS evidence",)


def test_zero_scan_message_count_is_invalid_not_lidar_invisibility(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[0]
    row = _evidence(manifest, run)
    row["sensor_counts"]["scan_message_count"] = 0
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert result.reasons == ("/scan message count must be positive",)


def test_low_obstacle_scan_hit_is_invalid_for_invisibility_claim(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = manifest.runs[0]
    row = _evidence(manifest, run)
    row["synchronized_samples"][0]["scan_hits_in_obstacle_footprints"] = 1
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert "low-obstacle scan hit count must be zero" in result.reasons[0]


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
            "header": {"stamp": stamp, "frame_id": "map"},
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
            "header": {"stamp": stamp, "frame_id": "map"},
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
            "header": {"stamp": stamp + 100_000_000, "frame_id": "map"},
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
        "reset_receipt": {"generation": 1},
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
    assert evidence["reset"]["calls"] == 1
    assert evidence["reset"]["events"] == 1
    assert evidence["reset"]["target_reset_epoch"] == 1
    assert evidence["module2_health"]["trusted_write_count"] == 2
    assert evidence["module2_health"]["candidate_cadence_hz"] == pytest.approx(10.0)
    assert evidence["layer"]["global"]["cells"] == 7
    assert evidence["layer"]["local"]["cells"] == 5
    assert evidence["critic"]["cost_delta_nonzero_count"] == 1
    assert evidence["freshness"]["ttl_expiry_observed"] is True
    assert evidence["freshness"]["ttl_expiry_zero_write"] is True
    assert evidence["freshness"]["ttl_expiry_critic_not_applied"] is None
    assert evidence["freshness"]["critic_ttl_status"] == causal.NOMINAL_TTL_STATUS
    assert evidence["freshness"]["nominal_post_route_critic_observation"] == (
        "N/A_NO_CONTROLLER_SCORING"
    )
    assert evidence["freshness"]["external_active_controller_probe_required"] is True
    assert evidence["freshness"]["critic_post_expiry_applied"] is False
    assert evidence["freshness"]["critic_stale_active_probe"] == "NOT_RUN"
    assert evidence["sensor_counts"]["scan_message_count"] == 1
    assert evidence["synchronized_samples"][0]["scan_point_count"] == 80
    assert evidence["synchronized_samples"][0]["depth_observation_valid"] is True
    assert evidence["synchronized_samples"][0]["rgbd_obstacle_footprints"][0]["source"] == (
        causal.PHYSICAL_DEPTH_FOOTPRINT_SOURCE
    )
    assert evidence["synchronized_samples"][0]["typed_obstacles"][0]["observed_spatial_error_m"] is not None
    assert evidence["passive"]["success"] is True


def test_recorder_uses_receipt_generation_and_excludes_other_episode_records():
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M2")

    def message(epoch, sequence):
        return {
            "header": {"stamp": 2_000_000_000, "frame_id": "map"},
            "reset_epoch": epoch,
            "module2_healthy": True,
            "trusted_write": True,
            "observation_valid": True,
            "validation_stamp": {"sec": 2, "nanosec": 0},
            "sequence": sequence,
            "validation_mode": causal.VALIDATION_STATIC_DEPTH_REVALIDATED,
            "validation_sensor_mask": causal.VALIDATION_SENSOR_DEPTH,
            "obstacles": [{
                "id": f"candidate-{sequence}",
                "pose_xy_m": [-0.45, -0.35],
                "radius_m": 0.2,
                "confidence": 0.9,
            }],
        }

    evidence = build_recorded_evidence(
        manifest,
        run,
        [
            RecordedMessage(
                "/bio_nav/module2/cognitive_obstacles", 250, message(1, 1)
            ),
            RecordedMessage(
                "/bio_nav/module2/cognitive_obstacles", 300, message(2, 2)
            ),
            RecordedMessage(
                "/bio_nav/module2/cognitive_obstacles", 350, message(3, 3)
            ),
            RecordedMessage(
                "/bio_nav/module2/cognitive_obstacles", 500, message(2, 4)
            ),
            RecordedMessage("/ground_truth/odom", 350, {
                "pose": {"pose": {"position": {"x": -1.0, "y": -1.0}}},
            }),
        ],
        {
            "reset_receipt": {"generation": 2},
            "reset_events": 1,
            "_evidence_window": {"start_ns": 200, "end_ns": 400},
        },
    )
    assert evidence["module2_health"]["message_count"] == 1
    assert [row["id"] for row in evidence["obstacle_validation"]] == ["candidate-2"]
    assert evidence["reset"]["target_reset_epoch"] == 2
    assert evidence["reset"]["excluded_record_count"] == 3


@pytest.mark.parametrize(
    ("target_reset_epoch", "expected_candidate", "expected_excluded"),
    [
        (2, "candidate-1", 1),
        (3, "candidate-2", 1),
    ],
)
def test_recorder_accepts_current_or_legacy_next_target_epoch_and_isolates_records(
    target_reset_epoch, expected_candidate, expected_excluded
):
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M2")

    def message(epoch, sequence):
        return {
            "header": {"stamp": 2_000_000_000, "frame_id": "map"},
            "reset_epoch": epoch,
            "module2_healthy": True,
            "trusted_write": True,
            "observation_valid": True,
            "validation_stamp": {"sec": 2, "nanosec": 0},
            "sequence": sequence,
            "validation_mode": causal.VALIDATION_STATIC_DEPTH_REVALIDATED,
            "validation_sensor_mask": causal.VALIDATION_SENSOR_DEPTH,
            "obstacles": [{
                "id": f"candidate-{sequence}",
                "pose_xy_m": [-0.45, -0.35],
                "radius_m": 0.2,
                "confidence": 0.9,
            }],
        }

    evidence = build_recorded_evidence(
        manifest,
        run,
        [
            RecordedMessage(
                "/bio_nav/module2/cognitive_obstacles", 250, message(2, 1)
            ),
            RecordedMessage(
                "/bio_nav/module2/cognitive_obstacles", 300, message(3, 2)
            ),
            RecordedMessage("/ground_truth/odom", 350, {
                "pose": {"pose": {"position": {"x": -1.0, "y": -1.0}}},
            }),
        ],
        {
            "reset_receipt": {"generation": 2},
            "target_reset_epoch": target_reset_epoch,
            "_evidence_window": {"start_ns": 200, "end_ns": 400},
        },
    )
    assert evidence["module2_health"]["message_count"] == 1
    assert [row["id"] for row in evidence["obstacle_validation"]] == [
        expected_candidate
    ]
    assert evidence["reset"]["target_reset_epoch"] == target_reset_epoch
    assert evidence["reset"]["excluded_record_count"] == expected_excluded


@pytest.mark.parametrize(
    "episode_result",
    [
        {},
        {"reset_receipt": {}},
        {"reset_receipt": {"generation": 0}},
        {"reset_receipt": {"generation": -1}},
        {"reset_receipt": {"generation": True}},
        {"reset_receipt": {"generation": "2"}},
    ],
)
def test_recorder_rejects_missing_or_invalid_receipt_generation(episode_result):
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M2")
    with pytest.raises(
        causal.CausalContractError,
        match=r"reset_receipt\.generation must be a positive integer",
    ):
        build_recorded_evidence(manifest, run, [], episode_result)


@pytest.mark.parametrize(
    "episode_result",
    [
        {"target_reset_epoch": 3},
        {
            "reset_receipt": {"generation": 0},
            "target_reset_epoch": 3,
        },
    ],
)
def test_recorder_rejects_invalid_receipt_even_with_explicit_target(episode_result):
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M2")
    with pytest.raises(
        causal.CausalContractError,
        match=r"reset_receipt\.generation must be a positive integer",
    ):
        build_recorded_evidence(manifest, run, [], episode_result)


@pytest.mark.parametrize("target_reset_epoch", [0, -1, True, "3", None])
def test_recorder_rejects_invalid_explicit_target_epoch(target_reset_epoch):
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M2")
    with pytest.raises(
        causal.CausalContractError,
        match="target_reset_epoch must be a positive integer",
    ):
        build_recorded_evidence(
            manifest,
            run,
            [],
            {
                "reset_receipt": {"generation": 2},
                "target_reset_epoch": target_reset_epoch,
            },
        )


def test_recorder_rejects_unrelated_explicit_target_epoch():
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M2")
    with pytest.raises(
        causal.CausalContractError,
        match=(
            r"target_reset_epoch must equal reset_receipt\.generation "
            r"or reset_receipt\.generation \+ 1"
        ),
    ):
        build_recorded_evidence(
            manifest,
            run,
            [],
            {
                "reset_receipt": {"generation": 2},
                "target_reset_epoch": 100,
            },
        )


@pytest.mark.parametrize(("terminal_ns", "expected_end"), [(350, 350), (None, 400)])
def test_episode_jsonl_bounds_evidence_by_reset_receipt_and_terminal_or_result(
    tmp_path, terminal_ns, expected_end
):
    rows = [
        {"event": "reset_receipt", "wall_time_ns": 200, "generation": 2},
    ]
    if terminal_ns is not None:
        rows.append({"event": "terminal_zero_confirmed", "wall_time_ns": terminal_ns})
    rows.append({
        "event": "episode_result",
        "wall_time_ns": 400,
        "reset_receipt": {"generation": 2},
    })
    path = tmp_path / "episode.jsonl"
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    result = causal._episode_result_from_jsonl(path)
    assert result["_evidence_window"] == {
        "start_ns": 200,
        "end_ns": expected_end,
        "end_event": (
            "terminal_zero_confirmed" if terminal_ns is not None else "episode_result"
        ),
    }


def test_m1_empty_startup_message_does_not_violate_shadow_geometry_semantics(tmp_path):
    empty = RecordedMessage("/bio_nav/module2/cognitive_obstacles", 1, {
        "trusted_write": False,
        "rejection_mask": 0,
        "obstacles": [],
    })
    nonempty_message = _typed_obstacle_message(
        pose=[-0.45, -0.35], position_stddev=[0.05, 0.05]
    )
    nonempty_message.trusted_write = False
    nonempty_message.rejection_mask = causal.SHADOW_REJECTION_UNTRUSTED
    nonempty_message.validation_mode = causal.VALIDATION_STATIC_DEPTH_REVALIDATED
    nonempty_message.validation_sensor_mask = causal.VALIDATION_SENSOR_DEPTH
    summary = causal._m1_shadow_candidate_summary([
        empty,
        RecordedMessage("/bio_nav/module2/cognitive_obstacles", 2, nonempty_message),
    ])
    assert summary == {
        "message_count": 2,
        "nonempty_message_count": 1,
        "static_depth_revalidated_geometry_count": 1,
        "trusted_write_count": 0,
        "shadow_rejection_count": 1,
        "invalid_geometry_count": 0,
    }
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M1")
    row = _evidence(manifest, run)
    row["shadow_obstacle_candidate"] = summary
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    result = next(item for item in evaluate(manifest, tmp_path).runs if item.run_id == run.run_id)
    assert result.verdict == "VALID"


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
        {"reset_receipt": {"generation": 1}},
    )
    assert evidence["freshness"]["critic_ttl_status"] == causal.NOMINAL_TTL_STATUS
    assert evidence["freshness"]["nominal_post_route_critic_observation"] == (
        "FAIL_POST_EXPIRY_APPLIED"
    )
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
        {"reset_receipt": {"generation": 1}},
    )
    assert evidence["freshness"]["critic_ttl_status"] == causal.NOMINAL_TTL_STATUS
    assert evidence["freshness"]["nominal_post_route_critic_observation"] == (
        "STALE_REJECTED"
    )
    assert evidence["freshness"]["critic_post_expiry_applied"] is False
    assert evidence["freshness"]["ttl_expiry_critic_not_applied"] is True


def test_recorder_requires_both_costmap_layers_to_clear_after_expiry():
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M3")
    evidence = build_recorded_evidence(
        manifest,
        run,
        _m3_ttl_lifecycle_records(include_local=False),
        {"reset_receipt": {"generation": 1}},
    )
    assert evidence["freshness"]["ttl_expiry_zero_write"] is False
    assert evidence["freshness"]["ttl_expiry_observed"] is False
    assert evidence["freshness"]["critic_ttl_status"] == causal.NOMINAL_TTL_STATUS
    assert evidence["freshness"]["nominal_post_route_critic_observation"] == (
        "N/A_NO_CONTROLLER_SCORING"
    )


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
        footprint = result["footprints"][0]
        assert footprint["source"] == causal.PHYSICAL_DEPTH_FOOTPRINT_SOURCE
        assert footprint["point_count"] == footprint["hit_count"] >= 3
        assert footprint["center"][:2] == pytest.approx([-0.45, -0.35])
        assert footprint["rectangle"] == pytest.approx([-0.60, -0.50, -0.30, -0.20])


def test_ground_only_depth_points_do_not_prove_low_box_visibility():
    manifest = load_manifest(CONFIG)
    depth, info, _ = _depth_projection_inputs()
    ground_transform = RecordedMessage("/tf_static", depth.stamp_ns, {
        "transforms": [{
            "header": {"stamp": depth.stamp_ns, "frame_id": "map"},
            "child_frame_id": "camera_optical",
            "transform": {
                "translation": {"x": -0.45, "y": -0.35, "z": -1.0},
                "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }],
    })
    result = causal._project_depth_obstacle(
        depth,
        info,
        [],
        [ground_transform],
        causal._load_frozen_obstacle(manifest),
        manifest.criteria,
    )
    assert result["valid"] is True
    assert result["reason"] == "no_low_points_in_obstacle_bounds"
    assert result["hit_count"] == 0
    assert result["footprints"] == []


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


def test_camera_info_numpy_intrinsics_are_accepted_and_invalid_arrays_fail_closed():
    numpy = pytest.importorskip("numpy")
    manifest = load_manifest(CONFIG)
    depth, info, transforms = _depth_projection_inputs()
    numpy_info = RecordedMessage(info.topic, info.stamp_ns, {
        **info.message,
        "k": numpy.asarray(info.message["k"], dtype=numpy.float64),
    })
    observed = causal._project_depth_obstacle(
        depth, numpy_info, [], transforms,
        causal._load_frozen_obstacle(manifest), manifest.criteria,
    )
    assert observed["valid"] is True
    assert observed["reason"] == "observed"

    for invalid_k in (None, numpy.asarray([100.0, 0.0]), numpy.asarray(1.0)):
        invalid_info = RecordedMessage(info.topic, info.stamp_ns, {
            **info.message,
            "k": invalid_k,
        })
        rejected = causal._project_depth_obstacle(
            depth, invalid_info, [], transforms,
            causal._load_frozen_obstacle(manifest), manifest.criteria,
        )
        assert rejected["valid"] is False
        assert rejected["reason"] == "invalid_camera_intrinsics"
        assert rejected["footprints"] == []


def _typed_obstacle_message(*, pose, position_stddev):
    return SimpleNamespace(
        input_healthy=True,
        module2_healthy=True,
        observation_valid=True,
        trusted_write=True,
        validation_mode=causal.VALIDATION_STATIC_DEPTH_REVALIDATED,
        validation_sensor_mask=causal.VALIDATION_SENSOR_DEPTH,
        rejection_mask=0,
        obstacles=[SimpleNamespace(
            id="v6_low_box_solo",
            class_id="unknown_low_obstacle",
            pose_xy_m=pose,
            radius_m=0.2,
            height_m=0.15,
            confidence=0.9,
            reliability=0.8,
            ood_probability=0.1,
            position_stddev_m=position_stddev,
            count=3,
            motion_class=causal.MOTION_STATIC,
            static_confirmed=True,
        )],
    )


def test_typed_obstacles_accept_real_ndarray_and_match_list_and_array():
    from array import array

    numpy = pytest.importorskip("numpy")
    expected = [{
        "id": "v6_low_box_solo",
        "x": -0.45,
        "y": -0.35,
        "radius_m": 0.2,
        "confidence": 0.9,
        "accepted": True,
        "trusted_write": True,
        "validation_mode": causal.VALIDATION_STATIC_DEPTH_REVALIDATED,
        "validation_sensor_mask": causal.VALIDATION_SENSOR_DEPTH,
        "rejection_mask": 0,
        "motion_class": causal.MOTION_STATIC,
        "static_confirmed": True,
    }]
    variants = (
        ([-0.45, -0.35], [0.05, 0.06]),
        (
            numpy.asarray([-0.45, -0.35], dtype=numpy.float64),
            numpy.asarray([0.05, 0.06], dtype=numpy.float64),
        ),
        (array("d", [-0.45, -0.35]), array("d", [0.05, 0.06])),
    )
    for pose, position_stddev in variants:
        assert causal._typed_obstacles(_typed_obstacle_message(
            pose=pose, position_stddev=position_stddev,
        )) == expected


def test_typed_obstacles_invalid_arrays_and_nonfinite_values_fail_closed():
    numpy = pytest.importorskip("numpy")
    invalid_arrays = (
        (numpy.asarray(1.0), numpy.asarray([0.05, 0.06])),
        (numpy.asarray([-0.45]), numpy.asarray([0.05, 0.06])),
        (numpy.asarray([-0.45, numpy.nan]), numpy.asarray([0.05, 0.06])),
        (numpy.asarray([-0.45, -0.35]), numpy.asarray([0.05])),
        (numpy.asarray([-0.45, -0.35]), numpy.asarray([0.05, numpy.inf])),
        (numpy.asarray([-0.45, -0.35]), numpy.asarray([0.05, -0.01])),
    )
    for pose, position_stddev in invalid_arrays:
        assert causal._typed_obstacles(_typed_obstacle_message(
            pose=pose, position_stddev=position_stddev,
        )) == []

    for field_name, invalid_value in (
        ("radius_m", 0.0),
        ("radius_m", numpy.nan),
        ("height_m", numpy.inf),
        ("confidence", 1.1),
        ("confidence", numpy.nan),
        ("reliability", -0.1),
        ("ood_probability", numpy.inf),
        ("count", 0),
    ):
        message = _typed_obstacle_message(
            pose=numpy.asarray([-0.45, -0.35]),
            position_stddev=numpy.asarray([0.05, 0.06]),
        )
        setattr(message.obstacles[0], field_name, invalid_value)
        assert causal._typed_obstacles(message) == []


def test_typed_obstacles_transform_source_frame_at_validation_stamp():
    numpy = pytest.importorskip("numpy")
    stamp = 2_000_000_000
    message = _typed_obstacle_message(
        pose=numpy.asarray([0.0, 0.0]),
        position_stddev=numpy.asarray([0.05, 0.06]),
    )
    message.header = SimpleNamespace(frame_id="base_link")
    message.validation_stamp = SimpleNamespace(sec=2, nanosec=0)
    transform = RecordedMessage("/tf", stamp, {"transforms": [{
        "header": {"stamp": stamp, "frame_id": "map"},
        "child_frame_id": "base_link",
        "transform": {
            "translation": {"x": -0.45, "y": -0.35, "z": 0.0},
            "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        },
    }]})
    observed = causal._typed_obstacles(
        message,
        tf_records=[transform],
        target_frame="map",
    )
    assert [(row["x"], row["y"]) for row in observed] == [(-0.45, -0.35)]
    assert causal._typed_obstacles(message, target_frame="map") == []


def test_m1_health_is_reduced_from_planning_prior_not_typed_obstacles():
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M1")
    stamp = 2_000_000_000
    records = [
        RecordedMessage("/bio_nav/module2/planning_prior", stamp + offset, {
            "module2_healthy": healthy,
            "observation_valid": healthy,
            "trusted_write": False,
        })
        for offset, healthy in ((0, True), (100_000_000, True), (200_000_000, False))
    ]
    records.append(RecordedMessage("/ground_truth/odom", stamp, {
        "pose": {"pose": {"position": {"x": -1.0, "y": -1.0}}},
    }))
    evidence = build_recorded_evidence(manifest, run, records, {
        "state": "SUCCEEDED",
        "reset_receipt": {"generation": 1},
        "reset_calls": 1,
        "reset_events": 1,
        "goal_publications": 1,
        "terminal_zero_confirmed": True,
    })
    assert evidence["module2_uds_connected"] is True
    assert evidence["module2_health"] == {
        "message_count": 3,
        "healthy_count": 2,
        "trusted_write_count": 0,
        "observation_valid_count": 2,
        "candidate_cadence_hz": pytest.approx(10.0),
        "scope": "low_obstacle_only",
    }
    assert evidence["obstacle_validation"] == []
    assert evidence["planning_prior"][0]["module2_healthy"] is True


def test_m1_recorder_accepts_numpy_fixed_array_shadow_candidate():
    numpy = pytest.importorskip("numpy")
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M1")
    message = _typed_obstacle_message(
        pose=numpy.asarray([-0.45, -0.35], dtype=numpy.float64),
        position_stddev=numpy.asarray([0.05, 0.06], dtype=numpy.float64),
    )
    message.trusted_write = False
    message.rejection_mask = causal.SHADOW_REJECTION_UNTRUSTED
    message.header = SimpleNamespace(
        stamp=SimpleNamespace(sec=2, nanosec=0), frame_id="map"
    )
    message.validation_stamp = SimpleNamespace(sec=2, nanosec=100_000_000)
    records = [
        RecordedMessage("/bio_nav/module2/cognitive_obstacles", 2_100_000_000, message),
        RecordedMessage("/bio_nav/module2/planning_prior", 2_000_000_000, {
            "module2_healthy": True,
            "observation_valid": True,
            "trusted_write": False,
        }),
        RecordedMessage("/bio_nav/module2/planning_prior", 2_100_000_000, {
            "module2_healthy": True,
            "observation_valid": True,
            "trusted_write": False,
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", 2_100_000_000, {
            "consumer": "/global_costmap:layer", "applied": False,
            "raised_cell_count": 0, "active_cell_count": 0,
            "maximum_cost_increase": 0,
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", 2_100_000_000, {
            "consumer": "/local_costmap:layer", "applied": False,
            "raised_cell_count": 0, "active_cell_count": 0,
            "maximum_cost_increase": 0,
        }),
        RecordedMessage("/ground_truth/odom", 2_100_000_000, {
            "pose": {"pose": {"position": {"x": -1.0, "y": -1.0}}},
        }),
    ]
    evidence = build_recorded_evidence(
        manifest,
        run,
        records,
        {"reset_receipt": {"generation": 1}},
    )
    assert evidence["shadow_obstacle_candidate"] == {
        "message_count": 1,
        "nonempty_message_count": 1,
        "static_depth_revalidated_geometry_count": 1,
        "trusted_write_count": 0,
        "shadow_rejection_count": 1,
        "invalid_geometry_count": 0,
    }
    assert evidence["obstacle_validation"][0]["trusted_write"] is False
    assert evidence["module2_health"]["healthy_count"] == 2


def test_m1_evaluator_rejects_missing_typed_shadow_candidate(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M1")
    row = _evidence(manifest, run)
    row["synchronized_samples"][0].update({
        "depth_observation_valid": "not_applicable",
        "depth_observation_reason": None,
        "depth_point_count": None,
        "depth_hits_in_obstacle_bounds": None,
        "rgbd_obstacle_footprints": [],
        "typed_obstacles": [],
    })
    row["obstacle_validation"] = []
    row["shadow_obstacle_candidate"].update({
        "message_count": 0,
        "nonempty_message_count": 0,
        "static_depth_revalidated_geometry_count": 0,
        "shadow_rejection_count": 0,
    })
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    assert "M1 lacks non-empty static-depth-revalidated" in result.reasons[0]


@pytest.mark.parametrize("fault", ["trusted", "wrong_rejection", "layer_write"])
def test_m1_evaluator_rejects_non_shadow_or_layer_write_evidence(tmp_path, fault):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    run = next(item for item in manifest.runs if item.arm == "M1")
    row = _evidence(manifest, run)
    if fault == "trusted":
        row["shadow_obstacle_candidate"]["trusted_write_count"] = 1
    elif fault == "wrong_rejection":
        row["shadow_obstacle_candidate"]["shadow_rejection_count"] = 0
    else:
        row["layer"]["local"]["applied_count"] = 1
    (tmp_path / f"{run.run_id}.json").write_text(json.dumps(row), encoding="utf-8")
    summary = evaluate(manifest, tmp_path)
    result = next(item for item in summary.runs if item.run_id == run.run_id)
    assert result.verdict == "INVALID"
    if fault == "layer_write":
        assert "M1 shadow obstacle layer applied or raised" in result.reasons[0]
        assert summary.selected_arm is None
        assert summary.selection_outcome == "NOT_SELECTED_INVALID_EVIDENCE"
    else:
        assert "M1 typed candidate violates untrusted shadow semantics" in result.reasons[0]


def test_phase_f_qos_records_transient_local_tf_static():
    document = yaml.safe_load(
        (PACKAGE / "config" / causal.PHASE_F_QOS_CONFIG).read_text(encoding="utf-8")
    )
    assert document["/tf_static"] == {
        "history": "keep_last",
        "depth": 100,
        "reliability": "reliable",
        "durability": "transient_local",
    }


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
        adapters=exact_adapter_templates(manifest, tmp_path / "module2-assets"),
        pilot=True,
        output_root=tmp_path,
    )
    assert exact["runs"][0]["commands"]["scene"][0].endswith(
        "run_v6_r5_phase_b_kujiale.sh"
    )
    assert exact["runs"][3]["commands"]["episode"][1] == "dispatch-episode"
    assert exact["runs"][2]["commands"]["producer_stop"][1] == "stop-producer"
    assert exact["runs"][2]["commands"]["producer_stop"][-2:] == (
        "--socket", exact["runs"][2]["setup"]["module2_socket"],
    )
    stack = exact["runs"][2]["commands"]["stack"]
    assert stack.count("--module2-asset-root") == 1
    assert stack[stack.index("--module2-asset-root") + 1] == str(
        tmp_path / "module2-assets"
    )
    assert "--module2-asset-root" not in exact["runs"][0]["commands"]["stack"]


def test_exact_plan_requires_explicit_asset_root_and_preserves_spaces(
    tmp_path, monkeypatch, capsys
):
    monkeypatch.setenv("BIO_NAV_MODULE2_ASSET_ROOT", "/fake/env/fallback")
    assert cli([
        "plan", "--config", str(CONFIG), "--exact-adapters",
    ]) == 2
    assert "--module2-asset-root is required with --exact-adapters" in (
        capsys.readouterr().err
    )

    asset_root = tmp_path / "module2 assets with spaces"
    assert cli([
        "plan", "--config", str(CONFIG), "--exact-adapters",
        "--module2-asset-root", str(asset_root), "--pilot",
        "--output-root", str(tmp_path / "plan"),
    ]) == 0
    plan = json.loads(capsys.readouterr().out)
    manifest = load_manifest(CONFIG)
    integration_contract = plan["integration_cli_contract"]
    stack_templates = plan["recommended_adapter_templates"]["stack"]
    assert integration_contract["M0"].count("--module2-asset-root") == 0
    assert stack_templates["M0"].count("--module2-asset-root") == 0
    for arm in ("M1", "M2", "M3"):
        assert integration_contract[arm].count("--module2-asset-root") == 1
        assert stack_templates[arm].count("--module2-asset-root") == 1
        values = causal._adapter_values(
            manifest,
            next(row for row in manifest.runs if row.arm == arm),
            tmp_path / "plan",
        )
        command = causal.render_adapter_command(stack_templates[arm], values)
        assert command[command.index("--module2-asset-root") + 1] == str(asset_root)
        dry_run = subprocess.run(
            [*command, "--dry-run"], capture_output=True, text=True, check=False
        )
        assert dry_run.returncode == 0, dry_run.stderr


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


def _run_campaign_with_fake_processes(
    tmp_path, monkeypatch, *, clear, startup_ready=True, cognitive_ready=True
):
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

    def fake_wait(managed, timeout_sec):
        events.append(("startup_reset_event_before_episode", 1))
        return {
            "ready": startup_ready,
            "generation": 1,
            "held": not startup_ready,
            "reason": "released:activation_gate" if startup_ready else "timeout",
        }

    def fake_cleanup(run_dir, socket, shutdown, env):
        events.append(("cleanup", None))
        return {"ok": True}

    def fake_cognitive(manifest_value, run_value, managed, socket, timeout_sec):
        events.append(("cognitive_readiness_before_episode", run_value.arm))
        return {
            "ready": cognitive_ready,
            "reason": "ready" if cognitive_ready else "timeout",
        }

    def fake_run(command, **kwargs):
        events.append(("run", command[0]))
        return SimpleNamespace(returncode=0)

    def fake_sleep(duration):
        events.append(("sleep", duration))

    def fake_record(*args, **kwargs):
        events.append(("record_evidence", None))
        return {"freshness": {
            "ttl_clear_applicability": causal.NOMINAL_TTL_STATUS,
            "external_active_controller_probe_required": True,
            "ttl_expiry_observed": clear,
            "ttl_expiry_zero_write": clear,
            "ttl_expiry_critic_not_applied": None,
        }}

    monkeypatch.setattr(causal, "_start_process", fake_start)
    monkeypatch.setattr(causal, "_stop_process", fake_stop)
    monkeypatch.setattr(causal, "_wait_for_startup_ready", fake_wait)
    monkeypatch.setattr(causal, "_wait_for_cognitive_ready", fake_cognitive)
    monkeypatch.setattr(causal, "_confirm_arm_cleanup", fake_cleanup)
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
    assert summary["runs"][0]["nominal_ttl_status"] == causal.NOMINAL_TTL_STATUS
    assert summary["runs"][0]["external_active_controller_probe_required"] is True
    assert events == [
        ("start", "scene"), ("start", "stack"),
        ("startup_reset_event_before_episode", 1),
        ("cognitive_readiness_before_episode", "M2"), ("start", "recorder"),
        ("run", "/episode"), ("run", "/producer-stop"),
        ("sleep", pytest.approx(1.5)),
        ("stop", "stack"), ("stop", "recorder"), ("stop", "scene"),
        ("cleanup", None),
        ("record_evidence", None),
    ]


def test_campaign_startup_timeout_never_starts_recorder_or_episode(
    tmp_path, monkeypatch
):
    summary, events = _run_campaign_with_fake_processes(
        tmp_path, monkeypatch, clear=True, startup_ready=False
    )
    assert summary["runs"][0]["state"] == "STARTUP_NOT_READY"
    assert ("start", "recorder") not in events
    assert not any(event[0] == "run" for event in events)
    assert events[:3] == [
        ("start", "scene"), ("start", "stack"),
        ("startup_reset_event_before_episode", 1),
    ]


def test_campaign_cognitive_timeout_never_starts_recorder_or_episode(
    tmp_path, monkeypatch
):
    summary, events = _run_campaign_with_fake_processes(
        tmp_path,
        monkeypatch,
        clear=True,
        startup_ready=True,
        cognitive_ready=False,
    )
    assert summary["runs"][0]["state"] == "MODULE2_NOT_READY"
    assert ("cognitive_readiness_before_episode", "M2") in events
    assert ("start", "recorder") not in events
    assert not any(event[0] == "run" for event in events)


@pytest.mark.parametrize(
    ("arm_name", "expected_run_state", "expected_campaign_state", "expected_rc"),
    [
        ("M0", "BASELINE_OUTCOME_RECORDED", "FINISHED", 0),
        ("M1", "BASELINE_OUTCOME_RECORDED", "FINISHED", 0),
        ("M2", "EPISODE_FAILED", "FINISHED_WITH_FAILURES", 2),
    ],
)
def test_campaign_records_expected_baseline_collision_but_active_collision_fails(
    tmp_path,
    monkeypatch,
    capsys,
    arm_name,
    expected_run_state,
    expected_campaign_state,
    expected_rc,
):
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == arm_name)
    one_run_manifest = replace(manifest, runs=(run,))

    monkeypatch.setattr(
        causal, "_start_process", lambda name, *args, **kwargs: SimpleNamespace(name=name)
    )
    monkeypatch.setattr(
        causal, "_stop_process",
        lambda process, timeout: {"name": process.name, "returncode": 0},
    )
    monkeypatch.setattr(
        causal, "_wait_for_startup_ready",
        lambda *args, **kwargs: {
            "ready": True,
            "generation": 1,
            "held": False,
            "reason": "released:activation_gate",
        },
    )
    monkeypatch.setattr(
        causal, "_wait_for_cognitive_ready",
        lambda *args, **kwargs: {"ready": True},
    )
    monkeypatch.setattr(
        causal, "_confirm_arm_cleanup", lambda *args, **kwargs: {"ok": True}
    )

    def fake_run(command, **kwargs):
        return SimpleNamespace(
            returncode=0 if command[0] == "/producer-stop" else 2
        )

    def fake_record(manifest_value, run_value, bag_dir, episode_jsonl, output):
        row = _evidence(manifest_value, run_value)
        row["passive"].update({"collision": True, "success": False})
        row["action"].update({
            "state": "STOP",
            "stop_reason": "collision_monitor_stop",
            "terminal_zero_confirmed": True,
        })
        Path(output).write_text(json.dumps(row), encoding="utf-8")
        return row

    monkeypatch.setattr(causal.subprocess, "run", fake_run)
    monkeypatch.setattr(causal.time, "sleep", lambda duration: None)
    monkeypatch.setattr(causal, "record_evidence_from_bag", fake_record)
    adapters = AdapterTemplates("/scene", "/stack", "/episode", "/producer-stop")
    summary = run_campaign(
        one_run_manifest,
        adapters,
        tmp_path / f"campaign-{arm_name}",
        pilot=False,
        shutdown_timeout_sec=0.1,
    )

    status = summary["runs"][0]
    evidence = json.loads(Path(status["evidence_file"]).read_text(encoding="utf-8"))
    assert status["state"] == expected_run_state
    assert status["episode_returncode"] == 2
    assert evidence["action"]["state"] == "STOP"
    assert evidence["action"]["stop_reason"] == "collision_monitor_stop"
    assert summary["state"] == expected_campaign_state
    if arm_name in {"M0", "M1"}:
        assert status["evidence_verdict"] == "VALID"
        assert status["baseline_outcome"] == "collision"

    monkeypatch.setattr(causal, "run_campaign", lambda *args, **kwargs: summary)
    assert cli([
        "run",
        "--config", str(CONFIG),
        "--scene-adapter", "/scene",
        "--stack-adapter", "/stack",
        "--episode-adapter", "/episode",
        "--producer-stop-adapter", "/producer-stop",
        "--module2-asset-root", str(tmp_path / "module2-assets"),
        "--output-root", str(tmp_path / f"cli-{arm_name}"),
    ]) == expected_rc
    capsys.readouterr()


def _canonical_prior(
    *,
    trusted=False,
    map_version=TEST_LIVE_MAP_VERSION,
    map_id=causal.KUJIALE_MAP_ID,
):
    mask = [False] * 256
    for state_id in causal.KUJIALE_VALID_STATE_IDS:
        mask[state_id] = True
    return SimpleNamespace(
        schema_version="bio_nav_planning_prior_v310",
        map_version=map_version,
        cognitive_tile_id=map_id,
        t_map_canvas=list(causal.KUJIALE_T_MAP_CANVAS),
        valid_state_mask=mask,
        trusted_write=trusted,
    )


def _canonical_constraints(
    *,
    map_version=TEST_LIVE_MAP_VERSION,
    map_id=causal.KUJIALE_MAP_ID,
):
    mask = [False] * 256
    for state_id in causal.KUJIALE_VALID_STATE_IDS:
        mask[state_id] = True
    return SimpleNamespace(
        map_version=map_version,
        cognitive_tile_id=map_id,
        t_map_canvas=list(causal.KUJIALE_T_MAP_CANVAS),
        reachable_state_mask=mask,
    )


def test_cognitive_prior_accepts_distinct_semantic_map_id_and_live_version():
    constraints = _canonical_constraints()
    prior = _canonical_prior()
    assert prior.map_version != prior.cognitive_tile_id
    assert causal._canonical_constraints_error(constraints) is None
    assert causal._canonical_prior_error(
        prior,
        "M2",
        expected_map_version=constraints.map_version,
    ) is None


def test_cognitive_prior_rejects_version_or_tile_mismatch():
    constraints = _canonical_constraints()
    assert "live CognitiveMapConstraints" in causal._canonical_prior_error(
        _canonical_prior(map_version="different-live-version"),
        "M2",
        expected_map_version=constraints.map_version,
    )
    assert "Kujiale map_id" in causal._canonical_prior_error(
        _canonical_prior(map_id="different-scene"),
        "M2",
        expected_map_version=constraints.map_version,
    )
    assert "Kujiale map_id" in causal._canonical_constraints_error(
        _canonical_constraints(map_id="different-scene")
    )


def test_cognitive_prior_rejects_wrong_mask_and_m1_trusted_write():
    wrong_mask = _canonical_prior()
    wrong_mask.valid_state_mask[0] = True
    assert "canonical 51-state mask" in causal._canonical_prior_error(wrong_mask, "M1")
    assert "untrusted shadow" in causal._canonical_prior_error(
        _canonical_prior(trusted=True), "M1"
    )
    assert causal._canonical_prior_error(_canonical_prior(trusted=True), "M2") is None


def test_cognitive_runtime_modes_are_arm_specific():
    assert causal._expected_cognitive_parameters("M1") == {
        "startup_profile": "estimated_shadow",
        "module2_mode": "shadow",
        "active_effect_scope": "none",
    }
    assert causal._expected_cognitive_parameters("M2") == {
        "startup_profile": "module2_causal_obstacle_active",
        "module2_mode": "active",
        "active_effect_scope": "obstacle_only",
    }


def test_cognitive_readiness_is_not_applicable_for_m0(tmp_path):
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M0")
    assert causal._wait_for_cognitive_ready(
        manifest, run, (), tmp_path / "unused.sock", timeout_sec=0.0
    ) == {"ready": True, "applicability": "N/A_MODULE2_OFF"}


def test_cognitive_readiness_waits_for_delayed_socket_and_canonical_prior(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from bio_nav_interfaces.msg import CognitiveMapConstraints, PlanningPrior
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import (
        DurabilityPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    context = Context()
    rclpy.init(args=None, context=context)
    node = rclpy.create_node("bio_nav_ros_bridge", context=context)
    for name, value in causal._expected_cognitive_parameters("M1").items():
        node.declare_parameter(name, value)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    diagnostic_publisher = node.create_publisher(DiagnosticArray, "/diagnostics", qos)
    prior_publisher = node.create_publisher(
        PlanningPrior, "/bio_nav/module2/planning_prior", qos
    )
    constraints_qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    constraints_publisher = node.create_publisher(
        CognitiveMapConstraints,
        "/bio_nav/cognitive_map/constraints",
        constraints_qos,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    stop = threading.Event()
    socket_path = tmp_path / "delayed.sock"
    server_holder = []
    started = time.monotonic()

    def publish() -> None:
        while not stop.wait(0.02):
            executor.spin_once(timeout_sec=0.0)
            if time.monotonic() - started < 0.15:
                continue
            if not server_holder:
                server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                server.bind(str(socket_path))
                server.listen(1)
                server_holder.append(server)
            diagnostic = DiagnosticArray()
            status = DiagnosticStatus()
            status.name = "bio_nav_ros_bridge"
            status.level = DiagnosticStatus.OK
            status.message = "ready"
            status.values = [
                KeyValue(key="state", value="RUNNING"),
                KeyValue(key="socket_connected", value="True"),
            ]
            diagnostic.status = [status]
            diagnostic_publisher.publish(diagnostic)
            constraints = CognitiveMapConstraints()
            constraints.map_version = TEST_LIVE_MAP_VERSION
            constraints.cognitive_tile_id = causal.KUJIALE_MAP_ID
            constraints.t_map_canvas = list(causal.KUJIALE_T_MAP_CANVAS)
            mask = [False] * 256
            for state_id in causal.KUJIALE_VALID_STATE_IDS:
                mask[state_id] = True
            constraints.reachable_state_mask = mask
            constraints_publisher.publish(constraints)
            prior = PlanningPrior()
            prior.schema_version = "bio_nav_planning_prior_v310"
            prior.map_version = constraints.map_version
            prior.cognitive_tile_id = causal.KUJIALE_MAP_ID
            prior.t_map_canvas = list(causal.KUJIALE_T_MAP_CANVAS)
            prior.valid_state_mask = mask
            prior.trusted_write = False
            prior_publisher.publish(prior)

    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    manifest = load_manifest(CONFIG)
    run = next(item for item in manifest.runs if item.arm == "M1")
    try:
        result = causal._wait_for_cognitive_ready(
            manifest, run, (), socket_path, timeout_sec=5.0
        )
        assert result["ready"] is True, result
        assert result["socket_listener"] is True
        assert result["cognitive_constraints"]["map_id"] == causal.KUJIALE_MAP_ID
        assert result["cognitive_constraints"]["map_version"] == TEST_LIVE_MAP_VERSION
        assert result["planning_prior"]["valid_state_count"] == 51
        assert time.monotonic() - started >= 0.15
    finally:
        stop.set()
        thread.join(timeout=1.0)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        context.shutdown()
        for server in server_holder:
            server.close()
        socket_path.unlink(missing_ok=True)


def test_startup_probe_private_context_reaches_ready_without_global_executor():
    rclpy = pytest.importorskip("rclpy")
    from rclpy.context import Context
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String

    context = Context()
    rclpy.init(args=None, context=context)
    node = rclpy.create_node(
        f"phase_f_startup_test_publisher_{os.getpid()}", context=context
    )
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    publisher = node.create_publisher(
        String, "/simulation/reset_stop_gate/status", qos
    )
    stop = threading.Event()

    def publish_ready() -> None:
        message = String()
        message.data = json.dumps({
            "generation": 1,
            "held": False,
            "reason": "released:activation_gate",
        })
        while not stop.wait(0.02):
            publisher.publish(message)

    thread = threading.Thread(target=publish_ready, daemon=True)
    thread.start()
    try:
        result = causal._wait_for_startup_ready([], 2.0)
    finally:
        stop.set()
        thread.join(timeout=1.0)
        node.destroy_node()
        context.shutdown()
    assert result == {
        "ready": True,
        "generation": 1,
        "held": False,
        "reason": "released:activation_gate",
    }


def test_startup_probe_private_context_timeout_is_not_executor_type_error():
    pytest.importorskip("rclpy")
    result = causal._wait_for_startup_ready([], 0.05)
    assert result["ready"] is False
    assert result["reason"] == (
        "startup reset generation 1 was not released before timeout"
    )
    assert "TypeError" not in result["reason"]


def test_campaign_missing_post_route_ttl_callbacks_defers_to_active_probe(
    tmp_path, monkeypatch
):
    summary, _ = _run_campaign_with_fake_processes(tmp_path, monkeypatch, clear=False)
    assert summary["runs"][0]["state"] == "EPISODE_FINISHED"
    assert summary["state"] == "FINISHED"


def test_stop_process_cleans_a_nested_child_in_a_new_process_group(tmp_path):
    nested_pid_file = tmp_path / "nested.pid"
    nested_code = (
        "import signal,sys,time; "
        "signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); "
        "signal.signal(signal.SIGTERM, lambda *_: sys.exit(0)); "
        "time.sleep(300)"
    )
    root_code = (
        "import os,pathlib,signal,subprocess,sys,time; "
        "child=subprocess.Popen([sys.executable,'-c',os.environ['NESTED_CODE']],"
        "start_new_session=True); "
        "pathlib.Path(os.environ['NESTED_PID_FILE']).write_text(str(child.pid)); "
        "signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); "
        "signal.signal(signal.SIGTERM, lambda *_: sys.exit(0)); "
        "time.sleep(300)"
    )
    env = os.environ.copy()
    env["NESTED_CODE"] = nested_code
    env["NESTED_PID_FILE"] = str(nested_pid_file)
    stream = (tmp_path / "tree.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        [sys.executable, "-c", root_code],
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=env,
        text=True,
    )
    deadline = time.monotonic() + 5.0
    while not nested_pid_file.exists() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert nested_pid_file.exists()
    nested_pid = int(nested_pid_file.read_text(encoding="utf-8"))
    try:
        result = causal._stop_process(
            causal._ManagedProcess("nested", process, stream), 1.0
        )
        assert result["cleanup_ok"] is True
        assert result["remaining_process_groups"] == []
        assert process.poll() is not None
        assert not Path(f"/proc/{nested_pid}").exists()
    finally:
        for pid in (process.pid, nested_pid):
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        process.wait(timeout=5.0)


def test_stop_process_rescans_late_recorded_group_and_lock(tmp_path):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    root_ready_file = tmp_path / "root.ready"
    ready_file = tmp_path / "late.ready"
    late_pid_file = tmp_path / "late.pid"
    late_code = textwrap.dedent(
        """
        import fcntl
        import os
        from pathlib import Path
        import signal
        import sys
        import time

        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
        stream = Path(os.environ["LATE_LOCK"]).open("a", encoding="utf-8")
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        Path(os.environ["LATE_READY"]).write_text("ready", encoding="utf-8")
        time.sleep(300)
        """
    )
    root_code = textwrap.dedent(
        """
        import os
        from pathlib import Path
        import signal
        import subprocess
        import sys
        import time

        def stop(*_args):
            child = subprocess.Popen(
                [sys.executable, "-c", os.environ["LATE_CODE"]],
                start_new_session=True,
                env=os.environ.copy(),
            )
            pgid = os.getpgid(child.pid)
            run_dir = Path(os.environ["RUN_DIR"])
            temporary = run_dir / ".module3_ros.identity.tmp"
            temporary.write_text(f"{child.pid} {pgid}\\n", encoding="utf-8")
            os.replace(temporary, run_dir / "module3_ros.identity")
            (run_dir / "module3_ros.pid").write_text(str(child.pid), encoding="utf-8")
            (run_dir / "module3_ros.pgid").write_text(str(pgid), encoding="utf-8")
            Path(os.environ["LATE_PID_FILE"]).write_text(str(child.pid), encoding="utf-8")
            raise SystemExit(0)

        signal.signal(signal.SIGINT, stop)
        signal.signal(signal.SIGTERM, stop)
        Path(os.environ["ROOT_READY"]).write_text("ready", encoding="utf-8")
        time.sleep(300)
        """
    )
    env = os.environ.copy()
    env.update({
        "LATE_CODE": late_code,
        "LATE_LOCK": str(runtime_dir / "ros.lock"),
        "LATE_READY": str(ready_file),
        "LATE_PID_FILE": str(late_pid_file),
        "ROOT_READY": str(root_ready_file),
        "RUN_DIR": str(tmp_path),
    })
    stream = (tmp_path / "late-tree.log").open("w", encoding="utf-8")
    process = subprocess.Popen(
        [sys.executable, "-c", root_code],
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=env,
        text=True,
    )
    try:
        deadline = time.monotonic() + 2.0
        while not root_ready_file.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert root_ready_file.exists()
        result = causal._stop_process(
            causal._ManagedProcess("stack", process, stream, tmp_path), 1.5
        )
        assert late_pid_file.exists()
        late_pid = int(late_pid_file.read_text(encoding="utf-8"))
        assert ready_file.exists()
        assert result["cleanup_ok"] is True
        assert result["remaining_process_groups"] == []
        assert not Path(f"/proc/{late_pid}").exists()
        cleanup = causal._confirm_arm_cleanup(
            tmp_path,
            tmp_path / "module2.sock",
            (result,),
            {"ISAAC_NAV_RUNTIME_DIR": str(runtime_dir)},
            timeout_sec=1.0,
            quiet_sec=0.1,
            poll_sec=0.02,
        )
        assert cleanup["ok"] is True
        assert cleanup["locks_free"] == {"ros": True, "isaac": True}
        assert cleanup["stale_runtime_files"] == []
    finally:
        if process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        process.wait(timeout=5.0)


def test_cleanup_quiet_window_resets_for_late_recorded_lock_holder(tmp_path):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    child_holder = []
    ready = threading.Event()
    code = textwrap.dedent(
        """
        import fcntl
        import os
        from pathlib import Path
        import signal
        import sys
        import time

        signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
        signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
        stream = Path(os.environ["LATE_LOCK"]).open("a", encoding="utf-8")
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        Path(os.environ["LATE_READY"]).write_text("ready", encoding="utf-8")
        time.sleep(300)
        """
    )
    ready_path = tmp_path / "quiet-late.ready"

    def launch_late() -> None:
        time.sleep(0.06)
        env = os.environ.copy()
        env.update({
            "LATE_LOCK": str(runtime_dir / "ros.lock"),
            "LATE_READY": str(ready_path),
        })
        child = subprocess.Popen(
            [sys.executable, "-c", code],
            start_new_session=True,
            env=env,
        )
        child_holder.append(child)
        deadline = time.monotonic() + 2.0
        while not ready_path.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        pgid = os.getpgid(child.pid)
        temporary = tmp_path / ".module3_ros.identity.tmp"
        temporary.write_text(f"{child.pid} {pgid}\n", encoding="utf-8")
        os.replace(temporary, tmp_path / "module3_ros.identity")
        ready.set()

    launcher = threading.Thread(target=launch_late, daemon=True)
    launcher.start()
    started = time.monotonic()
    try:
        result = causal._confirm_arm_cleanup(
            tmp_path,
            tmp_path / "module2.sock",
            ({"name": "stack", "returncode": 0, "cleanup_ok": True},),
            {"ISAAC_NAV_RUNTIME_DIR": str(runtime_dir)},
            timeout_sec=2.0,
            quiet_sec=0.25,
            poll_sec=0.02,
        )
        launcher.join(timeout=2.0)
        assert ready.is_set()
        assert result["ok"] is True
        assert result["attempts"] > 5
        assert time.monotonic() - started >= 0.25
        assert causal._lock_is_free(runtime_dir / "ros.lock")
        assert child_holder and child_holder[0].poll() is not None
    finally:
        for child in child_holder:
            if child.poll() is None:
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            child.wait(timeout=5.0)


def test_cleanup_timeout_fails_closed_for_unknown_lock_holder(tmp_path):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    lock = (runtime_dir / "ros.lock").open("a", encoding="utf-8")
    fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    try:
        result = causal._confirm_arm_cleanup(
            tmp_path,
            tmp_path / "module2.sock",
            ({"name": "stack", "returncode": 0, "cleanup_ok": True},),
            {"ISAAC_NAV_RUNTIME_DIR": str(runtime_dir)},
            timeout_sec=0.15,
            quiet_sec=0.05,
            poll_sec=0.02,
        )
    finally:
        fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        lock.close()
    assert result["ok"] is False
    assert result["locks_free"]["ros"] is False


def test_cleanup_removes_stale_exact_socket_and_next_arm_sees_none(tmp_path):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    socket_path = tmp_path / "module2.sock"
    stale = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    stale.bind(str(socket_path))
    stale.close()
    assert socket_path.exists()
    first = causal._confirm_arm_cleanup(
        tmp_path,
        socket_path,
        ({"name": "stack", "returncode": 0, "cleanup_ok": True},),
        {"ISAAC_NAV_RUNTIME_DIR": str(runtime_dir)},
        timeout_sec=0.5,
        quiet_sec=0.05,
        poll_sec=0.01,
    )
    assert first["ok"] is True
    assert first["module2_socket_absent"] is True
    assert not socket_path.exists()
    second = causal._confirm_arm_cleanup(
        tmp_path,
        socket_path,
        ({"name": "stack", "returncode": 0, "cleanup_ok": True},),
        {"ISAAC_NAV_RUNTIME_DIR": str(runtime_dir)},
        timeout_sec=0.5,
        quiet_sec=0.05,
        poll_sec=0.01,
    )
    assert second["ok"] is True
    assert not socket_path.exists()


def test_cleanup_never_unlinks_active_exact_socket_listener(tmp_path):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    socket_path = tmp_path / "module2.sock"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(socket_path))
    server.listen(1)
    try:
        assert causal._unix_socket_listener_active(socket_path) is True
        result = causal._confirm_arm_cleanup(
            tmp_path,
            socket_path,
            ({"name": "stack", "returncode": 0, "cleanup_ok": True},),
            {"ISAAC_NAV_RUNTIME_DIR": str(runtime_dir)},
            timeout_sec=0.15,
            quiet_sec=0.05,
            poll_sec=0.01,
        )
        assert result["ok"] is False
        assert result["module2_socket_listener_active"] is True
        assert socket_path.exists()
    finally:
        server.close()
        socket_path.unlink(missing_ok=True)


def _phase_f_identity(run_dir: Path, name: str) -> tuple[int, int]:
    values = (run_dir / f"{name}.identity").read_text(
        encoding="utf-8").split()
    assert len(values) == 2
    return int(values[0]), int(values[1])


def _phase_f_pid_is_running(pid: int) -> bool:
    stat = Path(f"/proc/{pid}/stat")
    return stat.is_file() and stat.read_text(encoding="utf-8").split()[2] != "Z"


def _start_fake_phase_f_stack(
    tmp_path: Path,
    *,
    arm: str = "M3",
    scene: str = "kujiale",
    condition: str = "static",
    producer_ignores_term: bool = False,
    localization_supervisor_mode: str | None = None,
    enable_route_prior: bool = False,
    route_prior_snapshot: Path | None = None,
) -> SimpleNamespace:
    root = PACKAGE.parents[2]
    project = tmp_path / "project"
    scripts = project / "scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copy2(root / "scripts/run_v6_low_obstacle_phase_f_stack.sh", scripts)
    shutil.copy2(root / "scripts/lib/v6_dynamic_startup.sh", scripts / "lib")
    (scripts / "lib/common.sh").write_text(
        """#!/usr/bin/env bash
export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
require_directory() { [[ -d "$1" ]]; }
require_file() { [[ -f "$1" ]]; }
source_ros() { :; }
validate_v6_dynamic_integration_overlay() { :; }
""",
        encoding="utf-8",
    )
    heartbeat = tmp_path / "module3.heartbeat"
    module3_argv = tmp_path / "module3.argv"
    fake_module3 = """#!/usr/bin/env bash
printf '%s\n' "$@" >"${FAKE_MODULE3_ARGV}"
mkdir -p "${ISAAC_NAV_RUNTIME_DIR}"
exec 9>"${ISAAC_NAV_RUNTIME_DIR}/ros.lock"
flock -n 9
log_signal() { printf '%s\n' "module3:$1" >>"${FAKE_SIGNAL_LOG}"; }
trap 'log_signal INT; exit 0' INT
trap 'log_signal TERM; exit 0' TERM
trap 'log_signal HUP; exit 0' HUP
while :; do
  printf '%s\n' "$(date +%s%N)" >>"${FAKE_MODULE3_HEARTBEAT}"
  sleep 0.02
done
"""
    (scripts / "run_v6_kujiale_low_obstacles.sh").write_text(
        fake_module3,
        encoding="utf-8",
    )
    (scripts / "run_v6_rivermark.sh").write_text(
        fake_module3,
        encoding="utf-8",
    )
    integration = tmp_path / "integration"
    integration_scripts = integration / "scripts"
    integration_scripts.mkdir(parents=True)
    module2_argv = tmp_path / "module2.argv"
    bridge_argv = tmp_path / "bridge.argv"
    socket_server = tmp_path / "socket_server.py"
    socket_server.write_text(
        """import os
import signal
import socket
import sys
import time

def record(name):
    with open(os.environ["FAKE_SIGNAL_LOG"], "a", encoding="utf-8") as stream:
        stream.write(f"module2:{name}\\n")

def stop(signum, _frame):
    name = signal.Signals(signum).name.removeprefix("SIG")
    record(name)
    if not (name == "TERM" and os.environ.get("FAKE_PRODUCER_IGNORE_TERM") == "1"):
        raise SystemExit(0)

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGHUP, stop)
server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
server.bind(sys.argv[1])
server.listen(1)
while True:
    time.sleep(1.0)
""",
        encoding="utf-8",
    )
    module2_launcher = """#!/usr/bin/env bash
printf '%s\n' "$@" >"${FAKE_MODULE2_ARGV}"
socket_path=""
while (($#)); do
  case "$1" in
    --socket) socket_path="$2"; shift 2 ;;
    *) shift ;;
  esac
done
exec python3 "${FAKE_SOCKET_SERVER}" "${socket_path}"
"""
    for name in (
        "run_module2_v310_server.sh",
        "run_v6_module2_causal_obstacle_server.sh",
    ):
        path = integration_scripts / name
        path.write_text(module2_launcher, encoding="utf-8")
        path.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    (fake_bin / "ros2").write_text(
        """#!/usr/bin/env bash
printf '%s\n' "$@" >"${FAKE_BRIDGE_ARGV}"
log_signal() { printf '%s\n' "bridge:$1" >>"${FAKE_SIGNAL_LOG}"; }
trap 'log_signal INT; exit 0' INT
if [[ "${FAKE_PRODUCER_IGNORE_TERM:-0}" == 1 ]]; then
  trap 'log_signal TERM' TERM
else
  trap 'log_signal TERM; exit 0' TERM
fi
trap 'log_signal HUP; exit 0' HUP
while :; do sleep 0.05; done
""",
        encoding="utf-8",
    )
    # Keep each just-launched wrapper in the stack PGID long enough to make the
    # historical first-ps race deterministic, then create the requested session.
    (fake_bin / "setsid").write_text(
        """#!/usr/bin/env bash
sleep "${FAKE_SETSID_DELAY_SEC}"
/usr/bin/setsid "$@" &
wait "$!"
""",
        encoding="utf-8",
    )
    for path in (
        scripts / "run_v6_low_obstacle_phase_f_stack.sh",
        scripts / "run_v6_kujiale_low_obstacles.sh",
        scripts / "run_v6_rivermark.sh",
        fake_bin / "ros2",
        fake_bin / "setsid",
    ):
        path.chmod(0o755)
    module2 = tmp_path / "module2"
    constraints = (
        module2 / "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    )
    constraints.parent.mkdir(parents=True)
    constraints.touch()
    (module2 / "configs/module2_pdf_v310_module3.yaml").touch()
    module2_asset_root = tmp_path / "module2-assets"
    checkpoint = (
        module2_asset_root / "weights/module2_srdr_v310_seed20260822.pt"
    )
    checkpoint.parent.mkdir(parents=True)
    checkpoint.touch()
    route_prior_catalog = tmp_path / "route-prior-catalog"
    route_prior_catalog.mkdir()
    (route_prior_catalog / "catalog.json").write_text(
        "{}\n", encoding="utf-8")
    run_dir = tmp_path / "run"
    socket_path = tmp_path / "socket/module2.sock"
    runtime_dir = tmp_path / "runtime"
    signal_log = tmp_path / "signals.log"
    candidate_manifest = tmp_path / "run4-candidate.json"
    candidate_manifest.touch()
    if route_prior_snapshot is None:
        route_prior_snapshot = tmp_path / "route-prior-snapshot"
    route_prior_snapshot.mkdir(parents=True, exist_ok=True)
    (route_prior_snapshot / "manifest.json").write_text("{}\n", encoding="utf-8")
    env = os.environ.copy()
    env.update({
        "PATH": f"{fake_bin}:{env['PATH']}",
        "BIO_NAV_INTEGRATION_ROOT": str(integration),
        "BIO_NAV_MODULE2_V310_ROOT": str(module2),
        "FAKE_MODULE3_HEARTBEAT": str(heartbeat),
        "FAKE_MODULE3_ARGV": str(module3_argv),
        "FAKE_MODULE2_ARGV": str(module2_argv),
        "FAKE_BRIDGE_ARGV": str(bridge_argv),
        "FAKE_SIGNAL_LOG": str(signal_log),
        "FAKE_PRODUCER_IGNORE_TERM": "1" if producer_ignores_term else "0",
        "FAKE_SOCKET_SERVER": str(socket_server),
        "FAKE_SETSID_DELAY_SEC": "0.08",
        "ISAAC_NAV_RUNTIME_DIR": str(runtime_dir),
        "BIO_NAV_PHASE_F_CLEANUP_INT_CHECKS": "20",
        "BIO_NAV_PHASE_F_CLEANUP_TERM_CHECKS": "20",
        "BIO_NAV_PHASE_F_CLEANUP_QUIET_CHECKS": "2",
    })
    command = [
        str(scripts / "run_v6_low_obstacle_phase_f_stack.sh"),
        arm, "--domain", "150", "--run-dir", str(run_dir),
        "--socket", str(socket_path),
        "--module2-asset-root", str(module2_asset_root),
    ]
    if scene == "rivermark":
        command.extend([
            "--scene", scene,
            "--condition", condition,
            "--route-prior-catalog-root", str(route_prior_catalog),
        ])
    if localization_supervisor_mode is not None:
        command.extend(
            [
                "--localization-supervisor-mode",
                localization_supervisor_mode,
                "--candidate-manifest",
                str(candidate_manifest),
            ]
        )
    if enable_route_prior:
        command.extend([
            "--enable-route-prior", "--route-prior-snapshot",
            str(route_prior_snapshot),
        ])
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )
    deadline = time.monotonic() + 8.0
    identities = ("stack", "module3_ros", "module2_server", "integration_bridge")
    while time.monotonic() < deadline:
        if process.poll() is not None:
            break
        if (
            all((run_dir / f"{name}.identity").is_file() for name in identities)
            and socket_path.exists()
            and heartbeat.is_file()
            and len(heartbeat.read_text(encoding="utf-8").splitlines()) >= 3
        ):
            break
        time.sleep(0.02)
    assert process.poll() is None, process.stdout.read() if process.stdout else ""
    assert all((run_dir / f"{name}.identity").is_file() for name in identities)
    assert socket_path.exists()
    return SimpleNamespace(
        process=process,
        script=scripts / "run_v6_low_obstacle_phase_f_stack.sh",
        run_dir=run_dir,
        socket=socket_path,
        runtime_dir=runtime_dir,
        heartbeat=heartbeat,
        module3_argv=module3_argv,
        module2_argv=module2_argv,
        bridge_argv=bridge_argv,
        candidate_manifest=candidate_manifest,
        module2_root=module2,
        module2_asset_root=module2_asset_root,
        checkpoint=checkpoint,
        route_prior_catalog=route_prior_catalog,
        route_prior_snapshot=route_prior_snapshot,
        signal_log=signal_log,
        env=env,
    )


@pytest.mark.parametrize("arm", ["M1", "M2", "M3"])
def test_phase_f_stack_explicitly_disables_route_prior_for_module2_arms(
    tmp_path, arm
):
    fake = _start_fake_phase_f_stack(tmp_path, arm=arm)
    try:
        assert fake.module3_argv.read_text(encoding="utf-8").splitlines() == [
            "ros", arm, "route_prior_enabled:=false"
        ]
        module2 = fake.module2_argv.read_text(encoding="utf-8").splitlines()
        assert module2[module2.index("--shadow-config") + 1] == (
            "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
        )
        assert "--candidate-manifest" not in module2
        bridge = fake.bridge_argv.read_text(encoding="utf-8").splitlines()
        asset_arg = f"module2_asset_root:={tmp_path / 'module2-assets'}"
        assert bridge.count(asset_arg) == 1
    finally:
        _stop_fake_phase_f_stack(fake)


def test_phase_f_rivermark_preflight_uses_separate_checkpoint_asset_root(
    tmp_path,
):
    fake = _start_fake_phase_f_stack(tmp_path, scene="rivermark")
    try:
        assert (fake.module2_root / "configs/module2_pdf_v310_module3.yaml").is_file()
        assert fake.checkpoint.is_file()
        assert not (fake.module2_root / "weights").exists()
        module2 = fake.module2_argv.read_text(encoding="utf-8").splitlines()
        assert module2[module2.index("--module2-root") + 1] == str(
            fake.module2_root.resolve())
        assert module2[module2.index("--module2-asset-root") + 1] == str(
            fake.module2_asset_root.resolve())
        assert module2[module2.index("--config") + 1] == (
            "configs/module2_pdf_v310_module3.yaml")
        assert module2[module2.index("--checkpoint") + 1] == (
            "weights/module2_srdr_v310_seed20260822.pt")
        assert module2[module2.index("--startup-profile") + 1] == (
            "module2_causal_obstacle_outdoor")
        bridge = fake.bridge_argv.read_text(encoding="utf-8").splitlines()
        assert "startup_profile:=module2_causal_obstacle_outdoor" in bridge
        assert fake.module3_argv.read_text(encoding="utf-8").splitlines() == [
            "ros", "static"]
    finally:
        _stop_fake_phase_f_stack(fake)

    fake.checkpoint.unlink()
    fake.module3_argv.unlink()
    failed = subprocess.run(
        [
            str(fake.script), "M3",
            "--scene", "rivermark",
            "--condition", "static",
            "--domain", "150",
            "--run-dir", str(tmp_path / "missing-run"),
            "--socket", str(tmp_path / "missing/module2.sock"),
            "--module2-asset-root", str(fake.module2_asset_root),
            "--route-prior-catalog-root", str(fake.route_prior_catalog),
        ],
        env=fake.env,
        capture_output=True,
        text=True,
        check=False,
        timeout=5.0,
    )
    assert failed.returncode != 0
    assert not fake.module3_argv.exists()
    source = fake.script.read_text(encoding="utf-8")
    assert (
        'require_file "${module2_asset_root}/weights/'
        'module2_srdr_v310_seed20260822.pt"'
    ) in source
    assert (
        'require_file "${module2_root}/weights/'
        'module2_srdr_v310_seed20260822.pt"'
    ) not in source


def test_final_route_prior_pilot_dry_run_defaults_off(tmp_path):
    root = PACKAGE.parents[2]
    result = subprocess.run(
        [
            str(root / "scripts/run_v6_low_obstacle_phase_f_stack.sh"),
            "M3",
            "--run-dir", str(tmp_path / "run"),
            "--socket", str(tmp_path / "module2.sock"),
            "--module2-asset-root", str(tmp_path / "module2-assets"),
            "--dry-run",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "graph_mode=gvg" in result.stdout
    assert "route_prior_enabled=false" in result.stdout
    assert "route_prior_snapshot_path" not in result.stdout
    assert "active_effect_scope=obstacle_only" in result.stdout
    assert "cognitive_graph_mode:=gvg route_prior_enabled:=false" in result.stdout
    assert "trajectory_topic:=/ground_truth/odom" in result.stdout
    assert (
        "module2_assets: --shadow-config "
        "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
        in result.stdout
    )
    assert "--candidate-manifest" not in result.stdout
    assert "module2_response_timeout_s" not in result.stdout
    assert "goal_prior_retry_window_s" not in result.stdout


@pytest.mark.parametrize("condition", ("static", "dynamic", "appearance"))
def test_phase_f_rivermark_m3_dry_run_uses_catalog_and_generic_assets(
    tmp_path, condition
):
    root = PACKAGE.parents[2]
    catalog = tmp_path / "rivermark catalog"
    catalog.mkdir()
    (catalog / "catalog.json").write_text("{}\n", encoding="utf-8")
    result = subprocess.run(
        [
            str(root / "scripts/run_v6_low_obstacle_phase_f_stack.sh"),
            "M3",
            "--scene", "rivermark",
            "--condition", condition,
            "--run-dir", str(tmp_path / "run"),
            "--socket", str(tmp_path / "module2.sock"),
            "--module2-asset-root", str(tmp_path / "module2-assets"),
            "--route-prior-catalog-root", str(catalog),
            "--dry-run",
        ],
        capture_output=True,
        text=True,
        check=True,
    )

    assert "scene=rivermark" in result.stdout
    assert f"condition={condition}" in result.stdout
    assert "route_prior_enabled=true" in result.stdout
    assert f"route_prior_snapshot_catalog_root={catalog}" in result.stdout
    assert "route_prior_snapshot_path" not in result.stdout
    assert (
        "module2_assets: --config configs/module2_pdf_v310_module3.yaml "
        "--checkpoint weights/module2_srdr_v310_seed20260822.pt"
        in result.stdout
    )
    assert "--shadow-config" not in result.stdout
    assert "--candidate-manifest" not in result.stdout
    lines = result.stdout.splitlines()
    module3 = next(line for line in lines if line.startswith("module3:"))
    bridge = next(line for line in lines if line.startswith("bridge:"))
    escaped_catalog = str(catalog).replace(" ", "\\ ")
    assert "run_v6_rivermark.sh ros" in module3
    assert module3.endswith(condition)
    assert f"route_prior_snapshot_catalog_root:={escaped_catalog}" in bridge
    assert "outdoor_context_switch_enabled:=true" in bridge
    assert "startup_profile:=module2_causal_obstacle_outdoor" in bridge


def test_phase_f_rivermark_rejects_non_m3_invalid_condition_and_snapshot(
    tmp_path,
):
    stack = PACKAGE.parents[2] / "scripts/run_v6_low_obstacle_phase_f_stack.sh"
    catalog = tmp_path / "catalog"
    catalog.mkdir()
    (catalog / "catalog.json").write_text("{}\n", encoding="utf-8")
    common = [
        "--scene", "rivermark",
        "--run-dir", str(tmp_path / "run"),
        "--socket", str(tmp_path / "module2.sock"),
        "--module2-asset-root", str(tmp_path / "module2-assets"),
        "--route-prior-catalog-root", str(catalog),
        "--dry-run",
    ]
    non_m3 = subprocess.run(
        [str(stack), "M2", *common], capture_output=True, text=True)
    assert non_m3.returncode == 2
    assert "Rivermark requires the M3 obstacle arm" in non_m3.stderr

    invalid = subprocess.run(
        [str(stack), "M3", *common, "--condition", "rain"],
        capture_output=True,
        text=True,
    )
    assert invalid.returncode == 2
    assert "condition must be static, dynamic, or appearance" in invalid.stderr

    snapshot = tmp_path / "indoor"
    snapshot.mkdir()
    conflict = subprocess.run(
        [str(stack), "M3", *common, "--route-prior-snapshot", str(snapshot)],
        capture_output=True,
        text=True,
    )
    assert conflict.returncode == 2
    assert "mutually exclusive" in conflict.stderr


def test_phase_f_asset_root_required_only_for_server_arms(tmp_path):
    stack = PACKAGE.parents[2] / "scripts/run_v6_low_obstacle_phase_f_stack.sh"
    common = [
        "--run-dir", str(tmp_path / "run"),
        "--socket", str(tmp_path / "module2.sock"),
        "--dry-run",
    ]
    missing = subprocess.run(
        [str(stack), "M1", *common],
        capture_output=True,
        text=True,
        check=False,
    )
    assert missing.returncode == 2
    assert "--module2-asset-root is required for M1" in missing.stderr

    baseline = subprocess.run(
        [str(stack), "M0", *common],
        capture_output=True,
        text=True,
        check=True,
    )
    assert "module2_asset_root" not in baseline.stdout
    assert "route_prior_snapshot_path" not in baseline.stdout


def test_final_route_prior_pilot_forwards_gvg_prior_to_m3_and_bridge(tmp_path):
    snapshot = tmp_path / "route prior snapshot with spaces"
    fake = _start_fake_phase_f_stack(
        tmp_path,
        arm="M3",
        enable_route_prior=True,
        route_prior_snapshot=snapshot,
    )
    try:
        assert fake.module3_argv.read_text(encoding="utf-8").splitlines() == [
            "ros",
            "M3",
            "route_prior_enabled:=true",
            f"route_prior_snapshot_path:={snapshot}",
        ]
        module2 = fake.module2_argv.read_text(encoding="utf-8").splitlines()
        assert not any("route_prior_snapshot_path" in arg for arg in module2)
        assert module2.count("--module2-asset-root") == 1
        assert module2[module2.index("--module2-asset-root") + 1] == str(
            tmp_path / "module2-assets"
        )
        assert module2[:4] == [
            "--startup-profile",
            "module2_causal_obstacle_active",
            "--active-effect-scope",
            "obstacle_only",
        ]
        bridge = fake.bridge_argv.read_text(encoding="utf-8").splitlines()
        asset_arg = f"module2_asset_root:={tmp_path / 'module2-assets'}"
        assert bridge.count(asset_arg) == 1
        assert "startup_profile:=module2_causal_obstacle_active" in bridge
        assert "cognitive_graph_mode:=gvg" in bridge
        assert "route_prior_enabled:=true" in bridge
        assert f"route_prior_snapshot_path:={fake.route_prior_snapshot}" in bridge
        assert "trajectory_topic:=/ground_truth/odom" in bridge
    finally:
        _stop_fake_phase_f_stack(fake)


@pytest.mark.parametrize("arm", ["M0", "M1", "M2"])
def test_final_route_prior_pilot_rejects_non_m3_arms(tmp_path, arm):
    root = PACKAGE.parents[2]
    result = subprocess.run(
        [
            str(root / "scripts/run_v6_low_obstacle_phase_f_stack.sh"),
            arm,
            "--run-dir", str(tmp_path / "run"),
            "--socket", str(tmp_path / "module2.sock"),
            "--enable-route-prior",
            "--dry-run",
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert "requires the M3 obstacle arm" in result.stderr


def test_final_route_prior_pilot_rejects_recovery_and_has_explicit_dry_run(
    tmp_path,
):
    root = PACKAGE.parents[2]
    stack = root / "scripts/run_v6_low_obstacle_phase_f_stack.sh"
    common = [
        str(stack), "M3",
        "--run-dir", str(tmp_path / "run"),
        "--socket", str(tmp_path / "module2.sock"),
        "--module2-asset-root", str(tmp_path / "module2-assets"),
        "--enable-route-prior",
    ]
    recovery = subprocess.run(
        [
            *common,
            "--localization-supervisor-mode", "active",
            "--candidate-manifest", str(tmp_path / "candidate.json"),
            "--dry-run",
        ],
        capture_output=True,
        text=True,
    )
    assert recovery.returncode == 2
    assert "incompatible with W0/W1 localization recovery" in recovery.stderr

    snapshot = tmp_path / "route prior snapshot with spaces"
    snapshot.mkdir()
    (snapshot / "manifest.json").write_text("{}\n", encoding="utf-8")
    dry_run = subprocess.run([
        *common, "--route-prior-snapshot", str(snapshot), "--dry-run"
    ], capture_output=True, text=True, check=True)
    assert "graph_mode=gvg" in dry_run.stdout
    assert "route_prior_enabled=true" in dry_run.stdout
    assert "cognitive_profile=M3" in dry_run.stdout
    assert "active_effect_scope=obstacle_only" in dry_run.stdout
    assert "cpg_navigation_writes=false" in dry_run.stdout
    assert "cognitive_graph_mode:=gvg route_prior_enabled:=true" in dry_run.stdout
    assert f"route_prior_snapshot_path={snapshot}" in dry_run.stdout
    assert "route_prior_semantics=frozen_snapshot_main_compatible" in dry_run.stdout
    lines = dry_run.stdout.splitlines()
    module3_line = next(line for line in lines if line.startswith("module3:"))
    bridge_line = next(line for line in lines if line.startswith("bridge:"))
    module2_line = next(line for line in lines if line.startswith("module2_assets:"))
    escaped_snapshot = str(snapshot).replace(" ", "\\ ")
    assert module3_line.count(
        f"route_prior_snapshot_path:={escaped_snapshot}"
    ) == 1
    assert bridge_line.count(
        f"route_prior_snapshot_path:={escaped_snapshot}"
    ) == 1
    assert "route_prior_snapshot_path" not in module2_line
    assert "module2_response_timeout_s" not in dry_run.stdout
    assert "goal_prior_retry_window_s" not in dry_run.stdout


def test_final_route_prior_pilot_requires_snapshot_path(tmp_path):
    root = PACKAGE.parents[2]
    result = subprocess.run([
        str(root / "scripts/run_v6_low_obstacle_phase_f_stack.sh"), "M3",
        "--run-dir", str(tmp_path / "run"),
        "--socket", str(tmp_path / "module2.sock"),
        "--module2-asset-root", str(tmp_path / "module2-assets"),
        "--enable-route-prior", "--dry-run",
    ], capture_output=True, text=True)
    assert result.returncode == 2
    assert "--route-prior-snapshot is required" in result.stderr


@pytest.mark.parametrize(
    ("mode", "expected_mode"), (("shadow", "shadow"), ("active", "active"))
)
def test_phase_f_stack_localization_extension_forwards_exact_onebox_m3_argv(
    tmp_path, mode, expected_mode
):
    fake = _start_fake_phase_f_stack(
        tmp_path, arm="M3", localization_supervisor_mode=mode
    )
    try:
        assert fake.module3_argv.read_text(encoding="utf-8").splitlines() == [
            "ros",
            "M3",
            "route_prior_enabled:=false",
            "initial_pose_source:=rviz",
            "activation_startup_policy:=wait_for_seed",
        ]
        module2 = fake.module2_argv.read_text(encoding="utf-8").splitlines()
        assert module2[:4] == [
            "--startup-profile",
            "module2_causal_obstacle_active",
            "--active-effect-scope",
            "obstacle_only",
        ]
        assert module2[module2.index("--candidate-manifest") + 1] == str(
            fake.candidate_manifest
        )
        assert "--shadow-config" not in module2
        bridge = fake.bridge_argv.read_text(encoding="utf-8").splitlines()
        assert "startup_profile:=module2_causal_obstacle_active" in bridge
        assert f"localization_supervisor_mode:={expected_mode}" in bridge
        assert f"localization_candidate_manifest:={fake.candidate_manifest}" in bridge
    finally:
        _stop_fake_phase_f_stack(fake)


def test_phase_f_stack_writes_live_deterministic_condition_contract(tmp_path):
    fake = _start_fake_phase_f_stack(tmp_path, arm="M3")
    contract_path = fake.run_dir / "stack.contract.json"
    try:
        payload = v6_formal.validate_condition_stack_contract(
            contract_path, expected_condition_id="indoor_static"
        )
        assert set(payload) == v6_formal.STACK_CONTRACT_KEYS
        assert payload["scene"] == "indoor"
        assert payload["condition"] == "static"
        assert payload["arm"] == "M3"
        assert payload["domain"] == 150
        assert payload["startup_profile"] == "module2_causal_obstacle_active"
        assert payload["pid"] == fake.process.pid
        assert payload["pgid"] == fake.process.pid
        assert payload["stack_session_id"] == v6_formal._stack_session_id(payload)
    finally:
        _stop_fake_phase_f_stack(fake)
    assert not contract_path.exists()


def _stop_fake_phase_f_stack(fake: SimpleNamespace) -> None:
    if fake.process.poll() is None:
        os.killpg(fake.process.pid, signal.SIGINT)
        fake.process.wait(timeout=10.0)


def test_phase_f_producer_stop_survives_nested_setsid_registration_race(tmp_path):
    fake = _start_fake_phase_f_stack(tmp_path)
    identities = {
        name: _phase_f_identity(fake.run_dir, name)
        for name in ("stack", "module3_ros", "module2_server", "integration_bridge")
    }
    try:
        stack_pgid = identities["stack"][1]
        assert stack_pgid == fake.process.pid
        assert len({pgid for _pid, pgid in identities.values()}) == 4
        for name in ("module3_ros", "module2_server", "integration_bridge"):
            pid, pgid = identities[name]
            assert pid == pgid
            assert pgid != stack_pgid

        before = len(fake.heartbeat.read_text(encoding="utf-8").splitlines())
        stop_started = time.monotonic()
        stopped = subprocess.run(
            [
                str(fake.script), "stop-producer", "--run-dir",
                str(fake.run_dir), "--socket", str(fake.socket),
            ],
            env=fake.env,
            text=True,
            capture_output=True,
            timeout=10.0,
            check=False,
        )
        stop_elapsed = time.monotonic() - stop_started
        assert stopped.returncode == 0, stopped.stderr
        assert stop_elapsed < 0.5
        # The raw TTL is 0.5 s and the configured observation margin is 1.0 s.
        # The ROS/Nav2 consumer must remain live across that complete window.
        time.sleep(1.55)
        after = len(fake.heartbeat.read_text(encoding="utf-8").splitlines())
        assert after > before
        assert fake.process.poll() is None
        assert _phase_f_pid_is_running(identities["module3_ros"][0])
        assert not _phase_f_pid_is_running(identities["module2_server"][0])
        assert not _phase_f_pid_is_running(identities["integration_bridge"][0])
        assert not fake.socket.exists()
        assert (fake.run_dir / "stack.identity").is_file()
        assert (fake.run_dir / "module3_ros.identity").is_file()
        assert not (fake.run_dir / "module2_server.identity").exists()
        assert not (fake.run_dir / "integration_bridge.identity").exists()
        with (fake.runtime_dir / "ros.lock").open("a+", encoding="utf-8") as lock:
            with pytest.raises(BlockingIOError):
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    finally:
        _stop_fake_phase_f_stack(fake)
    assert not list(fake.run_dir.glob("*.identity"))
    assert not list(fake.run_dir.glob("*.pid"))
    assert not list(fake.run_dir.glob("*.pgid"))
    assert causal._lock_is_free(fake.runtime_dir / "ros.lock")


def test_phase_f_fast_producer_stop_kills_term_resistant_groups(tmp_path):
    fake = _start_fake_phase_f_stack(tmp_path, producer_ignores_term=True)
    identities = {
        name: _phase_f_identity(fake.run_dir, name)
        for name in ("module2_server", "integration_bridge")
    }
    try:
        started = time.monotonic()
        stopped = subprocess.run(
            [
                str(fake.script), "stop-producer", "--run-dir",
                str(fake.run_dir), "--socket", str(fake.socket),
            ],
            env=fake.env,
            text=True,
            capture_output=True,
            timeout=2.0,
            check=False,
        )
        elapsed = time.monotonic() - started
        assert stopped.returncode == 0, stopped.stderr
        assert elapsed < 0.5
        signals = fake.signal_log.read_text(encoding="utf-8").splitlines()
        assert "module2:TERM" in signals
        assert "bridge:TERM" in signals
        assert "module2:INT" not in signals
        assert "bridge:INT" not in signals
        assert all(
            not _phase_f_pid_is_running(pid) for pid, _pgid in identities.values()
        )
        assert not fake.socket.exists()
        assert fake.process.poll() is None
    finally:
        _stop_fake_phase_f_stack(fake)


def test_phase_f_producer_stop_rejects_consumer_process_group(tmp_path):
    fake = _start_fake_phase_f_stack(tmp_path)
    original = {
        suffix: (fake.run_dir / f"module2_server.{suffix}").read_text(
            encoding="utf-8")
        for suffix in ("identity", "pid", "pgid")
    }
    identities = {
        name: _phase_f_identity(fake.run_dir, name)
        for name in ("stack", "module2_server", "integration_bridge")
    }
    try:
        stack_pid, stack_pgid = identities["stack"]
        (fake.run_dir / "module2_server.identity").write_text(
            f"{stack_pid} {stack_pgid}\n", encoding="utf-8")
        (fake.run_dir / "module2_server.pid").write_text(
            f"{stack_pid}\n", encoding="utf-8")
        (fake.run_dir / "module2_server.pgid").write_text(
            f"{stack_pgid}\n", encoding="utf-8")
        result = subprocess.run(
            [
                str(fake.script), "stop-producer", "--run-dir",
                str(fake.run_dir), "--socket", str(fake.socket),
            ],
            env=fake.env,
            text=True,
            capture_output=True,
            timeout=5.0,
            check=False,
        )
        assert result.returncode != 0
        assert "process groups are not isolated" in result.stderr
        assert fake.process.poll() is None
        assert _phase_f_pid_is_running(identities["module2_server"][0])
        assert _phase_f_pid_is_running(identities["integration_bridge"][0])
        assert fake.socket.exists()
    finally:
        for suffix, content in original.items():
            (fake.run_dir / f"module2_server.{suffix}").write_text(
                content, encoding="utf-8")
        _stop_fake_phase_f_stack(fake)


def test_phase_f_normal_trap_shutdown_cleans_all_registered_groups(tmp_path):
    fake = _start_fake_phase_f_stack(tmp_path)
    identities = {
        name: _phase_f_identity(fake.run_dir, name)
        for name in ("module3_ros", "module2_server", "integration_bridge")
    }
    _stop_fake_phase_f_stack(fake)
    signals = fake.signal_log.read_text(encoding="utf-8").splitlines()
    assert "module2:INT" in signals
    assert all(not _phase_f_pid_is_running(pid) for pid, _pgid in identities.values())
    assert not list(fake.run_dir.glob("*.identity"))
    assert not list(fake.run_dir.glob("*.pid"))
    assert not list(fake.run_dir.glob("*.pgid"))
    assert not fake.socket.exists()
    assert causal._lock_is_free(fake.runtime_dir / "ros.lock")


def test_phase_f_stack_trap_cleans_its_nested_new_process_group(tmp_path):
    root = PACKAGE.parents[2]
    project = tmp_path / "project"
    scripts = project / "scripts"
    (scripts / "lib").mkdir(parents=True)
    shutil.copy2(root / "scripts/run_v6_low_obstacle_phase_f_stack.sh", scripts)
    shutil.copy2(root / "scripts/lib/v6_dynamic_startup.sh", scripts / "lib")
    (scripts / "lib/common.sh").write_text(
        """#!/usr/bin/env bash
export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
require_directory() { [[ -d "$1" ]]; }
require_file() { [[ -f "$1" ]]; }
source_ros() { :; }
validate_v6_dynamic_integration_overlay() { :; }
""",
        encoding="utf-8",
    )
    nested_pid_file = tmp_path / "stack_nested.pid"
    (scripts / "run_v6_kujiale_low_obstacles.sh").write_text(
        """#!/usr/bin/env bash
setsid --wait -- python3 -c 'import signal,sys,time; signal.signal(signal.SIGINT, lambda *_: sys.exit(0)); signal.signal(signal.SIGTERM, lambda *_: sys.exit(0)); time.sleep(300)' &
nested_pid="$!"
printf '%s\n' "${nested_pid}" >"${FAKE_NESTED_PID_FILE}"
trap 'exit 0' INT TERM
wait "${nested_pid}"
""",
        encoding="utf-8",
    )
    for path in (
        scripts / "run_v6_low_obstacle_phase_f_stack.sh",
        scripts / "run_v6_kujiale_low_obstacles.sh",
    ):
        path.chmod(0o755)
    integration = tmp_path / "integration"
    integration.mkdir()
    module2 = tmp_path / "module2"
    constraints = (
        module2 / "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    )
    constraints.parent.mkdir(parents=True)
    constraints.touch()
    run_dir = tmp_path / "run"
    socket = tmp_path / "socket/module2.sock"
    env = os.environ.copy()
    env.update({
        "BIO_NAV_INTEGRATION_ROOT": str(integration),
        "BIO_NAV_MODULE2_V310_ROOT": str(module2),
        "FAKE_NESTED_PID_FILE": str(nested_pid_file),
        "BIO_NAV_PHASE_F_CLEANUP_INT_CHECKS": "10",
        "BIO_NAV_PHASE_F_CLEANUP_TERM_CHECKS": "10",
    })
    process = subprocess.Popen(
        [
            str(scripts / "run_v6_low_obstacle_phase_f_stack.sh"),
            "M0", "--domain", "150", "--run-dir", str(run_dir),
            "--socket", str(socket),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )
    deadline = time.monotonic() + 5.0
    while (
        (not nested_pid_file.exists() or not (run_dir / "module3_ros.pgid").exists())
        and time.monotonic() < deadline
    ):
        time.sleep(0.01)
    assert nested_pid_file.exists()
    nested_pid = int(nested_pid_file.read_text(encoding="utf-8"))
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=10.0)
        assert not Path(f"/proc/{nested_pid}").exists()
        assert not list(run_dir.glob("*.pid"))
        assert not list(run_dir.glob("*.pgid"))
        assert not socket.exists()
    finally:
        for pid in (process.pid, nested_pid):
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
        if process.poll() is None:
            process.wait(timeout=5.0)


def test_campaign_releases_ros_and_isaac_locks_before_arm_two(
    tmp_path, monkeypatch
):
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()
    monkeypatch.setenv("ISAAC_NAV_RUNTIME_DIR", str(runtime_dir))
    manifest = load_manifest(CONFIG)
    manifest = replace(manifest, runs=manifest.runs[:2])
    held = {}
    scene_starts = 0

    def fake_start(name, command, log_path, *, env=None):
        nonlocal scene_starts
        if name == "scene":
            if scene_starts:
                assert causal._lock_is_free(runtime_dir / "isaac.lock")
                assert causal._lock_is_free(runtime_dir / "ros.lock")
            scene_starts += 1
        if name in {"scene", "stack"}:
            component = "isaac" if name == "scene" else "ros"
            stream = (runtime_dir / f"{component}.lock").open("a", encoding="utf-8")
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            held[name] = stream
        return SimpleNamespace(name=name)

    def fake_stop(process, timeout_sec):
        stream = held.pop(process.name, None)
        if stream is not None:
            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
            stream.close()
        return {
            "name": process.name,
            "returncode": 0,
            "cleanup_ok": True,
            "remaining_process_groups": [],
        }

    monkeypatch.setattr(causal, "_start_process", fake_start)
    monkeypatch.setattr(causal, "_stop_process", fake_stop)
    monkeypatch.setattr(
        causal, "_wait_for_startup_ready",
        lambda managed, timeout: {
            "ready": True, "generation": 1, "held": False,
            "reason": "released:activation_gate",
        },
    )
    monkeypatch.setattr(
        causal, "_wait_for_cognitive_ready",
        lambda *args, **kwargs: {"ready": True},
    )
    monkeypatch.setattr(
        causal.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(returncode=0)
    )
    monkeypatch.setattr(
        causal, "record_evidence_from_bag", lambda *args, **kwargs: {"freshness": {}}
    )
    summary = run_campaign(
        manifest,
        AdapterTemplates("/scene", "/stack", "/episode", "/producer-stop"),
        tmp_path / "campaign",
        pilot=False,
        shutdown_timeout_sec=0.1,
    )
    assert scene_starts == 2
    assert [row["state"] for row in summary["runs"]] == [
        "EPISODE_FINISHED", "EPISODE_FINISHED",
    ]
    assert causal._lock_is_free(runtime_dir / "isaac.lock")
    assert causal._lock_is_free(runtime_dir / "ros.lock")


def test_arm_cleanup_failure_aborts_before_the_next_arm(tmp_path, monkeypatch):
    manifest = load_manifest(CONFIG)
    manifest = replace(manifest, runs=manifest.runs[:2])
    starts = []

    def fake_start(name, command, log_path, *, env=None):
        starts.append(name)
        return SimpleNamespace(name=name)

    monkeypatch.setattr(causal, "_start_process", fake_start)
    monkeypatch.setattr(
        causal, "_stop_process",
        lambda process, timeout: {"name": process.name, "returncode": 0},
    )
    monkeypatch.setattr(
        causal, "_wait_for_startup_ready",
        lambda managed, timeout: {
            "ready": True, "generation": 1, "held": False,
            "reason": "released:activation_gate",
        },
    )
    monkeypatch.setattr(
        causal, "_confirm_arm_cleanup",
        lambda *args, **kwargs: {"ok": False, "locks_free": {"ros": False}},
    )
    monkeypatch.setattr(
        causal.subprocess, "run", lambda *args, **kwargs: SimpleNamespace(returncode=0)
    )
    monkeypatch.setattr(
        causal, "record_evidence_from_bag", lambda *args, **kwargs: {"freshness": {}}
    )
    summary = run_campaign(
        manifest,
        AdapterTemplates("/scene", "/stack", "/episode", "/producer-stop"),
        tmp_path / "campaign",
        pilot=False,
        shutdown_timeout_sec=0.1,
    )
    assert starts == ["scene", "stack", "recorder"]
    assert len(summary["runs"]) == 1
    assert summary["runs"][0]["state"] == "ARM_CLEANUP_FAILED"
    assert summary["state"] == "FINISHED_WITH_FAILURES"


def test_pilot_evaluator_uses_only_one_four_arm_repeat(tmp_path):
    manifest = load_manifest(CONFIG)
    _write_evidence(tmp_path, manifest)
    summary = evaluate(manifest, tmp_path, pilot=True)
    assert summary.verdict == "PASS_ENGINEERING_CAUSAL_CANDIDATE_TTL_PENDING"
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
    assert "cleanup_exact_socket" in stack
    assert "socket_has_listener" in stack
    assert "refusing to unlink active Module2 socket" in stack
    assert 'wait "${module3_pid}"' in stack
    assert "setsid --wait --" in stack
    assert "descendant_groups" in stack
    assert 'kill "-${signal_name}" -- "-${pgid}"' in stack
    assert '"${directory}/${name}.identity"' in stack
    assert 'mv -f "${prefix}.identity.tmp"' in stack
    assert "independent_group_candidate" in stack
    assert "stable_checks" in stack
    assert '[[ "${pgid}" == "${own_pgid}" ]]' in stack
    assert "validate_producer_stop_isolation" in stack
    assert "verify_consumer_after_producer_stop" in stack
    assert "ros_lock_is_owned" in stack
    assert stack.count("exit_if_terminating") >= 8
    assert '"${directory}/${name}.pgid"' in stack
    assert 'register_child integration_bridge "${integration_bridge_pid}"' in stack
    assert 'register_child module2_server "${module2_server_pid}"' in stack
    assert 'module2_root="${BIO_NAV_MODULE2_V310_ROOT:-}"' in stack
    assert 'export BIO_NAV_MODULE2_V310_ROOT="${module2_root}"' in stack
    assert "canonical_constraints_file" in stack
    assert stack.index('require_file "${canonical_constraints_file}"') < stack.index(
        'module3_entry=("${script_dir}/run_v6_kujiale_low_obstacles.sh"'
    )
    assert 'setsid --wait -- "${module3_entry[@]}"' in stack
    assert (
        "/home/lyb/Workspace/Bio_Nav/worktrees/"
        "v6-compute-amcl-dual-odom/bio_nav_module2"
    ) not in stack
    assert "run_ros_profile gvg fail_closed auto M3 mixed final" in wrapper
    assert "cognitive_graph_mode:=\"${graph_mode}\"" in wrapper
    assert "run_v6_r5_phase_b_kujiale.sh\" isaac" in wrapper


def test_phase_f_recorder_uses_explicit_best_effort_sensor_qos(tmp_path):
    manifest = load_manifest(CONFIG)
    command = causal._rosbag_command(manifest, tmp_path / "bag")
    option = command.index("--qos-profile-overrides-path")
    qos_path = Path(command[option + 1])
    assert qos_path.name == causal.PHASE_F_QOS_CONFIG
    assert qos_path.is_file()
    qos = causal.yaml.safe_load(qos_path.read_text(encoding="utf-8"))
    for topic in (
        "/scan",
        "/camera/front/depth/image_raw",
        "/camera/front/camera_info",
        "/clock",
    ):
        assert qos[topic]["reliability"] == "best_effort"
        assert qos[topic]["durability"] == "volatile"
    for topic in (
        "/cmd_vel_nav",
        "/cmd_vel_smoothed",
        "/cmd_vel_sim",
        "/collision_monitor_state",
    ):
        assert topic in command
        assert qos[topic] == {
            "history": "keep_last",
            "depth": 1,
            "reliability": "reliable",
            "durability": "volatile",
        }
    assert qos["/tf_static"]["depth"] == 100
    assert command[command.index("--output") + 1] == str(tmp_path / "bag")
    assert "/scan" in command
    assert "/clock" in command


def _run_low_obstacle_wrapper(
    tmp_path, *arguments, create_constraints=True, check=True
):
    root = PACKAGE.parents[2]
    project = tmp_path / "project"
    scripts = project / "scripts"
    (scripts / "lib").mkdir(parents=True, exist_ok=True)
    shutil.copy2(root / "scripts/run_v6_kujiale_low_obstacles.sh", scripts)
    (scripts / "lib/common.sh").write_text(
        """#!/usr/bin/env bash
export PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
die() { printf '%s\\n' "$*" >&2; exit 1; }
require_file() { [[ -f "$1" ]] || die "missing: $1"; }
require_directory() { [[ -d "$1" ]] || die "missing directory: $1"; }
source_ros() { :; }
""",
        encoding="utf-8",
    )
    run_ros = scripts / "run_ros.sh"
    run_ros.write_text(
        "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n", encoding="utf-8"
    )
    run_ros.chmod(0o755)
    run_isaac = scripts / "run_v6_r5_phase_b_kujiale.sh"
    run_isaac.write_text(
        "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n", encoding="utf-8"
    )
    run_isaac.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir(exist_ok=True)
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\"\n", encoding="utf-8"
    )
    fake_ros2.chmod(0o755)
    scenario_directory = project / "ros2_ws/src/robot_experiments/config"
    scenario_directory.mkdir(parents=True, exist_ok=True)
    for name in (
        "v6_final_kujiale_static.yaml",
        "v6_final_kujiale_dynamic.yaml",
        "v6_final_kujiale_appearance.yaml",
    ):
        (scenario_directory / name).touch()
    nav2_config = (
        project
        / "ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml"
    )
    nav2_config.parent.mkdir(parents=True, exist_ok=True)
    nav2_config.touch()
    spawn_poses = (
        project
        / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
    )
    spawn_poses.parent.mkdir(parents=True, exist_ok=True)
    spawn_poses.touch()
    module2 = tmp_path / "installed_module2"
    constraints = (
        module2 / "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    )
    if create_constraints:
        constraints.parent.mkdir(parents=True, exist_ok=True)
        constraints.touch()
    else:
        module2.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.pop("V6_COGNITIVE_PROFILE", None)
    env["BIO_NAV_MODULE2_V310_ROOT"] = str(module2)
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    result = subprocess.run(
        [str(scripts / "run_v6_kujiale_low_obstacles.sh"), *arguments],
        check=check,
        capture_output=True,
        text=True,
        env=env,
    )
    return project, constraints, result


def test_v6_low_obstacle_default_condition_is_explicit_static(tmp_path):
    project, _constraints, implicit = _run_low_obstacle_wrapper(
        tmp_path, "runner", str(tmp_path / "runs"), "run_indices:=2,4"
    )
    _project, _constraints, explicit = _run_low_obstacle_wrapper(
        tmp_path,
        "--condition",
        "static",
        "runner",
        str(tmp_path / "runs"),
        "run_indices:=2,4",
    )

    assert implicit.stdout == explicit.stdout
    assert (
        f"scenario_file:={project}/ros2_ws/src/robot_experiments/config/"
        "v6_final_kujiale_static.yaml"
    ) in implicit.stdout.splitlines()


@pytest.mark.parametrize(
    ("condition", "scenario_name"),
    [
        ("static", "v6_final_kujiale_static.yaml"),
        ("dynamic", "v6_final_kujiale_dynamic.yaml"),
        ("appearance", "v6_final_kujiale_appearance.yaml"),
    ],
)
def test_v6_runner_condition_selects_scenario_and_fixed_nav2_config(
    tmp_path, condition, scenario_name
):
    project, _constraints, result = _run_low_obstacle_wrapper(
        tmp_path,
        "--condition",
        condition,
        "runner",
        str(tmp_path / "runs"),
        "run_indices:=1,3",
        "record_bag:=false",
        "record_evidence:=true",
        "resume:=true",
        "require_module2_planning_ready:=true",
        "clear_slam_localization_buffer:=false",
        "reset_map_base_translation_tolerance_m:=0.10",
        "navigation_execution_backend:=route_guided",
    )
    argv = result.stdout.splitlines()

    assert (
        f"scenario_file:={project}/ros2_ws/src/robot_experiments/config/"
        f"{scenario_name}"
    ) in argv
    assert "nav2_profile:=v6_low_obstacle_isolation" in argv
    assert (
        f"nav2_config_file:={project}/ros2_ws/src/robot_navigation/config/"
        "nav2_v6_low_obstacle_isolation.yaml"
    ) in argv
    assert f"output_directory:={tmp_path / 'runs'}" in argv
    assert "run_indices:=1,3" in argv
    assert "record_bag:=false" in argv
    assert "record_evidence:=true" in argv
    assert "resume:=true" in argv
    assert "require_module2_planning_ready:=true" in argv
    assert "clear_slam_localization_buffer:=false" in argv
    assert "reset_map_base_translation_tolerance_m:=0.10" in argv
    assert "navigation_execution_backend:=route_guided" in argv
    if condition == "dynamic":
        assert not any(argument.startswith("dynamic_variant_id:=") for argument in argv)
        assert not any(argument.startswith("dynamic_case_id:=") for argument in argv)


@pytest.mark.parametrize(
    ("condition", "override"),
    [
        ("static", "scenario_file:=/tmp/not-static.yaml"),
        ("dynamic", "spawn_poses_file:=/tmp/not-kujiale.yaml"),
        ("appearance", "nav2_profile:=caller-profile"),
        ("static", "nav2_config_file:=/tmp/not-isolation.yaml"),
        ("appearance", "dynamic_case_id:=other"),
        ("static", "dynamic_variant_id:=v5"),
        ("dynamic", "dynamic_seed:=9001"),
        ("appearance", "robot_config_file:=/tmp/not-jackal.yaml"),
    ],
)
def test_v6_runner_rejects_condition_identity_overrides(
    tmp_path, condition, override
):
    _project, _constraints, result = _run_low_obstacle_wrapper(
        tmp_path,
        "--condition",
        condition,
        "runner",
        str(tmp_path / "runs"),
        override,
        check=False,
    )

    assert result.returncode == 1
    assert result.stdout == ""
    assert "fixes runner scenario/spawn/Nav2 identity" in result.stderr
    assert f"rejected override: {override}" in result.stderr


@pytest.mark.parametrize(
    ("condition", "config_name", "appearance_arguments"),
    [
        ("static", "v6_kujiale_low_obstacles_frozen.yaml", ["--appearance-profile", "baseline"]),
        ("dynamic", "v6_kujiale_dynamic_g2_crossing.yaml", []),
        (
            "appearance",
            "v6_kujiale_low_obstacles_frozen.yaml",
            ["--appearance-config", "kujiale_appearance_profiles.yaml", "--appearance-profile", "baseline"],
        ),
    ],
)
def test_v6_isaac_condition_wires_physical_and_appearance_argv(
    tmp_path, condition, config_name, appearance_arguments
):
    project, _constraints, result = _run_low_obstacle_wrapper(
        tmp_path,
        "--condition",
        condition,
        "isaac",
        "--headless",
        "--validate-only",
    )
    argv = result.stdout.splitlines()

    assert "--disable-dlss" not in argv
    assert "--no-disable-dlss" not in argv

    assert argv[:4] == [
        "isaac",
        "--dynamic-obstacle-config",
        f"{project}/isaac_sim/configs/experiments/{config_name}",
        "--dynamic-obstacles",
    ]
    assert argv[-2:] == ["--headless", "--validate-only"]
    if condition == "appearance":
        assert argv[4:] == [
            "--appearance-config",
            f"{project}/isaac_sim/configs/experiments/{appearance_arguments[1]}",
            "--appearance-profile",
            "baseline",
            "--headless",
            "--validate-only",
        ]
    else:
        assert argv[4:-2] == appearance_arguments
    if condition == "dynamic":
        assert "--dynamic-case-id" not in argv
        assert "--dynamic-variant-id" not in argv


@pytest.mark.parametrize(
    ("condition", "override_arguments", "rejected"),
    [
        ("static", ("--dynamic-obstacle-config", "/tmp/other.yaml"), "--dynamic-obstacle-config"),
        ("dynamic", ("--dynamic-obstacles",), "--dynamic-obstacles"),
        ("appearance", ("--no-dynamic-obstacles",), "--no-dynamic-obstacles"),
        ("static", ("--dynamic-case-id=other",), "--dynamic-case-id=other"),
        ("dynamic", ("--dynamic-variant-id", "v5"), "--dynamic-variant-id"),
        ("appearance", ("--dynamic-seed=9001",), "--dynamic-seed=9001"),
        ("static", ("--appearance-config=/tmp/other.yaml",), "--appearance-config=/tmp/other.yaml"),
        ("dynamic", ("--appearance-profile", "bright_warm"), "--appearance-profile"),
        ("appearance", ("--environment-usd=/tmp/other.usd",), "--environment-usd=/tmp/other.usd"),
        ("static", ("--environment-root", "/tmp/scenes"), "--environment-root"),
        ("dynamic", ("--spawn-poses-file=/tmp/other.yaml",), "--spawn-poses-file=/tmp/other.yaml"),
        ("appearance", ("--spawn-pose", "other"), "--spawn-pose"),
    ],
)
def test_v6_isaac_rejects_condition_identity_overrides(
    tmp_path, condition, override_arguments, rejected
):
    _project, _constraints, result = _run_low_obstacle_wrapper(
        tmp_path,
        "--condition",
        condition,
        "isaac",
        *override_arguments,
        check=False,
    )

    assert result.returncode == 1
    assert result.stdout == ""
    assert "fixes Isaac scene/obstacle/appearance identity" in result.stderr
    assert f"rejected override: {rejected}" in result.stderr


def test_v6_physical_configs_stay_default_off_and_wrapper_activates_them():
    root = PACKAGE.parents[2]
    for name in (
        "v6_kujiale_low_obstacles_frozen.yaml",
        "v6_kujiale_dynamic_g2_crossing.yaml",
    ):
        document = yaml.safe_load(
            (root / "isaac_sim/configs/experiments" / name).read_text(
                encoding="utf-8"
            )
        )
        assert document["enabled"] is False


@pytest.mark.parametrize(
    ("profile_arguments", "condition"),
    [
        (("ros", "M2"), "dynamic"),
        (("shadow",), "appearance"),
        (("ros-d", "hybrid"), "dynamic"),
    ],
)
def test_v6_ros_profiles_do_not_fork_by_condition(
    tmp_path, profile_arguments, condition
):
    _project, _constraints, baseline = _run_low_obstacle_wrapper(
        tmp_path, *profile_arguments
    )
    _project, _constraints, selected = _run_low_obstacle_wrapper(
        tmp_path, "--condition", condition, *profile_arguments
    )

    assert selected.stdout == baseline.stdout


@pytest.mark.parametrize(
    "arguments",
    [
        ("--condition", "invalid", "runner"),
        ("--condition",),
    ],
)
def test_v6_wrapper_rejects_invalid_condition(tmp_path, arguments):
    _project, _constraints, result = _run_low_obstacle_wrapper(
        tmp_path, *arguments, check=False
    )

    assert result.returncode == 1
    assert result.stdout == ""


def test_v6_low_obstacle_profile_is_registered_as_appearance_safe():
    runner = (PACKAGE / "robot_experiments/experiment_runner.py").read_text(
        encoding="utf-8"
    )
    appearance_profiles = runner.split(
        "APPEARANCE_NAV2_PROFILES = frozenset({", 1
    )[1].split("})", 1)[0]
    assert '"v6_low_obstacle_isolation"' in appearance_profiles


def test_v6_runner_fixes_spawn_and_keeps_module2_readiness_explicit(tmp_path):
    project, _constraints, baseline = _run_low_obstacle_wrapper(
        tmp_path / "baseline",
        "runner",
        str(tmp_path / "m0"),
        "experiment_arm:=M0",
    )
    baseline_argv = baseline.stdout.splitlines()
    assert (
        f"spawn_poses_file:={project}/isaac_sim/configs/environments/"
        "kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
        in baseline_argv
    )
    assert "experiment_arm:=M0" in baseline_argv
    assert not any(
        argument.startswith("require_module2_planning_ready:=")
        for argument in baseline_argv
    )

    _project, _constraints, route_prior = _run_low_obstacle_wrapper(
        tmp_path / "route-prior",
        "runner",
        str(tmp_path / "m3-route-prior"),
        "experiment_arm:=M3",
        "require_module2_planning_ready:=true",
        "module2_planning_ready_timeout_sec:=120.0",
    )
    route_prior_argv = route_prior.stdout.splitlines()
    assert "require_module2_planning_ready:=true" in route_prior_argv
    assert "module2_planning_ready_timeout_sec:=120.0" in route_prior_argv


@pytest.mark.parametrize("arm", ["M0", "M1", "M2", "M3"])
def test_phase_f_mixed_argv_uses_occupancy_map_without_posegraph(tmp_path, arm):
    project, constraints, result = _run_low_obstacle_wrapper(tmp_path, "ros", arm)
    argv = result.stdout.splitlines()
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
        "spawn_pose_name:=long_route_start_g1",
        f"map_file:={project}/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        f"route_graph_file:={project}/ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson",
        f"cognitive_constraints_override_file:={constraints}",
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
    project, _constraints, result = _run_low_obstacle_wrapper(tmp_path, "shadow")
    argv = result.stdout.splitlines()
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


def test_phase_f_mixed_fails_before_ros_when_canonical_constraints_are_missing(
    tmp_path,
):
    _project, constraints, result = _run_low_obstacle_wrapper(
        tmp_path, "ros", "M0", create_constraints=False, check=False
    )

    assert result.returncode == 1
    assert result.stdout == ""
    assert f"missing: {constraints}" in result.stderr


def test_phase_f_rejects_caller_map_context_override(tmp_path):
    _project, _constraints, result = _run_low_obstacle_wrapper(
        tmp_path,
        "ros",
        "M2",
        "cognitive_constraints_override_file:=/tmp/not-canonical.yaml",
        check=False,
    )

    assert result.returncode == 1
    assert "rejected override: cognitive_constraints_override_file" in result.stderr


def test_legacy_shadow_preserves_caller_map_context_argument(tmp_path):
    override = "/tmp/legacy-shadow-constraints.yaml"
    _project, _constraints, result = _run_low_obstacle_wrapper(
        tmp_path,
        "shadow",
        f"cognitive_constraints_override_file:={override}",
    )

    assert result.stdout.splitlines()[-1] == (
        f"cognitive_constraints_override_file:={override}"
    )


@pytest.mark.parametrize("command", ["manifest", "plan"])
def test_non_runtime_cli_commands_emit_json(command, capsys, tmp_path):
    argv = [command, "--config", str(CONFIG)]
    assert cli(argv) == 0
    assert json.loads(capsys.readouterr().out)
