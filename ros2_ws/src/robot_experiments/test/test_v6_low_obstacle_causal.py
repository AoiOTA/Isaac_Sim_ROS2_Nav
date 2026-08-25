import json
from pathlib import Path

import pytest

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
            "max_typed_obstacle_age_sec": 0.8 if stale else 0.1,
            "stale_applied_count": 1 if stale else 0,
            "stopped_before_dispatch": stale,
            "layer_zero_write": stale,
            "critic_not_applied": stale,
            "ttl_expiry_observed": run.arm in {"M2", "M3"},
            "ttl_expiry_zero_write": run.arm in {"M2", "M3"},
            "ttl_expiry_critic_not_applied": run.arm in {"M2", "M3"},
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
        RecordedMessage("/camera/front/depth/image_raw", stamp, {"header": {"stamp": stamp}}),
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
            "obstacles": [{
                "id": "v6_low_box_solo", "pose_xy_m": [-0.45, -0.35],
                "radius_m": 0.2, "confidence": 0.9,
            }],
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", stamp, {
            "consumer": "/global_costmap:layer", "raised_cell_count": 7,
            "active_cell_count": 7, "maximum_cost": 60, "applied": True,
            "message_age_ms": 20.0,
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", stamp, {
            "consumer": "/local_costmap:layer", "raised_cell_count": 5,
            "active_cell_count": 5, "maximum_cost": 55, "applied": True,
            "message_age_ms": 20.0,
        }),
        RecordedMessage("/bio_nav/cognitive_risk_critic/status", stamp, {
            "applied": True, "fallback_reason": "cost_delta_applied=true;obstacle_applied=true",
            "message_age_ms": 20.0,
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", stamp + 700_000_000, {
            "consumer": "/global_costmap:layer", "raised_cell_count": 0,
            "active_cell_count": 0, "maximum_cost": 0, "applied": False,
            "message_age_ms": 700.0,
        }),
        RecordedMessage("/bio_nav/cognitive_obstacle_layer/status", stamp + 700_000_000, {
            "consumer": "/local_costmap:layer", "raised_cell_count": 0,
            "active_cell_count": 0, "maximum_cost": 0, "applied": False,
            "message_age_ms": 700.0,
        }),
        RecordedMessage("/bio_nav/cognitive_risk_critic/status", stamp + 700_000_000, {
            "applied": False, "fallback_reason": "stale", "message_age_ms": 700.0,
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
    assert evidence["freshness"]["ttl_expiry_critic_not_applied"] is True
    assert evidence["synchronized_samples"][0]["scan_point_count"] == 80
    assert evidence["passive"]["success"] is True


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
    assert "run_ros_profile gvg fail_closed auto M3 mixed final" in wrapper
    assert "cognitive_graph_mode:=\"${graph_mode}\"" in wrapper
    assert "run_v6_r5_phase_b_kujiale.sh\" isaac" in wrapper


@pytest.mark.parametrize("command", ["manifest", "plan"])
def test_non_runtime_cli_commands_emit_json(command, capsys):
    assert cli([command, "--config", str(CONFIG)]) == 0
    assert json.loads(capsys.readouterr().out)
