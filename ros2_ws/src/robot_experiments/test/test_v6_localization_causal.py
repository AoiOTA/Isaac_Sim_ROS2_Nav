import json
from pathlib import Path

import pytest

from robot_experiments.v6_localization_causal import (
    ARMS,
    CORE_CASES,
    COUNTERBALANCED_ORDERS,
    DISPATCHER_TOPICS,
    build_plan,
    cli,
    load_manifest,
)
from robot_experiments.v6_localization_causal_evaluator import (
    absolute_error_samples,
    evaluate_campaign,
)


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "v6_localization_causal.yaml"


def test_manifest_expands_exact_core60_with_counterbalanced_orders():
    manifest = load_manifest(CONFIG)
    runs = manifest["core_runs"]
    assert len(runs) == 60
    assert len({run["run_id"] for run in runs}) == 60
    for case in CORE_CASES:
        rows = [run for run in runs if run["case"] == case]
        assert len(rows) == 20
        assert {arm: sum(run["arm"] == arm for run in rows) for arm in ARMS} == {
            arm: 5 for arm in ARMS
        }
        for repeat, order in enumerate(COUNTERBALANCED_ORDERS, start=1):
            assert tuple(run["arm"] for run in rows if run["repeat"] == repeat) == order
    assert {row["case"] for row in manifest["engineering_preflight_runs"]} == {"S1", "S2"}
    assert not any(row["counts_toward_core60"] for row in manifest["engineering_preflight_runs"])


def test_arms_and_all_common_runtime_axes_are_frozen():
    manifest = load_manifest(CONFIG)
    common = manifest["common_runtime"]
    assert common == {
        "scene_id": "kujiale_0026_A_to_B_door_open",
        "odometry_mode": "estimated",
        "cognitive_profile": "M0",
        "module2_planning_influence": False,
        "cognitive_graph_mode": "gvg",
        "direct_rgbd_costmap_enabled": False,
        "use_rviz": False,
        "structure_tf_source": "isaac",
        "goal_id": "G2",
        "initial_physical_pose": "long_route_start_g2",
        "automatic_rescue_enabled": False,
    }
    assert manifest["arms"]["L0"]["integration_enabled"] is False
    assert manifest["arms"]["L1"]["startup_initialpose_writes"] == 0
    assert manifest["arms"]["L2"]["startup_initialpose_writes"] == 1
    assert manifest["arms"]["L3"]["manual_rescue_allowed_after_lost"] is True


def test_dispatcher_is_gt_free_and_s3_is_one_shot_manual_only_for_l3():
    manifest = load_manifest(CONFIG)
    assert not [topic for topic in DISPATCHER_TOPICS if topic.startswith("/ground_truth/")]
    plan = build_plan(manifest)
    assert plan["core_run_count"] == 60
    for row in plan["runs"]:
        assert not [topic for topic in row["dispatcher_topics"] if topic.startswith("/ground_truth/")]
        if row["case"] != "S3":
            continue
        trigger = [step for step in row["steps"] if step["action"] == "call_trigger_once"]
        assert len(trigger) == 1 and trigger[0]["retry"] is False
        rescue = [step for step in row["steps"] if step["action"] == "request_manual_rescue_once_after_lost"]
        assert len(rescue) == (1 if row["arm"] == "L3" else 0)


def test_absolute_error_has_no_first_frame_alignment():
    errors = absolute_error_samples(
        [
            {"stamp_s": 0.0, "x": 1.0, "y": 0.0, "yaw_deg": 20.0},
            {"stamp_s": 1.0, "x": 1.0, "y": 0.0, "yaw_deg": 20.0},
        ],
        [
            {"stamp_s": 0.0, "x": 0.0, "y": 0.0, "yaw_deg": 0.0},
            {"stamp_s": 1.0, "x": 0.0, "y": 0.0, "yaw_deg": 0.0},
        ],
    )
    assert [row.position_error_m for row in errors] == pytest.approx([1.0, 1.0])
    assert [row.yaw_error_deg for row in errors] == pytest.approx([20.0, 20.0])


def _evidence(run):
    case = run["case"]
    arm = run["arm"]
    samples = []
    truth = []
    for stamp in range(0, 15):
        if case == "S0":
            converge_at = 8 if arm in {"L0", "L1"} else 4
            error = 1.0 if stamp < converge_at else 0.1
        elif case == "S3":
            recover_at = 5 if arm == "L3" else 9 if arm == "L2" else 12
            error = 0.1 if stamp == 0 or stamp >= recover_at else 1.0
        else:
            error = 0.1
        samples.append({"stamp_s": stamp, "x": error, "y": 0.0, "yaw_deg": 0.0})
        truth.append({"stamp_s": stamp, "x": 0.0, "y": 0.0, "yaw_deg": 0.0})
    initialpose = []
    expected = run["expected"]
    if expected["total_initialpose_writes"]:
        source = "integration" if expected["integration_initialpose_writes"] else "runner"
        initialpose = [{"stamp_s": 1.0, "source": source}]
    rescues = (
        [{"stamp_s": 1.0, "source": "runner_explicit"}]
        if expected["manual_rescue_requests"] else []
    )
    return {
        "run_id": run["run_id"],
        "arm": arm,
        "case": case,
        "seed": run["seed"],
        "kidnap_trigger_stamp_s": 0.0 if case == "S3" else None,
        "kidnap_service_calls": 1 if case == "S3" else 0,
        "dispatcher": {
            "topics": list(DISPATCHER_TOPICS),
            "estimated_map_poses": samples,
            "initialpose_events": initialpose,
            "manual_rescue_events": rescues,
            "pause_intervals": [{"start_s": 0.0, "end_s": 0.5}] if case == "S3" else [],
            "cmd_vel": [{"stamp_s": 0.25, "linear_x": 0.0, "angular_z": 0.0}],
            "publisher_owners": {"/odom": "robot_localization", "map->odom": "amcl"},
            "integration_activity": {
                "mode": run["expected"]["integration_mode"],
                "initialpose_writes": run["expected"]["integration_initialpose_writes"],
                "pose_correction_writes": 0 if arm == "L1" else run["expected"]["integration_initialpose_writes"],
            },
        },
        "passive_evaluator": {
            "ground_truth_map_poses": truth,
            "collision_count": 0,
        },
    }


def test_synthetic_contract_exercises_convergence_lost_recovery_and_wrong_reseed(tmp_path):
    manifest = load_manifest(CONFIG)
    for run in manifest["core_runs"]:
        (tmp_path / f"{run['run_id']}.json").write_text(
            json.dumps(_evidence(run)), encoding="utf-8"
        )
    result = evaluate_campaign(manifest, tmp_path)
    assert result["verdict"] == "PASS_CRITERIA"
    assert result["aggregate"]["L2_S0_fast_convergence"] == 5
    assert result["aggregate"]["L3_S3_fast_recovery"] == 5
    assert not [row for row in result["results"] if row["case"] == "W0" and row["initialpose_count"]]


def test_run_without_adapter_is_explicit_not_run(capsys):
    assert cli(["run", "--config", str(CONFIG)]) == 2
    output = json.loads(capsys.readouterr().out)
    assert output["state"] == "NOT_RUN"
    assert output["qualification"] == "ENGINEERING_CAUSAL_NOT_RUN"
