import json
import math
from pathlib import Path

import pytest

from robot_experiments.estimated_state_metrics import evaluate_trajectory
from robot_experiments.estimated_state_metrics import PoseSample
from robot_experiments.estimated_state_metrics import stream_diagnostics
from robot_experiments.v6_estimated_calibration import assess_shadow_promotion
from robot_experiments.v6_estimated_calibration import build_dispatch_plan
from robot_experiments.v6_estimated_calibration import build_manifest
from robot_experiments.v6_estimated_calibration import evaluate_campaign
from robot_experiments.v6_estimated_calibration import load_config
from robot_experiments.v6_estimated_calibration import main


PACKAGE_ROOT = Path(__file__).parents[1]
CONFIG = PACKAGE_ROOT / "config" / "v6_estimated_calibration.yaml"


def _sample(stamp, x, y=0.0, yaw=0.0, covariance=None):
    return PoseSample(int(round(stamp * 1.0e9)), x, y, yaw, covariance)


def _manifest():
    return build_manifest(load_config(CONFIG))


def test_exact_45_episode_matrix_is_arm_grouped_and_ordered():
    manifest = _manifest()
    episodes = manifest["episodes"]

    assert len(episodes) == manifest["episode_count"] == 45
    assert [episode["arm"] for episode in episodes] == ["off"] * 15 + ["shadow"] * 15 + ["fused"] * 15
    for offset in (0, 15, 30):
        assert [episode["scenario_id"] for episode in episodes[offset:offset + 15]] == (
            ["straight_3m"] * 3
            + ["ccw_360"] * 3
            + ["cw_360"] * 3
            + ["s_route"] * 3
            + ["rivermark_static_start_to_g1"] * 3
        )
    assert all(episode["reset_count"] == 1 and episode["retry_count"] == 0 for episode in episodes)
    assert len({episode["reset_seed"] for episode in episodes}) == 45


def test_primitive_and_rivermark_contract_are_exact():
    episodes = _manifest()["episodes"]

    assert episodes[0]["execution"]["segments"] == [{"duration_sec": 10.0, "linear_x": 0.30, "angular_z": 0.0}]
    assert episodes[3]["execution"]["segments"] == [{"duration_sec": 12.566, "linear_x": 0.0, "angular_z": 0.50}]
    assert episodes[6]["execution"]["segments"] == [{"duration_sec": 12.566, "linear_x": 0.0, "angular_z": -0.50}]
    assert episodes[9]["execution"]["segments"] == [
        {"duration_sec": 2.5, "linear_x": 0.25, "angular_z": 0.45},
        {"duration_sec": 5.0, "linear_x": 0.25, "angular_z": -0.45},
        {"duration_sec": 2.5, "linear_x": 0.25, "angular_z": 0.45},
    ]
    route = episodes[12]["execution"]
    assert route["type"] == "nav2_route_goal"
    assert route["wrapper"] == "current_occupancy_only_estimated"
    assert route["goal"] == {"x": 1.521014, "y": 131.813786, "yaw_deg": 135.0}


def test_arm_arguments_and_ground_truth_firewall():
    episodes = _manifest()["episodes"]

    assert episodes[0]["launch_arguments"] == {
        "odometry_source": "wheel_imu", "lidar_backend": "off", "lidar_odom_validated": False,
    }
    assert episodes[15]["launch_arguments"] == {
        "odometry_source": "wheel_imu", "lidar_backend": "rf2o", "lidar_odom_validated": False,
    }
    assert episodes[30]["launch_arguments"] == {
        "odometry_source": "wheel_imu_lidar", "lidar_backend": "rf2o", "lidar_odom_validated": True,
    }
    assert episodes[30]["requires_shadow_validation"] is True
    assert episodes[30]["validated_argument_is_not_evidence"] is True
    assert all(not topic.startswith("/ground_truth/") for topic in episodes[0]["dispatcher_topics"])
    assert "/ground_truth/odom" in episodes[0]["passive_evaluator_topics"]


def test_motion_dispatcher_uses_reset_response_estimated_readiness_and_tf_only():
    motion_source = (
        PACKAGE_ROOT / "robot_experiments" / "motion_benchmark.py"
    ).read_text(encoding="utf-8")
    evaluator_source = (
        PACKAGE_ROOT / "robot_experiments" / "estimated_state_evaluator.py"
    ).read_text(encoding="utf-8")

    assert '"/ground_truth/' not in motion_source
    assert '"/odom"' in motion_source
    assert 'reset_response.success' in motion_source
    assert 'lookup_transform("odom", "base_link", Time())' in motion_source
    assert 'math.hypot(sample.x, sample.y)' not in motion_source
    assert "'/ground_truth/odom'" in evaluator_source
    assert "evaluator_only_ground_truth" in evaluator_source


def _good_shadow_evaluation():
    covariance = {
        "finite_fraction": 1.0,
        "symmetric_fraction": 1.0,
        "positive_semidefinite_fraction": 1.0,
    }
    return {
        "episodes": [
            {
                "episode_id": f"shadow-{index}",
                "arm": "shadow",
                "status": "EVALUATED",
                "selected_metrics": {
                    "input": {
                        "backward_stamps": 0,
                        "pose_jump_count": 0,
                        "yaw_jump_count": 0,
                        "frequency_hz": 50.0,
                        "covariance": covariance,
                    },
                    "scale": {"linear": 1.0, "yaw": 1.0},
                    "association": {"best_estimate_time_offset_ms": 10.0},
                },
            }
            for index in range(15)
        ]
    }


def test_fused_dispatch_is_fail_closed_until_gate_and_explicit_flag():
    manifest = _manifest()
    default = build_dispatch_plan(manifest)
    evidence_without_flag = build_dispatch_plan(manifest, shadow_evaluation=_good_shadow_evaluation())
    promoted = build_dispatch_plan(
        manifest,
        explicit_fused_promotion=True,
        shadow_evaluation=_good_shadow_evaluation(),
    )

    assert default["fused_dispatch_allowed"] is False
    assert all(row["dispatch_status"] == "BLOCKED" for row in default["episodes"] if row["arm"] == "fused")
    assert evidence_without_flag["shadow_gate"]["passed"] is True
    assert evidence_without_flag["fused_dispatch_allowed"] is False
    assert promoted["fused_dispatch_allowed"] is True
    assert all(row["dispatch_status"] == "PLANNED" for row in promoted["episodes"])


def test_shadow_gate_rejects_jump_and_missing_covariance_health():
    evaluation = _good_shadow_evaluation()
    evaluation["episodes"][2]["selected_metrics"]["input"]["pose_jump_count"] = 1
    evaluation["episodes"][3]["selected_metrics"]["input"]["covariance"]["positive_semidefinite_fraction"] = 0.9

    result = assess_shadow_promotion(evaluation)

    assert result["passed"] is False
    assert any("pose_jumps" in reason for reason in result["failure_reasons"])
    assert any("covariance_psd" in reason for reason in result["failure_reasons"])


def test_run_command_explicitly_records_adapter_absent_not_run(tmp_path):
    assert main(["--config", str(CONFIG), "run", "--output-dir", str(tmp_path)]) == 0
    report = json.loads((tmp_path / "calibration_run.json").read_text(encoding="utf-8"))

    assert report["status"] == "NOT_RUN"
    assert report["reason"] == "runtime_adapter_not_implemented"
    assert len(report["episodes"]) == 45
    assert all(row["run_status"] == "NOT_RUN" for row in report["episodes"])


def test_missing_reports_remain_not_run_in_evaluation(tmp_path):
    evaluation = evaluate_campaign(_manifest(), tmp_path)

    assert evaluation["status"] == "INCOMPLETE_NOT_RUN"
    assert evaluation["evaluated_count"] == 0
    assert all(row["status"] == "NOT_RUN" for row in evaluation["episodes"])
    assert evaluation["fused_comparison"]["status"] == "NOT_AVAILABLE"


def test_absolute_and_aligned_ate_are_distinct_and_rpe_is_fixed_interval():
    truth = [_sample(index * 0.5, index * 0.5) for index in range(7)]
    estimate = [_sample(index * 0.5, 10.0 + index * 0.55) for index in range(7)]

    result = evaluate_trajectory(estimate, truth, 0)

    assert result.summary["absolute_ate"]["xy_m"]["rmse"] > 9.0
    assert result.summary["aligned_ate"]["xy_m"]["rmse"] < 0.20
    assert result.summary["relative_ate"] == result.summary["aligned_ate"]
    assert result.summary["rpe_fixed_1s"]["xy_m"]["count"] == 5
    assert result.summary["rpe_fixed_1m"]["xy_m"]["count"] == 5
    assert result.summary["scale"]["linear"] == pytest.approx(1.1)


def test_bounded_time_offset_scale_bias_covariance_nees_and_no_nis():
    covariance = (
        0.04, 0.0, 0.0,
        0.0, 0.04, 0.0,
        0.0, 0.0, 0.01,
    )
    truth = [_sample(index, 0.0, 0.0, index * 0.2) for index in range(5)]
    estimate = [_sample(index + 0.10, 0.01, 0.0, index * 0.22, covariance) for index in range(5)]

    result = evaluate_trajectory(
        estimate,
        truth,
        max_time_delta_ns=20_000_000,
        max_time_offset_ns=200_000_000,
        time_offset_step_ns=10_000_000,
    )

    assert result.summary["association"]["best_estimate_time_offset_ms"] == pytest.approx(-100.0)
    assert result.summary["scale"]["yaw"] == pytest.approx(1.1)
    assert result.summary["scale"]["yaw_change_bias_rad"] == pytest.approx(0.08)
    assert result.summary["planar_nees"]["status"] == "AVAILABLE"
    assert result.summary["planar_nees"]["not_nis"] is True
    assert result.summary["nis"]["status"] == "NOT_AVAILABLE"
    diagnostics = stream_diagnostics(estimate)
    assert diagnostics["covariance"]["finite_fraction"] == 1.0
    assert diagnostics["covariance"]["symmetric_fraction"] == 1.0
    assert diagnostics["covariance"]["positive_semidefinite_fraction"] == 1.0


def test_covariance_symmetry_and_psd_are_checked_separately():
    asymmetric = (1.0, 0.2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    indefinite = (1.0, 2.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    diagnostics = stream_diagnostics([
        _sample(0.0, 0.0, covariance=asymmetric),
        _sample(0.1, 0.0, covariance=indefinite),
    ])

    assert diagnostics["covariance"]["finite_fraction"] == 1.0
    assert diagnostics["covariance"]["symmetric_fraction"] == pytest.approx(0.5)
    assert diagnostics["covariance"]["positive_semidefinite_fraction"] == 0.0
