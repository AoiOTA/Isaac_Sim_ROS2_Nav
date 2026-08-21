from __future__ import annotations

import math
from pathlib import Path

import pytest

from robot_experiments.imu_regime_analysis import (
    ScalarSample,
    YawSample,
    analyze_segment,
    command_windows,
    phase_window_metrics,
    summarize,
)
from robot_experiments.motion_benchmark import load_motion_config


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config/v6_imu_regime_diagnostic.yaml"


def _series(scale: float, *, duplicate: bool = False):
    times = [index * 0.1 for index in range(101)]
    if duplicate:
        times[2] = times[1]
    raw = [ScalarSample(stamp, 0.5) for stamp in times]
    corrected = [ScalarSample(stamp, 0.5 * scale) for stamp in times]
    gt = [YawSample(stamp, 0.5 * scale * stamp) for stamp in times]
    return raw, corrected, gt


def test_v6_diagnostic_yaml_uses_strict_schema_and_frozen_order():
    config = load_motion_config(CONFIG)
    assert [item.identifier for item in config.primitives] == [
        "cw_360",
        "ccw_360",
        "arc_v005_cw",
        "arc_v005_ccw",
        "arc_v010_cw",
        "arc_v010_ccw",
        "arc_v025_cw",
        "arc_v025_ccw",
        "s_route",
    ]
    assert [config.reset_seed + index for index in range(len(config.primitives))] == list(range(8610, 8619))
    assert [(item.segments[0].linear_x, item.segments[0].angular_z) for item in config.primitives[2:8]] == [
        (0.05, -0.5), (0.05, 0.5),
        (0.10, -0.5), (0.10, 0.5),
        (0.25, -0.5), (0.25, 0.5),
    ]
    assert [(segment.duration_sec, segment.linear_x, segment.angular_z) for segment in config.primitives[-1].segments] == [
        (2.5, 0.25, 0.45), (5.0, 0.25, -0.45), (2.5, 0.25, 0.45)
    ]
    assert "external passive 10 s window" in CONFIG.read_text(encoding="utf-8")


def test_synthetic_segment_recovers_scale_and_corrected_metrics():
    raw, corrected, gt = _series(0.93)
    result = analyze_segment(
        identifier="cw", reset_generation=3,
        command_linear_mps=0.0, command_angular_radps=-0.5,
        raw=raw, corrected=corrected, ground_truth=gt,
    )
    assert result["status"] == "OK"
    assert result["k_star"] == pytest.approx(0.93, abs=1e-12)
    assert result["corrected_endpoint_error_rad"] == pytest.approx(0.0, abs=1e-12)
    assert result["corrected_aligned_rmse_rad"] == pytest.approx(0.0, abs=1e-12)
    interval = result["scale_interval_le_5deg"][0]
    assert interval[0] < 0.93 < interval[1]


def test_summary_confirms_empty_global_constant_intersection():
    first = analyze_segment(
        identifier="pure", reset_generation=1,
        command_linear_mps=0.0, command_angular_radps=0.5,
        raw=_series(0.93)[0], corrected=_series(0.93)[1], ground_truth=_series(0.93)[2],
    )
    second = analyze_segment(
        identifier="arc", reset_generation=2,
        command_linear_mps=0.25, command_angular_radps=0.5,
        raw=_series(1.00)[0], corrected=_series(1.00)[1], ground_truth=_series(1.00)[2],
    )
    result = summarize([first, second])
    assert result["verdict"] == "CONFIRMED_NO_GLOBAL_CONSTANT"
    assert result["global_scale_intersection"] == []
    assert set(result["bins"]) == {"v=0.00/CCW", "v=0.25/CCW"}


def test_stamp_anomaly_fails_and_missing_data_is_ambiguous():
    raw, corrected, gt = _series(0.93, duplicate=True)
    invalid = analyze_segment(
        identifier="bad_stamp", reset_generation=1,
        command_linear_mps=0.0, command_angular_radps=0.5,
        raw=raw, corrected=corrected, ground_truth=gt,
    )
    assert invalid["status"] == "STAMP_INVALID"
    assert summarize([invalid])["verdict"] == "FAIL"
    missing = analyze_segment(
        identifier="missing", reset_generation=2,
        command_linear_mps=0.1, command_angular_radps=-0.5,
        raw=[], corrected=[], ground_truth=[],
    )
    assert missing["status"] == "INSUFFICIENT_DATA"
    assert summarize([missing])["verdict"] == "AMBIGUOUS"


def test_goal_identity_non_degrade_can_promote_nonempty_candidate():
    raw, corrected, gt = _series(0.98)
    segment = analyze_segment(
        identifier="arc", reset_generation=1,
        command_linear_mps=0.25, command_angular_radps=0.5,
        raw=raw, corrected=corrected, ground_truth=gt,
    )
    goal = {
        "raw_integrated_yaw_rad": [0.0, 0.5, 1.0],
        "ground_truth_relative_yaw_rad": [0.0, 0.49, 0.98],
    }
    result = summarize([segment], goal=goal)
    assert result["verdict"] == "PASS_CANDIDATE"
    assert result["goal_identity_non_degrade_interval"]


def test_phase_metrics_expose_assist_delta_and_stamp_quality():
    phase = []
    for index in range(3):
        phase.append({
            "kind": "loop",
            "simulation_time_after_app_s": 1.0 + index * 0.1,
            "pre_assist_body": {"forward_speed_mps": 0.20, "yaw_rate_radps": 0.40},
            "post_assist_body": {"forward_speed_mps": 0.25, "yaw_rate_radps": 0.45},
            "imu_graph_after_app": {
                "read_imu_sensor_time_s": {"value": 1.0 + index * 0.1, "error": None},
                "publish_imu_timestamp_s": {"value": 1.0 + index * 0.1, "error": None},
            },
            "before_app_monotonic_ns": 10 + index * 10,
            "after_app_monotonic_ns": 11 + index * 10,
            "after_assist_monotonic_ns": 12 + index * 10,
            "before_ground_truth_monotonic_ns": 13 + index * 10,
            "after_ground_truth_monotonic_ns": 14 + index * 10,
        })
    metrics = phase_window_metrics(phase, start_s=1.0, end_s=1.2)
    assert metrics["assist_forward_delta_mps_median"] == pytest.approx(0.05)
    assert metrics["assist_yaw_rate_delta_radps_median"] == pytest.approx(0.05)
    assert metrics["sensor_publish_stamp_delta_s_max_abs"] == 0.0
    assert metrics["publish_imu_stamp_quality"]["duplicate_count"] == 0
    assert metrics["monotonic_order_violation_count"] == 0


def test_command_windows_are_reset_and_target_partitioned():
    phase = []
    for loop, generation, target in (
        (0, 1, [0.0, 0.0]),
        (1, 1, [0.0, -0.5]),
        (2, 1, [0.0, -0.5]),
        (3, 1, [0.0, 0.0]),
        (4, 2, [0.25, 0.5]),
        (5, 2, [0.25, 0.5]),
    ):
        phase.append({
            "kind": "loop", "loop_sequence": loop,
            "reset_generation": generation,
            "simulation_time_after_app_s": float(loop),
            "assist": {"target": target},
        })
    report = {"primitives": [
        {"id": "cw", "reset_receipt": {"generation": 1}},
        {"id": "arc", "reset_receipt": {"generation": 2}},
    ]}
    windows = command_windows(phase, report)
    assert [(item["id"], item["start_s"], item["end_s"]) for item in windows] == [
        ("cw", 1.0, 3.0), ("arc", 4.0, 5.0)
    ]


def test_command_windows_include_exact_external_stationary_10s():
    phase = [
        {
            "kind": "loop",
            "reset_generation": 1,
            "reset_generation_after_ground_truth": 1,
            "simulation_time_after_app_s": float(index),
            "assist": {"target": [0.0, 0.0]},
        }
        for index in range(11)
    ]
    windows = command_windows(phase, {"primitives": []})
    assert windows == [{
        "generation": 1,
        "linear": 0.0,
        "angular": 0.0,
        "start_s": 0.0,
        "end_s": 10.0,
        "id": "stationary_external_10s",
    }]
