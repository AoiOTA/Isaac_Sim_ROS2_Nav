from __future__ import annotations

import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from robot_experiments.imu_regime_analysis import (
    EvidenceError,
    EXPECTED_COMMANDS,
    EXPECTED_PRIMITIVE_IDS,
    EXPECTED_SEGMENT_COUNTS,
    ScalarSample,
    YawSample,
    analyze_segment,
    command_windows,
    goal_identity_non_degrade_interval,
    load_mcap,
    phase_window_metrics,
    summarize,
    run_analysis,
    validate_benchmark_report,
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
    assert config.spawn_pose_name == "flat20_start"
    assert config.stationary_reference is not None
    assert config.stationary_reference.identifier == "stationary_reference"
    assert config.stationary_reference.duration_sec == 10.0
    assert config.stationary_reference.reset_seed == 8609


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


def test_common_grid_uses_overlap_boundaries_without_extrapolation():
    raw = [ScalarSample(index * 0.1, 0.5) for index in range(101)]
    corrected = [ScalarSample(0.02 + index * 0.1, 0.465) for index in range(101)]
    gt = [YawSample(0.04 + index * 0.1, 0.465 * (0.04 + index * 0.1)) for index in range(101)]
    result = analyze_segment(
        identifier="offset", reset_generation=2,
        command_linear_mps=0.1, command_angular_radps=0.5,
        raw=raw, corrected=corrected, ground_truth=gt,
    )
    assert result["status"] == "OK"
    assert result["coverage"]["t0_s"] == pytest.approx(0.04)
    assert result["coverage"]["t1_s"] == pytest.approx(10.0)
    assert result["coverage"]["interpolation"] == "linear_no_extrapolation"
    assert result["k_star"] == pytest.approx(0.93, abs=1e-10)
    assert result["corrected_aligned_rmse_rad"] == pytest.approx(0.0, abs=1e-10)


def test_incomplete_two_window_summary_cannot_confirm_global_constant():
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
    assert result["verdict"] == "AMBIGUOUS"
    assert result["global_scale_intersection"] == []
    assert set(result["bins"]) == {"v=0.00/CCW", "v=0.25/CCW"}


def test_stamp_anomaly_fails_and_missing_data_is_ambiguous():
    raw, corrected, gt = _series(0.93, duplicate=True)
    invalid = analyze_segment(
        identifier="bad_stamp", reset_generation=1,
        command_linear_mps=0.0, command_angular_radps=0.5,
        raw=raw, corrected=corrected, ground_truth=gt,
    )
    assert invalid["status"] == "DATA_INVALID"
    assert summarize([invalid])["verdict"] == "FAIL"
    missing = analyze_segment(
        identifier="missing", reset_generation=2,
        command_linear_mps=0.1, command_angular_radps=-0.5,
        raw=[], corrected=[], ground_truth=[],
    )
    assert missing["status"] == "INSUFFICIENT_DATA"
    assert summarize([missing])["verdict"] == "AMBIGUOUS"


def test_single_window_and_unprovenanced_goal_cannot_promote_candidate():
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
    assert result["verdict"] == "AMBIGUOUS"
    assert result["goal_evidence_issue"]["code"] == "goal_truncated"


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


def _phase_row(index, *, generation=1):
    stamp = 1.0 + index * 0.1
    return {
        "kind": "loop", "reset_generation": generation,
        "reset_generation_after_ground_truth": generation,
        "simulation_time_after_app_s": stamp,
        "pre_assist_body": {"forward_speed_mps": 0.2, "yaw_rate_radps": 0.4},
        "post_assist_body": {"forward_speed_mps": 0.25, "yaw_rate_radps": 0.45},
        "imu_graph_after_app": {
            "read_imu_ang_vel": {"value": [0.0, 0.0, 0.4], "error": None},
            "read_imu_sensor_time_s": {"value": stamp, "error": None},
            "publish_imu_angular_velocity": {"value": [0.0, 0.0, 0.4], "error": None},
            "publish_imu_timestamp_s": {"value": stamp, "error": None},
        },
        "before_app_monotonic_ns": 10 + index * 10,
        "after_app_monotonic_ns": 11 + index * 10,
        "after_assist_monotonic_ns": 12 + index * 10,
        "before_ground_truth_monotonic_ns": 13 + index * 10,
        "after_ground_truth_monotonic_ns": 14 + index * 10,
    }


def test_phase_four_attribute_gate_distinguishes_ambiguous_and_fail():
    phase = [_phase_row(index) for index in range(3)]
    assert phase_window_metrics(phase, start_s=1.0, end_s=1.2, reset_generation=1)["status"] == "OK"
    missing = [dict(row) for row in phase]
    missing[0] = {**missing[0], "imu_graph_after_app": {}}
    assert phase_window_metrics(missing, start_s=1.0, end_s=1.2, reset_generation=1)["status"] == "AMBIGUOUS"
    invalid = [dict(row) for row in phase]
    invalid_graph = dict(invalid[0]["imu_graph_after_app"])
    invalid_graph["read_imu_ang_vel"] = {"value": [0.0, 1.0], "error": None}
    invalid[0] = {**invalid[0], "imu_graph_after_app": invalid_graph}
    assert phase_window_metrics(invalid, start_s=1.0, end_s=1.2, reset_generation=1)["status"] == "FAIL"
    crossing = [dict(row) for row in phase]
    crossing[1] = {**crossing[1], "reset_generation_after_ground_truth": 2}
    assert phase_window_metrics(crossing, start_s=1.0, end_s=1.2, reset_generation=1)["status"] == "FAIL"


def _receipt(seed, generation):
    return {
        "requested_seed": seed, "actual_seed": seed,
        "generation": generation, "pose": "flat20_start",
    }


def _benchmark_report():
    stationary = {
        "id": "stationary_reference", "passed": True, "stopped": False,
        "collision_detected": False, "sample_count": 100, "segments": [],
        "reset_seed": 8609, "requested_duration_sec": 10.0,
        "measured_duration_sec": 10.0, "zero_command_count": 201,
        "final_zero_published": True, "max_odometry_displacement_m": 0.0,
        "reset_receipt": _receipt(8609, 1),
    }
    primitives = []
    for index, (identifier, count, commands) in enumerate(zip(EXPECTED_PRIMITIVE_IDS, EXPECTED_SEGMENT_COUNTS, EXPECTED_COMMANDS)):
        primitives.append({
            "id": identifier, "passed": True, "stopped": False,
            "collision_detected": False, "sample_count": 100,
            "segments": [
                {
                    "segment_index": segment_index,
                    "command_linear_mps": command[0],
                    "command_angular_radps": command[1],
                    "steady_sample_count": 10,
                }
                for segment_index, command in enumerate(commands)
            ],
            "reset_seed": 8610 + index, "final_zero_published": True,
            "reset_receipt": _receipt(8610 + index, index + 2),
        })
    receipts = [stationary["reset_receipt"], *[item["reset_receipt"] for item in primitives]]
    return {
        "passed": True, "stopped": False, "collision_detected": False,
        "sample_count": 1000, "segment_count": 11,
        "primitive_count": 9, "passed_primitive_count": 9,
        "final_zero_published": True, "spawn_pose_name": "flat20_start",
        "stationary_reference": stationary, "primitives": primitives,
        "reset_receipts": receipts,
    }


def test_command_windows_require_stationary_and_three_s_segments():
    report = _benchmark_report()
    rows = []
    sequence = 0

    def add(generation, stamp, target):
        nonlocal sequence
        rows.append({
            "kind": "loop", "loop_sequence": sequence,
            "reset_generation": generation,
            "reset_generation_after_ground_truth": generation,
            "simulation_time_after_app_s": stamp,
            "assist": {"target": list(target)},
        })
        sequence += 1

    for stamp in (0.0, 5.0, 10.0):
        add(1, stamp, (0.0, 0.0))
    for generation, commands in zip(range(2, 10), EXPECTED_COMMANDS[:8]):
        stamp = generation * 20.0
        add(generation, stamp, commands[0])
        add(generation, stamp + 1.0, commands[0])
        add(generation, stamp + 1.1, (0.0, 0.0))
    stamp = 200.0
    for index, target in enumerate(((0.25, 0.45), (0.25, -0.45), (0.25, 0.45))):
        add(10, stamp + index * 2.0, target)
        add(10, stamp + index * 2.0 + 1.0, target)
    add(10, stamp + 6.1, (0.0, 0.0))
    windows = command_windows(rows, report)
    assert len(windows) == 12
    assert windows[0]["id"] == "stationary_reference"
    assert [item["id"] for item in windows[-3:]] == ["s_route[0]", "s_route[1]", "s_route[2]"]


def test_report_contract_rejects_extra_duplicate_stop_collision_and_receipt():
    report = _benchmark_report()
    assert validate_benchmark_report(report) is report
    for mutate, code in (
        (lambda value: value["primitives"].append(dict(value["primitives"][0])), "benchmark_extra"),
        (lambda value: value["primitives"].__setitem__(1, dict(value["primitives"][0])), "benchmark_duplicate"),
        (lambda value: value["primitives"][0].__setitem__("stopped", True), "benchmark_explicit_failure"),
        (lambda value: value["primitives"][0].__setitem__("collision_detected", True), "benchmark_explicit_failure"),
        (lambda value: value["primitives"][0]["reset_receipt"].__setitem__("actual_seed", 9), "receipt_mismatch"),
    ):
        import copy
        candidate = copy.deepcopy(report)
        mutate(candidate)
        with pytest.raises(EvidenceError) as raised:
            validate_benchmark_report(candidate)
        assert raised.value.code == code


def test_report_contract_classifies_truncation_and_zero_samples_ambiguous():
    report = _benchmark_report()
    report["primitives"] = report["primitives"][:-1]
    with pytest.raises(EvidenceError) as raised:
        validate_benchmark_report(report)
    assert raised.value.verdict == "AMBIGUOUS"
    report = _benchmark_report()
    report["primitives"][0]["sample_count"] = 0
    with pytest.raises(EvidenceError) as raised:
        validate_benchmark_report(report)
    assert raised.value.verdict == "AMBIGUOUS"


def test_goal_contract_rejects_nonfinite_and_requires_source():
    goal = {
        "schema_version": 1, "source": "goal_mcap_derived",
        "source_mcap": "/tmp/goal.mcap", "reset_receipt": {"generation": 1, "pose": "goal"},
        "outcome": "SUCCEEDED", "collision_detected": False,
        "raw_integrated_yaw_rad": [0.0, 0.5, 1.0],
        "ground_truth_relative_yaw_rad": [0.0, 0.49, 0.98],
    }
    assert goal_identity_non_degrade_interval(goal)
    goal["raw_integrated_yaw_rad"][1] = math.nan
    with pytest.raises(EvidenceError) as raised:
        goal_identity_non_degrade_interval(goal)
    assert raised.value.verdict == "FAIL"


def _install_fake_mcap(monkeypatch, topic_types, records=()):
    class Reader:
        def __init__(self):
            self.records = list(records)

        def open(self, storage, converter):
            self.storage = storage

        def get_all_topics_and_types(self):
            return [SimpleNamespace(name=name, type=value) for name, value in topic_types.items()]

        def has_next(self):
            return bool(self.records)

        def read_next(self):
            return self.records.pop(0)

    rosbag = SimpleNamespace(
        SequentialReader=Reader,
        StorageOptions=lambda **kwargs: kwargs,
        ConverterOptions=lambda *args: args,
    )
    serialization = SimpleNamespace(deserialize_message=lambda payload, _kind: payload)
    utilities = SimpleNamespace(get_message=lambda value: value)
    monkeypatch.setitem(sys.modules, "rosbag2_py", rosbag)
    monkeypatch.setitem(sys.modules, "rclpy.serialization", serialization)
    monkeypatch.setitem(sys.modules, "rosidl_runtime_py.utilities", utilities)


def test_mcap_exact_topic_types_fail_closed(monkeypatch, tmp_path):
    _install_fake_mcap(monkeypatch, {
        "/imu/data_raw": "sensor_msgs/msg/Imu",
        "/imu/data": "geometry_msgs/msg/Vector3",
        "/ground_truth/odom": "nav_msgs/msg/Odometry",
    })
    with pytest.raises(EvidenceError) as raised:
        load_mcap(tmp_path / "bag")
    assert raised.value.verdict == "FAIL"
    assert raised.value.code == "mcap_topic_type"


def test_structured_output_survives_truncated_report(tmp_path):
    benchmark = tmp_path / "benchmark.json"
    benchmark.write_text('{"passed": true', encoding="utf-8")
    result = run_analysis(
        mcap=tmp_path / "bag",
        phase_jsonl=tmp_path / "phase.jsonl",
        benchmark_report=benchmark,
    )
    assert result["verdict"] == "AMBIGUOUS"
    assert result["evidence_errors"][0]["code"] == "benchmark_unreadable"
    assert result["segments"] == []
