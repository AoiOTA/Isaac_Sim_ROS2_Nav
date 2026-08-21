from __future__ import annotations

import json
import math
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import pytest
import yaml
import robot_experiments.imu_regime_analysis as imu_analysis

from robot_experiments.imu_regime_analysis import (
    EvidenceError,
    EXPECTED_COMMANDS,
    EXPECTED_PRIMITIVE_IDS,
    EXPECTED_SEGMENT_COUNTS,
    CommandSample,
    McapStreams,
    ResetSample,
    ScalarSample,
    YawSample,
    analyze_segment,
    command_windows,
    goal_identity_non_degrade_interval,
    load_goal_mcap,
    load_mcap,
    phase_window_metrics,
    summarize,
    run_analysis,
    resolve_diagnostic_resources,
    validate_benchmark_report,
    validate_phase_trace,
)
from robot_experiments.motion_benchmark import load_motion_config


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / "config/v6_imu_regime_diagnostic.yaml"
SPAWN = PACKAGE_ROOT.parents[2] / "isaac_sim/configs/environments/v6_calibration_flat_20m.spawn.yaml"
FEATURES = PACKAGE_ROOT.parents[2] / "isaac_sim/configs/experiments/v6_calibration_grid_features.yaml"


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


@pytest.mark.parametrize("value", ["bad", None, math.nan, math.inf, True])
def test_phase_malformed_simulation_time_is_structured_evidence_error(value):
    row = _phase_row(0)
    row["simulation_time_after_app_s"] = value
    with pytest.raises(EvidenceError) as raised:
        phase_window_metrics([row], start_s=0.0, end_s=2.0)
    assert raised.value.verdict == "FAIL"
    assert raised.value.code == "phase_simulation_time_invalid"


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
    durations = ((12.566,), (12.566,), *((4.0,) for _ in range(6)), (2.5, 5.0, 2.5))
    for index, (identifier, count, commands, primitive_durations) in enumerate(zip(EXPECTED_PRIMITIVE_IDS, EXPECTED_SEGMENT_COUNTS, EXPECTED_COMMANDS, durations)):
        primitives.append({
            "id": identifier, "passed": True, "stopped": False,
            "collision_detected": False, "sample_count": 100,
            "segments": [
                {
                    "segment_index": segment_index,
                    "duration_sec": primitive_durations[segment_index],
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
        "command_rate_hz": 20.0,
        "thresholds": {
            "linear_mae_mps": 0.06, "angular_mae_radps": 0.12,
            "radius_relative_error_percent": 20.0, "tracking_fraction": 0.85,
            "transition_latency_sec": 0.45, "overshoot_ratio": 1.25,
            "wrong_direction_fraction": 0.05,
        },
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
    durations = (12.566, 12.566, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0)
    for generation, commands, duration in zip(range(2, 10), EXPECTED_COMMANDS[:8], durations):
        stamp = generation * 20.0
        add(generation, stamp, commands[0])
        add(generation, stamp + duration, commands[0])
        add(generation, stamp + duration + 0.1, (0.0, 0.0))
    stamp = 200.0
    for index, (target, duration) in enumerate(zip(((0.25, 0.45), (0.25, -0.45), (0.25, 0.45)), (2.5, 5.0, 2.5))):
        add(10, stamp, target)
        add(10, stamp + duration, target)
        stamp += duration + 0.1
    add(10, stamp, (0.0, 0.0))
    windows = command_windows(rows, report)
    assert len(windows) == 12
    assert windows[0]["id"] == "stationary_reference"
    assert [item["id"] for item in windows[-3:]] == ["s_route[0]", "s_route[1]", "s_route[2]"]


def _session_a_command_streams(report):
    topics = ("/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel", "/cmd_vel_sim")
    streams = {topic: [] for topic in topics}
    streams["/simulation/reset_event"] = []
    entries = [report["stationary_reference"], *report["primitives"]]
    commands = [((0.0, 0.0),), *EXPECTED_COMMANDS]
    durations = [(10.0,), (12.566,), (12.566,), *((4.0,) for _ in range(6)), (2.5, 5.0, 2.5)]
    phase = []
    for epoch_index, (entry, entry_commands, entry_durations) in enumerate(zip(entries, commands, durations)):
        base = epoch_index * 30.0
        streams["/simulation/reset_event"].append(ResetSample(base, base))
        generation = entry["reset_receipt"]["generation"]
        phase.append({"kind": "loop", "reset_generation": generation, "reset_generation_after_ground_truth": generation})
        start = base + 1.0
        for topic in topics:
            for offset in (0.1, 0.3, 0.5, 0.7, 0.9):
                stamp = base + offset
                streams[topic].append(CommandSample(stamp, 0.0, 0.0, stamp))
        for command, duration in zip(entry_commands, entry_durations):
            count = math.ceil(duration / 0.05)
            for sample_index in range(count):
                stamp = start + sample_index * 0.05
                for topic in topics:
                    streams[topic].append(CommandSample(stamp, command[0], command[1], stamp))
            start += duration
        for sample_index in range(18):
            stamp = start + sample_index * 0.05
            for topic in topics:
                streams[topic].append(CommandSample(stamp, 0.0, 0.0, stamp))
    return McapStreams(streams, provenance={}), phase


def _schema2_report():
    import copy

    report = copy.deepcopy(_benchmark_report())
    report["schema_version"] = 2
    report["final_settle_sec"] = 0.8
    entries = [report["stationary_reference"], *report["primitives"]]
    commands = [((0.0, 0.0),), *EXPECTED_COMMANDS]
    durations = [
        (10.0,), (12.566,), (12.566,),
        *((4.0,) for _ in range(6)), (2.5, 5.0, 2.5),
    ]
    for epoch_index, (entry, entry_commands, entry_durations) in enumerate(
        zip(entries, commands, durations)
    ):
        cursor = epoch_index * 30.0 + 1.0
        entry["segment_schedule"] = []
        for segment_index, (command, duration) in enumerate(
            zip(entry_commands, entry_durations)
        ):
            entry["segment_schedule"].append({
                "segment_index": segment_index,
                "start_sim_s": cursor,
                "end_sim_s": cursor + duration,
                "expected_duration_s": duration,
                "command_linear_mps": command[0],
                "command_angular_radps": command[1],
                "intent_publish_count": math.ceil(duration * 20.0),
                "completion": "COMPLETED",
                "truncated": False,
            })
            cursor += duration
        entry["final_zero_publish_receipt"] = {
            "first_sim_s": cursor,
            "last_sim_s": cursor + 0.8,
            "publish_count": 16,
            "requested_duration_s": 0.8,
            "command_linear_mps": 0.0,
            "command_angular_radps": 0.0,
        }
    return report


def test_schema2_stationary_and_moving_windows_have_complete_chain_receipts():
    report = _schema2_report()
    streams, phase = _session_a_command_streams(report)
    assert validate_benchmark_report(report) is report
    windows = command_windows(phase, report, streams)
    assert len(windows) == 12
    stationary = windows[0]
    assert stationary["id"] == "stationary_reference"
    assert stationary["schedule_receipt"]["intent_publish_count"] == 200
    assert all(
        evidence["all_zero"]
        and evidence["start_boundary_gap_s"] <= 0.25
        and evidence["end_boundary_gap_s"] <= 0.25
        for evidence in stationary["command_chain_coverage"].values()
    )
    assert all(
        evidence["all_zero"]
        for evidence in stationary["final_zero"]["chain_coverage"].values()
    )


@pytest.mark.parametrize("boundary", ["start", "end"])
def test_schema2_moving_window_rejects_start_or_end_dropout(boundary):
    report = _schema2_report()
    streams, phase = _session_a_command_streams(report)
    schedule = report["primitives"][0]["segment_schedule"][0]
    start = schedule["start_sim_s"]
    end = schedule["end_sim_s"]
    if boundary == "start":
        streams["/cmd_vel_sim"] = [
            sample for sample in streams["/cmd_vel_sim"]
            if not (start <= sample.stamp_s < start + 0.30)
        ]
    else:
        streams["/cmd_vel_sim"] = [
            sample for sample in streams["/cmd_vel_sim"]
            if not (end - 0.30 < sample.stamp_s <= end)
        ]
    with pytest.raises(EvidenceError) as raised:
        command_windows(phase, report, streams)
    assert raised.value.verdict == "AMBIGUOUS"
    assert raised.value.code in {"command_boundary_coverage", "command_stream_gap"}


def test_schema2_hold_rejects_internal_dropout():
    report = _schema2_report()
    streams, phase = _session_a_command_streams(report)
    streams["/cmd_vel"] = [
        sample for sample in streams["/cmd_vel"]
        if not (0.25 < sample.stamp_s < 0.65)
    ]
    with pytest.raises(EvidenceError) as raised:
        command_windows(phase, report, streams)
    assert raised.value.verdict == "AMBIGUOUS"
    assert raised.value.code == "command_stream_gap"


@pytest.mark.parametrize(
    "topic", ["/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel", "/cmd_vel_sim"]
)
def test_schema2_stationary_rejects_nonzero_on_every_command_stage(topic):
    report = _schema2_report()
    streams, phase = _session_a_command_streams(report)
    samples = list(streams[topic])
    index = next(
        index for index, sample in enumerate(samples)
        if 2.0 <= sample.stamp_s <= 2.1
    )
    sample = samples[index]
    samples[index] = CommandSample(
        sample.stamp_s, 0.01, 0.0, sample.recorded_s
    )
    streams[topic] = samples
    with pytest.raises(EvidenceError) as raised:
        command_windows(phase, report, streams)
    assert raised.value.verdict == "FAIL"
    assert raised.value.code in {"intent_schedule_value", "command_zero_leak"}


def test_mcap_intent_restores_one_cw_and_three_s_segments_despite_downstream_plateaus():
    report = _benchmark_report()
    streams, phase = _session_a_command_streams(report)
    # Downstream ramp/plateau values are coverage only and cannot split intent.
    for topic in ("/cmd_vel_smoothed", "/cmd_vel", "/cmd_vel_sim"):
        samples = list(streams[topic])
        for index, sample in enumerate(samples):
            if sample.angular_radps == -0.5 and index % 3 == 0:
                samples[index] = CommandSample(sample.stamp_s, sample.linear_mps / 2.0, -0.35, sample.recorded_s)
        streams[topic] = samples
    windows = command_windows(phase, report, streams)
    assert sum(item["id"] == "cw_360" for item in windows) == 1
    assert [item["id"] for item in windows[-3:]] == ["s_route[0]", "s_route[1]", "s_route[2]"]


@pytest.mark.parametrize("mutation,code", [
    ("intent_missing", "intent_missing"),
    ("intent_sign", "intent_missing"),
    ("intent_duration", "intent_missing"),
    ("final_leak", "final_nav_zero"),
    ("hidden_reset", "phase_hidden_reset"),
])
def test_session_a_binding_rejects_missing_wrong_short_leak_and_hidden_reset(mutation, code):
    report = _benchmark_report()
    streams, phase = _session_a_command_streams(report)
    generation = report["primitives"][0]["reset_receipt"]["generation"]
    reset = streams["/simulation/reset_event"][1]
    next_reset = streams["/simulation/reset_event"][2]
    nav = streams["/cmd_vel_nav"]
    epoch = [sample for sample in nav if reset.recorded_s <= sample.recorded_s < next_reset.recorded_s]
    if mutation == "intent_missing":
        streams["/cmd_vel_nav"] = [sample for sample in nav if sample not in epoch or sample.angular_radps != -0.5]
    elif mutation == "intent_sign":
        streams["/cmd_vel_nav"] = [
            CommandSample(sample.stamp_s, sample.linear_mps, 0.5, sample.recorded_s)
            if sample in epoch and sample.angular_radps == -0.5 else sample for sample in nav
        ]
    elif mutation == "intent_duration":
        matching = [sample for sample in epoch if sample.angular_radps == -0.5]
        streams["/cmd_vel_nav"] = [sample for sample in nav if sample not in matching[len(matching) // 4:]]
    elif mutation == "final_leak":
        last = max(sample.stamp_s for sample in epoch)
        streams["/cmd_vel_nav"].append(CommandSample(last, 0.1, 0.1, last))
        streams["/cmd_vel_nav"].sort(key=lambda sample: sample.recorded_s)
    else:
        phase.append({"kind": "loop", "reset_generation": 99, "reset_generation_after_ground_truth": 99})
    with pytest.raises(EvidenceError) as raised:
        command_windows(phase, report, streams)
    assert raised.value.code == code


def test_phase_crossing_is_rejected_and_final_sim_zero_defects_are_classified():
    crossing = [_phase_row(0, generation=3)]
    crossing[0]["reset_generation_after_ground_truth"] = 4
    assert phase_window_metrics(crossing, start_s=1.0, end_s=1.0, reset_generation=3)["status"] == "FAIL"

    report = _benchmark_report()
    streams, phase = _session_a_command_streams(report)
    reset = streams["/simulation/reset_event"][1]
    next_reset = streams["/simulation/reset_event"][2]
    sim = streams["/cmd_vel_sim"]
    epoch = [sample for sample in sim if reset.recorded_s <= sample.recorded_s < next_reset.recorded_s]
    zeros = [sample for sample in epoch if sample.angular_radps == 0.0 and sample.stamp_s > reset.stamp_s + 13.0]
    streams["/cmd_vel_sim"] = [sample for sample in sim if sample not in zeros[2:-2]]
    windows = command_windows(phase, report, streams)
    cw = next(item for item in windows if item["id"] == "cw_360")
    assert any(item["code"] in {"final_sim_zero_short", "final_sim_zero_gap"} for item in cw["capture_issues"])


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


def test_schema2_schedule_receipts_are_required_and_truncation_fails_closed():
    import copy

    report = _benchmark_report()
    report["schema_version"] = 2
    report["final_settle_sec"] = 0.8
    entries = [report["stationary_reference"], *report["primitives"]]
    commands = [((0.0, 0.0),), *EXPECTED_COMMANDS]
    durations = [(10.0,), (12.566,), (12.566,), *((4.0,) for _ in range(6)), (2.5, 5.0, 2.5)]
    for epoch_index, (entry, entry_commands, entry_durations) in enumerate(zip(entries, commands, durations)):
        cursor = epoch_index * 30.0 + 1.0
        entry["segment_schedule"] = []
        for segment_index, (command, duration) in enumerate(zip(entry_commands, entry_durations)):
            entry["segment_schedule"].append({
                "segment_index": segment_index,
                "start_sim_s": cursor,
                "end_sim_s": cursor + duration,
                "expected_duration_s": duration,
                "command_linear_mps": command[0],
                "command_angular_radps": command[1],
                "intent_publish_count": math.ceil(duration * 20.0),
                "completion": "COMPLETED",
                "truncated": False,
            })
            cursor += duration
        entry["final_zero_publish_receipt"] = {
            "first_sim_s": cursor, "last_sim_s": cursor + 0.8,
            "publish_count": 16,
            "requested_duration_s": 0.8,
            "command_linear_mps": 0.0,
            "command_angular_radps": 0.0,
        }
    assert validate_benchmark_report(report) is report
    stopped = copy.deepcopy(report)
    stopped["primitives"][0]["segment_schedule"][0]["completion"] = "TRUNCATED"
    stopped["primitives"][0]["segment_schedule"][0]["truncated"] = True
    with pytest.raises(EvidenceError) as raised:
        validate_benchmark_report(stopped)
    assert raised.value.code == "benchmark_schedule_contract"
    short_settle = copy.deepcopy(report)
    short_settle["stationary_reference"]["final_zero_publish_receipt"][
        "last_sim_s"
    ] -= 0.4
    with pytest.raises(EvidenceError) as raised:
        validate_benchmark_report(short_settle)
    assert raised.value.code == "benchmark_zero_receipt"
    report = _benchmark_report()
    report["primitives"][0]["sample_count"] = 0
    with pytest.raises(EvidenceError) as raised:
        validate_benchmark_report(report)
    assert raised.value.verdict == "AMBIGUOUS"


def test_goal_contract_rejects_nonfinite_and_requires_source():
    goal = {
        "schema_version": 1, "source": "goal_mcap_derived",
        "source_mcap": "/tmp/goal.mcap", "reset_receipt": {
            "requested_seed": 7, "actual_seed": 7, "generation": 1, "pose": "goal"
        },
        "outcome": "SUCCEEDED", "collision_detected": False,
        "bag_verified": True, "goal_window": {"start_s": 1.0, "end_s": 2.0},
        "attempt_provenance": {
            "terminal_count": 1, "terminal_values": [True],
            "terminal_timestamps_s": [2.1],
        },
        "stream_coverage": {
            "maximum_allowed_gap_s": 0.25,
            "maximum_gap_s": {"raw": 0.1, "ground_truth": 0.1},
        },
        "raw_integrated_yaw_rad": [0.0, 0.5, 1.0],
        "ground_truth_relative_yaw_rad": [0.0, 0.49, 0.98],
    }
    assert goal_identity_non_degrade_interval(goal)
    goal["raw_integrated_yaw_rad"][1] = math.nan
    with pytest.raises(EvidenceError) as raised:
        goal_identity_non_degrade_interval(goal)
    assert raised.value.verdict == "FAIL"


def _install_fake_mcap(monkeypatch, topic_types, records=()):
    class ReadOrderSortBy:
        File = "file"

    class ReadOrder:
        def __init__(self, sort_by=None, reverse=False):
            self.sort_by = sort_by
            self.reverse = reverse

    class Reader:
        def __init__(self):
            self.records = list(records)

        def open(self, storage, converter):
            self.storage = storage

        def get_all_topics_and_types(self):
            return [SimpleNamespace(name=name, type=value) for name, value in topic_types.items()]

        def set_read_order(self, order):
            return order.sort_by == ReadOrderSortBy.File and order.reverse is False

        def has_next(self):
            return bool(self.records)

        def read_next(self):
            return self.records.pop(0)

    rosbag = SimpleNamespace(
        SequentialReader=Reader,
        ReadOrder=ReadOrder,
        ReadOrderSortBy=ReadOrderSortBy,
        StorageOptions=lambda **kwargs: kwargs,
        ConverterOptions=lambda *args: args,
    )
    serialization = SimpleNamespace(deserialize_message=lambda payload, _kind: payload)
    utilities = SimpleNamespace(get_message=lambda value: value)
    monkeypatch.setitem(sys.modules, "rosbag2_py", rosbag)
    monkeypatch.setitem(sys.modules, "rclpy.serialization", serialization)
    monkeypatch.setitem(sys.modules, "rosidl_runtime_py.utilities", utilities)


def _diagnostic_mcap_records(clock_values=(1.0, 2.0, 3.0)):
    records = []
    recorded_ns = 1_000_000_000

    def stamp(value):
        sec = int(value)
        return SimpleNamespace(
            sec=sec, nanosec=int(round((value - sec) * 1_000_000_000))
        )

    for index, clock_value in enumerate(clock_values):
        header_stamp = stamp(float(index + 1))
        records.append((
            "/clock", SimpleNamespace(clock=stamp(clock_value)), recorded_ns
        ))
        recorded_ns += 1
        if index == 0:
            records.append(("/simulation/reset_event", SimpleNamespace(), recorded_ns))
            recorded_ns += 1
        imu = SimpleNamespace(
            header=SimpleNamespace(stamp=header_stamp),
            angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=0.1),
        )
        odom = SimpleNamespace(
            header=SimpleNamespace(stamp=header_stamp),
            pose=SimpleNamespace(pose=SimpleNamespace(orientation=SimpleNamespace(
                x=0.0, y=0.0, z=0.0, w=1.0
            ))),
        )
        twist = SimpleNamespace(
            linear=SimpleNamespace(x=0.0),
            angular=SimpleNamespace(z=0.0),
        )
        for topic, message in (
            ("/imu/data_raw", imu), ("/imu/data", imu),
            ("/ground_truth/odom", odom),
            ("/cmd_vel_nav", twist), ("/cmd_vel_smoothed", twist),
            ("/cmd_vel", twist), ("/cmd_vel_sim", twist),
        ):
            records.append((topic, message, recorded_ns))
            recorded_ns += 1
    return records


def test_mcap_clock_file_order_is_strict_and_command_duplicates_are_explicit(
    monkeypatch, tmp_path
):
    records = _diagnostic_mcap_records()
    duplicate = next(record for record in records if record[0] == "/cmd_vel")
    records.insert(records.index(duplicate) + 1, duplicate)
    _install_fake_mcap(monkeypatch, imu_analysis.EXPECTED_TOPIC_TYPES, records)
    streams = load_mcap(tmp_path / "bag")
    assert streams.provenance["clock_order_contract"] == (
        "strict_positive_increasing_file_order"
    )
    assert streams.provenance["command_clock_duplicate_counts"]["/cmd_vel"] == 1


def test_mcap_rejects_clock_backward_in_file_order(monkeypatch, tmp_path):
    _install_fake_mcap(
        monkeypatch, imu_analysis.EXPECTED_TOPIC_TYPES,
        _diagnostic_mcap_records((1.0, 2.0, 1.5)),
    )
    with pytest.raises(EvidenceError) as raised:
        load_mcap(tmp_path / "bag")
    assert raised.value.verdict == "FAIL"
    assert raised.value.code == "mcap_clock_order"


def test_mcap_rejects_command_receive_stamp_backward(monkeypatch, tmp_path):
    records = _diagnostic_mcap_records()
    indices = [index for index, record in enumerate(records) if record[0] == "/cmd_vel"]
    first = records[indices[0]]
    second = records[indices[1]]
    records[indices[1]] = (second[0], second[1], first[2] - 1)
    _install_fake_mcap(monkeypatch, imu_analysis.EXPECTED_TOPIC_TYPES, records)
    with pytest.raises(EvidenceError) as raised:
        load_mcap(tmp_path / "bag")
    assert raised.value.verdict == "FAIL"
    assert raised.value.code == "mcap_command_file_order"


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


def test_mcap_without_confirmed_file_order_is_ambiguous(monkeypatch, tmp_path):
    _install_fake_mcap(monkeypatch, {
        "/imu/data_raw": "sensor_msgs/msg/Imu",
        "/imu/data": "sensor_msgs/msg/Imu",
        "/ground_truth/odom": "nav_msgs/msg/Odometry",
    })
    del sys.modules["rosbag2_py"].ReadOrderSortBy
    with pytest.raises(EvidenceError) as raised:
        load_mcap(tmp_path / "bag")
    assert raised.value.verdict == "AMBIGUOUS"
    assert raised.value.code == "mcap_file_order_unavailable"


def test_structured_output_survives_truncated_report(tmp_path):
    benchmark = tmp_path / "benchmark.json"
    benchmark.write_text('{"passed": true', encoding="utf-8")
    result = run_analysis(
        mcap=tmp_path / "bag",
        phase_jsonl=tmp_path / "phase.jsonl",
        benchmark_report=benchmark,
        config_path=CONFIG,
        spawn_poses_path=SPAWN,
        obstacle_config_path=FEATURES,
    )
    assert result["verdict"] == "AMBIGUOUS"
    assert result["evidence_errors"][0]["code"] == "benchmark_unreadable"
    assert result["segments"] == []


def test_performance_failure_keeps_segment_metrics_but_never_authorizes_scale(monkeypatch, tmp_path):
    report = _benchmark_report()
    report["passed"] = False
    report["primitives"][2]["passed"] = False
    report["passed_primitive_count"] = 8
    benchmark = tmp_path / "benchmark.json"
    benchmark.write_text(json.dumps(report), encoding="utf-8")
    phase_path = tmp_path / "phase.jsonl"
    phase_path.write_text('{}\n', encoding="utf-8")
    raw, corrected, gt = _series(0.93)
    streams = McapStreams({
        "/imu/data_raw": raw,
        "/imu/data": corrected,
        "/ground_truth/odom": gt,
    }, provenance={})
    identifiers = [
        "stationary_reference", *EXPECTED_PRIMITIVE_IDS[:-1],
        "s_route[0]", "s_route[1]", "s_route[2]",
    ]
    commands = [
        (0.0, 0.0), *[item[0] for item in EXPECTED_COMMANDS[:-1]],
        *EXPECTED_COMMANDS[-1],
    ]
    windows = [
        {
            "id": identifier, "generation": index + 1,
            "linear": command[0], "angular": command[1],
            "start_s": 0.0, "end_s": 10.0,
            "expected_duration_s": 10.0,
            "observed_command_duration_s": 10.0,
            "duration_tolerance_s": 0.25,
            "capture_issues": [],
        }
        for index, (identifier, command) in enumerate(zip(identifiers, commands))
    ]
    monkeypatch.setattr(imu_analysis, "validate_phase_trace", lambda phase, resources: {})
    monkeypatch.setattr(imu_analysis, "load_mcap", lambda path: streams)
    monkeypatch.setattr(imu_analysis, "command_windows", lambda phase, report, streams: windows)
    monkeypatch.setattr(imu_analysis, "phase_window_metrics", lambda *args, **kwargs: {"status": "OK"})
    result = run_analysis(
        mcap=tmp_path / "bag",
        phase_jsonl=phase_path,
        benchmark_report=benchmark,
        config_path=CONFIG,
        spawn_poses_path=SPAWN,
        obstacle_config_path=FEATURES,
    )
    assert result["performance_status"] == "FAIL"
    assert result["verdict"] == "FAIL"
    assert result["scale_selection_authorized"] is False
    assert len(result["segments"]) == 12
    assert all(item["k_star"] is not None for item in result["segments"])
    assert all(item["raw_aligned_rmse_rad"] is not None for item in result["segments"])
    assert all(item["raw_aligned_p95_rad"] is not None for item in result["segments"])
    assert all(item["scale_interval_le_5deg"] for item in result["segments"])


def test_schema1_retro_metrics_are_preserved_but_scale_is_never_authorized(
    monkeypatch, tmp_path
):
    report = _benchmark_report()
    benchmark = tmp_path / "benchmark.json"
    benchmark.write_text(json.dumps(report), encoding="utf-8")
    phase_path = tmp_path / "phase.jsonl"
    phase_path.write_text('{}\n', encoding="utf-8")
    goal_metadata = tmp_path / "goal.json"
    goal_metadata.write_text('{}\n', encoding="utf-8")
    raw, corrected, gt = _series(0.93)
    streams = McapStreams({
        "/imu/data_raw": raw,
        "/imu/data": corrected,
        "/ground_truth/odom": gt,
    }, provenance={})
    identifiers = [
        "stationary_reference", *EXPECTED_PRIMITIVE_IDS[:-1],
        "s_route[0]", "s_route[1]", "s_route[2]",
    ]
    commands = [
        (0.0, 0.0), *[item[0] for item in EXPECTED_COMMANDS[:-1]],
        *EXPECTED_COMMANDS[-1],
    ]
    windows = [{
        "id": identifier, "generation": index + 1,
        "linear": command[0], "angular": command[1],
        "start_s": 0.0, "end_s": 10.0,
        "expected_duration_s": 10.0,
        "observed_command_duration_s": 10.0,
        "duration_tolerance_s": 0.25,
        "capture_issues": [],
    } for index, (identifier, command) in enumerate(zip(identifiers, commands))]
    monkeypatch.setattr(imu_analysis, "validate_phase_trace", lambda phase, resources: {})
    monkeypatch.setattr(imu_analysis, "load_mcap", lambda path: streams)
    monkeypatch.setattr(imu_analysis, "command_windows", lambda phase, report, streams: windows)
    monkeypatch.setattr(imu_analysis, "phase_window_metrics", lambda *args, **kwargs: {"status": "OK"})
    monkeypatch.setattr(imu_analysis, "load_goal_mcap", lambda *args, **kwargs: {})

    def candidate_summary(results, **_kwargs):
        return {
            "verdict": "PASS_CANDIDATE",
            "segments": list(results),
            "bins": {},
            "goal_identity_non_degrade_interval": [[0.9, 1.0]],
            "global_scale_intersection": [[0.92, 0.94]],
        }

    monkeypatch.setattr(imu_analysis, "summarize", candidate_summary)
    result = run_analysis(
        mcap=tmp_path / "bag",
        phase_jsonl=phase_path,
        benchmark_report=benchmark,
        goal_evaluator=goal_metadata,
        goal_mcap=tmp_path / "goal_bag",
        config_path=CONFIG,
        spawn_poses_path=SPAWN,
        obstacle_config_path=FEATURES,
    )
    assert len(result["segments"]) == 12
    assert all(item["k_star"] is not None for item in result["segments"])
    assert result["capture_contract_status"] == "AMBIGUOUS"
    assert result["verdict"] == "AMBIGUOUS"
    assert result["scale_selection_authorized"] is False
    assert any(
        issue["code"] == "benchmark_schema_v1_capture_ambiguity"
        for issue in result["evidence_errors"]
    )


def _complete_phase_rows(duration_override=None, *, stationary_duration=10.0):
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

    add(1, 0.0, (0.0, 0.0))
    add(1, stationary_duration, (0.0, 0.0))
    cursor = 20.0
    durations = ((12.566,), (12.566,), *((4.0,) for _ in range(6)), (2.5, 5.0, 2.5))
    for primitive_index, (generation, commands, primitive_durations) in enumerate(
        zip(range(2, 11), EXPECTED_COMMANDS, durations)
    ):
        for segment_index, (command, duration) in enumerate(zip(commands, primitive_durations)):
            actual = (
                duration_override
                if duration_override == (primitive_index, segment_index, "short")
                else duration
            )
            if actual == (primitive_index, segment_index, "short"):
                raise AssertionError("invalid test override")
            if duration_override == (primitive_index, segment_index):
                actual = 0.6 if duration <= 4.0 else 1.0
            add(generation, cursor, command)
            add(generation, cursor + actual, command)
            cursor += actual + 0.1
        add(generation, cursor, (0.0, 0.0))
        cursor += 2.0
    return rows


@pytest.mark.parametrize(
    "primitive_index,segment_index",
    [
        (primitive_index, segment_index)
        for primitive_index, count in enumerate(EXPECTED_SEGMENT_COUNTS)
        for segment_index in range(count)
    ],
)
def test_every_short_primitive_window_is_never_candidate(primitive_index, segment_index):
    with pytest.raises(EvidenceError) as raised:
        command_windows(
            _complete_phase_rows((primitive_index, segment_index)),
            _benchmark_report(),
        )
    assert raised.value.verdict == "AMBIGUOUS"
    assert raised.value.code == "primitive_phase_short"


def test_short_stationary_window_is_ambiguous():
    with pytest.raises(EvidenceError) as raised:
        command_windows(
            _complete_phase_rows(stationary_duration=1.0),
            _benchmark_report(),
        )
    assert raised.value.verdict == "AMBIGUOUS"
    assert raised.value.code == "stationary_phase_window"


def test_report_duration_threshold_and_generation_contracts_fail_closed():
    import copy

    report = _benchmark_report()
    report["primitives"][0]["segments"][0]["duration_sec"] = 0.6
    with pytest.raises(EvidenceError) as raised:
        validate_benchmark_report(report)
    assert raised.value.code == "benchmark_segment_contract"

    report = _benchmark_report()
    report["thresholds"]["angular_mae_radps"] = 0.2
    with pytest.raises(EvidenceError) as raised:
        validate_benchmark_report(report)
    assert raised.value.code == "benchmark_thresholds"

    for field, value in (("reset_seed", 8610.0), ("reset_seed", True)):
        report = _benchmark_report()
        report["primitives"][0][field] = value
        with pytest.raises(EvidenceError) as raised:
            validate_benchmark_report(report)
        assert raised.value.code == "benchmark_seed"

    report = _benchmark_report()
    report["primitives"][4]["reset_receipt"]["generation"] += 1
    report["reset_receipts"] = [
        report["stationary_reference"]["reset_receipt"],
        *[item["reset_receipt"] for item in report["primitives"]],
    ]
    with pytest.raises(EvidenceError) as raised:
        validate_benchmark_report(report)
    assert raised.value.code == "generation_gap"

    report = _benchmark_report()
    report["primitives"][0]["reset_receipt"]["actual_seed"] = 8610.0
    with pytest.raises(EvidenceError) as raised:
        validate_benchmark_report(report)
    assert raised.value.code == "receipt_mismatch"


def test_wrong_explicit_config_and_phase_config_source_fail(tmp_path):
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    config["primitives"][0]["segments"][0]["duration_sec"] = 1.0
    wrong = tmp_path / "wrong.yaml"
    wrong.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(EvidenceError) as raised:
        resolve_diagnostic_resources(wrong, SPAWN, FEATURES)
    assert raised.value.code == "diagnostic_config_mismatch"

    resources = resolve_diagnostic_resources(CONFIG, SPAWN, FEATURES)
    provenance = {
        "contract": "v6_imu_regime_flat20_features_v2",
        "environment_usd": "/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Grid/default_environment.usd",
        "spawn_poses_file": str(SPAWN.resolve()),
        "spawn_pose": "flat20_start",
        "odometry_mode": "realistic", "navigation_mode": "mapping",
        "obstacle_authoring_enabled": True,
        "obstacle_config_file": str(FEATURES.resolve()),
        "obstacle_config_id": "v6_calibration_grid_features",
        "obstacle_seed": 20260821, "obstacle_count": 7,
        "moving_obstacle_count": 0, "ground_truth_enabled": True,
        "diagnostic_config_file": str(wrong.resolve()),
    }
    phase = [{
        "kind": "manifest", "schema": "bio_nav_v6_imu_regime_phase_trace_v1",
        "passive": True, "provenance": provenance,
    }]
    with pytest.raises(EvidenceError) as raised:
        validate_phase_trace(phase, resources)
    assert raised.value.code == "phase_provenance_mismatch"


def test_installed_resource_manifest_resolves_real_share_layout(monkeypatch, tmp_path):
    share = tmp_path / "share/robot_experiments"
    (share / "config").mkdir(parents=True)
    (share / "environments").mkdir()
    shutil.copy2(CONFIG, share / "config/v6_imu_regime_diagnostic.yaml")
    shutil.copy2(PACKAGE_ROOT / "config/v6_imu_regime_resources.json", share / "config/v6_imu_regime_resources.json")
    shutil.copy2(SPAWN, share / "environments/v6_calibration_flat_20m.spawn.yaml")
    shutil.copy2(FEATURES, share / "config/v6_calibration_grid_features.yaml")
    packages = SimpleNamespace(get_package_share_directory=lambda _name: str(share))
    monkeypatch.setitem(sys.modules, "ament_index_python", SimpleNamespace(packages=packages))
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", packages)
    resources = resolve_diagnostic_resources()
    assert resources.config_path == (share / "config/v6_imu_regime_diagnostic.yaml").resolve()
    assert resources.spawn_poses_path == (share / "environments/v6_calibration_flat_20m.spawn.yaml").resolve()
    assert resources.obstacle_config_path == (share / "config/v6_calibration_grid_features.yaml").resolve()
    assert resources.identity["contract"] == "v6_imu_regime_flat20_v2"
    geometry = resources.identity["obstacle_config"]
    assert geometry["prim_type"] == "UsdGeom.Cube"
    assert geometry["cube_size"] == 1.0
    assert geometry["rotation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert geometry["collision_enabled"] is True
    assert geometry["rigid_body_enabled"] is True
    assert geometry["kinematic_enabled"] is True
    assert [item["id"] for item in geometry["obstacles"]] == [
        "flat20_wall_west", "flat20_wall_east", "flat20_wall_south",
        "flat20_wall_north", "flat20_feature_southwest",
        "flat20_feature_northeast", "flat20_feature_northwest",
    ]
    assert all(item["scale"] == item["size"] for item in geometry["obstacles"])
    assert all(item["height_m"] == item["size"][2] for item in geometry["obstacles"])
    assert all(item["parked"] and item["stationary"] for item in geometry["obstacles"])
    assert all(item["velocity_mps"] == 0.0 for item in geometry["obstacles"])

    installed_features = share / "config/v6_calibration_grid_features.yaml"
    changed = yaml.safe_load(installed_features.read_text(encoding="utf-8"))
    changed["obstacles"][4]["start"] = [-5.5, -4.0, 0.6]
    installed_features.write_text(yaml.safe_dump(changed), encoding="utf-8")
    with pytest.raises(EvidenceError) as raised:
        resolve_diagnostic_resources()
    assert raised.value.code == "obstacle_config_mismatch"


@pytest.mark.parametrize(
    "mutate",
    [
        lambda data: data["obstacles"][0].__setitem__("id", "renamed_wall"),
        lambda data: data["obstacles"][0].__setitem__("size", [9.0, 9.0, 9.0]),
        lambda data: data["obstacles"][0].__setitem__("start", [-9.0, 0.0, 0.5]),
        lambda data: data["obstacles"][0].__setitem__("mass", 1.0),
        lambda data: data["obstacles"][0].__setitem__("mode", "linear"),
        lambda data: data["obstacles"][0].__setitem__("speed", 0.1),
        lambda data: data.__setitem__("seed", 20260822),
        lambda data: data["obstacles"].pop(),
    ],
    ids=(
        "renamed", "nine_by_nine", "moved_origin", "mass", "moving_mode",
        "velocity", "seed", "count",
    ),
)
def test_flat20_obstacle_geometry_is_exact(mutate, tmp_path):
    obstacle = yaml.safe_load(FEATURES.read_text(encoding="utf-8"))
    mutate(obstacle)
    changed = tmp_path / "changed_features.yaml"
    changed.write_text(yaml.safe_dump(obstacle), encoding="utf-8")
    with pytest.raises(EvidenceError) as raised:
        resolve_diagnostic_resources(CONFIG, SPAWN, changed)
    assert raised.value.verdict == "FAIL"
    assert raised.value.code == "obstacle_config_mismatch"


@pytest.mark.parametrize("pose_name", ["flat20_start", "mapping_start"])
def test_flat20_spawn_origin_and_alias_are_exact(pose_name, tmp_path):
    spawn = yaml.safe_load(SPAWN.read_text(encoding="utf-8"))
    spawn["spawn_poses"][pose_name]["map"]["position"] = [1.0, 0.0]
    changed = tmp_path / "changed_spawn.yaml"
    changed.write_text(yaml.safe_dump(spawn), encoding="utf-8")
    with pytest.raises(EvidenceError) as raised:
        resolve_diagnostic_resources(CONFIG, changed, FEATURES)
    assert raised.value.verdict == "FAIL"
    assert raised.value.code == "spawn_resource_mismatch"


def _goal_records(
    *, collision=False, completion=True, completions=None, raw_nonfinite=False,
    second_reset=False, duplicate_raw=False, raw_gap=False,
    corrected_gap=False, route_request=False, post_terminal_command=False,
    duplicate_corrected=False, backward_corrected=False,
    zero_corrected=False, corrected_header_offset_s=0.0,
    sensor_bag_time_offset_s=0.0, extra_commands=(), sample_start=2.0,
    sample_count=11,
):
    def stamp(value):
        sec = int(value)
        return SimpleNamespace(sec=sec, nanosec=int(round((value - sec) * 1e9)))

    def imu(value, rate):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(value)),
            angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=rate),
        )

    def odom(value, yaw):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(value)),
            pose=SimpleNamespace(pose=SimpleNamespace(orientation=SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
            ))),
        )

    def twist(linear, angular):
        return SimpleNamespace(
            linear=SimpleNamespace(x=linear), angular=SimpleNamespace(z=angular)
        )

    def goal():
        return SimpleNamespace(
            header=SimpleNamespace(stamp=stamp(1.75), frame_id="map"),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )

    receipt = json.dumps({
        "pose": "goal", "seed": 42, "odometry": "realistic",
        "generation": 2, "case_id": "", "variant_id": "",
    }, separators=(",", ":"), sort_keys=True)
    sample_times = [sample_start + index * 0.1 for index in range(sample_count)]
    raw_times = list(sample_times)
    if raw_gap:
        raw_times = [value for value in raw_times if value < 2.4 or value > 2.9]
    if duplicate_raw:
        raw_times[2] = raw_times[1]
    records = [
        ("/simulation/reset_event", SimpleNamespace(), int(1.0e9)),
        ("/rosout", SimpleNamespace(stamp=stamp(1.1), msg=f"complete; reset_receipt={receipt}"), int(1.1e9)),
        ("/simulation/collision", SimpleNamespace(data=False), int(1.5e9)),
    ]
    if route_request:
        records.append(("/bio_nav/route_goal", goal(), int(1.8e9)))
    records.extend(
        ("/cmd_vel", twist(linear, angular), int(value * 1e9))
        for value, linear, angular in extra_commands
    )
    for index, value in enumerate(raw_times):
        jitter = (0.03 if index % 2 else -0.03) if sensor_bag_time_offset_s else 0.0
        recorded = value + sensor_bag_time_offset_s + jitter
        records.append(("/imu/data_raw", imu(value, math.nan if raw_nonfinite and index == 1 else 0.5), int(recorded * 1e9)))
    for index, value in enumerate(sample_times):
        jitter = (0.02 if index % 2 else -0.02) if sensor_bag_time_offset_s else 0.0
        recorded = value + sensor_bag_time_offset_s + jitter
        records.append(("/ground_truth/odom", odom(value, 0.5 * (value - 2.0)), int(recorded * 1e9)))
        records.append(("/cmd_vel", twist(0.25, 0.4), int(value * 1e9)))
    corrected_times = (
        [value for value in sample_times if value < 2.4 or value > 2.9]
        if corrected_gap else sample_times
    )
    corrected_header_times = [value + corrected_header_offset_s for value in corrected_times]
    if duplicate_corrected:
        corrected_header_times[2] = corrected_header_times[1]
    if backward_corrected:
        corrected_header_times[2] = corrected_header_times[1] - 0.05
    if zero_corrected:
        corrected_header_times[0] = 0.0
    for index, (recorded_base, header_value) in enumerate(zip(corrected_times, corrected_header_times)):
        jitter = (0.04 if index % 2 else -0.04) if sensor_bag_time_offset_s else 0.0
        recorded = recorded_base + sensor_bag_time_offset_s + jitter
        records.append(("/imu/data", imu(header_value, 0.465), int(recorded * 1e9)))
    records.extend([
        ("/simulation/collision", SimpleNamespace(data=collision), int(2.5e9)),
        ("/simulation/collision", SimpleNamespace(data=False), int(3.4e9)),
    ])
    terminal_values = [(3.5, completion)] if completions is None else completions
    records.extend(
        ("/bio_nav/route_goal_complete", SimpleNamespace(data=value), int(stamp_s * 1e9))
        for stamp_s, value in terminal_values
    )
    if post_terminal_command:
        records.append(("/cmd_vel", twist(0.1, 0.1), int(3.6e9)))
    if second_reset:
        records.append(("/simulation/reset_event", SimpleNamespace(), int(1.2e9)))
    return records


def _goal_types(*, corrected=False, route_request=False):
    result = {
        "/imu/data_raw": "sensor_msgs/msg/Imu",
        "/ground_truth/odom": "nav_msgs/msg/Odometry",
        "/cmd_vel": "geometry_msgs/msg/Twist",
        "/simulation/reset_event": "std_msgs/msg/Empty",
        "/simulation/collision": "std_msgs/msg/Bool",
        "/bio_nav/route_goal_complete": "std_msgs/msg/Bool",
        "/rosout": "rcl_interfaces/msg/Log",
    }
    if corrected:
        result["/imu/data"] = "sensor_msgs/msg/Imu"
    if route_request:
        result["/bio_nav/route_goal"] = "geometry_msgs/msg/PoseStamped"
    return result


def _goal_metadata(path, *, route_request=False):
    result = {
        "schema_version": 1, "source": "goal_mcap_outcome_metadata",
        "source_mcap": str(path.resolve()),
        "reset_receipt": {
            "requested_seed": 42, "actual_seed": 42,
            "generation": 2, "pose": "goal",
        },
        "outcome": "SUCCEEDED", "collision_detected": False,
        # These untrusted arrays must be ignored in favor of MCAP derivation.
        "raw_integrated_yaw_rad": [999.0],
        "ground_truth_relative_yaw_rad": [999.0],
    }
    if route_request:
        result["route_goal_request"] = {
            "recorded_s": 1.8,
            "header_stamp_s": 1.75,
            "frame_id": "map",
            "position_m": [1.0, 2.0, 0.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
        }
    return result


def _write_real_goal_mcap(path, records, topic_types):
    rosbag2_py = pytest.importorskip("rosbag2_py")
    serialization = pytest.importorskip("rclpy.serialization")
    utilities = pytest.importorskip("rosidl_runtime_py.utilities")
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    message_types = {}
    for index, (topic, type_name) in enumerate(topic_types.items()):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=index, name=topic, type=type_name, serialization_format="cdr",
        ))
        message_types[topic] = utilities.get_message(type_name)

    def copy_stamp(target, source):
        target.sec = int(source.sec)
        target.nanosec = int(source.nanosec)

    for topic, source, received_ns in records:
        if topic not in message_types:
            continue
        message = message_types[topic]()
        if topic in {"/imu/data_raw", "/imu/data"}:
            copy_stamp(message.header.stamp, source.header.stamp)
            message.angular_velocity.x = float(source.angular_velocity.x)
            message.angular_velocity.y = float(source.angular_velocity.y)
            message.angular_velocity.z = float(source.angular_velocity.z)
        elif topic == "/ground_truth/odom":
            copy_stamp(message.header.stamp, source.header.stamp)
            source_q = source.pose.pose.orientation
            target_q = message.pose.pose.orientation
            target_q.x = float(source_q.x)
            target_q.y = float(source_q.y)
            target_q.z = float(source_q.z)
            target_q.w = float(source_q.w)
        elif topic == "/cmd_vel":
            message.linear.x = float(source.linear.x)
            message.angular.z = float(source.angular.z)
        elif topic in {"/simulation/collision", "/bio_nav/route_goal_complete"}:
            message.data = bool(source.data)
        elif topic == "/rosout":
            copy_stamp(message.stamp, source.stamp)
            message.msg = str(source.msg)
        writer.write(topic, serialization.serialize_message(message), int(received_ns))
    writer.close()


def test_goal_mcap_derives_arrays_and_ignores_manual_arrays(monkeypatch, tmp_path):
    bag = tmp_path / "goal"
    _install_fake_mcap(monkeypatch, _goal_types(), _goal_records())
    result = load_goal_mcap(bag, _goal_metadata(bag))
    assert result["bag_verified"] is True
    assert result["outcome"] == "SUCCEEDED"
    assert result["collision_detected"] is False
    assert result["raw_integrated_yaw_rad"] != [999.0]
    assert result["ground_truth_relative_yaw_rad"] != [999.0]
    assert result["goal_window"]["start_s"] == pytest.approx(2.0)
    assert result["goal_window"]["end_s"] == pytest.approx(3.0)
    assert result["attempt_provenance"]["terminal_count"] == 1
    assert result["attempt_provenance"]["terminal_values"] == [True]
    assert result["attempt_provenance"]["terminal_timestamps_s"] == pytest.approx([3.5])
    assert result["goal_window"]["binding_source"] == "reset_terminal_single_command_attempt"
    assert result["stream_coverage"]["maximum_gap_s"]["raw"] == pytest.approx(0.1)
    assert result["stream_coverage"]["maximum_gap_s"]["ground_truth"] == pytest.approx(0.1)
    assert result["stream_coverage"]["common_coverage_fraction"] == pytest.approx(1.0)


def test_goal_mcap_imu_headers_are_authoritative_over_jittered_bag_time(monkeypatch, tmp_path):
    bag = tmp_path / "goal"
    _install_fake_mcap(
        monkeypatch,
        _goal_types(corrected=True),
        _goal_records(sensor_bag_time_offset_s=100.0),
    )
    result = load_goal_mcap(bag, _goal_metadata(bag))
    assert result["goal_window"]["common_t0_s"] == pytest.approx(2.0)
    assert result["goal_window"]["common_t1_s"] == pytest.approx(3.0)
    assert result["stream_coverage"]["maximum_gap_s"]["raw"] == pytest.approx(0.1)
    assert result["stream_coverage"]["maximum_gap_s"]["corrected"] == pytest.approx(0.1)
    assert result["corrected_integrated_yaw_rad"][-1] == pytest.approx(0.465)


def test_real_goal_mcap_file_order_preserves_headers_across_received_inversion(tmp_path):
    bag = tmp_path / "goal"
    records = _goal_records(extra_commands=(
        (2.1, 0.25, 0.4),
        (1.9, 0.25, 0.4),
    ))
    raw_indices = [
        index for index, (topic, _message, _received_ns) in enumerate(records)
        if topic == "/imu/data_raw"
    ]
    first = raw_indices[1]
    second = raw_indices[2]
    topic, message, _received_ns = records[first]
    records[first] = (topic, message, 102_200_000_000)
    topic, message, _received_ns = records[second]
    records[second] = (topic, message, 102_100_000_000)
    _write_real_goal_mcap(bag, records, _goal_types())

    result = load_goal_mcap(bag, _goal_metadata(bag))

    assert result["mcap_provenance"]["read_order"] == "file"
    assert result["mcap_provenance"]["yaw_time_basis"] == "header_stamp_in_file_publish_order"
    assert result["mcap_provenance"]["event_time_basis"] == "received_timestamp_sorted_after_collection"
    assert result["goal_window"]["start_s"] == pytest.approx(1.9)
    assert result["stream_coverage"]["maximum_gap_s"]["raw"] == pytest.approx(0.1)


@pytest.mark.parametrize(
    "record_options",
    [
        {"duplicate_raw": True},
        {"backward_corrected": True},
    ],
)
def test_real_goal_mcap_file_order_exposes_header_duplicate_and_backward(
    tmp_path, record_options,
):
    bag = tmp_path / "goal"
    topic_types = _goal_types(corrected="backward_corrected" in record_options)
    _write_real_goal_mcap(bag, _goal_records(**record_options), topic_types)

    with pytest.raises(EvidenceError) as raised:
        load_goal_mcap(bag, _goal_metadata(bag))

    assert raised.value.verdict == "FAIL"
    assert raised.value.code == "goal_stamp_order"


@pytest.mark.parametrize(
    "record_options,expected_verdict,expected_code",
    [
        ({"zero_corrected": True}, "FAIL", "goal_stamp_invalid"),
        ({"duplicate_corrected": True}, "FAIL", "goal_stamp_order"),
        ({"backward_corrected": True}, "FAIL", "goal_stamp_order"),
        ({"corrected_header_offset_s": -0.5}, "AMBIGUOUS", "goal_sample_gap"),
    ],
)
def test_goal_mcap_rejects_invalid_or_stale_corrected_headers(
    monkeypatch, tmp_path, record_options, expected_verdict, expected_code,
):
    bag = tmp_path / "goal"
    _install_fake_mcap(
        monkeypatch,
        _goal_types(corrected=True),
        _goal_records(**record_options),
    )
    with pytest.raises(EvidenceError) as raised:
        load_goal_mcap(bag, _goal_metadata(bag))
    assert raised.value.verdict == expected_verdict
    assert raised.value.code == expected_code


def test_goal_mcap_binds_single_route_request_and_checks_metadata(monkeypatch, tmp_path):
    bag = tmp_path / "goal"
    _install_fake_mcap(
        monkeypatch, _goal_types(route_request=True),
        _goal_records(route_request=True),
    )
    result = load_goal_mcap(bag, _goal_metadata(bag, route_request=True))
    assert result["attempt_provenance"]["route_request_count"] == 1
    assert result["attempt_provenance"]["binding_source"] == "route_goal_pose_stamped"
    assert result["attempt_provenance"]["route_goal_request"]["frame_id"] == "map"

    metadata = _goal_metadata(bag, route_request=True)
    metadata["route_goal_request"]["position_m"][0] = 9.0
    _install_fake_mcap(
        monkeypatch, _goal_types(route_request=True),
        _goal_records(route_request=True),
    )
    with pytest.raises(EvidenceError) as raised:
        load_goal_mcap(bag, metadata)
    assert raised.value.code == "goal_request_mismatch"


@pytest.mark.parametrize("stamp_s", [1.0, 1.4])
def test_goal_mcap_recorded_request_rejects_nonzero_from_reset_until_request(
    monkeypatch, tmp_path, stamp_s,
):
    bag = tmp_path / "goal"
    _install_fake_mcap(
        monkeypatch, _goal_types(route_request=True),
        _goal_records(
            route_request=True,
            extra_commands=((stamp_s, 0.1, 0.1),),
        ),
    )
    with pytest.raises(EvidenceError) as raised:
        load_goal_mcap(bag, _goal_metadata(bag, route_request=True))
    assert raised.value.verdict == "FAIL"
    assert raised.value.code == "goal_command_before_request"


def test_goal_mcap_recorded_request_ignores_pre_reset_and_accepts_request_boundary(
    monkeypatch, tmp_path,
):
    bag = tmp_path / "goal"
    _install_fake_mcap(
        monkeypatch, _goal_types(route_request=True),
        _goal_records(
            route_request=True,
            extra_commands=((0.9, 0.1, 0.1), (1.8, 0.25, 0.4)),
        ),
    )
    result = load_goal_mcap(bag, _goal_metadata(bag, route_request=True))
    assert result["goal_window"]["start_s"] == pytest.approx(1.8)
    assert result["goal_window"]["binding_source"] == "route_goal_pose_stamped"


def test_goal_mcap_without_request_accepts_first_nonzero_at_reset(monkeypatch, tmp_path):
    bag = tmp_path / "goal"
    _install_fake_mcap(
        monkeypatch, _goal_types(),
        _goal_records(sample_start=1.0, sample_count=21),
    )
    result = load_goal_mcap(bag, _goal_metadata(bag))
    assert result["goal_window"]["start_s"] == pytest.approx(1.0)
    assert result["goal_window"]["binding_source"] == "reset_terminal_single_command_attempt"


@pytest.mark.parametrize(
    "record_options,topic_options,expected_code",
    [
        ({"corrected_gap": True}, {"corrected": True}, "goal_sample_gap"),
        ({"post_terminal_command": True}, {}, "goal_command_after_terminal"),
    ],
)
def test_goal_mcap_rejects_optional_stream_gap_and_post_terminal_motion(
    monkeypatch, tmp_path, record_options, topic_options, expected_code,
):
    bag = tmp_path / "goal"
    _install_fake_mcap(
        monkeypatch, _goal_types(**topic_options), _goal_records(**record_options)
    )
    with pytest.raises(EvidenceError) as raised:
        load_goal_mcap(bag, _goal_metadata(bag))
    assert raised.value.code == expected_code


@pytest.mark.parametrize(
    "record_options,metadata_mutation,expected_verdict,expected_code",
    [
        ({"raw_nonfinite": True}, None, "FAIL", "goal_imu_nonfinite"),
        ({"collision": True}, None, "FAIL", "goal_collision"),
        ({"second_reset": True}, None, "FAIL", "goal_reset_count"),
        ({"completion": False}, None, "FAIL", "goal_outcome_false"),
        ({"completions": [(3.4, False), (3.5, True)]}, None, "FAIL", "goal_outcome_count"),
        ({"completions": [(3.4, True), (3.5, True)]}, None, "FAIL", "goal_outcome_count"),
        ({"raw_gap": True}, None, "AMBIGUOUS", "goal_sample_gap"),
        ({"duplicate_raw": True}, None, "FAIL", "goal_stamp_order"),
        ({}, ("outcome", "FAILED"), "FAIL", "goal_outcome_invalid"),
    ],
)
def test_goal_mcap_adversarial_contract(
    monkeypatch, tmp_path, record_options, metadata_mutation,
    expected_verdict, expected_code,
):
    bag = tmp_path / "goal"
    _install_fake_mcap(monkeypatch, _goal_types(), _goal_records(**record_options))
    metadata = _goal_metadata(bag)
    if metadata_mutation is not None:
        metadata[metadata_mutation[0]] = metadata_mutation[1]
    with pytest.raises(EvidenceError) as raised:
        load_goal_mcap(bag, metadata)
    assert raised.value.verdict == expected_verdict
    assert raised.value.code == expected_code


def test_goal_mcap_path_and_reset_number_types_fail(monkeypatch, tmp_path):
    bag = tmp_path / "goal"
    _install_fake_mcap(monkeypatch, _goal_types(), _goal_records())
    metadata = _goal_metadata(bag)
    metadata["source_mcap"] = str(tmp_path / "other")
    with pytest.raises(EvidenceError) as raised:
        load_goal_mcap(bag, metadata)
    assert raised.value.code == "goal_source_mismatch"

    for field, value in (("actual_seed", 42.0), ("generation", True)):
        metadata = _goal_metadata(bag)
        metadata["reset_receipt"][field] = value
        with pytest.raises(EvidenceError) as raised:
            load_goal_mcap(bag, metadata)
        assert raised.value.verdict == "FAIL"
        assert raised.value.code == "integer_contract"
