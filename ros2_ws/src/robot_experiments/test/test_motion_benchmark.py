from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from robot_experiments.motion_benchmark import (
    evaluate_motion_primitive,
    load_motion_config,
    MotionDispatchBarrier,
    MotionBenchmarkError,
    MotionPrimitive,
    MotionSample,
    MotionSegment,
    MotionThresholds,
    ResetStopGateStatus,
    wait_for_motion_dispatch_barrier,
)


CONFIG = Path(__file__).resolve().parents[1] / "config/motion_benchmark.yaml"


def test_motion_benchmark_is_upstream_of_final_command_authority():
    source = (
        Path(__file__).resolve().parents[1]
        / "robot_experiments/motion_benchmark.py"
    ).read_text(encoding="utf-8")
    assert 'Twist, "/cmd_vel_nav", reliable' in source
    assert 'Twist, "/cmd_vel", reliable' not in source


def _gate_status(generation: int, held: bool, received_at: float):
    return ResetStopGateStatus(
        generation=generation,
        held=held,
        eligible_generation=generation if held else None,
        received_at=received_at,
    )


def test_motion_dispatch_waits_for_delayed_same_generation_release():
    barrier = MotionDispatchBarrier(7, reset_started_at=1.0, settle_sec=0.1)
    assert not barrier.observe(
        gate_status=_gate_status(7, True, 1.1),
        collision_monitor_active=True,
        estimated_state_ready=True,
        now=1.1,
    )
    assert not barrier.observe(
        gate_status=_gate_status(7, False, 1.2),
        collision_monitor_active=True,
        estimated_state_ready=True,
        now=1.2,
    )
    assert barrier.observe(
        gate_status=_gate_status(7, False, 1.3),
        collision_monitor_active=True,
        estimated_state_ready=True,
        now=1.3,
    )


def test_motion_dispatch_never_release_times_out():
    now = [0.0]
    held = _gate_status(3, True, 0.1)
    barrier = MotionDispatchBarrier(3, reset_started_at=0.0, settle_sec=0.0)

    def spin_once(_timeout):
        now[0] += 0.05

    with pytest.raises(TimeoutError, match="gate_released=False"):
        wait_for_motion_dispatch_barrier(
            barrier,
            spin_once=spin_once,
            snapshot=lambda: (held, True, True),
            timeout_sec=0.2,
            monotonic=lambda: now[0],
        )


def test_motion_dispatch_rejects_wrong_reset_generation():
    barrier = MotionDispatchBarrier(8, reset_started_at=2.0, settle_sec=0.0)
    with pytest.raises(RuntimeError, match="generation mismatch"):
        barrier.observe(
            gate_status=_gate_status(9, False, 2.1),
            collision_monitor_active=True,
            estimated_state_ready=True,
            now=2.1,
        )


def test_motion_dispatch_success_requires_collision_monitor_and_estimated_state():
    barrier = MotionDispatchBarrier(5, reset_started_at=1.0, settle_sec=0.0)
    released = _gate_status(5, False, 1.1)
    assert not barrier.observe(
        gate_status=released,
        collision_monitor_active=False,
        estimated_state_ready=True,
        now=1.1,
    )
    assert not barrier.observe(
        gate_status=released,
        collision_monitor_active=True,
        estimated_state_ready=False,
        now=1.2,
    )
    assert barrier.observe(
        gate_status=released,
        collision_monitor_active=True,
        estimated_state_ready=True,
        now=1.3,
    )


def test_motion_benchmark_config_covers_required_primitives():
    config = load_motion_config(CONFIG)
    identifiers = {primitive.identifier for primitive in config.primitives}
    assert {
        "spin_left",
        "spin_right",
        "forward_circle_left",
        "forward_circle_right",
        "reverse_circle_left",
        "reverse_circle_right",
        "reverse_straight",
        "forward_sharp_slalom",
        "reverse_sharp_slalom",
        "rapid_spin_reversal",
    } <= identifiers
    assert config.command_rate_hz >= 20.0


def test_motion_benchmark_rejects_duplicate_primitive_ids(tmp_path):
    document = yaml.safe_load(CONFIG.read_text())
    document["primitives"].append(document["primitives"][0])
    target = tmp_path / "motion.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(MotionBenchmarkError, match="duplicate"):
        load_motion_config(target)


def _thresholds() -> MotionThresholds:
    return MotionThresholds(
        linear_mae_mps=0.06,
        angular_mae_radps=0.12,
        radius_relative_error_percent=20.0,
        tracking_fraction=0.85,
        transition_latency_sec=0.45,
        overshoot_ratio=1.25,
        wrong_direction_fraction=0.05,
    )


def test_motion_evaluation_accepts_smooth_forward_arc():
    primitive = MotionPrimitive(
        "arc",
        (MotionSegment(2.0, 0.30, 0.80),),
    )
    samples = [
        MotionSample(
            received_at=index * 0.05,
            stamp_s=index * 0.05,
            x=index * 0.015,
            y=0.0,
            yaw=index * 0.04,
            linear_speed=0.295,
            angular_speed=0.79,
            segment_index=0,
            segment_elapsed=index * 0.05,
            command_linear=0.30,
            command_angular=0.80,
        )
        for index in range(40)
    ]
    result = evaluate_motion_primitive(
        primitive,
        samples,
        False,
        _thresholds(),
        0.30,
    )
    assert result["passed"]
    assert result["maximum_radius_relative_error_percent"] < 5.0


def test_motion_evaluation_measures_turn_reversal_latency():
    primitive = MotionPrimitive(
        "slalom",
        (
            MotionSegment(1.0, 0.30, 1.0),
            MotionSegment(1.0, 0.30, -1.0),
        ),
    )
    samples: list[MotionSample] = []
    for segment_index, command in enumerate((1.0, -1.0)):
        for index in range(20):
            elapsed = index * 0.05
            actual_angular = command if segment_index == 0 or elapsed >= 0.15 else 0.2
            samples.append(
                MotionSample(
                    received_at=len(samples) * 0.05,
                    stamp_s=len(samples) * 0.05,
                    x=len(samples) * 0.015,
                    y=0.0,
                    yaw=0.0,
                    linear_speed=0.30,
                    angular_speed=actual_angular,
                    segment_index=segment_index,
                    segment_elapsed=elapsed,
                    command_linear=0.30,
                    command_angular=command,
                )
            )
    result = evaluate_motion_primitive(
        primitive,
        samples,
        False,
        _thresholds(),
        0.30,
    )
    assert result["passed"]
    assert result["maximum_turn_transition_latency_sec"] == pytest.approx(0.15)


def test_motion_evaluation_rejects_collision_and_wrong_reverse_direction():
    primitive = MotionPrimitive(
        "reverse",
        (MotionSegment(1.0, -0.30, 0.0),),
    )
    samples = [
        MotionSample(
            received_at=index * 0.05,
            stamp_s=index * 0.05,
            x=index * 0.01,
            y=0.0,
            yaw=0.0,
            linear_speed=0.20,
            angular_speed=0.0,
            segment_index=0,
            segment_elapsed=index * 0.05,
            command_linear=-0.30,
            command_angular=0.0,
        )
        for index in range(20)
    ]
    result = evaluate_motion_primitive(
        primitive,
        samples,
        True,
        _thresholds(),
        0.20,
    )
    assert not result["passed"]
    assert "collision_detected" in result["failure_reasons"]
    assert "wrong_direction" in result["failure_reasons"]
    assert math.isclose(result["translation_tracking_fraction"], 0.0)
