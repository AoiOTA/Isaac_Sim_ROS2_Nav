from __future__ import annotations

import math
from pathlib import Path

import pytest

from isaac_sim.src.robot.skid_steer_motion_assist import (
    _yaw_command_scale,
    SkidSteerMotionAssistError,
    SkidSteerMotionAssistState,
)


def test_motion_assist_tracks_reverse_and_turning_with_bounded_acceleration():
    state = SkidSteerMotionAssistState(
        command_timeout_sec=0.25,
        max_linear_acceleration=2.0,
        max_angular_acceleration=6.0,
    )
    state.observe(-0.4, 1.2, 10.0)
    assert state.next_command(0.0, 0.0, 10.0, 0.1) \
        == pytest.approx((-0.2, 0.6))
    assert state.next_command(-0.2, 0.6, 10.1, 0.1) \
        == pytest.approx((-0.4, 1.2))


def test_motion_assist_rejects_stale_or_regressed_commands():
    state = SkidSteerMotionAssistState(
        command_timeout_sec=0.25,
        max_linear_acceleration=2.0,
        max_angular_acceleration=6.0,
    )
    assert state.next_command(0.2, 0.2, 1.0, 0.1) is None
    state.observe(0.4, 0.8, 2.0)
    assert state.next_command(0.2, 0.2, 1.9, 0.1) is None
    assert state.next_command(0.2, 0.2, 2.3, 0.1) is None


def test_motion_assist_reset_and_validation_fail_safe():
    state = SkidSteerMotionAssistState(
        command_timeout_sec=0.25,
        max_linear_acceleration=2.0,
        max_angular_acceleration=6.0,
    )
    state.observe(0.4, 0.8, 1.0)
    state.reset()
    assert state.next_command(0.0, 0.0, 1.0, 0.1) is None
    with pytest.raises(SkidSteerMotionAssistError):
        state.observe(float("nan"), 0.0, 1.0)
    with pytest.raises(SkidSteerMotionAssistError):
        state.next_command(0.0, 0.0, 1.0, 0.0)


def test_in_place_yaw_scale_is_exact_and_arc_tracking_is_unchanged():
    assert _yaw_command_scale(0.0) == pytest.approx(1.0)
    assert _yaw_command_scale(-0.0) == pytest.approx(1.0)
    assert _yaw_command_scale(0.10) == pytest.approx(0.9625)
    assert _yaw_command_scale(0.20) == pytest.approx(1.0)
    assert _yaw_command_scale(-0.40) == pytest.approx(1.0)
    assert all(
        math.isfinite(_yaw_command_scale(speed))
        for speed in (-0.40, -0.10, 0.0, 0.10, 0.40)
    )


def test_motion_assist_remains_after_the_physics_sensor_step():
    source = (
        Path(__file__).parents[1] / "apps" / "navigation_sim.py"
    ).read_text(encoding="utf-8")
    loop_start = source.index("            app.update()")
    assist_update = source.index("                motion_assist.update()")
    assert loop_start < assist_update
