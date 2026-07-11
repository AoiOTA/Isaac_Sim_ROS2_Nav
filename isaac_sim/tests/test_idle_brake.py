from __future__ import annotations

import pytest

from isaac_sim.src.robot.idle_brake import IdleBrake, IdleBrakeError, IdleBrakeState


def test_idle_brake_stops_without_a_command_and_after_timeout():
    state = IdleBrakeState(timeout_sec=0.25, command_deadband=0.001)
    assert state.should_brake(1.0)

    state.observe(0.2, 0.0, 1.0)
    assert not state.should_brake(1.20)
    assert state.should_brake(1.26)


def test_zero_or_small_command_engages_brake_immediately():
    state = IdleBrakeState(timeout_sec=0.25, command_deadband=0.001)
    state.observe(0.0, 0.0, 1.0)
    assert state.should_brake(1.0)
    state.observe(0.0005, -0.0005, 2.0)
    assert state.should_brake(2.0)


def test_idle_brake_reset_and_invalid_values():
    state = IdleBrakeState(timeout_sec=0.25, command_deadband=0.001)
    state.observe(0.2, 0.1, 1.0)
    state.reset()
    assert state.should_brake(1.0)
    with pytest.raises(IdleBrakeError, match="finite"):
        state.observe(float("nan"), 0.0, 2.0)


def test_adapter_reset_synchronizes_sleep_state_and_next_command_wakes():
    calls = []

    class Robot:
        def zero_all_velocities(self):
            calls.append("zero")

        def put_to_sleep(self):
            calls.append("sleep")

        def wake_up(self):
            calls.append("wake")

    now = {"value": 10.0}
    brake = IdleBrake.__new__(IdleBrake)
    brake.robot = Robot()
    brake.clock = lambda: now["value"]
    brake.state = IdleBrakeState(
        timeout_sec=0.25, command_deadband=0.001
    )
    brake._braking = False

    brake.reset()
    assert calls == ["zero", "sleep"]
    assert brake._braking is True

    brake.state.observe(0.2, 0.0, now["value"])
    assert brake.update() is False
    assert calls[-1] == "wake"
