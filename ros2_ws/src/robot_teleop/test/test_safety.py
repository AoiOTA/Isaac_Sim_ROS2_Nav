"""Unit tests for the pure teleop safety policy."""

import pytest

from robot_teleop.safety import MotionCommand
from robot_teleop.safety import STOP
from robot_teleop.safety import TeleopConfig
from robot_teleop.safety import TeleopController
from robot_teleop.safety import TeleopRuntime


@pytest.mark.parametrize(
    ('key', 'expected'),
    [
        ('w', MotionCommand(0.30, 0.0)),
        ('W', MotionCommand(0.30, 0.0)),
        ('up', MotionCommand(0.30, 0.0)),
        ('s', MotionCommand(-0.30, 0.0)),
        ('down', MotionCommand(-0.30, 0.0)),
        ('a', MotionCommand(0.0, 0.60)),
        ('left', MotionCommand(0.0, 0.60)),
        ('d', MotionCommand(0.0, -0.60)),
        ('right', MotionCommand(0.0, -0.60)),
    ],
)
def test_motion_keys_create_expected_bounded_command(key, expected):
    """W/A/S/D and arrows map to the configured planar velocities."""
    controller = TeleopController(TeleopConfig())

    assert not controller.handle_key(key, 10.0)
    assert controller.command(10.1) == expected


def test_deadman_stops_at_configured_wall_timeout():
    """A motion lease becomes zero no later than its safety deadline."""
    controller = TeleopController(TeleopConfig(command_timeout_sec=0.18))
    controller.handle_key('w', 5.0)

    assert not controller.command(5.179).stopped
    assert controller.command(5.18) == STOP


@pytest.mark.parametrize('key', [' ', 'unknown', '\x1b'])
def test_stop_and_unknown_keys_disarm_motion_immediately(key):
    """Space and unrecognised terminal input cannot preserve motion."""
    controller = TeleopController(TeleopConfig())
    controller.handle_key('w', 1.0)

    assert not controller.handle_key(key, 1.01)
    assert controller.command(1.01) == STOP


@pytest.mark.parametrize('key', ['q', 'Q', '\x03', '\x04', 'eof'])
def test_exit_keys_request_exit_with_zero_velocity(key):
    """Q, control exits, and EOF atomically stop and request exit."""
    controller = TeleopController(TeleopConfig())
    controller.handle_key('d', 1.0)

    assert controller.handle_key(key, 1.01)
    assert controller.command(1.01) == STOP


def test_configured_speeds_are_clamped_to_robot_limits():
    """Configured commands cannot exceed the Isaac robot speed limits."""
    controller = TeleopController(TeleopConfig(
        linear_speed=3.0,
        angular_speed=4.0,
        max_linear_speed=1.0,
        max_angular_speed=1.5,
    ))

    controller.handle_key('w', 0.0)
    assert controller.command(0.0).linear_x == pytest.approx(1.0)
    controller.handle_key('a', 0.1)
    assert controller.command(0.1).angular_z == pytest.approx(1.5)


def test_timeout_above_safety_limit_is_rejected():
    """The package cannot be configured above the 0.20 second contract."""
    with pytest.raises(ValueError, match='no greater than 0.20s'):
        TeleopConfig(command_timeout_sec=0.201)


def test_wall_time_regression_forces_stop():
    """An impossible monotonic regression fails safe to zero."""
    controller = TeleopController(TeleopConfig())
    controller.handle_key('w', 10.0)

    assert controller.command(9.0) == STOP


def test_runtime_publishes_immediate_and_final_zero():
    """The runtime emits zero for Q and once again during final cleanup."""
    published = []
    runtime = TeleopRuntime(
        TeleopController(TeleopConfig()), published.append)
    runtime.handle_key('w', 0.0)
    runtime.tick(0.01)

    assert runtime.handle_key('q', 0.02)
    runtime.close()
    runtime.close()
    assert published[0] == MotionCommand(0.30, 0.0)
    assert published[-2:] == [STOP, STOP]


def test_runtime_eof_publishes_zero_before_exit_and_during_cleanup():
    """A closed terminal cannot leave the last motion command armed."""
    published = []
    runtime = TeleopRuntime(
        TeleopController(TeleopConfig()), published.append)
    runtime.handle_key('w', 0.0)
    runtime.tick(0.01)

    assert runtime.handle_key('eof', 0.02)
    assert published[-1] == STOP
    runtime.close()
    assert published[-1] == STOP
