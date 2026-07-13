"""Unit tests for the pure teleop safety policy."""

import threading

import pytest

from robot_teleop.safety import MotionCommand
from robot_teleop.safety import STOP
from robot_teleop.safety import TeleopConfig
from robot_teleop.safety import TeleopController
from robot_teleop.safety import TeleopRuntime


@pytest.mark.parametrize(
    ('key', 'expected'),
    [
        ('w', MotionCommand(0.50, 0.0)),
        ('W', MotionCommand(0.50, 0.0)),
        ('up', MotionCommand(0.50, 0.0)),
        ('s', MotionCommand(-0.50, 0.0)),
        ('down', MotionCommand(-0.50, 0.0)),
        ('a', MotionCommand(0.0, 0.80)),
        ('left', MotionCommand(0.0, 0.80)),
        ('d', MotionCommand(0.0, -0.80)),
        ('right', MotionCommand(0.0, -0.80)),
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


@pytest.mark.parametrize(
    ('key', 'linear', 'angular'),
    [
        ('+', 0.55, 0.90),
        ('=', 0.55, 0.90),
        ('-', 0.45, 0.70),
        (']', 0.55, 0.80),
        ('[', 0.45, 0.80),
        ('.', 0.50, 0.90),
        (',', 0.50, 0.70),
    ],
)
def test_runtime_speed_keys_adjust_only_the_requested_targets(
        key, linear, angular):
    """Every documented tuning key applies one configured target step."""
    controller = TeleopController(TeleopConfig())

    assert not controller.handle_key(key, 1.0)
    status = controller.speed_feedback()
    assert status.linear_speed == pytest.approx(linear)
    assert status.angular_speed == pytest.approx(angular)
    assert controller.command(1.0) == STOP


def test_speed_reset_restores_configured_defaults():
    """Zero restores both configured defaults without starting motion."""
    controller = TeleopController(TeleopConfig(
        linear_speed=0.45,
        angular_speed=0.75,
    ))
    controller.handle_key('+', 1.0)
    controller.handle_key(']', 1.01)

    assert not controller.handle_key('0', 1.02)
    feedback = controller.take_feedback()
    assert feedback.kind == 'reset'
    assert feedback.linear_speed == pytest.approx(0.45)
    assert feedback.angular_speed == pytest.approx(0.75)
    assert controller.command(1.02) == STOP


def test_speed_limits_are_clamped_and_reported():
    """Repeated tuning cannot cross configured minima or maxima."""
    controller = TeleopController(TeleopConfig(
        linear_speed=0.95,
        angular_speed=1.40,
    ))
    controller.handle_key('+', 1.0)
    maximum = controller.take_feedback()
    assert maximum.linear_speed == pytest.approx(1.0)
    assert maximum.angular_speed == pytest.approx(1.5)
    assert set(maximum.limits) == {'linear maximum', 'angular maximum'}

    for index in range(20):
        controller.handle_key('-', 1.01 + index * 0.01)
    minimum = controller.take_feedback()
    assert minimum.linear_speed == pytest.approx(0.1)
    assert minimum.angular_speed == pytest.approx(0.2)
    assert set(minimum.limits) == {'linear minimum', 'angular minimum'}


def test_adjustment_updates_active_magnitude_without_extending_deadman():
    """Tuning is immediate but cannot renew an existing motion lease."""
    controller = TeleopController(TeleopConfig(command_timeout_sec=0.18))
    controller.handle_key('w', 5.0)

    controller.handle_key('+', 5.10)
    assert controller.command(5.10) == MotionCommand(0.55, 0.0)
    assert controller.command(5.179) == MotionCommand(0.55, 0.0)
    assert controller.command(5.18) == STOP


@pytest.mark.parametrize('key', ['h', 'H', '?'])
def test_help_reports_current_speed_without_starting_or_renewing_motion(key):
    """Help is informational and leaves the wall-time lease unchanged."""
    controller = TeleopController(TeleopConfig())
    controller.handle_key('a', 3.0)

    assert not controller.handle_key(key, 3.10)
    feedback = controller.take_feedback()
    assert feedback.kind == 'help'
    assert feedback.linear_speed == pytest.approx(0.50)
    assert feedback.angular_speed == pytest.approx(0.80)
    assert controller.command(3.18) == STOP


def test_non_ascii_input_fails_safe_without_crashing():
    """Unexpected Unicode terminal input stops instead of raising."""
    controller = TeleopController(TeleopConfig())
    controller.handle_key('w', 1.0)

    assert not controller.handle_key('你', 1.01)
    assert controller.command(1.01) == STOP


def test_speed_mutation_is_thread_safe_and_remains_bounded():
    """Concurrent tuning cannot lose bounds or corrupt controller state."""
    controller = TeleopController(TeleopConfig())
    barrier = threading.Barrier(9)

    def adjust(key):
        barrier.wait()
        for _ in range(100):
            controller.handle_key(key, 1.0)

    workers = [
        threading.Thread(target=adjust, args=(key,))
        for key in ('+', '+', '-', '-', ']', '[', '.', ',')
    ]
    for worker in workers:
        worker.start()
    barrier.wait()
    for worker in workers:
        worker.join(timeout=2.0)

    assert not any(worker.is_alive() for worker in workers)
    status = controller.speed_feedback()
    assert 0.10 <= status.linear_speed <= 1.00
    assert 0.20 <= status.angular_speed <= 1.50
    assert controller.command(1.0) == STOP


def test_timeout_above_safety_limit_is_rejected():
    """The package cannot be configured above the 0.20 second contract."""
    with pytest.raises(ValueError, match='no greater than 0.20s'):
        TeleopConfig(command_timeout_sec=0.201)


@pytest.mark.parametrize(
    'kwargs',
    [
        {'min_linear_speed': 1.1, 'max_linear_speed': 1.0},
        {'min_angular_speed': 1.6, 'max_angular_speed': 1.5},
        {'linear_speed_step': 0.0},
        {'angular_speed_step': float('nan')},
    ],
)
def test_invalid_speed_bounds_and_steps_are_rejected(kwargs):
    with pytest.raises(ValueError):
        TeleopConfig(**kwargs)


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
    assert published[0] == MotionCommand(0.50, 0.0)
    assert published[-2:] == [STOP, STOP]


def test_runtime_publishes_adjusted_active_speed_immediately():
    """A tuning key updates an active output without waiting for the next tick."""
    published = []
    runtime = TeleopRuntime(
        TeleopController(TeleopConfig()), published.append)
    runtime.handle_key('w', 0.0)
    runtime.tick(0.01)

    runtime.handle_key('+', 0.02)

    assert published[-1] == MotionCommand(0.55, 0.0)
    assert runtime.tick(0.18) == STOP


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
