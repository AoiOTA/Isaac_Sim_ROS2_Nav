"""Node-level tests for stamp integration, staleness and slip covariance."""

import time
from types import SimpleNamespace

import pytest

from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from robot_odometry.wheel_odometry_node import SLIP_COVARIANCE_SCALE
from robot_odometry.wheel_odometry_node import STALE_COVARIANCE_SCALE
from robot_odometry.wheel_odometry_node import WheelOdometryNode
from sensor_msgs.msg import JointState


NAMES = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]
YAW_INDEX = 5 * 6 + 5


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init(args=[])
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    instance = WheelOdometryNode()
    recorder = SimpleNamespace(odom=[], diagnostics=[])
    instance._odom_publisher = SimpleNamespace(publish=recorder.odom.append)
    instance._diagnostics_publisher = SimpleNamespace(
        publish=recorder.diagnostics.append)
    yield instance, recorder
    instance.destroy_node()


def _now_s(node):
    return node.get_clock().now().nanoseconds * 1.0e-9


def _drive(node, stamp_s, velocities):
    message = JointState()
    message.name = NAMES
    message.velocity = velocities
    message.header.stamp.sec = int(stamp_s)
    message.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1.0e9)
    node._joint_state_callback(message)


def _diag_status(recorder):
    return recorder.diagnostics[-1].status[0]


def _diag_values(recorder):
    return {entry.key: entry.value for entry in _diag_status(recorder).values}


def test_pose_freezes_and_covariance_inflates_when_joint_states_stop(node):
    instance, recorder = node
    t0 = _now_s(instance)
    _drive(instance, t0, [1.0] * 4)
    _drive(instance, t0 + 0.02, [1.0] * 4)
    instance._timer_callback()

    fresh = recorder.odom[-1]
    assert fresh.pose.pose.position.x > 0.0
    assert list(fresh.pose.covariance) \
        == pytest.approx(instance._pose_covariance)

    time.sleep(0.15)
    instance._timer_callback()
    instance._timer_callback()

    held = recorder.odom[-1]
    assert held.pose.pose.position.x == fresh.pose.pose.position.x
    assert held.pose.pose.position.y == fresh.pose.pose.position.y
    assert held.pose.covariance[0] == pytest.approx(
        instance._pose_covariance[0] * STALE_COVARIANCE_SCALE)
    assert held.pose.covariance[YAW_INDEX] == pytest.approx(
        instance._pose_covariance[YAW_INDEX] * STALE_COVARIANCE_SCALE)
    # The stamp stays at the last integrated joint sample while held.
    assert held.header.stamp == fresh.header.stamp

    status = _diag_status(recorder)
    assert status.level == DiagnosticStatus.STALE
    assert _diag_values(recorder)['stale'] == 'true'
    assert _diag_values(recorder)['covariance_scale'] \
        == f'{STALE_COVARIANCE_SCALE:g}'


def test_non_monotonic_samples_are_skipped_without_resetting_pose(node):
    instance, _ = node
    _drive(instance, 100.0, [1.0] * 4)
    _drive(instance, 100.02, [1.0] * 4)
    x_before = instance._integrator.pose[0]
    assert x_before > 0.0

    _drive(instance, 99.0, [5.0] * 4)  # regression
    _drive(instance, 100.02, [5.0] * 4)  # duplicate of the last stamp
    assert instance._non_monotonic_samples == 2
    assert instance._integrator.pose[0] == x_before
    assert instance._last_integrated_sample.stamp_s == pytest.approx(100.02)

    _drive(instance, 100.04, [1.0] * 4)
    assert instance._integrator.pose[0] > x_before


def test_same_side_wheel_spread_inflates_covariance(node):
    instance, recorder = node
    t0 = _now_s(instance)
    # Left spread is 4 rad/s (> 3.0) while the side means stay equal,
    # so slip triggers without also triggering the turn inflation.
    velocities = [-1.0, 1.0, 3.0, 1.0]
    _drive(instance, t0, velocities)
    _drive(instance, t0 + 0.02, velocities)
    instance._timer_callback()

    message = recorder.odom[-1]
    assert message.pose.covariance[0] == pytest.approx(
        instance._pose_covariance[0] * SLIP_COVARIANCE_SCALE)
    assert message.pose.covariance[YAW_INDEX] == pytest.approx(
        instance._pose_covariance[YAW_INDEX] * SLIP_COVARIANCE_SCALE)
    values = _diag_values(recorder)
    assert values['slip_detected'] == 'true'
    assert values['covariance_scale'] == f'{SLIP_COVARIANCE_SCALE:g}'
    assert _diag_status(recorder).level == DiagnosticStatus.WARN


def test_turning_inflates_only_the_yaw_covariance(node):
    instance, recorder = node
    t0 = _now_s(instance)
    velocities = [0.5, 2.5, 0.5, 2.5]  # clean turn, no same-side spread
    _drive(instance, t0, velocities)
    _drive(instance, t0 + 0.02, velocities)
    instance._timer_callback()

    message = recorder.odom[-1]
    assert message.pose.covariance[0] \
        == pytest.approx(instance._pose_covariance[0])
    assert message.pose.covariance[YAW_INDEX] == pytest.approx(
        instance._pose_covariance[YAW_INDEX]
        * instance._turn_yaw_covariance_scale)
    values = _diag_values(recorder)
    assert values['turn_detected'] == 'true'
    assert values['yaw_covariance_scale'] \
        == f'{instance._turn_yaw_covariance_scale:g}'


def test_covariance_returns_to_baseline_after_joint_states_recover(node):
    instance, recorder = node
    t0 = _now_s(instance)
    _drive(instance, t0, [1.0] * 4)
    _drive(instance, t0 + 0.02, [1.0] * 4)
    instance._timer_callback()

    time.sleep(0.15)
    instance._timer_callback()
    assert recorder.odom[-1].pose.covariance[0] == pytest.approx(
        instance._pose_covariance[0] * STALE_COVARIANCE_SCALE)

    _drive(instance, _now_s(instance), [1.0] * 4)
    instance._timer_callback()

    recovered = recorder.odom[-1]
    assert recovered.pose.covariance[0] \
        == pytest.approx(instance._pose_covariance[0])
    assert recovered.pose.covariance[YAW_INDEX] \
        == pytest.approx(instance._pose_covariance[YAW_INDEX])
    assert _diag_status(recorder).level == DiagnosticStatus.OK
    assert _diag_values(recorder)['stale'] == 'false'
