import math
from types import MethodType, SimpleNamespace

import pytest
from robot_odometry.kinematics import WheelOdometry
from robot_odometry.kinematics import WheelOdometryConfig
from robot_odometry.wheel_odometry_node import WheelOdometryNode
from sensor_msgs.msg import JointState


NAMES = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def __init__(self):
        self.warnings = []

    def warning(self, message):
        self.warnings.append(message)


def _adapter():
    publisher = _Publisher()
    logger = _Logger()
    adapter = SimpleNamespace(
        _integrator=WheelOdometry(WheelOdometryConfig()),
        _odom_frame='odom',
        _base_frame='base_link',
        _pose_covariance=[0.0] * 36,
        _twist_covariance=[0.0] * 36,
        _last_joint_stamp_ns=None,
        _last_rejection=None,
        _odom_publisher=publisher,
        get_logger=lambda: logger,
    )
    adapter._to_message = MethodType(WheelOdometryNode._to_message, adapter)
    return adapter, publisher


def _joint_state(stamp_ns, velocities=None, names=None):
    message = JointState()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.name = NAMES if names is None else names
    message.velocity = [1.0] * 4 if velocities is None else velocities
    return message


def _consume(adapter, message):
    WheelOdometryNode._joint_state_callback(adapter, message)


def test_input_stamp_drives_exactly_one_integration_and_output_stamp():
    adapter, publisher = _adapter()
    first = _joint_state(2_000_000_010)
    second = _joint_state(2_100_000_010)

    _consume(adapter, first)
    assert len(publisher.messages) == 1
    assert publisher.messages[-1].header.stamp == first.header.stamp
    assert publisher.messages[-1].pose.pose.position.x == pytest.approx(0.0)

    _consume(adapter, second)
    assert len(publisher.messages) == 2
    assert publisher.messages[-1].header.stamp == second.header.stamp
    assert publisher.messages[-1].pose.pose.position.x == pytest.approx(0.0098)

    pose_after_second = adapter._integrator.pose
    _consume(adapter, second)
    _consume(adapter, _joint_state(2_050_000_010))
    assert len(publisher.messages) == 2
    assert adapter._integrator.pose == pytest.approx(pose_after_second)


def test_zero_invalid_and_incomplete_samples_never_advance_pose():
    adapter, publisher = _adapter()

    _consume(adapter, _joint_state(0))
    _consume(adapter, _joint_state(
        1_000_000_000,
        names=NAMES[:-1],
        velocities=[1.0] * 3,
    ))
    _consume(adapter, _joint_state(
        1_100_000_000,
        velocities=[1.0, 1.0, math.nan, 1.0],
    ))

    assert publisher.messages == []
    assert adapter._integrator.pose == pytest.approx((0.0, 0.0, 0.0))

    _consume(adapter, _joint_state(1_200_000_000))
    assert len(publisher.messages) == 1
    assert publisher.messages[-1].pose.pose.position.x == pytest.approx(0.0098)


def test_no_timer_path_can_reintegrate_a_stopped_input():
    adapter, publisher = _adapter()
    _consume(adapter, _joint_state(3_000_000_000))
    pose_when_input_stops = adapter._integrator.pose

    assert not hasattr(WheelOdometryNode, '_timer_callback')
    assert adapter._integrator.pose == pytest.approx(pose_when_input_stops)
    assert len(publisher.messages) == 1
