import inspect
import math
from types import MethodType, SimpleNamespace

import pytest
from rclpy._rclpy_pybind11 import RCLError
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
        self.infos = []

    def warning(self, message):
        self.warnings.append(message)

    def info(self, message):
        self.infos.append(message)


class _Context:
    def __init__(self, valid=True):
        self.valid = valid

    def ok(self):
        return self.valid


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
        _stamp_diagnostic_interval=1000,
        _stamp_counters={
            'accepted': 0,
            'duplicate': 0,
            'backward': 0,
        },
        _warned_rejections=set(),
        _shutting_down=False,
        _shutdown_suppressed_callbacks=0,
        context=_Context(),
        _odom_publisher=publisher,
        get_logger=lambda: logger,
    )
    adapter._to_message = MethodType(WheelOdometryNode._to_message, adapter)
    adapter._record_stamp_event = MethodType(
        WheelOdometryNode._record_stamp_event, adapter)
    adapter._warn_rejection = MethodType(
        WheelOdometryNode._warn_rejection, adapter)
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


def test_callback_after_shutdown_never_integrates_or_publishes():
    adapter, publisher = _adapter()
    adapter._shutting_down = True

    _consume(adapter, _joint_state(3_100_000_000))

    assert publisher.messages == []
    assert adapter._integrator.pose == pytest.approx((0.0, 0.0, 0.0))
    assert adapter._stamp_counters == {
        'accepted': 0,
        'duplicate': 0,
        'backward': 0,
    }
    assert adapter._shutdown_suppressed_callbacks == 1


def test_runtime_publish_rclerror_is_not_swallowed():
    adapter, _ = _adapter()

    class _FailingPublisher:
        def publish(self, message):
            del message
            raise RCLError('runtime publisher failure')

    adapter._odom_publisher = _FailingPublisher()
    with pytest.raises(RCLError, match='runtime publisher failure'):
        _consume(adapter, _joint_state(3_200_000_000))


def test_shutdown_race_publish_rclerror_is_suppressed():
    adapter, _ = _adapter()

    class _ShutdownPublisher:
        def publish(self, message):
            del message
            adapter._shutting_down = True
            raise RCLError('context is not valid')

    adapter._odom_publisher = _ShutdownPublisher()
    _consume(adapter, _joint_state(3_300_000_000))

    assert adapter._shutdown_suppressed_callbacks == 1
    assert adapter._stamp_counters['accepted'] == 1


def test_destroy_marks_shutdown_before_destroying_ros_entities():
    method = inspect.getsource(WheelOdometryNode.destroy_node)
    assert method.index('self._shutting_down = True') < method.index(
        'super().destroy_node()')


def test_alternating_duplicate_samples_are_rejected_without_warning_storm():
    adapter, publisher = _adapter()

    for index in range(2001):
        stamp_ns = 4_000_000_000 + index * 10_000_000
        sample = _joint_state(stamp_ns)
        _consume(adapter, sample)
        _consume(adapter, sample)

    logger = adapter.get_logger()
    assert len(publisher.messages) == 2001
    assert adapter._stamp_counters == {
        'accepted': 2001,
        'duplicate': 2001,
        'backward': 0,
    }
    assert len(logger.warnings) == 3
    assert 'duplicate=1' in logger.warnings[0]
    assert 'duplicate=1000' in logger.warnings[1]
    assert 'duplicate=2000' in logger.warnings[2]
    assert len(logger.infos) == 2


def test_backward_samples_remain_rejected_and_report_periodically():
    adapter, publisher = _adapter()
    _consume(adapter, _joint_state(10_000_000_000))
    pose = adapter._integrator.pose

    for index in range(1001):
        _consume(adapter, _joint_state(9_000_000_000 - index))

    logger = adapter.get_logger()
    assert len(publisher.messages) == 1
    assert adapter._integrator.pose == pytest.approx(pose)
    assert adapter._stamp_counters['backward'] == 1001
    assert len(logger.warnings) == 2
    assert 'backward=1' in logger.warnings[0]
    assert 'backward=1000' in logger.warnings[1]
