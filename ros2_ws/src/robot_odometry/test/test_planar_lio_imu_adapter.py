"""Tests for the isolated planar FAST-LIO IMU adapter."""

import math
from pathlib import Path

import pytest

from rclpy.qos import qos_profile_sensor_data

from robot_odometry.planar_lio_imu_adapter import make_planar_lio_imu

from sensor_msgs.msg import Imu


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _message() -> Imu:
    message = Imu()
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = 456
    message.header.frame_id = 'imu_link'
    message.orientation.x = 0.1
    message.orientation.y = 0.2
    message.orientation.z = 0.3
    message.orientation.w = 0.9
    message.orientation_covariance = [float(i) for i in range(9)]
    message.angular_velocity.x = 0.4
    message.angular_velocity.y = -0.5
    message.angular_velocity.z = 0.6
    message.angular_velocity_covariance = [0.01 * i for i in range(9)]
    message.linear_acceleration.x = 1.1
    message.linear_acceleration.y = 1.2
    message.linear_acceleration.z = 9.7
    message.linear_acceleration_covariance = [0.02 * i for i in range(9)]
    return message


def test_preserves_every_field_except_gyro_xy_exactly():
    """Only angular velocity x/y may change in an accepted message."""
    source = _message()
    output = make_planar_lio_imu(source)

    assert output is not source
    assert output.header == source.header
    assert output.orientation == source.orientation
    assert list(output.orientation_covariance) == list(
        source.orientation_covariance
    )
    assert output.linear_acceleration == source.linear_acceleration
    assert list(output.linear_acceleration_covariance) == list(
        source.linear_acceleration_covariance
    )
    assert output.angular_velocity.x == 0.0
    assert output.angular_velocity.y == 0.0
    assert output.angular_velocity.z == source.angular_velocity.z
    assert list(output.angular_velocity_covariance) == list(
        source.angular_velocity_covariance
    )
    assert source.angular_velocity.x == 0.4
    assert source.angular_velocity.y == -0.5


@pytest.mark.parametrize(
    'field',
    ['orientation.x', 'angular_velocity.z', 'linear_acceleration.y'],
)
def test_nonfinite_payload_is_rejected(field):
    """Reject non-finite orientation, angular velocity, or acceleration."""
    message = _message()
    owner, attribute = field.split('.')
    setattr(getattr(message, owner), attribute, math.nan)
    with pytest.raises(ValueError, match='non-finite'):
        make_planar_lio_imu(message)


def test_zero_and_malformed_stamps_are_rejected():
    """Reject zero and structurally invalid source stamps."""
    message = _message()
    message.header.stamp.sec = 0
    message.header.stamp.nanosec = 0
    with pytest.raises(ValueError, match='nonzero'):
        make_planar_lio_imu(message)

    message.header.stamp.nanosec = 1_000_000_000
    with pytest.raises(ValueError, match='invalid'):
        make_planar_lio_imu(message)


def test_sensor_data_qos_and_no_tf_or_canonical_publisher():
    """Keep SensorData QoS and exclude TF/canonical IMU publication."""
    assert qos_profile_sensor_data.depth == 5
    source = (
        PACKAGE_ROOT / 'robot_odometry/planar_lio_imu_adapter.py'
    ).read_text(encoding='utf-8')
    assert 'tf2' not in source
    assert 'Transform' not in source
    assert "output_topic', '/imu/lio'" in source
    assert "input_topic', '/imu/data'" in source


def test_launch_is_default_off_and_topics_are_isolated():
    """Require explicit launch opt-in without changing EKF input."""
    source = (PACKAGE_ROOT / 'launch/planar_lio_imu.launch.py').read_text(
        encoding='utf-8'
    )
    assert "DeclareLaunchArgument('enabled', default_value='false')" in source
    assert "default_value='/imu/data'" in source
    assert "default_value='/imu/lio'" in source
    assert 'IfCondition(enabled)' in source

    ekf = (
        PACKAGE_ROOT.parent / 'robot_localization_config/config/ekf.yaml'
    ).read_text(encoding='utf-8')
    assert 'imu0: /imu/data' in ekf
    assert '/imu/lio' not in ekf
