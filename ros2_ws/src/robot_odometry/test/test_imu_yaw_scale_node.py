"""Unit tests for the calibrated IMU yaw-rate scaling relay."""

import math
import types

import pytest
from sensor_msgs.msg import Imu

from robot_odometry.imu_yaw_scale_node import ImuYawScaleNode


def _make_node(scale=0.922, bias=0.0, covariance=0.005):
    node = object.__new__(ImuYawScaleNode)
    node._scale = scale
    node._bias = bias
    node._covariance = covariance
    published = []
    node._publisher = types.SimpleNamespace(publish=published.append)
    node._published = published
    return node


def _imu(rate, cov=0.0):
    message = Imu()
    message.angular_velocity.z = rate
    message.angular_velocity_covariance[8] = cov
    message.orientation.w = 1.0
    return message


def test_yaw_rate_is_scaled():
    node = _make_node()
    node._on_imu(_imu(1.085))
    out = node._published[0]
    assert out.angular_velocity.z == pytest.approx(1.085 * 0.922)


def test_bias_is_removed_before_scaling():
    node = _make_node(scale=1.0, bias=0.02)
    node._on_imu(_imu(0.52))
    out = node._published[0]
    assert out.angular_velocity.z == pytest.approx(0.5)


def test_zero_covariance_is_replaced():
    node = _make_node()
    node._on_imu(_imu(0.3, cov=0.0))
    assert node._published[0].angular_velocity_covariance[8] == pytest.approx(0.005)


def test_existing_covariance_scales_quadratically():
    node = _make_node(scale=0.5)
    node._on_imu(_imu(0.3, cov=0.04))
    assert node._published[0].angular_velocity_covariance[8] == pytest.approx(0.01)


def test_other_fields_pass_through():
    node = _make_node()
    message = _imu(0.3, cov=0.04)
    message.linear_acceleration.x = 9.81
    message.header.frame_id = 'imu_link'
    node._on_imu(message)
    out = node._published[0]
    assert out.linear_acceleration.x == pytest.approx(9.81)
    assert out.header.frame_id == 'imu_link'
    assert out.angular_velocity_covariance[0] == 0.0
    assert math.isclose(out.orientation.w, 1.0)
