"""Small adapter checks skipped cleanly when ROS 2 Python is unavailable."""

import pytest


pytest.importorskip("rclpy")
from nav_msgs.msg import Odometry  # noqa: E402

from robot_experiments.experiment_runner import _sample_from_odometry  # noqa: E402


def test_odometry_sampler_rejects_nonfinite_data_and_extracts_yaw():
    message = Odometry()
    message.header.stamp.sec = 12
    message.header.stamp.nanosec = 500_000_000
    message.pose.pose.position.x = 1.0
    message.pose.pose.position.y = 2.0
    message.pose.pose.orientation.z = 2**-0.5
    message.pose.pose.orientation.w = 2**-0.5
    message.twist.twist.linear.x = 0.1
    message.twist.twist.angular.z = -0.2
    sample = _sample_from_odometry(message)
    assert sample is not None
    assert sample.yaw_rad == pytest.approx(1.5707963267948966)
    assert sample.stamp_s == pytest.approx(12.5)
    message.pose.pose.position.x = float("nan")
    assert _sample_from_odometry(message) is None
