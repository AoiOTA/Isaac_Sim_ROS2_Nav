"""Small adapter checks skipped cleanly when ROS 2 Python is unavailable."""

import json
import math

import pytest


pytest.importorskip("rclpy")
from bio_nav_interfaces.msg import RouteProgress  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402

from robot_experiments.experiment_runner import (  # noqa: E402
    ExperimentRunner,
    _diagnostic_float,
    _sample_from_odometry,
)


def _route_progress_message() -> RouteProgress:
    message = RouteProgress()
    message.request_id = 7
    message.edge_id = 11
    message.edge_index = 2
    message.arc_length_m = 1.25
    message.lateral_error_m = 0.125
    message.remaining_m = 3.5
    message.projected_point.x = 4.25
    message.projected_point.y = -1.5
    message.lookahead_goal.pose.position.x = 4.75
    message.lookahead_goal.pose.position.y = -1.0
    return message


def _route_progress_runner() -> ExperimentRunner:
    runner = ExperimentRunner.__new__(ExperimentRunner)
    runner._navigation_active = True
    runner._canonical_routes = []
    runner._route_progress_samples = []
    return runner


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


def test_nonfinite_read_only_diagnostic_is_encoded_as_json_null():
    assert _diagnostic_float(float("nan")) is None
    assert _diagnostic_float(float("inf")) is None
    assert _diagnostic_float(1.25) == 1.25


def test_route_progress_preserves_finite_diagnostics():
    runner = _route_progress_runner()
    runner._route_progress_callback(_route_progress_message())

    sample = runner._route_progress_samples[0]
    assert sample["lateral_error_m"] == pytest.approx(0.125)
    assert sample["projected_point"] == pytest.approx([4.25, -1.5])


def test_route_progress_normalizes_only_nonfinite_optional_diagnostics():
    runner = _route_progress_runner()
    message = _route_progress_message()
    message.lateral_error_m = float("nan")
    message.projected_point.x = float("inf")
    message.projected_point.y = float("-inf")

    runner._route_progress_callback(message)

    sample = runner._route_progress_samples[0]
    assert sample["lateral_error_m"] is None
    assert sample["projected_point"] == [None, None]
    json.dumps({"route_progress": runner._route_progress_samples}, allow_nan=False)


def test_route_progress_keeps_required_nonfinite_value_strict():
    runner = _route_progress_runner()
    message = _route_progress_message()
    message.arc_length_m = float("nan")

    runner._route_progress_callback(message)

    assert math.isnan(runner._route_progress_samples[0]["arc_length_m"])
    with pytest.raises(ValueError, match="Out of range float values"):
        json.dumps({"route_progress": runner._route_progress_samples}, allow_nan=False)
