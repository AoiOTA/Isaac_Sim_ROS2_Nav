import math
from pathlib import Path

import pytest
import yaml

from robot_odometry.kinematics import covariance_from_diagonal
from robot_odometry.kinematics import WheelOdometry
from robot_odometry.kinematics import WheelOdometryConfig


NAMES = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]


def _integrator(max_step=0.25):
    return WheelOdometry(WheelOdometryConfig(max_integration_step=max_step))


def test_straight_motion_uses_both_wheels_on_each_side():
    odometry = _integrator()
    odometry.update(NAMES, [1.0, 1.0, 3.0, 3.0], 1.0)
    result = odometry.update(NAMES, [1.0, 1.0, 3.0, 3.0], 1.1)

    assert result.accepted
    assert result.sample.linear_velocity == pytest.approx(0.196)
    assert result.sample.angular_velocity == pytest.approx(0.0)
    assert result.sample.x == pytest.approx(0.0196)
    assert result.sample.y == pytest.approx(0.0)


def test_opposed_wheel_velocities_rotate_in_place():
    odometry = _integrator()
    velocities = [-1.0, 1.0, -1.0, 1.0]
    odometry.update(NAMES, velocities, 5.0)
    result = odometry.update(NAMES, velocities, 5.1)

    expected_wz = 2.0 * 0.098 / 0.800
    assert result.sample.linear_velocity == pytest.approx(0.0)
    assert result.sample.angular_velocity == pytest.approx(expected_wz)
    assert result.sample.yaw == pytest.approx(expected_wz * 0.1)


def test_missing_joint_consumes_time_but_never_integrates_gap():
    odometry = _integrator()
    odometry.update(NAMES, [1.0] * 4, 1.0)
    missing = odometry.update(NAMES[:-1], [1.0] * 3, 1.1)
    recovered = odometry.update(NAMES, [1.0] * 4, 1.2)

    assert not missing.accepted
    assert missing.reason == 'missing_required_joint'
    assert recovered.sample.dt == pytest.approx(0.1)
    assert recovered.sample.x == pytest.approx(0.0098)


def test_time_regression_resets_pose_safely():
    odometry = _integrator()
    odometry.update(NAMES, [1.0] * 4, 10.0)
    odometry.update(NAMES, [1.0] * 4, 10.1)
    assert odometry.pose[0] > 0.0

    regressed = odometry.update(NAMES, [1.0] * 4, 2.0)
    assert not regressed.accepted
    assert regressed.reason == 'time_regression_reset'
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))

    resumed = odometry.update(NAMES, [1.0] * 4, 2.1)
    assert resumed.accepted
    assert resumed.sample.x == pytest.approx(0.0098)


def test_duplicate_and_large_steps_are_skipped():
    odometry = _integrator(max_step=0.2)
    odometry.update(NAMES, [1.0] * 4, 1.0)
    assert odometry.update(NAMES, [1.0] * 4, 1.0).reason == 'duplicate_stamp'
    gap = odometry.update(NAMES, [1.0] * 4, 2.0)
    assert gap.reason == 'integration_gap_skipped'
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))


def test_explicit_reset_clears_pose_and_timestamp():
    odometry = _integrator()
    odometry.update(NAMES, [1.0] * 4, 1.0)
    odometry.update(NAMES, [1.0] * 4, 1.1)
    odometry.reset()
    assert odometry.pose == pytest.approx((0.0, 0.0, 0.0))
    assert odometry.last_stamp_s is None


def test_covariance_diagonal_expansion_and_validation():
    covariance = covariance_from_diagonal([1, 2, 3, 4, 5, 6])
    assert [covariance[index * 6 + index] for index in range(6)] \
        == [1, 2, 3, 4, 5, 6]
    assert math.fsum(covariance) == 21
    with pytest.raises(ValueError):
        covariance_from_diagonal([1, 2])


def test_ros_adapter_relies_on_rclpy_builtin_sim_time_parameter():
    source = (
        __file__.replace('test/test_kinematics.py', '')
        + 'robot_odometry/wheel_odometry_node.py'
    )
    with open(source, encoding='utf-8') as source_file:
        assert "declare_parameter('use_sim_time'" not in source_file.read()


def test_realistic_odometry_uses_the_calibrated_effective_track_width():
    root = Path(__file__).resolve().parents[4]
    robot = yaml.safe_load(
        (root / 'isaac_sim/configs/robots/jackal.yaml').read_text())
    odometry = yaml.safe_load(
        (Path(__file__).resolve().parents[1]
         / 'config/wheel_odometry.yaml').read_text())
    # Odometry kinematics uses the GT-calibrated effective track width
    # (0.823 = controller 0.800 x 1.0285 yaw-gain correction, validation doc
    # section 0.2); it intentionally differs from the controller-side value.
    assert odometry['wheel_odometry']['ros__parameters']['track_width'] \
        == pytest.approx(0.823)
    assert robot['controller']['wheel_distance'] \
        > robot['geometric_track_width']
