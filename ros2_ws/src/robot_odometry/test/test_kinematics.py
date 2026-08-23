import math
from pathlib import Path

import pytest

from robot_odometry.kinematics import covariance_from_diagonal
from robot_odometry.kinematics import WheelOdometry
from robot_odometry.kinematics import WheelOdometryConfig
import yaml


NAMES = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]


def _integrator(max_step=0.25):
    return WheelOdometry(WheelOdometryConfig(max_integration_step=max_step))


def _wheel_velocities(linear, angular):
    config = WheelOdometryConfig()
    left = (linear - 0.5 * angular * config.track_width) \
        / config.wheel_radius
    right = (linear + 0.5 * angular * config.track_width) \
        / config.wheel_radius
    return [left, right, left, right]


def _guarded_integrator():
    return WheelOdometry(WheelOdometryConfig(
        yaw_disagreement_guard_enabled=True,
    ))


def _guard_update(
        odometry, stamp, *, linear=0.30, wheel_wz=0.20,
        imu_wz=-0.20, imu_stamp=None):
    if imu_stamp is None and imu_wz is not None:
        imu_stamp = stamp - 0.03
    return odometry.update(
        NAMES,
        _wheel_velocities(linear, wheel_wz),
        stamp,
        imu_angular_velocity=imu_wz,
        imu_stamp_s=imu_stamp,
    )


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


def test_realistic_odometry_uses_the_controller_effective_track_width():
    root = Path(__file__).resolve().parents[4]
    robot = yaml.safe_load(
        (root / 'isaac_sim/configs/robots/jackal.yaml').read_text())
    odometry = yaml.safe_load(
        (Path(__file__).resolve().parents[1]
         / 'config/wheel_odometry.yaml').read_text())
    assert odometry['wheel_odometry']['ros__parameters']['track_width'] \
        == pytest.approx(robot['controller']['wheel_distance'])
    assert robot['controller']['wheel_distance'] \
        > robot['geometric_track_width']


def test_default_off_is_exactly_equivalent_with_unusable_imu_inputs():
    baseline = WheelOdometry(WheelOdometryConfig())
    disabled = WheelOdometry(WheelOdometryConfig(
        yaw_disagreement_guard_enabled=False,
    ))
    velocities = _wheel_velocities(0.30, 0.20)

    for stamp in (1.0, 1.02, 1.04, 1.06):
        expected = baseline.update(NAMES, velocities, stamp)
        actual = disabled.update(
            NAMES,
            velocities,
            stamp,
            imu_angular_velocity=math.nan,
            imu_stamp_s=math.inf,
        )
        assert actual == expected
    assert disabled.pose == baseline.pose


def test_yaw_disagreement_requires_three_samples_and_clamps_both_signs():
    odometry = _guarded_integrator()
    outputs = [
        _guard_update(odometry, 2.0 + index * 0.02).sample.linear_velocity
        for index in range(3)
    ]
    assert outputs == pytest.approx([0.30, 0.30, 0.05])
    assert odometry.yaw_disagreement_guard_active

    odometry.reset()
    outputs = [
        _guard_update(
            odometry, 3.0 + index * 0.02, linear=-0.30,
        ).sample.linear_velocity
        for index in range(3)
    ]
    assert outputs == pytest.approx([-0.30, -0.30, -0.05])


def test_yaw_disagreement_hysteresis_keeps_one_episode_then_clears_in_three():
    odometry = _guarded_integrator()
    for index in range(3):
        _guard_update(odometry, 4.0 + index * 0.02)

    # One agreeing sample, then renewed disagreement, does not chatter.
    agreeing = _guard_update(
        odometry, 4.06, imu_wz=0.20,
    )
    assert agreeing.sample.linear_velocity == pytest.approx(0.05)
    assert odometry.yaw_disagreement_guard_active
    renewed = _guard_update(odometry, 4.08)
    assert renewed.sample.linear_velocity == pytest.approx(0.05)

    first = _guard_update(odometry, 4.10, imu_wz=0.20)
    second = _guard_update(odometry, 4.12, imu_wz=0.20)
    third = _guard_update(odometry, 4.14, imu_wz=0.20)
    assert first.sample.linear_velocity == pytest.approx(0.05)
    assert second.sample.linear_velocity == pytest.approx(0.05)
    assert third.sample.linear_velocity == pytest.approx(0.30)
    assert not odometry.yaw_disagreement_guard_active


@pytest.mark.parametrize(
    'imu_wz, imu_stamp_offset',
    [
        (None, None),
        (-0.20, -0.051),
        (-0.20, 0.001),
        (math.nan, -0.01),
    ],
    ids=['missing', 'stale', 'future', 'nonfinite'],
)
def test_unusable_imu_fails_open_and_safely_clears(
        imu_wz, imu_stamp_offset):
    odometry = _guarded_integrator()
    for index in range(3):
        _guard_update(odometry, 5.0 + index * 0.02)
    assert odometry.yaw_disagreement_guard_active

    for index in range(3):
        stamp = 5.06 + index * 0.02
        imu_stamp = (
            None if imu_stamp_offset is None else stamp + imu_stamp_offset)
        result = _guard_update(
            odometry,
            stamp,
            imu_wz=imu_wz,
            imu_stamp=imu_stamp,
        )
        assert result.sample.linear_velocity == pytest.approx(0.30)
    assert not odometry.yaw_disagreement_guard_active


def test_reset_clears_yaw_disagreement_detector_state():
    odometry = _guarded_integrator()
    for index in range(3):
        _guard_update(odometry, 6.0 + index * 0.02)
    assert odometry.yaw_disagreement_guard_active

    odometry.reset()
    assert not odometry.yaw_disagreement_guard_active
    first = _guard_update(odometry, 7.0)
    assert first.sample.linear_velocity == pytest.approx(0.30)


def test_low_magnitude_exit_uses_one_fifth_of_entry_threshold():
    odometry = _guarded_integrator()
    for index in range(3):
        _guard_update(odometry, 8.0 + index * 0.02)

    outputs = [
        _guard_update(
            odometry,
            8.06 + index * 0.02,
            wheel_wz=0.02,
            imu_wz=-0.02,
        ).sample.linear_velocity
        for index in range(3)
    ]
    assert outputs == pytest.approx([0.05, 0.05, 0.30])
    assert not odometry.yaw_disagreement_guard_active
