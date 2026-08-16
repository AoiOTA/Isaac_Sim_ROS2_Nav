import math

import pytest

from robot_odometry.calibration_metrics import PlanarPose
from robot_odometry.calibration_metrics import segment_motion
from robot_odometry.calibration_metrics import wrap_angle
from robot_odometry.calibration_metrics import yaw_from_quaternion


def test_yaw_from_quaternion_identity_and_ninety_degrees():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == 0.0
    half = math.sqrt(0.5)
    assert math.isclose(
        yaw_from_quaternion(0.0, 0.0, half, half), math.pi / 2.0)


def test_wrap_angle_bounds_and_continuity():
    assert wrap_angle(0.0) == 0.0
    assert math.isclose(wrap_angle(3.0 * math.pi), math.pi)
    assert math.isclose(wrap_angle(-3.0 * math.pi), math.pi)
    assert math.isclose(wrap_angle(2.0 * math.pi + 0.25), 0.25)
    assert wrap_angle(math.pi) == math.pi


def test_segment_motion_pure_forward():
    start = PlanarPose(1.0, 2.0, 0.0)
    end = PlanarPose(6.0, 2.0, 0.0)

    forward, lateral, dyaw = segment_motion(start, end)

    assert math.isclose(forward, 5.0)
    assert math.isclose(lateral, 0.0, abs_tol=1e-9)
    assert math.isclose(dyaw, 0.0, abs_tol=1e-9)


def test_segment_motion_measured_in_start_heading_frame():
    start = PlanarPose(0.0, 0.0, math.pi / 2.0)
    end = PlanarPose(-5.0, 0.0, math.pi / 2.0)

    forward, lateral, dyaw = segment_motion(start, end)

    assert math.isclose(forward, 0.0, abs_tol=1e-9)
    assert math.isclose(lateral, 5.0)
    assert math.isclose(dyaw, 0.0, abs_tol=1e-9)


def test_segment_motion_pure_rotation_wraps_yaw():
    start = PlanarPose(0.0, 0.0, math.pi - 0.1)
    end = PlanarPose(0.0, 0.0, -math.pi + 0.1)

    forward, lateral, dyaw = segment_motion(start, end)

    assert math.isclose(forward, 0.0, abs_tol=1e-9)
    assert math.isclose(lateral, 0.0, abs_tol=1e-9)
    assert math.isclose(dyaw, 0.2)


def test_segment_motion_frame_independence_for_same_motion():
    # Same physical 5 m forward drive expressed in two source frames whose
    # start headings differ by 90 degrees must give the same triple.
    odom = segment_motion(PlanarPose(0.0, 0.0, 0.0), PlanarPose(5.0, 0.0, 0.0))
    gt = segment_motion(
        PlanarPose(0.45, -5.35, math.pi / 2.0),
        PlanarPose(0.45, -0.35, math.pi / 2.0),
    )

    assert math.isclose(odom[0], gt[0])
    assert math.isclose(odom[1], gt[1], abs_tol=1e-9)
    assert math.isclose(odom[2], gt[2], abs_tol=1e-9)


def test_segment_motion_rejects_nothing_but_stays_finite_on_zero_motion():
    forward, lateral, dyaw = segment_motion(
        PlanarPose(1.0, 1.0, 0.7), PlanarPose(1.0, 1.0, 0.7))

    assert forward == 0.0
    assert lateral == 0.0
    assert dyaw == 0.0


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__]))
