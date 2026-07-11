from __future__ import annotations

import math

import pytest

from isaac_sim.src.ground_truth.transforms import (
    Pose2D,
    compose,
    compute_map_t_usd,
    inverse,
    usd_pose_to_map,
)


def test_pose_inverse_round_trip_is_identity():
    pose = Pose2D(2.0, -3.0, math.radians(37.0))
    identity = compose(pose, inverse(pose))
    assert identity.x == pytest.approx(0.0, abs=1e-12)
    assert identity.y == pytest.approx(0.0, abs=1e-12)
    assert identity.yaw == pytest.approx(0.0, abs=1e-12)


def test_map_alignment_matches_calibrated_start_and_relative_motion():
    usd_start = Pose2D.from_degrees(4.0, 1.0, 90.0)
    map_start = Pose2D.from_degrees(-2.0, 3.0, -30.0)
    map_t_usd = compute_map_t_usd(usd_start, map_start)
    aligned = usd_pose_to_map(map_t_usd, usd_start)
    assert aligned.x == pytest.approx(map_start.x)
    assert aligned.y == pytest.approx(map_start.y)
    assert aligned.yaw == pytest.approx(map_start.yaw)
