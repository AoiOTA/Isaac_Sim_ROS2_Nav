from __future__ import annotations

import math

from isaac_sim.src.diagnostics.r2c2_free_space_envelope import (
    Bounds3D,
    Collider,
    REQUIRED_CLEARANCE_M,
    assess_envelope,
    classify_collider,
    polygon_aabb_distance,
    transform_footprint,
)


FOOTPRINT = [[0.255, 0.210], [0.255, -0.210], [-0.230, -0.210], [-0.230, 0.210]]


def _collider(path: str, values: tuple[float, float, float, float, float, float]) -> Collider:
    return Collider(path, Bounds3D(*values), True)


def test_thick_floor_and_ceiling_are_not_lateral_obstacles():
    floor = _collider("/floor", (-5.0, -5.0, -0.55, 5.0, 5.0, -0.05))
    ceiling = _collider("/ceiling", (-5.0, -5.0, 2.64, 5.0, 5.0, 2.80))
    wall = _collider("/wall", (2.0, -1.0, -0.01, 2.2, 1.0, 1.0))
    assert classify_collider(floor, support_plane_z=-0.0345, robot_max_z=0.60) == "SUPPORT"
    assert classify_collider(ceiling, support_plane_z=-0.0345, robot_max_z=0.60) == "OVERHEAD"
    assert classify_collider(wall, support_plane_z=-0.0345, robot_max_z=0.60) == "LATERAL_CANDIDATE"


def test_envelope_requires_full_support_and_uses_only_vertical_overlap():
    floor = _collider("/floor", (-5.0, -5.0, -0.55, 5.0, 5.0, -0.05))
    ceiling = _collider("/ceiling", (-5.0, -5.0, 2.64, 5.0, 5.0, 2.80))
    table = _collider("/table", (2.0, -1.0, 0.0, 2.4, 1.0, 0.5))
    classified, segments = assess_envelope(
        footprint=FOOTPRINT, start_x=0.0, start_y=0.0, start_yaw=0.0,
        support_plane_z=-0.0345, robot_max_z=0.60,
        colliders=[floor, ceiling, table],
    )
    kinds = {item["path"]: item["classification"] for item in classified}
    assert kinds == {"/floor": "SUPPORT", "/ceiling": "OVERHEAD", "/table": "LATERAL_CANDIDATE"}
    assert all(item.support_coverage == 1.0 for item in segments)
    assert min(item.minimum_clearance_m for item in segments) >= REQUIRED_CLEARANCE_M


def test_support_gap_and_low_lateral_obstacle_fail_closed():
    partial_floor = _collider("/floor", (-5.0, -5.0, -0.55, 0.1, 5.0, -0.05))
    low_box = _collider("/low-box", (0.35, -0.10, 0.0, 0.5, 0.10, 0.3))
    _, segments = assess_envelope(
        footprint=FOOTPRINT, start_x=0.0, start_y=0.0, start_yaw=0.0,
        support_plane_z=-0.0345, robot_max_z=0.60,
        colliders=[partial_floor, low_box],
    )
    assert any(item.support_coverage < 1.0 for item in segments)
    assert any(item.minimum_clearance_m < REQUIRED_CLEARANCE_M for item in segments)
    assert not all(item.valid for item in segments)


def test_aggregate_disabled_and_nonfinite_contracts():
    aggregate = Collider("/aggregate", Bounds3D(-1, -1, 0, 1, 1, 1), True, aggregate=True)
    disabled = Collider("/disabled", Bounds3D(-1, -1, 0, 1, 1, 1), False)
    assert classify_collider(aggregate, support_plane_z=0.0, robot_max_z=0.6) == "AGGREGATE_EXCLUDED"
    assert classify_collider(disabled, support_plane_z=0.0, robot_max_z=0.6) == "DISABLED"
    classified, segments = assess_envelope(
        footprint=FOOTPRINT, start_x=0.0, start_y=0.0, start_yaw=0.0,
        support_plane_z=0.0, robot_max_z=0.6,
        colliders=[Collider("/bad", Bounds3D(0, 0, 0, math.nan, 1, 1), True)],
    )
    assert classified[0]["classification"] == "INVALID"
    assert classified[0]["bounds"] == {"min": None, "max": None}
    assert all(not item.valid and item.minimum_clearance_m == 0.0 for item in segments)


def test_rotated_polygon_clearance_and_threshold_edge():
    polygon = transform_footprint(FOOTPRINT, 0.0, 0.0, math.pi / 4.0)
    exact = Bounds3D(0.455, -0.01, 0.0, 0.555, 0.01, 0.5)
    assert polygon_aabb_distance(polygon, exact) >= 0.0
    assert polygon_aabb_distance(transform_footprint(FOOTPRINT, 0.0, 0.0, 0.0), Bounds3D(0.455, -0.1, 0, 0.50, 0.1, 1)) == 0.20
