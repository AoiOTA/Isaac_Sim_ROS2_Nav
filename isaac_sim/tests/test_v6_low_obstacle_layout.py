from __future__ import annotations

import math
from pathlib import Path

from PIL import Image
import numpy as np
import pytest
import yaml

from isaac_sim.src.experiment.scenario import load_dynamic_scenario


ROOT = Path(__file__).resolve().parents[2]
EXPERIMENTS = ROOT / "isaac_sim/configs/experiments"
OBSTACLE_CONFIG = EXPERIMENTS / "v6_kujiale_low_obstacles_frozen.yaml"
MANIFEST = EXPERIMENTS / "v6_kujiale_low_obstacles_frozen_manifest.yaml"
RUNNER_SCENARIO = (
    ROOT
    / "ros2_ws/src/robot_experiments/config/v6_kujiale_low_obstacles_static.yaml"
)


def _manifest():
    return yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))


def _map():
    manifest = _manifest()
    source = (MANIFEST.parent / manifest["occupancy_map"]).resolve()
    metadata = yaml.safe_load(source.read_text(encoding="utf-8"))
    image = np.asarray(
        Image.open(source.parent / metadata["image"]).convert("L")
    )
    return metadata, image


def _map_cell(metadata, image, x, y):
    origin_x, origin_y, _ = metadata["origin"]
    resolution = float(metadata["resolution"])
    height, width = image.shape
    column = int((x - origin_x) / resolution)
    row = height - 1 - int((y - origin_y) / resolution)
    assert 0 <= column < width and 0 <= row < height
    return int(image[row, column])


def _rectangle_is_free(metadata, image, position, size):
    resolution = float(metadata["resolution"])
    xs = np.arange(
        position[0] - size[0] / 2.0,
        position[0] + size[0] / 2.0 + resolution / 4.0,
        resolution / 2.0,
    )
    ys = np.arange(
        position[1] - size[1] / 2.0,
        position[1] + size[1] / 2.0 + resolution / 4.0,
        resolution / 2.0,
    )
    return all(
        _map_cell(metadata, image, float(x), float(y)) >= 250
        for x in xs
        for y in ys
    )


def _open_side_clearance(metadata, image, position, size, axis, direction):
    resolution = float(metadata["resolution"])
    half_extent = size[0 if axis == 0 else 1] / 2.0
    distance = half_extent
    while distance < half_extent + 10.0:
        probe = [position[0], position[1]]
        probe[axis] += direction * distance
        if _map_cell(metadata, image, *probe) < 250:
            return distance - half_extent
        distance += resolution / 4.0
    return 10.0


def _point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    ratio = max(
        0.0,
        min(
            1.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
            / (dx * dx + dy * dy),
        ),
    )
    closest = (start[0] + ratio * dx, start[1] + ratio * dy)
    return math.dist(point[:2], closest)


def _ccw(ax, ay, bx, by, cx, cy):
    return (by - ay) * (cx - ax) - (bx - ax) * (cy - ay)


def _segments_intersect(a, b, c, d):
    d1 = _ccw(c[0], c[1], d[0], d[1], a[0], a[1])
    d2 = _ccw(c[0], c[1], d[0], d[1], b[0], b[1])
    d3 = _ccw(a[0], a[1], b[0], b[1], c[0], c[1])
    d4 = _ccw(a[0], a[1], b[0], b[1], d[0], d[1])
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))


def _segment_rectangle_distance(start, end, position, size):
    """Exact distance between a segment and an axis-aligned footprint."""
    half_x, half_y = size[0] / 2.0, size[1] / 2.0
    cx, cy = position[0], position[1]
    corners = [
        (cx - half_x, cy - half_y),
        (cx + half_x, cy - half_y),
        (cx + half_x, cy + half_y),
        (cx - half_x, cy + half_y),
    ]
    for point in (start, end):
        if abs(point[0] - cx) <= half_x and abs(point[1] - cy) <= half_y:
            return 0.0
    edges = list(zip(corners, corners[1:] + corners[:1]))
    if any(_segments_intersect(start, end, edge[0], edge[1]) for edge in edges):
        return 0.0
    return min(
        min(
            _point_segment_distance(edge[0], start, end),
            _point_segment_distance(edge[1], start, end),
            _point_segment_distance(start, edge[0], edge[1]),
            _point_segment_distance(end, edge[0], edge[1]),
        )
        for edge in edges
    )


def test_v6_layout_identity_is_frozen_and_separate_from_the_source_usd():
    manifest = _manifest()
    assert manifest["status"] == "frozen"
    assert "draft" not in manifest["layout_id"]
    assert manifest["revision"] == 2
    assert manifest["frozen_date"] == "2026-08-26"
    assert manifest["source_usd_mutated"] is False
    assert manifest["source_usd"] == "kujiale_0026_A_to_B_door_open.usd"
    assert manifest["activation_profile"] == "v6-low-obstacles"


def test_v6_obstacles_are_default_off_unique_stationary_and_height_is_causal():
    scenario = load_dynamic_scenario(OBSTACLE_CONFIG)
    manifest = _manifest()
    geometry = manifest["geometry_contract"]
    sensors = manifest["sensors"]
    ids = [item.obstacle_id for item in scenario.obstacles]

    assert scenario.enabled is False
    assert len(ids) == len(set(ids)) == 1
    assert ids == ["v6_low_box_solo"]
    assert all(item.mode == "stationary" for item in scenario.obstacles)
    assert all(item.start == item.end for item in scenario.obstacles)
    assert all(item.start[2] - item.size[2] / 2.0 == pytest.approx(0.0)
               for item in scenario.obstacles)
    assert all(item.start[2] + item.size[2] / 2.0 == pytest.approx(0.16)
               for item in scenario.obstacles)
    assert geometry["obstacle_top_z_m"] < sensors["rgbd_origin_z_m"]
    assert geometry["obstacle_top_z_m"] < sensors["lidar_plane_z_m"]
    wheel_low, wheel_high = geometry["wheel_vertical_extent_m"]
    assert max(wheel_low, geometry["obstacle_bottom_z_m"]) < min(
        wheel_high, geometry["obstacle_top_z_m"]
    )


def test_v6_obstacle_is_on_free_map_with_open_bypass():
    scenario = load_dynamic_scenario(OBSTACLE_CONFIG)
    manifest = _manifest()
    geometry = manifest["geometry_contract"]
    metadata, image = _map()

    for obstacle in scenario.obstacles:
        assert _rectangle_is_free(
            metadata, image, obstacle.start, obstacle.size
        ), obstacle.obstacle_id
        clearances = [
            _open_side_clearance(
                metadata, image, obstacle.start, obstacle.size, axis, direction
            )
            for axis in (0, 1)
            for direction in (-1, 1)
        ]
        assert max(clearances) >= geometry["minimum_open_bypass_side_m"]


def test_v6_layout_challenges_g1_g2_and_keeps_measured_east_bypass():
    scenario = load_dynamic_scenario(OBSTACLE_CONFIG)
    manifest = _manifest()
    runner = yaml.safe_load(RUNNER_SCENARIO.read_text(encoding="utf-8"))[
        "scenario"
    ]
    obstacle = scenario.obstacles[0]
    placement = manifest["causal_placement"]
    assert obstacle.start[:2] == pytest.approx((-0.45, -0.35))
    assert placement["route"] == "G1_to_G2"
    assert placement["ground_truth_center_min_distance_m"] <= 0.02
    assert placement["compute_center_min_distance_m"] <= 0.02
    assert placement["padded_overlap_interval_sec"][1] > \
        placement["padded_overlap_interval_sec"][0]
    assert placement["initial_spawn_clearance_m"] > 4.0
    assert placement["static_boundary_clearance_m"] >= 0.65
    lateral_clearance = (
        abs(placement["east_bypass_robot_center_x_m"] - obstacle.start[0])
        - obstacle.size[0] / 2.0
        - manifest["geometry_contract"]["robot_lateral_half_width_m"]
    )
    assert lateral_clearance == pytest.approx(
        placement["east_bypass_box_clearance_m"], abs=1.0e-4
    )
    assert placement["east_bypass_static_clearance_m"] >= 0.20
    metadata, image = _map()
    robot_size = [
        2.0 * manifest["geometry_contract"]["robot_lateral_half_width_m"],
        manifest["geometry_contract"]["robot_max_footprint_dimension_m"],
    ]
    assert _rectangle_is_free(
        metadata,
        image,
        [placement["east_bypass_robot_center_x_m"], obstacle.start[1]],
        robot_size,
    )

    map_yaml = (MANIFEST.parent / manifest["occupancy_map"]).resolve().read_text(
        encoding="utf-8"
    )
    assert all(item.obstacle_id not in map_yaml for item in scenario.obstacles)
    assert runner["map_version"] == runner["posegraph_version"] == \
        "v6_kujiale_isaacgen_v1"
    assert runner["spawn_pose_name"] == "long_route_start_g1"
    assert runner["goal"]["position"] == [0.45, -5.35]


def test_v6_box_is_depth_visible_but_below_scan_plane():
    manifest = _manifest()
    placement = manifest["causal_placement"]
    sensors = manifest["sensors"]
    geometry = manifest["geometry_contract"]
    assert placement["rgbd_first_line_of_sight_sec"] < \
        placement["rgbd_within_2m_sec"]
    assert placement["lidar_plane_above_obstacle"] is True
    assert geometry["obstacle_top_z_m"] < sensors["lidar_plane_z_m"]
    assert geometry["obstacle_top_z_m"] < sensors["rgbd_origin_z_m"]


def test_map_to_usd_manifest_matches_the_calibrated_180_degree_transform():
    manifest = _manifest()
    assert manifest["map_to_usd"] == {
        "x": "2.9-map_x",
        "y": "-0.2-map_y",
        "yaw_offset_deg": 180.0,
    }
    for obstacle in manifest["obstacles"]:
        map_position = obstacle["map_position"]
        usd_position = obstacle["usd_position"]
        assert usd_position == pytest.approx(
            [2.9 - map_position[0], -0.2 - map_position[1], map_position[2]]
        )


def test_explicit_profile_keeps_rgbd_topics_but_not_direct_costmap_input():
    manifest = _manifest()
    camera = yaml.safe_load(
        (ROOT / "isaac_sim/configs/sensors/camera.yaml").read_text(
            encoding="utf-8"
        )
    )
    profile = camera["profiles"]["rgbd_navigation"]
    front = camera["cameras"]["front"]
    overlay = (
        ROOT
        / "ros2_ws/src/robot_navigation/config/"
        "nav2_v6_low_obstacle_isolation.yaml"
    ).read_text(encoding="utf-8")
    wrapper = (
        ROOT / "scripts/run_v6_kujiale_low_obstacles.sh"
    ).read_text(encoding="utf-8")
    default_isaac = (
        ROOT / "scripts/run_kujiale_4x20_isaac.sh"
    ).read_text(encoding="utf-8")

    assert profile["enabled"] is True
    assert profile["depth_points_enabled"] is True
    assert front["rgb"]["enabled"] is True
    assert front["depth"]["enabled"] is True
    assert front["depth_points"]["enabled"] is True
    assert manifest["sensors"]["depth_points_direct_to_costmap"] is False
    assert "/camera/front/depth/points" not in overlay
    assert (
        "plugins: [obstacle_layer, cognitive_obstacle_layer, inflation_layer]"
        in overlay
    )
    assert (
        "plugins: [static_layer, obstacle_layer, cognitive_obstacle_layer, inflation_layer]"
        in overlay
    )
    assert "run_v6_r5_phase_b_kujiale.sh\" isaac" in wrapper
    assert "--dynamic-obstacles" in wrapper
    assert "odometry_mode:=mixed" in wrapper
    assert "v6_low_obstacle_isolation" in wrapper
    assert "--camera-profile rgbd_navigation" in default_isaac
    static_branch, dynamic_branch = default_isaac.split("  dynamic)", 1)
    dynamic_branch, v6_branch = dynamic_branch.split("  v6-low-obstacles)", 1)
    assert "kujiale_long_range_static.yaml" in static_branch
    assert "kujiale_long_range_dynamic.yaml" in dynamic_branch
    assert "v6_kujiale_low_obstacles_frozen.yaml" in v6_branch


def test_legacy_4x20_profile_is_unchanged_but_phase_f_uses_phase_b_mixed():
    source = (
        ROOT / "scripts/run_kujiale_4x20_isaac.sh"
    ).read_text(encoding="utf-8")
    before_case, case_body = source.split('case "${mode}" in', 1)
    static_branch, after_static = case_body.split("  dynamic)", 1)
    dynamic_branch, v6_branch = after_static.split("  v6-low-obstacles)", 1)

    assert 'odometry_mode="ideal"' in before_case
    assert 'odometry_mode="realistic"' not in static_branch
    assert 'odometry_mode="realistic"' not in dynamic_branch
    assert 'odometry_mode="realistic"' in v6_branch
    assert '--mode "${odometry_mode}"' in source
    assert source.index('--mode "${odometry_mode}"') < source.index('"$@"')
    wrapper = (ROOT / "scripts/run_v6_kujiale_low_obstacles.sh").read_text(
        encoding="utf-8"
    )
    assert "run_v6_r5_phase_b_kujiale.sh\" isaac" in wrapper
    assert "odometry_mode:=mixed" in wrapper
