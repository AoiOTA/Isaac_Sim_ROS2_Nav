from __future__ import annotations

from hashlib import sha256
import math
from pathlib import Path

import pytest
import yaml

from isaac_sim.src.experiment.scenario import load_dynamic_scenario
from isaac_sim.src.robot.spawn_pose_manager import load_spawn_poses
from isaac_sim.tools.v6_calibration_flat_20m_generate import generate_map
from isaac_sim.tools.v6_calibration_flat_20m_generate import load_rectangles


ROOT = Path(__file__).resolve().parents[2]
EXTERNAL_GRID = Path(
    "/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Grid/"
    "default_environment.usd"
)
FEATURES = ROOT / "isaac_sim/configs/experiments/v6_calibration_grid_features.yaml"
SPAWN = ROOT / "isaac_sim/configs/environments/v6_calibration_flat_20m.spawn.yaml"
MAP_YAML = ROOT / "data/maps/v6_calibration_flat_20m.yaml"
MAP_PGM = ROOT / "data/maps/v6_calibration_flat_20m.pgm"
MANIFEST = ROOT / "data/maps/manifests/v6_calibration_flat_20m.yaml"


def _pgm(payload: bytes) -> tuple[int, int, bytes]:
    magic, dimensions, maximum, pixels = payload.split(b"\n", 3)
    assert magic == b"P5" and maximum == b"255"
    width, height = (int(value) for value in dimensions.split())
    return width, height, pixels


def _distance_to_rectangle(x, y, rectangle):
    center_x, center_y, size_x, size_y = rectangle
    dx = max(abs(x - center_x) - size_x / 2.0, 0.0)
    dy = max(abs(y - center_y) - size_y / 2.0, 0.0)
    return math.hypot(dx, dy)


def _primitive_samples():
    samples = [(step * 0.03, 0.0) for step in range(101)]
    x = y = yaw = 0.0
    samples.append((x, y))
    for duration, linear, angular in ((2.5, 0.25, 0.45), (5.0, 0.25, -0.45), (2.5, 0.25, 0.45)):
        steps = int(round(duration / 0.01))
        for _ in range(steps):
            x += linear * math.cos(yaw) * 0.01
            y += linear * math.sin(yaw) * 0.01
            yaw += angular * 0.01
            samples.append((x, y))
    return samples


def test_external_grid_is_the_only_environment_asset_and_spawn_is_identity():
    assert EXTERNAL_GRID.is_file()
    assert EXTERNAL_GRID.read_bytes().startswith(b"PXR-USDC")
    manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    assert manifest["source"]["external_environment_asset"] == str(EXTERNAL_GRID)
    assert not (ROOT / "isaac_sim/assets/environments/v6_calibration_flat_20m.usda").exists()
    pose = load_spawn_poses(SPAWN)["mapping_start"]
    diagnostic_pose = load_spawn_poses(SPAWN)["flat20_start"]
    assert diagnostic_pose.usd == pose.usd
    assert diagnostic_pose.map == pose.map
    assert pose.usd.position == (0.0, 0.0, 0.0635)
    assert pose.usd.yaw_deg == pose.map.yaw_deg == 0.0
    assert pose.map.position == (0.0, 0.0)
    assert pose.map.calibrated is True
    assert pose.map.map_version == "flat20_v1"


def test_stationary_runtime_geometry_has_exact_walls_and_asymmetric_features():
    scenario = load_dynamic_scenario(FEATURES)
    assert scenario.enabled is True
    assert scenario.coordinate_frame == "map"
    assert scenario.spawn_pose_name == "mapping_start"
    assert len(scenario.obstacles) == 7
    assert all(item.mode == "stationary" and item.start == item.end for item in scenario.obstacles)
    assert scenario.seed == 20260821
    assert all(item.speed == 0.0 for item in scenario.obstacles)
    assert all(
        item.start[2] - item.size[2] / 2.0 <= 0.333
        < item.start[2] + item.size[2] / 2.0
        for item in scenario.obstacles
    )
    walls = scenario.obstacles[:4]
    assert [item.obstacle_id for item in walls] == [
        "flat20_wall_west", "flat20_wall_east",
        "flat20_wall_south", "flat20_wall_north",
    ]
    assert [item.size for item in walls] == [
        (0.2, 20.2, 1.0), (0.2, 20.2, 1.0),
        (20.2, 0.2, 1.0), (20.2, 0.2, 1.0),
    ]
    features = scenario.obstacles[4:]
    assert [item.start[:2] for item in features] == [(-6.0, -4.0), (6.0, 5.0), (-3.0, 7.0)]
    assert len({item.size for item in features}) == 3
    assert all(item.start[2] - item.size[2] / 2.0 <= 0.333 < item.start[2] + item.size[2] / 2.0 for item in features)


def test_map_dimensions_origin_and_pixels_are_generated_from_exact_geometry():
    map_data = yaml.safe_load(MAP_YAML.read_text(encoding="utf-8"))
    assert map_data["resolution"] == pytest.approx(0.05)
    assert map_data["origin"] == [-10.1, -10.1, 0.0]
    width, height, pixels = _pgm(MAP_PGM.read_bytes())
    assert (width, height, len(pixels)) == (404, 404, 404 * 404)
    assert MAP_PGM.read_bytes() == generate_map(load_rectangles(FEATURES))


def test_map_bundle_binding_covers_map_and_feature_geometry():
    manifest = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    pose = load_spawn_poses(SPAWN)["mapping_start"]
    digest = sha256()
    for path in (MAP_PGM, MAP_YAML, FEATURES):
        digest.update(path.read_bytes())
    assert pose.map.map_bundle_sha256 == digest.hexdigest()
    assert manifest["calibration"]["map_bundle_sha256"] == digest.hexdigest()


def test_three_meter_and_s_route_envelopes_remain_open():
    rectangles = load_rectangles(FEATURES)
    for x, y in _primitive_samples():
        assert max(abs(x), abs(y)) < 4.0
        assert min(_distance_to_rectangle(x, y, rectangle) for rectangle in rectangles) > 1.5
