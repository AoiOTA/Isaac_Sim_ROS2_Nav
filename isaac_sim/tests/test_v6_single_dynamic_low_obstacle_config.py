from pathlib import Path

import pytest

from isaac_sim.src.experiment.dynamic_obstacles import DynamicObstacleManager
from isaac_sim.src.experiment.scenario import load_dynamic_scenario


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "isaac_sim/configs/experiments/v6_single_dynamic_low_obstacle.yaml"


def test_single_dynamic_low_box_geometry_motion_and_visibility_contract():
    scenario = load_dynamic_scenario(CONFIG)
    case = scenario.cases["single_dynamic_low_box"]
    actor = case.obstacle

    assert scenario.enabled is False
    assert scenario.coordinate_frame == "map"
    assert scenario.seed == 8601
    assert actor.obstacle_id == "v6_dynamic_low_box_solo"
    assert actor.size == pytest.approx((0.30, 0.30, 0.16))
    assert case.waypoints[0] == pytest.approx((-1.25, -0.35, 0.08))
    assert case.waypoints[1] == pytest.approx((-0.45, -0.35, 0.08))
    assert actor.speed == pytest.approx(0.25)
    assert case.max_acceleration == pytest.approx(0.50)
    assert actor.post_motion == "park"
    assert case.variant("v1").seed == 8601
    assert tuple(item.variant_id for item in case.variants) == (
        "v1", "v2", "v3", "v4", "v5",
    )
    assert tuple(item.seed for item in case.variants) == (8601,) * 5
    assert tuple(item.start_delay_sec for item in case.variants) == pytest.approx(
        (0.0, 0.15, 0.30, 0.45, 0.60)
    )
    lower = actor.start[2] - actor.size[2] / 2.0
    upper = actor.start[2] + actor.size[2] / 2.0
    assert lower == pytest.approx(0.0)
    assert upper == pytest.approx(0.16)
    assert upper < 0.333


def test_single_dynamic_low_box_gate_requires_the_frozen_g2_approach():
    case = load_dynamic_scenario(CONFIG).cases["single_dynamic_low_box"]
    manager = object.__new__(DynamicObstacleManager)

    assert manager._gate_passed(
        case, {"x": 0.50, "y": -2.59, "vy": 0.25, "speed": 0.25}
    )
    assert not manager._gate_passed(
        case, {"x": 0.50, "y": -2.61, "vy": 0.25, "speed": 0.25}
    )
    assert not manager._gate_passed(
        case, {"x": 0.29, "y": -2.59, "vy": 0.25, "speed": 0.25}
    )
    assert not manager._gate_passed(
        case, {"x": 0.50, "y": -2.59, "vy": -0.25, "speed": 0.25}
    )
    assert not manager._gate_passed(
        case, {"x": 0.50, "y": -2.59, "vy": 0.19, "speed": 0.19}
    )


def test_single_dynamic_low_box_profile_reaches_park_point_with_limits():
    case = load_dynamic_scenario(CONFIG).cases["single_dynamic_low_box"]
    manager = object.__new__(DynamicObstacleManager)
    start = manager._trajectory(case, case.variant("v1"), 0.0)
    _, _, duration = manager._profile(
        0.80, case.obstacle.speed, case.max_acceleration, 0.0
    )
    finish = manager._trajectory(case, case.variant("v1"), duration + 0.01)

    assert start[0] == pytest.approx((-1.25, -0.35, 0.08))
    assert start[1] == pytest.approx(0.0)
    assert finish[0] == pytest.approx((-0.45, -0.35, 0.08))
    assert finish[1] == pytest.approx(0.0)
    assert finish[2] == pytest.approx(1.0)
    assert finish[3] == "clearing"
