from __future__ import annotations

import math
from pathlib import Path

import pytest
import yaml

from isaac_sim.src.experiment.scenario import load_dynamic_scenario
from isaac_sim.src.experiment.dynamic_obstacles import DynamicObstacleManager


ROOT = Path(__file__).resolve().parents[2]


def test_dynamic_obstacle_phases_are_seeded_and_repeatable():
    scenario = load_dynamic_scenario(ROOT / "isaac_sim/configs/experiments/dynamic.yaml")
    assert scenario.sampled_phases(123) == scenario.sampled_phases(123)
    assert scenario.sampled_phases(123) != scenario.sampled_phases(124)
    assert set(scenario.sampled_phases()) == {item.obstacle_id for item in scenario.obstacles}
    assert scenario.enabled is False


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("size", [0.5, 0.0, 1.0], "size values must be positive"),
        ("phase_jitter", -0.1, "must be non-negative"),
        ("speed", float("nan"), "must be finite"),
        ("repeat", "false", "repeat must be boolean"),
    ],
)
def test_invalid_dynamic_obstacle_physics_is_rejected(
    tmp_path, field, value, message
):
    source = ROOT / "isaac_sim/configs/experiments/dynamic.yaml"
    document = yaml.safe_load(source.read_text())
    document["obstacles"][0][field] = value
    target = tmp_path / "dynamic.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        load_dynamic_scenario(target)


def test_zero_length_dynamic_trajectory_is_rejected(tmp_path):
    source = ROOT / "isaac_sim/configs/experiments/dynamic.yaml"
    document = yaml.safe_load(source.read_text())
    document["obstacles"][0]["end"] = document["obstacles"][0]["start"]
    target = tmp_path / "dynamic.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="non-zero length"):
        load_dynamic_scenario(target)


def test_dynamic_baseline_uses_one_shot_trajectories():
    scenario = load_dynamic_scenario(
        ROOT / "isaac_sim/configs/experiments/dynamic.yaml"
    )
    assert scenario.obstacles
    assert all(not obstacle.repeat for obstacle in scenario.obstacles)


def test_dynamic_profile_is_smooth_and_honours_motion_limits():
    distance, max_speed, max_acceleration = 0.80, 0.40, 0.50
    start = DynamicObstacleManager._profile(
        distance, max_speed, max_acceleration, 0.0
    )
    _, _, duration = start
    middle = DynamicObstacleManager._profile(
        distance, max_speed, max_acceleration, duration / 2.0
    )
    finish = DynamicObstacleManager._profile(
        distance, max_speed, max_acceleration, duration
    )

    assert duration == pytest.approx(math.pi, abs=1e-6)
    assert start[:2] == pytest.approx((0.0, 0.0))
    assert middle[:2] == pytest.approx((distance / 2.0, max_speed))
    assert finish[:2] == pytest.approx((distance, 0.0))
