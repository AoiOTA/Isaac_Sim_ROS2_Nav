from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from isaac_sim.src.experiment.scenario import load_dynamic_scenario


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
