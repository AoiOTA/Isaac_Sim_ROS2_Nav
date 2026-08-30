import hashlib
import json
import math
from pathlib import Path

import pytest
import yaml

import robot_experiments.kujiale_reference as kujiale_reference
from robot_experiments.kujiale_reference import build_v6_optimal_reference


ROOT = Path(__file__).resolve().parents[4]
CONFIG = ROOT / "ros2_ws/src/robot_experiments/config"
SCENARIO = CONFIG / "v6_final_kujiale_static.yaml"
SPAWN = ROOT / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
OBSTACLES = ROOT / "isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen.yaml"
MAP = ROOT / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml"
FROZEN = CONFIG / "v6_kujiale_isaacgen_v1_low_box_solo_optimal_reference.json"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_v6_frozen_reference_matches_deterministic_regeneration():
    result = build_v6_optimal_reference(SCENARIO, SPAWN, OBSTACLES, MAP)
    frozen_bytes = FROZEN.read_bytes()

    assert frozen_bytes == (
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n"
    ).encode("utf-8")
    assert json.loads(frozen_bytes) == result
    assert [item["id"] for item in result["legs"]] == ["G2", "G3", "G4", "G5", "G1"]
    for resolution in ("length_m_0_05", "length_m_0_025"):
        lengths = [float(item[resolution]) for item in result["legs"]]
        assert all(math.isfinite(value) and value > 0.0 for value in lengths)
        assert result[f"total_{resolution}"] == pytest.approx(sum(lengths))
    assert result["convergence_percent"] <= 1.0
    assert result["converged"] is True
    assert result["static_obstacle_polygons"] == [
        [[-0.9, -0.5], [-0.6, -0.5], [-0.6, -0.2], [-0.9, -0.2]]
    ]

    algorithm = result["algorithm"]
    assert algorithm["generator"] == "robot_experiments/kujiale_reference.py"
    assert algorithm["generator_sha256"] == _sha256(
        ROOT / "ros2_ws/src/robot_experiments/robot_experiments/kujiale_reference.py"
    )
    assert algorithm["optimal_path"] == "robot_experiments/optimal_path.py"
    assert algorithm["optimal_path_sha256"] == _sha256(
        ROOT / "ros2_ws/src/robot_experiments/robot_experiments/optimal_path.py"
    )
    assert result["scenario"] == {
        "id": "v6_final_kujiale_static",
        "file": SCENARIO.name,
        "sha256": _sha256(SCENARIO),
        "spawn_pose_name": "long_route_start_g1",
        "spawn_file": SPAWN.name,
        "spawn_sha256": _sha256(SPAWN),
        "obstacle_layout_id": "kujiale_v6_low_obstacles_indoor_center_connected_r3_20260829",
        "obstacle_id": "v6_low_box_solo",
        "obstacle_file": OBSTACLES.name,
        "obstacle_sha256": _sha256(OBSTACLES),
    }
    assert result["map"]["yaml"] == MAP.name
    assert result["map"]["yaml_sha256"] == _sha256(MAP)
    map_image = MAP.parent / result["map"]["image"]
    assert result["map"]["image_sha256"] == _sha256(map_image)
    path_values = (
        algorithm["generator"],
        algorithm["optimal_path"],
        result["scenario"]["file"],
        result["scenario"]["spawn_file"],
        result["scenario"]["obstacle_file"],
        result["map"]["yaml"],
        result["map"]["image"],
    )
    assert all(not Path(value).is_absolute() for value in path_values)


def test_static_scenario_binds_packaged_v6_reference():
    scenario = yaml.safe_load(SCENARIO.read_text(encoding="utf-8"))["scenario"]
    assert scenario["configs"]["optimal_reference"] == FROZEN.name


def test_legacy_campaign_cli_remains_available(monkeypatch, tmp_path):
    campaign = CONFIG / "kujiale_long_range_campaign.yaml"
    output = tmp_path / "legacy.json"
    expected = {
        "converged": True,
        "legs": [{"id": "G2", "length_m_0_05": 1.0}],
        "total_length_m_0_05": 1.0,
    }
    calls = []

    def fake_build(campaign_file, map_file):
        calls.append((Path(campaign_file), Path(map_file)))
        return expected

    monkeypatch.setattr(kujiale_reference, "build_optimal_reference", fake_build)
    with pytest.raises(SystemExit) as exit_info:
        kujiale_reference.main(
            [
                "--campaign-file",
                str(campaign),
                "--map-file",
                str(MAP),
                "--output",
                str(output),
            ]
        )

    assert exit_info.value.code == 0
    assert calls == [(campaign, MAP)]
    assert json.loads(output.read_text(encoding="utf-8")) == expected


def test_cli_rejects_mixed_legacy_and_v6_sources(tmp_path):
    with pytest.raises(SystemExit) as exit_info:
        kujiale_reference.main(
            [
                "--campaign-file",
                str(CONFIG / "kujiale_long_range_campaign.yaml"),
                "--scenario-file",
                str(SCENARIO),
                "--map-file",
                str(MAP),
                "--output",
                str(tmp_path / "invalid.json"),
            ]
        )
    assert exit_info.value.code == 2
