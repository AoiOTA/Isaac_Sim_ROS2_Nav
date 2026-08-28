from pathlib import Path
import json

import pytest

from robot_experiments.rivermark_reference import build_rivermark_reference


ROOT = Path(__file__).resolve().parents[4]


def test_rivermark_reference_sums_all_five_route_legs_and_converges():
    result = build_rivermark_reference(
        ROOT / "ros2_ws/src/robot_experiments/config/attempt31_rivermark_static.yaml",
        ROOT / "data/rivermark_demo/rivermark.spawn.yaml",
        ROOT / "data/rivermark_demo/rivermark_selected.yaml",
    )

    assert [item["id"] for item in result["legs"]] == [
        "G1", "G2", "G3", "G4", "G5"
    ]
    assert result["total_length_m_0_05"] == pytest.approx(
        sum(item["length_m_0_05"] for item in result["legs"])
    )
    assert result["total_length_m_0_05"] == pytest.approx(113.05615792887396)
    assert result["convergence_percent"] < 1.0
    assert result["converged"] is True
    provenance_paths = {
        "map.yaml": result["map"]["yaml"],
        "map.image": result["map"]["image"],
        "scenario.file": result["scenario"]["file"],
        "scenario.spawn_file": result["scenario"]["spawn_file"],
    }
    assert provenance_paths == {
        "map.yaml": "rivermark_selected.yaml",
        "map.image": "rivermark_selected.pgm",
        "scenario.file": "attempt31_rivermark_static.yaml",
        "scenario.spawn_file": "rivermark.spawn.yaml",
    }
    assert all(not Path(value).is_absolute() for value in provenance_paths.values())

    frozen = json.loads(
        (ROOT / "data/rivermark_demo/rivermark_optimal_reference.json").read_text(
            encoding="utf-8"
        )
    )
    assert frozen == result
