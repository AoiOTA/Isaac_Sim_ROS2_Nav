from collections import Counter
from pathlib import Path

from robot_experiments.scenario import load_scenario


CONFIG = Path(__file__).resolve().parents[1] / "config"


def test_dataset_v2_scenarios_isolate_phases_and_match_frozen_seeds() -> None:
    expected = {
        ("development", "static"): range(15001, 15041),
        ("development", "dynamic"): range(15101, 15141),
        ("gate", "static"): range(15201, 15211),
        ("gate", "dynamic"): range(15301, 15311),
        ("confirmation", "static"): range(15401, 15411),
        ("confirmation", "dynamic"): range(15501, 15511),
    }
    for (phase, mode), seeds in expected.items():
        scenario = load_scenario(
            CONFIG / f"isaac_kujiale_dataset_v2_{phase}_{mode}.yaml"
        )
        assert scenario.scenario_id == (
            f"isaac_kujiale_dataset_v2_{phase}_{mode}"
        )
        assert scenario.scenario_type == mode
        assert [row.seed for row in scenario.run_matrix] == list(seeds)


def test_development_dynamic_has_balanced_focused_cases() -> None:
    scenario = load_scenario(
        CONFIG / "isaac_kujiale_dataset_v2_development_dynamic.yaml"
    )
    counts = Counter(row.case_id for row in scenario.run_matrix)
    assert counts == {
        "full_route_three_stage": 20,
        "crossing": 5,
        "oncoming": 5,
        "same_direction_slow": 5,
        "temporary_block": 5,
    }
