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


def test_attempt_03_gate_and_confirmation_use_fresh_seed_family() -> None:
    expected = {
        ("gate", "static"): range(16001, 16011),
        ("gate", "dynamic"): range(16101, 16111),
        ("confirmation", "static"): range(16201, 16211),
        ("confirmation", "dynamic"): range(16301, 16311),
    }
    for (phase, mode), seeds in expected.items():
        scenario = load_scenario(
            CONFIG
            / f"isaac_kujiale_dataset_v2_attempt_03_{phase}_{mode}.yaml"
        )
        assert scenario.scenario_id == (
            f"isaac_kujiale_dataset_v2_attempt_03_{phase}_{mode}"
        )
        assert [row.seed for row in scenario.run_matrix] == list(seeds)


def test_attempt_04_gate_and_confirmation_use_third_seed_family() -> None:
    expected = {
        ("gate", "static"): range(16401, 16411),
        ("gate", "dynamic"): range(16501, 16511),
        ("confirmation", "static"): range(16601, 16611),
        ("confirmation", "dynamic"): range(16701, 16711),
    }
    for (phase, mode), seeds in expected.items():
        scenario = load_scenario(
            CONFIG
            / f"isaac_kujiale_dataset_v2_attempt_04_{phase}_{mode}.yaml"
        )
        assert scenario.scenario_id == (
            f"isaac_kujiale_dataset_v2_attempt_04_{phase}_{mode}"
        )
        assert [row.seed for row in scenario.run_matrix] == list(seeds)


def test_attempt_05_gate_and_confirmation_use_fourth_seed_family() -> None:
    expected = {
        ("gate", "static"): range(16801, 16811),
        ("gate", "dynamic"): range(16901, 16911),
        ("confirmation", "static"): range(17001, 17011),
        ("confirmation", "dynamic"): range(17101, 17111),
    }
    for (phase, mode), seeds in expected.items():
        scenario = load_scenario(
            CONFIG
            / f"isaac_kujiale_dataset_v2_attempt_05_{phase}_{mode}.yaml"
        )
        assert scenario.scenario_id == (
            f"isaac_kujiale_dataset_v2_attempt_05_{phase}_{mode}"
        )
        assert [row.seed for row in scenario.run_matrix] == list(seeds)


def test_attempt_06_gate_and_confirmation_use_fifth_seed_family() -> None:
    expected = {
        ("gate", "static"): range(17201, 17211),
        ("gate", "dynamic"): range(17301, 17311),
        ("confirmation", "static"): range(17401, 17411),
        ("confirmation", "dynamic"): range(17501, 17511),
    }
    for (phase, mode), seeds in expected.items():
        scenario = load_scenario(
            CONFIG
            / f"isaac_kujiale_dataset_v2_attempt_06_{phase}_{mode}.yaml"
        )
        assert scenario.scenario_id == (
            f"isaac_kujiale_dataset_v2_attempt_06_{phase}_{mode}"
        )
        assert [row.seed for row in scenario.run_matrix] == list(seeds)


def test_attempt_07_gate_and_confirmation_use_sixth_seed_family() -> None:
    expected = {
        ("gate", "static"): range(17601, 17611),
        ("gate", "dynamic"): range(17701, 17711),
        ("confirmation", "static"): range(17801, 17811),
        ("confirmation", "dynamic"): range(17901, 17911),
    }
    for (phase, mode), seeds in expected.items():
        scenario = load_scenario(
            CONFIG
            / f"isaac_kujiale_dataset_v2_attempt_07_{phase}_{mode}.yaml"
        )
        assert scenario.scenario_id == (
            f"isaac_kujiale_dataset_v2_attempt_07_{phase}_{mode}"
        )
        assert [row.seed for row in scenario.run_matrix] == list(seeds)
