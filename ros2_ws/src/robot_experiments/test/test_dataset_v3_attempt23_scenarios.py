"""Contract tests for the Attempt-23 global A/B scenario configs."""

from pathlib import Path

import yaml

CONFIG = (
    Path(__file__).resolve().parents[1]
    / "config"
)


def _load(name: str) -> dict:
    text = (CONFIG / name).read_text(encoding="utf-8")
    assert "attempt23 prereg SHA256:" in text
    return yaml.safe_load(text)["scenario"]


def test_attempt23_global_ab_static_seed_band() -> None:
    scenario = _load("isaac_kujiale_dataset_v3_attempt23_global_ab_static.yaml")
    assert scenario["id"] == "isaac_kujiale_dataset_v3_attempt23_global_ab_static"
    seeds = [row["seed"] for row in scenario["runs"]["matrix"]]
    assert seeds == list(range(31801, 31831))
    assert {row["condition_id"] for row in scenario["runs"]["matrix"]} == {
        "attempt23_global_ab_static"
    }


def test_attempt23_global_ab_dynamic_seed_band_and_case_cycle() -> None:
    scenario = _load("isaac_kujiale_dataset_v3_attempt23_global_ab_dynamic.yaml")
    assert scenario["id"] == "isaac_kujiale_dataset_v3_attempt23_global_ab_dynamic"
    matrix = scenario["runs"]["matrix"]
    seeds = [row["seed"] for row in matrix]
    assert seeds == list(range(31901, 31931))
    cases = [row["case_id"] for row in matrix]
    assert len(set(cases)) == 5  # five frozen dynamic cases cycle per seed
    assert all(
        row["condition_id"] == "attempt23_global_ab_dynamic" for row in matrix
    )
