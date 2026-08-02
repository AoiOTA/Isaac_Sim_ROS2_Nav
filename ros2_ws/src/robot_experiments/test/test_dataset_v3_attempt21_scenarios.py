from pathlib import Path

import yaml

from robot_experiments.experiment_runner import APPEARANCE_NAV2_PROFILES


CONFIG = Path(__file__).resolve().parents[1] / "config"
SEEDS = {
    ("development", "static"): (18401, 18440),
    ("development", "dynamic"): (18501, 18540),
    ("gate", "static"): (18601, 18610),
    ("gate", "dynamic"): (18701, 18710),
    ("confirmation", "static"): (18801, 18810),
    ("confirmation", "dynamic"): (18901, 18910),
}
STATIC_V2_SEEDS = {
    "gate": (19601, 19610),
    "confirmation": (19801, 19810),
}
STATIC_V3_SEEDS = {
    "gate": (20601, 20610),
    "confirmation": (20801, 20810),
}
STATIC_V4_SEEDS = {
    "gate": (21601, 21610),
    "confirmation": (21801, 21810),
}


def test_attempt21_scenarios_are_adjacent_renderer_only_pairs():
    for (phase, mode), (first, last) in SEEDS.items():
        source = CONFIG / f"isaac_kujiale_dataset_v3_attempt21_{phase}_{mode}.yaml"
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        scenario = payload["scenario"]
        matrix = scenario["runs"]["matrix"]
        assert scenario["id"] == f"isaac_kujiale_dataset_v3_attempt21_{phase}_{mode}"
        assert len(matrix) == 2 * (last - first + 1)
        for offset, seed in enumerate(range(first, last + 1)):
            baseline, appearance = matrix[2 * offset : 2 * offset + 2]
            assert baseline["seed"] == appearance["seed"] == seed
            assert baseline["case_id"] == appearance["case_id"]
            assert baseline["variant_id"] == appearance["variant_id"]
            assert baseline["appearance_profile_id"] == "baseline"
            assert appearance["appearance_profile_id"] in {
                "dim_warm", "dim_cool", "bright_warm", "bright_cool"
            }
            assert baseline["condition_id"] == f"{mode}_baseline"
            assert appearance["condition_id"] == f"{mode}_appearance"


def test_attempt21_scenarios_do_not_change_default_navigation_profiles():
    for phase, mode in SEEDS:
        source = CONFIG / f"isaac_kujiale_dataset_v3_attempt21_{phase}_{mode}.yaml"
        scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
        assert "nav2_profile" not in scenario
        assert scenario["map_version"] == "warehouse_new"


def test_attempt21_static_collection_profile_is_appearance_safe():
    assert {
        "attempt21_static_collection",
        "bio_nav_rgbd_risk_shadow",
        "bio_nav_rgbd_risk_ab",
    } <= APPEARANCE_NAV2_PROFILES


def test_attempt21_static_v2_uses_fresh_formal_seed_families():
    for phase, (first, last) in STATIC_V2_SEEDS.items():
        source = CONFIG / (
            f"isaac_kujiale_dataset_v3_attempt21_{phase}_static_v2.yaml"
        )
        scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
        seeds = [row["seed"] for row in scenario["runs"]["matrix"]]
        assert scenario["id"] == (
            f"isaac_kujiale_dataset_v3_attempt21_{phase}_static_v2"
        )
        assert seeds[::2] == list(range(first, last + 1))
        assert seeds[1::2] == list(range(first, last + 1))


def test_attempt21_static_v3_uses_fresh_formal_seed_families():
    for phase, (first, last) in STATIC_V3_SEEDS.items():
        source = CONFIG / (
            f"isaac_kujiale_dataset_v3_attempt21_{phase}_static_v3.yaml"
        )
        scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
        seeds = [row["seed"] for row in scenario["runs"]["matrix"]]
        assert scenario["id"] == (
            f"isaac_kujiale_dataset_v3_attempt21_{phase}_static_v3"
        )
        assert seeds[::2] == list(range(first, last + 1))
        assert seeds[1::2] == list(range(first, last + 1))


def test_attempt21_static_v4_uses_motion_repair_seed_families():
    for phase, (first, last) in STATIC_V4_SEEDS.items():
        source = CONFIG / (
            f"isaac_kujiale_dataset_v3_attempt21_{phase}_static_v4.yaml"
        )
        text = source.read_text(encoding="utf-8")
        assert "Attempt-21 prereg SHA256:" in text
        scenario = yaml.safe_load(text)["scenario"]
        seeds = [row["seed"] for row in scenario["runs"]["matrix"]]
        assert scenario["id"] == (
            f"isaac_kujiale_dataset_v3_attempt21_{phase}_static_v4"
        )
        assert seeds[::2] == list(range(first, last + 1))
        assert seeds[1::2] == list(range(first, last + 1))


def test_attempt21_dynamic_variants_exist_in_every_selected_obstacle_case():
    dynamic_config = yaml.safe_load(
        (
            CONFIG.parents[3]
            / "isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml"
        ).read_text(encoding="utf-8")
    )
    cases = dynamic_config["cases"]
    case_sets = dynamic_config["case_sets"]
    for phase in ("development", "gate", "confirmation"):
        source = CONFIG / f"isaac_kujiale_dataset_v3_attempt21_{phase}_dynamic.yaml"
        matrix = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]["runs"]["matrix"]
        for run in matrix:
            selected = case_sets.get(run["case_id"], [run["case_id"]])
            for case_id in selected:
                assert run["variant_id"] in cases[case_id]["variants"]
