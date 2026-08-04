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
STATIC_V5_SEEDS = {
    "gate": (22601, 22610),
    "confirmation": (22801, 22810),
}
STATIC_ONLINE_V6_SEEDS = tuple(range(22401, 22421))
STATIC_ONLINE_V7_SEEDS = tuple(range(23001, 23011))
STATIC_ONLINE_V8_SEEDS = tuple(range(23101, 23111))
STATIC_ONLINE_V10_SEEDS = tuple(range(23301, 23311))
STATIC_ONLINE_V11_SEEDS = tuple(range(23401, 23411))
STATIC_ONLINE_V12_SEEDS = tuple(range(23501, 23511))
STATIC_FUSION_SUPPLEMENT_V15_SEEDS = tuple(range(23601, 23611))
STATIC_ONLINE_V9_SEEDS = tuple(range(23201, 23211))
ATTEMPT22_DYNAMIC_DEVELOPMENT_SEEDS = tuple(range(31101, 31141))


def test_attempt21_static_v15_fusion_supplement_preserves_six_obstacles():
    source = CONFIG / "isaac_kujiale_attempt21_static_fusion_supplement_v15.yaml"
    scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
    assert scenario["id"] == "isaac_kujiale_attempt21_static_fusion_supplement_v15"
    assert tuple(row["seed"] for row in scenario["runs"]["matrix"]) == (
        STATIC_FUSION_SUPPLEMENT_V15_SEEDS
    )
    assert all(
        row["variant_id"] == "attempt21_static_fusion_supplement_v15"
        for row in scenario["runs"]["matrix"]
    )
    assert len(scenario["obstacles"]["static"]) == 6
    assert scenario["obstacles"]["trajectories"] == []
    assert scenario["success"]["static_geometric_overlap_is_diagnostic_only"] is True


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


def test_attempt21_static_v5_uses_action_ack_repair_seed_families():
    for phase, (first, last) in STATIC_V5_SEEDS.items():
        source = CONFIG / (
            f"isaac_kujiale_dataset_v3_attempt21_{phase}_static_v5.yaml"
        )
        text = source.read_text(encoding="utf-8")
        assert "Attempt-21 prereg SHA256:" in text
        scenario = yaml.safe_load(text)["scenario"]
        seeds = [row["seed"] for row in scenario["runs"]["matrix"]]
        assert scenario["id"] == (
            f"isaac_kujiale_dataset_v3_attempt21_{phase}_static_v5"
        )
        assert seeds[::2] == list(range(first, last + 1))
        assert seeds[1::2] == list(range(first, last + 1))


def test_attempt21_static_online_v6_is_reserved_for_motion_repair():
    source = CONFIG / "isaac_kujiale_attempt21_static_ab_v6.yaml"
    scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
    assert scenario["id"] == "isaac_kujiale_attempt21_static_ab_v6"
    matrix = scenario["runs"]["matrix"]
    assert tuple(row["seed"] for row in matrix) == STATIC_ONLINE_V6_SEEDS
    assert all(row["variant_id"] == "attempt21_static_ab_v6" for row in matrix)


def test_attempt21_static_online_v7_uses_ten_fresh_user_approved_routes():
    source = CONFIG / "isaac_kujiale_attempt21_static_ab_v7.yaml"
    scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
    assert scenario["id"] == "isaac_kujiale_attempt21_static_ab_v7"
    matrix = scenario["runs"]["matrix"]
    assert tuple(row["seed"] for row in matrix) == STATIC_ONLINE_V7_SEEDS
    assert all(row["variant_id"] == "attempt21_static_ab_v7" for row in matrix)
    assert [row["appearance_profile_id"] for row in matrix] == [
        "baseline", "dim_warm", "dim_cool", "bright_warm", "bright_cool"
    ] * 2


def test_attempt21_static_online_v8_uses_fresh_profile_repair_routes():
    source = CONFIG / "isaac_kujiale_attempt21_static_ab_v8.yaml"
    scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
    assert scenario["id"] == "isaac_kujiale_attempt21_static_ab_v8"
    matrix = scenario["runs"]["matrix"]
    assert tuple(row["seed"] for row in matrix) == STATIC_ONLINE_V8_SEEDS
    assert all(row["variant_id"] == "attempt21_static_ab_v8" for row in matrix)


def test_attempt21_static_online_v9_uses_fresh_costmap_repair_routes():
    source = CONFIG / "isaac_kujiale_attempt21_static_ab_v9.yaml"
    scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
    assert scenario["id"] == "isaac_kujiale_attempt21_static_ab_v9"
    matrix = scenario["runs"]["matrix"]
    assert tuple(row["seed"] for row in matrix) == STATIC_ONLINE_V9_SEEDS
    assert all(row["variant_id"] == "attempt21_static_ab_v9" for row in matrix)
    assert scenario["obstacles"]["trajectories"] == []
    assert len(scenario["obstacles"]["static"]) == 6
    assert [row["appearance_profile_id"] for row in matrix] == [
        "baseline", "dim_warm", "dim_cool", "bright_warm", "bright_cool"
    ] * 2


def test_attempt21_static_online_v10_uses_fresh_exact_contact_routes():
    source = CONFIG / "isaac_kujiale_attempt21_static_ab_v10.yaml"
    scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
    assert scenario["id"] == "isaac_kujiale_attempt21_static_ab_v10"
    matrix = scenario["runs"]["matrix"]
    assert tuple(row["seed"] for row in matrix) == STATIC_ONLINE_V10_SEEDS
    assert all(row["variant_id"] == "attempt21_static_ab_v10" for row in matrix)
    assert scenario["obstacles"]["trajectories"] == []
    assert len(scenario["obstacles"]["static"]) == 6


def test_attempt21_static_online_v11_uses_fresh_module3_safety_repair_routes():
    source = CONFIG / "isaac_kujiale_attempt21_static_ab_v11.yaml"
    scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
    assert scenario["id"] == "isaac_kujiale_attempt21_static_ab_v11"
    matrix = scenario["runs"]["matrix"]
    assert tuple(row["seed"] for row in matrix) == STATIC_ONLINE_V11_SEEDS
    assert all(row["variant_id"] == "attempt21_static_ab_v11" for row in matrix)
    assert scenario["obstacles"]["trajectories"] == []
    assert len(scenario["obstacles"]["static"]) == 6


def test_attempt21_static_online_v12_uses_user_accepted_task_contact_contract():
    source = CONFIG / "isaac_kujiale_attempt21_static_ab_v12.yaml"
    scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
    assert scenario["id"] == "isaac_kujiale_attempt21_static_ab_v12"
    matrix = scenario["runs"]["matrix"]
    assert tuple(row["seed"] for row in matrix) == STATIC_ONLINE_V12_SEEDS
    assert all(row["variant_id"] == "attempt21_static_ab_v12" for row in matrix)
    assert scenario["success"]["maximum_static_geometric_overlap_m"] == 0.010
    assert scenario["obstacles"]["trajectories"] == []
    assert len(scenario["obstacles"]["static"]) == 6


def test_attempt21_static_v13_separates_contact_sensor_from_sat_diagnostic():
    source = CONFIG / "isaac_kujiale_attempt21_static_task_sensor_v13.yaml"
    scenario = yaml.safe_load(source.read_text(encoding="utf-8"))["scenario"]
    assert scenario["id"] == "isaac_kujiale_attempt21_static_task_sensor_v13"
    matrix = scenario["runs"]["matrix"]
    assert tuple(row["seed"] for row in matrix) == tuple(range(23505, 23511))
    assert all(
        row["variant_id"] == "attempt21_static_task_sensor_v13"
        for row in matrix
    )
    assert scenario["success"]["maximum_static_geometric_overlap_m"] == 0.010
    assert scenario["success"]["static_geometric_overlap_is_diagnostic_only"] is True
    assert scenario["obstacles"]["trajectories"] == []
    assert len(scenario["obstacles"]["static"]) == 6


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


def test_attempt22_dynamic_development_uses_frozen_new_seed_family():
    source = CONFIG / "isaac_kujiale_dataset_v3_attempt22_development_dynamic.yaml"
    text = source.read_text(encoding="utf-8")
    assert "attempt22 prereg SHA256:" in text
    scenario = yaml.safe_load(text)["scenario"]
    assert scenario["id"] == (
        "isaac_kujiale_dataset_v3_attempt22_development_dynamic"
    )
    matrix = scenario["runs"]["matrix"]
    assert tuple(row["seed"] for row in matrix[::2]) == (
        ATTEMPT22_DYNAMIC_DEVELOPMENT_SEEDS
    )
    assert tuple(row["seed"] for row in matrix[1::2]) == (
        ATTEMPT22_DYNAMIC_DEVELOPMENT_SEEDS
    )
    assert all(row["condition_id"] == "dynamic_baseline" for row in matrix[::2])
    assert all(row["condition_id"] == "dynamic_appearance" for row in matrix[1::2])
    assert "nav2_profile" not in scenario
