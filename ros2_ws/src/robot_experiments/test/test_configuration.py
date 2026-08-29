import math
from collections import Counter, defaultdict
from pathlib import Path

import pytest
import yaml

from robot_experiments.configuration import ConfigurationError
from robot_experiments.scenario import (
    load_scenario,
    project_usd_xy_to_map,
    validate_dynamic_physical_contract,
    validate_dynamic_runtime_contract,
    validate_navigation_runner_scenario,
)
from robot_experiments.spawn_poses import (
    PoseDefinition,
    SpawnPose,
    load_spawn_pose,
)


PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
CONFIG = PACKAGE_ROOT / "config"
FIXTURES = Path(__file__).parent / "fixtures"

V6_FINAL_KUJIALE_SCENARIOS = tuple(
    f"v6_final_kujiale_{category}.yaml"
    for category in ("static", "dynamic", "appearance")
)
FINAL_RIVERMARK_SCENARIOS = tuple(
    f"final_rivermark_{category}.yaml"
    for category in ("static", "dynamic", "appearance")
)


@pytest.mark.parametrize(
    ("filename", "scenario_type", "seed_count"),
    [
        ("static.yaml", "static", 4),
        ("static_long_range.yaml", "static", 1),
        ("static_benchmark.yaml", "static", 20),
        ("static_complex_route.yaml", "static", 3),
        ("dynamic.yaml", "dynamic", 4),
        ("dynamic_benchmark.yaml", "dynamic", 20),
        ("dynamic_complex_route.yaml", "dynamic", 3),
        ("kujiale_static_visual.yaml", "static", 1),
        ("kujiale_dynamic_visual.yaml", "dynamic", 1),
        ("kujiale_dynamic_visual_g2_g3.yaml", "dynamic", 1),
        ("incremental_mapping.yaml", "incremental", 1),
    ],
)
def test_example_scenarios_parse(filename, scenario_type, seed_count):
    scenario = load_scenario(CONFIG / filename)
    assert scenario.scenario_type == scenario_type
    assert len(scenario.seeds) == seed_count
    assert scenario.goal.frame_id == "map"
    assert scenario.success.position_tolerance_m <= 0.25
    assert scenario.success.orientation_tolerance_deg <= 10.0


def test_v6_low_obstacle_scenario_selects_only_the_frozen_layout():
    scenario = load_scenario(CONFIG / "v6_kujiale_low_obstacles_static.yaml")
    assert scenario.scenario_id == "v6_kujiale_low_obstacles_static"
    assert scenario.scenario_type == "static"
    assert [row.seed for row in scenario.run_matrix] == [8601]
    assert {row.condition_id for row in scenario.run_matrix} == {
        "v6_low_obstacles"
    }
    assert scenario.dynamic_config_file == (
        "../../../../isaac_sim/configs/experiments/"
        "v6_kujiale_low_obstacles_frozen.yaml"
    )
    assert scenario.obstacles["layout_id"] == (
        "kujiale_v6_low_obstacles_indoor_center_connected_r3_20260829"
    )
    assert scenario.run_matrix[0].variant_id == "v6_phase_f_r3"
    assert scenario.goal.require_orientation is False
    assert tuple(goal.goal_id for goal in scenario.route) == (
        "G2", "G3", "G4", "G5", "G1",
    )
    assert all(goal.require_orientation is False for goal in scenario.route)
    assert tuple(goal.yaw_deg for goal in scenario.route) == (
        -160.0, -105.0, -68.0, -42.0, 90.0,
    )
    assert scenario.success.maximum_static_geometric_overlap_m == pytest.approx(
        0.010
    )
    assert scenario.success.static_geometric_overlap_is_diagnostic_only is True
    assert [item["id"] for item in scenario.obstacles["static"]] == [
        "v6_low_box_solo"
    ]
    assert scenario.obstacle_trajectories == ()
    frozen = yaml.safe_load((
        PACKAGE_ROOT.parents[2]
        / "isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen.yaml"
    ).read_text(encoding="utf-8"))
    assert frozen["obstacles"] == [{
        "id": "v6_low_box_solo", "mode": "stationary", "trigger_group": None,
        "size": [0.30, 0.30, 0.16], "mass": 5.0,
        "start": [-0.75, -0.35, 0.08], "end": [-0.75, -0.35, 0.08],
        "speed": 0.0, "delay_sec": 0.0, "jitter_sec": 0.0,
        "post_motion": "hold",
    }]


@pytest.mark.parametrize("filename", V6_FINAL_KUJIALE_SCENARIOS)
def test_v6_final_kujiale_scenarios_are_canonical_single_obstacle_routes(filename):
    scenario = load_scenario(CONFIG / filename)
    category = filename.removeprefix("v6_final_kujiale_").removesuffix(".yaml")

    assert scenario.scenario_id == filename.removesuffix(".yaml")
    assert scenario.scenario_type == ("dynamic" if category == "dynamic" else "static")
    assert len(scenario.route) == 5
    assert tuple(goal.goal_id for goal in scenario.route) == (
        "G2", "G3", "G4", "G5", "G1",
    )
    assert len(scenario.run_matrix) == 20
    assert len({
        (row.seed, row.case_id, row.variant_id, row.appearance_profile_id)
        for row in scenario.run_matrix
    }) == 20
    if category == "static":
        assert [row.seed for row in scenario.run_matrix] == list(
            range(8601, 8621)
        )
    elif category == "dynamic":
        assert [row.seed for row in scenario.run_matrix] == [
            seed for seed in range(8601, 8605) for _ in range(5)
        ]
        assert Counter(row.variant_id for row in scenario.run_matrix) == {
            "v1": 4, "v2": 4, "v3": 4, "v4": 4, "v5": 4,
        }
    else:
        assert [row.seed for row in scenario.run_matrix] == list(
            range(8601, 8621)
        )
        assert Counter(
            row.appearance_profile_id for row in scenario.run_matrix
        ) == {
            "dim_warm": 5,
            "dim_cool": 5,
            "bright_warm": 5,
            "bright_cool": 5,
        }
    assert (
        len(scenario.obstacles["static"]) + len(scenario.obstacle_trajectories)
    ) == 1
    assert scenario.dynamic_config_file is not None
    assert scenario.resolve_path(scenario.dynamic_config_file).is_file()
    for configured in (
        scenario.robot_config_file,
        scenario.nav2_config_file,
        scenario.dynamic_config_file,
        scenario.appearance_config_file,
    ):
        if configured is not None:
            assert not Path(configured).is_absolute()


@pytest.mark.parametrize("filename", FINAL_RIVERMARK_SCENARIOS)
def test_final_rivermark_scenarios_freeze_twenty_physical_matching_runs(filename):
    scenario = load_scenario(CONFIG / filename)
    category = filename.removeprefix("final_rivermark_").removesuffix(".yaml")

    assert scenario.scenario_id == filename.removesuffix(".yaml")
    assert scenario.scenario_type == ("dynamic" if category == "dynamic" else "static")
    assert scenario.map_version == "rivermark_0_05_v1"
    assert scenario.posegraph_version == "occupancy_only_posegraph_unused"
    assert scenario.nav2_config_file == (
        "../../robot_navigation/config/nav2_stable.yaml"
    )
    assert "v6_low_obstacle_isolation" not in scenario.nav2_config_file
    base = yaml.safe_load((
        REPOSITORY_ROOT
        / "ros2_ws/src/robot_navigation/config/nav2_params.yaml"
    ).read_text(encoding="utf-8"))
    stable = yaml.safe_load(
        scenario.resolve_path(scenario.nav2_config_file).read_text(
            encoding="utf-8"
        )
    )
    for costmap_name in ("local_costmap", "global_costmap"):
        base_parameters = base[costmap_name][costmap_name]["ros__parameters"]
        overlay_parameters = stable.get(costmap_name, {}).get(
            costmap_name, {}
        ).get("ros__parameters", {})
        merged = {**base_parameters, **overlay_parameters}
        assert "depth_voxel_layer" in merged["plugins"]
        assert merged["depth_voxel_layer"]["enabled"] is True
    assert len(scenario.run_matrix) == 20
    assert len({
        (row.seed, row.case_id, row.variant_id, row.appearance_profile_id)
        for row in scenario.run_matrix
    }) == 20
    assert tuple(goal.goal_id for goal in scenario.route) == (
        "G1", "G2", "G3", "G4", "G5",
    )

    assert (
        len(scenario.obstacles["static"]) + len(scenario.obstacle_trajectories)
    ) == 1
    if category == "dynamic":
        assert {row.case_id for row in scenario.run_matrix} == {"crossing"}
        assert Counter(row.variant_id for row in scenario.run_matrix) == {
            "v1": 4,
            "v2": 4,
            "v3": 4,
            "v4": 4,
            "v5": 4,
        }
        assert scenario.dynamic_config_file is not None
        assert scenario.resolve_path(scenario.dynamic_config_file).is_file()
        assert scenario.appearance_config_file is None
    elif category == "appearance":
        assert scenario.dynamic_config_file is not None
        assert scenario.resolve_path(scenario.dynamic_config_file).is_file()
        assert scenario.appearance_config_file is not None
        assert Counter(
            row.appearance_profile_id for row in scenario.run_matrix
        ) == {
            "dim_warm": 5,
            "dim_cool": 5,
            "bright_warm": 5,
            "bright_cool": 5,
        }
    else:
        assert scenario.dynamic_config_file is not None
        assert scenario.resolve_path(scenario.dynamic_config_file).is_file()
        assert scenario.appearance_config_file is None


@pytest.mark.parametrize(
    ("filename", "spawn_file"),
    [
        (
            "v6_final_kujiale_dynamic.yaml",
            REPOSITORY_ROOT
            / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml",
        ),
        (
            "final_rivermark_dynamic.yaml",
            REPOSITORY_ROOT / "data/rivermark_demo/rivermark.spawn.yaml",
        ),
    ],
)
def test_final_dynamic_scenarios_match_their_physical_actors(filename, spawn_file):
    scenario = load_scenario(CONFIG / filename)
    spawn_pose = load_spawn_pose(spawn_file, scenario.spawn_pose_name)
    validate_dynamic_physical_contract(
        scenario,
        spawn_pose,
        scenario.resolve_path(scenario.dynamic_config_file),
    )
    variants = tuple(row.variant_id for row in scenario.run_matrix)
    if filename.startswith("v6_final_kujiale"):
        assert len(variants) == 20
        assert Counter(variants) == {
            "v1": 4, "v2": 4, "v3": 4, "v4": 4, "v5": 4,
        }
    else:
        assert len(variants) == 20
        assert Counter(variants) == {
            "v1": 4, "v2": 4, "v3": 4, "v4": 4, "v5": 4,
        }


def test_v6_pilot_kujiale_dynamic_hotreset_v1_matches_final_contract():
    pilot_path = CONFIG / "v6_pilot_kujiale_dynamic_hotreset_v1.yaml"
    final_path = CONFIG / "v6_final_kujiale_dynamic.yaml"
    pilot_document = yaml.safe_load(pilot_path.read_text(encoding="utf-8"))
    final_document = yaml.safe_load(final_path.read_text(encoding="utf-8"))
    pilot_scenario = pilot_document["scenario"]
    final_scenario = final_document["scenario"]

    pilot = load_scenario(pilot_path)
    final = load_scenario(final_path)
    assert pilot.scenario_id == "v6_pilot_kujiale_dynamic_hotreset_v1"
    assert [
        (row.seed, row.case_id, row.variant_id, row.condition_id)
        for row in pilot.run_matrix
    ] == [
        (8601, "single_dynamic_low_box", "v1", "dynamic_hotreset_cold"),
        (8601, "single_dynamic_low_box", "v1", "dynamic_hotreset_hot"),
    ]
    assert [
        (row.seed, row.variant_id) for row in final.run_matrix
    ] == [
        (seed, variant)
        for seed in range(8601, 8605)
        for variant in ("v1", "v2", "v3", "v4", "v5")
    ]

    assert pilot_document["schema_version"] == final_document["schema_version"]
    assert {
        **pilot_scenario,
        "id": final_scenario["id"],
        "runs": {
            **pilot_scenario["runs"],
            "matrix": final_scenario["runs"]["matrix"],
        },
    } == final_scenario

    spawn_file = (
        REPOSITORY_ROOT
        / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
    )
    spawn_pose = load_spawn_pose(spawn_file, pilot.spawn_pose_name)
    validate_dynamic_physical_contract(
        pilot,
        spawn_pose,
        pilot.resolve_path(pilot.dynamic_config_file),
    )


def test_v6_pilot_kujiale_static_hotreset_matches_final_contract():
    pilot_path = CONFIG / "v6_pilot_kujiale_static_hotreset.yaml"
    final_path = CONFIG / "v6_final_kujiale_static.yaml"
    pilot_document = yaml.safe_load(pilot_path.read_text(encoding="utf-8"))
    final_document = yaml.safe_load(final_path.read_text(encoding="utf-8"))
    pilot_scenario = pilot_document["scenario"]
    final_scenario = final_document["scenario"]

    pilot = load_scenario(pilot_path)
    final = load_scenario(final_path)
    assert pilot.scenario_id == "v6_pilot_kujiale_static_hotreset"
    assert [
        (row.seed, row.case_id, row.variant_id, row.condition_id)
        for row in pilot.run_matrix
    ] == [
        (8601, "v6_low_box_solo", "baseline", "static_hotreset_cold"),
        (8601, "v6_low_box_solo", "baseline", "static_hotreset_hot"),
    ]
    # case_id and variant_id are schema-required row identities. The static
    # scenario keeps its frozen actor in obstacles.static and has no dynamic
    # trajectories for those identities to select or trigger.
    assert pilot.scenario_type == "static"
    assert pilot.obstacles["static"] == [{"id": "v6_low_box_solo"}]
    assert pilot.obstacle_trajectories == ()
    assert [row.seed for row in final.run_matrix] == list(range(8601, 8621))
    assert final_document["scenario"]["runs"] == {
        "seeds": list(range(8601, 8621)),
        "timeout_sec": 600.0,
        "leg_timeout_sec": 180.0,
    }

    assert pilot_document["schema_version"] == final_document["schema_version"]
    assert {
        key: value
        for key, value in pilot_scenario.items()
        if key not in {"id", "runs"}
    } == {
        key: value
        for key, value in final_scenario.items()
        if key not in {"id", "runs"}
    }
    assert pilot.timeout_sec == final.timeout_sec
    assert pilot.leg_timeout_sec == final.leg_timeout_sec
    assert pilot.dynamic_config_file == final.dynamic_config_file
    assert pilot.resolve_path(pilot.dynamic_config_file).is_file()
    assert pilot.obstacles == final.obstacles
    assert pilot.route == final.route
    assert pilot.success == final.success


def test_v6_final_kujiale_physical_geometry_variants_and_appearance_profiles():
    static = yaml.safe_load((
        REPOSITORY_ROOT
        / "isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen.yaml"
    ).read_text(encoding="utf-8"))
    assert len(static["obstacles"]) == 1
    assert static["obstacles"][0]["id"] == "v6_low_box_solo"
    assert static["obstacles"][0]["start"] == [-0.75, -0.35, 0.08]
    assert static["obstacles"][0]["size"] == [0.30, 0.30, 0.16]

    dynamic = yaml.safe_load((
        REPOSITORY_ROOT
        / "isaac_sim/configs/experiments/v6_single_dynamic_low_obstacle.yaml"
    ).read_text(encoding="utf-8"))
    assert list(dynamic["cases"]) == ["single_dynamic_low_box"]
    case = dynamic["cases"]["single_dynamic_low_box"]
    assert case["trigger_group"] == "G2"
    assert case["obstacle"]["id"] == "v6_dynamic_low_box_solo"
    assert case["obstacle"]["size"] == [0.30, 0.30, 0.16]
    assert case["obstacle"]["waypoints"] == [
        [-1.25, -0.35, 0.08], [-0.45, -0.35, 0.08],
    ]
    assert case["obstacle"]["speed"] == pytest.approx(0.25)
    assert [row["start_delay_sec"] for row in case["variants"].values()] == [
        0.0, 0.15, 0.30, 0.45, 0.60,
    ]

    appearance = load_scenario(CONFIG / "v6_final_kujiale_appearance.yaml")
    assert [row.seed for row in appearance.run_matrix] == list(
        range(8601, 8621)
    )
    assert [row.appearance_profile_id for row in appearance.run_matrix] == [
        profile
        for _ in range(5)
        for profile in ("dim_warm", "dim_cool", "bright_warm", "bright_cool")
    ]


def _region_ids_for_xy(regions, x, y):
    identifiers = []
    for region in regions:
        xs = [point[0] for point in region["core_polygon_map"]]
        ys = [point[1] for point in region["core_polygon_map"]]
        if min(xs) <= x <= max(xs) and min(ys) <= y <= max(ys):
            identifiers.append(region["id"])
    return identifiers


def test_final_rivermark_physical_geometry_regions_and_appearance_profiles():
    static = yaml.safe_load((
        REPOSITORY_ROOT
        / "data/rivermark_demo/final_rivermark_static_obstacles.yaml"
    ).read_text(encoding="utf-8"))
    assert [item["id"] for item in static["obstacles"]] == [
        "rivermark_static_arc12",
    ]
    assert static["obstacles"][0] == {
        "id": "rivermark_static_arc12",
        "mode": "stationary",
        "trigger_group": None,
        "size": [0.30, 0.30, 0.16],
        "mass": 5.0,
        "start": [11.663, 126.343, 6.29],
        "end": [11.663, 126.343, 6.29],
        "speed": 0.0,
        "delay_sec": 0.0,
        "jitter_sec": 0.0,
        "post_motion": "hold",
    }

    dynamic = yaml.safe_load((
        REPOSITORY_ROOT
        / "data/rivermark_demo/final_rivermark_dynamic.yaml"
    ).read_text(encoding="utf-8"))
    assert list(dynamic["cases"]) == ["crossing"]
    case = dynamic["cases"]["crossing"]
    assert case["trigger_group"] == "G3"
    assert case["obstacle"]["id"] == "rivermark_crossing_cart"
    assert case["obstacle"]["waypoints"] == [
        [-16.9516, 150.855, 6.29], [-20.469, 148.3825, 6.29],
    ]
    assert case["obstacle"]["size"] == [0.30, 0.30, 0.16]
    assert case["obstacle"]["mass"] == pytest.approx(4.0)
    assert case["obstacle"]["speed"] == pytest.approx(0.55)
    assert case["obstacle"]["max_acceleration"] == pytest.approx(0.50)
    assert case["obstacle"]["post_motion"] == "retire"
    assert len(case["variants"]) == 5
    assert [
        variant["start_delay_sec"] for variant in case["variants"].values()
    ] == pytest.approx([0.0, 0.15, 0.30, 0.45, 0.60])

    region_document = yaml.safe_load((
        REPOSITORY_ROOT / "data/rivermark_demo/rivermark_regions.yaml"
    ).read_text(encoding="utf-8"))
    assert len(region_document["regions"]) == 50
    assert _region_ids_for_xy(
        region_document["regions"], 11.663, 126.343
    ) == ["rivermark_a:region_09"]
    assert _region_ids_for_xy(
        region_document["regions"], -16.9516, 150.855
    ) == ["rivermark_a:region_22"]
    assert _region_ids_for_xy(
        region_document["regions"], -20.469, 148.3825
    ) == ["rivermark_a:region_22"]

    appearance = load_scenario(CONFIG / "final_rivermark_appearance.yaml")
    assert appearance.obstacles["static"] == [{"id": "rivermark_static_arc12"}]
    assert appearance.obstacle_trajectories == ()
    assert appearance.dynamic_config_file == (
        "../../../../data/rivermark_demo/final_rivermark_static_obstacles.yaml"
    )
    assert Counter(
        row.appearance_profile_id for row in appearance.run_matrix
    ) == {
        "dim_warm": 5,
        "dim_cool": 5,
        "bright_warm": 5,
        "bright_cool": 5,
    }


def test_dynamic_scenario_preserves_reproducible_trajectories():
    scenario = load_scenario(CONFIG / "dynamic.yaml")
    assert {item["motion"] for item in scenario.obstacle_trajectories} == {
        "crossing",
        "oncoming",
    }
    assert all(item["repeat"] is False for item in scenario.obstacle_trajectories)
    assert scenario.dynamic_config_file is not None


def test_scenario_accepts_per_run_appearance_profiles_only_with_the_appearance_contract(tmp_path):
    document = yaml.safe_load((CONFIG / "kujiale_static_visual.yaml").read_text())
    scenario = document["scenario"]
    scenario["configs"]["appearance"] = "kujiale_appearance_profiles.yaml"
    scenario["runs"] = {
        "matrix": [
            {"seed": 8801, "case_id": "static", "variant_id": "v1", "appearance_profile_id": "dim_warm", "condition_id": "static_appearance"},
        ],
        "timeout_sec": 600.0,
        "leg_timeout_sec": 180.0,
    }
    target = tmp_path / "appearance.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    loaded = load_scenario(target)
    assert loaded.appearance_config_file == "kujiale_appearance_profiles.yaml"
    assert loaded.run_matrix[0].appearance_profile_id == "dim_warm"
    assert loaded.run_matrix[0].condition_id == "static_appearance"

    del scenario["runs"]["matrix"][0]["appearance_profile_id"]
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="appearance_profile_id"):
        load_scenario(target)


def test_long_benchmarks_share_the_same_far_goal():
    static = load_scenario(CONFIG / "static_benchmark.yaml")
    dynamic = load_scenario(CONFIG / "dynamic_benchmark.yaml")
    assert static.goal.position == dynamic.goal.position == (2.0, 5.0)
    assert math.dist((0.0, 0.0), static.goal.position) > 5.0


def test_dynamic_scenario_matches_isaac_physical_configuration():
    for filename in (
        "dynamic.yaml",
        "dynamic_benchmark.yaml",
        "dynamic_complex_route.yaml",
        "kujiale_dynamic_visual.yaml",
        "kujiale_dynamic_visual_g2_g3.yaml",
        "kujiale_dynamic_visual_g5_g1.yaml",
        "kujiale_4x20_dynamic_pair.yaml",
        "kujiale_g2_dynamic_safety_smoke.yaml",
        "kujiale_contact_observability_dynamic.yaml",
    ):
        scenario = load_scenario(CONFIG / filename)
        spawn_file = (
            PACKAGE_ROOT.parents[2]
            / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml"
            if filename.startswith("kujiale_")
            else PACKAGE_ROOT.parents[2] / "isaac_sim/configs/spawn_poses.yaml"
        )
        spawn_pose = load_spawn_pose(spawn_file, scenario.spawn_pose_name)
        assert scenario.dynamic_config_file is not None
        validate_dynamic_physical_contract(
            scenario,
            spawn_pose,
            scenario.resolve_path(scenario.dynamic_config_file),
        )


def test_g2_dynamic_safety_smoke_is_one_fresh_failing_family_route():
    scenario = load_scenario(CONFIG / "kujiale_g2_dynamic_safety_smoke.yaml")
    assert scenario.scenario_id == "kujiale_g2_dynamic_safety_smoke"
    assert [(item.seed, item.case_id, item.variant_id, item.condition_id,
             item.appearance_profile_id) for item in scenario.run_matrix] == [
        (12001, "full_route_three_stage", "v3", "dynamic_safety_smoke",
         "bright_warm"),
    ]
    assert tuple(goal.goal_id for goal in scenario.route) == (
        "G2", "G3", "G4", "G5", "G1",
    )


def test_4x20_static_and_dynamic_pairs_are_balanced_and_seed_paired():
    static = load_scenario(CONFIG / "kujiale_4x20_static_pair.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_4x20_dynamic_pair.yaml")
    for scenario, baseline, varied in (
        (static, "static_baseline", "static_appearance"),
        (dynamic, "dynamic_baseline", "dynamic_appearance"),
    ):
        assert len(scenario.run_matrix) == 40
        by_condition = Counter(item.condition_id for item in scenario.run_matrix)
        assert by_condition == Counter({baseline: 20, varied: 20})
        pairs = defaultdict(list)
        for item in scenario.run_matrix:
            pairs[item.seed].append(item)
        assert len(pairs) == 20
        assert all(len(items) == 2 for items in pairs.values())
        assert all(
            {item.condition_id for item in items} == {baseline, varied}
            for items in pairs.values()
        )
        assert all(
            item.appearance_profile_id == "baseline"
            for item in scenario.run_matrix
            if item.condition_id == baseline
        )
        assert Counter(
            item.appearance_profile_id
            for item in scenario.run_matrix
            if item.condition_id == varied
        ) == Counter({"dim_warm": 5, "dim_cool": 5, "bright_warm": 5, "bright_cool": 5})

    profiles_by_variant = defaultdict(set)
    variants = Counter()
    for item in dynamic.run_matrix:
        if item.condition_id == "dynamic_appearance":
            profiles_by_variant[item.variant_id].add(item.appearance_profile_id)
        if item.condition_id == "dynamic_baseline":
            variants[item.variant_id] += 1
    assert variants == Counter({"v1": 4, "v2": 4, "v3": 4, "v4": 4, "v5": 4})
    assert all(
        profiles == {"dim_warm", "dim_cool", "bright_warm", "bright_cool"}
        for profiles in profiles_by_variant.values()
    )


def test_attempt21_static_ab_uses_fresh_frozen_rows_and_six_map_extrinsic_obstacles():
    revisions = (
        ("isaac_kujiale_attempt21_static_ab.yaml", 19001, "attempt21_static_ab_v1"),
        ("isaac_kujiale_attempt21_static_ab_v2.yaml", 19101, "attempt21_static_ab_v2"),
        ("isaac_kujiale_attempt21_static_ab_v3.yaml", 19201, "attempt21_static_ab_v3"),
    )
    for filename, first_seed, variant_id in revisions:
        scenario = load_scenario(CONFIG / filename)
        assert scenario.scenario_type == "static"
        assert [row.seed for row in scenario.run_matrix] == list(
            range(first_seed, first_seed + 20)
        )
        assert {row.condition_id for row in scenario.run_matrix} == {"static_ab"}
        assert {row.variant_id for row in scenario.run_matrix} == {variant_id}
        assert Counter(
            row.appearance_profile_id for row in scenario.run_matrix
        ) == Counter(
            {
                "baseline": 4,
                "dim_warm": 4,
                "dim_cool": 4,
                "bright_warm": 4,
                "bright_cool": 4,
            }
        )
        assert [item["id"] for item in scenario.obstacles["static"]] == [
            "rgbd_low_box_west",
            "rgbd_low_box_center",
            "rgbd_low_box_east",
            "rgbd_low_box_north",
            "rgbd_low_bar_east",
            "rgbd_low_bar_north",
        ]
        assert scenario.obstacle_trajectories == ()


def test_attempt21_static_ab_v12_uses_engineering_contact_tolerance():
    scenario = load_scenario(CONFIG / "isaac_kujiale_attempt21_static_ab_v12.yaml")
    assert [row.seed for row in scenario.run_matrix] == list(range(23501, 23511))
    assert scenario.success.maximum_static_geometric_overlap_m == pytest.approx(0.010)
    assert {row.variant_id for row in scenario.run_matrix} == {
        "attempt21_static_ab_v12"
    }


def test_attempt21_static_task_sensor_v13_uses_diagnostic_sat_contract():
    scenario = load_scenario(
        CONFIG / "isaac_kujiale_attempt21_static_task_sensor_v13.yaml"
    )
    assert [row.seed for row in scenario.run_matrix] == list(range(23505, 23511))
    assert scenario.success.maximum_static_geometric_overlap_m == pytest.approx(0.010)
    assert scenario.success.static_geometric_overlap_is_diagnostic_only is True
    assert {row.variant_id for row in scenario.run_matrix} == {
        "attempt21_static_task_sensor_v13"
    }


def test_kujiale_dynamic_visual_is_one_controlled_g1_to_g2_observation():
    static = load_scenario(CONFIG / "kujiale_static_visual.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_dynamic_visual.yaml")
    assert static.seeds == (7201,)
    assert dynamic.seeds == (7401,)
    assert [goal.goal_id for goal in static.route] == ["G2", "G3", "G4", "G5", "G1"]
    assert dynamic.route == ()
    assert dynamic.goal.goal_id == "G2"
    assert dynamic.run_matrix[0].case_id == "crossing"
    # G4 is now the former toilet waypoint. Every intermediate target is in
    # a high-clearance part of its room and points toward the next leg; the
    # narrow-passage waypoint is intentionally gone.
    expected_yaws = (-160.0, -105.0, -68.0, -42.0, 90.0)
    assert tuple(goal.yaw_deg for goal in static.route) == expected_yaws
    expected_positions = (
        (0.80, 4.80),
        (-2.20, 3.25),
        (-3.00, -0.45),
        (-2.20, -2.95),
        (0.45, -5.35),
    )
    assert tuple(goal.position for goal in static.route) == expected_positions
    for filename in (
        "kujiale_static_long_range.yaml",
        "kujiale_static_pilot.yaml",
        "kujiale_dynamic_long_range.yaml",
        "kujiale_dynamic_pilot.yaml",
    ):
        scenario = load_scenario(CONFIG / filename)
        assert tuple(goal.yaw_deg for goal in scenario.route) == expected_yaws
    assert static.route[-1] == static.goal
    assert dynamic.goal.position == (0.80, 4.80)


def test_g2_g3_focus_starts_at_the_calibrated_g2_pose():
    scenario = load_scenario(CONFIG / "kujiale_dynamic_visual_g2_g3.yaml")
    spawn = load_spawn_pose(
        PACKAGE_ROOT.parents[2]
        / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml",
        scenario.spawn_pose_name,
    )

    assert scenario.spawn_pose_name == "long_route_start_g2"
    assert spawn.map.position == (0.80, 4.80)
    assert spawn.usd.position == (2.10, -5.00, 0.0635)
    assert spawn.map_calibrated is True
    assert scenario.route == ()
    assert scenario.goal.goal_id == "G3"


def test_g5_g1_focus_starts_at_the_calibrated_g5_pose():
    scenario = load_scenario(CONFIG / "kujiale_dynamic_visual_g5_g1.yaml")
    spawn = load_spawn_pose(
        PACKAGE_ROOT.parents[2]
        / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml",
        scenario.spawn_pose_name,
    )

    assert scenario.spawn_pose_name == "long_route_start_g5"
    assert spawn.map.position == (-2.20, -2.95)
    assert spawn.usd.position == (5.10, 2.75, 0.0635)
    assert spawn.map_calibrated is True
    assert scenario.route == ()
    assert scenario.goal.goal_id == "G1"


def test_complex_routes_are_long_continuous_and_end_at_goal():
    for filename in (
        "static_complex_route.yaml",
        "dynamic_complex_route.yaml",
    ):
        scenario = load_scenario(CONFIG / filename)
        assert len(scenario.route) == 6
        assert scenario.route[-1] == scenario.goal
        assert scenario.success.minimum_ground_truth_path_length_m >= 49.0
        assert scenario.success.minimum_reverse_distance_m == 0.0
        assert scenario.success.maximum_reverse_distance_fraction == 0.02
        assert scenario.success.minimum_curved_distance_fraction == 0.05
        straight_line_lower_bound = sum(
            math.dist(previous.position, current.position)
            for previous, current in zip(scenario.route, scenario.route[1:])
        )
        straight_line_lower_bound += math.dist(
            (0.0, 0.0), scenario.route[0].position
        )
        assert straight_line_lower_bound > 45.0


def test_route_final_pose_must_match_goal(tmp_path):
    document = yaml.safe_load(
        (CONFIG / "static_complex_route.yaml").read_text()
    )
    document["scenario"]["route"][-1]["position"] = [0.5, 0.0]
    with pytest.raises(ConfigurationError, match="must exactly match"):
        load_scenario(_write_scenario(tmp_path, document))


def test_usd_to_map_projection_supports_translation_and_nonzero_yaw():
    spawn_pose = SpawnPose(
        name="rotated",
        usd=PoseDefinition(position=(10.0, 20.0, 0.1), yaw_deg=30.0),
        map=PoseDefinition(position=(1.0, -2.0), yaw_deg=120.0),
        map_calibrated=True,
        position_stddev_m=0.01,
        yaw_stddev_deg=0.1,
    )
    assert project_usd_xy_to_map((12.0, 21.0), spawn_pose) == pytest.approx(
        (0.0, 0.0)
    )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda obstacle: obstacle.__setitem__("size", [0.6, 0.5, 1.0]),
            "XY dimensions mismatch",
        ),
        (
            lambda obstacle: obstacle.__setitem__("speed", 0.5),
            "duration mismatch",
        ),
        (
            lambda obstacle: obstacle.__setitem__("start", [6.1, -1.5, 0.5]),
            "start endpoint mismatch",
        ),
        (
            lambda obstacle: obstacle.__setitem__("repeat", True),
            "repeat mismatch",
        ),
    ],
)
def test_dynamic_physical_contract_rejects_geometry_or_motion_mismatch(
    tmp_path, mutation, message
):
    scenario = load_scenario(CONFIG / "dynamic.yaml")
    spawn_pose = load_spawn_pose(
        PACKAGE_ROOT.parents[2] / "isaac_sim/configs/spawn_poses.yaml",
        scenario.spawn_pose_name,
    )
    assert scenario.dynamic_config_file is not None
    physical_config = scenario.resolve_path(scenario.dynamic_config_file)
    document = yaml.safe_load(physical_config.read_text(encoding="utf-8"))
    mutation(document["obstacles"][0])
    target = tmp_path / "physical.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ConfigurationError, match=message):
        validate_dynamic_physical_contract(scenario, spawn_pose, target)


def _write_scenario(tmp_path, document):
    target = tmp_path / "scenario.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    return target


def test_incremental_scenario_enforces_plan_improvement_target():
    scenario = load_scenario(CONFIG / "incremental_mapping.yaml")
    assert scenario.incremental_mapping is not None
    assert scenario.incremental_mapping["minimum_time_improvement_percent"] >= 30.0
    with pytest.raises(ConfigurationError, match="mapping workflow descriptors"):
        validate_navigation_runner_scenario(scenario)


def test_navigation_runner_accepts_static_and_dynamic_scenarios():
    validate_navigation_runner_scenario(load_scenario(CONFIG / "static.yaml"))
    validate_navigation_runner_scenario(load_scenario(CONFIG / "dynamic.yaml"))


def test_runtime_contract_rejects_wrong_dynamic_state_or_identity():
    static = load_scenario(CONFIG / "static.yaml")
    dynamic = load_scenario(CONFIG / "dynamic.yaml")
    with pytest.raises(ConfigurationError, match="disabled"):
        validate_dynamic_runtime_contract(
            static,
            runtime_enabled=True,
            runtime_config_hash="hash",
            runtime_obstacle_ids=(),
            expected_config_hash=None,
        )
    with pytest.raises(ConfigurationError, match="--dynamic-obstacles"):
        validate_dynamic_runtime_contract(
            dynamic,
            runtime_enabled=False,
            runtime_config_hash="hash",
            runtime_obstacle_ids=("crossing_box", "cart_proxy_b"),
            expected_config_hash="hash",
        )
    with pytest.raises(ConfigurationError, match="configuration hash"):
        validate_dynamic_runtime_contract(
            dynamic,
            runtime_enabled=True,
            runtime_config_hash="wrong",
            runtime_obstacle_ids=("crossing_box", "cart_proxy_b"),
            expected_config_hash="expected",
        )
    with pytest.raises(ConfigurationError, match="IDs"):
        validate_dynamic_runtime_contract(
            dynamic,
            runtime_enabled=True,
            runtime_config_hash="hash",
            runtime_obstacle_ids=("wrong",),
            expected_config_hash="hash",
        )
    validate_dynamic_runtime_contract(
        dynamic,
        runtime_enabled=True,
        runtime_config_hash="hash",
        runtime_obstacle_ids=("cart_proxy_b", "crossing_box"),
        expected_config_hash="hash",
    )


def test_uncalibrated_map_pose_is_rejected():
    with pytest.raises(ConfigurationError, match="no calibrated map pose"):
        load_spawn_pose(FIXTURES / "spawn_poses_uncalibrated.yaml", "mapping_start")


def test_calibrated_map_pose_loads_from_parameterized_yaml(tmp_path):
    document = yaml.safe_load((FIXTURES / "spawn_poses_calibrated.yaml").read_text())
    target = tmp_path / "spawn.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    pose = load_spawn_pose(target, "mapping_start")
    assert pose.map_calibrated is True
    assert pose.map.position == (0.5, -0.25)
    assert pose.usd.position == (1.0, 2.0, 0.15)


def test_unknown_spawn_pose_lists_available_names():
    with pytest.raises(ConfigurationError, match="mapping_start"):
        load_spawn_pose(
            FIXTURES / "spawn_poses_uncalibrated.yaml",
            "missing",
            require_calibrated=False,
        )


def test_scenario_rejects_looser_plan_threshold(tmp_path):
    document = yaml.safe_load((CONFIG / "static.yaml").read_text())
    document["scenario"]["success"]["position_tolerance_m"] = 0.251
    with pytest.raises(ConfigurationError, match="0.25 m"):
        load_scenario(_write_scenario(tmp_path, document))


def test_dynamic_waypoint_times_must_increase(tmp_path):
    document = yaml.safe_load((CONFIG / "dynamic.yaml").read_text())
    trajectory = document["scenario"]["obstacles"]["trajectories"][0]
    trajectory["waypoints"][1]["time_sec"] = 0.0
    with pytest.raises(ConfigurationError, match="strictly increasing"):
        load_scenario(_write_scenario(tmp_path, document))


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda scenario: scenario["goal"].__setitem__("frame_id", "odom"), "must be map"),
        (lambda scenario: scenario["runs"].__setitem__("seeds", [-1]), "non-negative"),
        (lambda scenario: scenario.__setitem__("unknown", True), "unknown scenario keys"),
        (
            lambda scenario: scenario["obstacles"]["static"].append(
                {"id": "not_authored", "unexpected": True}
            ),
            "unknown scenario.obstacles.static",
        ),
    ],
)
def test_scenario_parser_rejects_unimplemented_or_unknown_contracts(
    tmp_path, mutation, message
):
    document = yaml.safe_load((CONFIG / "static.yaml").read_text())
    mutation(document["scenario"])
    with pytest.raises(ConfigurationError, match=message):
        load_scenario(_write_scenario(tmp_path, document))


def test_dynamic_obstacle_ids_must_be_unique(tmp_path):
    document = yaml.safe_load((CONFIG / "dynamic.yaml").read_text())
    trajectories = document["scenario"]["obstacles"]["trajectories"]
    trajectories[1]["id"] = trajectories[0]["id"]
    with pytest.raises(ConfigurationError, match="duplicate dynamic obstacle id"):
        load_scenario(_write_scenario(tmp_path, document))


def test_schema_declares_all_three_scenario_variants():
    schema = yaml.safe_load((CONFIG / "scenario.schema.yaml").read_text())
    variants = schema["properties"]["scenario"]["properties"]["type"]["enum"]
    assert set(variants) == {"static", "dynamic", "incremental"}
