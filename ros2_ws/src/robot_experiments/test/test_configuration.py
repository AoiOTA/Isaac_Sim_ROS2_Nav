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
CONFIG = PACKAGE_ROOT / "config"
FIXTURES = Path(__file__).parent / "fixtures"


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


def test_r2c4_r13_shadow_matrix_is_the_reserved_eight_route_contract():
    static = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r13_static.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r13_dynamic.yaml")
    assert [(item.seed, item.condition_id, item.appearance_profile_id) for item in static.run_matrix] == [
        (8801, "static_baseline", "baseline"),
        (8801, "static_appearance", "dim_warm"),
        (8802, "static_appearance", "dim_cool"),
        (8802, "static_baseline", "baseline"),
    ]
    assert [(item.seed, item.condition_id, item.appearance_profile_id, item.variant_id) for item in dynamic.run_matrix] == [
        (8901, "dynamic_baseline", "baseline", "v1"),
        (8901, "dynamic_appearance", "dim_warm", "v1"),
        (8902, "dynamic_appearance", "dim_cool", "v2"),
        (8902, "dynamic_baseline", "baseline", "v2"),
    ]


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


def test_r2c4_development_matrix_is_new_and_balanced():
    static = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_development_static.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_development_dynamic.yaml")
    assert [row.seed for row in static.run_matrix] == [8001, 8001, 8002, 8002]
    assert [row.seed for row in dynamic.run_matrix] == [8101, 8101, 8102, 8102]
    assert [row.condition_id for row in static.run_matrix] == ["static_baseline", "static_appearance", "static_appearance", "static_baseline"]
    assert [row.condition_id for row in dynamic.run_matrix] == ["dynamic_baseline", "dynamic_appearance", "dynamic_appearance", "dynamic_baseline"]
    assert [row.appearance_profile_id for row in static.run_matrix] == ["baseline", "dim_warm", "dim_cool", "baseline"]
    assert [row.appearance_profile_id for row in dynamic.run_matrix] == ["baseline", "dim_warm", "dim_cool", "baseline"]


def test_r3_r1_r1_r1_authorization_only_scenarios_have_valid_distinct_routes():
    static = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_static.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_dynamic.yaml")
    assert [row.seed for row in static.run_matrix] == [9200]
    assert [row.seed for row in dynamic.run_matrix] == [9300]
    for scenario in (static, dynamic):
        assert len(scenario.route) == 2
        assert scenario.route[0].frame_id == scenario.route[1].frame_id == "map"
        assert scenario.route[0].position != scenario.route[1].position
        assert scenario.route[-1] == scenario.goal


def test_r3_r1_r1_r1_r1_authorization_only_scenarios_use_fresh_seeds():
    static = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_r1_static.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_r1_dynamic.yaml")
    assert [row.seed for row in static.run_matrix] == [9400]
    assert [row.seed for row in dynamic.run_matrix] == [9500]
    for scenario in (static, dynamic):
        assert len(scenario.route) == 2
        assert scenario.route[0].frame_id == scenario.route[1].frame_id == "map"
        assert scenario.route[0].position != scenario.route[1].position
        assert scenario.route[-1] == scenario.goal


def test_r3_r1_r1_r1_r2_authorization_only_contract_matches_runtime_manifests():
    static = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_r2_static.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_r2_dynamic.yaml")
    assert [row.seed for row in static.run_matrix] == [9600]
    assert [row.seed for row in dynamic.run_matrix] == [9700]
    assert {item["id"] for item in static.obstacles["static"]} == {
        "rgbd_low_box_west", "rgbd_low_box_center", "rgbd_low_box_east",
        "rgbd_low_box_north", "rgbd_low_bar_east", "rgbd_low_bar_north",
    }
    assert {item["id"] for item in dynamic.obstacle_trajectories} == {
        "crossing_actor", "oncoming_actor", "same_direction_slow_actor",
        "local_bypass_actor", "g2_g3_exit_actor", "g5_g1_crossing_actor",
        "temporary_block_actor",
    }
    spawn_pose = load_spawn_pose(
        PACKAGE_ROOT.parents[2]
        / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml",
        dynamic.spawn_pose_name,
    )
    assert dynamic.dynamic_config_file is not None
    validate_dynamic_physical_contract(
        dynamic, spawn_pose, dynamic.resolve_path(dynamic.dynamic_config_file)
    )
    for scenario in (static, dynamic):
        assert len(scenario.route) == 2
        assert scenario.route[0].frame_id == scenario.route[1].frame_id == "map"
        assert scenario.route[0].position != scenario.route[1].position
        assert scenario.route[-1] == scenario.goal


def test_r3_camera_repair_authorization_scenarios_use_fresh_seeds_and_full_manifests():
    static = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_r3_static.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_r3_dynamic.yaml")
    assert [row.seed for row in static.run_matrix] == [9800]
    assert [row.seed for row in dynamic.run_matrix] == [9900]
    assert len(static.obstacles["static"]) == 6
    assert len(dynamic.obstacle_trajectories) == 7
    assert dynamic.run_matrix[0].case_id == "full_route_three_stage"


def test_r11_authorization_scenarios_use_fresh_seeds_and_full_manifests():
    static = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_r11_static.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_stage2_2_r2c4_r3_r1_r1_r1_r11_dynamic.yaml")
    assert [row.seed for row in static.run_matrix] == [12000]
    assert [row.seed for row in dynamic.run_matrix] == [12100]
    assert len(static.obstacles["static"]) == 6
    assert len(dynamic.obstacle_trajectories) == 7
    assert dynamic.run_matrix[0].case_id == "full_route_three_stage"


def test_stage2_2_r2b_development_matrix_is_frozen_and_balanced():
    static = load_scenario(CONFIG / "kujiale_stage2_2_r2b_development_static.yaml")
    dynamic = load_scenario(CONFIG / "kujiale_stage2_2_r2b_development_dynamic.yaml")
    assert tuple(goal.goal_id for goal in static.route) == ("G2", "G3", "G4", "G5", "G1")
    assert tuple(goal.goal_id for goal in dynamic.route) == ("G2", "G3", "G4", "G5", "G1")
    assert [item.seed for item in static.run_matrix] == [7801, 7801, 7802, 7802]
    assert [item.seed for item in dynamic.run_matrix] == [7901, 7901, 7902, 7902]
    assert [item.appearance_profile_id for item in static.run_matrix] == [
        "baseline", "dim_warm", "dim_cool", "baseline",
    ]
    assert [item.appearance_profile_id for item in dynamic.run_matrix] == [
        "baseline", "dim_warm", "dim_cool", "baseline",
    ]
    assert {item.variant_id for item in dynamic.run_matrix if item.seed == 7901} == {"v1"}
    assert {item.variant_id for item in dynamic.run_matrix if item.seed == 7902} == {"v2"}
    assert static.success.minimum_ground_truth_path_length_m >= 20.0
    assert dynamic.success.minimum_ground_truth_path_length_m >= 20.0


def test_stage2_2_r2d1_replacement_preserves_g1_matrix_without_gate_identity():
    gate_static = load_scenario(CONFIG / "kujiale_stage2_2_g1_gate_static.yaml")
    gate_dynamic = load_scenario(CONFIG / "kujiale_stage2_2_g1_gate_dynamic.yaml")
    replacement_static = load_scenario(
        CONFIG / "kujiale_stage2_2_r2d1_replacement_static.yaml"
    )
    replacement_dynamic = load_scenario(
        CONFIG / "kujiale_stage2_2_r2d1_replacement_dynamic.yaml"
    )

    assert replacement_static.scenario_id == (
        "kujiale_stage2_2_r2d1_replacement_static"
    )
    assert replacement_dynamic.scenario_id == (
        "kujiale_stage2_2_r2d1_replacement_dynamic"
    )
    assert "gate" not in replacement_static.scenario_id
    assert "gate" not in replacement_dynamic.scenario_id
    assert replacement_static.run_matrix == gate_static.run_matrix
    assert replacement_dynamic.run_matrix == gate_dynamic.run_matrix
    assert replacement_static.route == gate_static.route
    assert replacement_dynamic.route == gate_dynamic.route
    assert replacement_static.success == gate_static.success
    assert replacement_dynamic.success == gate_dynamic.success
    assert len(replacement_static.run_matrix) == 10
    assert len(replacement_dynamic.run_matrix) == 10


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
