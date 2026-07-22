import math
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
    ):
        scenario = load_scenario(CONFIG / filename)
        spawn_pose = load_spawn_pose(
            PACKAGE_ROOT.parents[2] / "isaac_sim/configs/spawn_poses.yaml",
            scenario.spawn_pose_name,
        )
        assert scenario.dynamic_config_file is not None
        validate_dynamic_physical_contract(
            scenario,
            spawn_pose,
            scenario.resolve_path(scenario.dynamic_config_file),
        )


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
