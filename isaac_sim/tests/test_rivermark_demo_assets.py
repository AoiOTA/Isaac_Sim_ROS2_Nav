from pathlib import Path
import json

import cv2
import math
import yaml

from isaac_sim.src.experiment.scenario import load_dynamic_scenario
from isaac_sim.src.experiment.appearance import load_appearance_profiles


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEMO_ROOT = PROJECT_ROOT / "data" / "rivermark_demo"


def test_selected_map_keeps_current_eighty_meter_extent_at_five_centimeters():
    metadata = yaml.safe_load(
        (DEMO_ROOT / "rivermark_selected.yaml").read_text(encoding="utf-8")
    )
    image = cv2.imread(
        str(DEMO_ROOT / metadata["image"]), cv2.IMREAD_UNCHANGED
    )

    assert float(metadata["resolution"]) == 0.05
    assert image is not None
    assert image.shape[:2] == (1600, 1600)
    assert image.shape[1] * float(metadata["resolution"]) == 80.0
    assert image.shape[0] * float(metadata["resolution"]) == 80.0


def test_demo_start_and_goal_are_inside_current_map_extent():
    metadata = yaml.safe_load(
        (DEMO_ROOT / "rivermark_selected.yaml").read_text(encoding="utf-8")
    )
    goals = yaml.safe_load(
        (DEMO_ROOT / "rivermark_demo_goals.yaml").read_text(encoding="utf-8")
    )
    origin_x, origin_y, _ = (float(value) for value in metadata["origin"])
    maximum_x = origin_x + 80.0
    maximum_y = origin_y + 80.0

    for name in ("start", "goal"):
        x, y, _ = (float(value) for value in goals[name])
        assert origin_x <= x < maximum_x
        assert origin_y <= y < maximum_y


def test_campaign_uses_external_goal_ground_truth_and_all_module2_arms():
    source = (PROJECT_ROOT / "scripts/run_rivermark_campaign.sh").read_text(
        encoding="utf-8"
    )
    assert "RIVERMARK_AUTO_GOAL=0" in source
    assert "RIVERMARK_GROUND_TRUTH=1" in source
    assert "navigation_execution_backend:=route_guided" in source
    assert "off|sr_medium|dr_medium|medium" in source
    assert 'experiment_arm:="${arm}"' in source
    assert '[[ "${reset_type}" == "std_srvs/srv/Trigger" ]]' in source
    assert "PopulatePointInstancerBucket invalid protoIndex=" in source
    assert 'controller_max_linear_velocity_mps="0.75"' in source
    assert 'controller_linear_velocity_std_mps="0.35"' in source
    assert 'rendering_hz="30"' in source
    assert "RIVERMARK_RENDERING_HZ" in source
    assert "observed_physics_hz" in source
    assert "observed_rendering_hz" in source
    assert "resume:=true" in source
    assert "runtime_tile_cache_contract.json" in source
    assert "cognitive_tile_cache_entries" in source
    assert "cognitive_tile_cache_hits" in source
    assert "cognitive_tile_cache_misses" in source
    assert "runtime_controller_contract.json" in source
    assert "controller vx_max mismatch" in source
    assert "runtime_geometry_ready" in source
    assert "/global_costmap/costmap_raw metadata size_x size_y" in source
    assert '[[ "${controller_state}" == active* ]]' in source
    assert '[[ "${planner_state}" == active* ]]' in source
    assert '[[ "${collision_state}" == active* ]]' in source
    assert "geometry did not remain 1600x1600" in source
    assert 'geometry_deadline=$((SECONDS + 20))' in source
    assert 'geometry_stable="true"' in source


def test_static_dynamic_and_appearance_use_the_same_five_graph_waypoints():
    config_root = (
        PROJECT_ROOT / "ros2_ws" / "src" / "robot_experiments" / "config"
    )
    scenarios = []
    for condition in ("static", "dynamic", "appearance"):
        document = yaml.safe_load(
            (config_root / f"attempt31_rivermark_{condition}.yaml").read_text(
                encoding="utf-8"
            )
        )
        scenarios.append(document["scenario"])

    routes = [scenario["route"] for scenario in scenarios]
    assert [waypoint["id"] for waypoint in routes[0]] == [
        "G1", "G2", "G3", "G4", "G5"
    ]
    assert routes[0] == routes[1] == routes[2]
    assert all(len(route) == 5 for route in routes)

    graph = json.loads(
        (DEMO_ROOT / "rivermark_selected.geojson").read_text(encoding="utf-8")
    )
    graph_points = {
        tuple(float(value) for value in feature["geometry"]["coordinates"])
        for feature in graph["features"]
        if feature["geometry"]["type"] == "Point"
    }
    assert all(
        tuple(float(value) for value in waypoint["position"]) in graph_points
        for waypoint in routes[0]
    )
    dynamic = scenarios[1]
    assert {item["case_id"] for item in dynamic["runs"]["matrix"]} == {
        "full_route_four_stage"
    }
    assert {item["trigger_group"] for item in dynamic["obstacles"]["trajectories"]} == {
        "G2", "G3", "G4", "G5"
    }


def test_dynamic_full_route_uses_four_heterogeneous_interactions():
    physical = load_dynamic_scenario(DEMO_ROOT / "rivermark_dynamic.yaml")
    selected = physical.selected_cases("full_route_four_stage")

    assert [item.case_id for item in selected] == [
        "oncoming", "crossing", "same_direction_slow", "temporary_block"
    ]
    assert [item.trigger_group for item in selected] == ["G2", "G3", "G4", "G5"]
    assert all(len(item.variants) == 5 for item in selected)
    assert len(selected[-1].waypoints) == 3
    assert selected[1].variants[-1].start_delay_sec == 1.0
    assert selected[-1].variants[-1].dwell_sec == 1.2


def test_dynamic_actor_swept_boxes_stay_on_verified_free_map_cells():
    metadata = yaml.safe_load(
        (DEMO_ROOT / "rivermark_selected.yaml").read_text(encoding="utf-8")
    )
    image = cv2.imread(
        str(DEMO_ROOT / metadata["image"]), cv2.IMREAD_GRAYSCALE
    )
    origin_x, origin_y, _ = (float(value) for value in metadata["origin"])
    resolution = float(metadata["resolution"])
    height, width = image.shape
    physical = load_dynamic_scenario(DEMO_ROOT / "rivermark_dynamic.yaml")

    for case in physical.selected_cases("full_route_four_stage"):
        half_x = case.obstacle.size[0] / 2.0
        half_y = case.obstacle.size[1] / 2.0
        for start, end in zip(case.waypoints, case.waypoints[1:]):
            for sample in range(101):
                ratio = sample / 100.0
                x = start[0] + ratio * (end[0] - start[0])
                y = start[1] + ratio * (end[1] - start[1])
                minimum_column = math.floor(
                    (x - half_x - origin_x) / resolution
                )
                maximum_column = math.floor(
                    (x + half_x - origin_x) / resolution
                )
                minimum_bottom_row = math.floor(
                    (y - half_y - origin_y) / resolution
                )
                maximum_bottom_row = math.floor(
                    (y + half_y - origin_y) / resolution
                )
                assert 0 <= minimum_column <= maximum_column < width
                assert 0 <= minimum_bottom_row <= maximum_bottom_row < height
                for column in range(minimum_column, maximum_column + 1):
                    for bottom_row in range(
                        minimum_bottom_row, maximum_bottom_row + 1
                    ):
                        row = height - 1 - bottom_row
                        assert image[row, column] >= 250, (
                            case.case_id, sample, column, row
                        )


def test_rivermark_appearance_profiles_change_light_and_colour_only():
    profiles = load_appearance_profiles(
        DEMO_ROOT / "rivermark_appearance_profiles.yaml"
    )
    assert profiles.require("dim_warm").light_intensity_scale == 0.4
    assert profiles.require("dim_cool").color_temperature_k == 7500
    assert profiles.require("bright_warm").material_hue_shift_deg == 35.0

    source = (PROJECT_ROOT / "scripts/run_rivermark_demo.sh").read_text(
        encoding="utf-8"
    )
    assert '--appearance-config "${demo_dir}/rivermark_appearance_profiles.yaml"' in source


def test_outdoor_initial_pose_accepts_best_effort_clock_and_scan():
    source = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "robot_experiments"
        / "robot_experiments"
        / "outdoor_initial_pose.py"
    ).read_text(encoding="utf-8")
    assert 'ReliabilityPolicy.BEST_EFFORT' in source
    assert 'LaserScan, "/scan", self._on_scan, qos_profile_sensor_data' in source

    launch_source = (
        PROJECT_ROOT
        / "ros2_ws"
        / "src"
        / "robot_bringup"
        / "launch"
        / "rivermark_navigation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'executable="initial_pose_policy"' in launch_source
    assert '"initial_pose_source": "auto"' in launch_source
    assert '"feasible_only_largest_component": "false"' in launch_source
    assert '"controller_max_linear_velocity_mps"' in launch_source
    assert 'default_value="0.75"' in launch_source
    assert 'default_value="0.35"' in launch_source
    assert "TimerAction(" in launch_source
    assert "period=2.0" in launch_source

    demo_source = (PROJECT_ROOT / "scripts" / "run_rivermark_demo.sh").read_text(
        encoding="utf-8"
    )
    assert "--navigation-mode localization" in demo_source
    assert "RIVERMARK_MAX_LINEAR_SPEED_MPS" in demo_source
    assert "RIVERMARK_LINEAR_SPEED_STD_MPS" in demo_source
    assert "RIVERMARK_RENDERING_HZ" in demo_source
    assert "ISAAC_NAV__SIMULATION__RENDERING_HZ" in demo_source


def test_frozen_route_server_graph_and_support_mapping_share_one_id_space():
    graph = json.loads(
        (DEMO_ROOT / "rivermark_selected.geojson").read_text(encoding="utf-8")
    )
    mapping = json.loads(
        (DEMO_ROOT / "rivermark_selected_mapping.json").read_text(
            encoding="utf-8"
        )
    )
    points = {
        int(feature["properties"]["id"]): tuple(
            float(value) for value in feature["geometry"]["coordinates"]
        )
        for feature in graph["features"]
        if feature["geometry"]["type"] == "Point"
    }
    edges = {
        int(feature["properties"]["id"])
        for feature in graph["features"]
        if feature["geometry"]["type"] == "MultiLineString"
    }

    assert len(points) == 7859
    assert len(edges) == 15608
    assert set(int(key) for key in mapping["support_to_canonical_edge"]) == edges
    assert points[297] == (-11.9432, 186.578)
    assert points[3321] == (-23.4432, 157.21382)
