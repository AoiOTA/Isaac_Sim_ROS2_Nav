from pathlib import Path
import json

import cv2
import math
import pytest
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


def test_final_static_layout_authors_one_low_stationary_box_on_free_cells():
    metadata = yaml.safe_load(
        (DEMO_ROOT / "rivermark_selected.yaml").read_text(encoding="utf-8")
    )
    image = cv2.imread(
        str(DEMO_ROOT / metadata["image"]), cv2.IMREAD_GRAYSCALE
    )
    origin_x, origin_y, _ = (float(value) for value in metadata["origin"])
    resolution = float(metadata["resolution"])
    height, width = image.shape
    physical = load_dynamic_scenario(
        DEMO_ROOT / "final_rivermark_static_obstacles.yaml"
    )

    assert len(physical.obstacles) == 1
    obstacle = physical.obstacles[0]
    assert obstacle.obstacle_id == "rivermark_static_arc12"
    assert obstacle.mode == "stationary"
    assert obstacle.size == pytest.approx((0.30, 0.30, 0.16))
    assert obstacle.mass == pytest.approx(5.0)
    assert obstacle.start == pytest.approx((11.663, 126.343, 6.29))
    assert obstacle.end == obstacle.start
    assert obstacle.post_motion == "hold"
    for obstacle in physical.obstacles:
        half_x, half_y = obstacle.size[0] / 2.0, obstacle.size[1] / 2.0
        minimum_column = math.floor(
            (obstacle.start[0] - half_x - origin_x) / resolution
        )
        maximum_column = math.floor(
            (obstacle.start[0] + half_x - origin_x) / resolution
        )
        minimum_bottom_row = math.floor(
            (obstacle.start[1] - half_y - origin_y) / resolution
        )
        maximum_bottom_row = math.floor(
            (obstacle.start[1] + half_y - origin_y) / resolution
        )
        assert 0 <= minimum_column <= maximum_column < width
        assert 0 <= minimum_bottom_row <= maximum_bottom_row < height
        for column in range(minimum_column, maximum_column + 1):
            for bottom_row in range(minimum_bottom_row, maximum_bottom_row + 1):
                assert image[height - 1 - bottom_row, column] >= 250


def test_final_dynamic_layout_uses_one_low_crossing_actor():
    physical = load_dynamic_scenario(
        DEMO_ROOT / "final_rivermark_dynamic.yaml"
    )
    selected = physical.selected_cases("crossing")

    assert len(selected) == 1
    case = selected[0]
    assert case.case_id == "crossing"
    assert case.trigger_group == "G3"
    assert case.obstacle.obstacle_id == "rivermark_crossing_cart"
    assert case.obstacle.size == pytest.approx((0.30, 0.30, 0.16))
    assert case.obstacle.mass == pytest.approx(4.0)
    assert case.waypoints == (
        (-16.9516, 150.855, 6.29),
        (-20.469, 148.3825, 6.29),
    )
    assert case.obstacle.speed == pytest.approx(0.55)
    assert case.max_acceleration == pytest.approx(0.50)
    assert case.obstacle.post_motion == "retire"
    assert [variant.start_delay_sec for variant in case.variants] == pytest.approx(
        [0.0, 0.15, 0.30, 0.45, 0.60]
    )


def test_final_campaign_wrapper_enables_fail_stop_and_new_scenario_identity():
    source = (PROJECT_ROOT / "scripts/run_final_rivermark_campaign.sh").read_text(
        encoding="utf-8"
    )
    campaign = (PROJECT_ROOT / "scripts/run_rivermark_campaign.sh").read_text(
        encoding="utf-8"
    )

    assert "RIVERMARK_SCENARIO_REVISION=final_rivermark" in source
    assert "RIVERMARK_FAIL_STOP=1" in source
    assert 'fail_stop:="${fail_stop}"' in campaign
    assert 'require_successful_resume:="${fail_stop}"' in campaign
    assert '"${condition}" == "static" || "${condition}" == "appearance"' in campaign
    assert 'dynamic_case="crossing"' in campaign
    assert 'RIVERMARK_DYNAMIC_CASE="${dynamic_case}"' in campaign


def test_final_visual_wrapper_selects_one_physical_obstacle_per_condition():
    source = (PROJECT_ROOT / "scripts/run_rivermark_visual.sh").read_text(
        encoding="utf-8"
    )

    assert 'RIVERMARK_VISUAL_REVISION:-final' in source
    assert "final_rivermark_static_obstacles.yaml" in source
    assert "final_rivermark_dynamic.yaml" in source
    assert 'export RIVERMARK_DYNAMIC_CASE="crossing"' in source
    assert "RIVERMARK_PHYSICAL_OBSTACLES=1" in source


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
    assert 'executable="v310_visualizer"' in launch_source
    assert '"region_config_file": str(files["region_config_file"])' in launch_source
    assert '"waypoint_config_file": str(files["waypoint_config_file"])' in launch_source
    assert 'description_share / "rviz" / "rivermark.rviz"' in launch_source
    assert 'DeclareLaunchArgument("use_rviz", default_value="true")' in launch_source

    demo_source = (PROJECT_ROOT / "scripts" / "run_rivermark_demo.sh").read_text(
        encoding="utf-8"
    )
    assert "--navigation-mode localization" in demo_source
    assert "RIVERMARK_MAX_LINEAR_SPEED_MPS" in demo_source
    assert "RIVERMARK_LINEAR_SPEED_STD_MPS" in demo_source
    assert "RIVERMARK_RENDERING_HZ" in demo_source
    assert "ISAAC_NAV__SIMULATION__RENDERING_HZ" in demo_source
    assert 'waypoint_config_file:="${demo_dir}/rivermark_demo_goals.yaml"' in demo_source
    assert 'use_rviz:="${rviz_enabled}"' in demo_source
    assert 'RIVERMARK_MODULE2_STARTUP_TIMEOUT_S' in demo_source
    assert '[[ ! -S "${socket}" ]]' in demo_source
    assert 'kill -0 "${module2_pid}"' in demo_source
    assert 'if [[ -n "${guidance_profile}" ]]' in demo_source
    assert 'bridge_launch_args+=(guidance_profile:="${guidance_profile}")' in demo_source
    assert 'RIVERMARK_BRIDGE_STARTUP_TIMEOUT_S' in demo_source
    assert 'grep -Fxq /bio_nav_ros_bridge' in demo_source
    assert 'grep -Fxq /bio_nav_edge_prior_bridge' in demo_source


def test_visual_launcher_waits_for_complete_runtime_and_cleans_process_groups():
    source = (PROJECT_ROOT / "scripts" / "run_rivermark_demo.sh").read_text(
        encoding="utf-8"
    )

    # Module2 must finish loading before the 12 GB USD competes for resources,
    # and RViz must not open until Isaac publishes real sensor messages.
    module2_start = source.index("Starting Rivermark Module2 before Isaac")
    isaac_start = source.index('"${module3_root}/scripts/run_isaac.sh"')
    sensor_ready = source.index("Rivermark Isaac sensors ready")
    nav2_start = source.index(
        "setsid -- ros2 launch robot_bringup rivermark_navigation.launch.py"
    )
    assert module2_start < isaac_start < sensor_ready < nav2_start

    assert 'remove_stale_module2_runtime "${socket}"' in source
    stale_cleanup = source[
        source.index("remove_stale_module2_runtime()"):
        source.index("cleanup()")
    ]
    assert "expected probe status" in stale_cleanup
    assert "return 0" in stale_cleanup
    assert 'setsid -- conda run --no-capture-output' in source
    assert 'PointCloud2,\n    "/lidar/points_raw"' in source
    assert 'required = {"clock", "lidar_points_raw", "odom"}' in source
    assert 'LaserScan, "/scan"' not in source
    assert "except KeyboardInterrupt:" in source
    assert "if rclpy.ok():" in source
    assert 'remember_process_group "${isaac_pid}"' in source
    assert 'stop_managed_process_groups INT' in source
    assert 'stop_managed_process_groups KILL' in source
    assert 'isaac-console.log' in source
    assert '>"${isaac_console_log}" 2>&1 &' in source
    assert 'recent actionable log lines' in source


def test_manual_rviz_launcher_disables_automatic_waypoint_publication():
    manual_source = (
        PROJECT_ROOT / "scripts" / "run_rivermark_manual.sh"
    ).read_text(encoding="utf-8")
    visual_source = (
        PROJECT_ROOT / "scripts" / "run_rivermark_visual.sh"
    ).read_text(encoding="utf-8")
    demo_source = (
        PROJECT_ROOT / "scripts" / "run_rivermark_demo.sh"
    ).read_text(encoding="utf-8")
    rviz_source = (
        PROJECT_ROOT
        / "ros2_ws/src/robot_description/rviz/rivermark.rviz"
    ).read_text(encoding="utf-8")

    assert "export RIVERMARK_AUTO_GOAL=0" in manual_source
    assert "export RIVERMARK_VISUAL_ROUTE=0" in manual_source
    assert 'RIVERMARK_AUTO_GOAL="${RIVERMARK_AUTO_GOAL:-1}"' in visual_source
    assert "Rivermark manual navigation ready" in demo_source
    assert "Class: rviz_default_plugins/SetGoal" in rviz_source
    assert "Value: /bio_nav/route_goal" in rviz_source


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
