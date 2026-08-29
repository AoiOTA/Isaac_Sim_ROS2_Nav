import ast
from collections import Counter
from pathlib import Path
import xml.etree.ElementTree as ET

import jsonschema
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).parents[1]
EXPECTED_PACKAGED_CONFIGS = (
    "attempt31_rivermark_appearance.yaml",
    "attempt31_rivermark_dynamic.yaml",
    "attempt31_rivermark_static.yaml",
    "final_rivermark_appearance.yaml",
    "final_rivermark_dynamic.yaml",
    "final_rivermark_static.yaml",
    "kujiale_4x20_dynamic_pair.yaml",
    "kujiale_4x20_static_pair.yaml",
    "kujiale_contact_observability_dynamic.yaml",
    "kujiale_dynamic_controlled_20.yaml",
    "kujiale_dynamic_full_route_5.yaml",
    "kujiale_dynamic_visual.yaml",
    "kujiale_dynamic_visual_g2_g3.yaml",
    "kujiale_dynamic_visual_g5_g1.yaml",
    "kujiale_g2_dynamic_safety_smoke.yaml",
    "kujiale_long_range_campaign.yaml",
    "kujiale_static_long_range.yaml",
    "kujiale_static_visual.yaml",
    "module1_targeted_teaching_kujiale_en.yaml",
    "module1_targeted_teaching_kujiale_sw.yaml",
    "module1_targeted_teaching_kujiale_t1.yaml",
    "module1_targeted_teaching_kujiale_t2.yaml",
    "module1_targeted_teaching_kujiale_v1.yaml",
    "module1_targeted_teaching_kujiale_v2.yaml",
    "module1_targeted_teaching_rosbag_qos.yaml",
    "optimal_reference.json",
    "scenario.schema.yaml",
    "static.yaml",
    "v6_estimated_calibration.yaml",
    "v6_final_kujiale_appearance.yaml",
    "v6_final_kujiale_dynamic.yaml",
    "v6_final_kujiale_static.yaml",
    "v6_imu_lidar_readiness.yaml",
    "v6_imu_regime_diagnostic.yaml",
    "v6_imu_regime_resources.json",
    "v6_kujiale_low_obstacle_causal.yaml",
    "v6_kujiale_low_obstacles_static.yaml",
    "v6_localization_causal.yaml",
    "v6_low_obstacle_phase_f_rosbag_qos.yaml",
    "v6_phase_g_causal.yaml",
    "v6_pilot_kujiale_dynamic_hotreset_v1.yaml",
    "v6_pilot_kujiale_static_hotreset.yaml",
    "v6_r3_phase2_kujiale_baseline.yaml",
    "v6_r5_phase_b_kujiale_exact_baseline.yaml",
    "v6_run4_shadow_probe.yaml",
    "v6_single_dynamic_low_obstacle.yaml",
)
EXPECTED_SOURCE_ONLY_CONFIGS = (
    "dynamic.yaml",
    "dynamic_benchmark.yaml",
    "dynamic_complex_route.yaml",
    "incremental_mapping.yaml",
    "isaac_kujiale_attempt21_static_ab.yaml",
    "isaac_kujiale_attempt21_static_ab_v10.yaml",
    "isaac_kujiale_attempt21_static_ab_v11.yaml",
    "isaac_kujiale_attempt21_static_ab_v12.yaml",
    "isaac_kujiale_attempt21_static_ab_v2.yaml",
    "isaac_kujiale_attempt21_static_ab_v3.yaml",
    "isaac_kujiale_attempt21_static_ab_v6.yaml",
    "isaac_kujiale_attempt21_static_ab_v7.yaml",
    "isaac_kujiale_attempt21_static_ab_v8.yaml",
    "isaac_kujiale_attempt21_static_ab_v9.yaml",
    "isaac_kujiale_attempt21_static_fusion_supplement_v15.yaml",
    "isaac_kujiale_attempt21_static_task_sensor_v13.yaml",
    "isaac_kujiale_dataset_v2_attempt_03_confirmation_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_03_confirmation_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_03_development_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_03_development_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_03_gate_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_03_gate_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_04_confirmation_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_04_confirmation_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_04_development_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_04_development_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_04_gate_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_04_gate_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_05_confirmation_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_05_confirmation_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_05_development_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_05_development_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_05_gate_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_05_gate_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_06_confirmation_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_06_confirmation_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_06_development_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_06_development_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_06_gate_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_06_gate_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_07_confirmation_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_07_confirmation_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_07_development_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_07_development_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_07_gate_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_07_gate_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_20_confirmation_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_20_confirmation_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_20_development_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_20_development_static.yaml",
    "isaac_kujiale_dataset_v2_attempt_20_gate_dynamic.yaml",
    "isaac_kujiale_dataset_v2_attempt_20_gate_static.yaml",
    "isaac_kujiale_dataset_v2_confirmation_dynamic.yaml",
    "isaac_kujiale_dataset_v2_confirmation_static.yaml",
    "isaac_kujiale_dataset_v2_development_dynamic.yaml",
    "isaac_kujiale_dataset_v2_development_static.yaml",
    "isaac_kujiale_dataset_v2_gate_dynamic.yaml",
    "isaac_kujiale_dataset_v2_gate_static.yaml",
    "isaac_kujiale_dataset_v2_smoke_dynamic.yaml",
    "isaac_kujiale_dataset_v2_smoke_static.yaml",
    "isaac_kujiale_dataset_v3_attempt21_confirmation_dynamic.yaml",
    "isaac_kujiale_dataset_v3_attempt21_confirmation_static.yaml",
    "isaac_kujiale_dataset_v3_attempt21_confirmation_static_v2.yaml",
    "isaac_kujiale_dataset_v3_attempt21_confirmation_static_v3.yaml",
    "isaac_kujiale_dataset_v3_attempt21_confirmation_static_v4.yaml",
    "isaac_kujiale_dataset_v3_attempt21_confirmation_static_v5.yaml",
    "isaac_kujiale_dataset_v3_attempt21_development_dynamic.yaml",
    "isaac_kujiale_dataset_v3_attempt21_development_static.yaml",
    "isaac_kujiale_dataset_v3_attempt21_gate_dynamic.yaml",
    "isaac_kujiale_dataset_v3_attempt21_gate_static.yaml",
    "isaac_kujiale_dataset_v3_attempt21_gate_static_v2.yaml",
    "isaac_kujiale_dataset_v3_attempt21_gate_static_v3.yaml",
    "isaac_kujiale_dataset_v3_attempt21_gate_static_v4.yaml",
    "isaac_kujiale_dataset_v3_attempt21_gate_static_v5.yaml",
    "isaac_kujiale_dataset_v3_attempt22_development_dynamic.yaml",
    "isaac_kujiale_dataset_v3_attempt23_final_confirmation_dynamic.yaml",
    "isaac_kujiale_dataset_v3_attempt23_final_confirmation_static.yaml",
    "isaac_kujiale_dataset_v3_attempt23_global_ab_dynamic.yaml",
    "isaac_kujiale_dataset_v3_attempt23_global_ab_static.yaml",
    "isaac_kujiale_dataset_v3_attempt23_global_shadow_dynamic.yaml",
    "isaac_kujiale_dataset_v3_attempt23_global_shadow_static.yaml",
    "kujiale_dynamic_long_range.yaml",
    "kujiale_dynamic_pilot.yaml",
    "kujiale_static_pilot.yaml",
    "motion_benchmark.yaml",
    "static_benchmark.yaml",
    "static_complex_route.yaml",
    "static_long_range.yaml",
)


def test_setup_packages_only_the_current_config_allowlist():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text()
    setup_tree = ast.parse(setup_source)
    assignments = [
        node
        for node in setup_tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == "PACKAGED_CONFIGS"
    ]
    assert len(assignments) == 1
    packaged_configs = ast.literal_eval(assignments[0].value)

    assert isinstance(packaged_configs, tuple)
    assert packaged_configs == EXPECTED_PACKAGED_CONFIGS
    assert len(packaged_configs) == 46
    assert len(packaged_configs) == len(set(packaged_configs))
    assert packaged_configs == tuple(sorted(packaged_configs))
    assert all((PACKAGE_ROOT / "config" / name).is_file() for name in packaged_configs)
    assert 'source_files("config", PACKAGED_CONFIGS)' in setup_source

    config_globs = [
        node
        for node in ast.walk(setup_tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "source_paths"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "config"
    ]
    assert not config_globs

    source_configs = tuple(
        sorted(
            path.name
            for path in (PACKAGE_ROOT / "config").iterdir()
            if path.is_file() and path.suffix in {".yaml", ".json"}
        )
    )
    assert len(source_configs) == 134
    source_only_configs = tuple(
        name for name in source_configs if name not in set(packaged_configs)
    )
    assert len(source_only_configs) == 88
    assert source_only_configs == EXPECTED_SOURCE_ONLY_CONFIGS


@pytest.mark.parametrize(
    "filename",
    [
        "initial_pose.launch.py",
        "experiment.launch.py",
        "scan_fault_bridge.launch.py",
    ],
)
def test_launch_files_define_generate_launch_description(filename):
    source = (PACKAGE_ROOT / "launch" / filename).read_text()
    tree = ast.parse(source)
    functions = {node.name for node in tree.body if isinstance(node, ast.FunctionDef)}
    assert "generate_launch_description" in functions
    assert "LaunchDescription" in source
    assert "Node(" in source


def test_package_metadata_declares_runtime_contract():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    assert root.findtext("name") == "robot_experiments"
    dependencies = {element.text for element in root.findall("exec_depend")}
    assert {
        "action_msgs",
        "geometry_msgs",
        "nav2_msgs",
        "nav_msgs",
        "rclpy",
        "rosgraph_msgs",
        "std_srvs",
        "tf2_ros",
    } <= dependencies
    assert root.find("export/build_type").text == "ament_python"


@pytest.mark.parametrize(
    "filename",
    [
        "static.yaml",
        "static_long_range.yaml",
        "static_benchmark.yaml",
        "dynamic.yaml",
        "dynamic_benchmark.yaml",
        "kujiale_contact_observability_dynamic.yaml",
        "kujiale_static_visual.yaml",
        "kujiale_dynamic_visual.yaml",
        "kujiale_dynamic_visual_g2_g3.yaml",
        "kujiale_dynamic_visual_g5_g1.yaml",
        "incremental_mapping.yaml",
        "final_rivermark_static.yaml",
        "final_rivermark_dynamic.yaml",
        "final_rivermark_appearance.yaml",
    ],
)
def test_source_examples_validate_against_source_schema(filename):
    schema = yaml.safe_load((PACKAGE_ROOT / "config" / "scenario.schema.yaml").read_text())
    instance = yaml.safe_load((PACKAGE_ROOT / "config" / filename).read_text())
    jsonschema.Draft202012Validator(schema).validate(instance)


def test_v6_final_kujiale_configs_freeze_twenty_unique_balanced_runs():
    config_root = PACKAGE_ROOT / "config"
    schema = yaml.safe_load((config_root / "scenario.schema.yaml").read_text())
    documents = {
        category: yaml.safe_load(
            (config_root / f"v6_final_kujiale_{category}.yaml").read_text()
        )
        for category in ("static", "dynamic", "appearance")
    }
    for document in documents.values():
        jsonschema.Draft202012Validator(schema).validate(document)

    static_seeds = documents["static"]["scenario"]["runs"]["seeds"]
    assert static_seeds == list(range(8601, 8621))
    assert len(static_seeds) == len(set(static_seeds)) == 20

    dynamic_rows = documents["dynamic"]["scenario"]["runs"]["matrix"]
    assert len(dynamic_rows) == 20
    assert len({
        (row["seed"], row["case_id"], row["variant_id"], row["condition_id"])
        for row in dynamic_rows
    }) == 20
    assert Counter(row["seed"] for row in dynamic_rows) == {
        seed: 5 for seed in range(8601, 8605)
    }
    assert Counter(row["variant_id"] for row in dynamic_rows) == {
        f"v{variant}": 4 for variant in range(1, 6)
    }

    appearance_rows = documents["appearance"]["scenario"]["runs"]["matrix"]
    assert len(appearance_rows) == 20
    assert len({
        (
            row["seed"], row["case_id"], row["variant_id"],
            row["appearance_profile_id"], row["condition_id"],
        )
        for row in appearance_rows
    }) == 20
    assert [row["seed"] for row in appearance_rows] == list(range(8601, 8621))
    assert Counter(row["appearance_profile_id"] for row in appearance_rows) == {
        "dim_warm": 5,
        "dim_cool": 5,
        "bright_warm": 5,
        "bright_cool": 5,
    }


def test_package_does_not_install_a_second_spawn_pose_truth_source():
    assert not (PACKAGE_ROOT / "config" / "spawn_poses.yaml").exists()
    launch_source = (PACKAGE_ROOT / "launch" / "initial_pose.launch.py").read_text()
    assert "ISAAC_NAV_SPAWN_POSES" in launch_source


def test_visual_route_wrapper_disables_all_project_evidence_output():
    root = PACKAGE_ROOT.parents[2]
    wrapper = (root / "scripts" / "run_visual_route.sh").read_text()
    launch = (PACKAGE_ROOT / "launch" / "experiment.launch.py").read_text()
    runner = (PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py").read_text()
    assert '"record_evidence:=false"' in wrapper
    assert "mkdir -p" not in wrapper
    assert 'DeclareLaunchArgument("record_evidence", default_value="true")' in launch
    assert 'DeclareLaunchArgument("record_bag", default_value="true")' in launch
    assert "if self._record_evidence:" in runner
    assert "if not self._record_bag:" in runner
    assert '"mcap_required": self._record_bag' in runner


def test_experiment_telemetry_records_contact_identity_diagnostics():
    runner = (PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py").read_text()
    assert '"/simulation/collision_diagnostics"' in runner


def test_experiment_telemetry_records_cognitive_obstacle_application():
    source = (
        PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py"
    ).read_text()
    tree = ast.parse(source)
    runner_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "ExperimentRunner"
    )
    recorder = next(
        node
        for node in runner_class.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "_begin_run_evidence"
    )
    topic_assignment = next(
        node
        for node in ast.walk(recorder)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "topics"
            for target in node.targets
        )
    )
    topics = ast.literal_eval(topic_assignment.value)
    assert {
        "/amcl_pose",
        "/bio_nav/module1/odom",
        "/simulation/reset_stop_gate/status",
        "/bio_nav/module2/srdr_edge_diagnostics",
        "/bio_nav/route_edge_costs",
        "/bio_nav/module2/cognitive_obstacles",
        "/bio_nav/cognitive_obstacle_layer/status",
        "/bio_nav/cognitive_risk_critic/status",
    } <= set(topics)
    assert len(topics) == len(set(topics))


def test_experiment_summary_requires_observed_executor_terminal_zero():
    source = (
        PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py"
    ).read_text()
    assert '"terminal_zero_confirmed": self._terminal_zero_confirmed' in source
    assert '"terminal_zero_reason": self._terminal_zero_reason' in source
    assert '"terminal_zero_timing": self._terminal_zero_timing()' in source
    assert "terminal_zero_confirmed=(" in source
    assert 'manifest.get("terminal_zero_confirmed") is True' in source
    assert '"first_zero_after_terminal_sec"' in source
    assert '"first_zero_after_navigation_sec"' not in source
    assert source.index("self._start_terminal_zero_observation()") < source.index(
        "nav2_succeeded, timed_out, nav2_status = self._navigate()"
    )
    assert 'self._mark_terminal_zero_barrier("navigate_action_return")' in source


def test_experiment_telemetry_records_the_complete_nearfield_safety_chain():
    runner = (
        PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py"
    ).read_text()
    for topic in (
        "/lidar/points_raw",
        "/lidar/points_scan",
        "/scan",
        "/scan_safety",
        "/local_costmap/costmap_raw",
        "/optimal_trajectory",
        "/trajectories",
        "/cmd_vel_nav",
        "/cmd_vel_smoothed",
        "/cmd_vel",
    ):
        assert f'"{topic}"' in runner
    for artifact in (
        "scan_safety.csv",
        "scan_safety.json",
    ):
        assert f'"{artifact}"' in runner


def test_experiment_telemetry_records_deterministic_appearance_pairs():
    runner = (
        PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py"
    ).read_text()
    for topic in (
        "/experiment/paired_appearance/baseline/image_raw",
        "/experiment/paired_appearance/variant/image_raw",
        "/experiment/paired_appearance/state",
    ):
        assert f'"{topic}"' in runner


def test_collision_monitor_startup_check_retries_short_queries_and_requires_stability():
    runner = (
        PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py"
    ).read_text()
    assert "deadline = time.monotonic() + self._reset_recovery_timeout_sec" in runner
    assert "query_deadline = min(deadline, time.monotonic() + 1.0)" in runner
    assert "future.cancel()" in runner
    assert 'latest_state = "query_timeout"' in runner
    assert "time.monotonic() - active_since" in runner
    assert ">= self._nav2_active_stability_sec" in runner


def test_attempt21_shadow_does_not_change_aggregate_costmap_bounds():
    fusion = (
        PACKAGE_ROOT.parent
        / "bio_nav_fusion/src/local_risk_grid_layer.cpp"
    ).read_text()
    assert "if (shadow_only_)" in fusion
    shadow_block = fusion.split("if (shadow_only_)", 1)[1].split("}", 1)[0]
    assert "must not enlarge the aggregate update bounds" in shadow_block
    assert "touch(" not in shadow_block
    assert "current_ = true" in shadow_block
    assert "return;" in shadow_block


def test_4x20_one_command_supervisor_keeps_stage_lifecycles_separate():
    root = PACKAGE_ROOT.parents[2]
    wrapper = (root / "scripts" / "run_kujiale_4x20_all.sh").read_text()
    assert 'run_kujiale_4x20_isaac.sh" "${mode}" --headless' in wrapper
    assert 'nav2_profile:="${nav2_profile}"' in wrapper
    assert "run_campaign pilot static" in wrapper
    assert "run_campaign static-pair" in wrapper
    assert "run_campaign pilot dynamic" in wrapper
    assert "run_campaign dynamic-pair" in wrapper
    assert wrapper.count("stop_stage") >= 3
    assert '"${SCRIPT_DIR}/run_kujiale_4x20.sh" report "${campaign_id}"' in wrapper
    assert 'setsid -- "${SCRIPT_DIR}/run_kujiale_4x20_isaac.sh"' not in wrapper
    assert 'setsid -- "${SCRIPT_DIR}/run_ros.sh"' not in wrapper
    assert "Keep them as direct children so their PIDs remain waitable" in wrapper
    assert "ros_launch_process_group_for()" in wrapper
    assert 'kill -INT -- "-${process_group}"' in wrapper
    assert "stopping ${active_mode} ROS launch process group" in wrapper


def test_experiment_manifest_records_consumed_attempt31_arm():
    runner = (
        PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py"
    ).read_text(encoding="utf-8")

    assert '"experiment_arm": self._experiment_arm or None' in runner


def test_rivermark_campaign_binds_runtime_profile_and_checks_child_evidence():
    root = PACKAGE_ROOT.parents[2]
    wrapper = (root / "scripts" / "run_rivermark_campaign.sh").read_text()

    assert "nav2_profile:=bio_nav_planning_only" in wrapper
    assert "Rivermark evidence count mismatch" in wrapper
    assert 'summary.get("experiment_arm") == arm' in wrapper
    assert "ROS_DOMAIN_ID must be an integer in [0, 232]" in wrapper
    assert 'runs.get("matrix", runs.get("seeds", []))' in wrapper
    assert "collision rates are evaluated later" in wrapper
    assert "runtime_controller_contract.json" in wrapper
    assert "timeout 5 ros2 lifecycle get /controller_server" in wrapper
    assert "timeout 5 ros2 service type /simulation/reset" in wrapper
    assert 'timeout 5 ros2 param get "${node}" "${parameter}"' in wrapper
    assert 'controller_max_linear_velocity_mps="0.75"' in wrapper
    assert 'controller_linear_velocity_std_mps="0.35"' in wrapper
    assert 'rendering_hz="30"' in wrapper
    assert "resume:=true" in wrapper
    assert "runtime_tile_cache_contract.json" in wrapper


def test_large_global_costmap_is_latest_state_not_reliable_backlog():
    runner = (
        PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py"
    ).read_text(encoding="utf-8")
    route_node = (
        PACKAGE_ROOT.parent
        / "robot_route_planner"
        / "robot_route_planner"
        / "ros_node.py"
    ).read_text(encoding="utf-8")

    for source in (runner, route_node):
        assert '"/global_costmap/costmap_raw"' in source
        assert "ReliabilityPolicy.BEST_EFFORT" in source
        assert "depth=1" in source


def test_g2_dynamic_safety_smoke_is_single_route_and_module2_free():
    root = PACKAGE_ROOT.parents[2]
    wrapper = (root / "scripts" / "run_g2_dynamic_safety_smoke.sh").read_text()
    assert "kujiale_g2_dynamic_safety_smoke.yaml" in wrapper
    assert "run_kujiale_4x20_isaac.sh\" dynamic --headless" in wrapper
    assert "nav2_profile:=dynamic_avoidance" in wrapper
    assert "FollowPath.CostCritic.cost_weight" in wrapper
    assert "inflation_layer.inflation_radius" in wrapper
    for actor_id in ("local_bypass_actor", "g2_g3_exit_actor", "g5_g1_crossing_actor"):
        assert actor_id in wrapper
    assert '"minimum_clearance_m_by_actor"' in wrapper
    assert "< 0.10" not in wrapper
    assert "module2" not in wrapper.lower()


def test_authorization_isaac_uses_rgb_ingress_without_depth_navigation_profile():
    root = PACKAGE_ROOT.parents[2]
    wrapper = (root / "scripts" / "run_kujiale_authorization_isaac.sh").read_text()
    assert '--camera-profile monitoring' in wrapper
    assert '--camera-profile off' not in wrapper
    assert 'run_kujiale_4x20_isaac.sh' in wrapper


def test_4x20_preflight_does_not_require_ripgrep_after_sourcing_ros():
    root = PACKAGE_ROOT.parents[2]
    controller = (root / "scripts" / "run_kujiale_4x20.sh").read_text()
    assert 'find "${environment_root}" -type f' in controller
    assert "rg --files \"${environment_root}\"" not in controller
    assert "verify_pilot_evidence()" in controller
    assert "A failed pilot must never allow the formal 40-round stage to run." in controller
    assert 'if ".incomplete-" in manifest_path.parent.name:' in controller
    assert "A failed pilot is intentionally retained under a sibling" in controller
    assert "evidence, never a second current pilot result." in controller


def test_experiment_launch_forces_run_indices_to_the_runner_string_contract():
    launch_source = (PACKAGE_ROOT / "launch" / "experiment.launch.py").read_text()
    assert "from launch_ros.parameter_descriptions import ParameterValue" in launch_source
    assert '"run_indices": ParameterValue(' in launch_source
    assert 'LaunchConfiguration("run_indices"), value_type=str' in launch_source


def test_experiment_launch_exposes_strict_slam_buffer_clear_switch_and_manifest():
    launch_source = (PACKAGE_ROOT / "launch" / "experiment.launch.py").read_text()
    runner_source = (
        PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py"
    ).read_text()

    assert (
        '"clear_slam_localization_buffer", default_value="true"'
        in launch_source
    )
    assert 'LaunchConfiguration("clear_slam_localization_buffer")' in launch_source
    assert '"clear_slam_localization_buffer": ParameterValue(' in launch_source
    assert "value_type=bool" in launch_source
    assert '"clear_slam_localization_buffer", True' in runner_source
    assert "if self._clear_slam_localization_buffer" in runner_source
    assert (
        '"clear_slam_localization_buffer": ('
        in runner_source
    )


def test_experiment_launch_types_and_forwards_module2_planning_readiness():
    launch_source = (PACKAGE_ROOT / "launch" / "experiment.launch.py").read_text()
    runner_source = (
        PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py"
    ).read_text()

    assert (
        '"require_module2_planning_ready", default_value="false"'
        in launch_source
    )
    assert (
        '"module2_planning_ready_timeout_sec", default_value="30.0"'
        in launch_source
    )
    assert 'LaunchConfiguration("require_module2_planning_ready")' in launch_source
    assert 'LaunchConfiguration(\n                                "module2_planning_ready_timeout_sec"' in launch_source
    assert launch_source.count("value_type=bool") >= 1
    assert launch_source.count("value_type=float") >= 1
    gate = runner_source.index(
        "if self._require_module2_planning_ready and not self._wait_until("
    )
    dispatch = runner_source.index("nav2_succeeded, timed_out, nav2_status = self._navigate()")
    assert gate < dispatch
    assert "lambda: self._planning_prior_ready_streak >= 5" in runner_source
    assert "Module2 planning prior did not become goal-query ready" in runner_source


def test_experiment_launch_exposes_fail_closed_pregoal_evidence_fence():
    launch_source = (PACKAGE_ROOT / "launch" / "experiment.launch.py").read_text()
    runner = (PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py").read_text()
    assert 'DeclareLaunchArgument("require_pregoal_authorization"' in launch_source
    assert 'DeclareLaunchArgument("pregoal_authorization_path"' in launch_source
    assert 'DeclareLaunchArgument("lifecycle_jsonl_path"' in launch_source
    for name in (
        "pregoal_expected_receipt",
        "pregoal_expected_schema",
        "pregoal_expected_campaign",
        "pregoal_expected_prereg_sha256",
    ):
        assert f'DeclareLaunchArgument("{name}"' in launch_source
        assert f'"{name}": LaunchConfiguration(' in launch_source
    assert "pre-goal authorization requires exactly one run index" in runner
    assert 'self._lifecycle_event("goal_dispatched")' in runner


def test_4x20_pilot_resume_requires_a_previous_success_and_preserves_formal_failures():
    root = PACKAGE_ROOT.parents[2]
    controller = (root / "scripts" / "run_kujiale_4x20.sh").read_text()
    runner = (PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py").read_text()
    launch = (PACKAGE_ROOT / "launch" / "experiment.launch.py").read_text()
    assert '"require_successful_resume:=true"' in controller
    assert 'DeclareLaunchArgument("require_successful_resume", default_value="false")' in launch
    assert "if self._require_successful_resume:" in runner
    assert 'manifest.get("result") != "success"' in runner
    assert 'manifest.get("terminal_zero_confirmed") is not True' in runner
    assert 'summary.get("strict_success") is not True' in runner
    assert "fully recorded *failed* pilot must be quarantined and retried" in runner


def test_incremental_map_comparison_has_an_installed_cli():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text()
    assert (
        "incremental_map_compare = "
        "robot_experiments.incremental_map_compare:main"
    ) in setup_source
    assert (
        "navigation_benchmark = "
        "robot_experiments.navigation_benchmark:main"
    ) in setup_source
    assert (
        "motion_benchmark = "
        "robot_experiments.motion_benchmark:main"
    ) in setup_source


def test_runner_only_publishes_a21_route_goals_and_never_controls_or_localizes_robot():
    source = (PACKAGE_ROOT / "robot_experiments" / "experiment_runner.py").read_text()
    tree = ast.parse(source)
    attribute_calls = {
        node.func.attr
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    # Final Qualification adds one explicit mission-dispatch publisher.  The
    # coordinator, Nav2 and Module3 remain the only planning/control owners.
    assert "create_publisher" in attribute_calls
    assert source.count("self.create_publisher(") == 1
    assert 'PoseStamped, "/bio_nav/route_goal"' in source
    assert 'Twist, "/cmd_vel"' not in source
    # A read-only /cmd_vel subscription is allowed for motion-quality metrics.
    assert "create_subscription(" in source
    assert '"command_topic", "/cmd_vel"' in source
    # The evidence recorder may subscribe to /initialpose, but the runner
    # must never create a publisher or construct an initial-pose command.
    assert "create_publisher(PoseWithCovarianceStamped" not in source
    assert "self._scenario.goal" in source
    assert "_verify_dynamic_runtime_contract" in source
    assert "dynamic_obstacles_config_sha256" in source
    assert '"/simulation/localization_seeded"' in source
    assert "localization_seed_event_grace_sec" in source
    assert "post-reset spawn-aligned TF/sample recovery gate" in source
    assert "stamp_s > tf_stamp_barrier_s" in source
    assert "ExternalShutdownException" in source
    assert "ExperimentIsolationError" in source
    assert "odom.stamp_s > sample_stamp_barrier_s" in source
    assert '"reset_map_base_translation_tolerance_m", 0.05' in source
    assert '"reset_map_base_translation_tolerance_m": (' in source

    launch_source = (PACKAGE_ROOT / "launch" / "experiment.launch.py").read_text()
    assert '"reset_map_base_translation_tolerance_m", default_value="0.05"' in launch_source
    assert (
        'LaunchConfiguration(\n                                '
        '"reset_map_base_translation_tolerance_m"'
    ) in launch_source
    assert "value_type=float" in launch_source


def test_initial_pose_contract_waits_for_clock_and_uses_reliable_qos():
    source = (PACKAGE_ROOT / "robot_experiments" / "initial_pose_publisher.py").read_text()
    assert '"/clock"' in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "require_calibrated=True" in source
    assert "wait_for_odom_to_base_tf" in source
    assert "lookup_transform(" in source
    assert "self._odom_frame, self._base_frame, Time()" in source
    assert "wait_for_map_to_odom_tf" not in source
    assert "publish_count" in source
    assert "Buffer(node=self)" in source
    assert '"/initial_pose/reseed"' in source
    assert "_initial_pose_callback" in source
    assert "external /initialpose accepted" in source
    assert "simulation clock rollback" in source
    assert "stay_alive_for_reseed" in source
    assert '"/scan"' in source
    assert '"/simulation/reset_event"' in source
    assert '"/initial_pose/status"' in source
    assert "PostResetScanBarrier" in source
    assert "manual RViz initial pose remains authoritative" in source
