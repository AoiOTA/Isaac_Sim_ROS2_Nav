from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_experiments"), "config", "static.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "spawn_poses_file",
                default_value=EnvironmentVariable(
                    "ISAAC_NAV_SPAWN_POSES",
                    default_value="",
                ),
                description=(
                    "Required path to isaac_sim/configs/spawn_poses.yaml; no package copy is installed"
                ),
            ),
            DeclareLaunchArgument("output_directory", default_value="data/experiment_runs"),
            DeclareLaunchArgument("record_evidence", default_value="true"),
            DeclareLaunchArgument("record_bag", default_value="true"),
            DeclareLaunchArgument(
                "clear_slam_localization_buffer", default_value="true"
            ),
            DeclareLaunchArgument(
                "require_module2_planning_ready", default_value="false"
            ),
            DeclareLaunchArgument(
                "module2_planning_ready_timeout_sec", default_value="30.0"
            ),
            DeclareLaunchArgument("authorization_only", default_value="false"),
            DeclareLaunchArgument("resume", default_value="false"),
            DeclareLaunchArgument("require_successful_resume", default_value="false"),
            DeclareLaunchArgument("fail_stop", default_value="false"),
            DeclareLaunchArgument("fail_stop_metric_contract", default_value=""),
            DeclareLaunchArgument("run_indices", default_value=""),
            DeclareLaunchArgument("robot_config_file", default_value=""),
            DeclareLaunchArgument("nav2_config_file", default_value=""),
            DeclareLaunchArgument("nav2_profile", default_value=""),
            DeclareLaunchArgument("experiment_arm", default_value=""),
            DeclareLaunchArgument("condition_stack_id", default_value=""),
            DeclareLaunchArgument("stack_session_id", default_value=""),
            DeclareLaunchArgument("dynamic_case_id", default_value=""),
            DeclareLaunchArgument("dynamic_variant_id", default_value=""),
            DeclareLaunchArgument("dynamic_seed", default_value="0"),
            DeclareLaunchArgument(
                "reset_map_base_translation_tolerance_m", default_value="0.05"
            ),
            # Attempt-23 paired A/B: the guidance sidecar hosts the
            # runner-facing NavigateToPose server; campaigns point the runner
            # at it through this override.  The default preserves every
            # existing campaign byte-for-byte.
            DeclareLaunchArgument("navigate_action", default_value="/navigate_to_pose"),
            DeclareLaunchArgument(
                "navigation_execution_backend",
                default_value="navigate_to_pose",
                description=(
                    "navigate_to_pose for legacy direct dispatch; route_guided "
                    "for the A21 Route Server/coordinator data path"
                ),
            ),
            # Evidence-only fence used by Stage 2.2-R2C4-R2.  These values do
            # not alter localization, planning, costmaps, or control.
            DeclareLaunchArgument("require_pregoal_authorization", default_value="false"),
            DeclareLaunchArgument("pregoal_authorization_path", default_value=""),
            DeclareLaunchArgument("lifecycle_jsonl_path", default_value=""),
            DeclareLaunchArgument("pregoal_expected_receipt", default_value=""),
            DeclareLaunchArgument("pregoal_expected_schema", default_value=""),
            DeclareLaunchArgument("pregoal_expected_campaign", default_value=""),
            DeclareLaunchArgument("pregoal_expected_prereg_sha256", default_value=""),
            Node(
                package="robot_experiments",
                executable="experiment_runner",
                name="experiment_runner",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "scenario_file": LaunchConfiguration("scenario_file"),
                        "spawn_poses_file": LaunchConfiguration("spawn_poses_file"),
                        "output_directory": LaunchConfiguration("output_directory"),
                        "record_evidence": LaunchConfiguration("record_evidence"),
                        "record_bag": LaunchConfiguration("record_bag"),
                        "clear_slam_localization_buffer": ParameterValue(
                            LaunchConfiguration("clear_slam_localization_buffer"),
                            value_type=bool,
                        ),
                        "require_module2_planning_ready": ParameterValue(
                            LaunchConfiguration("require_module2_planning_ready"),
                            value_type=bool,
                        ),
                        "module2_planning_ready_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "module2_planning_ready_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "authorization_only": ParameterValue(
                            LaunchConfiguration("authorization_only"), value_type=bool
                        ),
                        "resume": LaunchConfiguration("resume"),
                        "require_successful_resume": ParameterValue(
                            LaunchConfiguration("require_successful_resume"),
                            value_type=bool,
                        ),
                        "fail_stop": ParameterValue(
                            LaunchConfiguration("fail_stop"), value_type=bool
                        ),
                        "fail_stop_metric_contract": LaunchConfiguration(
                            "fail_stop_metric_contract"
                        ),
                        # A single index such as "2" would otherwise be
                        # inferred as an INTEGER by launch, while the runner
                        # intentionally accepts a comma-separated STRING.
                        "run_indices": ParameterValue(
                            LaunchConfiguration("run_indices"), value_type=str
                        ),
                        "robot_config_file": LaunchConfiguration("robot_config_file"),
                        "nav2_config_file": LaunchConfiguration("nav2_config_file"),
                        "nav2_profile": LaunchConfiguration("nav2_profile"),
                        # Values such as "off" are valid arm identifiers but
                        # YAML would otherwise coerce them to boolean false.
                        "experiment_arm": ParameterValue(
                            LaunchConfiguration("experiment_arm"), value_type=str
                        ),
                        "condition_stack_id": ParameterValue(
                            LaunchConfiguration("condition_stack_id"),
                            value_type=str,
                        ),
                        "stack_session_id": ParameterValue(
                            LaunchConfiguration("stack_session_id"),
                            value_type=str,
                        ),
                        "dynamic_case_id": LaunchConfiguration("dynamic_case_id"),
                        "dynamic_variant_id": LaunchConfiguration("dynamic_variant_id"),
                        "dynamic_seed": LaunchConfiguration("dynamic_seed"),
                        "reset_map_base_translation_tolerance_m": ParameterValue(
                            LaunchConfiguration(
                                "reset_map_base_translation_tolerance_m"
                            ),
                            value_type=float,
                        ),
                        "navigate_action": LaunchConfiguration("navigate_action"),
                        "navigation_execution_backend": LaunchConfiguration(
                            "navigation_execution_backend"
                        ),
                        "require_pregoal_authorization": ParameterValue(
                            LaunchConfiguration("require_pregoal_authorization"),
                            value_type=bool,
                        ),
                        "pregoal_authorization_path": LaunchConfiguration(
                            "pregoal_authorization_path"
                        ),
                        "lifecycle_jsonl_path": LaunchConfiguration(
                            "lifecycle_jsonl_path"
                        ),
                        "pregoal_expected_receipt": LaunchConfiguration(
                            "pregoal_expected_receipt"
                        ),
                        "pregoal_expected_schema": LaunchConfiguration(
                            "pregoal_expected_schema"
                        ),
                        "pregoal_expected_campaign": LaunchConfiguration(
                            "pregoal_expected_campaign"
                        ),
                        "pregoal_expected_prereg_sha256": LaunchConfiguration(
                            "pregoal_expected_prereg_sha256"
                        ),
                    }
                ],
            ),
        ]
    )
