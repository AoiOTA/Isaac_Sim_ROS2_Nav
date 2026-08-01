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
            DeclareLaunchArgument("authorization_only", default_value="false"),
            DeclareLaunchArgument("resume", default_value="false"),
            DeclareLaunchArgument("require_successful_resume", default_value="false"),
            DeclareLaunchArgument("run_indices", default_value=""),
            DeclareLaunchArgument("robot_config_file", default_value=""),
            DeclareLaunchArgument("nav2_config_file", default_value=""),
            DeclareLaunchArgument("nav2_profile", default_value=""),
            DeclareLaunchArgument("dynamic_case_id", default_value=""),
            DeclareLaunchArgument("dynamic_variant_id", default_value=""),
            DeclareLaunchArgument("dynamic_seed", default_value="0"),
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
                        "authorization_only": ParameterValue(
                            LaunchConfiguration("authorization_only"), value_type=bool
                        ),
                        "resume": LaunchConfiguration("resume"),
                        "require_successful_resume": ParameterValue(
                            LaunchConfiguration("require_successful_resume"),
                            value_type=bool,
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
                        "dynamic_case_id": LaunchConfiguration("dynamic_case_id"),
                        "dynamic_variant_id": LaunchConfiguration("dynamic_variant_id"),
                        "dynamic_seed": LaunchConfiguration("dynamic_seed"),
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
