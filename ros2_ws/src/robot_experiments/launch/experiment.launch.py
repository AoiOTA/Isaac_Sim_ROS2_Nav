from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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
            DeclareLaunchArgument("robot_config_file", default_value=""),
            DeclareLaunchArgument("nav2_config_file", default_value=""),
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
                        "robot_config_file": LaunchConfiguration("robot_config_file"),
                        "nav2_config_file": LaunchConfiguration("nav2_config_file"),
                    }
                ],
            ),
        ]
    )
