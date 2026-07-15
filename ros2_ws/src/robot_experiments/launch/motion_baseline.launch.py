from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("robot_experiments"),
                        "config",
                        "motion_baseline.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("environment_id", default_value=""),
            DeclareLaunchArgument("odometry_mode", default_value=""),
            DeclareLaunchArgument(
                "output_file", default_value="data/reports/motion_baseline.json"
            ),
            Node(
                package="robot_experiments",
                executable="motion_baseline_runner",
                name="motion_baseline_runner",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "config_file": LaunchConfiguration("config_file"),
                        "environment_id": LaunchConfiguration("environment_id"),
                        "odometry_mode": LaunchConfiguration("odometry_mode"),
                        "output_file": LaunchConfiguration("output_file"),
                    }
                ],
            ),
        ]
    )
