"""Launch the isolated KISS-ICP local-odometry shadow only when enabled."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    enabled = LaunchConfiguration("enabled")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = (
        get_package_share_directory("pointcloud_local_odometry")
        + "/config/kiss_shadow.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enabled",
                default_value="false",
                description="Start the isolated KISS-ICP odometry shadow",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the simulation clock",
            ),
            Node(
                condition=IfCondition(enabled),
                package="kiss_icp",
                executable="kiss_icp_node",
                name="kiss_icp_node",
                output="screen",
                parameters=[config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("pointcloud_topic", "/lio/points_raw"),
                    ("kiss/odometry", "/local_odom/kiss_shadow"),
                ],
            ),
        ]
    )
