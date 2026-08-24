"""Launch the isolated NDT local-odometry shadow only when enabled."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    enabled = LaunchConfiguration("enabled")
    config = (
        get_package_share_directory("pointcloud_local_odometry")
        + "/config/ndt_shadow.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enabled",
                default_value="false",
                description="Start the isolated NDT odometry shadow",
            ),
            Node(
                condition=IfCondition(enabled),
                package="pointcloud_local_odometry",
                executable="ndt_local_odometry_node",
                name="ndt_local_odometry_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
