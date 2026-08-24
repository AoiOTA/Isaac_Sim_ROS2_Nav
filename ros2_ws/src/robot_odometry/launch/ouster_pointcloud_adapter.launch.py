"""Default-off strict Isaac Ouster PointCloud2 adapter."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    enabled = LaunchConfiguration("enabled")
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument(
            "input_topic", default_value="/lio/points_raw_isaac"
        ),
        DeclareLaunchArgument("output_topic", default_value="/lio/points_raw"),
        DeclareLaunchArgument(
            "expected_frame", default_value="lio_lidar_link"
        ),
        Node(
            condition=IfCondition(enabled),
            package="robot_odometry",
            executable="ouster_pointcloud_adapter",
            name="ouster_pointcloud_adapter",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "expected_frame": LaunchConfiguration("expected_frame"),
                "max_scan_duration_ns": 120_000_000,
                "use_sim_time": True,
            }],
        ),
    ])
