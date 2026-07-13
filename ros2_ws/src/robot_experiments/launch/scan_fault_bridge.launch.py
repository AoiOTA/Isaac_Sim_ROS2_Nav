"""Launch the opt-in LaserScan fault bridge for Collision Monitor tests."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/scan"),
            DeclareLaunchArgument("output_topic", default_value="/scan_fault"),
            DeclareLaunchArgument(
                "control_topic", default_value="/scan_fault/control"
            ),
            DeclareLaunchArgument(
                "status_topic", default_value="/scan_fault/status"
            ),
            DeclareLaunchArgument(
                "reset_event_topic", default_value="/simulation/reset_event"
            ),
            DeclareLaunchArgument("status_period_s", default_value="1.0"),
            DeclareLaunchArgument(
                "reset_rollback_tolerance_s", default_value="0.000001"
            ),
            Node(
                package="robot_experiments",
                executable="scan_fault_bridge",
                name="scan_fault_bridge",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                        "control_topic": LaunchConfiguration("control_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "reset_event_topic": LaunchConfiguration(
                            "reset_event_topic"
                        ),
                        "status_period_s": LaunchConfiguration("status_period_s"),
                        "reset_rollback_tolerance_s": LaunchConfiguration(
                            "reset_rollback_tolerance_s"
                        ),
                    }
                ],
            ),
        ]
    )
