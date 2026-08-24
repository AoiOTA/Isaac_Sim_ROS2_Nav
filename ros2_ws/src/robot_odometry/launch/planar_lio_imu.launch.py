"""Explicitly enabled planar IMU adapter for FAST-LIO diagnostics."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create an adapter process only after explicit opt-in."""
    config = Path(get_package_share_directory('robot_odometry')) / (
        'config/planar_lio_imu.yaml'
    )
    enabled = LaunchConfiguration('enabled')
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('input_topic', default_value='/imu/data'),
        DeclareLaunchArgument('output_topic', default_value='/imu/lio'),
        Node(
            condition=IfCondition(enabled),
            package='robot_odometry',
            executable='planar_lio_imu_adapter',
            name='planar_lio_imu_adapter',
            output='screen',
            parameters=[
                str(config),
                {
                    'input_topic': LaunchConfiguration('input_topic'),
                    'output_topic': LaunchConfiguration('output_topic'),
                },
            ],
        ),
    ])
