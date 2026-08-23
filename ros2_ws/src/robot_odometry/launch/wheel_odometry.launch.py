from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_odometry'))
    default_config = package_share / 'config' / 'wheel_odometry.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'wheel_odometry_params_file', default_value=str(default_config)),
        DeclareLaunchArgument(
            'yaw_disagreement_guard_enabled', default_value='false'),
        Node(
            package='robot_odometry',
            executable='wheel_odometry_node',
            name='wheel_odometry',
            output='screen',
            parameters=[
                LaunchConfiguration('wheel_odometry_params_file'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'yaw_disagreement_guard_enabled': LaunchConfiguration(
                        'yaw_disagreement_guard_enabled'),
                },
            ],
        ),
    ])
