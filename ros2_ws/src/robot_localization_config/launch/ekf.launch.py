from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('robot_localization_config'))
    default_config = package_share / 'config' / 'ekf.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'ekf_params_file', default_value=str(default_config)),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                LaunchConfiguration('ekf_params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[('odometry/filtered', '/odom')],
        ),
    ])
