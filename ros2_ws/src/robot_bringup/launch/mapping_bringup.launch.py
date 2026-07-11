from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    core_launch = (
        Path(get_package_share_directory('robot_bringup'))
        / 'launch'
        / 'ros_stack.launch.py'
    )
    return LaunchDescription([
        DeclareLaunchArgument('odometry_mode', default_value='ideal'),
        DeclareLaunchArgument('structure_tf_source', default_value='isaac'),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        DeclareLaunchArgument(
            'wheel_odometry_params_file', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=''),
        DeclareLaunchArgument('use_self_filter', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core_launch)),
            launch_arguments={
                'operation': 'mapping',
                'odometry_mode': LaunchConfiguration('odometry_mode'),
                'structure_tf_source': LaunchConfiguration(
                    'structure_tf_source'),
                'posegraph_file': '',
                'robot_description_file': LaunchConfiguration(
                    'robot_description_file'),
                'wheel_odometry_params_file': LaunchConfiguration(
                    'wheel_odometry_params_file'),
                'nav2_params_file': LaunchConfiguration('nav2_params_file'),
                'use_self_filter': LaunchConfiguration('use_self_filter'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
