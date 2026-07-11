from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TRUE_VALUES = {'1', 'true', 'yes', 'on'}
FALSE_VALUES = {'0', 'false', 'no', 'off'}


def _as_bool(value, name):
    normalized = value.strip().lower()
    if normalized in TRUE_VALUES:
        return True
    if normalized in FALSE_VALUES:
        return False
    raise RuntimeError(f'{name} must be a boolean, got: {value!r}')


def _launch_setup(context):
    use_self_filter = _as_bool(
        LaunchConfiguration('use_self_filter').perform(context),
        'use_self_filter',
    )
    input_topic = (
        LaunchConfiguration('filtered_cloud_topic').perform(context)
        if use_self_filter
        else LaunchConfiguration('raw_cloud_topic').perform(context)
    )

    actions = []
    if use_self_filter:
        actions.append(LogInfo(msg=(
            'Self-filter routing enabled: an external filter must publish '
            f'{input_topic}'
        )))

    actions.append(Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            LaunchConfiguration('projection_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('cloud_in', input_topic),
            ('scan', LaunchConfiguration('scan_topic')),
        ],
    ))
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_perception'))
    default_config = package_share / 'config' / 'pointcloud_to_laserscan.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_self_filter', default_value='false'),
        DeclareLaunchArgument(
            'raw_cloud_topic', default_value='/lidar/points_raw'),
        DeclareLaunchArgument(
            'filtered_cloud_topic', default_value='/lidar/points_scan'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'projection_config', default_value=str(default_config)),
        OpaqueFunction(function=_launch_setup),
    ])
