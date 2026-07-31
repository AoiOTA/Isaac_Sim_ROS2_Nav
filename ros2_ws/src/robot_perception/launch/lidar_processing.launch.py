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
    enable_safety_scan = _as_bool(
        LaunchConfiguration('enable_safety_scan').perform(context),
        'enable_safety_scan',
    )
    raw_cloud_topic = LaunchConfiguration('raw_cloud_topic').perform(context)
    filtered_cloud_topic = LaunchConfiguration(
        'filtered_cloud_topic').perform(context)
    legacy_input_topic = (
        filtered_cloud_topic
        if use_self_filter
        else raw_cloud_topic
    )

    actions = []
    if use_self_filter or enable_safety_scan:
        actions.append(Node(
            package='robot_perception',
            executable='lidar_self_filter',
            name='lidar_self_filter',
            output='screen',
            parameters=[
                LaunchConfiguration('self_filter_config'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'input_topic': raw_cloud_topic,
                    'output_topic': filtered_cloud_topic,
                },
            ],
        ))
        actions.append(LogInfo(msg=(
            'LiDAR self filter enabled: '
            f'{raw_cloud_topic} -> {filtered_cloud_topic}'
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
            ('cloud_in', legacy_input_topic),
            ('scan', LaunchConfiguration('scan_topic')),
        ],
    ))
    if enable_safety_scan:
        actions.append(Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_safety',
            output='screen',
            parameters=[
                LaunchConfiguration('safety_projection_config'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('cloud_in', filtered_cloud_topic),
                ('scan', LaunchConfiguration('safety_scan_topic')),
            ],
        ))
        actions.append(LogInfo(msg=(
            'Independent near-field safety scan enabled: '
            f'{filtered_cloud_topic} -> '
            f'{LaunchConfiguration("safety_scan_topic").perform(context)}'
        )))
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_perception'))
    default_config = package_share / 'config' / 'pointcloud_to_laserscan.yaml'
    safety_config = (
        package_share / 'config' / 'pointcloud_to_laserscan_safety.yaml')
    self_filter_config = (
        package_share / 'config' / 'self_filter_optional.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_self_filter', default_value='false'),
        DeclareLaunchArgument('enable_safety_scan', default_value='false'),
        DeclareLaunchArgument(
            'raw_cloud_topic', default_value='/lidar/points_raw'),
        DeclareLaunchArgument(
            'filtered_cloud_topic', default_value='/lidar/points_scan'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'safety_scan_topic', default_value='/scan_safety'),
        DeclareLaunchArgument(
            'projection_config', default_value=str(default_config)),
        DeclareLaunchArgument(
            'safety_projection_config', default_value=str(safety_config)),
        DeclareLaunchArgument(
            'self_filter_config', default_value=str(self_filter_config)),
        OpaqueFunction(function=_launch_setup),
    ])
