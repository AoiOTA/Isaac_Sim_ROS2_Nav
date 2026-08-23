from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode


def _launch_setup(context):
    backend = LaunchConfiguration(
        'localization_backend').perform(context).strip().lower()
    if backend not in {'ideal', 'grid'}:
        raise RuntimeError(
            'localization_backend must be grid or ideal')
    map_file = LaunchConfiguration('map_file').perform(context).strip()
    if not map_file:
        raise RuntimeError(
            'map_file is required for immutable navigation map serving')
    if not Path(map_file).is_file():
        raise RuntimeError(f'occupancy map YAML does not exist: {map_file}')

    use_sim_time = LaunchConfiguration('use_sim_time')
    node_names = ['map_server']
    actions = [LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        sigterm_timeout='15.0',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        }],
    )]

    if backend == 'grid':
        actions.extend([
            LogInfo(msg=(
                'V6-GRID localization: GridLocalizationTFManager is the '
                'sole map->odom owner')),
            ComposableNodeContainer(
                package='rclcpp_components',
                executable='component_container_mt',
                name='grid_localization_container',
                namespace='',
                output='screen',
                composable_node_descriptions=[
                    ComposableNode(
                        package='isaac_ros_pointcloud_utils',
                        plugin=(
                            'nvidia::isaac_ros::pointcloud_utils::'
                            'LaserScantoFlatScanNode'),
                        name='laserscan_to_flatscan',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'input_qos': 'SENSOR_DATA',
                        }],
                        remappings=[
                            ('scan', '/scan'),
                            ('flatscan', '/flatscan'),
                        ],
                    ),
                    ComposableNode(
                        package='isaac_ros_occupancy_grid_localizer',
                        plugin=(
                            'nvidia::isaac_ros::occupancy_grid_localizer::'
                            'OccupancyGridLocalizerNode'),
                        name='occupancy_grid_localizer',
                        parameters=[map_file, {
                            'use_sim_time': use_sim_time,
                            'loc_result_frame': 'map',
                            'map_yaml_path': map_file,
                        }],
                        remappings=[
                            ('flatscan', '/flatscan'),
                            ('localization_result', '/localization_result'),
                            ('trigger_grid_search_localization',
                             '/trigger_grid_search_localization'),
                        ],
                    ),
                ],
            ),
            Node(
                package='robot_grid_localization',
                executable='grid_localization_tf_manager',
                name='grid_localization_tf_manager',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ])
    else:
        actions.extend([
            LogInfo(msg=(
                'Ideal evaluator baseline: publishing calibrated map->odom')),
            Node(
                package='robot_bringup',
                executable='ideal_localization_tf',
                name='ideal_localization_tf',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
                    'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
                    'map_to_odom_yaw_deg': LaunchConfiguration(
                        'map_to_odom_yaw_deg'),
                }],
            ),
        ])

    actions.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        sigterm_timeout='15.0',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': LaunchConfiguration('autostart')},
            {'node_names': node_names},
            {'bond_timeout': 10.0},
        ],
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'localization_backend', default_value='grid',
            description='V6-GRID production backend or ideal evaluator'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_yaw_deg', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
