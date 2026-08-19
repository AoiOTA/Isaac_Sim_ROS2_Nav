from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def _launch_setup(context):
    backend = LaunchConfiguration(
        'localization_backend').perform(context).strip().lower()
    if backend not in {'ideal', 'amcl'}:
        raise RuntimeError('localization_backend must be ideal or amcl')
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

    if backend == 'amcl':
        params_file = Path(
            LaunchConfiguration('amcl_params_file').perform(context).strip()
        ).expanduser()
        if not params_file.is_file():
            raise RuntimeError(f'AMCL params file does not exist: {params_file}')
        actions.extend([
            LogInfo(msg=(
                'Estimated localization: AMCL is the sole map->odom owner')),
            LifecycleNode(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace='',
                output='screen',
                sigterm_timeout='15.0',
                parameters=[str(params_file), {'use_sim_time': use_sim_time}],
                remappings=[('scan', '/scan'), ('map', '/map')],
            ),
        ])
        node_names.append('amcl')
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
    package_share = Path(get_package_share_directory('robot_mapping'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'localization_backend', default_value='amcl',
            description='ideal evaluator baseline or formal amcl backend'),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=str(package_share / 'config' / 'amcl_kujiale.yaml')),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_yaw_deg', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
