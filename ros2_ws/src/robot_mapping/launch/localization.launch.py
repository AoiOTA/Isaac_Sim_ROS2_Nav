from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def _posegraph_prefix(value):
    for suffix in ('.posegraph', '.data'):
        if value.endswith(suffix):
            return value[:-len(suffix)]
    return value


def _launch_setup(context):
    prefix = _posegraph_prefix(
        LaunchConfiguration('posegraph_file').perform(context).strip())
    if not prefix:
        raise RuntimeError(
            'posegraph_file is required for SLAM Toolbox localization')
    missing = [
        prefix + suffix
        for suffix in ('.posegraph', '.data')
        if not Path(prefix + suffix).is_file()
    ]
    if missing:
        raise RuntimeError(
            'SLAM Toolbox pose graph is incomplete; missing: '
            + ', '.join(missing))
    map_file = LaunchConfiguration('map_file').perform(context).strip()
    if not map_file:
        raise RuntimeError(
            'map_file is required for immutable navigation map serving')
    if not Path(map_file).is_file():
        raise RuntimeError(f'occupancy map YAML does not exist: {map_file}')

    autostart = LaunchConfiguration('autostart')
    map_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': map_file,
        }],
    )
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('localization_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_lifecycle_manager': True,
                'mode': 'localization',
                'map_file_name': prefix,
            },
        ],
        # SLAM Toolbox owns localization TF only.  Its scan-rasterized map is
        # diagnostic and must not replace the immutable saved map consumed by
        # Nav2, otherwise moving objects become static-map ghosts.
        remappings=[
            ('scan', '/scan'),
            ('map', '/slam_toolbox/map'),
        ],
    )
    localization_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': autostart,
            'node_names': ['map_server', 'slam_toolbox'],
        }],
    )
    # One manager owns both localization lifecycle nodes. This is also the
    # canonical name queried by the Navigation 2 RViz panel.
    return [map_node, slam_node, localization_manager]


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_mapping'))
    default_config = package_share / 'config' / 'slam_localization.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'localization_params_file', default_value=str(default_config)),
        DeclareLaunchArgument(
            'posegraph_file',
            default_value='',
            description=(
                'Serialized SLAM Toolbox prefix; both .posegraph and .data '
                'files must exist'),
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Saved OccupancyGrid YAML served on /map',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
