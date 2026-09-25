from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _posegraph_prefix(value):
    for suffix in ('.posegraph', '.data'):
        if value.endswith(suffix):
            return value[:-len(suffix)]
    return value


def _validate_amcl_contract(path_value):
    path = Path(path_value).expanduser()
    if not path.is_file():
        raise RuntimeError(f'amcl_params_file does not exist: {path}')
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    parameters = document.get('amcl', {}).get('ros__parameters', {})
    expected = {
        'base_frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'global_frame_id': 'map',
        'scan_topic': '/scan',
        'tf_broadcast': True,
    }
    for name, value in expected.items():
        if parameters.get(name) != value:
            raise RuntimeError(
                f'amcl_params_file requires {name}={value!r}')


def _launch_setup(context):
    localization_backend = LaunchConfiguration(
        'localization_backend').perform(context).strip().lower()
    if localization_backend not in {'ideal', 'slam_toolbox', 'amcl'}:
        raise RuntimeError(
            'localization_backend must be ideal, slam_toolbox, or amcl')

    prefix = _posegraph_prefix(
        LaunchConfiguration('posegraph_file').perform(context).strip())
    if localization_backend == 'slam_toolbox':
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
    try:
        ceres_num_threads = int(
            LaunchConfiguration('ceres_num_threads').perform(context))
    except ValueError as exc:
        raise RuntimeError('ceres_num_threads must be an integer') from exc
    if ceres_num_threads < 1:
        raise RuntimeError('ceres_num_threads must be positive')
    if localization_backend == 'amcl':
        _validate_amcl_contract(
            LaunchConfiguration('amcl_params_file').perform(context))
        raise RuntimeError(
            'localization_backend=amcl is a validated P0 parameter contract; '
            'AMCL process bringup is scheduled for P3/P4')

    autostart = LaunchConfiguration('autostart')
    map_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        sigterm_timeout='15.0',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': map_file,
        }],
    )
    configure_map = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )
    activate_map = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='Activating immutable occupancy map server'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(map_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(autostart),
    )
    # Install the inactive-state handler before starting the lifecycle
    # process.  A 1600x1600 map normally takes long enough to hide this race,
    # but a loaded simulator can schedule CONFIGURE completion before a later
    # RegisterEventHandler action executes, leaving map_server inactive and
    # Nav2 on its 100x100 default costmap.
    actions = [activate_map, map_node, configure_map]
    if localization_backend == 'slam_toolbox':
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
                    'use_lifecycle_manager': False,
                    'mode': 'localization',
                    'map_file_name': prefix,
                    'ceres_num_threads': ceres_num_threads,
                },
            ],
            # SLAM Toolbox owns localization TF only.  Its scan-rasterized map
            # is diagnostic and must not replace the immutable saved map.
            remappings=[
                ('scan', '/scan'),
                ('map', '/slam_toolbox/map'),
            ],
        )
        configure_slam = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=IfCondition(autostart),
        )
        activate_slam = RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=slam_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    LogInfo(msg='Activating SLAM Toolbox localization mode'),
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            ),
            condition=IfCondition(autostart),
        )
        # Register the transition handler before emitting CONFIGURE.
        actions.extend([activate_slam, slam_node, configure_slam])
    else:
        actions.extend([
            LogInfo(msg=(
                'Ideal odometry localization: publishing fresh map->odom '
                'transform aligned to the selected spawn')),
            Node(
                package='robot_bringup',
                executable='ideal_localization_tf',
                name='ideal_localization_tf',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
                    'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
                    'map_to_odom_yaw_deg': LaunchConfiguration(
                        'map_to_odom_yaw_deg'),
                }],
            ),
        ])
    return actions


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_mapping'))
    default_config = package_share / 'config' / 'slam_localization.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('ceres_num_threads', default_value='12'),
        DeclareLaunchArgument(
            'localization_params_file', default_value=str(default_config)),
        DeclareLaunchArgument(
            'localization_backend',
            default_value='ideal',
            description='ideal, slam_toolbox, or amcl (P0 contract only)'),
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
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value=str(
                package_share / 'config' / 'amcl_p0_contract.yaml'),
        ),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.0'),
        DeclareLaunchArgument('map_to_odom_yaw_deg', default_value='0.0'),
        OpaqueFunction(function=_launch_setup),
    ])
