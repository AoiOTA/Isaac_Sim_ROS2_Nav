from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _launch_setup(context):
    use_posegraph_localization = (
        LaunchConfiguration('use_posegraph_localization')
        .perform(context)
        .strip()
        .lower()
    )
    if use_posegraph_localization not in {'true', 'false'}:
        raise RuntimeError(
            'use_posegraph_localization must be true or false')
    use_posegraph_localization = use_posegraph_localization == 'true'

    prefix = _posegraph_prefix(
        LaunchConfiguration('posegraph_file').perform(context).strip())
    if use_posegraph_localization:
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
    actions = [map_node, activate_map, configure_map]
    if use_posegraph_localization:
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
        actions.extend([slam_node, activate_slam, configure_slam])
    else:
        actions.extend([
            LogInfo(msg=(
                'Ideal odometry localization: publishing fresh identity '
                'map->odom transform')),
            Node(
                package='robot_bringup',
                executable='ideal_localization_tf',
                name='ideal_localization_tf',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            'use_posegraph_localization',
            default_value='true',
            description=(
                'Use SLAM Toolbox for map->odom; false publishes a fresh '
                'identity transform for calibrated ideal odometry'),
        ),
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
