from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _posegraph_prefix(value):
    for suffix in ('.posegraph', '.data'):
        if value.endswith(suffix):
            return value[:-len(suffix)]
    return value


def _bool_argument(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in {'true', 'false'}:
        raise RuntimeError(f'{name} must be true or false')
    return value == 'true'


def _launch_setup(context):
    prefix = _posegraph_prefix(
        LaunchConfiguration('posegraph_file').perform(context).strip())
    if prefix:
        missing = [
            prefix + suffix
            for suffix in ('.posegraph', '.data')
            if not Path(prefix + suffix).is_file()
        ]
        if missing:
            raise RuntimeError(
                'incremental mapping pose graph is incomplete; missing: '
                + ', '.join(missing))
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_scan_matching = _bool_argument(context, 'use_scan_matching')
    do_loop_closing = _bool_argument(context, 'do_loop_closing')

    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('mapping_params_file'),
            {
                'use_sim_time': use_sim_time,
                'use_lifecycle_manager': False,
                'mode': 'mapping',
                'map_file_name': prefix,
                'use_scan_matching': use_scan_matching,
                'do_loop_closing': do_loop_closing,
            },
        ],
        remappings=[('scan', '/scan')],
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='Activating SLAM Toolbox mapping mode'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(autostart),
    )

    # The inactive->active handler must exist before CONFIGURE is emitted; the
    # lifecycle transition can finish within the same launch event cycle.
    return [slam_node, activate, configure]


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_mapping'))
    default_config = package_share / 'config' / 'slam_mapping.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'use_scan_matching',
            default_value='true',
            description=(
                'Correct odometry with scan matching. Ideal odometry bringup '
                'overrides this to false because its pose is ground truth.'),
        ),
        DeclareLaunchArgument(
            'do_loop_closing',
            default_value='true',
            description=(
                'Enable loop closure. Ideal odometry bringup overrides this '
                'to false to keep map->odom anchored to ground truth.'),
        ),
        DeclareLaunchArgument(
            'mapping_params_file', default_value=str(default_config)),
        DeclareLaunchArgument(
            'posegraph_file',
            default_value='',
            description=(
                'Optional serialized pose graph prefix for incremental '
                'mapping; both .posegraph and .data files must exist'),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
