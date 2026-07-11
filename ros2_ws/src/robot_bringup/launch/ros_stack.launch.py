from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from robot_bringup.mode_contract import validate_mode
from robot_bringup.mode_contract import validate_robot_runtime_files


def _shutdown_if_gate_exited(context):
    """Stop the stack on gate failure without re-emitting global shutdown."""
    if context.is_shutdown:
        return []
    return [EmitEvent(event=Shutdown(
        reason='Nav2 activation gate exited'))]


def _include(package, launch_file, arguments):
    path = Path(get_package_share_directory(package)) / 'launch' / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(path)),
        launch_arguments=arguments.items(),
    )


def _launch_setup(context):
    selection = validate_mode(
        operation=LaunchConfiguration('operation').perform(context),
        odometry_mode=LaunchConfiguration('odometry_mode').perform(context),
        structure_tf_source=LaunchConfiguration(
            'structure_tf_source').perform(context),
        posegraph_file=LaunchConfiguration('posegraph_file').perform(context),
        map_file=LaunchConfiguration('map_file').perform(context),
        check_posegraph_files=True,
    )
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    use_self_filter = LaunchConfiguration('use_self_filter').perform(context)
    description_share = Path(
        get_package_share_directory('robot_description'))
    odometry_share = Path(get_package_share_directory('robot_odometry'))
    navigation_share = Path(get_package_share_directory('robot_navigation'))
    runtime_files = validate_robot_runtime_files(
        description_file=(
            LaunchConfiguration('robot_description_file').perform(context)
            or str(description_share / 'urdf' / 'jackal.urdf.xacro')
        ),
        wheel_odometry_params_file=(
            LaunchConfiguration(
                'wheel_odometry_params_file').perform(context)
            or str(odometry_share / 'config' / 'wheel_odometry.yaml')
        ),
        nav2_params_file=(
            LaunchConfiguration('nav2_params_file').perform(context)
            or str(navigation_share / 'config' / 'nav2_params.yaml')
        ),
    )
    actions = [LogInfo(msg=(
        'ROS stack mode: '
        f'operation={selection.operation}, '
        f'odometry={selection.odometry_mode}, '
        f'structure_tf={selection.structure_tf_source}'
    ))]

    actions.append(_include(
        'robot_description',
        'description.launch.py',
        {
            'use_sim_time': use_sim_time,
            'publish_tf': (
                'true'
                if selection.structure_tf_source == 'rsp'
                else 'false'
            ),
            'xacro_file': runtime_files.description_file,
        },
    ))

    actions.append(_include(
        'robot_perception',
        'lidar_processing.launch.py',
        {
            'use_sim_time': use_sim_time,
            'use_self_filter': use_self_filter,
        },
    ))

    if selection.odometry_mode == 'realistic':
        actions.extend([
            _include(
                'robot_odometry',
                'wheel_odometry.launch.py',
                {
                    'use_sim_time': use_sim_time,
                    'wheel_odometry_params_file': (
                        runtime_files.wheel_odometry_params_file),
                },
            ),
            _include(
                'robot_localization_config',
                'ekf.launch.py',
                {'use_sim_time': use_sim_time},
            ),
        ])

    if selection.operation in {'mapping', 'incremental_mapping'}:
        actions.append(_include(
            'robot_mapping',
            'mapping.launch.py',
            {
                'use_sim_time': use_sim_time,
                'posegraph_file': selection.posegraph_prefix,
            },
        ))
        if selection.operation == 'incremental_mapping':
            actions.append(_include(
                'robot_experiments',
                'initial_pose.launch.py',
                {
                    'spawn_poses_file': LaunchConfiguration(
                        'spawn_poses_file').perform(context),
                    'spawn_pose_name': LaunchConfiguration(
                        'spawn_pose_name').perform(context),
                    'wait_for_odom_to_base_tf': 'true',
                },
            ))
    else:
        actions.extend([
            _include(
                'robot_mapping',
                'localization.launch.py',
                {
                    'use_sim_time': use_sim_time,
                    'posegraph_file': selection.posegraph_prefix,
                    'map_file': selection.occupancy_map_file,
                },
            ),
            _include(
                'robot_experiments',
                'initial_pose.launch.py',
                {
                    'spawn_poses_file': LaunchConfiguration(
                        'spawn_poses_file').perform(context),
                    'spawn_pose_name': LaunchConfiguration(
                        'spawn_pose_name').perform(context),
                    'wait_for_odom_to_base_tf': 'true',
                },
            ),
        ])

    if selection.operation == 'navigation':
        actions.append(_include(
            'robot_navigation',
            'navigation.launch.py',
            {
                'use_sim_time': use_sim_time,
                'autostart': 'false',
                'nav2_params_file': runtime_files.nav2_params_file,
            },
        ))
        gate_config = (
            Path(get_package_share_directory('robot_bringup'))
            / 'config'
            / 'activation_gate.yaml'
        )
        activation_gate = Node(
            package='robot_bringup',
            executable='nav2_activation_gate',
            name='nav2_activation_gate',
            output='screen',
            parameters=[
                str(gate_config),
                {'use_sim_time': use_sim_time},
            ],
        )
        actions.extend([
            activation_gate,
            RegisterEventHandler(OnProcessExit(
                target_action=activation_gate,
                on_exit=[OpaqueFunction(
                    function=_shutdown_if_gate_exited)],
            )),
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'operation',
            default_value='mapping',
            description='mapping, localization, or navigation'),
        DeclareLaunchArgument(
            'odometry_mode',
            default_value='ideal',
            description='ideal or realistic'),
        DeclareLaunchArgument(
            'structure_tf_source',
            default_value='isaac',
            description='isaac or rsp'),
        DeclareLaunchArgument('posegraph_file', default_value=''),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        DeclareLaunchArgument(
            'wheel_odometry_params_file', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=''),
        DeclareLaunchArgument(
            'spawn_poses_file',
            default_value=EnvironmentVariable(
                'ISAAC_NAV_SPAWN_POSES', default_value=''),
            description=(
                'Calibrated spawn pose YAML; defaults to '
                'ISAAC_NAV_SPAWN_POSES')),
        DeclareLaunchArgument(
            'spawn_pose_name', default_value='mapping_start'),
        DeclareLaunchArgument('use_self_filter', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        OpaqueFunction(function=_launch_setup),
    ])
