from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from robot_bringup.interactive_policy import resolve_interactive_selection
from robot_bringup.interactive_policy import teleop_terminal_command
from robot_bringup.mode_contract import validate_mode
from robot_bringup.mode_contract import validate_nav2_profile
from robot_bringup.mode_contract import validate_nav2_profile_params_file
from robot_bringup.mode_contract import validate_robot_runtime_files
from robot_experiments.spawn_poses import load_spawn_pose


_TELEOP_SPEED_ARGUMENTS = (
    ('teleop_linear_speed', 'linear_speed'),
    ('teleop_angular_speed', 'angular_speed'),
    ('teleop_linear_speed_step', 'linear_speed_step'),
    ('teleop_angular_speed_step', 'angular_speed_step'),
    ('teleop_min_linear_speed', 'min_linear_speed'),
    ('teleop_min_angular_speed', 'min_angular_speed'),
    ('teleop_max_linear_speed', 'max_linear_speed'),
    ('teleop_max_angular_speed', 'max_angular_speed'),
)


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
    initial_pose_source = LaunchConfiguration(
        'initial_pose_source').perform(context).strip().lower()
    if initial_pose_source not in {'auto', 'rviz'}:
        raise RuntimeError('initial_pose_source must be auto or rviz')
    project_root_value = LaunchConfiguration(
        'project_root').perform(context).strip()
    spawn_poses_file = LaunchConfiguration(
        'spawn_poses_file').perform(context).strip()
    selection = validate_mode(
        operation=LaunchConfiguration('operation').perform(context),
        odometry_mode=LaunchConfiguration('odometry_mode').perform(context),
        structure_tf_source=LaunchConfiguration(
            'structure_tf_source').perform(context),
        posegraph_file=LaunchConfiguration('posegraph_file').perform(context),
        map_file=LaunchConfiguration('map_file').perform(context),
        check_posegraph_files=True,
        map_manifest_file=LaunchConfiguration(
            'map_manifest_file').perform(context),
        project_root=project_root_value,
        initial_pose_source=initial_pose_source,
        spawn_poses_file=spawn_poses_file,
        spawn_pose_name=LaunchConfiguration(
            'spawn_pose_name').perform(context),
        localization_backend=LaunchConfiguration(
            'localization_backend').perform(context),
        route_graph_file=LaunchConfiguration(
            'route_graph_file').perform(context),
    )
    selected_spawn = None
    if (selection.operation in {'localization', 'navigation'}
            and selection.odometry_mode == 'ideal'
            and initial_pose_source == 'auto'):
        selected_spawn = load_spawn_pose(
            spawn_poses_file,
            LaunchConfiguration('spawn_pose_name').perform(context),
            require_calibrated=True,
        )
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    posegraph_calibration_value = LaunchConfiguration(
        'posegraph_calibration').perform(context).strip().lower()
    if posegraph_calibration_value not in {'true', 'false'}:
        raise RuntimeError('posegraph_calibration must be true or false')
    posegraph_calibration = posegraph_calibration_value == 'true'
    if posegraph_calibration and not (
            selection.operation == 'localization'
            and selection.odometry_mode == 'ideal'):
        raise RuntimeError(
            'posegraph_calibration is only valid for Ideal localization')
    # The calibration diagnostic swaps the ideal map->odom publisher for SLAM
    # Toolbox pose-graph localization while keeping ideal odometry.
    localization_backend = (
        'slam_toolbox'
        if posegraph_calibration
        else selection.localization_backend)
    use_self_filter = LaunchConfiguration('use_self_filter').perform(context)
    if (selection.operation == 'incremental_mapping'
            and initial_pose_source != 'auto'):
        raise RuntimeError(
            'incremental_mapping requires initial_pose_source=auto')
    description_share = Path(
        get_package_share_directory('robot_description'))
    try:
        interactive = resolve_interactive_selection(
            operation=selection.operation,
            interactive=LaunchConfiguration('interactive').perform(context),
            use_rviz=LaunchConfiguration('use_rviz').perform(context),
            rviz_config=LaunchConfiguration('rviz_config').perform(context),
            use_teleop=LaunchConfiguration('use_teleop').perform(context),
            robot_description_share=description_share,
        )
    except ValueError as exc:
        raise RuntimeError(str(exc)) from exc
    odometry_share = Path(get_package_share_directory('robot_odometry'))
    navigation_share = Path(get_package_share_directory('robot_navigation'))
    nav2_profile = validate_nav2_profile(
        LaunchConfiguration('nav2_profile').perform(context))
    requested_nav2_overlay = LaunchConfiguration(
        'nav2_profile_params_file').perform(context).strip()
    nav2_profile_params_file = Path(requested_nav2_overlay).expanduser() \
        if requested_nav2_overlay else (
            navigation_share / 'config' / f'nav2_{nav2_profile}.yaml')
    try:
        nav2_controller_profile = validate_nav2_profile_params_file(
            nav2_profile_params_file)
    except ValueError as exc:
        raise RuntimeError(
            f'invalid nav2_profile_params_file: {exc}') from exc
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
        f'localization={localization_backend}, '
        f'structure_tf={selection.structure_tf_source}, '
        f'rviz={interactive.use_rviz}, teleop={interactive.use_teleop}, '
        f'nav2_profile={nav2_profile}, '
        f'controller_frequency='
        f'{nav2_controller_profile.controller_frequency:g}Hz, '
        f'model_dt={nav2_controller_profile.model_dt:g}s, '
        f'map_version={selection.map_version or "none"}, '
        f'map_bundle={selection.map_bundle_sha256 or "none"}'
    ))]

    actions.append(Node(
        package='robot_bringup',
        executable='initial_pose_policy',
        name='initial_pose_policy',
        output='screen',
        parameters=[{
            'initial_pose_source': initial_pose_source,
        }],
    ))

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

    perception = _include(
        'robot_perception',
        'lidar_processing.launch.py',
        {
            'use_sim_time': use_sim_time,
            'use_self_filter': use_self_filter,
            # Mapping/localization retain the frozen /scan contract. Only
            # navigation starts the parallel self-filtered near-field stream
            # consumed by the local safety chain.
            'enable_safety_scan': (
                'true' if selection.operation == 'navigation' else 'false'),
        },
    )
    if interactive.use_rviz:
        # RViz display constructors briefly use their default Reliable sensor
        # QoS before the saved Best Effort properties are applied. Starting
        # the Best Effort /scan publisher only after the config is loaded
        # avoids a misleading one-shot incompatibility warning while keeping
        # the final endpoint QoS unchanged.
        actions.append(TimerAction(period=1.5, actions=[perception]))
    else:
        actions.append(perception)

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
        # Ideal odometry normally keeps scan matching off (ground-truth
        # poses).  ideal_mapping_scan_matching:=true keeps it on so the
        # serialized pose graph contains graph vertices/edges - the
        # vertex-less archive from the default Ideal profile crashes
        # localization_slam_toolbox_node on its first scan.
        ideal_scan_matching = (
            LaunchConfiguration('ideal_mapping_scan_matching')
            .perform(context).strip().lower() in {'true', '1', 'yes', 'on'})
        # ideal_mapping_do_loop_closing:=true additionally enables loop
        # closure for Ideal mapping (off by default: ground-truth poses
        # need no global correction and symmetric aisles can mis-snap).
        ideal_loop_closing = (
            LaunchConfiguration('ideal_mapping_do_loop_closing')
            .perform(context).strip().lower() in {'true', '1', 'yes', 'on'})
        actions.append(_include(
            'robot_mapping',
            'mapping.launch.py',
            {
                'use_sim_time': use_sim_time,
                'posegraph_file': selection.posegraph_prefix,
                # IsaacComputeOdometry is ground truth in Ideal mode.  Scan
                # matching in visually repetitive rooms can otherwise pull
                # map->odom away from that exact pose during curved motion.
                'use_scan_matching': (
                    'false'
                    if selection.odometry_mode == 'ideal'
                    and not ideal_scan_matching
                    else 'true'
                ),
                'do_loop_closing': (
                    'false'
                    if selection.odometry_mode == 'ideal'
                    and not ideal_loop_closing
                    else 'true'
                ),
            },
        ))
        if (selection.operation == 'incremental_mapping'
                and initial_pose_source == 'auto'):
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
        localization_arguments = {
            'use_sim_time': use_sim_time,
            'posegraph_file': selection.posegraph_prefix,
            'map_file': selection.occupancy_map_file,
            # The backend selects the map->odom owner; with AMCL the
            # posegraph stays empty and ideal_localization_tf is not
            # started.
            'localization_backend': localization_backend,
            'map_to_odom_x': (
                str(selected_spawn.map.position[0])
                if selected_spawn is not None else '0.0'
            ),
            'map_to_odom_y': (
                str(selected_spawn.map.position[1])
                if selected_spawn is not None else '0.0'
            ),
            'map_to_odom_yaw_deg': (
                str(selected_spawn.map.yaw_deg)
                if selected_spawn is not None else '0.0'
            ),
        }
        # Empty keeps localization.launch.py's own amcl.yaml default; the
        # estimated isaac_compute entrypoint passes amcl_isaac_odom.yaml.
        amcl_params_file = LaunchConfiguration(
            'amcl_params_file').perform(context).strip()
        if amcl_params_file:
            localization_arguments['amcl_params_file'] = amcl_params_file
        actions.append(_include(
            'robot_mapping',
            'localization.launch.py',
            localization_arguments,
        ))
        if initial_pose_source == 'auto':
            actions.append(_include(
                'robot_experiments',
                'initial_pose.launch.py',
                {
                    'spawn_poses_file': LaunchConfiguration(
                        'spawn_poses_file').perform(context),
                    'spawn_pose_name': LaunchConfiguration(
                        'spawn_pose_name').perform(context),
                    'wait_for_odom_to_base_tf': 'true',
                    'stay_alive_for_reseed': 'true',
                },
            ))

    if selection.operation == 'navigation':
        navigation_arguments = {
            'use_sim_time': use_sim_time,
            'autostart': 'false',
            'nav2_params_file': runtime_files.nav2_params_file,
            'nav2_profile_params_file': str(nav2_profile_params_file),
            'structural_map_file': selection.occupancy_map_file,
            # The backend selects the A21 overlay economics and the
            # route-cost clearance margins inside navigation.launch.py.
            'localization_backend': localization_backend,
            'module2_enabled': LaunchConfiguration(
                'module2_enabled').perform(context),
            'route_graph_file': LaunchConfiguration(
                'route_graph_file').perform(context),
            'feasible_only_largest_component': LaunchConfiguration(
                'feasible_only_largest_component').perform(context),
            'module2_response_timeout_s': LaunchConfiguration(
                'module2_response_timeout_s').perform(context),
            'voxel_grid_topic': (
                'stvl_voxel_grid'
                if nav2_profile in {
                    'dynamic_avoidance', 'estimated_dynamic',
                    'bio_nav_planning_only',
                    'bio_nav_risk_only', 'bio_nav_tiebreak_risk',
                    'bio_nav_rgbd_risk_shadow',
                    'bio_nav_rgbd_risk_ab',
                    'bio_nav_rgbd_risk_static_opt_in'}
                else 'voxel_grid'
            ),
        }
        # Empty keeps the navigation.launch.py default (the engineering
        # defaults envelope); a non-empty value binds the last-precedence
        # controller velocity envelope (e.g. the estimated-chain seam cap).
        controller_max_velocity = LaunchConfiguration(
            'controller_max_linear_velocity_mps').perform(context).strip()
        if controller_max_velocity:
            navigation_arguments['controller_max_linear_velocity_mps'] = (
                controller_max_velocity)
        actions.append(_include(
            'robot_navigation',
            'navigation.launch.py',
            navigation_arguments,
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
                {
                    'use_sim_time': use_sim_time,
                    'initial_pose_source': initial_pose_source,
                    'localization_backend': localization_backend,
                },
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

    if interactive.use_rviz or interactive.use_teleop:
        if not project_root_value:
            raise RuntimeError(
                'PROJECT_ROOT is required for managed RViz/Teleop; use '
                'scripts/run_ros.sh or pass project_root:=<repository>')
        project_root = Path(project_root_value).expanduser().resolve()
        if interactive.use_rviz:
            run_rviz = project_root / 'scripts' / 'run_rviz.sh'
            if not run_rviz.is_file():
                raise RuntimeError(f'RViz launcher not found: {run_rviz}')
            actions.insert(1, ExecuteProcess(
                cmd=[
                    str(run_rviz),
                    selection.operation,
                    interactive.rviz_config,
                ],
                # run_ros.sh is itself a process-group supervisor.  Managed
                # RViz must create its own group instead of inheriting the
                # supervisor's recursion guard through ros2 launch.
                additional_env={
                    'ISAAC_NAV_DEDICATED_PROCESS_GROUP': '0',
                },
                output='screen',
            ))
        if interactive.use_teleop:
            run_teleop = project_root / 'scripts' / 'run_teleop.sh'
            terminal_wrapper = (
                project_root / 'scripts' / 'run_teleop_terminal.sh')
            if not run_teleop.is_file():
                raise RuntimeError(
                    f'Mapping teleop launcher not found: {run_teleop}')
            if not terminal_wrapper.is_file():
                raise RuntimeError(
                    f'Teleop terminal wrapper not found: {terminal_wrapper}')
            teleop_arguments = []
            for launch_name, parameter_name in _TELEOP_SPEED_ARGUMENTS:
                value = LaunchConfiguration(launch_name).perform(
                    context).strip()
                if not value:
                    raise RuntimeError(f'{launch_name} must not be empty')
                teleop_arguments.append(f'{parameter_name}:={value}')
            try:
                terminal_command = teleop_terminal_command(
                    run_teleop,
                    arguments=teleop_arguments,
                )
            except ValueError as exc:
                raise RuntimeError(str(exc)) from exc
            actions.extend([
                ExecuteProcess(
                    cmd=[str(terminal_wrapper), *terminal_command],
                    output='screen',
                ),
                LogInfo(msg=(
                    'Mapping Teleop is running in a separate terminal.\n'
                    'Click the window titled "Isaac Nav Mapping Teleop"\n'
                    'before pressing W/A/S/D or the arrow keys.'
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
            'localization_backend',
            default_value='',
            description=(
                'ideal, amcl, or slam_toolbox; empty derives from '
                'odometry_mode (ideal->ideal, realistic->slam_toolbox)')),
        DeclareLaunchArgument(
            'structure_tf_source',
            default_value='isaac',
            description='isaac or rsp'),
        DeclareLaunchArgument('posegraph_file', default_value=''),
        DeclareLaunchArgument('ceres_num_threads', default_value='12'),
        DeclareLaunchArgument('map_file', default_value=''),
        DeclareLaunchArgument(
            'ideal_mapping_scan_matching',
            default_value='false',
            description=(
                'Keep scan matching ON for Ideal-odometry mapping. The '
                'default Ideal profile records a vertex-less scan archive '
                'that cannot serve SLAM Toolbox localization; true writes '
                'graph vertices/edges while keeping Ideal ground-truth '
                'poses and no loop closing.')),
        DeclareLaunchArgument(
            'ideal_mapping_do_loop_closing',
            default_value='false',
            description=(
                'Enable loop closure for Ideal-odometry mapping. Off by '
                'default: Ideal ground-truth poses need no global '
                'correction and symmetric aisles can mis-snap.')),
        # Keep the manifest explicit at the core-launch boundary so direct
        # users get the same map-integrity validation as the wrapper launches.
        DeclareLaunchArgument(
            'map_manifest_file',
            default_value='',
            description='optional map-bundle manifest used to validate map and posegraph inputs'),
        DeclareLaunchArgument(
            'posegraph_calibration',
            default_value='false',
            description=(
                'Use SLAM Toolbox Pose Graph localization in Ideal '
                'localization solely to measure Map Pose calibration')),
        DeclareLaunchArgument('robot_description_file', default_value=''),
        DeclareLaunchArgument(
            'wheel_odometry_params_file', default_value=''),
        DeclareLaunchArgument(
            'amcl_params_file',
            default_value='',
            description=(
                'optional AMCL params YAML override; empty uses the '
                'robot_mapping localization.launch.py default (amcl.yaml)')),
        DeclareLaunchArgument(
            'controller_max_linear_velocity_mps',
            default_value='',
            description=(
                'optional last-precedence controller velocity envelope; '
                'empty uses the engineering-defaults value inside '
                'navigation.launch.py')),
        DeclareLaunchArgument('nav2_params_file', default_value=''),
        DeclareLaunchArgument(
            'nav2_profile',
            default_value='stable',
            description=(
                'stable, performance, dynamic_avoidance, estimated_static, '
                'estimated_dynamic, or optional BioNav '
                'planning-only, risk-only, combined, RGB-D risk Shadow, '
                'controlled RGB-D static A/B, or explicit static opt-in overlay')),
        DeclareLaunchArgument(
            'nav2_profile_params_file',
            default_value='',
            description='explicit benchmark/custom Nav2 overlay YAML'),
        DeclareLaunchArgument('module2_enabled', default_value='true'),
        DeclareLaunchArgument('route_graph_file', default_value=''),
        DeclareLaunchArgument(
            'feasible_only_largest_component', default_value='false'),
        DeclareLaunchArgument(
            'module2_response_timeout_s', default_value='0.0'),
        DeclareLaunchArgument(
            'spawn_poses_file',
            default_value=EnvironmentVariable(
                'ISAAC_NAV_SPAWN_POSES', default_value=''),
            description=(
                'Calibrated spawn pose YAML; defaults to '
                'ISAAC_NAV_SPAWN_POSES')),
        DeclareLaunchArgument(
            'spawn_pose_name', default_value='mapping_start'),
        DeclareLaunchArgument(
            'initial_pose_source',
            default_value='auto',
            description='auto or rviz (localization/navigation only)'),
        DeclareLaunchArgument(
            'interactive',
            default_value='true',
            description='false disables RViz and keyboard Teleop'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='launch the operation-specific RViz workflow'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='auto',
            description='auto or a custom .rviz path'),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='auto',
            description='auto, true, or false; only Mapping may enable it'),
        DeclareLaunchArgument('teleop_linear_speed', default_value='0.50'),
        DeclareLaunchArgument('teleop_angular_speed', default_value='0.80'),
        DeclareLaunchArgument(
            'teleop_linear_speed_step', default_value='0.05'),
        DeclareLaunchArgument(
            'teleop_angular_speed_step', default_value='0.10'),
        DeclareLaunchArgument(
            'teleop_min_linear_speed', default_value='0.10'),
        DeclareLaunchArgument(
            'teleop_min_angular_speed', default_value='0.20'),
        DeclareLaunchArgument(
            'teleop_max_linear_speed', default_value='1.00'),
        DeclareLaunchArgument(
            'teleop_max_angular_speed', default_value='1.50'),
        DeclareLaunchArgument(
            'project_root',
            default_value=EnvironmentVariable(
                'PROJECT_ROOT', default_value=''),
            description='repository root used by managed interaction scripts'),
        DeclareLaunchArgument('use_self_filter', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        OpaqueFunction(function=_launch_setup),
    ])
