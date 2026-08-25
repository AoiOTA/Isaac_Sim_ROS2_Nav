from pathlib import Path
import tempfile

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
from robot_bringup.mode_contract import cognitive_nav2_parameters
from robot_bringup.mode_contract import resolve_ekf_profile
from robot_bringup.mode_contract import validate_cognitive_graph_mode
from robot_bringup.mode_contract import validate_cognitive_profile
from robot_bringup.mode_contract import validate_mode
from robot_bringup.mode_contract import validate_nav2_profile
from robot_bringup.mode_contract import validate_nav2_profile_params_file
from robot_bringup.mode_contract import validate_robot_runtime_files
from robot_experiments.spawn_poses import load_spawn_pose
from robot_localization_config.ekf_input_policy import validate_lidar_gate
import yaml


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


def _write_cognitive_nav2_overlay(profile):
    """Write the exact-node overlay that must follow the A21 overlay."""
    document = cognitive_nav2_parameters(profile)
    with tempfile.NamedTemporaryFile(
            mode='w', prefix=f'v6_cognitive_{profile.name.lower()}_',
            suffix='.yaml', delete=False, encoding='utf-8') as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
        return Path(stream.name)


def _write_activation_gate_runtime_overlay(
        *, use_sim_time, initial_pose_source, startup_timeout,
        startup_timeout_policy):
    """Write runtime overrides under the gate's exact ROS node key."""
    normalized_use_sim_time = str(use_sim_time).strip().lower()
    if normalized_use_sim_time not in {'true', 'false'}:
        raise RuntimeError('use_sim_time must be true or false')
    try:
        normalized_startup_timeout = float(startup_timeout)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(
            'activation_startup_timeout must be a number') from exc
    document = {
        'nav2_activation_gate': {
            'ros__parameters': {
                'use_sim_time': normalized_use_sim_time == 'true',
                'initial_pose_source': initial_pose_source,
                'startup_timeout': normalized_startup_timeout,
                'startup_timeout_policy': startup_timeout_policy,
            },
        },
    }
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='nav2_activation_gate_runtime_',
            suffix='.yaml', delete=False, encoding='utf-8') as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
        return Path(stream.name)


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
        localization_map_contract=LaunchConfiguration(
            'localization_map_contract').perform(context),
        localization_owner=LaunchConfiguration(
            'localization_owner').perform(context),
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
    if posegraph_calibration:
        raise RuntimeError(
            'posegraph_calibration is retired from localization bringup; '
            'use mapping to rebuild maps and AMCL for estimated localization')
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
    mapping_share = Path(get_package_share_directory('robot_mapping'))
    navigation_share = Path(get_package_share_directory('robot_navigation'))
    requested_ekf_profile = LaunchConfiguration(
        'ekf_profile').perform(context).strip()
    lidar_odometry_backend = LaunchConfiguration(
        'lidar_odometry_backend').perform(context).strip().lower()
    lidar_validated_value = LaunchConfiguration(
        'lidar_odometry_validated').perform(context).strip().lower()
    if lidar_validated_value not in {'true', 'false'}:
        raise RuntimeError('lidar_odometry_validated must be true or false')
    localization_config_share = Path(
        get_package_share_directory('robot_localization_config'))
    requested_ekf_params = LaunchConfiguration(
        'ekf_params_file').perform(context).strip()
    try:
        ekf_profile = resolve_ekf_profile(
            selection.odometry_mode,
            requested_ekf_profile,
            lidar_odometry_backend,
            lidar_validated_value == 'true',
            requested_ekf_params,
        )
    except ValueError as exc:
        raise RuntimeError(f'invalid EKF profile contract: {exc}') from exc
    ekf_params_file = (
        Path(requested_ekf_params).expanduser()
        if requested_ekf_params
        else localization_config_share / 'config' / f'ekf_{ekf_profile}.yaml'
    )
    if not ekf_params_file.is_file():
        raise RuntimeError(f'EKF params file does not exist: {ekf_params_file}')
    try:
        ekf_uses_lidar = validate_lidar_gate(
            ekf_params_file, lidar_validated_value == 'true')
    except ValueError as exc:
        raise RuntimeError(f'invalid EKF input policy: {exc}') from exc
    if ekf_uses_lidar and lidar_odometry_backend != 'rf2o':
        raise RuntimeError(
            'loaded EKF params reference LiDAR odometry and require '
            'lidar_odometry_backend=rf2o')
    localization_profile = LaunchConfiguration(
        'localization_profile').perform(context).strip().lower()
    if localization_profile not in {'kujiale', 'rivermark'}:
        raise RuntimeError(
            'localization_profile must be kujiale or rivermark')
    requested_amcl_params = LaunchConfiguration(
        'amcl_params_file').perform(context).strip()
    amcl_params_file = (
        Path(requested_amcl_params).expanduser()
        if requested_amcl_params
        else mapping_share / 'config' / f'amcl_{localization_profile}.yaml'
    )
    nav2_profile = validate_nav2_profile(
        LaunchConfiguration('nav2_profile').perform(context))
    bringup_share = Path(get_package_share_directory('robot_bringup'))
    try:
        cognitive_profile = validate_cognitive_profile(
            LaunchConfiguration('cognitive_profile').perform(context),
            bringup_share / 'config' / 'modes.yaml',
        )
    except ValueError as exc:
        raise RuntimeError(f'invalid cognitive_profile: {exc}') from exc
    try:
        cognitive_graph_mode = validate_cognitive_graph_mode(
            LaunchConfiguration('cognitive_graph_mode').perform(context))
    except ValueError as exc:
        raise RuntimeError(f'invalid cognitive_graph_mode: {exc}') from exc
    cognitive_overlay_file = None
    if nav2_profile == 'v6_low_obstacle_isolation':
        cognitive_overlay_file = _write_cognitive_nav2_overlay(
            cognitive_profile)
    module2_enabled = (
        'true' if cognitive_profile.module2_enabled else 'false'
    ) if nav2_profile == 'v6_low_obstacle_isolation' else (
        LaunchConfiguration('module2_enabled').perform(context)
    )
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
        f'structure_tf={selection.structure_tf_source}, '
        f'localization_map_contract={selection.localization_map_contract}, '
        f'localization_owner={selection.localization_owner}, '
        f'rviz={interactive.use_rviz}, teleop={interactive.use_teleop}, '
        f'localization_collection='
        f'{interactive.localization_collection}, '
        f'nav2_profile={nav2_profile}, '
        f'cognitive_profile={cognitive_profile.name}, '
        f'module2_enabled={module2_enabled}, '
        f'cognitive_graph_mode={cognitive_graph_mode}, '
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

    if selection.odometry_mode in {'realistic', 'estimated', 'mixed'}:
        normalized_use_sim_time = str(use_sim_time).strip().lower()
        if normalized_use_sim_time not in {'true', 'false'}:
            raise RuntimeError('use_sim_time must be true or false')
        requested_imu_calibration = LaunchConfiguration(
            'imu_calibration_params_file').perform(context).strip()
        imu_calibration_params_file = (
            Path(requested_imu_calibration).expanduser()
            if requested_imu_calibration
            else odometry_share / 'config' / 'imu_calibration.yaml'
        )
        if not imu_calibration_params_file.is_file():
            raise RuntimeError(
                'IMU calibration params file does not exist: '
                f'{imu_calibration_params_file}')
        actions.append(Node(
            package='robot_odometry',
            executable='imu_yaw_calibrator',
            name='imu_yaw_calibrator',
            output='screen',
            parameters=[
                str(imu_calibration_params_file),
                {'use_sim_time': normalized_use_sim_time == 'true'},
            ],
        ))
        actions.append(_include(
            'robot_odometry',
            'wheel_odometry.launch.py',
            {
                'use_sim_time': use_sim_time,
                'wheel_odometry_params_file': (
                    runtime_files.wheel_odometry_params_file),
            },
        ))
        if selection.odometry_mode != 'mixed':
            actions.append(_include(
                'robot_odometry',
                'lidar_odometry.launch.py',
                {
                    'use_sim_time': use_sim_time,
                    'lidar_odometry_backend': lidar_odometry_backend,
                    'lidar_odometry_params_file': LaunchConfiguration(
                        'lidar_odometry_params_file').perform(context),
                },
            ))
        actions.append(_include(
            'robot_localization_config',
            'ekf.launch.py',
            {
                'use_sim_time': use_sim_time,
                'ekf_profile': ekf_profile,
                'lidar_odometry_validated': lidar_validated_value,
                'ekf_params_file': str(ekf_params_file),
            },
        ))

    if selection.operation in {'mapping', 'incremental_mapping'}:
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
                    else 'true'
                ),
                'do_loop_closing': (
                    'false'
                    if selection.odometry_mode == 'ideal'
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
        actions.extend([
            _include(
                'robot_mapping',
                'localization.launch.py',
                {
                    'use_sim_time': use_sim_time,
                    'map_file': selection.occupancy_map_file,
                    'localization_backend': selection.localization_owner,
                    'amcl_params_file': str(amcl_params_file),
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
                },
            ),
        ])
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
                'module2_enabled': module2_enabled,
                'route_graph_file': selection.route_graph_file,
                'feasible_only_largest_component': LaunchConfiguration(
                    'feasible_only_largest_component').perform(context),
                'module2_response_timeout_s': LaunchConfiguration(
                    'module2_response_timeout_s').perform(context),
                'module2_prior_ttl_s': LaunchConfiguration(
                    'module2_prior_ttl_s').perform(context),
                'cognitive_graph_mode': cognitive_graph_mode,
                'cognitive_constraints_override_file': LaunchConfiguration(
                    'cognitive_constraints_override_file').perform(context),
                'voxel_grid_topic': (
                    'stvl_voxel_grid'
                    if nav2_profile in {
                        'dynamic_avoidance', 'bio_nav_planning_only',
                        'bio_nav_risk_only', 'bio_nav_tiebreak_risk',
                        'bio_nav_rgbd_risk_shadow',
                        'bio_nav_rgbd_risk_ab',
                        'bio_nav_rgbd_risk_static_opt_in'}
                    else 'voxel_grid'
                ),
            }
        if cognitive_overlay_file is not None:
            navigation_arguments['cognitive_profile_params_file'] = str(
                cognitive_overlay_file)
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
        gate_runtime_overlay = _write_activation_gate_runtime_overlay(
            use_sim_time=use_sim_time,
            initial_pose_source=initial_pose_source,
            startup_timeout=LaunchConfiguration(
                'activation_startup_timeout').perform(context),
            startup_timeout_policy=LaunchConfiguration(
                'activation_startup_policy').perform(context),
        )
        activation_gate = Node(
            package='robot_bringup',
            executable='nav2_activation_gate',
            name='nav2_activation_gate',
            output='screen',
            parameters=[
                str(gate_config),
                str(gate_runtime_overlay),
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
            if interactive.localization_collection:
                teleop_arguments.append('--localization-collection')
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
            if interactive.localization_collection:
                teleop_log = (
                    'Simulation-only manual localization collection Teleop '
                    'is running in a separate terminal.\n'
                    'Click the window titled "Isaac Nav Localization '
                    'Collection Teleop"\n'
                    'before pressing W/A/S/D or the arrow keys.'
                )
            else:
                teleop_log = (
                    'Mapping Teleop is running in a separate terminal.\n'
                    'Click the window titled "Isaac Nav Mapping Teleop"\n'
                    'before pressing W/A/S/D or the arrow keys.'
                )
            actions.extend([
                ExecuteProcess(
                    cmd=[str(terminal_wrapper), *terminal_command],
                    output='screen',
                ),
                LogInfo(msg=teleop_log),
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
            description=(
                'ideal, realistic compatibility, estimated, or mixed '
                'compute_amcl_dual_odom')),
        DeclareLaunchArgument(
            'structure_tf_source',
            default_value='isaac',
            description='isaac or rsp'),
        DeclareLaunchArgument('posegraph_file', default_value=''),
        DeclareLaunchArgument(
            'localization_map_contract', default_value='posegraph_bundle',
            description='posegraph_bundle or explicit AMCL occupancy_only'),
        DeclareLaunchArgument(
            'localization_owner', default_value='auto',
            description='auto, ideal, or amcl'),
        DeclareLaunchArgument('ceres_num_threads', default_value='12'),
        DeclareLaunchArgument('map_file', default_value=''),
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
            'imu_calibration_params_file',
            default_value='',
            description=(
                'Raw-to-corrected IMU parameters; defaults to the '
                'Isaac V6 calibrated profile')),
        DeclareLaunchArgument('ekf_profile', default_value='wheel_imu'),
        DeclareLaunchArgument(
            'lidar_odometry_validated', default_value='false'),
        DeclareLaunchArgument('ekf_params_file', default_value=''),
        DeclareLaunchArgument(
            'lidar_odometry_backend', default_value='off'),
        DeclareLaunchArgument(
            'lidar_odometry_params_file', default_value=''),
        DeclareLaunchArgument(
            'localization_profile', default_value='kujiale'),
        DeclareLaunchArgument('amcl_params_file', default_value=''),
        DeclareLaunchArgument('nav2_params_file', default_value=''),
        DeclareLaunchArgument(
            'nav2_profile',
            default_value='stable',
            description=(
                'stable, performance, dynamic_avoidance, or optional BioNav '
                'planning-only, risk-only, combined, RGB-D risk Shadow, '
                'controlled RGB-D static A/B, or explicit static opt-in overlay')),
        DeclareLaunchArgument(
            'nav2_profile_params_file',
            default_value='',
            description='explicit benchmark/custom Nav2 overlay YAML'),
        DeclareLaunchArgument(
            'cognitive_profile', default_value='M0',
            description=(
                'M0, M1, M2, or M3 Module2 local-planning contract; '
                'independent from cognitive_graph_mode')),
        DeclareLaunchArgument('module2_enabled', default_value='true'),
        DeclareLaunchArgument('route_graph_file', default_value=''),
        DeclareLaunchArgument(
            'feasible_only_largest_component', default_value='false'),
        DeclareLaunchArgument(
            'module2_response_timeout_s', default_value='0.0'),
        DeclareLaunchArgument('module2_prior_ttl_s', default_value='2.0'),
        DeclareLaunchArgument(
            'cognitive_graph_mode', default_value='gvg',
            description='gvg, shadow, hybrid, or primary'),
        DeclareLaunchArgument(
            'cognitive_constraints_override_file', default_value='',
            description=(
                'optional fixed-scene Module1 YAML providing the canonical '
                '16x16 reachable mask and T_map_canvas')),
        DeclareLaunchArgument(
            'activation_startup_timeout', default_value='120.0',
            description='bounded Nav2 activation readiness deadline'),
        DeclareLaunchArgument(
            'activation_startup_policy', default_value='fail_closed',
            description='fail_closed or wait_for_seed'),
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
            description=(
                'auto, true, false, or localization_collection; the last '
                'is simulation-only manual localization data collection')),
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
