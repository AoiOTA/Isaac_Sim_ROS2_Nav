from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
import yaml


def _a21_nav2_parameters(defaults):
    metric = defaults['metric_planning']
    route = defaults['route_server']
    mppi = defaults['mppi_route_guidance']
    common = {
        'tolerance': float(metric['tolerance_m']),
        'downsample_costmap': False,
        'allow_unknown': True,
        'max_iterations': int(metric['max_iterations']),
        'max_on_approach_iterations': int(metric['max_on_approach_iterations']),
        'max_planning_time': float(metric['max_planning_time_s']),
    }
    lattice = dict(common)
    lattice.update({
        'plugin': 'nav2_smac_planner::SmacPlannerLattice',
        'motion_model_for_search': 'STATE_LATTICE',
        'lattice_filepath': str(metric['primitive_file']),
        'allow_reverse_expansion': bool(metric['allow_reverse']),
        'analytic_expansion_ratio': float(metric['analytic_expansion_ratio']),
        'analytic_expansion_max_length': float(
            metric['analytic_expansion_max_length_m']),
        'reverse_penalty': float(metric['reverse_penalty']),
        'change_penalty': float(metric['change_penalty']),
        'non_straight_penalty': float(metric['non_straight_penalty']),
        'cost_penalty': float(metric['cost_penalty']),
        'rotation_penalty': float(metric['rotation_penalty']),
        'retrospective_penalty': float(metric['retrospective_penalty']),
        'lookup_table_size': float(metric['lookup_table_size_m']),
        'cache_obstacle_heuristic': bool(metric['cache_obstacle_heuristic']),
        'smooth_path': bool(metric['smooth_path']),
    })
    grid_2d = dict(common)
    grid_2d.update({
        'plugin': 'nav2_smac_planner::SmacPlanner2D',
        'cost_travel_multiplier': 1.2,
        'use_final_approach_orientation': False,
    })
    return {
        'planner': {
            'expected_planner_frequency': float(metric['planner_rate_hz']),
            # Route Server owns macro-corridor selection.  The default metric
            # planner should therefore produce a locally continuous grid path,
            # not re-introduce a second nonholonomic route choice.  Repeated
            # Lattice replans at the G4 turn emitted start-cell reversals and
            # intermittent Start-occupied/no-path failures.  Keep Lattice as
            # an explicit diagnostic alternative rather than the BT default.
            'planner_plugins': ['GridBased', 'GridLattice'],
            'GridBased': grid_2d,
            'GridLattice': lattice,
        },
        'controller': {
            'FollowPath': {
                'vx_max': float(mppi['max_linear_velocity_mps']),
                'vx_std': float(mppi['linear_velocity_std_mps']),
                'critics': [
                    'ConstraintCritic', 'CostCritic', 'GoalCritic',
                    'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic',
                    'VelocityDeadbandCritic',
                ],
                'PathAlignCritic': {
                    'cost_weight': float(mppi['path_align_weight']),
                    'use_path_orientations': bool(
                        mppi['use_path_orientations']),
                },
                'PathAngleCritic': {
                    'cost_weight': float(mppi['path_angle_weight']),
                    'mode': int(mppi['path_angle_mode']),
                },
                'CostCritic': {
                    'cost_weight': float(mppi['cost_critic_weight']),
                    'near_collision_cost': int(
                        mppi['cost_critic_near_collision_cost']),
                },
                'PathFollowCritic': {
                    'cost_weight': float(mppi['path_follow_weight']),
                },
                # final60d G5 failures spent 85--88% of aligned samples below
                # 0.05 m/s, while all 34 successful rows spent 0% there.
                # Penalize ineffective sampled trajectories without changing
                # velocity bounds, collision checks, Route or recovery logic.
                'VelocityDeadbandCritic': {
                    'enabled': True,
                    'cost_power': 1,
                    'cost_weight': float(mppi['velocity_deadband_weight']),
                    'deadband_velocities': [
                        float(mppi['velocity_deadband_mps']),
                        0.0,
                        float(mppi['angular_deadband_radps']),
                    ],
                },
                'enforce_path_inversion': bool(mppi['enforce_path_inversion']),
            },
        },
        'route': {
            'boundary_radius_to_achieve_node': float(
                route['boundary_radius_to_achieve_node_m']),
            'radius_to_achieve_node': float(
                route['radius_to_achieve_node_m']),
            'smooth_corners': bool(route['smooth_corners']),
            'operations': ['AdjustSpeedLimit', 'ReroutingService'],
            'ReroutingService': {'plugin': 'nav2_route::ReroutingService'},
            'AdjustSpeedLimit': {'plugin': 'nav2_route::AdjustSpeedLimit'},
            'edge_cost_functions': ['DistanceScorer', 'DynamicEdgesScorer'],
            'DistanceScorer': {'plugin': 'nav2_route::DistanceScorer'},
            'DynamicEdgesScorer': {'plugin': 'nav2_route::DynamicEdgesScorer'},
        },
    }


def _write_a21_overlay(parameters):
    document = {
        'controller_server': {
            'ros__parameters': parameters['controller'],
        },
        'planner_server': {
            'ros__parameters': parameters['planner'],
        },
        'route_server': {
            # ``graph_filepath`` is intentionally supplied only by the launch
            # argument below.  An exact-node value in this overlay has higher
            # ROS parameter precedence than a later launch dictionary (which
            # is emitted as ``/**``), so keeping the warehouse default here
            # silently defeated benchmark-specific graph selection.
            'ros__parameters': parameters['route'],
        },
    }
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='attempt30_a21_nav2_', suffix='.yaml',
            delete=False, encoding='utf-8') as stream:
        yaml.safe_dump(document, stream, sort_keys=False)
        return Path(stream.name)


def _write_controller_envelope_overlay():
    """Create an exact-node, launch-substituted final controller overlay."""
    document = """controller_server:
  ros__parameters:
    FollowPath:
      vx_max: $(var controller_max_linear_velocity_mps)
      vx_std: $(var controller_linear_velocity_std_mps)
"""
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='controller_envelope_', suffix='.yaml',
            delete=False, encoding='utf-8') as stream:
        stream.write(document)
        return Path(stream.name)


def _write_empty_overlay():
    """Create a neutral final overlay for legacy profiles."""
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='cognitive_profile_disabled_', suffix='.yaml',
            delete=False, encoding='utf-8') as stream:
        stream.write('{}\n')
        return Path(stream.name)


def _write_route_guided_bt(template_file, metric_defaults):
    template = template_file.read_text(encoding='utf-8')
    replacements = {
        '@PLANNER_RATE_HZ@': format(
            float(metric_defaults['planner_rate_hz']), '.12g'),
        '@TRANSIENT_RETRY_COUNT@': str(
            int(metric_defaults['transient_retry_count'])),
        '@TRANSIENT_RETRY_WAIT_S@': format(
            float(metric_defaults['transient_retry_wait_s']), '.12g'),
    }
    rendered = template
    for token, value in replacements.items():
        if rendered.count(token) != 1:
            raise RuntimeError(
                f'A21 route BT must contain exactly one {token} token')
        rendered = rendered.replace(token, value)
    with tempfile.NamedTemporaryFile(
            mode='w', prefix='attempt30_a21_route_bt_', suffix='.xml',
            delete=False, encoding='utf-8') as stream:
        stream.write(rendered)
        return Path(stream.name)


def generate_launch_description():
    package_share = Path(get_package_share_directory('robot_navigation'))
    default_config = package_share / 'config' / 'nav2_params.yaml'
    default_profile = package_share / 'config' / 'nav2_stable.yaml'
    route_share = Path(get_package_share_directory('robot_route_planner'))
    bridge_share = Path(get_package_share_directory('bio_nav_ros_bridge'))
    graph_file = route_share / 'config' / 'warehouse_new_gvg_v1.geojson'
    defaults_file = bridge_share / 'config' / 'engineering_defaults.yaml'
    defaults = yaml.safe_load(defaults_file.read_text(encoding='utf-8'))
    a21 = _a21_nav2_parameters(defaults)
    a21_overlay = _write_a21_overlay(a21)
    controller_envelope_overlay = _write_controller_envelope_overlay()
    empty_cognitive_overlay = _write_empty_overlay()
    default_nav_to_pose_bt = package_share / 'behavior_trees' / (
        'navigate_to_pose_with_dead_end_recovery.xml')
    default_nav_through_poses_bt = package_share / 'behavior_trees' / (
        'navigate_through_poses_with_dead_end_recovery.xml')
    route_guided_bt_template = package_share / 'behavior_trees' / (
        'navigate_route_lookahead.xml')
    route_guided_bt = _write_route_guided_bt(
        route_guided_bt_template, defaults['metric_planning'])
    params_file = LaunchConfiguration('nav2_params_file')
    profile_params_file = LaunchConfiguration('nav2_profile_params_file')
    cognitive_profile_params_file = LaunchConfiguration(
        'cognitive_profile_params_file')
    voxel_grid_topic = LaunchConfiguration('voxel_grid_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    structural_map_file = LaunchConfiguration('structural_map_file')
    route_graph_file = LaunchConfiguration('route_graph_file')
    feasible_only_largest_component = LaunchConfiguration(
        'feasible_only_largest_component')
    module2_enabled = LaunchConfiguration('module2_enabled')
    execute_route_navigation = LaunchConfiguration('execute_route_navigation')
    module2_response_timeout_s = LaunchConfiguration(
        'module2_response_timeout_s')
    module2_prior_ttl_s = LaunchConfiguration('module2_prior_ttl_s')
    cognitive_graph_mode = LaunchConfiguration('cognitive_graph_mode')
    route_prior_enabled = LaunchConfiguration('route_prior_enabled')
    route_tracking_lookahead_m = LaunchConfiguration(
        'route_tracking_lookahead_m')
    region_config_file = LaunchConfiguration('region_config_file')
    cognitive_constraints_override_file = LaunchConfiguration(
        'cognitive_constraints_override_file')
    region_switch_min_dwell_s = LaunchConfiguration(
        'region_switch_min_dwell_s')
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params_file', default_value=str(default_config)),
        DeclareLaunchArgument(
            'nav2_profile_params_file', default_value=str(default_profile)),
        DeclareLaunchArgument(
            'cognitive_profile_params_file',
            default_value=str(empty_cognitive_overlay),
            description=(
                'exact-node M0-M3 overlay; generated by robot_bringup for '
                'the V6 profile and neutral for legacy profiles')),
        DeclareLaunchArgument('structural_map_file', default_value=''),
        DeclareLaunchArgument(
            'route_graph_file', default_value=str(graph_file)),
        DeclareLaunchArgument(
            'feasible_only_largest_component', default_value='false'),
        DeclareLaunchArgument('module2_enabled', default_value='true'),
        DeclareLaunchArgument('execute_route_navigation', default_value='true'),
        DeclareLaunchArgument('module2_response_timeout_s', default_value='0.0'),
        DeclareLaunchArgument('module2_prior_ttl_s', default_value='2.0'),
        DeclareLaunchArgument(
            'cognitive_graph_mode', default_value='gvg',
            description='gvg, shadow, hybrid, or primary'),
        DeclareLaunchArgument(
            'route_prior_enabled', default_value='true',
            description='resolved boolean edge-prior consumption gate'),
        DeclareLaunchArgument(
            'route_tracking_lookahead_m', default_value='0.0'),
        DeclareLaunchArgument('region_config_file', default_value=''),
        DeclareLaunchArgument(
            'cognitive_constraints_override_file', default_value=''),
        DeclareLaunchArgument('region_switch_min_dwell_s', default_value='0.5'),
        # Preserve the qualified A21 values by default, but let an outdoor
        # caller bind a last-precedence controller envelope explicitly.
        DeclareLaunchArgument(
            'controller_max_linear_velocity_mps',
            default_value=format(
                float(defaults['mppi_route_guidance'][
                    'max_linear_velocity_mps']), '.12g')),
        DeclareLaunchArgument(
            'controller_linear_velocity_std_mps',
            default_value=format(
                float(defaults['mppi_route_guidance'][
                    'linear_velocity_std_mps']), '.12g')),
        # STVL publishes a PointCloud2 named voxel_grid, while Nav2's built-in
        # VoxelLayer publishes nav2_msgs/VoxelGrid on that name.  The dynamic
        # profile remaps STVL to an independent topic so RViz can display both
        # message types without creating conflicting same-name subscriptions.
        DeclareLaunchArgument('voxel_grid_topic', default_value='voxel_grid'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter('use_sim_time', use_sim_time),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[
                params_file,
                profile_params_file,
                str(a21_overlay),
                cognitive_profile_params_file,
                # A node-specific params file wins over the node-specific A21
                # overlay.  A plain launch dictionary becomes a /** wildcard
                # file and cannot override an exact controller_server entry.
                ParameterFile(
                    str(controller_envelope_overlay), allow_substs=True),
            ],
            remappings=[
                ('cmd_vel', '/cmd_vel_nav'),
                ('voxel_grid', voxel_grid_topic),
            ],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[
                params_file,
                profile_params_file,
                str(a21_overlay),
                cognitive_profile_params_file,
            ],
        ),
        Node(
            package='nav2_route',
            executable='route_server',
            name='route_server',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[
                params_file,
                profile_params_file,
                str(a21_overlay),
                {'graph_filepath': route_graph_file},
            ],
            # Route Server and Planner Server otherwise both publish /plan.
            # Keep /plan an unambiguous Smac evidence topic; Route Server's
            # own visualization remains available under its explicit owner.
            remappings=[('plan', '/route_server/plan')],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[params_file, profile_params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[
                params_file,
                {
                    'default_nav_to_pose_bt_xml': str(
                        default_nav_to_pose_bt),
                    'default_nav_through_poses_bt_xml': str(
                        default_nav_through_poses_bt),
                },
            ],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[params_file, profile_params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[params_file, profile_params_file],
        ),
        Node(
            package='robot_route_planner',
            executable='structural_graph',
            name='bio_nav_route_coordinator',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'engineering_defaults_file': str(defaults_file),
                'map_yaml': structural_map_file,
                'feasible_only_largest_component': (
                    feasible_only_largest_component),
                'module2_enabled': module2_enabled,
                'module2_response_timeout_s': module2_response_timeout_s,
                'module2_prior_ttl_s': module2_prior_ttl_s,
                'cognitive_graph_mode': cognitive_graph_mode,
                'route_prior_enabled': route_prior_enabled,
                'route_tracking_lookahead_m': ParameterValue(
                    route_tracking_lookahead_m, value_type=float),
                'region_config_file': region_config_file,
                'cognitive_constraints_override_file': (
                    cognitive_constraints_override_file),
                'region_switch_min_dwell_s': region_switch_min_dwell_s,
                'execute_navigation': execute_route_navigation,
                'route_guided_bt_xml': str(route_guided_bt),
                'odometry_topic': '/odom',
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            sigterm_timeout='15.0',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes},
                # A simulation reset deliberately pauses every managed node
                # while TF and costmaps are re-seeded.  On a loaded Isaac
                # process that transition can take longer than Nav2's
                # default four-second bond timeout, which otherwise causes a
                # freshly resumed controller to be declared dead before its
                # first heartbeat.  Keep the timeout bounded, but long enough
                # to cover the documented reset transaction.
                {'bond_timeout': 10.0},
            ],
        ),
    ])
