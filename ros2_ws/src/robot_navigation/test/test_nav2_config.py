import ast
import importlib.util
import math
from pathlib import Path
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _config():
    return yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_params.yaml').read_text())


def _profile(name):
    return yaml.safe_load(
        (PACKAGE_ROOT / 'config' / f'nav2_{name}.yaml').read_text())


def _params(config, node):
    return config[node]['ros__parameters']


def _navigation_launch_module():
    path = PACKAGE_ROOT / 'launch' / 'navigation.launch.py'
    spec = importlib.util.spec_from_file_location('robot_navigation_launch', path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_planner_controller_and_costmaps_are_strictly_two_dimensional():
    config = _config()
    planner = _params(config, 'planner_server')['GridBased']
    controller = _params(config, 'controller_server')['FollowPath']
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = config['global_costmap']['global_costmap'][
        'ros__parameters']

    assert planner['plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert controller['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert controller['motion_model'] == 'DiffDrive'
    assert controller['visualize'] is True
    assert 'TrajectoryVisualizer' not in controller
    assert controller['time_steps'] == 20
    assert math.isclose(controller['model_dt'], 0.10)
    assert controller['batch_size'] == 700
    assert controller['vx_std'] == 0.35
    assert controller['retry_attempt_limit'] == 3
    assert controller['regenerate_noises'] is True
    assert math.isclose(controller['time_steps'] * controller['model_dt'], 2.0)
    assert controller['transform_tolerance'] >= 0.5
    assert local['rolling_window'] is True
    # Nav2 Jazzy declares these two parameters as integers.
    assert type(local['width']) is int
    assert type(local['height']) is int
    assert local['plugins'] == [
        'obstacle_layer', 'depth_voxel_layer', 'inflation_layer']
    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'depth_voxel_layer',
        'inflation_layer']
    voxel = local['depth_voxel_layer']
    assert voxel['plugin'] == 'nav2_costmap_2d::VoxelLayer'
    # Costmap2D declares observation_sources as a string parameter, unlike the
    # Collision Monitor's string-array parameter with the same name.
    assert voxel['observation_sources'] == 'camera_depth'
    assert voxel['camera_depth']['topic'] == '/camera/front/depth/points'
    assert voxel['camera_depth']['sensor_frame'] == 'camera_front_optical_frame'
    assert voxel['camera_depth']['data_type'] == 'PointCloud2'
    assert voxel['camera_depth']['marking'] is True
    assert voxel['camera_depth']['clearing'] is False
    assert voxel['camera_depth']['min_obstacle_height'] == 0.05
    assert voxel['camera_depth']['max_obstacle_height'] == 0.50
    assert voxel['camera_depth']['obstacle_max_range'] == 2.0
    assert voxel['camera_depth']['raytrace_max_range'] == 2.5
    assert voxel['camera_depth']['observation_persistence'] == 1.0
    assert voxel['camera_depth']['expected_update_rate'] == 0.0
    assert voxel['combination_method'] == 1
    global_voxel = global_costmap['depth_voxel_layer']
    assert global_voxel['plugin'] == 'nav2_costmap_2d::VoxelLayer'
    assert global_voxel['camera_depth']['clearing'] is True
    assert global_voxel['camera_depth']['observation_persistence'] == 1.0
    assert local['obstacle_layer']['scan']['topic'] == '/scan_safety'
    assert local['obstacle_layer']['scan']['raytrace_min_range'] == 0.05
    assert local['obstacle_layer']['scan']['obstacle_min_range'] == 0.05
    assert global_costmap['obstacle_layer']['scan']['topic'] == '/scan'


def test_stable_overlay_restores_the_verified_static_mppi_budget():
    stable = _profile('stable')
    parameters = stable['controller_server']['ros__parameters']
    follow_path = parameters['FollowPath']

    assert parameters['controller_frequency'] == 10.0
    assert follow_path['time_steps'] == 20
    assert math.isclose(follow_path['model_dt'], 0.10)
    assert follow_path['batch_size'] == 700
    assert math.isclose(follow_path['time_steps'] * follow_path['model_dt'], 2.0)
    assert follow_path['vx_std'] == 0.35
    assert follow_path['wz_std'] == 0.75
    assert follow_path['vx_max'] == 0.75
    assert follow_path['wz_max'] == 1.35
    assert follow_path['CostCritic']['cost_weight'] == 1.35
    assert follow_path['PathFollowCritic']['cost_weight'] == 10.0

    local = stable['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = stable['global_costmap']['global_costmap'][
        'ros__parameters']
    assert local['obstacle_layer']['scan']['raytrace_min_range'] == 0.05
    assert local['obstacle_layer']['scan']['inf_is_valid'] is False
    assert global_costmap['obstacle_layer']['scan']['raytrace_min_range'] == 0.40
    assert global_costmap['obstacle_layer']['scan']['inf_is_valid'] is False

    smoother = stable['velocity_smoother']['ros__parameters']
    assert smoother['smoothing_frequency'] == 20.0
    assert smoother['max_velocity'] == [0.75, 0.0, 1.35]
    assert smoother['max_accel'] == [1.25, 0.0, 3.50]


def test_planning_only_scan_layers_have_a_strict_clearing_margin():
    profile = _profile('bio_nav_planning_only')
    local = profile['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = profile['global_costmap']['global_costmap'][
        'ros__parameters']

    # Rivermark publishes +inf for an empty beam.  Accept it as free space,
    # retain no scan history, and clear farther than either layer can mark.
    for costmap in (local, global_costmap):
        obstacle = costmap['obstacle_layer']
        scan = obstacle['scan']
        assert obstacle['footprint_clearing_enabled'] is True
        assert scan['clearing'] is True
        assert scan['marking'] is True
        assert scan['observation_persistence'] == 0.0
        assert scan['inf_is_valid'] is True
        assert scan['obstacle_max_range'] == 24.0
        assert scan['raytrace_max_range'] == 25.0
        assert scan['obstacle_max_range'] < scan['raytrace_max_range']

    # The outdoor global costmap has no long-lived depth voxel layer.
    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer']


def test_attempt30_planning_only_profile_binds_the_16m_global_costmap():
    profile = _profile('bio_nav_planning_only')
    global_costmap = profile['global_costmap']['global_costmap'][
        'ros__parameters']

    assert global_costmap['rolling_window'] is False
    assert global_costmap['width'] == 16
    assert global_costmap['height'] == 16
    assert global_costmap['origin_x'] == -8.0
    assert global_costmap['origin_y'] == -8.0


def test_attempt22_reachability_profile_is_preinflation_and_observer_only():
    profile = _profile('attempt22_reachability_shadow')
    global_costmap = profile['global_costmap']['global_costmap'][
        'ros__parameters']

    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'depth_voxel_layer',
        'reachability_observer_layer', 'inflation_layer']
    observer = global_costmap['reachability_observer_layer']
    assert observer['plugin'] == \
        'bio_nav_fusion::ReachabilityObserverLayer'
    assert observer['enabled'] is True
    assert observer['output_topic'] == \
        '/global_costmap/reachability_observer_input'
    assert profile['planner_server']['ros__parameters']['GridBased'][
        'plugin'] == 'nav2_smac_planner::SmacPlanner2D'
    assert 'cognitive_risk_layer' not in global_costmap['plugins']
    assert 'local_rgbd_risk_layer' not in global_costmap['plugins']
    follow_path = profile['controller_server']['ros__parameters']['FollowPath']
    assert follow_path['critics'][-1] == 'PredictiveRiskCritic'
    critic = follow_path['PredictiveRiskCritic']
    assert critic['enabled'] is True
    assert critic['shadow_only'] is True
    assert critic['active_authorized'] is False
    assert critic['expected_model_sha256'] == 64 * '0'
    assert critic['expected_calibration_sha256'] == 64 * '0'
    assert critic['expected_qualification_sha256'] == 64 * '0'


def test_dynamic_avoidance_overlay_preserves_validated_navigation_geometry():
    dynamic = _profile('dynamic_avoidance')
    controller_server = dynamic['controller_server']['ros__parameters']
    behavior_server = dynamic['behavior_server']['ros__parameters']
    base = _config()
    base_controller = base['controller_server']['ros__parameters']
    base_local = base['local_costmap']['local_costmap']['ros__parameters']

    # Dynamic perception must not replace the whole-house-validated MPPI or
    # widen static-wall soft inflation until a feasible corridor disappears.
    assert controller_server == {
        'controller_frequency': 10.0,
        'FollowPath': {
            'time_steps': 20,
            'model_dt': 0.10,
            'batch_size': 700,
        },
    }
    assert 'velocity_smoother' not in dynamic
    local = dynamic['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = dynamic['global_costmap']['global_costmap']['ros__parameters']
    # The frozen dynamic qualification has no RGB-D-only static props.  LiDAR
    # clears its own layer, while STVL gives RGB-D marks a bounded lifetime so
    # a retired actor cannot persist indefinitely in the depth layer.
    assert local['obstacle_layer']['scan'] == {
        'raytrace_min_range': 0.05,
        'inf_is_valid': False,
    }
    assert local['plugins'] == [
        'obstacle_layer', 'depth_stvl_layer', 'inflation_layer',
    ]
    assert 'depth_voxel_layer' not in local['plugins']
    depth = local['depth_stvl_layer']
    assert depth['plugin'] == (
        'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer')
    assert depth['voxel_decay'] == 0.75
    assert depth['decay_model'] == 0
    assert depth['observation_sources'] == (
        'camera_depth_mark camera_depth_clear')
    assert depth['camera_depth_mark'] == {
        'data_type': 'PointCloud2',
        'topic': '/camera/front/depth/points',
        'marking': True,
        'clearing': False,
        'obstacle_range': 2.0,
        'min_obstacle_height': 0.05,
        'max_obstacle_height': 0.50,
        'expected_update_rate': 0.0,
        'observation_persistence': 0.0,
        'filter': 'voxel',
        'voxel_min_points': 0,
        'clear_after_reading': True,
    }
    assert depth['camera_depth_clear'] == {
        'data_type': 'PointCloud2',
        'topic': '/camera/front/depth/points',
        'marking': False,
        'clearing': True,
        'min_z': 0.05,
        'max_z': 2.5,
        'vertical_fov_angle': 1.272,
        'horizontal_fov_angle': 1.839,
        'decay_acceleration': 15.0,
        'model_type': 0,
    }
    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer',
    ]
    assert global_costmap['obstacle_layer']['scan'] == {
        'raytrace_min_range': 0.40,
        'inf_is_valid': False,
    }
    assert base_controller['controller_frequency'] == 10.0
    assert base_controller['FollowPath']['time_steps'] == 20
    assert base_controller['FollowPath']['batch_size'] == 700
    # v25 confines costmap-ahead reverse checking to dynamic recovery.  The
    # shared/static profile retains its validated zero-look-ahead behavior.
    assert behavior_server['simulate_ahead_time'] == 1.0
    assert _params(_config(), 'behavior_server')['simulate_ahead_time'] == 0.0
    assert base_local['inflation_layer']['inflation_radius'] == 0.40
    assert 'near_collision_cost' not in base_controller['FollowPath']['CostCritic']
    base_depth = base_local['depth_voxel_layer']
    assert base_depth['plugin'] == 'nav2_costmap_2d::VoxelLayer'
    assert base_depth['camera_depth']['marking'] is True
    dependencies = {
        item.text
        for item in ElementTree.parse(PACKAGE_ROOT / 'package.xml').getroot()
        .findall('exec_depend')
    }
    assert 'spatio_temporal_voxel_layer' in dependencies


def test_attempt23_global_prior_profile_is_risk_free_and_dynamic_ready():
    profile = _profile('attempt23_global_prior')
    controller_server = profile['controller_server']['ros__parameters']
    controller = controller_server['FollowPath']
    local = profile['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = profile['global_costmap']['global_costmap'][
        'ros__parameters']

    # Attempt-23 removes the deleted PredictiveRiskCritic entirely: local
    # avoidance is stock Nav2 only and nothing consumes Module2 risk grids.
    assert controller['critics'] == [
        'ConstraintCritic', 'CostCritic', 'GoalCritic', 'GoalAngleCritic',
        'PathAlignCritic', 'PathFollowCritic', 'PathAngleCritic',
        'PreferForwardCritic']
    assert 'PredictiveRiskCritic' not in controller

    # The dynamic MPPI envelope carries over from nav2_dynamic_avoidance.
    assert controller_server['controller_frequency'] == 15.0
    assert controller['time_steps'] == 30
    assert math.isclose(controller['model_dt'], 1.0 / 15.0)
    assert math.isclose(
        controller['time_steps'] * controller['model_dt'], 2.0)
    assert controller['batch_size'] == 500
    assert controller['vx_std'] == 0.90
    assert controller['vx_max'] == 1.20
    assert controller['ax_max'] == 3.50
    assert controller['gamma'] == 0.030
    assert controller['CostCritic']['near_collision_cost'] == 20
    assert controller['CostCritic']['cost_weight'] == 4.00
    assert controller['PathFollowCritic']['cost_weight'] == 14.0

    # Local costmap keeps the STVL depth layer and the 0.75 m envelope.
    assert local['plugins'] == [
        'obstacle_layer', 'depth_stvl_layer', 'inflation_layer']
    depth = local['depth_stvl_layer']
    assert depth['plugin'] == (
        'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer')
    assert depth['voxel_decay'] == 0.75
    assert depth['camera_depth_clear']['decay_acceleration'] == 15.0
    assert local['inflation_layer']['inflation_radius'] == 0.75

    # The global side drops the depth layer against dynamic residue but
    # keeps the read-only ReachabilityObserverLayer global prior snapshot.
    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'reachability_observer_layer',
        'inflation_layer']
    observer = global_costmap['reachability_observer_layer']
    assert observer['plugin'] == 'bio_nav_fusion::ReachabilityObserverLayer'
    assert observer['enabled'] is True
    assert observer['output_topic'] == \
        '/global_costmap/reachability_observer_input'
    assert profile['planner_server']['ros__parameters']['GridBased'][
        'plugin'] == 'nav2_smac_planner::SmacPlanner2D'

    assert profile['behavior_server']['ros__parameters'][
        'simulate_ahead_time'] == 1.0
    smoother = profile['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [1.20, 0.0, 3.40]
    assert smoother['max_accel'] == [3.50, 0.0, 6.50]

    # The ApproachZone predictive layer was evaluated and rejected on the
    # Global Shadow evidence: it deadlocked the corridor-follow dynamic case
    # (seed 31704/31709).  StopZone and SlowdownZone stay as defined in the
    # base configuration as the LiDAR safety layers.
    assert profile['collision_monitor']['ros__parameters']['ApproachZone'][
        'enabled'] is False


def test_jazzy_command_chain_uses_unstamped_twist_and_safety_timeouts():
    config = _config()
    navigator = _params(config, 'bt_navigator')
    controller = _params(config, 'controller_server')
    behavior = _params(config, 'behavior_server')
    smoother = _params(config, 'velocity_smoother')
    collision = _params(config, 'collision_monitor')

    assert controller['enable_stamped_cmd_vel'] is False
    assert controller['controller_frequency'] == 10.0
    assert controller['goal_checker']['plugin'] == \
        'nav2_controller::PositionGoalChecker'
    assert controller['goal_checker']['xy_goal_tolerance'] == 0.20
    assert 'yaw_goal_tolerance' not in controller['goal_checker']
    assert 'GoalAngleCritic' not in controller['FollowPath']['critics']
    assert behavior['enable_stamped_cmd_vel'] is False
    assert smoother['enable_stamped_cmd_vel'] is False
    assert collision['enable_stamped_cmd_vel'] is False
    assert smoother['velocity_timeout'] > 0.0
    assert collision['source_timeout'] > 0.0
    assert navigator['default_server_timeout'] >= 500
    assert collision['cmd_vel_in_topic'] == '/cmd_vel_smoothed'
    assert collision['cmd_vel_out_topic'] == '/cmd_vel'
    assert collision['observation_sources'] == ['scan_safety']
    assert collision['scan_safety']['topic'] == '/scan_safety'
    assert collision['scan_safety']['type'] == 'scan'
    assert collision['scan_safety']['enabled'] is True
    assert set(collision['polygons']) == {
        'StopZone', 'SlowdownZone', 'ApproachZone'}

    launch_source = (
        PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()
    assert "remappings=[('cmd_vel', '/cmd_vel_nav')]" in launch_source
    assert "package='nav2_velocity_smoother'" in launch_source
    assert "package='nav2_collision_monitor'" in launch_source


def test_xy_only_goals_do_not_invent_rotation_shim_parameters():
    config = _config()
    controller = _params(config, 'controller_server')
    planner = _params(config, 'planner_server')['GridBased']
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()

    assert controller['goal_checker']['plugin'] == \
        'nav2_controller::PositionGoalChecker'
    assert planner['use_final_approach_orientation'] is False
    assert 'RotationShimController' not in launch_source
    assert 'GoalAngleCritic' not in launch_source
    assert 'rotate_to_goal_heading' not in controller['FollowPath']


def test_mppi_terminal_reverse_limits_are_coherent():
    config = _config()
    controller_server = _params(config, 'controller_server')
    controller = controller_server['FollowPath']
    smoother = _params(config, 'velocity_smoother')

    assert controller_server['progress_checker']['plugin'] \
        == 'nav2_controller::PoseProgressChecker'
    assert controller_server['progress_checker'][
        'required_movement_angle'] > 0.0
    assert -0.20 <= controller['vx_min'] <= -0.10
    assert controller['vx_std'] == 0.35
    assert controller['vx_max'] == 0.75
    assert controller['wz_std'] == 0.75
    assert controller['wz_max'] == 1.35
    assert controller['gamma'] == 0.015
    assert controller['PathAngleCritic']['mode'] == 0
    assert controller['PathFollowCritic']['cost_weight'] == 10.0
    assert controller['PathFollowCritic']['cost_weight'] \
        > controller['PathAlignCritic']['cost_weight']
    assert controller['PathFollowCritic']['offset_from_furthest'] >= 8
    assert 'PreferForwardCritic' in controller['critics']
    prefer_forward = controller['PreferForwardCritic']
    assert prefer_forward['enabled'] is True
    assert prefer_forward['cost_weight'] > 0.0
    assert prefer_forward['threshold_to_consider'] <= 0.5
    assert controller['regenerate_noises'] is True
    assert controller['visualize'] is True

    assert smoother['scale_velocities'] is True
    assert smoother['feedback'] == 'OPEN_LOOP'
    # Static follows the historical 10 Hz optimizer / 20 Hz smoother chain.
    assert smoother['smoothing_frequency'] == 20.0
    assert smoother['max_velocity'] == [
        controller['vx_max'], 0.0, controller['wz_max']]
    # Both MPPI terminal centring and the behavior server's BackUp remain
    # inside the command chain's bounded negative velocity envelope.
    assert -0.30 <= smoother['min_velocity'][0] <= -0.18
    assert smoother['min_velocity'][1:] == [0.0, -controller['wz_max']]
    assert smoother['max_accel'] == [
        controller['ax_max'], 0.0, controller['az_max']]
    assert smoother['max_decel'] == [
        controller['ax_min'], 0.0, -controller['az_max']]


def test_narrow_passage_profile_preserves_physical_collision_safety():
    config = _config()
    controller = _params(config, 'controller_server')['FollowPath']
    planner = _params(config, 'planner_server')['GridBased']
    collision = _params(config, 'collision_monitor')
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = config['global_costmap']['global_costmap'][
        'ros__parameters']

    physical = ast.literal_eval(local['footprint'])
    stop = ast.literal_eval(collision['StopZone']['points'])
    slowdown = ast.literal_eval(collision['SlowdownZone']['points'])

    physical_x = [point[0] for point in physical]
    physical_y = [point[1] for point in physical]
    stop_x = [point[0] for point in stop]
    stop_y = [point[1] for point in stop]
    slowdown_x = [point[0] for point in slowdown]
    slowdown_y = [point[1] for point in slowdown]

    # Emergency stop reserves at least 170 mm ahead for a 10 Hz scan interval
    # plus actuation latency, while its narrow lateral shell remains suitable
    # for indoor doorways.
    assert max(stop_x) - max(physical_x) >= 0.17
    assert min(physical_x) - min(stop_x) >= 0.019
    assert max(stop_y) - max(physical_y) >= 0.019
    assert min(physical_y) - min(stop_y) >= 0.019
    assert max(stop_y) <= 0.24

    assert max(slowdown_x) > max(stop_x)
    assert min(slowdown_x) < min(stop_x)
    assert max(slowdown_y) > max(stop_y)
    assert min(slowdown_y) < min(stop_y)
    # Parallel walls must not permanently halve the command in a traversable
    # indoor corridor.  The slowdown shell remains outside the emergency stop
    # shell while retaining enough speed for stable MPPI path tracking.
    assert max(slowdown_y) == 0.232
    assert max(slowdown_x) - max(stop_x) >= 0.10
    assert collision['SlowdownZone']['min_points'] == 6
    assert 0.85 <= collision['SlowdownZone']['slowdown_ratio'] <= 0.92
    assert collision['ApproachZone']['time_before_collision'] >= 1.0
    assert collision['ApproachZone']['enabled'] is False

    for costmap in (local, global_costmap):
        assert costmap['footprint'] == local['footprint']
        assert 0.0 <= costmap['footprint_padding'] <= 0.005
        inflation = costmap['inflation_layer']
        padded_radius = max(
            math.hypot(x, y) for x, y in physical
        ) + costmap['footprint_padding']
        assert inflation['inflation_radius'] > padded_radius
        assert inflation['inflation_radius'] <= 0.45
        assert inflation['cost_scaling_factor'] >= 6.0

    voxel = local['depth_voxel_layer']
    assert voxel['z_voxels'] == 16
    # A front-only RGB-D camera must not turn every unobserved local column
    # into a 2D unknown obstacle over currently free space.
    assert voxel['unknown_threshold'] == voxel['z_voxels']

    assert planner['cost_travel_multiplier'] <= 1.5
    assert planner['tolerance'] == 0.10
    assert planner['tolerance'] < _params(config, 'controller_server')[
        'goal_checker']['xy_goal_tolerance']
    # Retain smoothing after the isolated-topic diagnostic disproved the
    # earlier mixed-topic attribution and exposed reversals in raw grid paths.
    assert planner['smoother']['max_iterations'] == 1000
    assert planner['smoother']['do_refinement'] is True
    assert controller['CostCritic']['consider_footprint'] is True
    assert controller['CostCritic']['cost_weight'] <= 2.5
    assert controller['CostCritic']['trajectory_point_step'] == 1
    assert controller['CostCritic']['collision_cost'] >= 1000000.0
    assert 'GoalAngleCritic' not in controller
    assert controller['PathAlignCritic']['max_path_occupancy_ratio'] >= 0.30


def test_attempt21_static_collection_tapers_only_the_rear_safety_shell():
    profile = _profile('attempt21_static_collection')
    navigator = profile['bt_navigator']['ros__parameters']
    collision = profile['collision_monitor']['ros__parameters']
    local = profile['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = profile['global_costmap']['global_costmap']['ros__parameters']
    stop = ast.literal_eval(collision['StopZone']['points'])
    slowdown = ast.literal_eval(collision['SlowdownZone']['points'])

    # Preserve the stable profile's forward stopping distance and full front
    # shell.  Only returns behind the axle use the padded physical half-width.
    assert max(x for x, _ in stop) == 0.430
    assert max(y for x, y in stop if x >= 0.0) == 0.230
    assert min(y for x, y in stop if x >= 0.0) == -0.230
    assert max(y for x, y in stop if x < 0.0) == 0.215
    assert min(y for x, y in stop if x < 0.0) == -0.215
    assert min(x for x, _ in stop) <= -0.250

    assert max(x for x, _ in slowdown) > max(x for x, _ in stop)
    assert min(x for x, _ in slowdown) < min(x for x, _ in stop)
    assert max(y for x, y in slowdown if x < 0.0) == 0.217
    assert min(y for x, y in slowdown if x < 0.0) == -0.217
    assert navigator['default_server_timeout'] == 30000
    # Keep the 9124afd static-navigation behavior: RGB-D is planned through
    # both VoxelLayers and normal footprint clearing remains active.
    assert 'depth_voxel_layer' not in local
    assert 'depth_voxel_layer' not in global_costmap
    assert 'footprint_padding' not in local
    assert 'footprint_padding' not in global_costmap
    assert collision['observation_sources'] == ['scan_safety']


def test_dead_end_recovery_backs_up_before_attempting_spin():
    config = _config()
    behavior = _params(config, 'behavior_server')
    collision = _params(config, 'collision_monitor')
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()
    cmake_source = (PACKAGE_ROOT / 'CMakeLists.txt').read_text()
    tree_names = [
        'navigate_to_pose_with_dead_end_recovery.xml',
        'navigate_through_poses_with_dead_end_recovery.xml',
    ]

    assert 'install(DIRECTORY behavior_trees config launch' in cmake_source
    # Behavior-server footprint rollout rejects recovery immediately when a
    # narrow local costmap already marks the current footprint occupied. The
    # lidar Collision Monitor remains the final hard-stop for reverse commands.
    assert behavior['simulate_ahead_time'] == 0.0
    assert collision['cmd_vel_in_topic'] == '/cmd_vel_smoothed'
    assert collision['cmd_vel_out_topic'] == '/cmd_vel'
    assert 'StopZone' in collision['polygons']
    assert collision['StopZone']['action_type'] == 'stop'
    for tree_name in tree_names:
        assert tree_name in launch_source
        tree_path = PACKAGE_ROOT / 'behavior_trees' / tree_name
        root = ElementTree.parse(tree_path).getroot()
        recovery_actions = next(
            node for node in root.iter('RoundRobin')
            if node.attrib.get('name') == 'RecoveryActions')
        actions = list(recovery_actions)

        assert [action.tag for action in actions] == [
            'Sequence', 'BackUp', 'Spin', 'Wait']
        backup = actions[1]
        assert 0.45 <= float(backup.attrib['backup_dist']) <= 0.65
        assert 0.15 <= float(backup.attrib['backup_speed']) <= 0.20
        assert float(backup.attrib['time_allowance']) >= 5.0


def test_a21_route_bt_uses_native_goal_updater_and_metric_owners():
    tree_path = PACKAGE_ROOT / 'behavior_trees' / 'navigate_route_lookahead.xml'
    tree = ElementTree.parse(tree_path).getroot()
    assert tree.find('.//GoalUpdater') is not None
    assert tree.find('.//ComputePathToPose') is not None
    assert tree.find('.//FollowPath') is not None
    rate = tree.find('.//RateController')
    assert rate is not None
    assert rate.attrib['hz'] == '@PLANNER_RATE_HZ@'
    recovery = tree.find('.//RecoveryNode')
    assert recovery is not None
    assert recovery.attrib['number_of_retries'] == '@TRANSIENT_RETRY_COUNT@'
    wait = recovery.find('.//Wait')
    assert wait is not None
    assert wait.attrib['wait_duration'] == '@TRANSIENT_RETRY_WAIT_S@'

    launch = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()
    assert 'execute_route_navigation' in launch
    assert 'navigate_route_lookahead.xml' in launch
    assert '_write_route_guided_bt' in launch
    assert "defaults['metric_planning']" in launch
    assert "remappings=[('plan', '/route_server/plan')]" in launch
    assert "'odometry_topic': '/odom'" in launch


def test_a21_runtime_overlay_keeps_grid_2d_default_and_lattice_explicit():
    module = _navigation_launch_module()
    defaults = {
        'metric_planning': {
            'planner_rate_hz': 2.0,
            'tolerance_m': 0.1,
            'max_iterations': 1000000,
            'max_on_approach_iterations': 1000,
            'max_planning_time_s': 2.0,
            'primitive_file': '/tmp/primitives.json',
            'allow_reverse': True,
            'analytic_expansion_ratio': 3.5,
            'analytic_expansion_max_length_m': 3.0,
            'reverse_penalty': 2.0,
            'change_penalty': 0.2,
            'non_straight_penalty': 1.2,
            'cost_penalty': 2.0,
            'rotation_penalty': 5.0,
            'retrospective_penalty': 0.015,
            'lookup_table_size_m': 20.0,
            'cache_obstacle_heuristic': False,
            'smooth_path': True,
        },
        'route_server': {
            'boundary_radius_to_achieve_node_m': 0.35,
            'radius_to_achieve_node_m': 0.5,
            'smooth_corners': False,
        },
        'mppi_route_guidance': {
            'max_linear_velocity_mps': 0.35,
            'linear_velocity_std_mps': 0.2,
            'path_align_weight': 5.0,
            'use_path_orientations': True,
            'path_angle_weight': 10.5,
            'path_angle_mode': 2,
            'cost_critic_weight': 4.0,
            'cost_critic_near_collision_cost': 253,
            'path_follow_weight': 10.0,
            'velocity_deadband_weight': 35.0,
            'velocity_deadband_mps': 0.05,
            'angular_deadband_radps': 0.10,
            'enforce_path_inversion': True,
        },
    }
    parameters = module._a21_nav2_parameters(defaults)
    planner = parameters['planner']
    assert planner['planner_plugins'] == ['GridBased', 'GridLattice']
    assert planner['GridBased']['plugin'] == (
        'nav2_smac_planner::SmacPlanner2D')
    assert planner['GridLattice']['plugin'] == (
        'nav2_smac_planner::SmacPlannerLattice')
    controller = parameters['controller']['FollowPath']
    assert 'GoalAngleCritic' not in controller['critics']
    assert controller['critics'][-1] == 'VelocityDeadbandCritic'
    assert controller['VelocityDeadbandCritic'] == {
        'enabled': True,
        'cost_power': 1,
        'cost_weight': 35.0,
        'deadband_velocities': [0.05, 0.0, 0.10],
    }


def test_navigation_launch_has_last_precedence_controller_envelope():
    source = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text(
        encoding='utf-8')

    assert "'controller_max_linear_velocity_mps'" in source
    assert "'controller_linear_velocity_std_mps'" in source
    assert 'controller_server:' in source
    assert 'vx_max: $(var controller_max_linear_velocity_mps)' in source
    assert 'vx_std: $(var controller_linear_velocity_std_mps)' in source
    assert 'ParameterFile(' in source
    assert 'allow_substs=True' in source
    assert source.index('str(a21_overlay),') < source.index(
        'cognitive_profile_params_file,') < source.index('ParameterFile(')


def test_navigation_launch_binds_typed_route_tracking_lookahead():
    source = (PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text(
        encoding='utf-8')

    assert "'route_tracking_lookahead_m', default_value='0.0'" in source
    assert "route_tracking_lookahead_m = LaunchConfiguration(" in source
    assert "'route_tracking_lookahead_m': ParameterValue(" in source
    assert 'route_tracking_lookahead_m, value_type=float)' in source
