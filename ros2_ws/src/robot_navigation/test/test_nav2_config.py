import ast
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
    assert local['obstacle_layer']['scan']['topic'] == '/scan'
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
    assert local['obstacle_layer']['scan']['raytrace_min_range'] == 0.40
    assert local['obstacle_layer']['scan']['inf_is_valid'] is False
    assert global_costmap['obstacle_layer']['scan']['raytrace_min_range'] == 0.40
    assert global_costmap['obstacle_layer']['scan']['inf_is_valid'] is False

    smoother = stable['velocity_smoother']['ros__parameters']
    assert smoother['smoothing_frequency'] == 20.0
    assert smoother['max_velocity'] == [0.75, 0.0, 1.35]
    assert smoother['max_accel'] == [1.25, 0.0, 3.50]


def test_dynamic_avoidance_overlay_uses_temporal_rgbd_voxels():
    dynamic = _profile('dynamic_avoidance')
    controller_server = dynamic['controller_server']['ros__parameters']
    controller = controller_server['FollowPath']
    local = dynamic['local_costmap']['local_costmap']['ros__parameters']
    global_costmap = dynamic['global_costmap']['global_costmap']['ros__parameters']

    assert controller_server['controller_frequency'] == 15.0
    assert controller['time_steps'] == 30
    assert math.isclose(controller['model_dt'], 1.0 / 15.0)
    assert math.isclose(
        controller['model_dt'],
        1.0 / controller_server['controller_frequency'])
    assert math.isclose(
        controller['time_steps'] * controller['model_dt'], 2.0)
    assert controller['batch_size'] == 500
    assert controller['vx_std'] == 0.90
    assert controller['wz_std'] == 3.40
    assert controller['vx_max'] == 1.20
    assert controller['wz_max'] == 3.40
    assert controller['ax_max'] == 3.50
    assert controller['az_max'] == 6.50
    # G2 dynamic-safety repair: enlarge only the dynamic pre-contact cost
    # envelope and its MPPI weight.  Hard collision handling remains in the
    # shared base configuration.
    assert controller['CostCritic']['cost_weight'] == 4.00
    assert controller['CostCritic']['near_collision_cost'] == 20
    assert controller['PathFollowCritic']['cost_weight'] == 14.0
    assert local['update_frequency'] == 10.0
    assert local['publish_frequency'] == 5.0
    assert local['plugins'] == [
        'obstacle_layer', 'depth_stvl_layer', 'inflation_layer']
    assert local['obstacle_layer']['scan']['obstacle_min_range'] == 0.10
    # Dynamic-only 0.75 m inflation moves the soft response before the
    # >=0.10 m actor-clearance boundary without modifying actor geometry or
    # trajectories.
    assert local['inflation_layer']['inflation_radius'] == 0.75
    base_local = _config()['local_costmap']['local_costmap']['ros__parameters']
    assert base_local['inflation_layer']['inflation_radius'] == 0.40
    base_controller = _config()['controller_server']['ros__parameters']['FollowPath']
    assert 'near_collision_cost' not in base_controller['CostCritic']
    # Dynamic runs deliberately do not inherit the base profile's global
    # RGB-D VoxelLayer: a front-facing camera cannot reliably clear a moving
    # actor after it leaves the field of view.  The rolling local STVL owns
    # dynamic RGB-D marking and temporal expiry instead.
    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer']
    depth = local['depth_stvl_layer']
    assert depth['plugin'] == (
        'spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer')
    assert depth['enabled'] is True
    assert depth['voxel_decay'] == 0.75
    assert depth['decay_model'] == 0
    assert depth['voxel_size'] == 0.05
    assert depth['publish_voxel_map'] is True
    assert depth['observation_sources'] == (
        'camera_depth_mark camera_depth_clear')
    marking = depth['camera_depth_mark']
    clearing = depth['camera_depth_clear']
    assert marking['topic'] == '/camera/front/depth/points'
    assert marking['marking'] is True
    assert marking['clearing'] is False
    assert marking['observation_persistence'] == 0.0
    assert clearing['topic'] == marking['topic']
    assert clearing['marking'] is False
    assert clearing['clearing'] is True
    assert clearing['decay_acceleration'] == 15.0
    assert clearing['model_type'] == 0
    for costmap in (local, global_costmap):
        scan = costmap['obstacle_layer']['scan']
        assert scan['clearing'] is True
        assert scan['marking'] is True
        assert scan['observation_persistence'] == 0.0
        assert scan['inf_is_valid'] is True
        assert scan['raytrace_min_range'] == 0.0


def test_jazzy_command_chain_uses_unstamped_twist_and_safety_timeouts():
    config = _config()
    navigator = _params(config, 'bt_navigator')
    controller = _params(config, 'controller_server')
    behavior = _params(config, 'behavior_server')
    smoother = _params(config, 'velocity_smoother')
    collision = _params(config, 'collision_monitor')

    assert controller['enable_stamped_cmd_vel'] is False
    assert controller['controller_frequency'] == 10.0
    assert controller['goal_checker']['xy_goal_tolerance'] == 0.20
    assert controller['goal_checker']['yaw_goal_tolerance'] <= 0.174532925
    assert behavior['enable_stamped_cmd_vel'] is False
    assert smoother['enable_stamped_cmd_vel'] is False
    assert collision['enable_stamped_cmd_vel'] is False
    assert smoother['velocity_timeout'] > 0.0
    assert collision['source_timeout'] > 0.0
    assert navigator['default_server_timeout'] >= 500
    assert collision['cmd_vel_in_topic'] == '/cmd_vel_smoothed'
    assert collision['cmd_vel_out_topic'] == '/cmd_vel'
    assert collision['observation_sources'] == ['scan']
    assert set(collision['polygons']) == {
        'StopZone', 'SlowdownZone', 'ApproachZone'}

    launch_source = (
        PACKAGE_ROOT / 'launch' / 'navigation.launch.py').read_text()
    assert "remappings=[('cmd_vel', '/cmd_vel_nav')]" in launch_source
    assert "package='nav2_velocity_smoother'" in launch_source
    assert "package='nav2_collision_monitor'" in launch_source


def test_mppi_turning_reverse_and_smoothing_limits_are_coherent():
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
    # Routine MPPI tracking stays forward-only, while the command chain must
    # pass the behavior server's bounded negative BackUp recovery velocity.
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

    # Emergency stop still encloses every physical corner with at least a
    # 20 mm shell, while remaining narrow enough for indoor doorways.
    assert max(stop_x) - max(physical_x) >= 0.019
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
    assert controller['CostCritic']['consider_footprint'] is True
    assert controller['CostCritic']['cost_weight'] <= 2.5
    assert controller['CostCritic']['trajectory_point_step'] == 1
    assert controller['CostCritic']['collision_cost'] >= 1000000.0
    assert controller['GoalAngleCritic']['threshold_to_consider'] == 0.20
    assert controller['PathAlignCritic']['max_path_occupancy_ratio'] >= 0.30


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
