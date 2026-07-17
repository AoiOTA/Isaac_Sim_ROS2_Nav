import ast
import math
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _config():
    return yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'nav2_params.yaml').read_text())


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
    assert controller['time_steps'] == 20
    assert controller['model_dt'] == 0.10
    assert controller['batch_size'] == 500
    assert controller['time_steps'] * controller['model_dt'] == 2.0
    assert controller['transform_tolerance'] >= 0.5
    assert local['rolling_window'] is True
    # Nav2 Jazzy declares these two parameters as integers.
    assert type(local['width']) is int
    assert type(local['height']) is int
    assert local['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert global_costmap['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer']
    assert 'voxel_layer' not in local
    assert 'voxel_layer' not in global_costmap
    assert local['obstacle_layer']['scan']['topic'] == '/scan'
    assert global_costmap['obstacle_layer']['scan']['topic'] == '/scan'


def test_jazzy_command_chain_uses_unstamped_twist_and_safety_timeouts():
    config = _config()
    navigator = _params(config, 'bt_navigator')
    controller = _params(config, 'controller_server')
    behavior = _params(config, 'behavior_server')
    smoother = _params(config, 'velocity_smoother')
    collision = _params(config, 'collision_monitor')

    assert controller['enable_stamped_cmd_vel'] is False
    assert controller['controller_frequency'] == 10.0
    assert controller['goal_checker']['xy_goal_tolerance'] < 0.25
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
    assert controller['vx_min'] == 0.0
    assert controller['wz_std'] >= 0.60
    assert controller['wz_max'] >= 1.0
    assert controller['PathAngleCritic']['mode'] == 0
    assert controller['PathAngleCritic']['cost_weight'] \
        > controller['PathFollowCritic']['cost_weight']
    assert 'PreferForwardCritic' in controller['critics']
    prefer_forward = controller['PreferForwardCritic']
    assert prefer_forward['enabled'] is True
    assert prefer_forward['cost_weight'] > 0.0
    assert prefer_forward['threshold_to_consider'] <= 0.5
    assert controller['regenerate_noises'] is True
    assert controller['visualize'] is False

    assert smoother['scale_velocities'] is True
    assert smoother['max_velocity'] == [
        controller['vx_max'], 0.0, controller['wz_max']]
    assert smoother['min_velocity'] == [
        controller['vx_min'], 0.0, -controller['wz_max']]
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
    assert 0.0 < collision['SlowdownZone']['slowdown_ratio'] < 1.0
    assert collision['ApproachZone']['time_before_collision'] >= 1.0

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

    assert planner['cost_travel_multiplier'] <= 1.5
    assert planner['tolerance'] \
        < _params(config, 'controller_server')['goal_checker'][
            'xy_goal_tolerance']
    assert controller['CostCritic']['consider_footprint'] is True
    assert controller['CostCritic']['trajectory_point_step'] == 1
    assert controller['CostCritic']['collision_cost'] >= 1000000.0
    assert controller['PathAlignCritic']['max_path_occupancy_ratio'] >= 0.30
