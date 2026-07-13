from pathlib import Path

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
    assert controller['time_steps'] == 20
    assert controller['model_dt'] == 0.10
    assert controller['batch_size'] == 1000
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
    assert launch_source.count("sigterm_timeout='15.0'") == 7


def test_nav2_profiles_are_small_overlays_with_valid_mppi_horizons():
    stable = _profile('stable')
    performance = _profile('performance')
    assert set(stable) == {'controller_server'}
    assert set(performance) == {'controller_server'}

    stable_controller = _params(stable, 'controller_server')
    performance_controller = _params(performance, 'controller_server')
    assert stable_controller['controller_frequency'] == 10.0
    assert stable_controller['FollowPath']['batch_size'] == 750
    assert performance_controller['controller_frequency'] == 10.0
    assert performance_controller['FollowPath']['batch_size'] == 1000
    for profile in (stable, performance):
        controller = _params(profile, 'controller_server')['FollowPath']
        assert controller['model_dt'] == 0.10
        assert controller['time_steps'] in {15, 20}
        assert controller['time_steps'] * controller['model_dt'] >= 1.5
    assert _params(_config(), 'collision_monitor')['source_timeout'] == 0.40
