from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _slam_params():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'slam_mapping.yaml').read_text())
    return document['slam_toolbox']['ros__parameters']


def _amcl_params(filename):
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / filename).read_text())
    return document['amcl']['ros__parameters']


def test_mapping_keeps_slam_toolbox_for_map_creation_only():
    params = _slam_params()
    assert params['mode'] == 'mapping'
    assert params['use_map_saver'] is True
    assert params['map_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_frame'] == 'base_link'


def test_estimated_localization_uses_map_server_amcl_and_one_manager():
    source = (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py').read_text()
    assert "package='nav2_map_server'" in source
    assert "package='nav2_amcl'" in source
    assert "name='lifecycle_manager_localization'" in source
    assert "{'node_names': node_names}" in source
    assert "package='slam_toolbox'" not in source
    assert "{'ideal', 'amcl', 'odom_static'}" in source


def test_odom_static_backend_starts_alignment_node_without_amcl(tmp_path):
    """Launch expansion: odom_static owns map->odom without AMCL."""
    import importlib.util

    from launch import LaunchContext
    from launch_ros.actions import LifecycleNode, Node

    spec = importlib.util.spec_from_file_location(
        'test_localization_launch',
        PACKAGE_ROOT / 'launch' / 'localization.launch.py')
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\n')
    context = LaunchContext()
    context.launch_configurations.update({
        'localization_backend': 'odom_static',
        'map_file': str(map_file),
        'use_sim_time': 'true',
        'autostart': 'true',
        'amcl_params_file': '',
        'map_to_odom_x': '0.45',
        'map_to_odom_y': '-5.35',
        'map_to_odom_yaw_deg': '90.0',
    })
    actions = module._launch_setup(context)
    nodes = {
        (action.node_package, action.node_executable)
        for action in actions
        if isinstance(action, (Node, LifecycleNode))
    }
    assert nodes == {
        ('nav2_map_server', 'map_server'),
        ('robot_bringup', 'odom_static_localization_tf'),
        ('nav2_lifecycle_manager', 'lifecycle_manager'),
    }
    executables = [
        action.node_executable for action in actions
        if isinstance(action, (Node, LifecycleNode))
    ]
    assert 'amcl' not in executables


def test_amcl_profiles_are_scene_specific_initial_values_with_tf_ownership():
    kujiale = _amcl_params('amcl_kujiale.yaml')
    rivermark = _amcl_params('amcl_rivermark.yaml')
    for params in (kujiale, rivermark):
        assert params['base_frame_id'] == 'base_link'
        assert params['odom_frame_id'] == 'odom'
        assert params['global_frame_id'] == 'map'
        assert params['scan_topic'] == '/scan'
        assert params['tf_broadcast'] is True
        assert params['robot_model_type'] == (
            'nav2_amcl::DifferentialMotionModel')
    assert kujiale['max_particles'] == 3000
    assert rivermark['max_particles'] == 6000


def test_localization_package_declares_required_runtime_dependencies():
    package = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
    assert package.count('<exec_depend>nav2_amcl</exec_depend>') == 1
    assert package.count('<exec_depend>nav2_map_server</exec_depend>') == 1
    assert package.count('<exec_depend>nav2_lifecycle_manager</exec_depend>') == 1
