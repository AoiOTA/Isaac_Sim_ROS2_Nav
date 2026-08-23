from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _slam_params():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'slam_mapping.yaml').read_text())
    return document['slam_toolbox']['ros__parameters']


def test_mapping_keeps_slam_toolbox_for_map_creation_only():
    params = _slam_params()
    assert params['mode'] == 'mapping'
    assert params['use_map_saver'] is True
    assert params['map_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_frame'] == 'base_link'


def test_production_localization_uses_grid_components_and_one_tf_manager():
    source = (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py').read_text()
    assert "package='nav2_map_server'" in source
    assert "package='isaac_ros_pointcloud_utils'" in source
    assert "'LaserScantoFlatScanNode'" in source
    assert "package='isaac_ros_occupancy_grid_localizer'" in source
    assert "'OccupancyGridLocalizerNode'" in source
    assert "package='robot_grid_localization'" in source
    assert source.count("executable='grid_localization_tf_manager'") == 1
    assert "name='lifecycle_manager_localization'" in source
    assert "{'node_names': node_names}" in source
    assert "package='slam_toolbox'" not in source
    assert "{'ideal', 'grid'}" in source
    assert "default_value='grid'" in source
    assert "package='nav2_amcl'" not in source
    assert 'odom_static_localization_tf' not in source
    assert "('localization_result', '/initialpose')" not in source


def test_grid_backend_expands_only_grid_localization_owners(tmp_path):
    import importlib.util

    from launch import LaunchContext
    from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node

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
        'localization_backend': 'grid',
        'map_file': str(map_file),
        'use_sim_time': 'true',
        'autostart': 'true',
        'map_to_odom_x': '0.45',
        'map_to_odom_y': '-5.35',
        'map_to_odom_yaw_deg': '90.0',
    })
    actions = module._launch_setup(context)
    nodes = {
        (action.node_package, action.node_executable)
        for action in actions
        if isinstance(action, (ComposableNodeContainer, Node, LifecycleNode))
    }
    assert nodes == {
        ('nav2_map_server', 'map_server'),
        ('rclcpp_components', 'component_container_mt'),
        ('robot_grid_localization', 'grid_localization_tf_manager'),
        ('nav2_lifecycle_manager', 'lifecycle_manager'),
    }
    executables = [
        action.node_executable for action in actions
        if isinstance(action, (Node, LifecycleNode))
    ]
    assert 'amcl' not in executables
    assert 'odom_static_localization_tf' not in executables


def test_localization_package_declares_required_runtime_dependencies():
    package = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
    assert '<exec_depend>nav2_amcl</exec_depend>' not in package
    assert package.count(
        '<exec_depend>isaac_ros_occupancy_grid_localizer</exec_depend>') == 1
    assert package.count(
        '<exec_depend>isaac_ros_pointcloud_utils</exec_depend>') == 1
    assert package.count(
        '<exec_depend>robot_grid_localization</exec_depend>') == 1
    assert package.count('<exec_depend>nav2_map_server</exec_depend>') == 1
    assert package.count('<exec_depend>nav2_lifecycle_manager</exec_depend>') == 1
