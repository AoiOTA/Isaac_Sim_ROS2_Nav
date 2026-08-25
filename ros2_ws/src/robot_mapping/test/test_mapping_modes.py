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


def test_production_localization_keeps_grid_and_adds_explicit_amcl_backend():
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
    assert "{'ideal', 'grid', 'amcl'}" in source
    assert "default_value='grid'" in source
    assert source.count("package='nav2_amcl'") == 1
    assert source.count("executable='amcl'") == 1
    assert 'odom_static_localization_tf' not in source
    assert "('localization_result', '/initialpose')" not in source


def test_grid_backend_expands_only_grid_localization_owners(tmp_path):
    import importlib.util

    from launch import LaunchContext
    from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
    from launch_ros.utilities import evaluate_parameters

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
    container = next(
        action for action in actions
        if isinstance(action, ComposableNodeContainer))
    converter, localizer = (
        container._ComposableNodeContainer__composable_node_descriptions)
    assert evaluate_parameters(context, converter.parameters) == ({
        'use_sim_time': True,
        'input_qos': 'SENSOR_DATA',
    },)
    assert evaluate_parameters(context, localizer.parameters) == (
        map_file,
        {
            'use_sim_time': True,
            'loc_result_frame': 'map',
            'map_yaml_path': str(map_file),
        },
    )
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


def test_amcl_backend_expands_only_map_server_amcl_and_lifecycle(tmp_path):
    import importlib.util

    from launch import LaunchContext
    from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
    from launch_ros.utilities import evaluate_parameters

    spec = importlib.util.spec_from_file_location(
        'test_amcl_localization_launch',
        PACKAGE_ROOT / 'launch' / 'localization.launch.py')
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    map_file = tmp_path / 'map.yaml'
    map_file.write_text('image: map.pgm\n')
    amcl_params = PACKAGE_ROOT / 'config' / 'amcl_kujiale.yaml'
    parsed_params = yaml.safe_load(amcl_params.read_text(encoding='utf-8'))
    assert parsed_params['amcl']['ros__parameters']['scan_topic'] == '/scan'
    assert parsed_params['amcl']['ros__parameters']['tf_broadcast'] is True

    context = LaunchContext()
    context.launch_configurations.update({
        'localization_backend': 'amcl',
        'map_file': str(map_file),
        'amcl_params_file': str(amcl_params),
        'use_sim_time': 'true',
        'autostart': 'true',
    })
    actions = module._launch_setup(context)
    nodes = {
        (action.node_package, action.node_executable)
        for action in actions
        if isinstance(action, (ComposableNodeContainer, Node, LifecycleNode))
    }
    assert nodes == {
        ('nav2_map_server', 'map_server'),
        ('nav2_amcl', 'amcl'),
        ('nav2_lifecycle_manager', 'lifecycle_manager'),
    }
    assert not any(
        isinstance(action, ComposableNodeContainer) for action in actions)
    amcl = next(
        action for action in actions
        if isinstance(action, LifecycleNode)
        and action.node_executable == 'amcl')
    assert amcl._Node__parameters[0].evaluate(context) == amcl_params
    lifecycle = next(
        action for action in actions
        if isinstance(action, Node)
        and action.node_executable == 'lifecycle_manager')
    evaluated = evaluate_parameters(context, lifecycle._Node__parameters)
    assert {'node_names': ('map_server', 'amcl')} in evaluated

    map_to_odom_candidates = [
        action for action in actions
        if isinstance(action, (Node, LifecycleNode))
        and action.node_executable in {
            'amcl', 'grid_localization_tf_manager', 'ideal_localization_tf'}
    ]
    assert [action.node_executable for action in map_to_odom_candidates] == [
        'amcl']


def test_localization_package_declares_required_runtime_dependencies():
    package = (PACKAGE_ROOT / 'package.xml').read_text(encoding='utf-8')
    assert package.count('<exec_depend>nav2_amcl</exec_depend>') == 1
    assert package.count(
        '<exec_depend>isaac_ros_occupancy_grid_localizer</exec_depend>') == 1
    assert package.count(
        '<exec_depend>isaac_ros_pointcloud_utils</exec_depend>') == 1
    assert package.count(
        '<exec_depend>robot_grid_localization</exec_depend>') == 1
    assert package.count('<exec_depend>nav2_map_server</exec_depend>') == 1
    assert package.count('<exec_depend>nav2_lifecycle_manager</exec_depend>') == 1
