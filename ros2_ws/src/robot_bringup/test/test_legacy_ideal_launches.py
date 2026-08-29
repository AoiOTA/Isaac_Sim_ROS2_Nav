"""Exercise legacy Ideal wrappers without starting ROS processes."""

import importlib.util
from dataclasses import replace
from pathlib import Path
import shutil
from types import SimpleNamespace

from launch import LaunchContext

from launch_ros.actions import Node

import pytest

from robot_bringup.mode_contract import validate_mode
from robot_experiments.configuration import ConfigurationError


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent


def _load_launch(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _context(values):
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


@pytest.mark.parametrize(
    ('filename', 'required_names', 'extra_values'),
    (
        (
            'rivermark_navigation.launch.py',
            ('map_file', 'route_graph_file', 'region_config_file',
             'waypoint_config_file'),
            {
                'start_x': '1.0',
                'start_y': '2.0',
                'start_yaw_deg': '30.0',
                'use_rviz': 'false',
                'controller_max_linear_velocity_mps': '0.75',
                'controller_linear_velocity_std_mps': '0.35',
            },
        ),
        (
            'multiroute_benchmark_navigation.launch.py',
            ('map_file', 'route_graph_file', 'spawn_poses_file'),
            {
                'spawn_pose_name': 'start',
                'map_to_odom_x': '1.0',
                'map_to_odom_y': '2.0',
                'map_to_odom_yaw_deg': '30.0',
                'region_config_file': '',
                'region_switch_min_dwell_s': '0.5',
            },
        ),
    ),
)
def test_legacy_navigation_expands_to_ideal_localization_nodes(
    tmp_path, monkeypatch, filename, required_names, extra_values
):
    """Both wrappers select Ideal TF ownership without AMCL defaults."""
    wrapper = _load_launch(
        PACKAGE_ROOT / 'launch' / filename,
        f'test_{filename.replace(".", "_")}',
    )
    included = []

    def capture_include(package, launch_file, arguments):
        value = SimpleNamespace(
            package=package,
            launch_file=launch_file,
            arguments=dict(arguments),
        )
        included.append(value)
        return value

    monkeypatch.setattr(wrapper, '_include', capture_include)
    monkeypatch.setattr(
        wrapper, 'get_package_share_directory', lambda _package: str(tmp_path))
    values = {
        'use_sim_time': 'true',
        'module2_enabled': 'false',
        'nav2_profile_params_file': str(tmp_path / 'nav2.yaml'),
        **extra_values,
    }
    for name in required_names:
        path = tmp_path / f'{name}.yaml'
        path.write_text('test\n')
        values[name] = str(path)

    wrapper._setup(_context(values))
    localization = next(
        item for item in included
        if item.package == 'robot_mapping'
        and item.launch_file == 'localization.launch.py'
    )
    assert localization.arguments['localization_backend'] == 'ideal'
    assert 'use_posegraph_localization' not in localization.arguments

    mapping = _load_launch(
        WORKSPACE_SRC / 'robot_mapping' / 'launch' / 'localization.launch.py',
        f'test_mapping_{filename.replace(".", "_")}',
    )
    actions = mapping._launch_setup(_context(localization.arguments))
    nodes = {
        (action.node_package, action.node_executable)
        for action in actions
        if isinstance(action, Node)
    }
    assert nodes == {
        ('nav2_map_server', 'map_server'),
        ('robot_bringup', 'ideal_localization_tf'),
        ('nav2_lifecycle_manager', 'lifecycle_manager'),
    }
    assert all('amcl_kujiale' not in repr(vars(action)) for action in actions)


def test_outdoor_mixed_ideal_uses_calibrated_spawn_and_indoor_amcl_is_unchanged(
        tmp_path):
    """The owner, not the reset seed source, selects the map->odom backend."""
    repository_root = PACKAGE_ROOT.parents[2]
    demo = repository_root / 'data' / 'rivermark_demo'
    map_file = demo / 'rivermark_selected.yaml'
    route_graph = demo / 'rivermark_selected.geojson'
    spawn_file = demo / 'rivermark.spawn.yaml'
    stack = _load_launch(
        PACKAGE_ROOT / 'launch' / 'ros_stack.launch.py',
        'test_outdoor_fixed_map_to_odom_stack',
    )

    outdoor = validate_mode(
        'navigation',
        'mixed',
        'isaac',
        map_file=str(map_file),
        localization_map_contract='occupancy_only',
        localization_owner='ideal',
        initial_pose_source='isaac',
        route_graph_file=str(route_graph),
    )
    selected_spawn = stack._load_localization_spawn(
        outdoor, str(spawn_file), 'rivermark_start')
    assert selected_spawn is not None
    assert selected_spawn.map.position == (21.2068, 119.978)
    assert selected_spawn.map.yaw_deg == pytest.approx(136.49593386440827)
    assert selected_spawn.map_calibrated is True

    copied_map = tmp_path / map_file.name
    copied_image = tmp_path / 'rivermark_selected.pgm'
    shutil.copy2(map_file, copied_map)
    shutil.copy2(demo / copied_image.name, copied_image)
    copied_selection = replace(
        outdoor, occupancy_map_file=str(copied_map))
    assert stack._load_localization_spawn(
        copied_selection, str(spawn_file), 'rivermark_start') is not None
    copied_image.write_bytes(copied_image.read_bytes() + b'\n')
    with pytest.raises(RuntimeError, match='map_bundle_sha256 does not match'):
        stack._load_localization_spawn(
            copied_selection, str(spawn_file), 'rivermark_start')
    shutil.copy2(demo / copied_image.name, copied_image)
    copied_map.write_bytes(copied_map.read_bytes() + b'# mutation\n')
    with pytest.raises(RuntimeError, match='map_bundle_sha256 does not match'):
        stack._load_localization_spawn(
            copied_selection, str(spawn_file), 'rivermark_start')

    uncalibrated_spawn_file = tmp_path / 'uncalibrated.spawn.yaml'
    uncalibrated_spawn_file.write_text(
        spawn_file.read_text(encoding='utf-8').replace(
            'calibrated: true', 'calibrated: false'),
        encoding='utf-8',
    )
    with pytest.raises(ConfigurationError, match='no calibrated map pose'):
        stack._load_localization_spawn(
            outdoor, str(uncalibrated_spawn_file), 'rivermark_start')

    mapping = _load_launch(
        WORKSPACE_SRC / 'robot_mapping' / 'launch' / 'localization.launch.py',
        'test_outdoor_fixed_map_to_odom_mapping',
    )
    common = {
        'use_sim_time': 'true',
        'autostart': 'true',
        'map_file': str(map_file),
        'amcl_params_file': str(
            WORKSPACE_SRC / 'robot_mapping' / 'config' / 'amcl_kujiale.yaml'),
        'map_to_odom_x': str(selected_spawn.map.position[0]),
        'map_to_odom_y': str(selected_spawn.map.position[1]),
        'map_to_odom_yaw_deg': str(selected_spawn.map.yaw_deg),
    }
    ideal_actions = mapping._launch_setup(_context({
        **common,
        'localization_backend': 'ideal',
    }))
    ideal_nodes = {
        (action.node_package, action.node_executable)
        for action in ideal_actions
        if isinstance(action, Node)
    }
    assert ('robot_bringup', 'ideal_localization_tf') in ideal_nodes
    assert ('nav2_amcl', 'amcl') not in ideal_nodes

    indoor = validate_mode(
        'navigation',
        'mixed',
        'isaac',
        map_file=str(map_file),
        localization_map_contract='occupancy_only',
        localization_owner='amcl',
        route_graph_file=str(route_graph),
    )
    assert indoor.localization_owner == 'amcl'
    assert stack._load_localization_spawn(
        indoor, str(spawn_file), 'rivermark_start') is None
    amcl_actions = mapping._launch_setup(_context({
        **common,
        'localization_backend': 'amcl',
    }))
    amcl_nodes = {
        (action.node_package, action.node_executable)
        for action in amcl_actions
        if isinstance(action, Node)
    }
    assert ('nav2_amcl', 'amcl') in amcl_nodes
    assert ('robot_bringup', 'ideal_localization_tf') not in amcl_nodes
