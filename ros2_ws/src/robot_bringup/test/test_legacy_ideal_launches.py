"""Exercise legacy Ideal wrappers without starting ROS processes."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

from launch import LaunchContext

from launch_ros.actions import Node

import pytest


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
