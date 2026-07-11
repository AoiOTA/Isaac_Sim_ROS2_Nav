from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _parameters(filename, node_name):
    document = yaml.safe_load((PACKAGE_ROOT / 'config' / filename).read_text())
    return document[node_name]['ros__parameters']


def test_projection_contract_matches_navigation_baseline():
    params = _parameters(
        'pointcloud_to_laserscan.yaml', 'pointcloud_to_laserscan')
    assert params['use_sim_time'] is True
    assert params['target_frame'] == 'base_link'
    assert params['min_height'] == 0.05
    assert params['max_height'] == 0.50
    assert params['range_min'] == 0.30
    assert params['range_max'] == 25.0
    assert params['scan_time'] == 0.10
    assert params['use_inf'] is True


def test_self_filter_is_explicitly_opt_in():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'self_filter_optional.yaml').read_text())
    params = document['self_filter']
    assert params['enabled'] is False
    assert params['input_topic'] == '/lidar/points_raw'
    assert params['output_topic'] == '/lidar/points_scan'

    launch_source = (
        PACKAGE_ROOT / 'launch' / 'lidar_processing.launch.py').read_text()
    assert "DeclareLaunchArgument('use_self_filter', default_value='false')" \
        in launch_source
    assert "default_value='/lidar/points_raw'" in launch_source
    assert "default_value='/scan'" in launch_source
