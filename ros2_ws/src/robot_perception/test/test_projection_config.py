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
    assert params['range_min'] == 0.40
    assert params['range_max'] == 25.0
    assert params['scan_time'] == 0.10
    assert params['use_inf'] is True


def test_safety_projection_covers_the_stop_zone_without_changing_scan():
    legacy = _parameters(
        'pointcloud_to_laserscan.yaml', 'pointcloud_to_laserscan')
    safety = _parameters(
        'pointcloud_to_laserscan_safety.yaml',
        'pointcloud_to_laserscan_safety')

    assert legacy['range_min'] == 0.40
    assert safety['target_frame'] == legacy['target_frame'] == 'base_link'
    assert safety['min_height'] == legacy['min_height']
    assert safety['max_height'] == legacy['max_height']
    assert safety['angle_min'] == legacy['angle_min']
    assert safety['angle_max'] == legacy['angle_max']
    assert safety['angle_increment'] == legacy['angle_increment']
    assert safety['scan_time'] == legacy['scan_time']
    assert safety['range_min'] == 0.05
    assert safety['range_max'] == legacy['range_max'] == 25.0
    assert safety['use_inf'] is True


def test_self_filter_matches_the_padded_physical_footprint():
    params = _parameters('self_filter_optional.yaml', 'lidar_self_filter')
    assert params['input_topic'] == '/lidar/points_raw'
    assert params['output_topic'] == '/lidar/points_scan'
    assert params['target_frame'] == 'base_link'
    assert params['min_xyz'] == [-0.235, -0.215, -0.05]
    assert params['max_xyz'] == [0.260, 0.215, 0.55]
    assert params['transform_timeout'] == 0.05

    launch_source = (
        PACKAGE_ROOT / 'launch' / 'lidar_processing.launch.py').read_text()
    assert "DeclareLaunchArgument('use_self_filter', default_value='false')" \
        in launch_source
    assert "DeclareLaunchArgument('enable_safety_scan', default_value='false')" \
        in launch_source
    assert "default_value='/lidar/points_raw'" in launch_source
    assert "default_value='/scan'" in launch_source
    assert "default_value='/scan_safety'" in launch_source
    assert "executable='lidar_self_filter'" in launch_source
    assert "name='pointcloud_to_laserscan_safety'" in launch_source
