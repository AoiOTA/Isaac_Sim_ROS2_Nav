from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _params(filename):
    document = yaml.safe_load((PACKAGE_ROOT / 'config' / filename).read_text())
    return document['slam_toolbox']['ros__parameters']


def test_mapping_and_localization_configs_have_exclusive_semantics():
    mapping = _params('slam_mapping.yaml')
    localization = _params('slam_localization.yaml')

    assert mapping['mode'] == 'mapping'
    assert mapping['use_map_saver'] is True
    assert localization['mode'] == 'localization'
    assert 'use_map_saver' not in localization
    assert localization['throttle_scans'] == 2
    assert localization['minimum_time_interval'] == 0.20
    for params in (mapping, localization):
        assert params['map_frame'] == 'map'
        assert params['odom_frame'] == 'odom'
        assert params['base_frame'] == 'base_link'
        assert params['scan_topic'] == '/scan'
        assert params['use_sim_time'] is True
        assert params['min_laser_range'] == 0.40


def test_launches_use_distinct_jazzy_executables_and_posegraph_pair():
    mapping_source = (
        PACKAGE_ROOT / 'launch' / 'mapping.launch.py').read_text()
    localization_source = (
        PACKAGE_ROOT / 'launch' / 'localization.launch.py').read_text()

    assert "executable='async_slam_toolbox_node'" in mapping_source
    assert "'mode': 'mapping'" in mapping_source
    assert "('.posegraph', '.data')" in mapping_source
    assert "'map_file_name': prefix" in mapping_source
    assert "'use_scan_matching': use_scan_matching" in mapping_source
    assert "'do_loop_closing': do_loop_closing" in mapping_source
    assert "'use_scan_matching'," in mapping_source
    assert "'do_loop_closing'," in mapping_source
    assert "executable='localization_slam_toolbox_node'" \
        in localization_source
    assert "package='nav2_map_server'" in localization_source
    assert "executable='map_server'" in localization_source
    assert "('map', '/slam_toolbox/map')" in localization_source
    assert "'yaml_filename': map_file" in localization_source
    assert "'mode': 'localization'" in localization_source
    assert "('.posegraph', '.data')" in localization_source
    assert "'use_posegraph_localization'" in localization_source
    assert "executable='ideal_localization_tf'" in localization_source
    assert 'return [slam_node, activate, configure]' in mapping_source
    assert 'activate_map' in localization_source
    assert 'configure_map' in localization_source
