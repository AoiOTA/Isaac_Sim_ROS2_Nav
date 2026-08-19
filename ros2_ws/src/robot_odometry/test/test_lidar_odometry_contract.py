from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_rf2o_is_topic_only_and_cannot_publish_a_second_tf_tree():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'rf2o.yaml').read_text())
    params = document['rf2o_laser_odometry_node']['ros__parameters']
    assert params['laser_scan_topic'] == '/scan'
    assert params['odom_topic'] == '/lidar/odom'
    assert params['publish_tf'] is False
    assert params['odom_frame_id'] == 'odom'


def test_rf2o_launch_fails_fast_when_explicitly_requested_but_missing():
    source = (PACKAGE_ROOT / 'launch' / 'lidar_odometry.launch.py').read_text()
    assert "get_package_prefix('rf2o_laser_odometry')" in source
    assert 'not installed' in source
    assert "('odom', '/lidar/odom')" in source
