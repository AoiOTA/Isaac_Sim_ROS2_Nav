from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _params():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'ekf.yaml').read_text())
    return document['ekf_filter_node']['ros__parameters']


def test_ekf_owns_only_local_odom_transform():
    params = _params()
    assert params['frequency'] == 50.0
    assert params['two_d_mode'] is True
    assert params['map_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_link_frame'] == 'base_link'
    assert params['world_frame'] == 'odom'
    assert params['publish_tf'] is True


def test_ekf_fuses_wheel_vx_and_imu_wz_without_skid_yaw_bias():
    params = _params()
    wheel_enabled = {
        index for index, enabled in enumerate(params['odom0_config']) if enabled}
    imu_enabled = {
        index for index, enabled in enumerate(params['imu0_config']) if enabled}
    assert params['odom0'] == '/wheel/odom'
    assert params['imu0'] == '/imu/data'
    assert len(params['odom0_config']) == 15
    assert len(params['imu0_config']) == 15
    assert wheel_enabled == {6}
    assert imu_enabled == {11}
    assert params['imu0_remove_gravitational_acceleration'] is False
