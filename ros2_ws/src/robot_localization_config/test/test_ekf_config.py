from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _params(filename='ekf_wheel_imu.yaml'):
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / filename).read_text())
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


def test_three_source_profile_adds_only_differential_lidar_planar_pose():
    params = _params('ekf_wheel_imu_lidar.yaml')
    lidar_enabled = {
        index for index, enabled in enumerate(params['odom1_config']) if enabled}
    assert params['odom1'] == '/lidar/odom'
    assert lidar_enabled == {0, 1, 5}
    assert params['odom1_differential'] is True
    assert params['odom1_relative'] is False


def test_ekf_launch_profiles_remap_the_single_filtered_output_to_odom():
    source = (PACKAGE_ROOT / 'launch' / 'ekf.launch.py').read_text()
    assert "{'wheel_imu', 'wheel_imu_lidar'}" in source
    assert "f'ekf_{profile}.yaml'" in source
    assert "('odometry/filtered', '/odom')" in source
