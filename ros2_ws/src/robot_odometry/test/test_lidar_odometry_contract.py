from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RF2O_ROOT = PACKAGE_ROOT.parent / 'rf2o_laser_odometry'


def test_rf2o_is_topic_only_and_cannot_publish_a_second_tf_tree():
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'rf2o.yaml').read_text())
    params = document['rf2o_laser_odometry_node']['ros__parameters']
    assert params['laser_scan_topic'] == '/scan'
    assert params['odom_topic'] == '/lidar/odom'
    assert params['publish_tf'] is False
    assert params['odom_frame_id'] == 'odom'
    assert params['base_frame_id'] == 'base_link'
    assert len(params['pose_covariance_diagonal']) == 6
    assert len(params['twist_covariance_diagonal']) == 6
    assert all(value > 0 for value in params['pose_covariance_diagonal'])
    assert all(value > 0 for value in params['twist_covariance_diagonal'])


def test_rf2o_launch_fails_fast_when_explicitly_requested_but_missing():
    source = (PACKAGE_ROOT / 'launch' / 'lidar_odometry.launch.py').read_text()
    assert "get_package_prefix('rf2o_laser_odometry')" in source
    assert 'not installed' in source
    assert "('odom', '/lidar/odom')" in source


def test_vendored_rf2o_has_one_ros_node_and_cannot_broadcast_tf():
    algorithm_header = (
        RF2O_ROOT
        / 'include'
        / 'rf2o_laser_odometry'
        / 'CLaserOdometry2D.hpp'
    ).read_text()
    wrapper_header = (
        RF2O_ROOT
        / 'include'
        / 'rf2o_laser_odometry'
        / 'CLaserOdometry2DNode.hpp'
    ).read_text()
    wrapper_source = (
        RF2O_ROOT / 'src' / 'CLaserOdometry2DNode.cpp').read_text()

    assert 'class CLaserOdometry2D: public rclcpp::Node' not in algorithm_header
    assert wrapper_header.count('public rclcpp::Node') == 1
    assert 'TransformListener>(\n    *buffer_, this, false)' in wrapper_source
    assert 'TransformBroadcaster' not in wrapper_header
    assert 'sendTransform' not in wrapper_source
    assert 'publish_tf must remain false' in wrapper_source


def test_vendored_rf2o_waits_for_tf_and_enforces_monotonic_scan_stamps():
    source = (RF2O_ROOT / 'src' / 'CLaserOdometry2DNode.cpp').read_text()
    callback_start = source.index('void CLaserOdometry2DNode::LaserCallBack')
    tf_check = source.index('if (!setLaserPoseFromTf())', callback_start)
    tf_failure = source.index('return;', tf_check)
    init_call = source.index('rf2o_ref.init(', tf_failure)
    lookup_start = source.index('bool CLaserOdometry2DNode::setLaserPoseFromTf')
    tf_lookup = source.index('lookupTransform(', lookup_start)
    lookup_failure = source.index('return false;', tf_lookup)

    assert tf_check < tf_failure < init_call
    assert tf_lookup < lookup_failure
    assert 'scan_stamp_ns <= last_scan_stamp_ns' in source
    assert 'odom.header.stamp = rf2o_ref.last_odom_time' in source


def test_vendored_rf2o_covariance_is_parameterized_and_finite_checked():
    source = (RF2O_ROOT / 'src' / 'CLaserOdometry2DNode.cpp').read_text()

    assert 'pose_covariance_diagonal' in source
    assert 'twist_covariance_diagonal' in source
    assert 'std::isfinite(value)' in source
    assert 'odom.pose.covariance[index * 6 + index]' in source
    assert 'odom.twist.covariance[index * 6 + index]' in source


def test_vendor_records_fixed_upstream_revision_and_license():
    upstream = (RF2O_ROOT / 'UPSTREAM.md').read_text()
    package_xml = (RF2O_ROOT / 'package.xml').read_text()

    assert 'b38c68e46387b98845ecbfeb6660292f967a00d3' in upstream
    assert 'https://github.com/MAPIRlab/rf2o_laser_odometry' in upstream
    assert (RF2O_ROOT / 'LICENSE').is_file()
    assert '<package format="3">' in package_xml
    assert '<depend>nav_msgs</depend>' in package_xml
    assert '<depend>boost</depend>' in package_xml
    assert 'cmake_modules' not in package_xml
