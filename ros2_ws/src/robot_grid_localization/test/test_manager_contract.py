from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_manager_freezes_standard_ros_interfaces_and_tf_ownership():
    source = (
        PACKAGE_ROOT / 'robot_grid_localization'
        / 'grid_localization_tf_manager.py').read_text()
    assert "'/localization_result'" in source
    assert "'/bio_nav/localization_pose'" in source
    assert "'/bio_nav/localization/status'" in source
    assert "'/bio_nav/relocalize'" in source
    assert "'/trigger_grid_search_localization'" in source
    assert "transform.child_frame_id = 'odom'" in source
    assert source.count('TransformBroadcaster(self)') == 1
    assert 'StaticTransformBroadcaster' not in source
    assert 'sendTransform(transform)' in source


def test_manager_uses_exact_result_stamp_for_odom_to_base_lookup():
    source = (
        PACKAGE_ROOT / 'robot_grid_localization'
        / 'grid_localization_tf_manager.py').read_text()
    assert "'odom', 'base_link'" in source
    assert 'Time.from_msg(message.header.stamp)' in source
    assert 'Time()' not in source
