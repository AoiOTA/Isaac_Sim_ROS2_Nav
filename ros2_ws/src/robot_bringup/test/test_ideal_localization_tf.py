from builtin_interfaces.msg import Time

from robot_bringup.ideal_localization_tf import identity_map_to_odom


def test_identity_map_to_odom_is_fresh_and_exact():
    stamp = Time(sec=12, nanosec=345)
    transform = identity_map_to_odom(stamp)

    assert transform.header.stamp == stamp
    assert transform.header.frame_id == 'map'
    assert transform.child_frame_id == 'odom'
    assert transform.transform.translation.x == 0.0
    assert transform.transform.translation.y == 0.0
    assert transform.transform.translation.z == 0.0
    assert transform.transform.rotation.x == 0.0
    assert transform.transform.rotation.y == 0.0
    assert transform.transform.rotation.z == 0.0
    assert transform.transform.rotation.w == 1.0
