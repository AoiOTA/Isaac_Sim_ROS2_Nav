import math

from builtin_interfaces.msg import Time

from robot_bringup.ideal_localization_tf import identity_map_to_odom
from robot_bringup.ideal_localization_tf import spawn_aligned_map_to_odom


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


def test_spawn_aligned_map_to_odom_places_ideal_odom_at_g1():
    transform = spawn_aligned_map_to_odom(
        Time(sec=12, nanosec=345), x=0.45, y=-5.35, yaw_deg=90.0)

    assert transform.transform.translation.x == 0.45
    assert transform.transform.translation.y == -5.35
    assert math.isclose(transform.transform.rotation.z, math.sqrt(0.5))
    assert math.isclose(transform.transform.rotation.w, math.sqrt(0.5))
