import math
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from tf2_ros import TransformException

from robot_bringup.odom_static_localization_tf import (
    OdomStaticLocalization,
    aligned_map_to_odom,
    compose_pose_2d,
    invert_pose_2d,
    yaw_from_quaternion,
    yaw_to_quaternion_zw,
)


SPAWN_G1 = (0.45, -5.35, math.radians(90.0))


def test_alignment_with_identity_odom_equals_spawn_pose():
    assert aligned_map_to_odom(SPAWN_G1, (0.0, 0.0, 0.0)) == SPAWN_G1


def test_alignment_recovers_spawn_after_estimator_offset():
    # Post-reset the EKF re-zeros odom->base_link; any residual offset at the
    # seed must be folded into map->odom so map->base lands on the spawn.
    odom_to_base = (0.12, -0.05, math.radians(3.0))
    map_to_odom = aligned_map_to_odom(SPAWN_G1, odom_to_base)
    map_to_base = compose_pose_2d(map_to_odom, odom_to_base)
    assert math.isclose(map_to_base[0], SPAWN_G1[0], abs_tol=1e-12)
    assert math.isclose(map_to_base[1], SPAWN_G1[1], abs_tol=1e-12)
    assert math.isclose(map_to_base[2], SPAWN_G1[2], abs_tol=1e-12)


def test_invert_compose_roundtrip_and_angle_wrapping():
    pose = (1.2, -0.7, math.radians(175.0))
    identity = compose_pose_2d(pose, invert_pose_2d(pose))
    assert math.isclose(identity[0], 0.0, abs_tol=1e-12)
    assert math.isclose(identity[1], 0.0, abs_tol=1e-12)
    assert math.isclose(identity[2], 0.0, abs_tol=1e-12)
    wrapped = compose_pose_2d(
        (0.0, 0.0, math.radians(170.0)), (0.0, 0.0, math.radians(40.0)))
    assert math.isclose(wrapped[2], math.radians(-150.0), abs_tol=1e-12)


def test_quaternion_yaw_roundtrip():
    for yaw_deg in (-160.0, -90.0, 0.0, 20.0, 90.0, 179.0):
        z, w = yaw_to_quaternion_zw(math.radians(yaw_deg))
        assert math.isclose(
            yaw_from_quaternion(0.0, 0.0, z, w),
            math.radians(yaw_deg),
            abs_tol=1e-12,
        )


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


def _stamp(sec=100):
    return Time(sec=sec, nanosec=0)


def _transform_msg(x, y, yaw):
    z, w = yaw_to_quaternion_zw(yaw)
    return SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=x, y=y, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=z, w=w),
        )
    )


def _node(odom_to_base, spawn=SPAWN_G1):
    """Node-shaped fake driving the real unbound methods."""
    tf_msgs = []
    tf_static_msgs = []
    amcl_msgs = []
    if odom_to_base is None:
        def lookup(_source, _target, _time):
            raise TransformException("no transform")
    else:
        def lookup(_source, _target, _time):
            return _transform_msg(*odom_to_base)
    node = SimpleNamespace(
        _spawn=spawn,
        _position_variance=0.05 ** 2,
        _yaw_variance=math.radians(1.0) ** 2,
        _map_frame='map',
        _odom_frame='odom',
        _base_frame='base_link',
        _tf_buffer=SimpleNamespace(lookup_transform=lookup),
        _broadcaster=SimpleNamespace(sendTransform=tf_msgs.append),
        _static_broadcaster=SimpleNamespace(
            sendTransform=tf_static_msgs.append),
        _amcl_pose_publisher=SimpleNamespace(publish=amcl_msgs.append),
        _alignment=None,
        _align_pending=False,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=_stamp)),
        get_logger=lambda: FakeLogger(),
    )
    for name in (
            '_lookup_odom_to_base', '_align', '_map_to_odom_transform',
            '_publish_amcl_pose'):
        setattr(node, name, getattr(OdomStaticLocalization, name).__get__(node))
    return node, tf_msgs, tf_static_msgs, amcl_msgs


def test_seed_trigger_aligns_then_streams_tf_and_amcl_pose():
    node, tf_msgs, tf_static_msgs, amcl_msgs = _node((0.0, 0.0, 0.0))

    # Before any enrollment seed the backend stays silent on every output.
    OdomStaticLocalization._tick(node)
    assert (tf_msgs, tf_static_msgs, amcl_msgs) == ([], [], [])

    OdomStaticLocalization._on_initialpose(node, SimpleNamespace())
    OdomStaticLocalization._tick(node)
    assert node._alignment == SPAWN_G1
    assert not node._align_pending
    assert len(tf_static_msgs) == 1
    latched = tf_static_msgs[0]
    assert latched.header.frame_id == 'map'
    assert latched.child_frame_id == 'odom'
    assert latched.transform.translation.x == SPAWN_G1[0]
    assert latched.transform.translation.y == SPAWN_G1[1]

    # Steady state: fresh /tf keepalive plus the synthetic /amcl_pose stream.
    OdomStaticLocalization._tick(node)
    assert len(tf_msgs) == 2  # one per post-alignment tick
    assert len(amcl_msgs) == 2
    pose = amcl_msgs[-1]
    assert pose.header.frame_id == 'map'
    assert pose.pose.pose.position.x == SPAWN_G1[0]
    assert pose.pose.pose.position.y == SPAWN_G1[1]
    expected_z, expected_w = yaw_to_quaternion_zw(SPAWN_G1[2])
    assert math.isclose(pose.pose.pose.orientation.z, expected_z)
    assert math.isclose(pose.pose.pose.orientation.w, expected_w)
    assert math.isclose(pose.pose.covariance[0], 0.05 ** 2)
    assert math.isclose(pose.pose.covariance[7], 0.05 ** 2)
    assert math.isclose(pose.pose.covariance[35], math.radians(1.0) ** 2)


def test_repeated_reset_realigns_with_fresh_odom_sample():
    node, _, tf_static_msgs, _ = _node((0.0, 0.0, 0.0))
    OdomStaticLocalization._on_initialpose(node, SimpleNamespace())
    OdomStaticLocalization._tick(node)
    assert node._alignment == SPAWN_G1

    # Second reset: the estimator sample at the seed moved; the new
    # alignment must absorb it and be re-latched on /tf_static.
    drifted = (0.10, 0.02, math.radians(-2.0))
    node._tf_buffer.lookup_transform = lambda *args: _transform_msg(*drifted)
    OdomStaticLocalization._on_initialpose(node, SimpleNamespace())
    OdomStaticLocalization._tick(node)
    assert len(tf_static_msgs) == 2
    expected = aligned_map_to_odom(SPAWN_G1, drifted)
    assert node._alignment == expected
    assert math.isclose(
        tf_static_msgs[-1].transform.translation.x, expected[0])


def test_alignment_waits_for_odom_to_base_tf():
    node, tf_msgs, tf_static_msgs, amcl_msgs = _node(None)
    OdomStaticLocalization._on_initialpose(node, SimpleNamespace())
    OdomStaticLocalization._tick(node)
    assert node._align_pending
    assert node._alignment is None
    assert (tf_msgs, tf_static_msgs, amcl_msgs) == ([], [], [])

    node._tf_buffer.lookup_transform = (
        lambda *args: _transform_msg(0.0, 0.0, 0.0))
    OdomStaticLocalization._tick(node)
    assert node._alignment == SPAWN_G1
    assert len(tf_static_msgs) == 1
