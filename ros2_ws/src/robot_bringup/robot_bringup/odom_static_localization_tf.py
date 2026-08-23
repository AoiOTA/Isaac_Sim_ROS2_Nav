"""Odom-static dev localization backend (V6 A/B arm B: no AMCL).

One fixed map->odom transform per episode: each enrollment seed
(``/initialpose``, the same trigger the enrollment machinery and the reset
service's deferred republisher use) re-anchors the transform once as

    map->odom = spawn_pose ∘ (odom->base_link)^-1

with the spawn pose supplied by launch parameters (resolved from the episode
spawn file) and odom->base_link sampled at the seed.  Afterwards the
transform stays fixed: no scan corrections, so EKF drift shows up directly
in the map-frame trajectory.

The aligned transform is rebroadcast on /tf with fresh stamps (the Nav2
activation gate requires a continuously fresh, stable map->odom stream) and
republished latched on /tf_static at every alignment.  A synthetic
/amcl_pose stream (fixed map->odom ∘ latest odom->base_link, enrollment
covariance convention) keeps the AMCL-mode interface for the runner, the B5
supervisor, and the bridge.  This node never touches Ground Truth.
"""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros import TransformListener


def normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def yaw_to_quaternion_zw(yaw: float) -> tuple[float, float]:
    half = yaw * 0.5
    return math.sin(half), math.cos(half)


def invert_pose_2d(pose: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, yaw = pose
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        -(cos_yaw * x + sin_yaw * y),
        sin_yaw * x - cos_yaw * y,
        normalize_yaw(-yaw),
    )


def compose_pose_2d(
    parent: tuple[float, float, float],
    child: tuple[float, float, float],
) -> tuple[float, float, float]:
    px, py, pyaw = parent
    cx, cy, cyaw = child
    cos_yaw = math.cos(pyaw)
    sin_yaw = math.sin(pyaw)
    return (
        px + cos_yaw * cx - sin_yaw * cy,
        py + sin_yaw * cx + cos_yaw * cy,
        normalize_yaw(pyaw + cyaw),
    )


def aligned_map_to_odom(
    spawn_map_pose: tuple[float, float, float],
    odom_to_base: tuple[float, float, float],
) -> tuple[float, float, float]:
    """map->odom = spawn_pose ∘ (odom->base_link)^-1 (2D planar composition)."""
    return compose_pose_2d(spawn_map_pose, invert_pose_2d(odom_to_base))


def _require_finite(values, label: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain only finite values")


class OdomStaticLocalization(Node):
    """Fixed map->odom backend re-anchored by each enrollment seed."""

    def __init__(self) -> None:
        super().__init__('odom_static_localization_tf')
        spawn_x = float(self.declare_parameter('spawn_map_x', 0.0).value)
        spawn_y = float(self.declare_parameter('spawn_map_y', 0.0).value)
        spawn_yaw_deg = float(
            self.declare_parameter('spawn_map_yaw_deg', 0.0).value)
        publish_rate_hz = float(
            self.declare_parameter('publish_rate_hz', 20.0).value)
        position_stddev_m = float(
            self.declare_parameter('position_stddev_m', 0.05).value)
        yaw_stddev_deg = float(
            self.declare_parameter('yaw_stddev_deg', 1.0).value)
        _require_finite(
            (spawn_x, spawn_y, spawn_yaw_deg), 'spawn map pose')
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be finite and positive')
        if position_stddev_m < 0.0 or yaw_stddev_deg < 0.0:
            raise ValueError('pose standard deviations must be non-negative')
        self._spawn = (spawn_x, spawn_y, math.radians(spawn_yaw_deg))
        self._position_variance = position_stddev_m ** 2
        self._yaw_variance = math.radians(yaw_stddev_deg) ** 2
        self._map_frame = 'map'
        self._odom_frame = 'odom'
        self._base_frame = 'base_link'

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._amcl_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', reliable)
        self._initialpose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._on_initialpose,
            reliable,
        )
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False)
        self._broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._alignment: tuple[float, float, float] | None = None
        self._align_pending = False
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._tick)

    def _on_initialpose(self, message: PoseWithCovarianceStamped) -> None:
        del message
        # Any enrollment seed (startup, reset, or reseed burst) re-anchors
        # the fixed transform; burst repeats are idempotent.
        self._align_pending = True

    def _lookup_odom_to_base(self) -> tuple[float, float, float] | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time())
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            translation.x,
            translation.y,
            yaw_from_quaternion(
                rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def _map_to_odom_transform(self, stamp) -> TransformStamped:
        assert self._alignment is not None
        x, y, yaw = self._alignment
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        z, w = yaw_to_quaternion_zw(yaw)
        transform.transform.rotation.z = z
        transform.transform.rotation.w = w
        return transform

    def _align(self, odom_to_base: tuple[float, float, float]) -> None:
        self._alignment = aligned_map_to_odom(self._spawn, odom_to_base)
        self._align_pending = False
        stamp = self.get_clock().now().to_msg()
        self._static_broadcaster.sendTransform(
            self._map_to_odom_transform(stamp))
        x, y, yaw = self._alignment
        self.get_logger().info(
            'Odom-static map->odom aligned to enrollment seed: '
            f'x={x:.3f}, y={y:.3f}, yaw_deg={math.degrees(yaw):.1f}')

    def _publish_amcl_pose(
        self, stamp, map_to_base: tuple[float, float, float]
    ) -> None:
        x, y, yaw = map_to_base
        message = PoseWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = self._map_frame
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        z, w = yaw_to_quaternion_zw(yaw)
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        # Enrollment covariance convention (0.05 m / 1 deg seed stddev).
        message.pose.covariance[0] = self._position_variance
        message.pose.covariance[7] = self._position_variance
        message.pose.covariance[35] = self._yaw_variance
        self._amcl_pose_publisher.publish(message)

    def _tick(self) -> None:
        odom_to_base = self._lookup_odom_to_base()
        if self._align_pending:
            if odom_to_base is None:
                return
            self._align(odom_to_base)
        if self._alignment is None:
            return
        stamp = self.get_clock().now().to_msg()
        self._broadcaster.sendTransform(self._map_to_odom_transform(stamp))
        if odom_to_base is not None:
            self._publish_amcl_pose(
                stamp, compose_pose_2d(self._alignment, odom_to_base))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomStaticLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
