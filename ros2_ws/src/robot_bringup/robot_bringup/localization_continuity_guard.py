"""
ROS adapter publishing a continuity-guarded map->odom transform.

AMCL broadcasts /amcl_pose only (tf_broadcast is disabled); this node turns
each AMCL pose candidate plus the EKF odom->base_link transform into a
map->odom candidate and passes it through the ContinuityGuard state
machine.  The guarded transform is the only map->odom this node publishes,
so Nav2 and the activation gate see smooth, capture-free localization while
AMCL remains the sole localization source.
"""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from robot_bringup.localization_guard_filter import ContinuityGuard
from robot_bringup.localization_guard_filter import GuardConfig
from robot_bringup.localization_guard_filter import PlanarPose
from robot_bringup.localization_guard_filter import STATE_INIT
from robot_bringup.localization_guard_filter import wrap_angle
from std_msgs.msg import Empty as EmptyMessage
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)


def _yaw_from_quaternion(q) -> float:
    """Return the planar yaw component of a quaternion message."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _candidate_map_to_odom(pose_msg, odom_tf) -> PlanarPose:
    """Compose amcl_pose (map->base) with inverse odom->base into map->odom."""
    mb = pose_msg.pose.pose.position
    map_yaw = _yaw_from_quaternion(pose_msg.pose.pose.orientation)
    ot = odom_tf.transform.translation
    odom_yaw = _yaw_from_quaternion(odom_tf.transform.rotation)
    cos_odom = math.cos(odom_yaw)
    sin_odom = math.sin(odom_yaw)
    inv_tx = -(cos_odom * ot.x + sin_odom * ot.y)
    inv_ty = -(-sin_odom * ot.x + cos_odom * ot.y)
    cos_map = math.cos(map_yaw)
    sin_map = math.sin(map_yaw)
    return PlanarPose(
        x=mb.x + cos_map * inv_tx - sin_map * inv_ty,
        y=mb.y + sin_map * inv_tx + cos_map * inv_ty,
        yaw=wrap_angle(map_yaw - odom_yaw),
    )


class LocalizationContinuityGuard(Node):
    """Filter AMCL map->odom candidates for capture-safe continuity."""

    def __init__(self) -> None:
        super().__init__('localization_continuity_guard')
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('candidate_topic', '/amcl_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('future_dating_s', 0.2)
        self.declare_parameter('accept_translation_m', 0.08)
        self.declare_parameter('accept_yaw_deg', 3.0)
        self.declare_parameter('far_translation_m', 0.25)
        self.declare_parameter('far_yaw_deg', 10.0)
        self.declare_parameter('far_accept_samples', 30)
        self.declare_parameter('resume_samples', 5)
        self.declare_parameter('blend_rate', 0.5)

        config = GuardConfig(
            accept_translation_m=float(
                self.get_parameter('accept_translation_m').value),
            accept_yaw_deg=float(self.get_parameter('accept_yaw_deg').value),
            far_translation_m=float(
                self.get_parameter('far_translation_m').value),
            far_yaw_deg=float(self.get_parameter('far_yaw_deg').value),
            far_accept_samples=int(
                self.get_parameter('far_accept_samples').value),
            resume_samples=int(self.get_parameter('resume_samples').value),
            blend_rate=float(self.get_parameter('blend_rate').value),
        )
        self._guard = ContinuityGuard(config)
        self._map_frame = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._future_dating = Duration(
            seconds=float(self.get_parameter('future_dating_s').value))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('candidate_topic').value,
            self._on_candidate,
            QoSProfile(depth=10),
        )
        self.create_subscription(
            EmptyMessage,
            '/simulation/reset_event',
            self._on_reset,
            QoSProfile(depth=10),
        )
        rate = float(self.get_parameter('publish_rate').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('publish_rate must be finite and positive')
        self._publish_timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'localization continuity guard ready: AMCL candidates are '
            'filtered before map->odom is published')

    def _lookup_odom_to_base(self, stamp):
        try:
            return self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                stamp,
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            try:
                return self._tf_buffer.lookup_transform(
                    self._odom_frame,
                    self._base_frame,
                    Time(),
                )
            except TransformException as exc:
                self.get_logger().warn(
                    f'odom->base_link lookup failed: {exc}',
                    throttle_duration_sec=5.0,
                )
                return None

    def _on_candidate(self, msg) -> None:
        odom_tf = self._lookup_odom_to_base(msg.header.stamp)
        if odom_tf is None:
            return
        candidate = _candidate_map_to_odom(msg, odom_tf)
        decision = self._guard.observe(candidate)
        if decision != 'accept':
            self.get_logger().info(
                f'guard decision={decision} state={self._guard.state} '
                f'candidate=({candidate.x:.3f},{candidate.y:.3f},'
                f'{math.degrees(candidate.yaw):.1f})')

    def _on_reset(self, _msg) -> None:
        self._guard.reset()
        self.get_logger().info('simulation reset: guard state cleared')

    def _publish(self) -> None:
        estimate = self._guard.estimate
        if estimate is None or self._guard.state == STATE_INIT:
            return
        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now() + self._future_dating).to_msg()
        transform.header.frame_id = self._map_frame
        transform.child_frame_id = self._odom_frame
        transform.transform.translation.x = estimate.x
        transform.transform.translation.y = estimate.y
        transform.transform.translation.z = 0.0
        half_yaw = estimate.yaw * 0.5
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationContinuityGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
