"""
ROS adapter publishing a continuity-guarded map->odom transform.

AMCL broadcasts /amcl_pose only (tf_broadcast is disabled); this node turns
each AMCL pose candidate plus the EKF odom->base_link transform into a
map->odom candidate and passes it through the ContinuityGuard state
machine.  The guarded transform is the only map->odom this node publishes,
so Nav2 and the activation gate see smooth, capture-free localization while
AMCL remains the sole localization source.

Hardening on top of the raw filter:

- AMCL covariance is gated (max_sigma_xy_m / max_sigma_yaw_deg); a breach
  forces HOLDING and never rebases.
- odom->base_link is only looked up at the exact AMCL stamp; a lookup
  failure or a TF time deviation beyond tf_max_deviation_ms keeps the
  frozen estimate instead of falling back to the latest transform.
- An AMCL watchdog (amcl_timeout_s) stops the map->odom republication
  entirely while the candidate stream is dead so a stale estimate is never
  future-dated into a fresh-looking transform.  The watchdog is
  motion-aware: AMCL legitimately goes quiet while the robot is stationary
  (its motion-model update thresholds), so silence only trips the watchdog
  when odom->base_link moved within the recent
  watchdog_stationary_window_s window.
- A stabilized far candidate never applies directly: the node enters
  RELOCALIZATION_PENDING, latches /bio_nav/relocalization_event with the
  cluster-mean map->base pose, keeps the frozen estimate for
  relocalize_settle_s, and only then rebases.  The only event-free rebase
  is the simulation-reset reseed (guard reset re-initializes on the next
  candidate).

The latched /localization_guard/status topic reports TRACKING, HOLDING,
STALE, or RELOCALIZATION_PENDING on every transition.
"""

import math
from collections import deque

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from robot_bringup.localization_guard_filter import ContinuityGuard
from robot_bringup.localization_guard_filter import GuardConfig
from robot_bringup.localization_guard_filter import PlanarPose
from robot_bringup.localization_guard_filter import STATE_INIT
from robot_bringup.localization_guard_filter import STATE_TRACKING
from robot_bringup.localization_guard_filter import wrap_angle
from std_msgs.msg import Empty as EmptyMessage
from std_msgs.msg import String as StringMessage
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)


STATUS_TOPIC = '/localization_guard/status'
RELOCALIZATION_EVENT_TOPIC = '/bio_nav/relocalization_event'

STATUS_TRACKING = 'TRACKING'
STATUS_HOLDING = 'HOLDING'
STATUS_STALE = 'STALE'
STATUS_RELOCALIZATION_PENDING = 'RELOCALIZATION_PENDING'


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


def _compose_map_to_base(candidate: PlanarPose, odom_tf):
    """Compose a map->odom candidate with odom->base into map->base."""
    ot = odom_tf.transform.translation
    odom_yaw = _yaw_from_quaternion(odom_tf.transform.rotation)
    cos_map = math.cos(candidate.yaw)
    sin_map = math.sin(candidate.yaw)
    return (
        candidate.x + cos_map * ot.x - sin_map * ot.y,
        candidate.y + sin_map * ot.x + cos_map * ot.y,
        wrap_angle(candidate.yaw + odom_yaw),
    )


def _covariance_sigmas(pose_msg):
    """Return (sigma_xy_m, sigma_yaw_deg) from a PoseWithCovarianceStamped."""
    covariance = pose_msg.pose.covariance
    sigma_xy = math.sqrt(max(covariance[0], covariance[7], 0.0))
    sigma_yaw_deg = math.degrees(math.sqrt(max(covariance[35], 0.0)))
    return sigma_xy, sigma_yaw_deg


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
        self.declare_parameter('cluster_trans_m', 0.05)
        self.declare_parameter('cluster_yaw_deg', 2.0)
        self.declare_parameter('stable_window_s', 1.25)
        self.declare_parameter('resume_samples', 5)
        self.declare_parameter('blend_rate', 0.5)
        self.declare_parameter('max_sigma_xy_m', 0.15)
        self.declare_parameter('max_sigma_yaw_deg', 5.0)
        self.declare_parameter('tf_max_deviation_ms', 50.0)
        self.declare_parameter('amcl_timeout_s', 0.5)
        # When false, a stable far candidate cluster raises
        # RELOCALIZATION_PENDING and publishes the event but never applies
        # the rebase: on sparse maps (rivermark outdoor) an attractive wrong
        # AMCL mode is stable enough to pass the cluster window, and a frozen
        # dead-reckoned map->odom is safer than rebasing into it.  Reseeds
        # after /simulation/reset_event are unaffected.
        self.declare_parameter('allow_autonomous_rebase', True)
        self.declare_parameter('watchdog_stationary_translation_m', 0.08)
        self.declare_parameter('watchdog_stationary_yaw_deg', 3.0)
        # Window over which odom motion is judged for watchdog suppression;
        # parked robots (waypoint reached, gate-stopped) produce no AMCL
        # updates, and that silence is legitimate.
        self.declare_parameter('watchdog_stationary_window_s', 1.0)
        self.declare_parameter('relocalize_settle_s', 1.0)

        config = GuardConfig(
            accept_translation_m=float(
                self.get_parameter('accept_translation_m').value),
            accept_yaw_deg=float(self.get_parameter('accept_yaw_deg').value),
            far_translation_m=float(
                self.get_parameter('far_translation_m').value),
            far_yaw_deg=float(self.get_parameter('far_yaw_deg').value),
            cluster_trans_m=float(
                self.get_parameter('cluster_trans_m').value),
            cluster_yaw_deg=float(
                self.get_parameter('cluster_yaw_deg').value),
            stable_window_s=float(
                self.get_parameter('stable_window_s').value),
            resume_samples=int(self.get_parameter('resume_samples').value),
            blend_rate=float(self.get_parameter('blend_rate').value),
        )
        self._guard = ContinuityGuard(config)
        self._map_frame = self.get_parameter('map_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._future_dating = Duration(
            seconds=float(self.get_parameter('future_dating_s').value))
        self._max_sigma_xy_m = float(
            self.get_parameter('max_sigma_xy_m').value)
        self._max_sigma_yaw_deg = float(
            self.get_parameter('max_sigma_yaw_deg').value)
        self._tf_max_deviation_ms = float(
            self.get_parameter('tf_max_deviation_ms').value)
        self._amcl_timeout = Duration(
            seconds=float(self.get_parameter('amcl_timeout_s').value))
        self._watchdog_stationary_translation = float(
            self.get_parameter('watchdog_stationary_translation_m').value)
        self._watchdog_stationary_yaw = math.radians(float(
            self.get_parameter('watchdog_stationary_yaw_deg').value))
        self._watchdog_stationary_window = float(
            self.get_parameter('watchdog_stationary_window_s').value)
        self._relocalize_settle = Duration(
            seconds=float(self.get_parameter('relocalize_settle_s').value))
        self._allow_autonomous_rebase = bool(
            self.get_parameter('allow_autonomous_rebase').value)
        self._last_candidate_at = None
        self._recent_odom = deque()
        self._stale = False
        self._pending_relocalization = None
        self._rebase_withheld = False
        self._status = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._broadcaster = TransformBroadcaster(self)

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_publisher = self.create_publisher(
            StringMessage, STATUS_TOPIC, latched)
        self._relocalization_event_publisher = self.create_publisher(
            PoseStamped, RELOCALIZATION_EVENT_TOPIC, latched)

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
        self._publish_status('guard starting')
        self.get_logger().info(
            'localization continuity guard ready: AMCL candidates are '
            'filtered before map->odom is published')

    def _current_status(self) -> str:
        if self._stale:
            return STATUS_STALE
        if self._pending_relocalization is not None:
            return STATUS_RELOCALIZATION_PENDING
        if self._guard.state == STATE_TRACKING:
            return STATUS_TRACKING
        return STATUS_HOLDING

    def _publish_status(self, reason: str) -> None:
        status = self._current_status()
        if status == self._status:
            return
        self.get_logger().info(
            f'guard status {self._status} -> {status}: {reason}')
        self._status = status
        message = StringMessage()
        message.data = status
        self._status_publisher.publish(message)

    def _lookup_odom_to_base(self, stamp):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                stamp,
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'odom->base_link lookup failed: {exc}; '
                'keeping the frozen estimate',
                throttle_duration_sec=5.0,
            )
            return None
        deviation_ms = abs(
            (Time.from_msg(transform.header.stamp) - Time.from_msg(stamp))
            .nanoseconds) * 1.0e-6
        if deviation_ms > self._tf_max_deviation_ms:
            self.get_logger().warn(
                f'odom->base_link transform is {deviation_ms:.1f}ms off the '
                f'AMCL stamp (limit {self._tf_max_deviation_ms:.1f}ms); '
                'keeping the frozen estimate',
                throttle_duration_sec=5.0,
            )
            return None
        return transform

    def _covariance_within_limits(self, msg) -> bool:
        sigma_xy, sigma_yaw_deg = _covariance_sigmas(msg)
        if (sigma_xy <= self._max_sigma_xy_m
                and sigma_yaw_deg <= self._max_sigma_yaw_deg):
            return True
        self.get_logger().warn(
            f'AMCL covariance too large: sigma_xy={sigma_xy:.3f}m '
            f'(limit {self._max_sigma_xy_m:.3f}m), '
            f'sigma_yaw={sigma_yaw_deg:.1f}deg '
            f'(limit {self._max_sigma_yaw_deg:.1f}deg); holding the '
            'frozen estimate',
            throttle_duration_sec=5.0,
        )
        return False

    def _on_candidate(self, msg) -> None:
        now = self.get_clock().now()
        self._last_candidate_at = now
        if self._stale:
            self._stale = False
            self._publish_status(
                'AMCL candidate stream recovered; resuming through the '
                'guard resume streak')
        if self._pending_relocalization is not None:
            # Frozen while a relocalization settles; the pending candidate
            # is applied by the publish timer.
            return
        odom_tf = self._lookup_odom_to_base(msg.header.stamp)
        if odom_tf is None:
            return
        if not self._covariance_within_limits(msg):
            self._guard.hold()
            self._publish_status('AMCL covariance exceeds guard limits')
            return
        candidate = _candidate_map_to_odom(msg, odom_tf)
        decision = self._guard.observe(candidate, now.nanoseconds * 1.0e-9)
        if decision == 'rebase' and self._rebase_withheld:
            # A far cluster was already reported and withheld once; stay
            # frozen instead of oscillating through PENDING again.
            self._guard.hold()
            decision = 'withheld'
        if decision == 'rebase':
            self._begin_relocalization(odom_tf, now)
        elif decision != 'accept':
            self.get_logger().info(
                f'guard decision={decision} state={self._guard.state} '
                f'candidate=({candidate.x:.3f},{candidate.y:.3f},'
                f'{math.degrees(candidate.yaw):.1f})')
        self._publish_status(f'guard decision={decision}')

    def _begin_relocalization(self, odom_tf, now) -> None:
        estimate = self._guard.cluster_mean
        self._pending_relocalization = {
            'estimate': estimate,
            'deadline': now + self._relocalize_settle,
        }
        map_x, map_y, map_yaw = _compose_map_to_base(estimate, odom_tf)
        event = PoseStamped()
        event.header.stamp = now.to_msg()
        event.header.frame_id = self._map_frame
        event.pose.position.x = map_x
        event.pose.position.y = map_y
        half_yaw = map_yaw * 0.5
        event.pose.orientation.z = math.sin(half_yaw)
        event.pose.orientation.w = math.cos(half_yaw)
        self._relocalization_event_publisher.publish(event)
        self.get_logger().warn(
            'far candidate cluster stable; relocalization pending '
            f'{self._relocalize_settle.nanoseconds * 1.0e-9:.2f}s settle: '
            f'map->base=({map_x:.3f},{map_y:.3f},'
            f'{math.degrees(map_yaw):.1f})')
        self._publish_status('stable far candidate cluster')

    def _accept_pending_relocalization(self) -> None:
        pending = self._pending_relocalization
        self._pending_relocalization = None
        if not self._allow_autonomous_rebase:
            # Freeze-only policy: the event was already published at PENDING
            # entry; keep the frozen estimate and report instead of rebasing
            # into a possibly-wrong far mode.
            self.get_logger().warn(
                'autonomous rebase disabled; keeping the frozen estimate '
                'after a stable far cluster '
                f'({pending["estimate"].x:.3f},{pending["estimate"].y:.3f},'
                f'{math.degrees(pending["estimate"].yaw):.1f})')
            self._publish_status(
                'rebase withheld: allow_autonomous_rebase=false')
            self._rebase_withheld = True
            self._guard.hold()
            return
        self._guard.apply_rebase(pending['estimate'])
        self.get_logger().warn(
            'relocalization settled; rebased map->odom to '
            f'({pending["estimate"].x:.3f},{pending["estimate"].y:.3f},'
            f'{math.degrees(pending["estimate"].yaw):.1f})')
        self._publish_status('relocalization settle elapsed; rebase applied')

    def _sample_odom_pose(self, now) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException:
            return
        translation = transform.transform.translation
        yaw = _yaw_from_quaternion(transform.transform.rotation)
        stamp_s = now.nanoseconds * 1.0e-9
        self._recent_odom.append((stamp_s, translation.x, translation.y, yaw))
        cutoff = stamp_s - self._watchdog_stationary_window
        while self._recent_odom and self._recent_odom[0][0] < cutoff:
            self._recent_odom.popleft()

    def _robot_recently_stationary(self) -> bool:
        if len(self._recent_odom) < 2:
            return False
        xs = [sample[1] for sample in self._recent_odom]
        ys = [sample[2] for sample in self._recent_odom]
        yaws = [sample[3] for sample in self._recent_odom]
        if max(xs) - min(xs) > self._watchdog_stationary_translation:
            return False
        if max(ys) - min(ys) > self._watchdog_stationary_translation:
            return False
        yaw_spread = max(
            abs(wrap_angle(a - b)) for a in yaws for b in yaws)
        return yaw_spread <= self._watchdog_stationary_yaw

    def _update_watchdog(self, now) -> None:
        if self._stale:
            if self._robot_recently_stationary():
                # A parked robot has a constant true map->odom; keeping the
                # frozen estimate flowing is exact, not future-dating.  As
                # soon as the odom window shows motion again without AMCL
                # candidates, the watchdog re-enters STALE below.
                self._stale = False
                self._publish_status(
                    'robot stationary; resuming frozen map->odom '
                    'republication while AMCL is quiet')
            return
        if self._last_candidate_at is None:
            return
        if now - self._last_candidate_at <= self._amcl_timeout:
            return
        if self._robot_recently_stationary():
            # AMCL publishes only past its motion-model thresholds, so a
            # recently stationary robot produces no candidates; map->odom is
            # invariant under dead reckoning and the frozen estimate stays
            # valid.  The decision uses the recent odom window (not motion
            # since the last candidate) so parking right after driving —
            # e.g. a completed waypoint — is also treated as stationary.
            self.get_logger().info(
                'AMCL candidate timeout ignored: robot stationary in the '
                'recent odom window',
                throttle_duration_sec=5.0,
            )
            return
        self._stale = True
        self._guard.hold()
        if self._pending_relocalization is not None:
            self._pending_relocalization = None
            self.get_logger().warn(
                'relocalization aborted: AMCL candidate stream timed out')
        self.get_logger().warn(
            'AMCL candidate stream timed out '
            f'({self._amcl_timeout.nanoseconds * 1.0e-9:.2f}s); '
            'map->odom republication stopped')
        self._publish_status('AMCL candidate timeout')

    def _on_reset(self, _msg) -> None:
        self._guard.reset()
        self._last_candidate_at = None
        self._recent_odom.clear()
        self._stale = False
        self._pending_relocalization = None
        self._rebase_withheld = False
        self.get_logger().info('simulation reset: guard state cleared')
        self._publish_status('simulation reset')

    def _publish(self) -> None:
        now = self.get_clock().now()
        self._sample_odom_pose(now)
        self._update_watchdog(now)
        if (self._pending_relocalization is not None
                and now >= self._pending_relocalization['deadline']):
            self._accept_pending_relocalization()
        estimate = self._guard.estimate
        if estimate is None or self._guard.state == STATE_INIT:
            return
        if self._stale:
            # Never future-date a stale estimate into a fresh-looking TF.
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
