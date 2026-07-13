"""Publish a calibrated map pose only after a fresh simulation epoch scan."""

from __future__ import annotations

import math
import os
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .configuration import ConfigurationError
from .spawn_poses import load_spawn_pose


class PostResetScanBarrier:
    """Accept scans only after an arm barrier and close to authoritative /clock."""

    def __init__(
        self,
        *,
        clock_tolerance_sec: float,
        rollback_tolerance_sec: float = 1.0e-6,
    ) -> None:
        if not math.isfinite(clock_tolerance_sec) or clock_tolerance_sec <= 0.0:
            raise ValueError("clock_tolerance_sec must be finite and positive")
        if (
            not math.isfinite(rollback_tolerance_sec)
            or rollback_tolerance_sec < 0.0
        ):
            raise ValueError(
                "rollback_tolerance_sec must be finite and non-negative"
            )
        self.clock_tolerance_sec = clock_tolerance_sec
        self.rollback_tolerance_sec = rollback_tolerance_sec
        self.barrier_stamp_s: float | None = None
        self.clock_stamp_s: float | None = None
        self.ready = False

    @staticmethod
    def _validate_stamp(stamp_s: float, label: str) -> None:
        if not math.isfinite(stamp_s) or stamp_s < 0.0:
            raise ValueError(f"{label} must be finite and non-negative")

    def arm(self, clock_stamp_s: float | None) -> None:
        if clock_stamp_s is not None:
            self._validate_stamp(clock_stamp_s, "clock stamp")
        self.barrier_stamp_s = clock_stamp_s
        self.clock_stamp_s = clock_stamp_s
        self.ready = False

    def observe_clock(self, clock_stamp_s: float) -> bool:
        self._validate_stamp(clock_stamp_s, "clock stamp")
        rolled_back = (
            self.clock_stamp_s is not None
            and clock_stamp_s + self.rollback_tolerance_sec < self.clock_stamp_s
        )
        self.clock_stamp_s = clock_stamp_s
        if self.barrier_stamp_s is None and clock_stamp_s > 0.0:
            self.barrier_stamp_s = clock_stamp_s
        return rolled_back

    def observe_scan(self, scan_stamp_s: float) -> bool:
        self._validate_stamp(scan_stamp_s, "scan stamp")
        if self.barrier_stamp_s is None or self.clock_stamp_s is None:
            return False
        if scan_stamp_s <= self.barrier_stamp_s:
            return False
        if abs(scan_stamp_s - self.clock_stamp_s) > self.clock_tolerance_sec:
            return False
        self.ready = True
        return True


class InitialPosePublisher(Node):
    """Finite auto publisher with reset recovery and manual-pose ownership."""

    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")
        configured_file = str(
            self.declare_parameter("spawn_poses_file", "").value
        ).strip()
        self._spawn_poses_file = configured_file or os.environ.get(
            "ISAAC_NAV_SPAWN_POSES", ""
        ).strip()
        if not self._spawn_poses_file:
            raise ConfigurationError(
                "spawn_poses_file is required (or set ISAAC_NAV_SPAWN_POSES)"
            )
        pose_name = str(
            self.declare_parameter("spawn_pose_name", "mapping_start").value
        )
        self._pose = load_spawn_pose(
            self._spawn_poses_file,
            pose_name,
            require_calibrated=True,
        )

        self._topic = str(
            self.declare_parameter("initial_pose_topic", "/initialpose").value
        )
        self._odom_frame = str(
            self.declare_parameter("odom_frame", "odom").value
        )
        self._base_frame = str(
            self.declare_parameter("base_frame", "base_link").value
        )
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._publish_count = int(
            self.declare_parameter("publish_count", 5).value
        )
        self._publish_period_sec = float(
            self.declare_parameter("publish_period_sec", 0.5).value
        )
        self._clock_timeout_sec = float(
            self.declare_parameter("clock_timeout_sec", 30.0).value
        )
        self._scan_timeout_sec = float(
            self.declare_parameter("scan_timeout_sec", 30.0).value
        )
        self._tf_timeout_sec = float(
            self.declare_parameter("tf_timeout_sec", 30.0).value
        )
        scan_clock_tolerance_sec = float(
            self.declare_parameter("scan_clock_tolerance_sec", 0.5).value
        )
        clock_rollback_tolerance_sec = float(
            self.declare_parameter(
                "clock_rollback_tolerance_sec", 1.0e-6
            ).value
        )
        self._wait_for_tf = bool(
            self.declare_parameter("wait_for_odom_to_base_tf", True).value
        )
        self._stay_alive = bool(
            self.declare_parameter("stay_alive_for_reseed", True).value
        )
        self._reseed_service_name = str(
            self.declare_parameter(
                "reseed_service", "/initial_pose/reseed"
            ).value
        )
        self._status_topic = str(
            self.declare_parameter(
                "status_topic", "/initial_pose/status"
            ).value
        )
        if self._publish_count < 1:
            raise ConfigurationError("publish_count must be at least one")
        periods = (
            self._publish_period_sec,
            self._clock_timeout_sec,
            self._scan_timeout_sec,
            self._tf_timeout_sec,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in periods):
            raise ConfigurationError("publisher periods and timeouts must be positive")
        try:
            self._scan_barrier = PostResetScanBarrier(
                clock_tolerance_sec=scan_clock_tolerance_sec,
                rollback_tolerance_sec=clock_rollback_tolerance_sec,
            )
        except ValueError as exc:
            raise ConfigurationError(str(exc)) from exc

        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, self._topic, reliable
        )
        self._status_publisher = self.create_publisher(
            String, self._status_topic, transient
        )
        self._initial_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self._topic,
            self._initial_pose_callback,
            reliable,
        )
        self._clock_subscription = self.create_subscription(
            Clock, "/clock", self._clock_callback, clock_qos
        )
        self._scan_subscription = self.create_subscription(
            LaserScan, "/scan", self._scan_callback, qos_profile_sensor_data
        )
        self._reset_event_subscription = self.create_subscription(
            Empty, "/simulation/reset_event", self._reset_event_callback, reliable
        )
        self._tf_buffer = Buffer(node=self)
        self._tf_listener = TransformListener(
            self._tf_buffer, self, spin_thread=False
        )
        self._last_clock = None
        self._last_clock_s: float | None = None
        self._clock_ready_at: float | None = None
        self._scan_ready_at: float | None = None
        self._started_at = time.monotonic()
        self._published = 0
        self._manual_override = False
        self.complete = False
        self.failure: str | None = None
        self._status = ""
        self._timer = self.create_timer(self._publish_period_sec, self._tick)
        self._reseed_service = self.create_service(
            Trigger,
            self._reseed_service_name,
            self._reseed_callback,
        )
        self._scan_barrier.arm(None)
        self._set_status("waiting_clock")

    def _set_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        message = String()
        message.data = status
        self._status_publisher.publish(message)

    def _clock_callback(self, message: Clock) -> None:
        stamp_s = message.clock.sec + message.clock.nanosec * 1.0e-9
        rolled_back = self._scan_barrier.observe_clock(stamp_s)
        self._last_clock_s = stamp_s
        if rolled_back:
            self._rearm(
                "simulation clock rollback",
                force_calibrated=False,
                barrier_stamp_s=stamp_s,
            )
        if message.clock.sec != 0 or message.clock.nanosec != 0:
            self._last_clock = message.clock
            if self._clock_ready_at is None:
                self._clock_ready_at = time.monotonic()
            if not self._manual_override and not self.complete:
                self._set_status("waiting_scan")

    def _scan_callback(self, message: LaserScan) -> None:
        stamp = message.header.stamp
        stamp_s = stamp.sec + stamp.nanosec * 1.0e-9
        if self._manual_override or self.complete:
            return
        if self._scan_barrier.observe_scan(stamp_s):
            if self._scan_ready_at is None:
                self._scan_ready_at = time.monotonic()
            self._set_status("waiting_tf")

    def _reset_event_callback(self, message: Empty) -> None:
        del message
        self._rearm(
            "physical simulation reset event",
            force_calibrated=True,
            barrier_stamp_s=self._last_clock_s,
        )

    def _reseed_callback(self, request, response):
        del request
        if self._manual_override:
            response.success = True
            response.message = (
                "manual RViz initial pose remains authoritative; "
                "automatic calibrated reseed was not armed"
            )
            self._set_status("manual_override")
            return response
        self._rearm(
            "reseed service request",
            force_calibrated=False,
            barrier_stamp_s=self._last_clock_s,
        )
        response.success = True
        response.message = (
            f"calibrated pose {self._pose.name!r} armed after a fresh /scan"
        )
        return response

    def _rearm(
        self,
        reason: str,
        *,
        force_calibrated: bool,
        barrier_stamp_s: float | None,
    ) -> bool:
        if self._manual_override and not force_calibrated:
            self.complete = True
            self.failure = None
            self._timer.cancel()
            self._set_status("manual_override")
            self.get_logger().info(
                f"initial pose not rearmed after {reason}: manual RViz pose retained"
            )
            return False
        if force_calibrated:
            self._manual_override = False
        self._published = 0
        self.complete = False
        self.failure = None
        now = time.monotonic()
        self._started_at = now
        self._clock_ready_at = now if self._last_clock is not None else None
        self._scan_ready_at = None
        self._scan_barrier.arm(barrier_stamp_s)
        self._tf_buffer.clear()
        self._timer.reset()
        self._set_status(
            "waiting_scan" if self._last_clock is not None else "waiting_clock"
        )
        self.get_logger().info(f"initial pose rearmed: {reason}")
        return True

    def _initial_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        if self._matches_calibrated_pose(message):
            return
        invalid_reason = self._manual_pose_error(message)
        if invalid_reason is not None:
            self.get_logger().warning(
                f"ignoring invalid external /initialpose: {invalid_reason}"
            )
            return
        self._manual_override = True
        self.complete = True
        self.failure = None
        self._timer.cancel()
        self._set_status("manual_override")
        self.get_logger().info(
            "external /initialpose accepted; stopping automatic publication"
        )

    def _manual_pose_error(
        self, message: PoseWithCovarianceStamped
    ) -> str | None:
        if message.header.frame_id != self._map_frame:
            return (
                f"frame_id must be {self._map_frame!r}; "
                f"got {message.header.frame_id!r}"
            )
        pose = message.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return "position and orientation must be finite"
        norm = math.sqrt(
            pose.orientation.x**2
            + pose.orientation.y**2
            + pose.orientation.z**2
            + pose.orientation.w**2
        )
        if not math.isclose(norm, 1.0, abs_tol=1.0e-3):
            return f"quaternion must be normalized; norm={norm:.6f}"
        return None

    def _matches_calibrated_pose(
        self, message: PoseWithCovarianceStamped
    ) -> bool:
        if message.header.frame_id != self._map_frame:
            return False
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = math.radians(self._pose.map.yaw_deg)
        expected_z = math.sin(yaw / 2.0)
        expected_w = math.cos(yaw / 2.0)
        return (
            math.isclose(
                position.x, self._pose.map.position[0], abs_tol=1.0e-9
            )
            and math.isclose(
                position.y, self._pose.map.position[1], abs_tol=1.0e-9
            )
            and math.isclose(orientation.x, 0.0, abs_tol=1.0e-9)
            and math.isclose(orientation.y, 0.0, abs_tol=1.0e-9)
            and math.isclose(orientation.z, expected_z, abs_tol=1.0e-9)
            and math.isclose(orientation.w, expected_w, abs_tol=1.0e-9)
        )

    def _fail(self, reason: str) -> None:
        if self.failure == reason:
            return
        self.failure = reason
        if not self._stay_alive:
            self._timer.cancel()
        self._set_status(f"failed:{reason}")
        self.get_logger().error(reason)

    def _clear_recoverable_failure(self) -> None:
        if self.failure is None:
            return
        self.get_logger().info(
            f"initial pose publisher recovered from: {self.failure}"
        )
        self.failure = None

    def _tf_ready(self) -> bool:
        if not self._wait_for_tf:
            return True
        try:
            self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time()
            )
            return True
        except TransformException:
            return False

    def _tick(self) -> None:
        if self._manual_override or self.complete:
            return
        now = time.monotonic()
        if self._last_clock is None:
            if now - self._started_at >= self._clock_timeout_sec:
                self._fail("timed out waiting for a non-zero /clock")
            return
        assert self._clock_ready_at is not None
        if not self._scan_barrier.ready:
            if now - self._clock_ready_at >= self._scan_timeout_sec:
                self._fail("timed out waiting for a fresh post-rearm /scan")
            return
        if not self._tf_ready():
            wait_started = self._scan_ready_at or self._clock_ready_at
            if now - wait_started >= self._tf_timeout_sec:
                self._fail(
                    f"timed out waiting for TF "
                    f"{self._odom_frame} -> {self._base_frame}"
                )
            return

        self._clear_recoverable_failure()
        self._set_status("publishing")
        message = PoseWithCovarianceStamped()
        message.header.frame_id = self._map_frame
        message.header.stamp = self._last_clock
        message.pose.pose.position.x = self._pose.map.position[0]
        message.pose.pose.position.y = self._pose.map.position[1]
        yaw = math.radians(self._pose.map.yaw_deg)
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = self._pose.position_stddev_m**2
        message.pose.covariance[7] = self._pose.position_stddev_m**2
        message.pose.covariance[35] = math.radians(
            self._pose.yaw_stddev_deg
        ) ** 2
        self._publisher.publish(message)
        self._published += 1
        if self._published >= self._publish_count:
            self.complete = True
            self._timer.cancel()
            self._set_status("complete")
            self.get_logger().info(
                f"published calibrated pose {self._pose.name!r} "
                f"{self._published} times"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: InitialPosePublisher | None = None
    try:
        node = InitialPosePublisher()
        if node._stay_alive:
            rclpy.spin(node)
        else:
            while rclpy.ok() and not node.complete and node.failure is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            if node.failure is not None:
                raise RuntimeError(node.failure)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
