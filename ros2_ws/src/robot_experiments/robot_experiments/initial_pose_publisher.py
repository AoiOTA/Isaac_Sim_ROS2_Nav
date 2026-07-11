"""Publish one calibrated map pose after simulation time and TF are ready."""

from __future__ import annotations

import math
import os
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformException, TransformListener

from .configuration import ConfigurationError
from .spawn_poses import load_spawn_pose


class InitialPosePublisher(Node):
    """State-machine publisher that never emits an uncalibrated initial pose."""

    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")
        configured_file = str(self.declare_parameter("spawn_poses_file", "").value).strip()
        self._spawn_poses_file = configured_file or os.environ.get(
            "ISAAC_NAV_SPAWN_POSES", ""
        ).strip()
        if not self._spawn_poses_file:
            raise ConfigurationError(
                "spawn_poses_file is required (or set ISAAC_NAV_SPAWN_POSES)"
            )
        pose_name = str(self.declare_parameter("spawn_pose_name", "mapping_start").value)
        self._pose = load_spawn_pose(
            self._spawn_poses_file,
            pose_name,
            require_calibrated=True,
        )

        self._topic = str(self.declare_parameter("initial_pose_topic", "/initialpose").value)
        self._odom_frame = str(self.declare_parameter("odom_frame", "odom").value)
        self._base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._publish_count = int(self.declare_parameter("publish_count", 5).value)
        self._publish_period_sec = float(
            self.declare_parameter("publish_period_sec", 0.5).value
        )
        self._clock_timeout_sec = float(
            self.declare_parameter("clock_timeout_sec", 30.0).value
        )
        self._tf_timeout_sec = float(self.declare_parameter("tf_timeout_sec", 30.0).value)
        self._wait_for_tf = bool(
            self.declare_parameter("wait_for_odom_to_base_tf", True).value
        )
        if self._publish_count < 1:
            raise ConfigurationError("publish_count must be at least one")
        if min(self._publish_period_sec, self._clock_timeout_sec, self._tf_timeout_sec) <= 0.0:
            raise ConfigurationError("publisher periods and timeouts must be positive")

        reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(PoseWithCovarianceStamped, self._topic, reliable)
        self._clock_subscription = self.create_subscription(
            Clock, "/clock", self._clock_callback, clock_qos
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._last_clock = None
        self._clock_ready_at: float | None = None
        self._started_at = time.monotonic()
        self._published = 0
        self.complete = False
        self.failure: str | None = None
        self._timer = self.create_timer(self._publish_period_sec, self._tick)

    def _clock_callback(self, message: Clock) -> None:
        if message.clock.sec != 0 or message.clock.nanosec != 0:
            self._last_clock = message.clock
            if self._clock_ready_at is None:
                self._clock_ready_at = time.monotonic()

    def _fail(self, reason: str) -> None:
        self.failure = reason
        self._timer.cancel()
        self.get_logger().error(reason)

    def _tf_ready(self) -> bool:
        if not self._wait_for_tf:
            return True
        try:
            self._tf_buffer.lookup_transform(self._odom_frame, self._base_frame, Time())
            return True
        except TransformException:
            return False

    def _tick(self) -> None:
        now = time.monotonic()
        if self._last_clock is None:
            if now - self._started_at >= self._clock_timeout_sec:
                self._fail("timed out waiting for a non-zero /clock")
            return
        if not self._tf_ready():
            assert self._clock_ready_at is not None
            if now - self._clock_ready_at >= self._tf_timeout_sec:
                self._fail(
                    f"timed out waiting for TF {self._odom_frame} -> {self._base_frame}"
                )
            return

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
        message.pose.covariance[35] = math.radians(self._pose.yaw_stddev_deg) ** 2
        self._publisher.publish(message)
        self._published += 1
        if self._published >= self._publish_count:
            self.complete = True
            self._timer.cancel()
            self.get_logger().info(
                f"published calibrated pose {self._pose.name!r} {self._published} times"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node: InitialPosePublisher | None = None
    try:
        node = InitialPosePublisher()
        while rclpy.ok() and not node.complete and node.failure is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.failure is not None:
            raise RuntimeError(node.failure)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
