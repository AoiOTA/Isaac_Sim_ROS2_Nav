"""Publish a simple map pose for the Attempt31 ideal-localization demo."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty


class OutdoorInitialPose(Node):
    def __init__(self) -> None:
        super().__init__("outdoor_initial_pose")
        self.x = float(self.declare_parameter("x", 0.0).value)
        self.y = float(self.declare_parameter("y", 0.0).value)
        self.yaw_deg = float(self.declare_parameter("yaw_deg", 0.0).value)
        self.publish_count = int(self.declare_parameter("publish_count", 5).value)
        self.period_s = float(self.declare_parameter("publish_period_s", 0.5).value)
        self.clock_ready = False
        self.scan_ready = False
        self.published = 0
        reliable_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE
        )
        clock_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", reliable_qos
        )
        # Isaac's /clock and the pointcloud-to-scan bridge are best-effort
        # publishers.  A default RELIABLE subscription is incompatible and
        # prevents the post-reset fresh-scan seed required by the runner.
        self.create_subscription(Clock, "/clock", self._on_clock, clock_qos)
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(
            Empty, "/simulation/reset_event", self._on_reset, reliable_qos
        )
        self.create_timer(self.period_s, self._tick)

    def _on_clock(self, message: Clock) -> None:
        self.clock_ready = bool(message.clock.sec or message.clock.nanosec)

    def _on_scan(self, _message: LaserScan) -> None:
        self.scan_ready = True

    def _on_reset(self, _message: Empty) -> None:
        self.scan_ready = False
        self.published = 0

    def _tick(self) -> None:
        if not self.clock_ready or not self.scan_ready or self.published >= self.publish_count:
            return
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        half = 0.5 * math.radians(self.yaw_deg)
        message.pose.pose.orientation.z = math.sin(half)
        message.pose.pose.orientation.w = math.cos(half)
        message.pose.covariance[0] = 0.05**2
        message.pose.covariance[7] = 0.05**2
        message.pose.covariance[35] = math.radians(2.0) ** 2
        self.publisher.publish(message)
        self.published += 1
        if self.published == self.publish_count:
            self.get_logger().info(
                f"published Rivermark initial pose ({self.x:.2f}, {self.y:.2f}, "
                f"{self.yaw_deg:.1f} deg)"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OutdoorInitialPose()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
