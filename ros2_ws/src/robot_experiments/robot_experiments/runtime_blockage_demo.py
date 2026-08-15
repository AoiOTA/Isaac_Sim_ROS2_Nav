"""Explicitly inject a persistent runtime-edge observation for a research demo."""

from __future__ import annotations

import argparse

import rclpy
from bio_nav_interfaces.msg import RouteProgress, RuntimeEdgeObservation
from rclpy.node import Node


class RuntimeBlockageDemo(Node):
    def __init__(self, *, edge_id: int | None, state: str) -> None:
        super().__init__("rivermark_runtime_blockage_demo")
        self.state = state
        self.edge_id = edge_id
        self.publish_count = 0
        self.required_count = 4 if state == "blocked" else 2
        self.publisher = self.create_publisher(
            RuntimeEdgeObservation, "/bio_nav/runtime_edge_observation", 10
        )
        self.create_subscription(
            RouteProgress, "/bio_nav/route_progress", self._on_progress, 10
        )
        interval_s = 1.1 if state == "blocked" else 5.2
        self.timer = self.create_timer(interval_s, self._publish)
        self.get_logger().warning(
            "ENGINEERING INJECTION: waiting for a route edge; this is not a "
            "sensor-derived obstacle observation"
        )

    def _on_progress(self, message: RouteProgress) -> None:
        if self.edge_id is None:
            self.edge_id = int(message.edge_id)

    def _publish(self) -> None:
        if self.edge_id is None:
            return
        message = RuntimeEdgeObservation()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.edge_id = int(self.edge_id)
        message.planning_failed = self.state == "blocked"
        message.occupied_ahead = self.state == "blocked"
        message.observed_clear = self.state == "clear"
        self.publisher.publish(message)
        self.publish_count += 1
        self.get_logger().warning(
            f"ENGINEERING INJECTION: edge={self.edge_id} state={self.state} "
            f"sample={self.publish_count}/{self.required_count}"
        )
        if self.publish_count >= self.required_count:
            self.timer.cancel()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--edge-id", type=int)
    parser.add_argument("--state", choices=("blocked", "clear"), default="blocked")
    arguments, ros_arguments = parser.parse_known_args()
    rclpy.init(args=ros_arguments)
    node = RuntimeBlockageDemo(edge_id=arguments.edge_id, state=arguments.state)
    try:
        while rclpy.ok() and not node.timer.is_canceled():
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
