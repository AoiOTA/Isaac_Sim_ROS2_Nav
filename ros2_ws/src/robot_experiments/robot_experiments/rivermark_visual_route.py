"""Drive the five Rivermark waypoints for the one-terminal RViz demo."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import time

import yaml


@dataclass(frozen=True)
class VisualWaypoint:
    goal_id: str
    x: float
    y: float
    yaw_deg: float


def load_visual_route(path: str | Path) -> tuple[VisualWaypoint, ...]:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("frame_id") != "map":
        raise ValueError("Rivermark visual route requires frame_id=map")
    result = []
    for raw in payload.get("route", ()):
        if not isinstance(raw, dict):
            raise ValueError("Rivermark visual route entries must be mappings")
        position = raw.get("position")
        if not isinstance(position, (list, tuple)) or len(position) != 2:
            raise ValueError("Rivermark visual waypoint requires XY position")
        waypoint = VisualWaypoint(
            goal_id=str(raw.get("id", "")),
            x=float(position[0]),
            y=float(position[1]),
            yaw_deg=float(raw.get("yaw_deg", 0.0)),
        )
        if not waypoint.goal_id or not all(
            math.isfinite(value) for value in (waypoint.x, waypoint.y, waypoint.yaw_deg)
        ):
            raise ValueError("Rivermark visual waypoint must be named and finite")
        result.append(waypoint)
    if [item.goal_id for item in result] != ["G1", "G2", "G3", "G4", "G5"]:
        raise ValueError("Rivermark visual route must be exactly G1..G5")
    return tuple(result)


class RivermarkVisualRoute:
    def __init__(self, node, *, route, dynamic: bool, leg_timeout_s: float) -> None:
        from bio_nav_interfaces.msg import CanonicalRoute
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool

        self.node = node
        self.route = route
        self.dynamic = bool(dynamic)
        self.leg_timeout_s = float(leg_timeout_s)
        self.PoseStamped = PoseStamped
        self.route_epoch = 0
        self.completion_epoch = 0
        self.latest_complete = False
        self.trigger_clients = {}
        self.complete_clients = {}
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = node.create_publisher(
            PoseStamped, "/bio_nav/route_goal", reliable
        )
        node.create_subscription(
            CanonicalRoute,
            "/bio_nav/canonical_route",
            self._on_route,
            latched,
        )
        node.create_subscription(
            Bool,
            "/bio_nav/route_goal_complete",
            self._on_complete,
            reliable,
        )

    def _on_route(self, _message) -> None:
        self.route_epoch += 1

    def _on_complete(self, message) -> None:
        self.completion_epoch += 1
        self.latest_complete = bool(message.data)

    def _wait_until(self, predicate, timeout_s: float) -> bool:
        import rclpy

        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())),
            )
        return bool(predicate())

    def _call_obstacle_service(self, goal_id: str, action: str) -> None:
        import rclpy
        from std_srvs.srv import Trigger

        clients = self.trigger_clients if action == "trigger" else self.complete_clients
        client = clients.get(goal_id)
        if client is None:
            client = self.node.create_client(
                Trigger, f"/experiment/obstacles/{goal_id}/{action}"
            )
            clients[goal_id] = client
        if not client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError(f"dynamic obstacle {action} service missing for {goal_id}")
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError(f"dynamic obstacle {action} timed out for {goal_id}")
        response = future.result()
        if response is None or not response.success:
            detail = "no response" if response is None else response.message
            raise RuntimeError(
                f"dynamic obstacle {action} failed for {goal_id}: {detail}"
            )

    def _pose(self, waypoint: VisualWaypoint):
        message = self.PoseStamped()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = waypoint.x
        message.pose.position.y = waypoint.y
        yaw = math.radians(waypoint.yaw_deg)
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        return message

    def run(self) -> None:
        if not self._wait_until(
            lambda: self.publisher.get_subscription_count() > 0, 60.0
        ):
            raise RuntimeError("Rivermark route coordinator is unavailable")
        for index, waypoint in enumerate(self.route, start=1):
            route_epoch = self.route_epoch
            completion_epoch = self.completion_epoch
            self.node.get_logger().info(
                f"dispatching {waypoint.goal_id} ({index}/{len(self.route)})"
            )
            self.publisher.publish(self._pose(waypoint))
            if not self._wait_until(lambda: self.route_epoch > route_epoch, 30.0):
                raise TimeoutError(f"canonical route timeout for {waypoint.goal_id}")
            if self.dynamic and waypoint.goal_id in {"G2", "G3", "G4", "G5"}:
                self._call_obstacle_service(waypoint.goal_id, "trigger")
            if not self._wait_until(
                lambda: self.completion_epoch > completion_epoch,
                self.leg_timeout_s,
            ):
                raise TimeoutError(f"navigation timeout for {waypoint.goal_id}")
            if not self.latest_complete:
                raise RuntimeError(f"navigation failed for {waypoint.goal_id}")
            if self.dynamic and waypoint.goal_id in {"G2", "G3", "G4", "G5"}:
                self._call_obstacle_service(waypoint.goal_id, "complete")
            self.node.get_logger().info(f"completed {waypoint.goal_id}")


def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--dynamic", action="store_true")
    parser.add_argument("--leg-timeout-s", type=float, default=240.0)
    arguments, ros_arguments = parser.parse_known_args()
    if arguments.leg_timeout_s <= 0.0:
        parser.error("--leg-timeout-s must be positive")
    route = load_visual_route(arguments.config)
    rclpy.init(args=ros_arguments)
    node = rclpy.create_node("rivermark_five_waypoint_visual_route")
    runner = RivermarkVisualRoute(
        node,
        route=route,
        dynamic=arguments.dynamic,
        leg_timeout_s=arguments.leg_timeout_s,
    )
    try:
        runner.run()
    except (ExternalShutdownException, KeyboardInterrupt):
        # Ctrl+C shuts the complete one-terminal stack down together.  The
        # route runner should not turn that expected shutdown into a traceback.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
