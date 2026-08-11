"""Capture one live route-guided BT/Smac/MPPI data-plane snapshot."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from bio_nav_interfaces.msg import (
    CanonicalRoute,
    RouteProgress,
    RuntimeEdgeStateArray,
)


class ExecutionProbe(Node):
    def __init__(self) -> None:
        super().__init__("attempt30_a21_execution_probe")
        self.path = None
        self.lookahead = None
        self.progress = None
        self.route = None
        self.runtime_states = None
        self.commands = []
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(NavPath, "/plan", self._path, 10)
        self.create_subscription(
            PoseStamped, "/bio_nav/route_lookahead_goal", self._lookahead, 10
        )
        self.create_subscription(
            RouteProgress, "/bio_nav/route_progress", self._progress, 10
        )
        self.create_subscription(
            CanonicalRoute, "/bio_nav/canonical_route", self._route, latched
        )
        self.create_subscription(
            RuntimeEdgeStateArray,
            "/bio_nav/runtime_edge_states",
            self._runtime_states,
            latched,
        )
        self.create_subscription(Twist, "/cmd_vel_nav", self._command, 20)

    def _path(self, message) -> None:
        self.path = message

    def _lookahead(self, message) -> None:
        self.lookahead = message

    def _progress(self, message) -> None:
        self.progress = message

    def _route(self, message) -> None:
        self.route = message

    def _command(self, message) -> None:
        self.commands.append(
            [float(message.linear.x), float(message.angular.z)]
        )
        self.commands = self.commands[-100:]

    def _runtime_states(self, message) -> None:
        self.runtime_states = message

    def ready(self) -> bool:
        return all(
            item is not None
            for item in (self.path, self.lookahead, self.progress, self.route)
        ) and len(self.commands) >= 10

    def evidence(self) -> dict:
        points = [
            [
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                float(pose.pose.orientation.z),
                float(pose.pose.orientation.w),
            ]
            for pose in self.path.poses
        ]
        length = sum(
            math.dist(first[:2], second[:2])
            for first, second in zip(points, points[1:])
        )
        return {
            "classification": "engineering_evidence_not_qualification",
            "pipeline": [
                "Route Server",
                "route projection/lookahead",
                "native GoalUpdater BT",
                "SmacPlannerLattice",
                "MPPI FollowPath",
            ],
            "request_id": int(self.route.request_id),
            "route_edge_ids": [int(value) for value in self.route.edge_ids],
            "route_progress": {
                "edge_id": int(self.progress.edge_id),
                "arc_length_m": float(self.progress.arc_length_m),
                "remaining_m": float(self.progress.remaining_m),
                "lateral_error_m": float(self.progress.lateral_error_m),
            },
            "runtime_edge_states": [
                {
                    "edge_id": int(state.edge_id),
                    "state": int(state.state),
                    "penalty_m": float(state.penalty_m),
                    "consecutive_failures": int(state.consecutive_failures),
                }
                for state in (
                    [] if self.runtime_states is None else self.runtime_states.states
                )
            ],
            "start_xy_yaw": [points[0][0], points[0][1], 0.0],
            "goal_xy_yaw": [
                float(self.lookahead.pose.position.x),
                float(self.lookahead.pose.position.y),
                0.0,
            ],
            "plans": {
                "BT+SmacLattice+MPPI": {
                    "accepted": True,
                    "action_status": 2,
                    "error_code": 0,
                    "pose_count": len(points),
                    "path_length_m": length,
                    "poses_xy_zw": points,
                }
            },
            "cmd_vel_samples": self.commands,
            "nonzero_command_count": sum(
                abs(linear) > 1.0e-6 or abs(angular) > 1.0e-6
                for linear, angular in self.commands
            ),
            "maximum_abs_linear_x": max(
                (abs(value[0]) for value in self.commands), default=0.0
            ),
            "maximum_abs_angular_z": max(
                (abs(value[1]) for value in self.commands), default=0.0
            ),
        }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args(argv)
    rclpy.init()
    node = ExecutionProbe()
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline and not node.ready():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.ready():
        missing = [
            name
            for name, value in (
                ("path", node.path),
                ("lookahead", node.lookahead),
                ("progress", node.progress),
                ("route", node.route),
                ("cmd_vel", node.commands),
            )
            if not value
        ]
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"execution evidence timed out; missing {missing}")
    evidence = node.evidence()
    node.destroy_node()
    rclpy.shutdown()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    compact = {key: evidence[key] for key in (
        "request_id", "nonzero_command_count", "maximum_abs_linear_x",
        "maximum_abs_angular_z")}
    plan = evidence["plans"]["BT+SmacLattice+MPPI"]
    compact["pose_count"] = plan["pose_count"]
    compact["path_length_m"] = plan["path_length_m"]
    print(json.dumps(compact, sort_keys=True))


if __name__ == "__main__":
    main()
