"""Run and record one real Isaac A21 route-guided closed loop."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from bio_nav_interfaces.msg import CanonicalRoute, NavigationGraph, RouteProgress

from .defaults import load_engineering_defaults
from .feasibility import _polygon_is_free, apply_footprint_feasibility
from .gvg import build_gvg
from .map_io import load_occupancy_map
from .route_ab_visualize import _route_xy
from .visualize import _map_background, _plot_graph


def _yaw(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


class ClosedLoopProbe(Node):
    def __init__(self, goal: tuple[float, float, float]) -> None:
        super().__init__("attempt30_a21_closed_loop_probe")
        self.goal = goal
        self.graph_message = None
        self.route = None
        self.progress = None
        self.last_plan = None
        self.completed = False
        self.failed = False
        self.odometry = []
        self.lookaheads = []
        self.commands = []
        self.plan_count = 0
        self.turn_plan_snapshots = []
        self.route_history = []
        self.obstacle_states = []
        self.physical_collision = False
        self.scan_min_ranges = []
        self.goal_pub = self.create_publisher(
            PoseStamped, "/bio_nav/route_goal", 10
        )
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            NavigationGraph, "/bio_nav/navigation_graph", self._graph, latched
        )
        self.create_subscription(
            CanonicalRoute, "/bio_nav/canonical_route", self._route, latched
        )
        self.create_subscription(
            RouteProgress, "/bio_nav/route_progress", self._progress, 20
        )
        self.create_subscription(
            Odometry, "/ground_truth/odom", self._odometry, 100
        )
        self.create_subscription(NavPath, "/plan", self._plan, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self._command, 50)
        self.create_subscription(
            Bool, "/bio_nav/route_goal_complete", self._complete, 10
        )
        self.create_subscription(
            String, "/experiment/obstacles/state", self._obstacle_state, 20
        )
        self.create_subscription(
            Bool, "/simulation/collision", self._collision, 20
        )
        self.create_subscription(
            LaserScan, "/scan_safety", self._scan, qos_profile_sensor_data
        )

    def _graph(self, message) -> None:
        self.graph_message = message

    def _route(self, message) -> None:
        self.route = message
        record = {
            "request_id": int(message.request_id),
            "graph_revision": int(message.graph_revision),
            "edge_ids": [int(value) for value in message.edge_ids],
            "trajectory_index": len(self.odometry),
        }
        if not self.route_history or record != self.route_history[-1]:
            self.route_history.append(record)

    def _progress(self, message) -> None:
        self.progress = message
        pose = message.lookahead_goal.pose
        self.lookaheads.append(
            [float(pose.position.x), float(pose.position.y)]
        )
        self.lookaheads = self.lookaheads[-3000:]

    def _odometry(self, message) -> None:
        pose = message.pose.pose
        self.odometry.append(
            [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ]
        )
        self.odometry = self.odometry[-20000:]

    def _plan(self, message) -> None:
        self.last_plan = message
        self.plan_count += 1
        if self.odometry and 3.2 <= self.odometry[-1][1] <= 4.5:
            self.turn_plan_snapshots.append({
                "robot_xy_zw": self.odometry[-1],
                "path_xy": [
                    [float(pose.pose.position.x), float(pose.pose.position.y)]
                    for pose in message.poses
                ],
                "path_xy_zw": [
                    [
                        float(pose.pose.position.x),
                        float(pose.pose.position.y),
                        float(pose.pose.orientation.z),
                        float(pose.pose.orientation.w),
                    ]
                    for pose in message.poses
                ],
            })
            self.turn_plan_snapshots = self.turn_plan_snapshots[-100:]

    def _command(self, message) -> None:
        self.commands.append(
            [float(message.linear.x), float(message.angular.z)]
        )
        self.commands = self.commands[-20000:]

    def _complete(self, message) -> None:
        if message.data:
            self.completed = True
        else:
            self.failed = True

    def _obstacle_state(self, message) -> None:
        try:
            state = json.loads(message.data)
        except json.JSONDecodeError:
            return
        state["trajectory_index"] = len(self.odometry)
        signature = [
            (item.get("id"), item.get("state"), item.get("progress"))
            for item in state.get("obstacles", [])
        ]
        if self.obstacle_states:
            previous = self.obstacle_states[-1]
            previous_signature = [
                (item.get("id"), item.get("state"), item.get("progress"))
                for item in previous.get("obstacles", [])
            ]
            if signature == previous_signature:
                return
        self.obstacle_states.append(state)
        self.obstacle_states = self.obstacle_states[-4000:]

    def _collision(self, message) -> None:
        self.physical_collision = self.physical_collision or bool(message.data)

    def _scan(self, message) -> None:
        valid = [
            float(value) for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        if valid:
            self.scan_min_ranges.append(min(valid))
            self.scan_min_ranges = self.scan_min_ranges[-20000:]

    def ready(self) -> bool:
        return self.graph_message is not None and bool(self.odometry)

    def publish_goal(self) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = self.goal[0]
        message.pose.position.y = self.goal[1]
        message.pose.orientation.z = math.sin(self.goal[2] / 2.0)
        message.pose.orientation.w = math.cos(self.goal[2] / 2.0)
        self.goal_pub.publish(message)

    def trigger_obstacle_group(self, group: str) -> str:
        client = self.create_client(
            Trigger, f"/experiment/obstacles/{group}/trigger"
        )
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"obstacle trigger service for {group} is unavailable")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(
                f"obstacle trigger for {group} failed: "
                f"{'' if response is None else response.message}"
            )
        return str(response.message)


def _distance_to_polyline(point: np.ndarray, polyline: np.ndarray) -> float:
    best = math.inf
    for start, end in zip(polyline, polyline[1:]):
        segment = end - start
        length2 = float(np.dot(segment, segment))
        fraction = 0.0 if length2 <= 0.0 else float(
            np.clip(np.dot(point - start, segment) / length2, 0.0, 1.0)
        )
        best = min(best, float(np.linalg.norm(point - (start + fraction * segment))))
    return best


def _export(
    node: ClosedLoopProbe,
    map_path: Path,
    defaults_path: Path,
    output_json: Path,
    output_image: Path,
    elapsed_s: float,
) -> dict:
    defaults = load_engineering_defaults(defaults_path)
    occupancy = load_occupancy_map(
        map_path,
        unknown_is_occupied=bool(defaults["graph"]["unknown_is_occupied"]),
    )
    graph = apply_footprint_feasibility(
        build_gvg(
            occupancy,
            defaults["graph"],
            defaults["footprint"],
            defaults["route_cost"],
        ),
        occupancy,
        defaults["footprint"],
    )
    trajectory = np.asarray(node.odometry, dtype=float)
    route_edges = [] if node.route is None else [int(value) for value in node.route.edge_ids]
    route_xy = _route_xy(graph, route_edges) if route_edges else np.empty((0, 2))
    path_xy = np.asarray(
        [] if node.last_plan is None else [
            [pose.pose.position.x, pose.pose.position.y]
            for pose in node.last_plan.poses
        ],
        dtype=float,
    )
    footprint = np.asarray(defaults["footprint"]["polygon_m"], dtype=float)
    radial = np.linalg.norm(footprint, axis=1)
    footprint += (
        float(defaults["footprint"]["padding_m"])
        * footprint
        / radial[:, None]
    )
    sample_step = max(1, len(trajectory) // 1000)
    collision_samples = []
    for index in range(0, len(trajectory), sample_step):
        x, y, z, w = trajectory[index]
        if not _polygon_is_free(occupancy, x, y, _yaw(z, w), footprint):
            collision_samples.append(
                {
                    "trajectory_index": index,
                    "x": float(x),
                    "y": float(y),
                    "yaw": _yaw(z, w),
                }
            )
    plan_collision_count = 0
    for snapshot in node.turn_plan_snapshots:
        snapshot_collisions = 0
        for x, y, z, w in snapshot["path_xy_zw"]:
            if not _polygon_is_free(occupancy, x, y, _yaw(z, w), footprint):
                snapshot_collisions += 1
        snapshot["sampled_footprint_collisions"] = snapshot_collisions
        plan_collision_count += snapshot_collisions
    travelled = sum(
        math.dist(first[:2], second[:2])
        for first, second in zip(trajectory, trajectory[1:])
    )
    goal_error = (
        math.dist(trajectory[-1, :2], node.goal[:2]) if len(trajectory) else math.inf
    )
    route_deviation = (
        np.asarray([
            _distance_to_polyline(point[:2], route_xy) for point in trajectory
        ], dtype=float)
        if len(trajectory) and len(route_xy) > 1 else np.empty(0)
    )
    actor_track = []
    actor_states_seen = []
    actor_min_clearance = math.inf
    for state in node.obstacle_states:
        for obstacle in state.get("obstacles", []):
            if obstacle.get("state") in {"waiting", "retired"}:
                continue
            position = obstacle.get("position", [])
            if len(position) >= 2:
                actor_track.append([
                    float(position[0]), float(position[1]),
                    int(state["trajectory_index"]),
                ])
            actor_states_seen.append(str(obstacle.get("state")))
            clearance = obstacle.get("min_clearance_m")
            if clearance is not None:
                actor_min_clearance = min(actor_min_clearance, float(clearance))
    actor_indices = [item[2] for item in actor_track]
    obstacle_end_index = max(actor_indices) if actor_indices else 0
    evidence = {
        "classification": "engineering_evidence_not_qualification",
        "completed": node.completed,
        "failed": node.failed,
        "elapsed_s": elapsed_s,
        "goal_xy_yaw": list(node.goal),
        "route_edge_ids": route_edges,
        "route_graph_revision": (
            None if node.route is None else int(node.route.graph_revision)
        ),
        "route_history": node.route_history,
        "odometry_samples": len(trajectory),
        "travelled_distance_m": travelled,
        "final_goal_error_m": goal_error,
        "plan_count": node.plan_count,
        "last_plan_pose_count": len(path_xy),
        "lookahead_samples": len(node.lookaheads),
        "command_samples": len(node.commands),
        "nonzero_command_samples": sum(
            abs(linear) > 1.0e-6 or abs(angular) > 1.0e-6
            for linear, angular in node.commands
        ),
        "sampled_static_footprint_collisions": len(collision_samples),
        "sampled_turn_plan_footprint_collisions": plan_collision_count,
        "physical_collision": node.physical_collision,
        "obstacle_states_seen": sorted(set(actor_states_seen)),
        "obstacle_state_sample_count": len(node.obstacle_states),
        "obstacle_actor_track_xy_index": actor_track,
        "obstacle_min_clearance_m": (
            None if math.isinf(actor_min_clearance) else actor_min_clearance
        ),
        "scan_min_range_m": (
            None if not node.scan_min_ranges else min(node.scan_min_ranges)
        ),
        "max_route_deviation_m": (
            None if not len(route_deviation) else float(route_deviation.max())
        ),
        "post_obstacle_route_deviation_m": (
            None if obstacle_end_index >= len(route_deviation) else
            float(route_deviation[obstacle_end_index:].min())
        ),
        "static_footprint_collision_samples": collision_samples,
        "trajectory_xy_zw": trajectory.tolist(),
        "last_smac_path_xy": path_xy.tolist(),
        "turn_plan_snapshots": node.turn_plan_snapshots,
        "lookahead_xy": node.lookaheads,
    }
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")

    figure, axes = plt.subplots(1, 2, figsize=(17, 9), constrained_layout=True)
    for axis in axes:
        _map_background(axis, occupancy)
        _plot_graph(axis, graph, alpha=0.14, linewidth=0.4)
        if len(route_xy):
            axis.plot(
                route_xy[:, 0], route_xy[:, 1], color="#2962ff",
                linewidth=2.0, label="canonical Route", zorder=7,
            )
        if len(trajectory):
            axis.plot(
                trajectory[:, 0], trajectory[:, 1], color="#ff6d00",
                linewidth=1.8, label="Isaac ground-truth trajectory", zorder=8,
            )
        if actor_track:
            actor_xy = np.asarray(actor_track, dtype=float)
            axis.plot(
                actor_xy[:, 0], actor_xy[:, 1], color="#d500f9",
                linewidth=2.3, marker="o", markersize=2.0,
                label="dynamic actor track", zorder=12,
            )
        if collision_samples:
            collision_xy = np.asarray(
                [[sample["x"], sample["y"]] for sample in collision_samples]
            )
            axis.scatter(
                collision_xy[:, 0], collision_xy[:, 1], marker="x", s=28,
                linewidths=1.3, c="#d50000", label="footprint collision sample",
                zorder=11,
            )
        if len(path_xy):
            axis.plot(
                path_xy[:, 0], path_xy[:, 1], color="#00c853",
                linewidth=1.7, label="last Smac path", zorder=9,
            )
        for snapshot_index, snapshot in enumerate(node.turn_plan_snapshots):
            snapshot_xy = np.asarray(snapshot["path_xy"], dtype=float)
            if len(snapshot_xy):
                axis.plot(
                    snapshot_xy[:, 0], snapshot_xy[:, 1], color="#00c853",
                    linewidth=0.65, alpha=0.22,
                    label=("Smac turn-plan snapshots"
                           if snapshot_index == 0 else None),
                    zorder=8,
                )
        if node.lookaheads:
            lookahead = np.asarray(node.lookaheads)
            axis.scatter(
                lookahead[::max(1, len(lookahead) // 30), 0],
                lookahead[::max(1, len(lookahead) // 30), 1],
                s=12, c="#e040fb", label="lookahead samples", zorder=9,
            )
        axis.scatter(
            node.goal[0], node.goal[1], marker="*", s=120, c="#d50000",
            edgecolors="black", label="mission goal", zorder=10,
        )
    axes[0].set_title("A21 real Isaac closed loop in full map context")
    if collision_samples:
        collision_xy = np.asarray(
            [[sample["x"], sample["y"]] for sample in collision_samples]
        )
        margin = 0.65
        axes[1].set_xlim(collision_xy[:, 0].min() - margin,
                         collision_xy[:, 0].max() + margin)
        axes[1].set_ylim(collision_xy[:, 1].min() - margin,
                         collision_xy[:, 1].max() + margin)
        footprint_stride = max(1, len(collision_samples) // 8)
        for sample in collision_samples[::footprint_stride]:
            yaw = sample["yaw"]
            rotation = np.asarray([
                [math.cos(yaw), -math.sin(yaw)],
                [math.sin(yaw), math.cos(yaw)],
            ])
            polygon = footprint @ rotation.T
            polygon[:, 0] += sample["x"]
            polygon[:, 1] += sample["y"]
            polygon = np.vstack([polygon, polygon[0]])
            axes[1].plot(
                polygon[:, 0], polygon[:, 1], color="#d50000",
                linewidth=0.8, alpha=0.65, zorder=10,
            )
        axes[1].set_title("Edge-28 collision detail with padded footprints")
    elif actor_track:
        actor_xy = np.asarray(actor_track, dtype=float)
        margin = 1.0
        axes[1].set_xlim(actor_xy[:, 0].min() - margin,
                         actor_xy[:, 0].max() + margin)
        axes[1].set_ylim(actor_xy[:, 1].min() - margin,
                         actor_xy[:, 1].max() + margin)
        axes[1].set_title("Temporary actor / Route / Smac / trajectory detail")
    elif len(trajectory):
        margin = 1.0
        axes[1].set_xlim(trajectory[-1, 0] - margin, trajectory[-1, 0] + margin)
        axes[1].set_ylim(trajectory[-1, 1] - margin, trajectory[-1, 1] + margin)
        axes[1].set_title("Final Route / Smac / trajectory alignment")
    axes[1].legend(loc="upper right", fontsize=8)
    output_image.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_image, dpi=180)
    plt.close(figure)
    return evidence


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", nargs=3, type=float, required=True)
    parser.add_argument("--map", required=True, dest="map_path")
    parser.add_argument("--defaults", required=True, dest="defaults_path")
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-image", required=True)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--obstacle-group")
    args = parser.parse_args(argv)
    rclpy.init()
    node = ClosedLoopProbe(tuple(args.goal))
    readiness_deadline = time.monotonic() + 30.0
    while rclpy.ok() and time.monotonic() < readiness_deadline and not node.ready():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.ready():
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("graph/ground-truth odometry did not become ready")
    previous_request = 0 if node.route is None else int(node.route.request_id)
    trigger_response = None
    if args.obstacle_group:
        trigger_response = node.trigger_obstacle_group(args.obstacle_group)
    request_deadline = time.monotonic() + 5.0
    next_publish = 0.0
    while rclpy.ok() and time.monotonic() < request_deadline:
        now = time.monotonic()
        if now >= next_publish:
            node.publish_goal()
            next_publish = now + 1.0
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.route is not None and int(node.route.request_id) > previous_request:
            break
    if node.route is None or int(node.route.request_id) <= previous_request:
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("route request was not observed after goal publication")
    start = time.monotonic()
    while (
        rclpy.ok()
        and time.monotonic() - start < args.timeout
        and not node.completed
        and not node.failed
    ):
        rclpy.spin_once(node, timeout_sec=0.1)
    evidence = _export(
        node,
        Path(args.map_path),
        Path(args.defaults_path),
        Path(args.output_json),
        Path(args.output_image),
        time.monotonic() - start,
    )
    if trigger_response is not None:
        evidence["obstacle_trigger_group"] = args.obstacle_group
        evidence["obstacle_trigger_response"] = trigger_response
        Path(args.output_json).write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n"
        )
    node.destroy_node()
    rclpy.shutdown()
    compact = {key: evidence[key] for key in (
        "completed", "failed", "elapsed_s", "travelled_distance_m",
        "final_goal_error_m", "plan_count", "command_samples",
        "sampled_static_footprint_collisions", "physical_collision")}
    print(json.dumps(compact, sort_keys=True))


if __name__ == "__main__":
    main()
