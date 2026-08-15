"""Call the configured Nav2 Smac planners and save compact path evidence."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def _pose(node: Node, x: float, y: float, yaw: float) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def _path_record(result) -> dict:
    points = [
        [
            float(pose.pose.position.x),
            float(pose.pose.position.y),
            float(pose.pose.orientation.z),
            float(pose.pose.orientation.w),
        ]
        for pose in result.path.poses
    ]
    length = sum(
        math.dist(first[:2], second[:2])
        for first, second in zip(points, points[1:])
    )
    return {
        "error_code": int(result.error_code),
        "error_msg": str(result.error_msg),
        "planning_time_s": float(
            result.planning_time.sec + result.planning_time.nanosec * 1e-9
        ),
        "pose_count": len(points),
        "path_length_m": length,
        "poses_xy_zw": points,
    }


def run_probe(
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    planner_ids: list[str],
) -> dict:
    rclpy.init()
    node = Node("attempt30_a21_smac_probe")
    client = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=10.0):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError("ComputePathToPose action is unavailable")

    plans = {}
    for planner_id in planner_ids:
        goal_message = ComputePathToPose.Goal()
        goal_message.start = _pose(node, *start)
        goal_message.goal = _pose(node, *goal)
        goal_message.planner_id = planner_id
        goal_message.use_start = True
        send_future = client.send_goal_async(goal_message)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            plans[planner_id] = {"accepted": False}
            continue
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=20.0)
        response = result_future.result()
        if response is None:
            plans[planner_id] = {"accepted": True, "timed_out": True}
            continue
        plans[planner_id] = {
            "accepted": True,
            "action_status": int(response.status),
            **_path_record(response.result),
        }

    node.destroy_node()
    rclpy.shutdown()
    return {
        "classification": "engineering_evidence_not_qualification",
        "action": "/compute_path_to_pose",
        "start_xy_yaw": list(start),
        "goal_xy_yaw": list(goal),
        "plans": plans,
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start", nargs=3, type=float, required=True)
    parser.add_argument("--goal", nargs=3, type=float, required=True)
    parser.add_argument(
        "--planners", nargs="+", default=["GridBased", "GridBaseline"]
    )
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    evidence = run_probe(tuple(args.start), tuple(args.goal), args.planners)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    compact = {
        planner: {
            key: record.get(key)
            for key in ("accepted", "action_status", "error_code", "pose_count", "path_length_m")
        }
        for planner, record in evidence["plans"].items()
    }
    print(json.dumps(compact, sort_keys=True))


if __name__ == "__main__":
    main()
