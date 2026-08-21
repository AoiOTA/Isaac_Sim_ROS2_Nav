"""Read-only live LiDAR readiness gate for the V6 flat20 IMU session."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct
import time
from typing import Any, Sequence

import yaml


CONFIG_NAME = "v6_imu_lidar_readiness.yaml"


@dataclass
class TopicState:
    count: int = 0
    last_stamp_s: float | None = None
    stamp_violation: bool = False
    finite_return_seen: bool = False


def load_contract(path: Path | None = None) -> tuple[Path, dict[str, Any]]:
    if path is None:
        from ament_index_python.packages import get_package_share_directory

        path = (
            Path(get_package_share_directory("robot_experiments"))
            / "config"
            / CONFIG_NAME
        )
    path = path.expanduser().resolve()
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    expected_topics = {
        "/lidar/points_raw": "sensor_msgs/msg/PointCloud2",
        "/lidar/points_scan": "sensor_msgs/msg/PointCloud2",
        "/scan": "sensor_msgs/msg/LaserScan",
        "/scan_safety": "sensor_msgs/msg/LaserScan",
    }
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != 1
        or document.get("contract") != "v6_imu_flat20_lidar_readiness_v1"
        or document.get("topics") != expected_topics
        or document.get("min_messages") != 2
        or document.get("max_age_sec") != 0.4
        or document.get("strict_header_stamps") is not True
        or document.get("require_finite_returns") is not True
        or not isinstance(document.get("timeout_sec"), (int, float))
        or isinstance(document.get("timeout_sec"), bool)
        or float(document["timeout_sec"]) <= 0.0
    ):
        raise ValueError("V6 IMU LiDAR readiness contract mismatch")
    return path, document


def _stamp_s(message: Any) -> float:
    stamp = message.header.stamp
    value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError("invalid header stamp")
    return value


def _pointcloud_has_finite_xyz(message: Any) -> bool:
    fields = {field.name: field for field in message.fields}
    if not {"x", "y", "z"}.issubset(fields) or int(message.point_step) <= 0:
        return False
    formats = {7: "f", 8: "d"}
    selected = []
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype not in formats or field.count != 1:
            return False
        selected.append((int(field.offset), formats[field.datatype]))
    prefix = ">" if message.is_bigendian else "<"
    point_step = int(message.point_step)
    data = memoryview(message.data)
    count = min(int(message.width) * int(message.height), len(data) // point_step)
    for index in range(count):
        base = index * point_step
        try:
            xyz = [
                struct.unpack_from(prefix + fmt, data, base + offset)[0]
                for offset, fmt in selected
            ]
        except (struct.error, ValueError):
            return False
        if all(math.isfinite(float(value)) for value in xyz):
            return True
    return False


def _scan_has_finite_range(message: Any) -> bool:
    return any(math.isfinite(float(value)) for value in message.ranges)


def observe(state: TopicState, message: Any, *, pointcloud: bool) -> None:
    try:
        stamp_s = _stamp_s(message)
    except (AttributeError, TypeError, ValueError):
        state.stamp_violation = True
        return
    if state.last_stamp_s is not None and stamp_s <= state.last_stamp_s:
        state.stamp_violation = True
    state.last_stamp_s = stamp_s
    state.count += 1
    state.finite_return_seen = state.finite_return_seen or (
        _pointcloud_has_finite_xyz(message)
        if pointcloud
        else _scan_has_finite_range(message)
    )


def evaluate(
    states: dict[str, TopicState], *, now_s: float, min_messages: int, max_age_sec: float
) -> dict[str, Any]:
    topics = {}
    for topic, state in sorted(states.items()):
        age = None if state.last_stamp_s is None else now_s - state.last_stamp_s
        ready = (
            state.count >= min_messages
            and not state.stamp_violation
            and state.finite_return_seen
            and age is not None
            and 0.0 <= age < max_age_sec
        )
        topics[topic] = {
            "ready": ready,
            "message_count": state.count,
            "last_stamp_s": state.last_stamp_s,
            "age_sec": age,
            "strict_stamps": not state.stamp_violation,
            "finite_return_seen": state.finite_return_seen,
        }
    return {"verdict": "PASS" if all(item["ready"] for item in topics.values()) else "STOP", "topics": topics}


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    config_path, contract = load_contract(args.config)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2

    rclpy.init(args=None)
    node = rclpy.create_node("v6_imu_lidar_preflight")
    states = {topic: TopicState() for topic in contract["topics"]}
    subscriptions = []
    for topic, message_type in contract["topics"].items():
        pointcloud = message_type.endswith("PointCloud2")
        subscriptions.append(node.create_subscription(
            PointCloud2 if pointcloud else LaserScan,
            topic,
            lambda message, topic=topic, pointcloud=pointcloud: observe(
                states[topic], message, pointcloud=pointcloud
            ),
            qos_profile_sensor_data,
        ))
    deadline = time.monotonic() + float(contract["timeout_sec"])
    result: dict[str, Any]
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now_s = node.get_clock().now().nanoseconds * 1.0e-9
            result = evaluate(
                states,
                now_s=now_s,
                min_messages=int(contract["min_messages"]),
                max_age_sec=float(contract["max_age_sec"]),
            )
            if result["verdict"] == "PASS":
                break
        else:
            now_s = node.get_clock().now().nanoseconds * 1.0e-9
            result = evaluate(
                states,
                now_s=now_s,
                min_messages=int(contract["min_messages"]),
                max_age_sec=float(contract["max_age_sec"]),
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    result["contract"] = contract["contract"]
    result["config"] = str(config_path)
    payload = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if result["verdict"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
