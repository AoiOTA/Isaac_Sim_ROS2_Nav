#!/usr/bin/env python3
"""Produce a receipt-bound, read-only collision timeline from a ROS 2 MCAP.

The experiment's ``/simulation/collision`` topic deliberately carries only a
``std_msgs/Bool``.  This tool preserves that evidence boundary: it reports the
first true sample and nearby navigation observations, but never guesses which
prim made contact.  Actor state is included only as contemporaneous context.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


SCHEMA = "bio_nav_collision_forensics_v1"
TOPICS = {
    "/simulation/collision",
    "/cmd_vel",
    "/odom",
    "/ground_truth/odom",
    "/scan",
    "/collision_monitor_state",
}


@dataclass(frozen=True)
class Sample:
    stamp_s: float
    value: dict[str, Any]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite(value: float) -> float | None:
    return value if math.isfinite(value) else None


def pose_value(message: Any) -> dict[str, float | None]:
    pose = message.pose.pose
    orientation = pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return {
        "x": finite(float(pose.position.x)),
        "y": finite(float(pose.position.y)),
        "yaw_rad": finite(yaw),
        "linear_speed_mps": finite(float(message.twist.twist.linear.x)),
        "angular_speed_radps": finite(float(message.twist.twist.angular.z)),
    }


def scan_value(message: Any) -> dict[str, float | int | None]:
    ranges = [float(item) for item in message.ranges]
    valid = [item for item in ranges if math.isfinite(item) and item >= float(message.range_min)]
    return {
        "valid_ranges": len(valid),
        "minimum_range_m": finite(min(valid)) if valid else None,
    }


def sample_window(samples: Iterable[Sample], center_s: float, window_s: float) -> list[dict[str, Any]]:
    return [
        {"stamp_s": round(sample.stamp_s, 9), **sample.value}
        for sample in samples
        if abs(sample.stamp_s - center_s) <= window_s
    ]


def parse_dynamic_rows(path: Path, center_s: float, window_s: float) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            stamp_s = float(row["stamp_s"])
            if abs(stamp_s - center_s) <= window_s:
                rows.append(
                    {
                        "stamp_s": stamp_s,
                        "id": row["id"],
                        "state": row["state"],
                        "position": json.loads(row["position"]),
                        "velocity_mps": float(row["velocity_mps"]),
                        "progress": float(row["progress"]),
                        "min_clearance_m": float(row["min_clearance_m"])
                        if row["min_clearance_m"]
                        else None,
                    }
                )
    return rows


def parse_events(path: Path, center_s: float, window_s: float) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        value = json.loads(line)
        stamp_s = value.get("simulation_time")
        if isinstance(stamp_s, (int, float)) and abs(float(stamp_s) - center_s) <= window_s:
            result.append(value)
    return result


def read_bag(path: Path) -> tuple[dict[str, list[Sample]], list[Sample], int]:
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:  # pragma: no cover - depends on ROS installation
        raise RuntimeError("ROS 2 Jazzy rosbag2 Python support is required") from error

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(path.parent), storage_id="mcap"), ConverterOptions("", ""))
    type_by_topic = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = TOPICS - set(type_by_topic)
    if missing:
        raise ValueError(f"MCAP is missing required topics: {sorted(missing)}")

    samples = {topic: [] for topic in TOPICS if topic != "/simulation/collision"}
    collisions: list[Sample] = []
    true_count = 0
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in TOPICS:
            continue
        message = deserialize_message(raw, get_message(type_by_topic[topic]))
        stamp_s = float(timestamp_ns) / 1.0e9
        if topic == "/simulation/collision":
            value = bool(message.data)
            collisions.append(Sample(stamp_s, {"data": value}))
            true_count += int(value)
        elif topic == "/cmd_vel":
            samples[topic].append(
                Sample(
                    stamp_s,
                    {
                        "linear_speed_mps": finite(float(message.linear.x)),
                        "angular_speed_radps": finite(float(message.angular.z)),
                    },
                )
            )
        elif topic in {"/odom", "/ground_truth/odom"}:
            samples[topic].append(Sample(stamp_s, pose_value(message)))
        elif topic == "/scan":
            samples[topic].append(Sample(stamp_s, scan_value(message)))
        elif topic == "/collision_monitor_state":
            samples[topic].append(
                Sample(
                    stamp_s,
                    {
                        "action_type": int(message.action_type),
                        "polygon_name": str(message.polygon_name),
                    },
                )
            )
    return samples, collisions, true_count


def analyze(bag: Path, events: Path, dynamic_obstacles: Path, window_s: float) -> dict[str, Any]:
    samples, collisions, true_count = read_bag(bag)
    first_true = next((sample for sample in collisions if sample.value["data"]), None)
    result: dict[str, Any] = {
        "schema": SCHEMA,
        "inputs": {
            "bag": {"path": str(bag), "sha256": sha256(bag)},
            "events": {"path": str(events), "sha256": sha256(events)},
            "dynamic_obstacles": {"path": str(dynamic_obstacles), "sha256": sha256(dynamic_obstacles)},
        },
        "collision_topic": {
            "sample_count": len(collisions),
            "true_count": true_count,
            "first_true_stamp_s": round(first_true.stamp_s, 9) if first_true else None,
            "contact_identity": "unknown_bool_topic_only",
        },
    }
    if first_true is None:
        result["window"] = None
        return result

    center_s = first_true.stamp_s
    result["window"] = {
        "center_stamp_s": round(center_s, 9),
        "radius_s": window_s,
        "cmd_vel": sample_window(samples["/cmd_vel"], center_s, window_s),
        "odom": sample_window(samples["/odom"], center_s, window_s),
        "ground_truth_odom": sample_window(samples["/ground_truth/odom"], center_s, window_s),
        "scan": sample_window(samples["/scan"], center_s, window_s),
        "collision_monitor_state": sample_window(samples["/collision_monitor_state"], center_s, window_s),
        "events": parse_events(events, center_s, window_s),
        "dynamic_obstacles": parse_dynamic_rows(dynamic_obstacles, center_s, window_s),
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True, help="path to telemetry_0.mcap")
    parser.add_argument("--events", type=Path, required=True, help="path to events.jsonl")
    parser.add_argument("--dynamic-obstacles", type=Path, required=True, help="path to dynamic_obstacles.csv.gz")
    parser.add_argument("--window-seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path, help="optional JSON output path")
    args = parser.parse_args()
    if not math.isfinite(args.window_seconds) or args.window_seconds <= 0.0:
        parser.error("--window-seconds must be finite and positive")
    for path in (args.bag, args.events, args.dynamic_obstacles):
        if not path.is_file():
            parser.error(f"input does not exist: {path}")

    result = analyze(args.bag.resolve(), args.events.resolve(), args.dynamic_obstacles.resolve(), args.window_seconds)
    text = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
