#!/usr/bin/env python3
"""Run a synthetic end-to-end probe of the independent near-field scan chain."""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import struct
import subprocess
import time
from pathlib import Path
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("nearfield_safety_contract_probe")
        self.publisher = self.create_publisher(
            PointCloud2, "/lidar/points_raw", qos_profile_sensor_data
        )
        self.filtered: list[PointCloud2] = []
        self.legacy: list[LaserScan] = []
        self.safety: list[tuple[LaserScan, float]] = []
        self.sent: dict[tuple[int, int], float] = {}
        self.create_subscription(
            PointCloud2,
            "/lidar/points_scan",
            self.filtered.append,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self.legacy.append,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            "/scan_safety",
            self._on_safety,
            qos_profile_sensor_data,
        )

    def _on_safety(self, message: LaserScan) -> None:
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        if key in self.sent:
            self.safety.append(
                (message, time.monotonic() - self.sent[key])
            )

    def cloud(self) -> PointCloud2:
        message = PointCloud2()
        message.header.frame_id = "base_link"
        message.header.stamp = self.get_clock().now().to_msg()
        message.height = 1
        points = (
            (0.0, 0.0, 0.2),
            (0.30, 0.0, 0.2),
            (0.50, 0.10, 0.2),
        )
        message.width = len(points)
        message.fields = [
            PointField(
                name="x", offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="y", offset=4, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="z", offset=8, datatype=PointField.FLOAT32, count=1
            ),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = b"".join(
            struct.pack("<fff", *point) for point in points
        )
        return message


def percentile95(values: list[float]) -> float:
    ordered = sorted(values)
    return ordered[math.ceil(0.95 * len(ordered)) - 1]


def finite_ranges(message: LaserScan) -> list[float]:
    return [value for value in message.ranges if math.isfinite(value)]


def stop_process_group(process: subprocess.Popen[Any]) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.frames < 20:
        raise SystemExit("--frames must be at least 20")
    output = args.output.resolve()
    if output.exists():
        raise SystemExit(f"refusing to overwrite {output}")

    launch = subprocess.Popen(
        [
            "ros2", "launch", "robot_perception",
            "lidar_processing.launch.py",
            "use_sim_time:=false",
            "enable_safety_scan:=true",
        ],
        start_new_session=True,
    )
    result: dict[str, Any] = {
        "schema": "robot_nearfield_safety_synthetic_contract_v1",
        "pass": False,
        "classification": "synthetic_transport_and_projection_only",
    }
    rclpy.init()
    probe = Probe()
    try:
        deadline = time.monotonic() + 10.0
        while (
            time.monotonic() < deadline
            and (
                probe.count_publishers("/lidar/points_scan") < 1
                or probe.count_publishers("/scan_safety") < 1
                or probe.publisher.get_subscription_count() < 2
            )
        ):
            if launch.poll() is not None:
                raise RuntimeError(
                    f"perception launch exited early: {launch.returncode}"
                )
            rclpy.spin_once(probe, timeout_sec=0.05)
        if time.monotonic() >= deadline:
            raise RuntimeError("near-field perception graph did not converge")

        for _ in range(args.frames):
            message = probe.cloud()
            key = (
                message.header.stamp.sec,
                message.header.stamp.nanosec,
            )
            probe.sent[key] = time.monotonic()
            probe.publisher.publish(message)
            frame_deadline = time.monotonic() + 0.12
            while time.monotonic() < frame_deadline:
                rclpy.spin_once(probe, timeout_sec=0.01)

        if not probe.filtered or not probe.safety or not probe.legacy:
            raise ValueError("one or more projected streams are missing")
        filtered = probe.filtered[-1]
        safety_message = probe.safety[-1][0]
        legacy_message = probe.legacy[-1]
        safety_ranges = finite_ranges(safety_message)
        legacy_ranges = finite_ranges(legacy_message)
        latencies = [latency for _, latency in probe.safety]
        p95 = percentile95(latencies)
        checks = {
            "filtered_point_count_is_two": filtered.width == 2,
            "filtered_frame_is_base_link":
                filtered.header.frame_id == "base_link",
            "filtered_stamp_preserved":
                filtered.header.stamp == safety_message.header.stamp,
            "safety_observes_0_30m":
                bool(safety_ranges) and min(safety_ranges) < 0.31,
            "legacy_excludes_sub_0_40m":
                bool(legacy_ranges) and min(legacy_ranges) > 0.50,
            "latency_p95_le_0_20s": p95 <= 0.20,
            "all_frames_received": len(probe.safety) == args.frames,
        }
        if not all(checks.values()):
            raise ValueError(f"synthetic contract failed: {checks}")
        result.update(
            {
                "pass": True,
                "checks": checks,
                "frames_requested": args.frames,
                "frames_received": len(probe.safety),
                "latency_p95_s": p95,
                "safety_nearest_m": min(safety_ranges),
                "legacy_nearest_m": min(legacy_ranges),
                "filtered_point_count": filtered.width,
            }
        )
    except Exception as error:
        result["error"] = f"{type(error).__name__}: {error}"
    finally:
        probe.destroy_node()
        rclpy.shutdown()
        stop_process_group(launch)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if result["pass"] else 1)


if __name__ == "__main__":
    main()
