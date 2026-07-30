#!/usr/bin/env python3
"""Produce a receipt-bound, read-only collision and near-field timeline.

The frozen collision topic carries only ``std_msgs/Bool``. This analyzer never
guesses the contact prim. It correlates the first true sample with the command
chain, scans, costmaps, selected MPPI trajectory, map occupancy, actor state,
and available Nav2 logs while explicitly listing topics absent from old bags.
"""

from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Iterable, Mapping

import yaml


SCHEMA = "bio_nav_collision_forensics_v2"
CORE_TOPICS = {
    "/simulation/collision",
    "/cmd_vel",
    "/odom",
    "/ground_truth/odom",
    "/scan",
    "/collision_monitor_state",
}
OPTIONAL_TOPICS = {
    "/cmd_vel_nav",
    "/cmd_vel_smoothed",
    "/local_costmap/costmap_raw",
    "/global_costmap/costmap_raw",
    "/optimal_trajectory",
    "/scan_safety",
    "/lidar/points_raw",
    "/lidar/points_scan",
}
READ_TOPICS = CORE_TOPICS | OPTIONAL_TOPICS
PADDED_FOOTPRINT = (
    (0.260, 0.215),
    (0.260, -0.215),
    (-0.235, -0.215),
    (-0.235, 0.215),
)
NAV2_PATTERNS = (
    re.compile(pattern, re.IGNORECASE)
    for pattern in (
        r"optimizer fail",
        r"optimizer reset",
        r"failed to create plan",
        r"failed to plan",
        r"no valid control",
        r"patience exceeded",
        r"running backup",
        r"backup failed",
    )
)
NAV2_PATTERNS = tuple(NAV2_PATTERNS)


class Sample:
    def __init__(self, stamp_s: float, value: dict[str, Any]) -> None:
        self.stamp_s = stamp_s
        self.value = value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite(value: float) -> float | None:
    return value if math.isfinite(value) else None


def yaw_from_orientation(orientation: Any) -> float:
    return math.atan2(
        2.0 * (
            orientation.w * orientation.z +
            orientation.x * orientation.y
        ),
        1.0 - 2.0 * (
            orientation.y * orientation.y +
            orientation.z * orientation.z
        ),
    )


def pose_value(message: Any) -> dict[str, float | None]:
    pose = message.pose.pose
    return {
        "x": finite(float(pose.position.x)),
        "y": finite(float(pose.position.y)),
        "yaw_rad": finite(yaw_from_orientation(pose.orientation)),
        "linear_speed_mps": finite(float(message.twist.twist.linear.x)),
        "angular_speed_radps": finite(float(message.twist.twist.angular.z)),
    }


def twist_value(message: Any) -> dict[str, float | None]:
    return {
        "linear_speed_mps": finite(float(message.linear.x)),
        "angular_speed_radps": finite(float(message.angular.z)),
    }


def scan_value(message: Any) -> dict[str, float | int | str | None]:
    ranges = [float(item) for item in message.ranges]
    valid = [
        item for item in ranges
        if math.isfinite(item) and item >= float(message.range_min)
    ]
    return {
        "frame_id": str(message.header.frame_id),
        "range_min_m": finite(float(message.range_min)),
        "range_max_m": finite(float(message.range_max)),
        "valid_ranges": len(valid),
        "minimum_range_m": finite(min(valid)) if valid else None,
    }


def path_value(message: Any) -> dict[str, Any]:
    poses = []
    for item in message.poses:
        poses.append({
            "x": finite(float(item.pose.position.x)),
            "y": finite(float(item.pose.position.y)),
            "yaw_rad": finite(yaw_from_orientation(item.pose.orientation)),
        })
    return {
        "frame_id": str(message.header.frame_id),
        "poses": poses,
    }


def costmap_value(message: Any) -> dict[str, Any]:
    metadata = message.metadata
    origin = metadata.origin
    return {
        "frame_id": str(message.header.frame_id),
        "size_x": int(metadata.size_x),
        "size_y": int(metadata.size_y),
        "resolution_m": float(metadata.resolution),
        "origin": {
            "x": float(origin.position.x),
            "y": float(origin.position.y),
            "yaw_rad": yaw_from_orientation(origin.orientation),
        },
        "data": tuple(int(item) for item in message.data),
    }


def sample_window(
    samples: Iterable[Sample],
    center_s: float,
    window_s: float,
) -> list[dict[str, Any]]:
    return [
        {"stamp_s": round(sample.stamp_s, 9), **sample.value}
        for sample in samples
        if abs(sample.stamp_s - center_s) <= window_s
    ]


def latest_at_or_before(
    samples: Iterable[Sample],
    stamp_s: float,
) -> Sample | None:
    return max(
        (sample for sample in samples if sample.stamp_s <= stamp_s),
        key=lambda sample: sample.stamp_s,
        default=None,
    )


def command_stop_summary(
    samples: Iterable[Sample],
    collision_stamp_s: float,
) -> dict[str, Any] | None:
    preceding = sorted(
        (
            sample for sample in samples
            if sample.stamp_s <= collision_stamp_s
        ),
        key=lambda sample: sample.stamp_s,
    )
    if not preceding:
        return None
    nonzero_indices = [
        index for index, sample in enumerate(preceding)
        if (
            abs(float(sample.value.get("linear_speed_mps") or 0.0)) >
            1.0e-6 or
            abs(float(sample.value.get("angular_speed_radps") or 0.0)) >
            1.0e-6
        )
    ]
    last_nonzero_index = nonzero_indices[-1] if nonzero_indices else None
    first_zero_after = None
    if last_nonzero_index is not None:
        for sample in preceding[last_nonzero_index + 1:]:
            if (
                abs(float(sample.value.get("linear_speed_mps") or 0.0)) <=
                1.0e-6 and
                abs(float(sample.value.get("angular_speed_radps") or 0.0)) <=
                1.0e-6
            ):
                first_zero_after = sample
                break
    last = preceding[-1]
    return {
        "last_nonzero": (
            {
                "stamp_s": round(
                    preceding[last_nonzero_index].stamp_s, 9),
                **preceding[last_nonzero_index].value,
            }
            if last_nonzero_index is not None else None
        ),
        "first_zero_after_last_nonzero": (
            {
                "stamp_s": round(first_zero_after.stamp_s, 9),
                **first_zero_after.value,
            }
            if first_zero_after else None
        ),
        "latest_at_or_before_collision": {
            "stamp_s": round(last.stamp_s, 9),
            **last.value,
        },
    }


def transform_polygon(
    pose: Mapping[str, Any],
    footprint: Iterable[tuple[float, float]] = PADDED_FOOTPRINT,
) -> list[tuple[float, float]]:
    x = float(pose["x"])
    y = float(pose["y"])
    yaw = float(pose["yaw_rad"])
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return [
        (
            x + cosine * local_x - sine * local_y,
            y + sine * local_x + cosine * local_y,
        )
        for local_x, local_y in footprint
    ]


def point_in_polygon(
    point_x: float,
    point_y: float,
    polygon: list[tuple[float, float]],
) -> bool:
    inside = False
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        cross = (
            (point_x - x1) * (y2 - y1) -
            (point_y - y1) * (x2 - x1)
        )
        if (
            abs(cross) <= 1.0e-9 and
            min(x1, x2) - 1.0e-9 <= point_x <= max(x1, x2) + 1.0e-9 and
            min(y1, y2) - 1.0e-9 <= point_y <= max(y1, y2) + 1.0e-9
        ):
            return True
        if (y1 > point_y) != (y2 > point_y):
            intersection_x = (
                (x2 - x1) * (point_y - y1) / (y2 - y1) + x1
            )
            if point_x < intersection_x:
                inside = not inside
        previous = current
    return inside


def costmap_footprint_summary(
    costmap_sample: Sample | None,
    pose_sample: Sample | None,
) -> dict[str, Any] | None:
    if costmap_sample is None or pose_sample is None:
        return None
    value = costmap_sample.value
    pose = pose_sample.value
    if any(pose.get(name) is None for name in ("x", "y", "yaw_rad")):
        return None
    size_x = int(value["size_x"])
    size_y = int(value["size_y"])
    resolution = float(value["resolution_m"])
    data = value["data"]
    if (
        size_x <= 0 or size_y <= 0 or resolution <= 0.0 or
        len(data) != size_x * size_y
    ):
        return None
    origin = value["origin"]
    origin_yaw = float(origin["yaw_rad"])
    origin_cosine = math.cos(origin_yaw)
    origin_sine = math.sin(origin_yaw)
    polygon = transform_polygon(pose)
    costs: list[int] = []
    for cell_y in range(size_y):
        local_y = (cell_y + 0.5) * resolution
        for cell_x in range(size_x):
            local_x = (cell_x + 0.5) * resolution
            world_x = (
                float(origin["x"]) +
                origin_cosine * local_x -
                origin_sine * local_y
            )
            world_y = (
                float(origin["y"]) +
                origin_sine * local_x +
                origin_cosine * local_y
            )
            if point_in_polygon(world_x, world_y, polygon):
                costs.append(int(data[cell_y * size_x + cell_x]))
    known = [cost for cost in costs if cost != 255]
    return {
        "costmap_stamp_s": round(costmap_sample.stamp_s, 9),
        "pose_stamp_s": round(pose_sample.stamp_s, 9),
        "costmap_frame_id": value["frame_id"],
        "pose": {
            "x": pose["x"],
            "y": pose["y"],
            "yaw_rad": pose["yaw_rad"],
        },
        "footprint_cell_count": len(costs),
        "inscribed_cell_count": sum(cost == 253 for cost in costs),
        "lethal_cell_count": sum(cost == 254 for cost in costs),
        "unknown_cell_count": sum(cost == 255 for cost in costs),
        "cost_ge_253_known_count": sum(cost in (253, 254) for cost in costs),
        "maximum_known_cost": max(known) if known else None,
    }


def trajectory_cost_summary(
    trajectory_sample: Sample | None,
    costmap_samples: Iterable[Sample],
) -> dict[str, Any] | None:
    if trajectory_sample is None:
        return None
    costmap_sample = latest_at_or_before(
        costmap_samples, trajectory_sample.stamp_s)
    if costmap_sample is None:
        return None
    collisions = []
    poses = trajectory_sample.value.get("poses", [])
    for index, pose in enumerate(poses):
        pose_sample = Sample(trajectory_sample.stamp_s, pose)
        summary = costmap_footprint_summary(costmap_sample, pose_sample)
        if summary and summary["cost_ge_253_known_count"] > 0:
            collisions.append({
                "index": index,
                "cost_ge_253_known_count":
                    summary["cost_ge_253_known_count"],
            })
    return {
        "trajectory_stamp_s": round(trajectory_sample.stamp_s, 9),
        "costmap_stamp_s": round(costmap_sample.stamp_s, 9),
        "costmap_is_not_newer_than_trajectory": (
            costmap_sample.stamp_s <= trajectory_sample.stamp_s
        ),
        "point_count": len(poses),
        "footprint_collision_point_count": len(collisions),
        "first_collision_point": collisions[0] if collisions else None,
    }


def parse_dynamic_rows(
    path: Path,
    center_s: float,
    window_s: float,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with gzip.open(path, "rt", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            stamp_s = float(row["stamp_s"])
            if abs(stamp_s - center_s) <= window_s:
                rows.append({
                    "stamp_s": stamp_s,
                    "id": row["id"],
                    "state": row["state"],
                    "position": json.loads(row["position"]),
                    "velocity_mps": float(row["velocity_mps"]),
                    "progress": float(row["progress"]),
                    "min_clearance_m": (
                        float(row["min_clearance_m"])
                        if row["min_clearance_m"] else None
                    ),
                })
    return rows


def actor_distance_summary(
    rows: Iterable[Mapping[str, Any]],
    ground_truth: Sample | None,
    center_s: float,
) -> list[dict[str, Any]]:
    if ground_truth is None:
        return []
    robot_x = ground_truth.value.get("x")
    robot_y = ground_truth.value.get("y")
    if robot_x is None or robot_y is None:
        return []
    nearest_by_id: dict[str, Mapping[str, Any]] = {}
    for row in rows:
        actor_id = str(row["id"])
        previous = nearest_by_id.get(actor_id)
        if (
            previous is None or
            abs(float(row["stamp_s"]) - center_s) <
            abs(float(previous["stamp_s"]) - center_s)
        ):
            nearest_by_id[actor_id] = row
    result = []
    for actor_id, row in sorted(nearest_by_id.items()):
        position = row["position"]
        result.append({
            "id": actor_id,
            "stamp_s": row["stamp_s"],
            "state": row["state"],
            "position": position,
            "robot_center_distance_m": math.hypot(
                float(position[0]) - float(robot_x),
                float(position[1]) - float(robot_y),
            ),
            "min_clearance_m": row["min_clearance_m"],
        })
    return result


def parse_events(
    path: Path,
    center_s: float,
    window_s: float,
) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        value = json.loads(line)
        stamp_s = value.get("simulation_time")
        if (
            isinstance(stamp_s, (int, float)) and
            abs(float(stamp_s) - center_s) <= window_s
        ):
            result.append(value)
    return result


def parse_nav2_log(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    matches = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8", errors="replace").splitlines(),
        start=1,
    ):
        if any(pattern.search(line) for pattern in NAV2_PATTERNS):
            matches.append({
                "line": line_number,
                "text": line[-500:],
            })
    return {
        "path": str(path),
        "sha256": sha256(path),
        "time_alignment": "log_clock_not_mapped_to_sim_time",
        "matching_line_count": len(matches),
        "matching_lines": matches[-200:],
    }


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    content = path.read_bytes()
    cursor = 0

    def token() -> bytes:
        nonlocal cursor
        while cursor < len(content):
            if content[cursor:cursor + 1] == b"#":
                newline = content.find(b"\n", cursor)
                cursor = len(content) if newline < 0 else newline + 1
            elif content[cursor:cursor + 1].isspace():
                cursor += 1
            else:
                break
        start = cursor
        while (
            cursor < len(content) and
            not content[cursor:cursor + 1].isspace()
        ):
            cursor += 1
        if start == cursor:
            raise ValueError("truncated PGM header")
        return content[start:cursor]

    if token() != b"P5":
        raise ValueError("only binary P5 PGM maps are supported")
    width = int(token())
    height = int(token())
    maximum = int(token())
    if width <= 0 or height <= 0 or maximum != 255:
        raise ValueError("PGM must be non-empty 8-bit data")
    if content[cursor:cursor + 2] == b"\r\n":
        cursor += 2
    elif content[cursor:cursor + 1].isspace():
        cursor += 1
    pixels = content[cursor:cursor + width * height]
    if len(pixels) != width * height:
        raise ValueError("PGM pixel data is truncated")
    return width, height, pixels


def occupancy_map_summary(
    map_yaml: Path | None,
    ground_truth: Sample | None,
) -> dict[str, Any] | None:
    if map_yaml is None or ground_truth is None:
        return None
    document = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image = Path(str(document["image"]))
    if not image.is_absolute():
        image = (map_yaml.parent / image).resolve()
    width, height, pixels = _read_pgm(image)
    resolution = float(document["resolution"])
    origin = [float(item) for item in document["origin"]]
    occupied_threshold = float(document["occupied_thresh"])
    negate = int(document.get("negate", 0))
    polygon = transform_polygon(ground_truth.value)
    occupied = 0
    footprint_cells = 0
    for cell_y in range(height):
        world_y = origin[1] + (cell_y + 0.5) * resolution
        for cell_x in range(width):
            world_x = origin[0] + (cell_x + 0.5) * resolution
            if not point_in_polygon(world_x, world_y, polygon):
                continue
            footprint_cells += 1
            pixel = pixels[(height - 1 - cell_y) * width + cell_x]
            probability = (
                pixel / 255.0 if negate else (255 - pixel) / 255.0
            )
            occupied += int(probability > occupied_threshold)
    return {
        "yaml": {"path": str(map_yaml), "sha256": sha256(map_yaml)},
        "image": {"path": str(image), "sha256": sha256(image)},
        "resolution_m": resolution,
        "origin": origin,
        "ground_truth_stamp_s": round(ground_truth.stamp_s, 9),
        "footprint_cell_count": footprint_cells,
        "occupied_cell_count": occupied,
        "footprint_overlaps_static_occupied": occupied > 0,
    }


def read_bag(
    path: Path,
) -> tuple[dict[str, list[Sample]], list[Sample], int, set[str]]:
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:  # pragma: no cover
        raise RuntimeError(
            "ROS 2 Jazzy rosbag2 Python support is required") from error

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(path.parent), storage_id="mcap"),
        ConverterOptions("", ""),
    )
    type_by_topic = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    missing = CORE_TOPICS - set(type_by_topic)
    if missing:
        raise ValueError(
            f"MCAP is missing required topics: {sorted(missing)}")

    samples = {
        topic: [] for topic in READ_TOPICS
        if topic != "/simulation/collision"
    }
    collisions: list[Sample] = []
    true_count = 0
    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in READ_TOPICS:
            continue
        message = deserialize_message(raw, get_message(type_by_topic[topic]))
        stamp_s = float(timestamp_ns) / 1.0e9
        if topic == "/simulation/collision":
            value = bool(message.data)
            collisions.append(Sample(stamp_s, {"data": value}))
            true_count += int(value)
        elif topic in {"/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed"}:
            samples[topic].append(Sample(stamp_s, twist_value(message)))
        elif topic in {"/odom", "/ground_truth/odom"}:
            samples[topic].append(Sample(stamp_s, pose_value(message)))
        elif topic in {"/scan", "/scan_safety"}:
            samples[topic].append(Sample(stamp_s, scan_value(message)))
        elif topic == "/collision_monitor_state":
            samples[topic].append(Sample(stamp_s, {
                "action_type": int(message.action_type),
                "polygon_name": str(message.polygon_name),
            }))
        elif topic in {
            "/local_costmap/costmap_raw",
            "/global_costmap/costmap_raw",
        }:
            samples[topic].append(Sample(stamp_s, costmap_value(message)))
        elif topic == "/optimal_trajectory":
            samples[topic].append(Sample(stamp_s, path_value(message)))
        # Raw/filtered point clouds are availability evidence in v2. Their
        # payloads are intentionally not expanded into the JSON report.
    return samples, collisions, true_count, set(type_by_topic)


def analyze(
    bag: Path,
    events: Path,
    dynamic_obstacles: Path,
    window_s: float,
    *,
    occupancy_map_yaml: Path | None = None,
    nav2_log: Path | None = None,
) -> dict[str, Any]:
    samples, collisions, true_count, available_topics = read_bag(bag)
    first_true = next(
        (sample for sample in collisions if sample.value["data"]), None)
    result: dict[str, Any] = {
        "schema": SCHEMA,
        "inputs": {
            "bag": {"path": str(bag), "sha256": sha256(bag)},
            "events": {"path": str(events), "sha256": sha256(events)},
            "dynamic_obstacles": {
                "path": str(dynamic_obstacles),
                "sha256": sha256(dynamic_obstacles),
            },
        },
        "topic_availability": {
            topic: topic in available_topics
            for topic in sorted(READ_TOPICS)
        },
        "unavailable_in_frozen_evidence": sorted(
            topic for topic in OPTIONAL_TOPICS
            if topic not in available_topics
        ),
        "collision_topic": {
            "sample_count": len(collisions),
            "true_count": true_count,
            "first_true_stamp_s": (
                round(first_true.stamp_s, 9) if first_true else None
            ),
            "contact_identity": "unknown_bool_topic_only",
        },
        "nav2_log": parse_nav2_log(nav2_log),
    }
    if first_true is None:
        result["window"] = None
        result["causal_assessment"] = {
            "status": "no_collision_in_input",
        }
        return result

    center_s = first_true.stamp_s
    ground_truth = latest_at_or_before(
        samples["/ground_truth/odom"], center_s)
    odom = latest_at_or_before(samples["/odom"], center_s)
    dynamic_rows = parse_dynamic_rows(
        dynamic_obstacles, center_s, window_s)
    local_costmap = latest_at_or_before(
        samples["/local_costmap/costmap_raw"], center_s)
    global_costmap = latest_at_or_before(
        samples["/global_costmap/costmap_raw"], center_s)
    trajectory = latest_at_or_before(
        samples["/optimal_trajectory"], center_s)
    legacy_scan = latest_at_or_before(samples["/scan"], center_s)
    safety_scan = latest_at_or_before(samples["/scan_safety"], center_s)

    result["window"] = {
        "center_stamp_s": round(center_s, 9),
        "radius_s": window_s,
        "cmd_vel_nav": sample_window(
            samples["/cmd_vel_nav"], center_s, window_s),
        "cmd_vel_smoothed": sample_window(
            samples["/cmd_vel_smoothed"], center_s, window_s),
        "cmd_vel": sample_window(
            samples["/cmd_vel"], center_s, window_s),
        "odom": sample_window(
            samples["/odom"], center_s, window_s),
        "ground_truth_odom": sample_window(
            samples["/ground_truth/odom"], center_s, window_s),
        "scan": sample_window(
            samples["/scan"], center_s, window_s),
        "scan_safety": sample_window(
            samples["/scan_safety"], center_s, window_s),
        "collision_monitor_state": sample_window(
            samples["/collision_monitor_state"], center_s, window_s),
        "events": parse_events(events, center_s, window_s),
        "dynamic_obstacles": dynamic_rows,
    }
    result["nearfield_contract"] = {
        "legacy_scan_at_collision": (
            {
                "stamp_s": round(legacy_scan.stamp_s, 9),
                **legacy_scan.value,
            }
            if legacy_scan else None
        ),
        "safety_scan_at_collision": (
            {
                "stamp_s": round(safety_scan.stamp_s, 9),
                **safety_scan.value,
            }
            if safety_scan else None
        ),
        "padded_footprint": [list(point) for point in PADDED_FOOTPRINT],
        "stop_zone_extent_m": {
            "front": 0.285,
            "rear": -0.250,
            "left": 0.230,
            "right": -0.230,
        },
        "legacy_scan_blind_radius_exceeds_stop_zone_side": bool(
            legacy_scan and
            legacy_scan.value.get("range_min_m") is not None and
            float(legacy_scan.value["range_min_m"]) > 0.230
        ),
    }
    result["command_stop_before_collision"] = {
        "cmd_vel_nav": command_stop_summary(
            samples["/cmd_vel_nav"], center_s),
        "cmd_vel_smoothed": command_stop_summary(
            samples["/cmd_vel_smoothed"], center_s),
        "cmd_vel": command_stop_summary(
            samples["/cmd_vel"], center_s),
    }
    result["geometry"] = {
        "odom_footprint_on_latest_local_costmap":
            costmap_footprint_summary(local_costmap, odom),
        "ground_truth_footprint_on_latest_global_costmap":
            costmap_footprint_summary(global_costmap, ground_truth),
        "ground_truth_footprint_on_static_map":
            occupancy_map_summary(occupancy_map_yaml, ground_truth),
        "latest_optimal_trajectory_on_contemporaneous_local_costmap":
            trajectory_cost_summary(
                trajectory,
                samples["/local_costmap/costmap_raw"],
            ),
    }
    result["actor_distance_at_collision"] = actor_distance_summary(
        dynamic_rows, ground_truth, center_s)

    static_map = result["geometry"][
        "ground_truth_footprint_on_static_map"]
    actor_distances = [
        item["robot_center_distance_m"]
        for item in result["actor_distance_at_collision"]
    ]
    moving_actor_distances = [
        item["robot_center_distance_m"]
        for item in result["actor_distance_at_collision"]
        if item["state"] == "moving"
    ]
    scan_blind = result["nearfield_contract"][
        "legacy_scan_blind_radius_exceeds_stop_zone_side"]
    result["causal_assessment"] = {
        "status": (
            "supports_static_wall_plus_legacy_scan_nearfield_blind_zone"
            if (
                static_map is not None and
                static_map["footprint_overlaps_static_occupied"] and
                scan_blind and
                (
                    not moving_actor_distances or
                    min(moving_actor_distances) > 1.0
                )
            )
            else "inconclusive_or_contradictory"
        ),
        "static_map_overlap": (
            static_map["footprint_overlaps_static_occupied"]
            if static_map else None
        ),
        "minimum_actor_center_distance_m": (
            min(actor_distances) if actor_distances else None
        ),
        "minimum_moving_actor_center_distance_m": (
            min(moving_actor_distances)
            if moving_actor_distances else None
        ),
        "legacy_scan_blind_zone": scan_blind,
        "contact_identity_remains_unknown": True,
        "module2_causal_role": "none_shadow_only",
    }
    return result


def render_markdown(result: Mapping[str, Any]) -> str:
    collision = result["collision_topic"]
    assessment = result["causal_assessment"]
    lines = [
        "# 10303 近场碰撞冻结取证 v2",
        "",
        f"- schema：`{result['schema']}`",
        f"- 首次碰撞：`{collision['first_true_stamp_s']}` s",
        f"- collision true count：`{collision['true_count']}`",
        f"- contact identity：`{collision['contact_identity']}`",
        f"- 因果判定：`{assessment['status']}`",
        f"- 静态地图足迹重叠：`{assessment.get('static_map_overlap')}`",
        "- 最小 moving actor 中心距离："
        f"`{assessment.get('minimum_moving_actor_center_distance_m')}` m",
        f"- legacy scan 近场盲区：`{assessment.get('legacy_scan_blind_zone')}`",
        "",
        "## 冻结证据中缺失的新增 topic",
        "",
    ]
    missing = result.get("unavailable_in_frozen_evidence", [])
    lines.extend(f"- `{topic}`" for topic in missing)
    if not missing:
        lines.append("- 无")
    lines.extend([
        "",
        "旧 MCAP 缺失的新 topic 只标记为 unavailable；不得用新运行补写旧证据。",
        "",
    ])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag", type=Path, required=True,
        help="path to telemetry_0.mcap")
    parser.add_argument(
        "--events", type=Path, required=True,
        help="path to events.jsonl")
    parser.add_argument(
        "--dynamic-obstacles", type=Path, required=True,
        help="path to dynamic_obstacles.csv.gz")
    parser.add_argument("--occupancy-map-yaml", type=Path)
    parser.add_argument("--nav2-log", type=Path)
    parser.add_argument("--window-seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path, help="optional JSON output path")
    parser.add_argument(
        "--markdown-output", type=Path,
        help="optional Markdown summary path")
    args = parser.parse_args()
    if (
        not math.isfinite(args.window_seconds) or
        args.window_seconds <= 0.0
    ):
        parser.error("--window-seconds must be finite and positive")
    for path in (
        args.bag,
        args.events,
        args.dynamic_obstacles,
        args.occupancy_map_yaml,
        args.nav2_log,
    ):
        if path is not None and not path.is_file():
            parser.error(f"input does not exist: {path}")

    result = analyze(
        args.bag.resolve(),
        args.events.resolve(),
        args.dynamic_obstacles.resolve(),
        args.window_seconds,
        occupancy_map_yaml=(
            args.occupancy_map_yaml.resolve()
            if args.occupancy_map_yaml else None
        ),
        nav2_log=args.nav2_log.resolve() if args.nav2_log else None,
    )
    text = json.dumps(
        result, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")
    if args.markdown_output:
        args.markdown_output.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_output.write_text(
            render_markdown(result), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
