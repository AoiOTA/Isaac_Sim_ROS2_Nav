"""Strict ROS topic, frame, and QoS contracts for OmniGraph builders."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from isaac_sim.src.yaml_utils import load_mapping, reject_unknown, require_keys


_QOS_TOKEN_MAP = {
    "history": {"keep_last": "keepLast", "keep_all": "keepAll"},
    "reliability": {"reliable": "reliable", "best_effort": "bestEffort"},
    "durability": {"volatile": "volatile", "transient_local": "transientLocal"},
}


def load_topics(path: str | Path) -> dict[str, Any]:
    data = load_mapping(path)
    allowed = {
        "schema_version",
        "clock",
        "pointcloud",
        "imu",
        "joint_states",
        "odom",
        "cmd_vel",
        "ground_truth_odom",
        "ground_truth_path",
        "camera_front_image",
        "camera_front_info",
        "camera_front_depth",
        "camera_front_depth_points",
        "tf",
        "tf_static",
        "frames",
    }
    reject_unknown(data, allowed, context="ROS topic contract")
    require_keys(data, allowed, context="ROS topic contract")
    if data["schema_version"] != 1:
        raise ValueError("ROS topic contract schema_version must be 1")
    if not all(isinstance(data[key], str) and data[key].startswith("/") for key in allowed - {"schema_version", "frames"}):
        raise ValueError("ROS topics must be absolute names")
    frames = data["frames"]
    if not isinstance(frames, dict) or set(frames) != {
        "odom", "base", "lidar", "rtx_lidar", "imu", "camera_front",
        "camera_front_optical"
    }:
        raise ValueError(
            "ROS frame contract must define odom/base/lidar/rtx_lidar/imu/camera_front/camera_front_optical"
        )
    if any(not isinstance(value, str) or not value or value.startswith("/") for value in frames.values()):
        raise ValueError("ROS frame IDs must be non-empty relative names")
    return data


def load_qos_profiles(path: str | Path) -> dict[str, str]:
    data = load_mapping(path)
    reject_unknown(data, {"schema_version", "profiles"}, context="ROS QoS contract")
    require_keys(data, {"schema_version", "profiles"}, context="ROS QoS contract")
    if data["schema_version"] != 1 or not isinstance(data["profiles"], dict):
        raise ValueError("invalid ROS QoS contract")
    result: dict[str, str] = {}
    for name, raw in data["profiles"].items():
        if not isinstance(name, str) or not isinstance(raw, dict):
            raise ValueError("QoS profiles must be named mappings")
        require_keys(raw, {"history", "depth", "reliability", "durability"}, context=f"QoS {name}")
        reject_unknown(raw, {"history", "depth", "reliability", "durability"}, context=f"QoS {name}")
        depth = raw["depth"]
        if isinstance(depth, bool) or not isinstance(depth, int) or depth < 1:
            raise ValueError(f"QoS {name}.depth must be a positive integer")
        encoded: dict[str, Any] = {"depth": depth}
        for field, mapping in _QOS_TOKEN_MAP.items():
            try:
                encoded[field] = mapping[raw[field]]
            except (KeyError, TypeError) as exc:
                raise ValueError(f"unsupported QoS {name}.{field}: {raw.get(field)!r}") from exc
        encoded.update(
            deadline=0.0,
            lifespan=0.0,
            liveliness="systemDefault",
            leaseDuration=0.0,
        )
        result[name] = json.dumps(encoded, sort_keys=True, separators=(",", ":"))
    return result
