"""Fail-closed geometric contact evidence for post-map static obstacles."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml


Point = tuple[float, float]


def load_robot_footprint(robot_config: Path) -> tuple[Point, ...]:
    document = yaml.safe_load(robot_config.read_text(encoding="utf-8"))
    raw = document.get("footprint") if isinstance(document, Mapping) else None
    if not isinstance(raw, list) or len(raw) < 3:
        raise ValueError("robot footprint must contain at least three vertices")
    vertices: list[Point] = []
    for item in raw:
        if not isinstance(item, list) or len(item) != 2:
            raise ValueError("robot footprint vertices must be [x, y]")
        point = (float(item[0]), float(item[1]))
        if not all(math.isfinite(value) for value in point):
            raise ValueError("robot footprint vertices must be finite")
        vertices.append(point)
    return tuple(vertices)


def transform_footprint(
    footprint: Sequence[Point], x: float, y: float, yaw: float
) -> tuple[Point, ...]:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return tuple(
        (
            x + cosine * local_x - sine * local_y,
            y + sine * local_x + cosine * local_y,
        )
        for local_x, local_y in footprint
    )


def axis_aligned_box(center: Point, size: Point) -> tuple[Point, ...]:
    half_x, half_y = 0.5 * size[0], 0.5 * size[1]
    return (
        (center[0] - half_x, center[1] - half_y),
        (center[0] + half_x, center[1] - half_y),
        (center[0] + half_x, center[1] + half_y),
        (center[0] - half_x, center[1] + half_y),
    )


def _axes(polygon: Sequence[Point]) -> tuple[Point, ...]:
    result: list[Point] = []
    for start, end in zip(polygon, (*polygon[1:], polygon[0])):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length > 1.0e-12:
            result.append((-dy / length, dx / length))
    return tuple(result)


def convex_contact_depth(
    first: Sequence[Point],
    second: Sequence[Point],
    *,
    numerical_margin_m: float = 0.0,
) -> float | None:
    """Return SAT overlap only when the two closed polygons really touch."""

    if numerical_margin_m < 0.0 or not math.isfinite(numerical_margin_m):
        raise ValueError("numerical contact margin must be finite and non-negative")
    minimum_overlap = math.inf
    for axis in (*_axes(first), *_axes(second)):
        first_projection = [x * axis[0] + y * axis[1] for x, y in first]
        second_projection = [x * axis[0] + y * axis[1] for x, y in second]
        overlap = min(max(first_projection), max(second_projection)) - max(
            min(first_projection), min(second_projection)
        )
        if overlap < -numerical_margin_m:
            return None
        minimum_overlap = min(minimum_overlap, overlap)
    return minimum_overlap if math.isfinite(minimum_overlap) else None


def static_contact_summary(
    ground_truth: Iterable[Any],
    obstacle_state: Sequence[Mapping[str, Any]],
    footprint: Sequence[Point],
    *,
    numerical_margin_m: float = 0.0,
) -> dict[str, Any]:
    """Compare every GT pose with every active map-frame static obstacle."""

    obstacles: list[tuple[str, tuple[Point, ...]]] = []
    for item in obstacle_state:
        position, size = item.get("position"), item.get("size")
        if (
            item.get("position_frame") != "map"
            or item.get("retired") is True
            or not isinstance(position, list)
            or len(position) < 2
            or not isinstance(size, list)
            or len(size) < 2
        ):
            continue
        values = (float(position[0]), float(position[1]), float(size[0]), float(size[1]))
        if not all(math.isfinite(value) for value in values) or min(values[2:]) <= 0.0:
            continue
        obstacles.append(
            (
                str(item.get("id", "")),
                axis_aligned_box((values[0], values[1]), (values[2], values[3])),
            )
        )
    poses = list(ground_truth)
    contacts: dict[str, dict[str, Any]] = {}
    contact_sample_count = 0
    for sample_index, sample in enumerate(poses):
        robot = transform_footprint(
            footprint, float(sample.x), float(sample.y), float(sample.yaw_rad)
        )
        for identifier, obstacle in obstacles:
            depth = convex_contact_depth(
                robot, obstacle, numerical_margin_m=numerical_margin_m
            )
            if depth is None:
                continue
            contact_sample_count += 1
            row = contacts.setdefault(
                identifier,
                {
                    "obstacle_id": identifier,
                    "first_sample_index": sample_index,
                    "first_stamp_s": float(sample.stamp_s),
                    "maximum_sat_overlap_m": depth,
                },
            )
            row["maximum_sat_overlap_m"] = max(
                float(row["maximum_sat_overlap_m"]), depth
            )
    return {
        "schema": "bio_nav_static_geometric_contact_v1",
        "observed": bool(poses and obstacles),
        "ground_truth_sample_count": len(poses),
        "static_obstacle_count": len(obstacles),
        "numerical_margin_m": numerical_margin_m,
        "contact_detected": bool(contacts),
        "contact_sample_count": contact_sample_count,
        "maximum_sat_overlap_m": max(
            (
                float(item["maximum_sat_overlap_m"])
                for item in contacts.values()
            ),
            default=0.0,
        ),
        "contacts": sorted(contacts.values(), key=lambda item: item["obstacle_id"]),
        "control_input": False,
    }


def exceeds_overlap_tolerance(
    summary: Mapping[str, Any], maximum_accepted_overlap_m: float
) -> bool:
    """Classify geometric overlap while preserving the zero-tolerance default."""

    if (
        maximum_accepted_overlap_m < 0.0
        or not math.isfinite(maximum_accepted_overlap_m)
    ):
        raise ValueError("maximum accepted overlap must be finite and non-negative")
    if summary.get("contact_detected") is not True:
        return False
    maximum = float(summary.get("maximum_sat_overlap_m", 0.0))
    return (
        maximum >= 0.0
        if maximum_accepted_overlap_m == 0.0
        else maximum > maximum_accepted_overlap_m
    )
