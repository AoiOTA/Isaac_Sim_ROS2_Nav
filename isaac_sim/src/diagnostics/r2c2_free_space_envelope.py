"""Stage 2.2-R2C2 three-dimensional free-space envelope diagnostics.

The R2C1 preflight treated every static collision AABB as a planar obstacle.
This module keeps the full vertical interval so floor support and overhead
geometry cannot become false lateral obstacles.  It is default-off and has no
control or navigation dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable, Sequence

from isaac_sim.src.diagnostics.r2c1_free_space_probe import (
    ProbeSegment,
    SegmentedFreeSpaceScript,
)


SCHEMA = "bio_nav_stage2_2_r2c2_free_space_envelope_v1"
REQUIRED_CLEARANCE_M = 0.20
SUPPORT_BAND_M = 0.05
SUPPORT_HEIGHT_VARIATION_M = 0.03
SAMPLES_PER_SEGMENT = 61


@dataclass(frozen=True)
class Bounds3D:
    min_x: float
    min_y: float
    min_z: float
    max_x: float
    max_y: float
    max_z: float

    def finite(self) -> bool:
        values = (self.min_x, self.min_y, self.min_z, self.max_x, self.max_y, self.max_z)
        return all(math.isfinite(value) for value in values) and self.min_x <= self.max_x and self.min_y <= self.max_y and self.min_z <= self.max_z

    def as_dict(self) -> dict[str, list[float] | None]:
        # JSON must not encode IEEE NaN/Infinity.  An unavailable world bound
        # is evidence for the fail-closed STATIC_COLLIDER_CLASSIFICATION_INVALID
        # receipt, not a reason to lose the whole cold-start trace.
        if not self.finite():
            return {"min": None, "max": None}
        return {"min": [self.min_x, self.min_y, self.min_z], "max": [self.max_x, self.max_y, self.max_z]}


@dataclass(frozen=True)
class Collider:
    path: str
    bounds: Bounds3D
    enabled: bool
    aggregate: bool = False


@dataclass(frozen=True)
class SegmentAssessment:
    segment_id: str
    support_coverage: float
    support_height_variation_m: float
    minimum_clearance_m: float
    closest_path: str | None
    valid: bool


def _finite_polygon(footprint: Sequence[Sequence[float]]) -> bool:
    return len(footprint) >= 3 and all(len(point) == 2 and all(math.isfinite(float(value)) for value in point) for point in footprint)


def transform_footprint(footprint: Sequence[Sequence[float]], x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    if not _finite_polygon(footprint) or not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("non-finite free-space footprint transform")
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [(x + cosine * float(px) - sine * float(py), y + sine * float(px) + cosine * float(py)) for px, py in footprint]


def _orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> bool:
    return min(a[0], b[0]) <= c[0] <= max(a[0], b[0]) and min(a[1], b[1]) <= c[1] <= max(a[1], b[1])


def _segments_intersect(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]) -> bool:
    ab_c, ab_d = _orientation(a, b, c), _orientation(a, b, d)
    cd_a, cd_b = _orientation(c, d, a), _orientation(c, d, b)
    if (ab_c > 0) != (ab_d > 0) and (cd_a > 0) != (cd_b > 0):
        return True
    epsilon = 1.0e-12
    return (abs(ab_c) <= epsilon and _on_segment(a, b, c)) or (abs(ab_d) <= epsilon and _on_segment(a, b, d)) or (abs(cd_a) <= epsilon and _on_segment(c, d, a)) or (abs(cd_b) <= epsilon and _on_segment(c, d, b))


def _point_in_aabb(point: tuple[float, float], bounds: Bounds3D) -> bool:
    return bounds.min_x <= point[0] <= bounds.max_x and bounds.min_y <= point[1] <= bounds.max_y


def _point_segment_distance(point: tuple[float, float], start: tuple[float, float], end: tuple[float, float]) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    norm = dx * dx + dy * dy
    if norm <= 1.0e-18:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    fraction = max(0.0, min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / norm))
    return math.hypot(point[0] - (start[0] + fraction * dx), point[1] - (start[1] + fraction * dy))


def polygon_aabb_distance(polygon: Sequence[tuple[float, float]], bounds: Bounds3D) -> float:
    """Exact two-dimensional polygon-to-AABB distance for convex robot footprints."""

    if not polygon or not bounds.finite():
        raise ValueError("invalid polygon or collider bounds")
    rectangle = [(bounds.min_x, bounds.min_y), (bounds.max_x, bounds.min_y), (bounds.max_x, bounds.max_y), (bounds.min_x, bounds.max_y)]
    edges = list(zip(polygon, polygon[1:] + polygon[:1]))
    rect_edges = list(zip(rectangle, rectangle[1:] + rectangle[:1]))
    if any(_point_in_aabb(point, bounds) for point in polygon) or any(_segments_intersect(a, b, c, d) for a, b in edges for c, d in rect_edges):
        return 0.0
    distances = [_point_segment_distance(point, start, end) for point in rectangle for start, end in edges]
    for point in polygon:
        dx = max(bounds.min_x - point[0], 0.0, point[0] - bounds.max_x)
        dy = max(bounds.min_y - point[1], 0.0, point[1] - bounds.max_y)
        distances.append(math.hypot(dx, dy))
    return min(distances)


def _swept_poses(segment: ProbeSegment, start_x: float, start_y: float, start_yaw: float) -> Iterable[tuple[float, float, float]]:
    for index in range(SAMPLES_PER_SEGMENT):
        elapsed = SegmentedFreeSpaceScript.action_s * index / (SAMPLES_PER_SEGMENT - 1)
        if abs(segment.angular_z) < 1.0e-12:
            local_x, local_y, delta_yaw = segment.linear_x * elapsed, 0.0, 0.0
        else:
            radius = segment.linear_x / segment.angular_z
            delta_yaw = segment.angular_z * elapsed
            local_x, local_y = radius * math.sin(delta_yaw), radius * (1.0 - math.cos(delta_yaw))
        cosine, sine = math.cos(start_yaw), math.sin(start_yaw)
        yield (start_x + cosine * local_x - sine * local_y, start_y + sine * local_x + cosine * local_y, start_yaw + delta_yaw)


def classify_collider(collider: Collider, *, support_plane_z: float, robot_max_z: float) -> str:
    if not collider.enabled:
        return "DISABLED"
    if collider.aggregate:
        return "AGGREGATE_EXCLUDED"
    if not collider.bounds.finite():
        return "INVALID"
    if abs(collider.bounds.max_z - support_plane_z) <= SUPPORT_BAND_M:
        return "SUPPORT"
    if collider.bounds.max_z < support_plane_z - SUPPORT_BAND_M:
        return "BELOW"
    if collider.bounds.min_z > robot_max_z:
        return "OVERHEAD"
    return "LATERAL_CANDIDATE"


def assess_envelope(*, footprint: Sequence[Sequence[float]], start_x: float, start_y: float, start_yaw: float, support_plane_z: float, robot_max_z: float, colliders: Sequence[Collider]) -> tuple[list[dict[str, object]], list[SegmentAssessment]]:
    """Classify static colliders and assess all six R2C1 future sweeps."""

    if not all(math.isfinite(value) for value in (start_x, start_y, start_yaw, support_plane_z, robot_max_z)) or robot_max_z <= support_plane_z:
        raise ValueError("invalid robot vertical envelope")
    classified: list[dict[str, object]] = []
    paths: set[str] = set()
    for collider in colliders:
        if collider.path in paths:
            raise ValueError(f"duplicate collider path: {collider.path}")
        paths.add(collider.path)
        classification = classify_collider(collider, support_plane_z=support_plane_z, robot_max_z=robot_max_z)
        classified.append({"path": collider.path, "bounds": collider.bounds.as_dict(), "enabled": collider.enabled, "classification": classification})
    if any(item["classification"] == "INVALID" for item in classified):
        # Keep all segment fields finite so the offline auditor can issue the
        # classifier receipt with its prescribed priority, while retaining the
        # offending collider path/bounds validity in the trace.
        return classified, [
            SegmentAssessment(segment.segment_id, 1.0, 0.0, 0.0, None, False)
            for segment in SegmentedFreeSpaceScript.segments
        ]
    active = [(collider, str(item["classification"])) for collider, item in zip(colliders, classified) if collider.enabled and not collider.aggregate]
    outcomes: list[SegmentAssessment] = []
    for segment in SegmentedFreeSpaceScript.segments:
        clearances: list[tuple[float, str]] = []
        support_tops: list[float] = []
        supported = 0
        for x, y, yaw in _swept_poses(segment, start_x, start_y, start_yaw):
            polygon = transform_footprint(footprint, x, y, yaw)
            sample_support = [collider.bounds.max_z for collider, kind in active if kind == "SUPPORT" and polygon_aabb_distance(polygon, collider.bounds) <= 1.0e-9]
            if sample_support:
                supported += 1
                support_tops.append(max(sample_support))
            for collider, kind in active:
                if kind == "LATERAL_CANDIDATE":
                    clearances.append((polygon_aabb_distance(polygon, collider.bounds), collider.path))
        minimum, path = min(clearances, default=(math.inf, None))
        coverage = supported / SAMPLES_PER_SEGMENT
        variation = (max(support_tops) - min(support_tops)) if support_tops else math.inf
        outcomes.append(SegmentAssessment(segment.segment_id, coverage, variation, minimum, path, coverage == 1.0 and variation <= SUPPORT_HEIGHT_VARIATION_M and minimum >= REQUIRED_CLEARANCE_M))
    return classified, outcomes


class EnvelopeTrace:
    """Append-only JSONL trace for one no-motion R2C2 cold start."""

    def __init__(self, path: Path, *, manifest: dict[str, object]) -> None:
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._output = self.path.open("x", encoding="utf-8")
        self.write({"schema": SCHEMA, "kind": "manifest", **manifest})

    def write(self, value: dict[str, object]) -> None:
        self._output.write(json.dumps(value, sort_keys=True) + "\n")
        self._output.flush()

    def record(self, *, robot_envelope: Bounds3D, support_plane_z: float, colliders: Sequence[dict[str, object]], assessments: Sequence[SegmentAssessment], receipt: str) -> None:
        self.write({"schema": SCHEMA, "kind": "robot_envelope", "bounds": robot_envelope.as_dict(), "support_plane_z": support_plane_z})
        for collider in colliders:
            self.write({"schema": SCHEMA, "kind": "collider", **collider})
        for result in assessments:
            self.write({"schema": SCHEMA, "kind": "segment_assessment", "segment_id": result.segment_id, "support_coverage": result.support_coverage, "support_height_variation_m": result.support_height_variation_m, "minimum_clearance_m": result.minimum_clearance_m, "closest_path": result.closest_path, "valid": result.valid})
        self.write({"schema": SCHEMA, "kind": "receipt", "receipt": receipt})

    def close(self) -> None:
        if not self._output.closed:
            self._output.close()
