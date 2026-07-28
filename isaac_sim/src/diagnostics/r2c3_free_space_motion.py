"""Stage 2.2-R2C3 envelope-bound free-space motion contracts.

This module contains only deterministic validation helpers.  The diagnostic
mode remains default-off and does not alter the ideal-odometry graph, motion
assist, bridge, or any navigation interface.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from isaac_sim.src.diagnostics.r2c2_free_space_envelope import (
    REQUIRED_CLEARANCE_M,
    SUPPORT_HEIGHT_VARIATION_M,
    SegmentAssessment,
)


SCHEMA = "bio_nav_stage2_2_r2c3_free_space_motion_trace_v1"
FROZEN_SEED = "stage2_2_r2c3_frozen_seed"
SOURCE_MESH_BOUNDS_TOLERANCE_M = 1.0e-6


@dataclass(frozen=True)
class EnvelopePreflight:
    """Frozen per-segment gate evaluated after reset and settling."""

    segment_id: str
    support_coverage: float
    support_height_variation_m: float
    minimum_clearance_m: float
    closest_path: str | None
    fallback_paths: tuple[str, ...]
    fallback_source_delta_max_m: float
    invalid_collider_paths: tuple[str, ...]
    valid: bool

    def trace_fields(self) -> dict[str, object]:
        return {
            "segment_id": self.segment_id,
            "support_coverage": self.support_coverage,
            "support_height_variation_m": self.support_height_variation_m,
            "minimum_clearance_m": self.minimum_clearance_m,
            "closest_path": self.closest_path,
            "fallback_paths": list(self.fallback_paths),
            "fallback_source_delta_max_m": self.fallback_source_delta_max_m,
            "invalid_collider_paths": list(self.invalid_collider_paths),
            "required_clearance_m": REQUIRED_CLEARANCE_M,
            "support_height_variation_limit_m": SUPPORT_HEIGHT_VARIATION_M,
            "source_mesh_bounds_tolerance_m": SOURCE_MESH_BOUNDS_TOLERANCE_M,
            "valid": self.valid,
        }


def evaluate_envelope_preflight(
    *,
    assessment: SegmentAssessment,
    classified_colliders: Sequence[Mapping[str, object]],
    frozen_fallback_paths: Sequence[str],
) -> EnvelopePreflight:
    """Apply the R2C3 fail-closed 3D envelope and fallback contract."""

    expected_fallbacks = tuple(sorted(str(path) for path in frozen_fallback_paths))
    fallback_rows = [
        row
        for row in classified_colliders
        if row.get("bounds_source") == "INVISIBLE_COLLISION_SUBTREE_FALLBACK"
    ]
    actual_fallbacks = tuple(sorted(str(row.get("path", "")) for row in fallback_rows))
    invalid_paths = tuple(
        sorted(
            str(row.get("path", ""))
            for row in classified_colliders
            if row.get("classification") in {"INVALID", "DISABLED"}
            or row.get("bounds_source") == "UNRESOLVED"
        )
    )
    fallback_deltas: list[float] = []
    fallback_valid = actual_fallbacks == expected_fallbacks
    for row in fallback_rows:
        raw_delta = row.get("fallback_bounds_source_delta_m")
        if not isinstance(raw_delta, (int, float)):
            fallback_valid = False
            fallback_deltas.append(math.inf)
            continue
        delta = float(raw_delta)
        fallback_deltas.append(delta)
        fallback_valid = (
            fallback_valid
            and math.isfinite(delta)
            and delta <= SOURCE_MESH_BOUNDS_TOLERANCE_M
        )
    maximum_delta = max(fallback_deltas, default=0.0)
    finite_metrics = all(
        math.isfinite(float(value))
        for value in (
            assessment.support_coverage,
            assessment.support_height_variation_m,
            assessment.minimum_clearance_m,
        )
    )
    valid = (
        assessment.valid
        and finite_metrics
        and assessment.support_coverage == 1.0
        and assessment.support_height_variation_m <= SUPPORT_HEIGHT_VARIATION_M
        and assessment.minimum_clearance_m >= REQUIRED_CLEARANCE_M
        and fallback_valid
        and not invalid_paths
    )
    return EnvelopePreflight(
        segment_id=assessment.segment_id,
        support_coverage=float(assessment.support_coverage),
        support_height_variation_m=float(
            assessment.support_height_variation_m
        ),
        minimum_clearance_m=float(assessment.minimum_clearance_m),
        closest_path=assessment.closest_path,
        fallback_paths=actual_fallbacks,
        fallback_source_delta_max_m=float(maximum_delta),
        invalid_collider_paths=invalid_paths,
        valid=valid,
    )
