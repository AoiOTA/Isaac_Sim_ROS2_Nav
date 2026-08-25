"""Module3-owned physical constraints for the 16x16 V3.10 canvas."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .feasibility import classify_edge, footprint_pose_is_free
from .map_io import OccupancyMap
from .models import Traversability


GRID_SIDE = 16
STATE_COUNT = 256
CANVAS_ORIGIN_M = -8.0
CELL_RESOLUTION_M = 1.0


@dataclass(frozen=True)
class CognitiveConstraints:
    map_version: str
    cognitive_tile_id: str
    tile_revision: int
    graph_revision: int
    t_map_canvas: np.ndarray
    reachable_state_mask: np.ndarray
    verified_transitions: np.ndarray
    structural_confidence: float
    stable_duration_s: float
    persistent_confirmed: bool


@dataclass(frozen=True)
class FixedSceneReachableOverride:
    """One fixed-scene canvas contract loaded from the Module1 scene YAML."""

    scene_id: str
    map_id: str
    t_map_canvas: np.ndarray
    reachable_state_mask: np.ndarray


@dataclass
class CognitiveConstraintsCache:
    """Revision-bound reusable results for logical 16x16 outdoor tiles."""

    values: dict[tuple[str, int, str | None], CognitiveConstraints]
    hits: int = 0
    misses: int = 0

    def __init__(self) -> None:
        self.values = {}
        self.hits = 0
        self.misses = 0

    def get(
        self, key: tuple[str, int, str | None]
    ) -> CognitiveConstraints | None:
        value = self.values.get(key)
        if value is None:
            self.misses += 1
        else:
            self.hits += 1
        return value

    def put(
        self,
        key: tuple[str, int, str | None],
        value: CognitiveConstraints,
    ) -> None:
        self.values[key] = value

    def invalidate(self) -> None:
        """Drop stale values while retaining cumulative audit counters."""

        self.values.clear()


def occupancy_grid_version(
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    data: np.ndarray,
) -> str:
    """Byte-identical map identity used by Integration's ROS bridge."""

    value = np.asarray(data, dtype=np.int8).reshape(-1)
    if int(width) <= 0 or int(height) <= 0 or value.size != int(width) * int(height):
        raise ValueError("occupancy grid geometry/data mismatch")
    digest = hashlib.sha256()
    digest.update(
        json.dumps(
            {
                "width": int(width),
                "height": int(height),
                "resolution": float(resolution),
                "origin_x": float(origin_x),
                "origin_y": float(origin_y),
                "transform": "identity_map_canvas",
            },
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    )
    digest.update(np.ascontiguousarray(value).tobytes())
    return digest.hexdigest()


def _canvas_centers() -> np.ndarray:
    axis = CANVAS_ORIGIN_M + (np.arange(GRID_SIDE) + 0.5) * CELL_RESOLUTION_M
    xx, yy = np.meshgrid(axis, axis, indexing="xy")
    return np.column_stack((xx.reshape(-1), yy.reshape(-1)))


def _canvas_to_map(points: np.ndarray, t_map_canvas: np.ndarray) -> np.ndarray:
    matrix = np.asarray(t_map_canvas, dtype=np.float64).reshape(3, 3)
    if not np.isfinite(matrix).all() or abs(float(np.linalg.det(matrix))) <= 1.0e-12:
        raise ValueError("T_map_canvas must be finite and invertible")
    inverse = np.linalg.inv(matrix)
    homogeneous = np.column_stack((points, np.ones(len(points))))
    transformed = (inverse @ homogeneous.T).T
    return transformed[:, :2] / transformed[:, 2:3]


def load_fixed_scene_reachable_override(
    path: str | Path,
) -> FixedSceneReachableOverride:
    """Load the authoritative scene/mask block without copying it into Module3."""

    source = Path(path).expanduser().resolve()
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("fixed-scene override must be a YAML mapping")
    scene = document.get("scene", document)
    if not isinstance(scene, dict):
        raise ValueError("fixed-scene override scene must be a mapping")
    scene_id = str(scene.get("scene_id", "")).strip()
    map_id = str(scene.get("map_id", "")).strip()
    if not scene_id or not map_id:
        raise ValueError("fixed-scene override requires scene_id and map_id")
    transform = np.asarray(scene.get("T_map_canvas"), dtype=np.float64)
    if transform.shape != (3, 3) or not np.isfinite(transform).all():
        raise ValueError("fixed-scene T_map_canvas must be finite [3,3]")
    if abs(float(np.linalg.det(transform))) <= 1.0e-12:
        raise ValueError("fixed-scene T_map_canvas must be invertible")
    raw_mask = scene.get("valid_state_mask")
    if not isinstance(raw_mask, list) or len(raw_mask) != STATE_COUNT:
        raise ValueError("fixed-scene valid_state_mask must contain 256 booleans")
    if any(not isinstance(value, bool) for value in raw_mask):
        raise ValueError("fixed-scene valid_state_mask must contain only booleans")
    mask = np.asarray(raw_mask, dtype=bool)
    raw_ids = scene.get("valid_state_ids")
    if not isinstance(raw_ids, list) or any(
        isinstance(value, bool) or not isinstance(value, int) for value in raw_ids
    ):
        raise ValueError("fixed-scene valid_state_ids must contain integers")
    if raw_ids != np.flatnonzero(mask).astype(int).tolist():
        raise ValueError("fixed-scene valid_state_ids disagree with valid_state_mask")
    return FixedSceneReachableOverride(
        scene_id=scene_id,
        map_id=map_id,
        t_map_canvas=transform,
        reachable_state_mask=mask,
    )


def build_cognitive_constraints(
    occupancy: OccupancyMap,
    *,
    map_version: str,
    graph_revision: int,
    footprint_settings: dict,
    t_map_canvas: np.ndarray | None = None,
    stable_duration_s: float = 0.0,
    persistent_confirmed: bool = True,
    cognitive_tile_id: str | None = None,
    fixed_scene_override_file: str | Path | None = None,
) -> CognitiveConstraints:
    """Rasterize only footprint-valid cells and swept 4-neighbour motions."""

    fixed_scene = (
        None
        if fixed_scene_override_file is None or not str(fixed_scene_override_file).strip()
        else load_fixed_scene_reachable_override(fixed_scene_override_file)
    )
    transform = (
        fixed_scene.t_map_canvas
        if fixed_scene is not None and t_map_canvas is None
        else np.eye(3, dtype=np.float64)
        if t_map_canvas is None
        else np.asarray(t_map_canvas, dtype=np.float64).reshape(3, 3)
    )
    if fixed_scene is not None:
        if fixed_scene.map_id != occupancy.map_version:
            raise ValueError(
                "fixed-scene map_id differs from the structural occupancy map"
            )
        if not np.allclose(transform, fixed_scene.t_map_canvas, atol=1.0e-12):
            raise ValueError(
                "fixed-scene T_map_canvas differs from the requested transform"
            )
    centers_map = _canvas_to_map(_canvas_centers(), transform)
    polygon = np.asarray(footprint_settings["polygon_m"], dtype=np.float64)
    padding = float(footprint_settings["padding_m"])
    # A reachable state must admit the physical footprint at every cardinal
    # departure heading.  This is conservative and prevents a point-robot mask.
    headings = (0.0, 0.5 * math.pi, math.pi, -0.5 * math.pi)
    reachable = (
        fixed_scene.reachable_state_mask.copy()
        if fixed_scene is not None
        else np.asarray(
            [
                all(
                    footprint_pose_is_free(
                        occupancy,
                        (float(point[0]), float(point[1])),
                        heading,
                        footprint_polygon_m=polygon,
                        footprint_padding_m=padding,
                    )
                    for heading in headings
                )
                for point in centers_map
            ],
            dtype=bool,
        )
    )
    transitions: list[tuple[int, int]] = []
    for source in np.flatnonzero(reachable):
        row, column = divmod(int(source), GRID_SIDE)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            target_row, target_column = row + dr, column + dc
            if not (0 <= target_row < GRID_SIDE and 0 <= target_column < GRID_SIDE):
                continue
            target = target_row * GRID_SIDE + target_column
            if not reachable[target]:
                continue
            path = np.asarray((centers_map[source], centers_map[target]))
            if classify_edge(
                occupancy,
                path,
                footprint_polygon_m=polygon,
                footprint_padding_m=padding,
                padded_inscribed_radius_m=float(
                    footprint_settings["padded_inscribed_radius_m"]
                ),
                sweep_sample_spacing_m=float(
                    footprint_settings["sweep_sample_spacing_m"]
                ),
            ) == Traversability.FEASIBLE:
                transitions.append((int(source), int(target)))
    transition_array = np.asarray(transitions, dtype=np.int64).reshape(-1, 2)
    confidence = 1.0 if reachable.any() and len(transition_array) else 0.0
    if cognitive_tile_id is None:
        tile_digest = hashlib.sha256()
        tile_digest.update(str(map_version).encode("utf-8"))
        tile_digest.update(transform.tobytes())
        tile_id = f"canvas16:{tile_digest.hexdigest()[:16]}"
    else:
        tile_id = str(cognitive_tile_id).strip()
        if not tile_id:
            raise ValueError("cognitive_tile_id must be non-empty")
    return CognitiveConstraints(
        map_version=str(map_version),
        cognitive_tile_id=tile_id,
        tile_revision=int(graph_revision),
        graph_revision=int(graph_revision),
        t_map_canvas=transform,
        reachable_state_mask=reachable,
        verified_transitions=transition_array,
        structural_confidence=confidence,
        stable_duration_s=max(0.0, float(stable_duration_s)),
        persistent_confirmed=bool(persistent_confirmed and confidence > 0.0),
    )


__all__ = [
    "CognitiveConstraints",
    "CognitiveConstraintsCache",
    "FixedSceneReachableOverride",
    "build_cognitive_constraints",
    "load_fixed_scene_reachable_override",
    "occupancy_grid_version",
]
