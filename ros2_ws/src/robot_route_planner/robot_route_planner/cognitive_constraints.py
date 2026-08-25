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
    transition_witnesses_map_xy: np.ndarray
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
    mask_algorithm: dict[str, object] | None = None


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


def _load_yaml_mapping(path: Path, *, description: str) -> dict[str, object]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"{description} must be a YAML mapping")
    return document


def _referenced_scene_document(
    source: Path, document: dict[str, object]
) -> dict[str, object] | None:
    """Resolve Module2's thin shadow config to its canonical enrollment YAML."""

    raw = document.get("scene_config")
    if not isinstance(raw, str) or not raw.strip():
        return None
    reference = Path(raw).expanduser()
    candidates = [reference] if reference.is_absolute() else [
        source.parent / reference,
        source.parent.parent / reference,
        source.parent / reference.name,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return _load_yaml_mapping(
                candidate.resolve(), description="referenced fixed-scene enrollment"
            )
    raise ValueError(f"fixed-scene scene_config does not exist: {raw}")


def load_fixed_scene_reachable_override(
    path: str | Path,
) -> FixedSceneReachableOverride:
    """Load the authoritative scene/mask block without copying it into Module3."""

    source = Path(path).expanduser().resolve()
    document = _load_yaml_mapping(source, description="fixed-scene override")
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
    algorithm_document = document
    referenced = _referenced_scene_document(source, document)
    if referenced is not None:
        referenced_scene = referenced.get("scene", referenced)
        if not isinstance(referenced_scene, dict):
            raise ValueError("referenced fixed-scene enrollment must contain a scene")
        referenced_mask = np.asarray(
            referenced_scene.get("valid_state_mask"), dtype=bool
        )
        referenced_transform = np.asarray(
            referenced_scene.get("T_map_canvas"), dtype=np.float64
        )
        if (
            str(referenced_scene.get("scene_id", "")).strip() != scene_id
            or str(referenced_scene.get("map_id", "")).strip() != map_id
            or referenced_mask.shape != (STATE_COUNT,)
            or not np.array_equal(referenced_mask, mask)
            or referenced_transform.shape != (3, 3)
            or not np.allclose(referenced_transform, transform, atol=1.0e-12)
        ):
            raise ValueError(
                "fixed-scene shadow and referenced enrollment disagree"
            )
        algorithm_document = referenced
    raw_algorithm = algorithm_document.get("mask_algorithm")
    if raw_algorithm is not None and not isinstance(raw_algorithm, dict):
        raise ValueError("fixed-scene mask_algorithm must be a mapping")
    return FixedSceneReachableOverride(
        scene_id=scene_id,
        map_id=map_id,
        t_map_canvas=transform,
        reachable_state_mask=mask,
        mask_algorithm=(
            None if raw_algorithm is None else dict(raw_algorithm)
        ),
    )


def _padded_inscribed_radius_m(polygon: np.ndarray, padding_m: float) -> float:
    starts = polygon
    ends = np.roll(polygon, -1, axis=0)
    segments = ends - starts
    lengths = np.linalg.norm(segments, axis=1)
    if len(polygon) < 3 or np.any(lengths <= np.finfo(float).eps):
        raise ValueError("footprint polygon must contain non-degenerate edges")
    cross = starts[:, 0] * ends[:, 1] - starts[:, 1] * ends[:, 0]
    return float(np.min(np.abs(cross) / lengths) + padding_m)


def fixed_scene_footprint_settings(
    fixed_scene: FixedSceneReachableOverride,
) -> dict[str, object]:
    """Build the footprint inputs used by the canonical enrollment mask."""

    algorithm = fixed_scene.mask_algorithm
    if algorithm is None:
        raise ValueError("fixed-scene enrollment has no mask_algorithm")
    polygon = np.asarray(algorithm.get("footprint_xy_m"), dtype=np.float64)
    if polygon.ndim != 2 or polygon.shape[1] != 2:
        raise ValueError("mask_algorithm footprint_xy_m must be [N,2]")
    padding = float(algorithm.get("footprint_padding_m"))
    spacing = float(algorithm.get("cardinal_swept_transition_step_m"))
    if padding < 0.0 or not math.isfinite(padding):
        raise ValueError("mask_algorithm footprint_padding_m must be non-negative")
    if spacing <= 0.0 or not math.isfinite(spacing):
        raise ValueError(
            "mask_algorithm cardinal_swept_transition_step_m must be positive"
        )
    return {
        "polygon_m": polygon.tolist(),
        "padding_m": padding,
        "padded_inscribed_radius_m": _padded_inscribed_radius_m(
            polygon, padding
        ),
        "sweep_sample_spacing_m": spacing,
    }


def _fixed_scene_transition_search_parameters(
    fixed_scene: FixedSceneReachableOverride,
    occupancy: OccupancyMap,
    footprint_settings: dict,
) -> tuple[float, float]:
    """Validate and return the canonical pose lattice and boundary inset."""

    algorithm = fixed_scene.mask_algorithm
    if algorithm is None:
        spacing = float(occupancy.resolution_m)
        return spacing, 0.5 * spacing
    source_resolution = float(algorithm.get("source_map_resolution_m"))
    polygon = np.asarray(algorithm.get("footprint_xy_m"), dtype=np.float64)
    padding = float(algorithm.get("footprint_padding_m"))
    sweep_spacing = float(algorithm.get("cardinal_swept_transition_step_m"))
    minimum_area = float(algorithm.get("any_pose_min_feasible_area_m2"))
    raw_bins = algorithm.get("heading_sensitivity_bins")
    if (
        not math.isfinite(source_resolution)
        or source_resolution <= 0.0
        or not math.isclose(
            source_resolution,
            float(occupancy.resolution_m),
            rel_tol=0.0,
            abs_tol=1.0e-12,
        )
    ):
        raise ValueError(
            "mask_algorithm source_map_resolution_m differs from occupancy map"
        )
    requested_polygon = np.asarray(
        footprint_settings["polygon_m"], dtype=np.float64
    )
    if polygon.shape != requested_polygon.shape or not np.allclose(
        polygon, requested_polygon, atol=1.0e-12
    ):
        raise ValueError(
            "mask_algorithm footprint differs from requested footprint_settings"
        )
    if not math.isclose(
        padding,
        float(footprint_settings["padding_m"]),
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError(
            "mask_algorithm padding differs from requested footprint_settings"
        )
    if not math.isclose(
        sweep_spacing,
        float(footprint_settings["sweep_sample_spacing_m"]),
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError(
            "mask_algorithm sweep step differs from requested footprint_settings"
        )
    if (
        not isinstance(raw_bins, list)
        or not raw_bins
        or any(
            isinstance(value, bool)
            or not isinstance(value, int)
            or value <= 0
            or value % 4
            for value in raw_bins
        )
        or algorithm.get("heading_sensitivity_consistent") is not True
    ):
        raise ValueError(
            "mask_algorithm heading sensitivity must consistently include cardinals"
        )
    if not math.isfinite(minimum_area) or minimum_area <= 0.0:
        raise ValueError(
            "mask_algorithm any_pose_min_feasible_area_m2 must be positive"
        )
    boundary_inset = max(0.5 * source_resolution, sweep_spacing)
    if boundary_inset >= 0.5 * CELL_RESOLUTION_M:
        raise ValueError("mask_algorithm sweep step is too large for a canvas cell")
    return source_resolution, boundary_inset


def _fixed_scene_transition_witness(
    occupancy: OccupancyMap,
    *,
    source: int,
    dr: int,
    dc: int,
    t_map_canvas: np.ndarray,
    footprint_settings: dict,
    spacing_m: float,
    boundary_inset_m: float,
) -> np.ndarray | None:
    """Find a directed physical sweep crossing the cells' shared boundary."""

    polygon = np.asarray(footprint_settings["polygon_m"], dtype=np.float64)
    padding = float(footprint_settings["padding_m"])
    source_row, source_column = divmod(source, GRID_SIDE)
    boundary = CANVAS_ORIGIN_M + (
        source_column + (1 if dc > 0 else 0)
        if dc
        else source_row + (1 if dr > 0 else 0)
    ) * CELL_RESOLUTION_M
    transverse_min = CANVAS_ORIGIN_M + (
        source_row if dc else source_column
    ) * CELL_RESOLUTION_M
    transverse_values = np.arange(
        transverse_min + 0.5 * spacing_m,
        transverse_min + CELL_RESOLUTION_M,
        spacing_m,
        dtype=np.float64,
    )
    sign = dc if dc else dr
    for transverse in transverse_values:
        path_canvas = (
            np.asarray(
                (
                    (boundary - sign * boundary_inset_m, transverse),
                    (boundary + sign * boundary_inset_m, transverse),
                ),
                dtype=np.float64,
            )
            if dc
            else np.asarray(
                (
                    (transverse, boundary - sign * boundary_inset_m),
                    (transverse, boundary + sign * boundary_inset_m),
                ),
                dtype=np.float64,
            )
        )
        path = _canvas_to_map(path_canvas, t_map_canvas)
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
            return path
    return None


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
    requested_tile_id = (
        None if cognitive_tile_id is None else str(cognitive_tile_id).strip()
    )
    if cognitive_tile_id is not None and not requested_tile_id:
        raise ValueError("cognitive_tile_id must be non-empty")
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
        if requested_tile_id is not None and requested_tile_id != fixed_scene.map_id:
            raise ValueError(
                "fixed-scene cognitive_tile_id differs from canonical map_id"
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
    transition_witnesses: list[np.ndarray] = []
    fixed_spacing = fixed_boundary_inset = None
    if fixed_scene is not None:
        fixed_spacing, fixed_boundary_inset = (
            _fixed_scene_transition_search_parameters(
                fixed_scene, occupancy, footprint_settings
            )
        )
    for source in np.flatnonzero(reachable):
        row, column = divmod(int(source), GRID_SIDE)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            target_row, target_column = row + dr, column + dc
            if not (0 <= target_row < GRID_SIDE and 0 <= target_column < GRID_SIDE):
                continue
            target = target_row * GRID_SIDE + target_column
            if not reachable[target]:
                continue
            path = (
                np.asarray((centers_map[source], centers_map[target]))
                if fixed_scene is None
                else _fixed_scene_transition_witness(
                    occupancy,
                    source=int(source),
                    dr=dr,
                    dc=dc,
                    t_map_canvas=transform,
                    footprint_settings=footprint_settings,
                    spacing_m=float(fixed_spacing),
                    boundary_inset_m=float(fixed_boundary_inset),
                )
            )
            if path is None:
                continue
            feasible = fixed_scene is not None or classify_edge(
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
            ) == Traversability.FEASIBLE
            if feasible:
                transitions.append((int(source), int(target)))
                transition_witnesses.append(np.asarray(path, dtype=np.float64))
    transition_array = np.asarray(transitions, dtype=np.int64).reshape(-1, 2)
    witness_array = np.asarray(
        transition_witnesses, dtype=np.float64
    ).reshape(-1, 2, 2)
    confidence = 1.0 if reachable.any() and len(transition_array) else 0.0
    if fixed_scene is not None and requested_tile_id is None:
        tile_id = fixed_scene.map_id
    elif requested_tile_id is None:
        tile_digest = hashlib.sha256()
        tile_digest.update(str(map_version).encode("utf-8"))
        tile_digest.update(transform.tobytes())
        tile_id = f"canvas16:{tile_digest.hexdigest()[:16]}"
    else:
        tile_id = requested_tile_id
    return CognitiveConstraints(
        map_version=str(map_version),
        cognitive_tile_id=tile_id,
        tile_revision=int(graph_revision),
        graph_revision=int(graph_revision),
        t_map_canvas=transform,
        reachable_state_mask=reachable,
        verified_transitions=transition_array,
        transition_witnesses_map_xy=witness_array,
        structural_confidence=confidence,
        stable_duration_s=max(0.0, float(stable_duration_s)),
        persistent_confirmed=bool(persistent_confirmed and confidence > 0.0),
    )


def cognitive_constraints_payload(
    value: CognitiveConstraints, *, include_witnesses: bool = False
) -> dict[str, object]:
    """Serialize the existing Module3 constraint payload for offline consumers."""

    if len(value.transition_witnesses_map_xy) != len(value.verified_transitions):
        raise ValueError("transition witnesses must align with verified transitions")
    payload: dict[str, object] = {
        "map_id": value.cognitive_tile_id,
        "map_version": value.map_version,
        "T_map_canvas": value.t_map_canvas.tolist(),
        "valid_state_ids": np.flatnonzero(
            value.reachable_state_mask
        ).astype(int).tolist(),
        "valid_state_mask": value.reachable_state_mask.tolist(),
        "verified_transitions": value.verified_transitions.astype(int).tolist(),
    }
    if include_witnesses:
        payload["transition_witnesses_map_xy"] = (
            value.transition_witnesses_map_xy.tolist()
        )
    return payload


def observed_adjacent_transition_report(
    value: CognitiveConstraints,
    state_ids: np.ndarray,
    state_label_valid: np.ndarray,
) -> dict[str, object]:
    """Evaluate GT-labelled adjacent transitions without influencing topology."""

    states = np.asarray(state_ids, dtype=np.int64).reshape(-1)
    valid = np.asarray(state_label_valid, dtype=bool).reshape(-1)
    if len(states) != len(valid):
        raise ValueError("state_ids and state_label_valid must have equal length")
    verified = {
        (int(source), int(target))
        for source, target in value.verified_transitions
    }
    observed: list[tuple[int, int]] = []
    for index in range(max(0, len(states) - 1)):
        if not valid[index] or not valid[index + 1]:
            continue
        source = int(states[index])
        target = int(states[index + 1])
        if source == target or not (
            0 <= source < STATE_COUNT and 0 <= target < STATE_COUNT
        ):
            continue
        source_row, source_column = divmod(source, GRID_SIDE)
        target_row, target_column = divmod(target, GRID_SIDE)
        if abs(source_row - target_row) + abs(source_column - target_column) != 1:
            continue
        if not (
            value.reachable_state_mask[source]
            and value.reachable_state_mask[target]
        ):
            continue
        observed.append((source, target))
    missing = [edge for edge in observed if edge not in verified]
    unique = set(observed)
    missing_unique = sorted(unique.difference(verified))
    return {
        "observed_adjacent_occurrences": len(observed),
        "missing_adjacent_occurrences": len(missing),
        "missing_occurrence_rate": (
            0.0 if not observed else len(missing) / len(observed)
        ),
        "observed_adjacent_unique": len(unique),
        "missing_adjacent_unique": len(missing_unique),
        "missing_unique_rate": (
            0.0 if not unique else len(missing_unique) / len(unique)
        ),
        "missing_unique_transitions": [list(edge) for edge in missing_unique],
    }


__all__ = [
    "CognitiveConstraints",
    "CognitiveConstraintsCache",
    "FixedSceneReachableOverride",
    "build_cognitive_constraints",
    "cognitive_constraints_payload",
    "fixed_scene_footprint_settings",
    "load_fixed_scene_reachable_override",
    "observed_adjacent_transition_report",
    "occupancy_grid_version",
]
