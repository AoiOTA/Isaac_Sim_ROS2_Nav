"""Export a physical A->B cognitive-constraint pair for V3.10 benchmarks."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from .cognitive_constraints import build_cognitive_constraints
from .defaults import load_engineering_defaults
from .map_io import OccupancyMap, load_occupancy_map
from .regions import load_region_config


def _map_identity(map_yaml: Path) -> str:
    """Bind the benchmark pair to the immutable base occupancy artifact."""

    digest = hashlib.sha256()
    digest.update(map_yaml.read_bytes())
    metadata = __import__("yaml").safe_load(map_yaml.read_text(encoding="utf-8"))
    image = Path(str(metadata["image"]))
    if not image.is_absolute():
        image = map_yaml.parent / image
    digest.update(image.resolve().read_bytes())
    return digest.hexdigest()


def _state_centers_canvas() -> np.ndarray:
    axis = -8.0 + (np.arange(16, dtype=np.float64) + 0.5)
    xx, yy = np.meshgrid(axis, axis, indexing="xy")
    return np.column_stack((xx.reshape(-1), yy.reshape(-1)))


def _canvas_points_to_map(
    points: np.ndarray, t_map_canvas: np.ndarray
) -> np.ndarray:
    """Transform canvas XY into the continuous global map frame."""

    transform = np.asarray(t_map_canvas, dtype=np.float64).reshape(3, 3)
    inverse = np.linalg.inv(transform)
    homogeneous = np.column_stack(
        (np.asarray(points, dtype=np.float64).reshape(-1, 2), np.ones(len(points)))
    )
    mapped = (inverse @ homogeneous.T).T
    return mapped[:, :2] / mapped[:, 2:3]


def persistent_local_change(
    occupancy: OccupancyMap,
    center_xy: tuple[float, float],
    *,
    radius_m: float,
) -> OccupancyMap:
    """Add one circular, persistent occupied region to a copied map."""

    if radius_m <= 0.0:
        raise ValueError("radius_m must be positive")
    rows, columns = np.indices(occupancy.free.shape)
    world_x = occupancy.origin_xy_m[0] + (columns + 0.5) * occupancy.resolution_m
    world_y = (
        occupancy.origin_xy_m[1]
        + (occupancy.free.shape[0] - rows - 0.5) * occupancy.resolution_m
    )
    changed = (world_x - center_xy[0]) ** 2 + (world_y - center_xy[1]) ** 2
    changed = changed <= radius_m**2
    free = np.array(occupancy.free, copy=True)
    free[changed] = False
    return OccupancyMap(
        free=free,
        resolution_m=occupancy.resolution_m,
        origin_xy_m=occupancy.origin_xy_m,
        map_version=f"{occupancy.map_version}:persistent-b",
        yaml_path=occupancy.yaml_path,
    )


def build_physical_pair(
    map_yaml: str | Path,
    engineering_defaults: str | Path,
    *,
    obstacle_radius_m: float = 0.48,
    t_map_canvas: np.ndarray | None = None,
    cognitive_tile_id: str | None = None,
) -> tuple[object, object, dict[str, object]]:
    """Build A and a local-persistent B inside one real cognitive region."""

    map_path = Path(map_yaml).expanduser().resolve()
    defaults = load_engineering_defaults(engineering_defaults)
    occupancy_a = load_occupancy_map(
        map_path,
        unknown_is_occupied=bool(defaults["graph"]["unknown_is_occupied"]),
    )
    identity = _map_identity(map_path)
    transform = (
        np.eye(3, dtype=np.float64)
        if t_map_canvas is None
        else np.asarray(t_map_canvas, dtype=np.float64).reshape(3, 3)
    )
    value_a = build_cognitive_constraints(
        occupancy_a,
        map_version=identity,
        graph_revision=1,
        footprint_settings=defaults["footprint"],
        persistent_confirmed=True,
        t_map_canvas=transform,
        cognitive_tile_id=cognitive_tile_id,
    )
    degree = np.zeros(256, dtype=np.int64)
    for source, _target in value_a.verified_transitions:
        degree[int(source)] += 1
    centers = _canvas_points_to_map(_state_centers_canvas(), transform)
    candidates = np.flatnonzero(value_a.reachable_state_mask & (degree >= 3))
    if candidates.size == 0:
        raise RuntimeError("warehouse map has no suitable reachable interior state")
    # Keep the local change central and reproducible while requiring a state
    # with multiple physically swept exits.
    target = int(candidates[np.argmin(np.linalg.norm(centers[candidates], axis=1))])
    center = tuple(float(item) for item in centers[target])
    occupancy_b = persistent_local_change(
        occupancy_a, center, radius_m=obstacle_radius_m
    )
    value_b = build_cognitive_constraints(
        occupancy_b,
        map_version=identity,
        graph_revision=2,
        footprint_settings=defaults["footprint"],
        persistent_confirmed=True,
        stable_duration_s=float(
            defaults["structural_updates"]["stable_for_s"]
        ),
        t_map_canvas=transform,
        cognitive_tile_id=value_a.cognitive_tile_id,
    )
    edges_a = {tuple(map(int, pair)) for pair in value_a.verified_transitions}
    edges_b = {tuple(map(int, pair)) for pair in value_b.verified_transitions}
    removed_states = np.flatnonzero(
        value_a.reachable_state_mask & ~value_b.reachable_state_mask
    )
    if target not in removed_states or not edges_a.difference(edges_b):
        raise RuntimeError("persistent obstacle did not change cognitive topology")
    metadata: dict[str, object] = {
        "schema": "bio_nav.v310.physical_cognitive_pair.v1",
        "source_map_yaml": str(map_path),
        "map_version": identity,
        "cognitive_tile_id": value_a.cognitive_tile_id,
        "obstacle_center_map_m": list(center),
        "obstacle_radius_m": float(obstacle_radius_m),
        "obstacle_state": target,
        "reachable_a": int(value_a.reachable_state_mask.sum()),
        "reachable_b": int(value_b.reachable_state_mask.sum()),
        "shared_states": int(
            (value_a.reachable_state_mask & value_b.reachable_state_mask).sum()
        ),
        "removed_states": int(len(removed_states)),
        "new_states": int(
            (~value_a.reachable_state_mask & value_b.reachable_state_mask).sum()
        ),
        "transitions_a": len(edges_a),
        "transitions_b": len(edges_b),
        "added_transitions": len(edges_b.difference(edges_a)),
        "removed_transitions": len(edges_a.difference(edges_b)),
        "persistent_confirmed": bool(value_b.persistent_confirmed),
    }
    return value_a, value_b, metadata


def export_physical_pair(
    map_yaml: str | Path,
    engineering_defaults: str | Path,
    output_prefix: str | Path,
    *,
    obstacle_radius_m: float = 0.48,
    t_map_canvas: np.ndarray | None = None,
    cognitive_tile_id: str | None = None,
) -> dict[str, Path]:
    """Write NPZ, JSON and a human-readable topology delta PNG."""

    value_a, value_b, metadata = build_physical_pair(
        map_yaml,
        engineering_defaults,
        obstacle_radius_m=obstacle_radius_m,
        t_map_canvas=t_map_canvas,
        cognitive_tile_id=cognitive_tile_id,
    )
    prefix = Path(output_prefix).expanduser().resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    npz_path = prefix.with_suffix(".npz")
    json_path = prefix.with_suffix(".json")
    png_path = prefix.with_suffix(".png")
    np.savez_compressed(
        npz_path,
        map_version=np.asarray(value_a.map_version),
        cognitive_tile_id=np.asarray(value_a.cognitive_tile_id),
        t_map_canvas=value_a.t_map_canvas,
        reachable_a=value_a.reachable_state_mask,
        reachable_b=value_b.reachable_state_mask,
        transitions_a=value_a.verified_transitions,
        transitions_b=value_b.verified_transitions,
    )
    json_path.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    import matplotlib

    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.patches import Circle

    defaults = load_engineering_defaults(engineering_defaults)
    occupancy_a = load_occupancy_map(
        map_yaml,
        unknown_is_occupied=bool(defaults["graph"]["unknown_is_occupied"]),
    )
    center = tuple(float(value) for value in metadata["obstacle_center_map_m"])
    occupancy_b = persistent_local_change(
        occupancy_a, center, radius_m=obstacle_radius_m
    )
    state_a = value_a.reachable_state_mask.reshape(16, 16)
    state_b = value_b.reachable_state_mask.reshape(16, 16)
    delta = state_a.astype(np.int8) - state_b.astype(np.int8)
    figure, axes = plt.subplots(2, 3, figsize=(12.5, 7.0), constrained_layout=True)
    extent = (
        occupancy_a.origin_xy_m[0],
        occupancy_a.origin_xy_m[0]
        + occupancy_a.free.shape[1] * occupancy_a.resolution_m,
        occupancy_a.origin_xy_m[1],
        occupancy_a.origin_xy_m[1]
        + occupancy_a.free.shape[0] * occupancy_a.resolution_m,
    )
    occupancy_delta = occupancy_a.free & ~occupancy_b.free
    for axis, image, title in zip(
        axes[0],
        (occupancy_a.free, occupancy_b.free, occupancy_delta),
        ("Region parent A", "Region persistent B", "Physical A-B occupied"),
    ):
        axis.imshow(
            image,
            origin="upper",
            extent=extent,
            cmap="gray" if title != "Physical A-B occupied" else "magma",
            interpolation="nearest",
        )
        axis.add_patch(
            Circle(center, obstacle_radius_m, fill=False, color="#ef4444", linewidth=1.5)
        )
        axis.set_title(title)
        axis.set_xlabel("map x (m)")
        axis.set_ylabel("map y (m)")
        axis.set_aspect("equal", adjustable="box")
    for axis, image, title in zip(
        axes[1],
        (state_a, state_b, delta),
        ("Parent A reachable", "Full B reachable", "A-B removed"),
    ):
        view = axis.imshow(image, origin="lower", interpolation="nearest")
        axis.set_title(title)
        axis.set_xlabel("canvas column")
        axis.set_ylabel("canvas row")
        figure.colorbar(view, ax=axis, fraction=0.046)
    figure.suptitle(
        "V3.10 physical persistent local structural change\n"
        f"state {metadata['obstacle_state']} at {metadata['obstacle_center_map_m']} m"
    )
    figure.savefig(png_path, dpi=180)
    plt.close(figure)
    return {"npz": npz_path, "json": json_path, "png": png_path}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--engineering-defaults", required=True)
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--obstacle-radius-m", type=float, default=0.48)
    parser.add_argument("--region-config")
    parser.add_argument("--region-id")
    arguments = parser.parse_args()
    transform = None
    tile_id = None
    if bool(arguments.region_config) != bool(arguments.region_id):
        parser.error("--region-config and --region-id must be supplied together")
    if arguments.region_config:
        config = load_region_config(arguments.region_config)
        matches = [
            region for region in config.regions
            if region.region_id == arguments.region_id
        ]
        if len(matches) != 1:
            parser.error(
                f"unknown region-id {arguments.region_id!r}; "
                f"available={tuple(region.region_id for region in config.regions)}"
            )
        transform = matches[0].t_map_canvas
        tile_id = matches[0].region_id
    paths = export_physical_pair(
        arguments.map_yaml,
        arguments.engineering_defaults,
        arguments.output_prefix,
        obstacle_radius_m=arguments.obstacle_radius_m,
        t_map_canvas=transform,
        cognitive_tile_id=tile_id,
    )
    print(json.dumps({key: str(value) for key, value in paths.items()}, indent=2))


if __name__ == "__main__":
    main()
