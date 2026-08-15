#!/usr/bin/env python3
"""Generate a full-scene Rivermark occupancy map with Isaac's OMap Generator.

Engineering/operator tool only: the output is a convenience map for manual
inspection and map_server experiments, not qualification evidence.  The
self-developed top-down pipeline in ``rivermark_prepare.py`` remains the
reference implementation for the audited navigation map.

The script opens ``rivermark.usd`` (read-only; override with ``RIVERMARK_USD``
or ``--usd``), enables the official ``isaacsim.asset.gen.omap`` extension,
runs its PhysX ``Generator`` over the whole stage at 0.05 m/cell, and exports
a ROS map_server compatible pgm+yaml pair to
``${MAPPING_OUTPUT_ROOT:-~/Workspace/Bio_Nav/runs/operator_maps}/<version>/``
where ``<version>`` comes from ``RIVERMARK_MAP_VERSION`` or ``--map-version``.
Existing map files are never overwritten.  No ROS dependency.

Run with the Isaac Sim python, e.g.::

    "${ISAAC_PYTHON}" isaac_sim/tools/rivermark_occupancy_generate.py \
        --headless --map-version v1

Stage loading and session-layer collider authoring reuse the audited
``rivermark_prepare.py`` helpers (imported, not modified).
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import traceback

import numpy as np
import yaml

from rivermark_prepare import (
    SEED_OFFSETS_M,
    _author_session_mesh_colliders,
    _stage_inventory,
    _wait_for_stage,
)

DEFAULT_USD = Path("/home/lyb/Rivermark/rivermark.usd")
DEFAULT_OUTPUT_ROOT = Path("~/Workspace/Bio_Nav/runs/operator_maps")
# Audited free ground point (rivermark_prepare candidate A) used as the
# flood-fill origin; the OMap Generator requires an unoccupied origin.
DEFAULT_SEED_X = -12.0182
DEFAULT_SEED_Y = 151.603
DEFAULT_SEED_GROUND_Z = 6.21

OCCUPIED_PGM = 0
FREE_PGM = 254
UNKNOWN_PGM = 205


def _env_flag(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd",
        type=Path,
        default=Path(os.environ.get("RIVERMARK_USD", str(DEFAULT_USD))),
        help="source USD (read-only); default: RIVERMARK_USD or %(default)s",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(os.environ.get("MAPPING_OUTPUT_ROOT", str(DEFAULT_OUTPUT_ROOT))),
        help="map output root; default: MAPPING_OUTPUT_ROOT or %(default)s",
    )
    parser.add_argument(
        "--map-version",
        default=os.environ.get("RIVERMARK_MAP_VERSION", ""),
        help="required version subdirectory; default: RIVERMARK_MAP_VERSION",
    )
    parser.add_argument("--map-stem", default="rivermark_occupancy")
    parser.add_argument("--resolution-m", type=float, default=0.05)
    parser.add_argument("--seed-x", type=float, default=DEFAULT_SEED_X)
    parser.add_argument("--seed-y", type=float, default=DEFAULT_SEED_Y)
    parser.add_argument("--seed-ground-z", type=float, default=DEFAULT_SEED_GROUND_Z)
    parser.add_argument("--mapping-height-m", type=float, default=0.35)
    parser.add_argument("--minimum-z-offset-m", type=float, default=-0.25)
    parser.add_argument("--maximum-z-offset-m", type=float, default=1.65)
    parser.add_argument(
        "--min-free-fraction",
        type=float,
        default=0.01,
        help="retry with offset seeds when the primary seed frees less",
    )
    parser.add_argument("--load-timeout-s", type=float, default=300.0)
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without a window; also enabled by RIVERMARK_MAPGEN_HEADLESS=1",
    )
    return parser


def _output_paths(output_dir: Path, stem: str) -> dict[str, Path]:
    return {
        "pgm": output_dir / f"{stem}.pgm",
        "png": output_dir / f"{stem}.png",
        "yaml": output_dir / f"{stem}.yaml",
        "receipt": output_dir / f"{stem}.receipt.json",
    }


def _full_scene_bounds(stage) -> tuple[list[float], list[float], str, int]:
    """Whole-stage world bounds, tolerant of invalid instancing children."""

    from pxr import UsdGeom

    cache = UsdGeom.BBoxCache(0.0, ["default", "render", "proxy"])
    root = stage.GetDefaultPrim()
    if not root.IsValid():
        root = stage.GetPrimAtPath("/World")
    if not root.IsValid():
        root = stage.GetPseudoRoot()
    try:
        aligned = cache.ComputeWorldBound(root).ComputeAlignedRange()
        if not aligned.IsEmpty():
            return list(aligned.GetMin()), list(aligned.GetMax()), "whole_stage", 0
    except Exception:
        # Some Rivermark instancing prototypes contain an invalid child prim
        # (see rivermark_prepare._stage_inventory).  Fall back to a per-prim
        # union that recurses past the failing prim instead of giving up.
        pass

    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    skipped = 0

    def visit(prim) -> None:
        nonlocal skipped
        try:
            aligned = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not aligned.IsEmpty():
                for axis in range(3):
                    minimum[axis] = min(minimum[axis], aligned.GetMin()[axis])
                    maximum[axis] = max(maximum[axis], aligned.GetMax()[axis])
                return
        except Exception:
            pass
        children = prim.GetChildren()
        if not children:
            skipped += 1
            return
        for child in children:
            visit(child)

    for child in root.GetChildren():
        visit(child)
    if math.isinf(minimum[0]) or math.isinf(maximum[0]):
        raise RuntimeError("could not compute Rivermark world bounds")
    return minimum, maximum, "per_prim_union", skipped


def _generate_raw(
    generator,
    app,
    *,
    seed_xy: tuple[float, float],
    origin_z: float,
    bounds_min: list[float],
    bounds_max: list[float],
    minimum_z_offset_m: float,
    maximum_z_offset_m: float,
) -> np.ndarray:
    generator.set_transform(
        (float(seed_xy[0]), float(seed_xy[1]), float(origin_z)),
        (
            float(bounds_min[0] - seed_xy[0]),
            float(bounds_min[1] - seed_xy[1]),
            float(minimum_z_offset_m),
        ),
        (
            float(bounds_max[0] - seed_xy[0]),
            float(bounds_max[1] - seed_xy[1]),
            float(maximum_z_offset_m),
        ),
    )
    app.update()
    generator.generate2d()
    app.update()
    dimensions = tuple(int(value) for value in generator.get_dimensions())
    raw = np.asarray(generator.get_buffer(), dtype=np.float32)
    if len(dimensions) < 2 or raw.size != dimensions[0] * dimensions[1]:
        raise RuntimeError(f"generator produced invalid dimensions {dimensions}")
    return raw.reshape(dimensions[1], dimensions[0])


def _write_outputs(
    paths: dict[str, Path],
    raw: np.ndarray,
    *,
    resolution_m: float,
    origin_xy: tuple[float, float],
    receipt: dict[str, object],
) -> dict[str, object]:
    from PIL import Image

    paths["pgm"].parent.mkdir(parents=True, exist_ok=True)
    # Generator values are configured as occupied=1, free=0, unknown=0.5.
    pgm = np.full(raw.shape, UNKNOWN_PGM, dtype=np.uint8)
    pgm[np.isclose(raw, 1.0)] = OCCUPIED_PGM
    pgm[np.isclose(raw, 0.0)] = FREE_PGM
    # Raw rows advance from minimum y. ROS images store maximum y at row zero.
    pgm = pgm[::-1, :]
    Image.fromarray(pgm, mode="L").save(paths["pgm"])
    Image.fromarray(pgm, mode="L").save(paths["png"])
    paths["yaml"].write_text(
        yaml.safe_dump(
            {
                "image": paths["pgm"].name,
                "mode": "trinary",
                "resolution": float(resolution_m),
                "origin": [float(origin_xy[0]), float(origin_xy[1]), 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    occupied = int(np.count_nonzero(np.isclose(raw, 1.0)))
    free = int(np.count_nonzero(np.isclose(raw, 0.0)))
    summary = {
        "shape_yx": list(raw.shape),
        "origin_xy": list(origin_xy),
        "resolution_m": float(resolution_m),
        "occupied_cells": occupied,
        "free_cells": free,
        "unknown_cells": int(raw.size - occupied - free),
        "free_fraction": float(free / raw.size),
        "pgm": str(paths["pgm"]),
        "png": str(paths["png"]),
        "yaml": str(paths["yaml"]),
        "anchor_checks": {
            "bottom_left_cell_center": [
                float(origin_xy[0] + 0.5 * resolution_m),
                float(origin_xy[1] + 0.5 * resolution_m),
            ],
            "top_right_cell_center": [
                float(origin_xy[0] + (raw.shape[1] - 0.5) * resolution_m),
                float(origin_xy[1] + (raw.shape[0] - 0.5) * resolution_m),
            ],
        },
    }
    paths["receipt"].write_text(
        json.dumps({**receipt, **summary}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return summary


def run(args: argparse.Namespace) -> int:
    asset = args.usd.expanduser().resolve()
    if not asset.is_file():
        raise FileNotFoundError(asset)
    if not str(args.map_version).strip():
        raise ValueError(
            "map version is required: set RIVERMARK_MAP_VERSION or --map-version"
        )
    if args.resolution_m <= 0.0:
        raise ValueError("resolution must be positive")
    output_dir = (
        args.output_root.expanduser().resolve() / str(args.map_version).strip()
    )
    paths = _output_paths(output_dir, args.map_stem)
    existing = [str(path) for path in paths.values() if path.exists()]
    if existing:
        raise FileExistsError(
            "refusing to overwrite existing map products: " + ", ".join(existing)
        )
    headless = bool(args.headless) or _env_flag("RIVERMARK_MAPGEN_HEADLESS")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": headless, "multi_gpu": False})
    failed = False
    try:
        import omni.kit.app
        import omni.physx
        import omni.timeline
        import omni.usd
        from pxr import UsdPhysics

        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("isaacsim.asset.gen.omap", True)
        app.update()
        from isaacsim.asset.gen.omap.bindings import _omap

        context = omni.usd.get_context()
        if context.get_stage() is not None:
            if not context.can_close_stage() or not context.close_stage():
                raise RuntimeError("could not close Isaac startup stage")
        if not context.open_stage(str(asset)):
            raise RuntimeError(f"failed to open {asset}")
        stage = _wait_for_stage(app, context, asset, args.load_timeout_s)
        if not list(stage.Traverse()):
            raise RuntimeError("Rivermark stage is empty")
        inventory = _stage_inventory(stage)
        if inventory["up_axis"] != "Z" or inventory["meters_per_unit"] != 1.0:
            raise RuntimeError("Rivermark must be a Z-up metre stage")
        physics_scenes = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.IsA(UsdPhysics.Scene)
        ]
        if not physics_scenes:
            UsdPhysics.Scene.Define(stage, "/World/BioNavOMapPhysicsScene")
            app.update()
        if int(inventory["collision_prim_count"]) <= 0:
            raise RuntimeError("Rivermark has no collision geometry")

        (
            session_collider_layer,
            session_collider_count,
            hidden_annotation_count,
        ) = _author_session_mesh_colliders(stage, app)
        del session_collider_layer

        bounds_min, bounds_max, bounds_method, bounds_skipped = _full_scene_bounds(
            stage
        )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(30):
            app.update()
        generator = _omap.Generator(
            omni.physx.get_physx_interface(), context.get_stage_id()
        )
        generator.update_settings(float(args.resolution_m), 1.0, 0.0, 0.5)
        origin_z = float(args.seed_ground_z + args.mapping_height_m)
        seed = (float(args.seed_x), float(args.seed_y))
        seeds = [seed] + [
            (float(args.seed_x + dx), float(args.seed_y + dy))
            for dx, dy in SEED_OFFSETS_M
        ]
        raw = None
        selected_seed = None
        seed_trials = []
        for trial_seed in seeds:
            trial_raw = _generate_raw(
                generator,
                app,
                seed_xy=trial_seed,
                origin_z=origin_z,
                bounds_min=bounds_min,
                bounds_max=bounds_max,
                minimum_z_offset_m=float(args.minimum_z_offset_m),
                maximum_z_offset_m=float(args.maximum_z_offset_m),
            )
            free = int(np.count_nonzero(np.isclose(trial_raw, 0.0)))
            occupied = int(np.count_nonzero(np.isclose(trial_raw, 1.0)))
            seed_trials.append(
                {
                    "seed_xy": list(trial_seed),
                    "occupied_cells": occupied,
                    "free_cells": free,
                }
            )
            if raw is None or free > int(np.count_nonzero(np.isclose(raw, 0.0))):
                raw = trial_raw.copy()
                selected_seed = trial_seed
            if free / trial_raw.size >= float(args.min_free_fraction):
                break
        timeline.stop()
        if raw is None or selected_seed is None:
            raise RuntimeError("occupancy generation produced no seed trials")

        receipt = {
            "schema_version": 1,
            "classification": (
                "engineering_operator_tool; not qualification evidence"
            ),
            "created_at": datetime.now(timezone.utc).isoformat(),
            "generator": "isaacsim.asset.gen.omap.bindings._omap.Generator",
            "asset": str(asset),
            "map_version": str(args.map_version).strip(),
            "headless": headless,
            "seed_requested_xy": list(seed),
            "seed_selected_xy": list(selected_seed),
            "seed_ground_z_m": float(args.seed_ground_z),
            "omap_origin_z_m": origin_z,
            "seed_trials": seed_trials,
            "minimum_z_offset_m": float(args.minimum_z_offset_m),
            "maximum_z_offset_m": float(args.maximum_z_offset_m),
            "world_bounds_min": bounds_min,
            "world_bounds_max": bounds_max,
            "world_bounds_method": bounds_method,
            "world_bounds_skipped_prims": bounds_skipped,
            "occupancy_collision_mode": "visible_usd_meshes_in_session_layer",
            "session_collider_count": session_collider_count,
            "hidden_nonphysical_annotation_count": hidden_annotation_count,
        }
        summary = _write_outputs(
            paths,
            raw,
            resolution_m=float(args.resolution_m),
            origin_xy=(float(bounds_min[0]), float(bounds_min[1])),
            receipt=receipt,
        )
        print(json.dumps(summary, indent=2, sort_keys=True))
    except Exception:
        failed = True
        traceback.print_exc()
        raise
    finally:
        app.close(exit_code=1 if failed else 0)
    return 0


def main() -> int:
    return run(_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
