#!/usr/bin/env python3
"""Generate the Kujiale 0026 occupancy map directly with Isaac's OMap Generator.

Deterministic alternative to teleop SLAM mapping for the estimated-odometry
project: voxelize the static collision geometry of
``kujiale_0026_A_to_B_door_open.usd`` with the official
``isaacsim.asset.gen.omap`` extension and export a ROS map_server compatible
pgm+yaml pair in the calibrated ``warehouse_new`` map frame.

Frame contract (from ``isaac_sim/configs/environments/
kujiale_0026_A_to_B_door_open.spawn.yaml``):
``map = (2.9 - usd_x, -0.2 - usd_y)``, ``yaw_map = yaw_usd - 180 deg``.
The generator works in the USD world frame; the exported pgm is rotated 180
degrees around the anchor ``(2.9, -0.2)`` so the result lands directly in the
warehouse_new frame used by the spawn calibration.

Runtime-spawned obstacles (kujiale_long_range_static.yaml et al.) are not
part of the USD and stay out of the static map, matching the baseline map
philosophy.  Engineering/operator tool only: the output is not qualification
evidence.  Run with the Isaac Sim python::

    "${ISAAC_PYTHON}" isaac_sim/tools/kujiale_occupancy_generate.py \
        --headless --map-version kujiale_omap_v1
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path

import numpy as np
import yaml


DEFAULT_USD = Path(
    "/home/lyb/kujiale_usd_rooms_20260717/kujiale_0026/"
    "kujiale_0026_A_to_B_door_open.usd"
)
DEFAULT_OUTPUT_ROOT = Path("~/Workspace/Bio_Nav/runs/operator_maps")

# Calibrated map->USD frame anchor from the spawn poses file.
ANCHOR_X = 2.9
ANCHOR_Y = -0.2

OCCUPIED_PGM = 0
FREE_PGM = 254
UNKNOWN_PGM = 205

# Open living-room floor point (mapping_start usd pose) for the flood fill.
DEFAULT_SEED_X = ANCHOR_X
DEFAULT_SEED_Y = ANCHOR_Y
SEED_OFFSETS_M = (
    (0.3, 0.0),
    (-0.3, 0.0),
    (0.0, 0.3),
    (0.0, -0.3),
    (0.6, 0.6),
    (-0.6, -0.6),
)


def _env_flag(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd",
        type=Path,
        default=Path(os.environ.get("KUJIALE_USD", str(DEFAULT_USD))),
        help="source USD (read-only); default: KUJIALE_USD or %(default)s",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path(os.environ.get("MAPPING_OUTPUT_ROOT", str(DEFAULT_OUTPUT_ROOT))),
        help="map output root; default: MAPPING_OUTPUT_ROOT or %(default)s",
    )
    parser.add_argument(
        "--map-version",
        default=os.environ.get("KUJIALE_MAP_VERSION", ""),
        help="required version subdirectory; default: KUJIALE_MAP_VERSION",
    )
    parser.add_argument("--map-stem", default="kujiale_omap")
    parser.add_argument("--resolution-m", type=float, default=0.05)
    parser.add_argument("--seed-x", type=float, default=DEFAULT_SEED_X)
    parser.add_argument("--seed-y", type=float, default=DEFAULT_SEED_Y)
    parser.add_argument("--seed-ground-z", type=float, default=0.0)
    parser.add_argument("--mapping-height-m", type=float, default=0.35)
    parser.add_argument("--minimum-z-offset-m", type=float, default=-0.25)
    parser.add_argument("--maximum-z-offset-m", type=float, default=1.65)
    parser.add_argument(
        "--window-m",
        type=float,
        default=18.0,
        help="generation window size in metres around the anchor (default: 18)",
    )
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
        help="run without a window; also enabled by KUJIALE_MAPGEN_HEADLESS=1",
    )
    return parser


def _wait_for_stage(app, context, asset: Path, timeout_s: float):
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    del timeline
    waited = 0.0
    stage = context.get_stage()
    while stage is None and waited < timeout_s:
        app.update()
        stage = context.get_stage()
        waited += 0.05
    if stage is None:
        raise RuntimeError(f"timed out loading {asset}")
    return stage


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


def run(args: argparse.Namespace) -> int:
    asset = args.usd.expanduser().resolve()
    if not asset.is_file():
        raise FileNotFoundError(asset)
    if not str(args.map_version).strip():
        raise ValueError(
            "map version is required: set KUJIALE_MAP_VERSION or --map-version"
        )
    if args.resolution_m <= 0.0:
        raise ValueError("resolution must be positive")
    output_dir = (
        args.output_root.expanduser().resolve() / str(args.map_version).strip()
    )
    paths = {
        "pgm": output_dir / f"{args.map_stem}.pgm",
        "png": output_dir / f"{args.map_stem}.png",
        "yaml": output_dir / f"{args.map_stem}.yaml",
        "receipt": output_dir / f"{args.map_stem}.receipt.json",
    }
    existing = [str(path) for path in paths.values() if path.exists()]
    if existing:
        raise FileExistsError(
            "refusing to overwrite existing map products: " + ", ".join(existing)
        )
    headless = bool(args.headless) or _env_flag("KUJIALE_MAPGEN_HEADLESS")

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
            raise RuntimeError("Kujiale stage is empty")
        physics_scenes = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.IsA(UsdPhysics.Scene)
        ]
        if not physics_scenes:
            UsdPhysics.Scene.Define(stage, "/World/BioNavOMapPhysicsScene")
            app.update()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(30):
            app.update()
        generator = _omap.Generator(
            omni.physx.get_physx_interface(), context.get_stage_id()
        )
        generator.update_settings(float(args.resolution_m), 1.0, 0.0, 0.5)
        half = 0.5 * float(args.window_m)
        center_x = float(args.seed_x)
        center_y = float(args.seed_y)
        bounds_min = [center_x - half, center_y - half, 0.0]
        bounds_max = [center_x + half, center_y + half, 0.0]
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

        # Generator values: occupied=1, free=0, unknown=0.5.
        pgm = np.full(raw.shape, UNKNOWN_PGM, dtype=np.uint8)
        pgm[np.isclose(raw, 1.0)] = OCCUPIED_PGM
        pgm[np.isclose(raw, 0.0)] = FREE_PGM
        # Raw rows advance from minimum y; ROS images store maximum y at row 0.
        pgm = pgm[::-1, :]
        usd_origin_x = float(bounds_min[0])
        usd_origin_y = float(bounds_min[1])
        height, width = pgm.shape
        # Rotate 180 degrees around the calibrated anchor so the map lands in
        # the warehouse_new frame: map = anchor - usd per axis.
        pgm = pgm[::-1, ::-1]
        resolution = float(args.resolution_m)
        map_origin_x = float(ANCHOR_X - usd_origin_x - width * resolution)
        map_origin_y = float(ANCHOR_Y - usd_origin_y - height * resolution)

        from PIL import Image

        paths["pgm"].parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(pgm, mode="L").save(paths["pgm"])
        Image.fromarray(pgm, mode="L").save(paths["png"])
        paths["yaml"].write_text(
            yaml.safe_dump(
                {
                    "image": paths["pgm"].name,
                    "mode": "trinary",
                    "resolution": resolution,
                    "origin": [map_origin_x, map_origin_y, 0.0],
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
        spawn_col = int((0.45 - map_origin_x) / resolution)
        spawn_row = height - 1 - int((-5.35 - map_origin_y) / resolution)
        spawn_value = (
            int(pgm[spawn_row, spawn_col])
            if 0 <= spawn_row < height and 0 <= spawn_col < width
            else None
        )
        receipt = {
            "schema_version": 1,
            "classification": "engineering/operator artifact; not qualification evidence",
            "created_at": datetime.now(timezone.utc).isoformat(),
            "source_usd": str(asset),
            "frame_transform": (
                "map = (2.9 - usd_x, -0.2 - usd_y), yaw_map = yaw_usd - 180 deg"
            ),
            "generator": "isaacsim.asset.gen.omap Generator",
            "resolution_m": resolution,
            "z_offsets_m": [
                float(args.minimum_z_offset_m),
                float(args.maximum_z_offset_m),
            ],
            "seed_xy": list(selected_seed),
            "seed_trials": seed_trials,
            "shape_yx": [height, width],
            "origin_map_xy": [map_origin_x, map_origin_y],
            "occupied_cells": occupied,
            "free_cells": free,
            "unknown_cells": int(raw.size - occupied - free),
            "free_fraction": float(free / raw.size),
            "spawn_cell_check": {
                "map_xy": [0.45, -5.35],
                "row_col": [spawn_row, spawn_col],
                "pgm_value": spawn_value,
                "expected": FREE_PGM,
            },
            "pgm": str(paths["pgm"]),
            "yaml": str(paths["yaml"]),
        }
        paths["receipt"].write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(receipt, indent=2, sort_keys=True))
    except Exception:
        failed = True
        raise
    finally:
        app.close()
    return 1 if failed else 0


def main() -> int:
    return run(_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
