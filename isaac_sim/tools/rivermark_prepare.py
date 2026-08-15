#!/usr/bin/env python3
"""Inspect Rivermark and export collision-derived candidate occupancy maps."""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
import traceback

import numpy as np
import yaml


CANDIDATES = {
    # x, y, audited local ground z from collision bounds.
    "A": (-12.0182, 151.603, 6.21),
    "B": (-20.27893, 41.94005, 5.90),
}
SEED_OFFSETS_M = (
    (10.0, 10.0),
    (20.0, 0.0),
    (-20.0, 0.0),
    (0.0, 20.0),
    (0.0, -20.0),
    (25.0, 25.0),
    (-25.0, -25.0),
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd", type=Path, default=Path("/home/lyb/Rivermark/rivermark.usd")
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--candidate", choices=("A", "B", "all"), default="all")
    parser.add_argument("--window-m", type=float, default=80.0)
    parser.add_argument("--resolution-m", type=float, default=0.05)
    parser.add_argument("--mapping-height-m", type=float, default=0.35)
    parser.add_argument("--minimum-z-offset-m", type=float, default=-0.25)
    parser.add_argument("--maximum-z-offset-m", type=float, default=1.65)
    parser.add_argument("--load-timeout-s", type=float, default=300.0)
    parser.add_argument("--render-size", type=int, default=1024)
    parser.add_argument("--max-traversable-step-m", type=float, default=0.03)
    parser.add_argument("--max-traversable-slope", type=float, default=0.45)
    parser.add_argument("--terrain-connect-step-m", type=float, default=0.25)
    parser.add_argument("--terrain-connect-slope", type=float, default=5.0)
    parser.add_argument("--height-barrier-thickness-cells", type=int, default=0)
    parser.add_argument(
        "--render-only",
        action="store_true",
        help="render aligned top-down RGB without generating occupancy",
    )
    return parser


def _wait_for_stage(app, context, expected_path: Path, timeout_s: float):
    deadline = time.monotonic() + float(timeout_s)
    stable = 0
    stable_stage = None
    while time.monotonic() < deadline:
        app.update()
        pending = int(context.get_stage_loading_status()[2])
        stage = context.get_stage()
        layer_path = ""
        if stage is not None:
            layer = stage.GetRootLayer()
            layer_path = layer.realPath or layer.identifier
        matches_asset = (
            bool(layer_path)
            and not layer_path.startswith("anon:")
            and Path(layer_path).resolve() == expected_path
        )
        root_prims = [] if stage is None else list(stage.GetPseudoRoot().GetChildren())
        has_content = bool(root_prims) and (
            stage.GetPrimAtPath("/World").IsValid() or stage.GetDefaultPrim().IsValid()
        )
        payloads_loaded = has_content and all(
            not prim.HasPayload() or prim.IsLoaded() for prim in stage.TraverseAll()
        )
        if pending == 0 and matches_asset and has_content and payloads_loaded:
            stable = stable + 1 if stage is stable_stage else 1
            stable_stage = stage
            if stable >= 10:
                return stage
        else:
            stable = 0
            stable_stage = None
    raise TimeoutError("Rivermark payloads did not finish loading")


def _stage_inventory(stage) -> dict[str, object]:
    from pxr import UsdGeom, UsdPhysics

    counts: dict[str, int] = {}
    collision_paths = []
    rigid_body_count = 0
    for prim in stage.Traverse():
        name = prim.GetTypeName() or "untyped"
        counts[name] = counts.get(name, 0) + 1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision_paths.append(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_count += 1
    bound_cache = UsdGeom.BBoxCache(0.0, ["default", "render", "proxy"])
    default_prim = stage.GetDefaultPrim()
    bound_prim = default_prim
    if not bound_prim.IsValid():
        bound_prim = stage.GetPrimAtPath("/World")
    if not bound_prim.IsValid():
        bound_prim = stage.GetPseudoRoot()
    try:
        bounds = bound_cache.ComputeWorldBound(bound_prim).ComputeAlignedRange()
        bounds_min = list(bounds.GetMin())
        bounds_max = list(bounds.GetMax())
        bounds_error = None
    except Exception as exc:
        # Some Rivermark instancing prototypes contain an invalid child prim.
        # Fixed candidate ROIs and PhysX collision geometry do not depend on a
        # whole-stage render bound, so preserve the diagnostic and continue.
        bounds_min = None
        bounds_max = None
        bounds_error = f"{type(exc).__name__}: {exc}"
    return {
        "default_prim": (
            str(default_prim.GetPath()) if default_prim.IsValid() else None
        ),
        "inventory_bound_prim": str(bound_prim.GetPath()),
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
        "world_bounds_min": bounds_min,
        "world_bounds_max": bounds_max,
        "world_bounds_error": bounds_error,
        "prim_count": int(sum(counts.values())),
        "prim_type_counts": counts,
        "collision_prim_count": len(collision_paths),
        "rigid_body_prim_count": rigid_body_count,
        "collision_path_examples": collision_paths[:100],
    }


def _write_map(
    output_dir: Path,
    candidate: str,
    raw: np.ndarray,
    *,
    resolution_m: float,
    origin_xy: tuple[float, float],
) -> dict[str, object]:
    from PIL import Image

    output_dir.mkdir(parents=True, exist_ok=True)
    # Generator values are configured as occupied=1, free=0, unknown=0.5.
    pgm = np.full(raw.shape, 205, dtype=np.uint8)
    pgm[np.isclose(raw, 1.0)] = 0
    pgm[np.isclose(raw, 0.0)] = 254
    # Raw rows advance from minimum y. ROS images store maximum y at row zero.
    pgm = pgm[::-1, :]
    stem = f"candidate_{candidate}_occupancy"
    pgm_path = output_dir / f"{stem}.pgm"
    png_path = output_dir / f"{stem}.png"
    yaml_path = output_dir / f"{stem}.yaml"
    Image.fromarray(pgm, mode="L").save(pgm_path)
    Image.fromarray(pgm, mode="L").save(png_path)
    yaml_path.write_text(
        yaml.safe_dump(
            {
                "image": pgm_path.name,
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
    return {
        "shape_yx": list(raw.shape),
        "origin_xy": list(origin_xy),
        "resolution_m": float(resolution_m),
        "occupied_cells": occupied,
        "free_cells": free,
        "unknown_cells": int(raw.size - occupied - free),
        "free_fraction": float(free / raw.size),
        "pgm": str(pgm_path),
        "png": str(png_path),
        "yaml": str(yaml_path),
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


def _topdown_products(
    stage,
    center,
    window_m: float,
    size: int,
    rgb_destination: Path,
    depth_destination: Path | None = None,
):
    import omni.replicator.core as rep
    from omni.replicator.core.functional import write_image
    from pxr import Gf, UsdGeom

    camera = UsdGeom.Camera.Define(stage, "/World/BioNavRivermarkTopdown")
    camera.CreateProjectionAttr().Set("orthographic")
    # USD camera aperture is authored in tenths of a stage unit. Rivermark is
    # metre-scaled, so an 80 m orthographic window requires aperture 800.
    camera.CreateHorizontalApertureAttr().Set(float(window_m) * 10.0)
    camera.CreateVerticalApertureAttr().Set(float(window_m) * 10.0)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 1000.0))
    xformable = UsdGeom.Xformable(camera.GetPrim())
    xformable.ClearXformOpOrder()
    transform = Gf.Matrix4d(1.0).SetLookAt(
        Gf.Vec3d(float(center[0]), float(center[1]), float(center[2]) + 150.0),
        Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])),
        Gf.Vec3d(0.0, 1.0, 0.0),
    ).GetInverse().GetOrthonormalized()
    xformable.MakeMatrixXform().Set(transform)
    render_product = rep.create.render_product(
        str(camera.GetPath()),
        resolution=(int(size), int(size)),
        force_new=True,
        name="BioNavRivermarkTopdown",
    )
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    depth_annotator = (
        rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        if depth_destination is not None
        else None
    )
    rgb_annotator.attach(render_product)
    if depth_annotator is not None:
        depth_annotator.attach(render_product)
    try:
        rep.orchestrator.set_capture_on_play(False)
        rep.orchestrator.step(rt_subframes=8, delta_time=0.0, pause_timeline=True)
        write_image(path=str(rgb_destination), data=rgb_annotator.get_data())
        if depth_annotator is not None and depth_destination is not None:
            depth = depth_annotator.get_data()
            if isinstance(depth, dict):
                depth = depth.get("data")
            depth = np.asarray(depth, dtype=np.float32).squeeze()
            if depth.shape != (int(size), int(size)):
                raise RuntimeError(f"unexpected top-down depth shape {depth.shape}")
            np.save(depth_destination, depth)
    finally:
        rgb_annotator.detach()
        if depth_annotator is not None:
            depth_annotator.detach()
        destroy = getattr(render_product, "destroy", None)
        if callable(destroy):
            destroy()


def _dilate(mask: np.ndarray, iterations: int) -> np.ndarray:
    result = np.asarray(mask, dtype=bool).copy()
    for _ in range(max(0, int(iterations))):
        padded = np.pad(result, 1, mode="constant", constant_values=False)
        expanded = result.copy()
        for row_offset in range(3):
            for column_offset in range(3):
                expanded |= padded[
                    row_offset : row_offset + result.shape[0],
                    column_offset : column_offset + result.shape[1],
                ]
        result = expanded
    return result


def _height_refined_occupancy(
    raw: np.ndarray,
    depth: np.ndarray,
    *,
    center: tuple[float, float, float],
    seed_xy: tuple[float, float],
    window_m: float,
    resolution_m: float,
    max_step_m: float,
    max_slope: float,
    terrain_connect_step_m: float,
    terrain_connect_slope: float,
    barrier_thickness_cells: int,
) -> tuple[
    np.ndarray,
    dict[str, object],
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """Combine fixed-height collision OMap with 2.5D reachability."""

    if raw.shape != depth.shape:
        raise ValueError(f"occupancy/depth shape mismatch: {raw.shape} vs {depth.shape}")
    camera_z = float(center[2]) + 150.0
    height = camera_z - depth
    valid = np.isfinite(height) & (depth > 0.0)
    half = 0.5 * float(window_m)
    column = int((float(seed_xy[0]) - (float(center[0]) - half)) / resolution_m)
    row = int(((float(center[1]) + half) - float(seed_xy[1])) / resolution_m)
    radius = max(1, int(math.ceil(2.0 / resolution_m)))
    row0, row1 = max(0, row - radius), min(height.shape[0], row + radius + 1)
    col0, col1 = max(0, column - radius), min(height.shape[1], column + radius + 1)
    local_valid = valid[row0:row1, col0:col1] & (
        np.abs(height[row0:row1, col0:col1] - float(center[2])) <= 1.0
    )
    if not np.any(local_valid):
        raise RuntimeError(f"no ground-like depth sample near seed {seed_xy}")
    local_rows, local_columns = np.nonzero(local_valid)
    local_distance = (local_rows + row0 - row) ** 2 + (
        local_columns + col0 - column
    ) ** 2
    nearest = int(np.argmin(local_distance))
    seed_row = int(local_rows[nearest] + row0)
    seed_column = int(local_columns[nearest] + col0)

    reachable = np.zeros(height.shape, dtype=bool)
    reachable[seed_row, seed_column] = True
    queue = deque(((seed_row, seed_column),))
    neighbor_offsets = (
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1), (0, 1),
        (1, -1), (1, 0), (1, 1),
    )
    while queue:
        current_row, current_column = queue.popleft()
        current_height = float(height[current_row, current_column])
        for row_offset, column_offset in neighbor_offsets:
            next_row = current_row + row_offset
            next_column = current_column + column_offset
            if (
                next_row < 0
                or next_row >= height.shape[0]
                or next_column < 0
                or next_column >= height.shape[1]
                or reachable[next_row, next_column]
                or not valid[next_row, next_column]
            ):
                continue
            distance_m = resolution_m * math.hypot(row_offset, column_offset)
            allowed_delta = min(
                float(terrain_connect_step_m),
                float(terrain_connect_slope) * distance_m,
            )
            if abs(float(height[next_row, next_column]) - current_height) <= allowed_delta:
                reachable[next_row, next_column] = True
                queue.append((next_row, next_column))

    # Global reachability alone cannot represent a curb when the raised
    # pavement is reachable through a ramp elsewhere. Detect every local
    # height discontinuity independently and mark the higher-side cell, while
    # retaining the lower road cell for travel parallel to the curb.
    steep_boundary = np.zeros(height.shape, dtype=bool)
    for row_offset, column_offset in ((0, 1), (1, 0), (1, 1), (1, -1)):
        source_rows = slice(
            max(0, -row_offset), height.shape[0] - max(0, row_offset)
        )
        source_columns = slice(
            max(0, -column_offset), height.shape[1] - max(0, column_offset)
        )
        target_rows = slice(
            max(0, row_offset), height.shape[0] - max(0, -row_offset)
        )
        target_columns = slice(
            max(0, column_offset), height.shape[1] - max(0, -column_offset)
        )
        source_height = height[source_rows, source_columns]
        target_height = height[target_rows, target_columns]
        pair_valid = valid[source_rows, source_columns] & valid[
            target_rows, target_columns
        ]
        distance_m = resolution_m * math.hypot(row_offset, column_offset)
        allowed_delta = min(float(max_step_m), float(max_slope) * distance_m)
        delta = target_height - source_height
        steep = pair_valid & (np.abs(delta) > allowed_delta)
        source_higher = steep & (delta < 0.0)
        target_higher = steep & (delta > 0.0)
        steep_boundary[source_rows, source_columns] |= source_higher
        steep_boundary[target_rows, target_columns] |= target_higher

    height_nontraversable = valid & ~reachable
    height_barrier = steep_boundary.copy()
    if int(barrier_thickness_cells) > 0:
        height_barrier = _dilate(height_barrier, barrier_thickness_cells)

    omap_image = raw[::-1, :]
    omap_occupied = np.isclose(omap_image, 1.0)
    omap_free = np.isclose(omap_image, 0.0)
    # A fixed horizontal OMap band falsely marks raised but traversable roads
    # as occupied. Terrain-connected depth wins for those cells; the OMap
    # remains authoritative only for geometry not connected to the ground.
    occupied = (omap_occupied & ~reachable) | height_barrier
    free = reachable & ~height_barrier
    free &= ~occupied
    refined_image = np.full(raw.shape, 0.5, dtype=np.float32)
    refined_image[free] = 0.0
    refined_image[occupied] = 1.0
    diagnostic = {
        "method": "orthographic_depth_reachability_intersect_physx_omap",
        "seed_requested_xy": list(seed_xy),
        "seed_pixel_rc": [seed_row, seed_column],
        "seed_height_m": float(height[seed_row, seed_column]),
        "max_step_m": float(max_step_m),
        "max_slope": float(max_slope),
        "terrain_connect_step_m": float(terrain_connect_step_m),
        "terrain_connect_slope": float(terrain_connect_slope),
        "barrier_thickness_cells": int(barrier_thickness_cells),
        "height_valid_cells": int(np.count_nonzero(valid)),
        "height_reachable_cells": int(np.count_nonzero(reachable)),
        "height_nontraversable_cells": int(
            np.count_nonzero(height_nontraversable)
        ),
        "steep_boundary_cells": int(np.count_nonzero(steep_boundary)),
        "height_barrier_cells": int(np.count_nonzero(height_barrier)),
        "combined_free_cells": int(np.count_nonzero(free)),
        "combined_occupied_cells": int(np.count_nonzero(occupied)),
    }
    return (
        refined_image[::-1, :],
        diagnostic,
        height,
        reachable,
        steep_boundary,
        height_nontraversable,
    )


def _author_session_mesh_colliders(stage, app) -> tuple[object, int, int]:
    """Mirror OMap UI's original-USD-mesh mode in an anonymous session layer."""

    from omni.physx.scripts import utils
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    layer = Sdf.Layer.CreateAnonymous("rivermark_occupancy_map")
    stage.GetSessionLayer().subLayerPaths.append(layer.identifier)
    authored = 0
    hidden_annotations = 0
    with Usd.EditContext(stage, layer):
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.BasisCurves) or prim.IsA(UsdGeom.NurbsCurves) or prim.IsA(
                UsdGeom.Points
            ):
                UsdGeom.Imageable(prim).MakeInvisible()
                hidden_annotations += 1
        with Sdf.ChangeBlock():
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.HasAPI(
                    UsdPhysics.RigidBodyAPI
                ):
                    utils.removePhysics(prim)
        app.update()
        with Sdf.ChangeBlock():
            for prim in stage.Traverse():
                imageable = UsdGeom.Imageable(prim)
                if imageable and imageable.ComputeVisibility(
                    Usd.TimeCode.Default()
                ) == UsdGeom.Tokens.invisible:
                    continue
                if prim.IsA(UsdGeom.Mesh):
                    points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
                    if points is None or len(points) == 0:
                        continue
                try:
                    solid_gprim = any(
                        prim.IsA(schema)
                        for schema in (
                            UsdGeom.Mesh,
                            UsdGeom.Cube,
                            UsdGeom.Sphere,
                            UsdGeom.Cylinder,
                            UsdGeom.Cone,
                            UsdGeom.Capsule,
                        )
                    )
                    if prim.HasAPI(UsdPhysics.CollisionAPI):
                        if not solid_gprim and not (
                            prim.IsA(UsdGeom.Xformable) and prim.IsInstanceable()
                        ):
                            continue
                        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                            approximation = (
                                UsdPhysics.MeshCollisionAPI(prim)
                                .GetApproximationAttr()
                                .Get()
                            )
                            if approximation == "none":
                                continue
                        if prim.IsA(UsdGeom.Gprim):
                            if prim.IsInstanceable():
                                UsdPhysics.CollisionAPI.Apply(prim)
                                UsdPhysics.MeshCollisionAPI.Apply(prim)
                            else:
                                utils.setCollider(prim, "none")
                            authored += 1
                    elif prim.IsA(UsdGeom.Xformable) and prim.IsInstanceable():
                        UsdPhysics.CollisionAPI.Apply(prim)
                        UsdPhysics.MeshCollisionAPI.Apply(prim)
                        authored += 1
                    elif solid_gprim:
                        utils.setCollider(prim, "none")
                        authored += 1
                except Exception:
                    continue
    app.update()
    return layer, authored, hidden_annotations


def run(args: argparse.Namespace) -> int:
    asset = args.usd.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if not asset.is_file():
        raise FileNotFoundError(asset)
    if args.window_m <= 0.0 or args.resolution_m <= 0.0:
        raise ValueError("window and resolution must be positive")

    from isaacsim import SimulationApp

    app = SimulationApp(
        {"headless": True, "renderer": "RaytracedLighting", "multi_gpu": False}
    )
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
        if args.render_only:
            output_dir.mkdir(parents=True, exist_ok=True)
            selected = CANDIDATES if args.candidate == "all" else {
                args.candidate: CANDIDATES[args.candidate]
            }
            rendered = {}
            for candidate, center in selected.items():
                path = output_dir / f"candidate_{candidate}_topdown_rgb.png"
                _topdown_products(
                    stage, center, float(args.window_m), args.render_size, path
                )
                rendered[candidate] = str(path)
            print(json.dumps(rendered, indent=2, sort_keys=True))
            return 0
        physics_scenes = [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.IsA(UsdPhysics.Scene)
        ]
        authored_probe_physics_scene = False
        if not physics_scenes:
            physics_path = "/World/BioNavOMapPhysicsScene"
            UsdPhysics.Scene.Define(stage, physics_path)
            physics_scenes = [physics_path]
            authored_probe_physics_scene = True
            app.update()
        if int(inventory["collision_prim_count"]) <= 0:
            raise RuntimeError("Rivermark has no collision geometry")

        (
            session_collider_layer,
            session_collider_count,
            hidden_annotation_count,
        ) = (
            _author_session_mesh_colliders(stage, app)
        )

        output_dir.mkdir(parents=True, exist_ok=True)
        preflight = {
            "schema_version": 1,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "asset": str(asset),
            "physics_scenes": physics_scenes,
            "physics_scene_authored_for_probe": authored_probe_physics_scene,
            "occupancy_collision_mode": "visible_usd_meshes_in_session_layer",
            "session_collider_count": session_collider_count,
            "hidden_nonphysical_annotation_count": hidden_annotation_count,
            **inventory,
        }
        (output_dir / "stage_preflight.json").write_text(
            json.dumps(preflight, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(30):
            app.update()
        selected = CANDIDATES if args.candidate == "all" else {
            args.candidate: CANDIDATES[args.candidate]
        }
        results = {}
        generator = _omap.Generator(
            omni.physx.get_physx_interface(), context.get_stage_id()
        )
        generator.update_settings(float(args.resolution_m), 1.0, 0.0, 0.5)
        half = 0.5 * float(args.window_m)
        for candidate, center in selected.items():
            seed_trials = []
            selected_raw = None
            selected_seed = None
            for offset_x, offset_y in SEED_OFFSETS_M:
                seed = (float(center[0] + offset_x), float(center[1] + offset_y))
                origin_z = float(center[2] + args.mapping_height_m)
                generator.set_transform(
                    (seed[0], seed[1], origin_z),
                    (
                        float(center[0] - half - seed[0]),
                        float(center[1] - half - seed[1]),
                        float(args.minimum_z_offset_m),
                    ),
                    (
                        float(center[0] + half - seed[0]),
                        float(center[1] + half - seed[1]),
                        float(args.maximum_z_offset_m),
                    ),
                )
                app.update()
                generator.generate2d()
                app.update()
                dimensions = tuple(int(value) for value in generator.get_dimensions())
                raw = np.asarray(generator.get_buffer(), dtype=np.float32)
                if len(dimensions) < 2 or raw.size != dimensions[0] * dimensions[1]:
                    raise RuntimeError(
                        f"candidate {candidate} produced invalid dimensions {dimensions}"
                    )
                raw = raw.reshape(dimensions[1], dimensions[0])
                occupied = int(np.count_nonzero(np.isclose(raw, 1.0)))
                free = int(np.count_nonzero(np.isclose(raw, 0.0)))
                seed_trials.append(
                    {
                        "seed_xy": list(seed),
                        "occupied_cells": occupied,
                        "free_cells": free,
                    }
                )
                if selected_raw is None or occupied > int(
                    np.count_nonzero(np.isclose(selected_raw, 1.0))
                ):
                    selected_raw = raw.copy()
                    selected_seed = seed
            raw = selected_raw
            if raw is None or selected_seed is None:
                raise RuntimeError(f"candidate {candidate} produced no seed trials")
            rgb_path = output_dir / f"candidate_{candidate}_topdown_rgb.png"
            depth_path = output_dir / f"candidate_{candidate}_topdown_depth.npy"
            # Match the occupancy grid exactly so every RGB/depth pixel maps to
            # one occupancy cell without resampling ambiguity.
            _topdown_products(
                stage,
                center,
                float(args.window_m),
                int(raw.shape[0]),
                rgb_path,
                depth_path,
            )
            depth = np.load(depth_path)
            original_omap = raw.copy()
            (
                raw,
                height_diagnostic,
                height,
                reachable,
                steep_boundary,
                height_nontraversable,
            ) = _height_refined_occupancy(
                raw,
                depth,
                center=center,
                seed_xy=selected_seed,
                window_m=float(args.window_m),
                resolution_m=float(args.resolution_m),
                max_step_m=float(args.max_traversable_step_m),
                max_slope=float(args.max_traversable_slope),
                terrain_connect_step_m=float(args.terrain_connect_step_m),
                terrain_connect_slope=float(args.terrain_connect_slope),
                barrier_thickness_cells=int(args.height_barrier_thickness_cells),
            )
            from PIL import Image

            reachable_path = output_dir / f"candidate_{candidate}_reachable.png"
            Image.fromarray((reachable.astype(np.uint8) * 255), mode="L").save(
                reachable_path
            )
            original_omap_path = output_dir / f"candidate_{candidate}_physx_omap.png"
            original_omap_image = np.full(original_omap.shape, 205, dtype=np.uint8)
            original_omap_image[np.isclose(original_omap, 1.0)] = 0
            original_omap_image[np.isclose(original_omap, 0.0)] = 254
            Image.fromarray(original_omap_image[::-1, :], mode="L").save(
                original_omap_path
            )
            height_classes_path = (
                output_dir / f"candidate_{candidate}_height_classes.png"
            )
            height_classes = np.zeros((*height.shape, 3), dtype=np.uint8)
            height_classes[reachable] = (70, 70, 70)
            height_classes[height_nontraversable] = (255, 140, 0)
            height_classes[steep_boundary] = (255, 0, 255)
            Image.fromarray(height_classes, mode="RGB").save(height_classes_path)
            height_path = output_dir / f"candidate_{candidate}_height.png"
            height_preview = np.zeros(height.shape, dtype=np.uint8)
            height_valid = np.isfinite(height)
            height_preview[height_valid] = np.clip(
                (height[height_valid] - (float(center[2]) - 0.5)) / 4.0 * 255.0,
                0.0,
                255.0,
            ).astype(np.uint8)
            Image.fromarray(height_preview, mode="L").save(height_path)

            origin = (float(center[0] - half), float(center[1] - half))
            result = _write_map(
                output_dir,
                candidate,
                raw,
                resolution_m=float(args.resolution_m),
                origin_xy=origin,
            )
            result["candidate_center_xy"] = list(center[:2])
            result["ground_z_m"] = float(center[2])
            result["omap_origin_z_m"] = float(center[2] + args.mapping_height_m)
            result["selected_free_seed_xy"] = list(selected_seed)
            result["seed_trials"] = seed_trials
            result["topdown_rgb"] = str(rgb_path)
            result["topdown_depth"] = str(depth_path)
            result["height_preview"] = str(height_path)
            result["reachable_mask"] = str(reachable_path)
            result["physx_omap_preview"] = str(original_omap_path)
            result["height_classes"] = str(height_classes_path)
            result["height_refinement"] = height_diagnostic
            results[candidate] = result
        timeline.stop()
        (output_dir / "candidate_probe.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "asset": str(asset),
                    "window_m": float(args.window_m),
                    "mapping_height_m": float(args.mapping_height_m),
                    "minimum_z_offset_m": float(args.minimum_z_offset_m),
                    "maximum_z_offset_m": float(args.maximum_z_offset_m),
                    "max_traversable_step_m": float(args.max_traversable_step_m),
                    "max_traversable_slope": float(args.max_traversable_slope),
                    "terrain_connect_step_m": float(args.terrain_connect_step_m),
                    "terrain_connect_slope": float(args.terrain_connect_slope),
                    "height_barrier_thickness_cells": int(
                        args.height_barrier_thickness_cells
                    ),
                    "candidates": results,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(json.dumps(results, indent=2, sort_keys=True))
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
