#!/usr/bin/env python3
"""Align a teleop SLAM occupancy map to the calibrated warehouse_new frame.

The indoor mapping supervisor (run_attempt32_indoor_mapping.sh) spawns the
robot at ``long_route_start_g1`` and slam_toolbox builds its map in the
slam frame: robot start pose = origin (0, 0, 0).  The calibrated
warehouse_new frame places that same spawn at (0.45, -5.35, +90 deg), so
the slam map must be rotated +90 degrees CCW and re-origined at the spawn
before it can serve AMCL/Nav2 with the existing spawn calibration.

Pure numpy/PIL/yaml; no ROS or Isaac dependency.  Never overwrites outputs.

Example::

    python3 scripts/align_slam_map_to_warehouse_frame.py \
      --input ~/Workspace/Bio_Nav/runs/operator_maps/kujiale_realistic_v3/kujiale_realistic_v3 \
      --output data/maps/occupancy/warehouse_new_realistic
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import yaml


FREE_PGM = 254


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        required=True,
        type=Path,
        help="input map stem without extension (<stem>.pgm/.yaml)",
    )
    parser.add_argument(
        "--output",
        required=True,
        type=Path,
        help="output map stem without extension (<stem>.pgm/.yaml); "
        "must not already exist",
    )
    parser.add_argument("--spawn-x", type=float, default=0.45)
    parser.add_argument("--spawn-y", type=float, default=-5.35)
    parser.add_argument(
        "--spawn-yaw-deg",
        type=float,
        default=90.0,
        help="calibrated map-frame yaw of the mapping spawn "
        "(long_route_start_g1: +90)",
    )
    parser.add_argument(
        "--resolution-m",
        type=float,
        default=0.05,
        help="must match the input map resolution",
    )
    return parser


def _load_pgm(path: Path) -> tuple[np.ndarray, int, int]:
    with open(path, "rb") as handle:
        if handle.readline().strip() != b"P5":
            raise ValueError(f"{path} is not a binary P5 pgm")
        line = handle.readline()
        while line.startswith(b"#"):
            line = handle.readline()
        width, height = (int(value) for value in line.split())
        max_value = int(handle.readline().strip())
        if max_value != 255:
            raise ValueError(f"{path} max value must be 255, got {max_value}")
        data = np.frombuffer(handle.read(width * height), dtype=np.uint8)
        if data.size != width * height:
            raise ValueError(f"{path} is truncated")
        return data.reshape(height, width).copy(), width, height


def run(args: argparse.Namespace) -> int:
    input_stem = args.input.expanduser().resolve()
    output_stem = args.output.expanduser().resolve()
    input_pgm = input_stem.with_suffix(".pgm")
    input_yaml = input_stem.with_suffix(".yaml")
    output_pgm = output_stem.with_suffix(".pgm")
    output_yaml = output_stem.with_suffix(".yaml")
    output_receipt = output_stem.with_suffix(".receipt.json")
    for path in (output_pgm, output_yaml, output_receipt):
        if path.exists():
            raise FileExistsError(f"refusing to overwrite {path}")
    source = yaml.safe_load(input_yaml.read_text(encoding="utf-8"))
    resolution = float(source.get("resolution", args.resolution_m))
    if not math.isclose(resolution, args.resolution_m, rel_tol=1e-6):
        raise ValueError(
            f"input resolution {resolution} != {args.resolution_m}"
        )
    arr, width, height = _load_pgm(input_pgm)
    slam_origin_x, slam_origin_y = (float(v) for v in source["origin"][:2])

    # Robot start cell = slam frame (0, 0).
    start_col = int((0.0 - slam_origin_x) / resolution)
    start_row = height - 1 - int((0.0 - slam_origin_y) / resolution)
    if not (0 <= start_row < height and 0 <= start_col < width):
        raise ValueError("slam origin does not contain the robot start cell")

    yaw_quadrants = round(args.spawn_yaw_deg / 90.0) % 4
    if not math.isclose(args.spawn_yaw_deg, yaw_quadrants * 90.0, abs_tol=1e-6):
        raise ValueError("spawn yaw must be a multiple of 90 degrees")
    rotated = arr
    rot_row, rot_col = start_row, start_col
    for _ in range(yaw_quadrants):
        prev_rows, prev_cols = rotated.shape
        rotated = np.rot90(rotated, k=1)
        # np.rot90(k=1): in[r, c] -> out[prev_cols - 1 - c, r]
        rot_row, rot_col = prev_cols - 1 - rot_col, rot_row
    out_height, out_width = rotated.shape
    origin_x = args.spawn_x - rot_col * resolution
    origin_y = args.spawn_y - (out_height - 1 - rot_row) * resolution

    output_pgm.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rotated, mode="L").save(output_pgm)
    output_yaml.write_text(
        yaml.safe_dump(
            {
                "image": output_pgm.name,
                "mode": source.get("mode", "trinary"),
                "resolution": resolution,
                "origin": [origin_x, origin_y, 0.0],
                "negate": int(source.get("negate", 0)),
                "occupied_thresh": float(source.get("occupied_thresh", 0.65)),
                "free_thresh": float(source.get("free_thresh", 0.196)),
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    spawn_col = int((args.spawn_x - origin_x) / resolution)
    spawn_row = out_height - 1 - int((args.spawn_y - origin_y) / resolution)
    spawn_value = (
        int(rotated[spawn_row, spawn_col])
        if 0 <= spawn_row < out_height and 0 <= spawn_col < out_width
        else None
    )
    receipt = {
        "schema_version": 1,
        "tool": "align_slam_map_to_warehouse_frame.py",
        "input": str(input_stem),
        "output": str(output_stem),
        "rotation_ccw_deg": yaw_quadrants * 90,
        "origin_map_xy": [origin_x, origin_y],
        "shape_yx": [out_height, out_width],
        "spawn_cell_check": {
            "map_xy": [args.spawn_x, args.spawn_y],
            "row_col": [spawn_row, spawn_col],
            "pgm_value": spawn_value,
            "expected": FREE_PGM,
        },
    }
    output_receipt.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))
    if spawn_value != FREE_PGM:
        print(
            "WARNING: spawn cell is not free in the aligned map; "
            "check the input coverage or the spawn anchor",
        )
        return 1
    return 0


def main() -> int:
    return run(_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
