#!/usr/bin/env python3
"""Create an aligned RGB / occupancy / overlay comparison for Rivermark."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy.ndimage import binary_erosion


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb", type=Path, required=True)
    parser.add_argument("--occupancy", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--aligned-rgb-output", type=Path)
    parser.add_argument("--edge-overlay-output", type=Path)
    args = parser.parse_args()

    rgb = Image.open(args.rgb).convert("RGB")
    occupancy = Image.open(args.occupancy).convert("L").resize(
        rgb.size, Image.Resampling.NEAREST
    )
    occupancy_array = np.asarray(occupancy)
    overlay_array = np.asarray(rgb).copy()
    occupied = occupancy_array < 50
    unknown = (occupancy_array >= 50) & (occupancy_array < 240)
    overlay_array[occupied] = (
        0.35 * overlay_array[occupied] + 0.65 * np.array([255, 20, 20])
    ).astype(np.uint8)
    overlay_array[unknown] = (
        0.45 * overlay_array[unknown] + 0.55 * np.array([255, 190, 0])
    ).astype(np.uint8)
    overlay = Image.fromarray(overlay_array, mode="RGB")
    occupied_edge = occupied & ~binary_erosion(occupied)
    unknown_edge = unknown & ~binary_erosion(unknown)
    edge_overlay_array = np.asarray(rgb).copy()
    edge_overlay_array[occupied_edge] = np.array([255, 0, 0], dtype=np.uint8)
    edge_overlay_array[unknown_edge] = np.array([255, 190, 0], dtype=np.uint8)
    edge_overlay = Image.fromarray(edge_overlay_array, mode="RGB")

    margin = 44
    canvas = Image.new("RGB", (rgb.width * 3, rgb.height + margin), "white")
    canvas.paste(rgb, (0, margin))
    canvas.paste(occupancy.convert("RGB"), (rgb.width, margin))
    canvas.paste(overlay, (rgb.width * 2, margin))
    draw = ImageDraw.Draw(canvas)
    draw.text((12, 14), "A. aligned top-down RGB (+Y up, +X right)", fill="black")
    draw.text(
        (rgb.width + 12, 14),
        "B. conservative 2.5D + collision occupancy",
        fill="black",
    )
    draw.text(
        (rgb.width * 2 + 12, 14),
        "C. overlay: red=occupied, amber=unknown",
        fill="black",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.output)
    if args.aligned_rgb_output is not None:
        args.aligned_rgb_output.parent.mkdir(parents=True, exist_ok=True)
        rgb.save(args.aligned_rgb_output)
    if args.edge_overlay_output is not None:
        args.edge_overlay_output.parent.mkdir(parents=True, exist_ok=True)
        edge_overlay.save(args.edge_overlay_output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
