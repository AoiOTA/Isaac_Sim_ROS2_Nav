#!/usr/bin/env python3
"""Generate the frozen 404x404 occupancy map for the V6 flat arena."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


RESOLUTION_M = 0.05
ORIGIN_X_M = -10.1
ORIGIN_Y_M = -10.1
WIDTH_CELLS = 404
HEIGHT_CELLS = 404
FREE = 254
OCCUPIED = 0


def _default_root() -> Path:
    return Path(__file__).resolve().parents[2]


def load_rectangles(config_path: Path) -> tuple[tuple[float, float, float, float], ...]:
    document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if document.get("schema_version") != 2 or document.get("enabled") is not True:
        raise ValueError("flat calibration features must be enabled schema v2")
    rectangles = []
    for item in document.get("obstacles", ()):
        if item.get("mode") != "stationary" or item.get("start") != item.get("end"):
            raise ValueError(f"{item.get('id', 'unknown')} must be stationary")
        center_x, center_y, _ = (float(value) for value in item["start"])
        size_x, size_y, _ = (float(value) for value in item["size"])
        rectangles.append((center_x, center_y, size_x, size_y))
    if len(rectangles) != 7:
        raise ValueError("flat calibration geometry requires four walls and three features")
    return tuple(rectangles)


def generate_map(rectangles: tuple[tuple[float, float, float, float], ...]) -> bytes:
    pixels = bytearray([FREE]) * (WIDTH_CELLS * HEIGHT_CELLS)
    epsilon = 1.0e-9
    for row in range(HEIGHT_CELLS):
        y = ORIGIN_Y_M + (HEIGHT_CELLS - row - 0.5) * RESOLUTION_M
        for column in range(WIDTH_CELLS):
            x = ORIGIN_X_M + (column + 0.5) * RESOLUTION_M
            if any(
                abs(x - center_x) <= size_x / 2.0 + epsilon
                and abs(y - center_y) <= size_y / 2.0 + epsilon
                for center_x, center_y, size_x, size_y in rectangles
            ):
                pixels[row * WIDTH_CELLS + column] = OCCUPIED
    header = f"P5\n{WIDTH_CELLS} {HEIGHT_CELLS}\n255\n".encode("ascii")
    return header + bytes(pixels)


def main(argv=None) -> int:
    root = _default_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--features",
        type=Path,
        default=root / "isaac_sim/configs/experiments/v6_calibration_grid_features.yaml",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "data/maps/v6_calibration_flat_20m.pgm",
    )
    arguments = parser.parse_args(argv)
    payload = generate_map(load_rectangles(arguments.features.resolve()))
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_bytes(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
