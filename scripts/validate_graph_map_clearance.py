#!/usr/bin/env python3
"""Validate route-support graph edge clearance against an occupancy map.

Every non-free pixel (occupied or unknown, value < 250) counts as blocking.
An Euclidean distance transform over the free cells gives per-cell clearance,
and every support-edge polyline from a route-support GeoJSON is resampled at
half-cell spacing to compute min/mean/p05 clearance per edge.  Each edge must
keep at least the footprint's padded inscribed radius from the engineering
defaults.  Writes a JSON report and a PNG overlay colored by edge clearance.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy.ndimage import distance_transform_edt, map_coordinates

FREE_PIXEL_MIN = 250
MAGENTA_MAX_M = 0.026
ORANGE_MAX_M = 0.30


def load_occupancy(yaml_path: Path) -> tuple[np.ndarray, float, tuple[float, float]]:
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(str(metadata["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    pixels = np.asarray(Image.open(image_path).convert("L"))
    free = pixels >= FREE_PIXEL_MIN
    origin = metadata["origin"]
    return free, float(metadata["resolution"]), (float(origin[0]), float(origin[1]))


def resample_polyline(points: np.ndarray, spacing_m: float) -> np.ndarray:
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment)))
    total = float(cumulative[-1])
    if total <= 0.0:
        return points[:1]
    distances = np.concatenate((np.arange(0.0, total, spacing_m), [total]))
    index = np.minimum(
        np.searchsorted(cumulative, distances, side="right") - 1, len(segment) - 1
    )
    safe = np.maximum(segment[index], np.finfo(np.float64).eps)
    fraction = (distances - cumulative[index]) / safe
    return points[index] + fraction[:, None] * (points[index + 1] - points[index])


def world_to_pixel(
    xy: np.ndarray, shape: tuple[int, int], resolution: float, origin: tuple[float, float]
) -> tuple[np.ndarray, np.ndarray]:
    column = (xy[:, 0] - origin[0]) / resolution - 0.5
    row = shape[0] - 0.5 - (xy[:, 1] - origin[1]) / resolution
    return row, column


def edge_clearances(
    geometry: dict,
    clearance_field: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    spacing_m: float,
) -> np.ndarray:
    if geometry["type"] == "MultiLineString":
        parts = geometry["coordinates"]
    elif geometry["type"] == "LineString":
        parts = [geometry["coordinates"]]
    else:
        raise ValueError(f"unsupported edge geometry: {geometry['type']}")
    samples = []
    for part in parts:
        points = np.asarray(part, dtype=np.float64)
        if len(points) == 0:
            continue
        xy = resample_polyline(points, spacing_m) if len(points) > 1 else points
        row, column = world_to_pixel(xy, clearance_field.shape, resolution, origin)
        samples.append(
            map_coordinates(
                clearance_field, [row, column], order=1, mode="constant", cval=0.0
            )
        )
    if not samples:
        return np.zeros(1, dtype=np.float64)
    return np.concatenate(samples)


def edge_color(min_clearance_m: float) -> tuple[int, int, int]:
    if min_clearance_m < MAGENTA_MAX_M:
        return (255, 0, 255)
    if min_clearance_m < ORANGE_MAX_M:
        return (255, 165, 0)
    return (0, 0, 255)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True, dest="map_path")
    parser.add_argument("--geojson", type=Path, required=True)
    parser.add_argument("--defaults", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--png", type=Path, required=True)
    args = parser.parse_args()

    defaults = yaml.safe_load(args.defaults.read_text(encoding="utf-8"))
    required_m = float(defaults["footprint"]["padded_inscribed_radius_m"])
    free, resolution, origin = load_occupancy(args.map_path)
    clearance_field = distance_transform_edt(free) * resolution
    spacing_m = resolution / 2.0

    graph = json.loads(args.geojson.read_text(encoding="utf-8"))
    edges = []
    nodes = []
    for feature in graph["features"]:
        if feature["geometry"]["type"] == "Point":
            nodes.append(feature)
        else:
            edges.append(feature)

    edge_rows = []
    all_samples = []
    for feature in edges:
        values = edge_clearances(
            feature["geometry"], clearance_field, resolution, origin, spacing_m
        )
        all_samples.append(values)
        metadata = feature.get("properties", {}).get("metadata", {})
        edge_rows.append(
            {
                "id": feature["properties"]["id"],
                "canonical_edge_id": metadata.get("canonical_edge_id"),
                "sample_count": int(len(values)),
                "min_m": float(values.min()),
                "mean_m": float(values.mean()),
                "p05_m": float(np.percentile(values, 5)),
            }
        )
    pooled = np.concatenate(all_samples) if all_samples else np.zeros(1)
    failing = [row for row in edge_rows if row["min_m"] < required_m]

    report = {
        "map_path": str(args.map_path),
        "map_version": args.map_path.stem,
        "geojson_path": str(args.geojson),
        "geojson_name": graph.get("name"),
        "defaults_path": str(args.defaults),
        "blocking_rule": "pixel < 250 (occupied + unknown)",
        "resolution_m": resolution,
        "origin_xy_m": [origin[0], origin[1]],
        "sample_spacing_m": spacing_m,
        "required_min_clearance_m": required_m,
        "support_edge_count": len(edge_rows),
        "support_node_count": len(nodes),
        "clearance_m": {
            "global_min": float(pooled.min()),
            "global_mean": float(pooled.mean()),
            "global_p05": float(np.percentile(pooled, 5)),
        },
        "edge_min_clearance_m": {
            "min": float(min(row["min_m"] for row in edge_rows)) if edge_rows else None,
            "mean": float(np.mean([row["min_m"] for row in edge_rows]))
            if edge_rows
            else None,
        },
        "edges_below_required_count": len(failing),
        "edges_below_required": failing,
        "all_edges_pass": not failing,
        "edges": edge_rows,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    scale = 4
    height, width = free.shape
    background = np.full((height, width, 3), 255, dtype=np.uint8)
    background[~free] = (0, 0, 0)
    image = Image.fromarray(background).resize(
        (width * scale, height * scale), Image.Resampling.NEAREST
    )
    margin = 56
    canvas = Image.new("RGB", (width * scale, height * scale + margin), "white")
    canvas.paste(image, (0, margin))
    draw = ImageDraw.Draw(canvas)

    def to_canvas(xy: np.ndarray) -> list[tuple[float, float]]:
        row, column = world_to_pixel(xy, (height, width), resolution, origin)
        return [(float(c) * scale, float(r) * scale + margin) for r, c in zip(row, column)]

    for feature, row_stats in zip(edges, edge_rows):
        geometry = feature["geometry"]
        parts = (
            geometry["coordinates"]
            if geometry["type"] == "MultiLineString"
            else [geometry["coordinates"]]
        )
        color = edge_color(row_stats["min_m"])
        for part in parts:
            points = np.asarray(part, dtype=np.float64)
            if len(points) >= 2:
                draw.line(to_canvas(points), fill=color, width=2)
    for feature in nodes:
        (cx, cy) = to_canvas(np.asarray([feature["geometry"]["coordinates"]]))[0]
        radius = 3
        draw.ellipse(
            (cx - radius, cy - radius, cx + radius, cy + radius),
            outline=(0, 0, 255),
            width=1,
        )
    draw.text(
        (8, 6),
        f"{args.map_path.stem} + {graph.get('name')} "
        f"({len(edge_rows)} support edges, spacing {spacing_m:.3f} m)",
        fill="black",
    )
    draw.text(
        (8, 22),
        f"edge min clearance: magenta < {MAGENTA_MAX_M} m, orange < {ORANGE_MAX_M} m, "
        f"blue >= {ORANGE_MAX_M} m; required >= {required_m} m",
        fill="black",
    )
    draw.text(
        (8, 38),
        f"global min {report['clearance_m']['global_min']:.3f} m, "
        f"mean {report['clearance_m']['global_mean']:.3f} m, "
        f"p05 {report['clearance_m']['global_p05']:.3f} m; "
        f"below required: {len(failing)}",
        fill="black",
    )
    args.png.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(args.png)

    print(
        json.dumps(
            {
                "map_version": report["map_version"],
                "support_edge_count": len(edge_rows),
                "clearance_m": report["clearance_m"],
                "edges_below_required_count": len(failing),
                "all_edges_pass": report["all_edges_pass"],
            },
            sort_keys=True,
        )
    )
    return 0 if not failing else 1


if __name__ == "__main__":
    raise SystemExit(main())
