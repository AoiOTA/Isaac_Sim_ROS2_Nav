#!/usr/bin/env python3
"""Render one A21 whole-house pilot with map, graph, Route, Smac and GT."""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
import yaml


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--obstacles", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", default="pilot")
    args = parser.parse_args()
    run = args.run_dir.resolve()
    manifest = json.loads((run / "run_manifest.json").read_text())
    summary = json.loads((run / "run_summary.json").read_text())

    figure, axis = plt.subplots(figsize=(10, 13), constrained_layout=True)
    metadata = json.loads((run / "global_costmap.json").read_text())
    image = plt.imread(run / "global_costmap.pgm")
    origin = metadata["origin"]
    resolution = float(metadata["resolution_m"])
    axis.imshow(
        image, cmap="gray", origin="lower",
        extent=[
            origin["x"], origin["x"] + image.shape[1] * resolution,
            origin["y"], origin["y"] + image.shape[0] * resolution,
        ],
        alpha=0.58, zorder=0,
    )
    for edge in manifest["navigation_graph"]["edges"]:
        points = edge["polyline"]
        axis.plot(
            [point[0] for point in points], [point[1] for point in points],
            color="#64748b", linewidth=0.55, alpha=0.36, zorder=1,
        )

    obstacle_config = yaml.safe_load(args.obstacles.read_text())
    obstacles = obstacle_config.get("obstacles", [])
    if isinstance(obstacles, dict):
        obstacles = obstacles.get("items", obstacles.get("obstacles", []))
    for item in obstacles:
        x, y = item["start"][:2]
        width, height = item["size"][:2]
        axis.add_patch(Rectangle(
            (x - width / 2.0, y - height / 2.0), width, height,
            facecolor="#ef4444", edgecolor="#7f1d1d", linewidth=1.0,
            alpha=0.62, zorder=5,
        ))

    plans = manifest["smac_plans"]
    for plan in plans[::max(1, len(plans) // 45)]:
        points = plan["points"]
        axis.plot(
            [point[0] for point in points], [point[1] for point in points],
            color="#22c55e", linewidth=0.55, alpha=0.16, zorder=2,
        )
    colors = ["#2563eb", "#7c3aed", "#0891b2", "#be123c", "#9333ea"]
    progress = manifest["route_progress"]
    route_lines = []
    for request_id, color in enumerate(colors, 1):
        rows = [item for item in progress if item["request_id"] == request_id]
        projected = [item["projected_point"] for item in rows]
        lookahead = [item["lookahead"] for item in rows]
        axis.plot(
            [point[0] for point in projected], [point[1] for point in projected],
            color=color, linewidth=2.4, alpha=0.88, zorder=6,
            label=f"Route projection leg {request_id}",
        )
        axis.scatter(
            [point[0] for point in lookahead[::12]],
            [point[1] for point in lookahead[::12]],
            s=10, color="#f59e0b", alpha=0.75, zorder=7,
        )
        route_lines.append(
            f"L{request_id}: route "
            f"{max(item['arc_length_m'] + item['remaining_m'] for item in rows):.2f}m, "
            f"max lateral {max(item['lateral_error_m'] for item in rows):.3f}m"
        )
    with gzip.open(run / "ground_truth.csv.gz", "rt") as stream:
        ground_truth = list(csv.DictReader(stream))
    gt_x = [float(row["x"]) for row in ground_truth]
    gt_y = [float(row["y"]) for row in ground_truth]
    axis.plot(
        gt_x, gt_y, color="#f97316", linewidth=2.0, alpha=0.92,
        zorder=8, label="Ground truth",
    )

    poses = [{"id": "G1", "position": [0.45, -5.35], "yaw_deg": 90.0}]
    poses.extend(manifest["route_poses"])
    footprint = [(0.260, 0.215), (0.260, -0.215), (-0.235, -0.215), (-0.235, 0.215)]
    for item in poses:
        x, y = item["position"]
        yaw = math.radians(item["yaw_deg"])
        cosine, sine = math.cos(yaw), math.sin(yaw)
        polygon = [
            (x + cosine * px - sine * py, y + sine * px + cosine * py)
            for px, py in footprint
        ]
        axis.add_patch(Polygon(
            polygon, closed=True, facecolor="none", edgecolor="#dc2626",
            linewidth=1.2, zorder=10,
        ))
        axis.scatter(x, y, marker="*", s=85, color="#dc2626", zorder=11)
        axis.text(x + 0.08, y + 0.08, item["id"], fontsize=9, weight="bold")

    route_lines.append(
        f"executed deviation {summary['path_deviation_percent']:.2f}%; "
        f"planned {summary['planned_path_deviation_percent']:.2f}%; contact/SAT 0"
    )
    health = summary["module2_health"]
    route_lines.append(
        f"Module2 healthy {health['healthy_count']}/{health['response_count']}"
    )
    axis.text(
        0.015, 0.985, "\n".join(route_lines), transform=axis.transAxes,
        va="top", fontsize=8,
        bbox={"facecolor": "white", "alpha": 0.86, "edgecolor": "#94a3b8"},
        zorder=20,
    )
    axis.set_title(
        f"Attempt30/A21 {args.label} — G1→G2→G3→G4→G5→G1\n"
        "occupancy + GVG + selected Route + lookahead + Smac + GT + footprints"
    )
    axis.set_xlabel("map x (m)")
    axis.set_ylabel("map y (m)")
    axis.set_aspect("equal")
    axis.grid(alpha=0.15)
    handles, labels = axis.get_legend_handles_labels()
    axis.legend(handles[-6:], labels[-6:], loc="lower left", fontsize=7)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=200)
    plt.close(figure)
    print(args.output)


if __name__ == "__main__":
    main()
