#!/usr/bin/env python3
"""Render one A21 whole-house pilot with map, graph, Route, Smac and GT."""

from __future__ import annotations

import argparse
import ast
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
    request_ids = list(dict.fromkeys(item["request_id"] for item in progress))
    for request_id, color in zip(request_ids, colors):
        rows = [item for item in progress if item["request_id"] == request_id]
        projected = [item["projected_point"] for item in rows]
        lookahead = [item["lookahead"] for item in rows]
        axis.plot(
            [point[0] for point in projected], [point[1] for point in projected],
            color=color, linewidth=2.4, alpha=0.88, zorder=6,
            label=f"Route projection request {request_id}",
        )
        axis.scatter(
            [point[0] for point in lookahead[::12]],
            [point[1] for point in lookahead[::12]],
            s=10, color="#f59e0b", alpha=0.75, zorder=7,
        )
        route_lines.append(
            f"request {request_id}: route "
            f"{max(item['arc_length_m'] + item['remaining_m'] for item in rows):.2f}m, "
            f"max lateral {max(item['lateral_error_m'] for item in rows):.3f}m"
        )

    actor_path = run / "dynamic_obstacles.csv.gz"
    if actor_path.is_file():
        with gzip.open(actor_path, "rt", encoding="utf-8") as stream:
            actor_rows = list(csv.DictReader(stream))
        required_actors = set(
            manifest.get("dynamic_interaction", {}).get("expected_ids", [])
        )
        actor_colors = {
            "local_bypass_actor": "#f97316",
            "g2_g3_exit_actor": "#06b6d4",
            "g5_g1_crossing_actor": "#eab308",
        }
        for actor_id in sorted(required_actors):
            rows = [item for item in actor_rows if item["id"] == actor_id]
            moving = [
                ast.literal_eval(item["position"])
                for item in rows
                if item["state"] in {"moving", "parked", "retired"}
            ]
            if not moving:
                continue
            color = actor_colors.get(actor_id, "#ec4899")
            axis.plot(
                [point[0] for point in moving], [point[1] for point in moving],
                color=color, linewidth=2.0, alpha=0.9, linestyle="--",
                zorder=9, label=actor_id,
            )
            end_x, end_y = moving[-1][:2]
            axis.add_patch(Rectangle(
                (end_x - 0.2, end_y - 0.2), 0.4, 0.4,
                facecolor=color, edgecolor="#111827", linewidth=0.8,
                alpha=0.45, zorder=9,
            ))
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

    executed = summary.get("path_deviation_percent")
    planned = summary.get("planned_path_deviation_percent")
    deviation = (
        f"executed deviation {executed:.2f}%; planned {planned:.2f}%"
        if isinstance(executed, (int, float)) and isinstance(planned, (int, float))
        else "deviation unavailable (run did not finish)"
    )
    contact = summary.get("static_geometric_contact", {})
    route_lines.append(
        f"{deviation}; physical collision-free={summary.get('physical_collision_free')}; "
        f"SAT max={float(contact.get('maximum_sat_overlap_m', 0.0)):.3f}m"
    )
    interaction = manifest.get("dynamic_interaction", {})
    if interaction:
        route_lines.append(
            "dynamic actors completed "
            f"{len(interaction.get('completed_ids', []))}/"
            f"{len(interaction.get('expected_ids', []))}"
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
    axis.legend(handles, labels, loc="lower left", fontsize=7)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=200)
    plt.close(figure)
    print(args.output)


if __name__ == "__main__":
    main()
