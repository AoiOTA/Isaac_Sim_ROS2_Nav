"""Overlay live Smac results with the occupancy map and canonical route."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .defaults import load_engineering_defaults
from .feasibility import _polygon_is_free, apply_footprint_feasibility
from .gvg import build_gvg
from .map_io import load_occupancy_map
from .route_ab_visualize import _parse_edge_ids, _route_xy
from .visualize import _draw_footprint, _map_background, _plot_graph


def _point_to_polyline_distance(point: np.ndarray, polyline: np.ndarray) -> float:
    starts = polyline[:-1]
    vectors = polyline[1:] - starts
    squared = np.einsum("ij,ij->i", vectors, vectors)
    fractions = np.divide(
        np.einsum("ij,ij->i", point - starts, vectors),
        squared,
        out=np.zeros_like(squared),
        where=squared > np.finfo(float).eps,
    )
    projected = starts + np.clip(fractions, 0.0, 1.0)[:, None] * vectors
    return float(np.min(np.linalg.norm(projected - point, axis=1)))


def export_smac_overlay(
    map_path: Path,
    defaults_path: Path,
    receipt_path: Path,
    route_edges: list[int],
    output_path: Path,
) -> dict:
    defaults = load_engineering_defaults(defaults_path)
    occupancy = load_occupancy_map(
        map_path,
        unknown_is_occupied=bool(defaults["graph"]["unknown_is_occupied"]),
    )
    graph = apply_footprint_feasibility(
        build_gvg(
            occupancy,
            defaults["graph"],
            defaults["footprint"],
            defaults["route_cost"],
        ),
        occupancy,
        defaults["footprint"],
    )
    route_xy = _route_xy(graph, route_edges)
    receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
    footprint = np.asarray(defaults["footprint"]["polygon_m"], dtype=float)
    radial = np.linalg.norm(footprint, axis=1)
    nonzero = radial > np.finfo(float).eps
    footprint[nonzero] += (
        float(defaults["footprint"]["padding_m"])
        * footprint[nonzero]
        / radial[nonzero, None]
    )
    colors = {
        "GridBased": "#d500f9",
        "GridBaseline": "#00c853",
        "BT+SmacLattice+MPPI": "#ff6d00",
    }
    metrics = {}

    figure, axes = plt.subplots(1, 2, figsize=(17, 9), constrained_layout=True)
    for axis in axes:
        _map_background(axis, occupancy)
        _plot_graph(axis, graph, alpha=0.13, linewidth=0.4)
        axis.plot(
            route_xy[:, 0],
            route_xy[:, 1],
            color="#2962ff",
            linewidth=2.1,
            label="canonical Route",
            zorder=7,
        )
        for planner, plan in receipt["plans"].items():
            points = np.asarray(plan.get("poses_xy_zw", []), dtype=float)
            if len(points):
                axis.plot(
                    points[:, 0],
                    points[:, 1],
                    color=colors.get(planner, "#ff6d00"),
                    linewidth=1.8,
                    linestyle="--" if planner == "GridBaseline" else "-",
                    label=planner,
                    zorder=8,
                )
        start = receipt["start_xy_yaw"]
        goal = receipt["goal_xy_yaw"]
        axis.scatter(start[0], start[1], c="black", s=55, label="start", zorder=10)
        axis.scatter(goal[0], goal[1], c="#ff1744", marker="*", s=120, label="lookahead", zorder=10)

    for planner, plan in receipt["plans"].items():
        points = np.asarray(plan.get("poses_xy_zw", []), dtype=float)
        collision_count = 0
        lateral_errors = []
        for x, y, z, w in points:
            yaw = 2.0 * math.atan2(z, w)
            if not _polygon_is_free(occupancy, x, y, yaw, footprint):
                collision_count += 1
            lateral_errors.append(
                _point_to_polyline_distance(np.asarray([x, y]), route_xy)
            )
        metrics[planner] = {
            "planner_error_code": int(plan.get("error_code", -1)),
            "pose_count": len(points),
            "footprint_collision_sample_count": collision_count,
            "maximum_route_lateral_error_m": max(lateral_errors, default=math.inf),
            "mean_route_lateral_error_m": float(np.mean(lateral_errors)) if lateral_errors else math.inf,
        }

    start = receipt["start_xy_yaw"]
    goal = receipt["goal_xy_yaw"]
    margin = 0.8
    axes[0].set_title("Route and live Smac paths in map/GVG context")
    axes[1].set_xlim(min(start[0], goal[0]) - margin, max(start[0], goal[0]) + margin)
    axes[1].set_ylim(min(start[1], goal[1]) - margin, max(start[1], goal[1]) + margin)
    axes[1].set_title("Local detail: metric path to current route lookahead")
    axes[1].legend(loc="upper right", fontsize=8)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    return {
        "classification": "engineering_visual_evidence",
        "image": output_path.name,
        "route_edge_ids": route_edges,
        "planners": metrics,
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, dest="map_path")
    parser.add_argument("--defaults", required=True, dest="defaults_path")
    parser.add_argument("--receipt", required=True)
    parser.add_argument("--route-edges", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    result = export_smac_overlay(
        Path(args.map_path),
        Path(args.defaults_path),
        Path(args.receipt),
        _parse_edge_ids(args.route_edges),
        Path(args.output),
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
