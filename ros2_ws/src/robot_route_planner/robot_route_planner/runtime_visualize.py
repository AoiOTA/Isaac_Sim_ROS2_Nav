"""Visualize transient and persistent runtime edge observations and reroutes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .defaults import load_engineering_defaults
from .feasibility import apply_footprint_feasibility
from .gvg import build_gvg
from .map_io import load_occupancy_map
from .route_ab_visualize import _parse_edge_ids, _route_xy
from .visualize import _map_background, _plot_graph


def export_runtime_overlay(
    map_path: Path,
    defaults_path: Path,
    baseline_edges: list[int],
    current_edges: list[int],
    observed_edge: int,
    state: str,
    output_path: Path,
    path_receipt: Path | None = None,
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
    edge = graph.edge_by_id()[observed_edge]
    baseline_xy = _route_xy(graph, baseline_edges)
    current_xy = _route_xy(graph, current_edges)
    center = edge.polyline_xy[len(edge.polyline_xy) // 2]
    color = "#ffab00" if state.upper() == "SUSPECT" else "#d50000"

    figure, axis = plt.subplots(figsize=(10, 14), constrained_layout=True)
    _map_background(axis, occupancy)
    _plot_graph(axis, graph, alpha=0.16, linewidth=0.45)
    axis.plot(
        baseline_xy[:, 0], baseline_xy[:, 1], color="#90a4ae",
        linewidth=1.5, linestyle=":", label="pre-observation route", zorder=6,
    )
    axis.plot(
        current_xy[:, 0], current_xy[:, 1], color="#2962ff",
        linewidth=2.3, label="current Route Server route", zorder=7,
    )
    axis.plot(
        edge.polyline_xy[:, 0], edge.polyline_xy[:, 1], color=color,
        linewidth=4.0, label=f"edge {observed_edge}: {state.upper()}", zorder=8,
    )
    axis.scatter(
        center[0], center[1], marker="X", s=100, c=color, edgecolors="black",
        label="simulated occupancy observation", zorder=9,
    )
    if path_receipt is not None:
        receipt = json.loads(path_receipt.read_text(encoding="utf-8"))
        first_plan = next(iter(receipt["plans"].values()))
        points = np.asarray(first_plan["poses_xy_zw"], dtype=float)
        axis.plot(
            points[:, 0], points[:, 1], color="#00c853", linewidth=1.8,
            label="current Smac path", zorder=9,
        )
    axis.set_title(
        f"Runtime edge {state.upper()}: structural graph revision remains 1"
    )
    axis.legend(loc="upper right", fontsize=8)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)
    return {
        "classification": "engineering_visual_evidence",
        "image": output_path.name,
        "runtime_state": state.upper(),
        "observed_edge": observed_edge,
        "graph_revision": graph.revision,
        "baseline_edge_ids": baseline_edges,
        "current_edge_ids": current_edges,
        "route_changed": baseline_edges != current_edges,
        "blocked_edge_used_by_current_route": observed_edge in current_edges,
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, dest="map_path")
    parser.add_argument("--defaults", required=True, dest="defaults_path")
    parser.add_argument("--baseline-edges", required=True)
    parser.add_argument("--current-edges", required=True)
    parser.add_argument("--observed-edge", required=True, type=int)
    parser.add_argument("--state", required=True, choices=["suspect", "blocked"])
    parser.add_argument("--path-receipt")
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    result = export_runtime_overlay(
        Path(args.map_path),
        Path(args.defaults_path),
        _parse_edge_ids(args.baseline_edges),
        _parse_edge_ids(args.current_edges),
        args.observed_edge,
        args.state,
        Path(args.output),
        Path(args.path_receipt) if args.path_receipt else None,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
