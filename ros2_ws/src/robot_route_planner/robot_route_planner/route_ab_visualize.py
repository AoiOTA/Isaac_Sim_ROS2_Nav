"""Visualize geometry-only and Module2-scored routes on the structural graph."""

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
from .visualize import _map_background, _plot_graph


def _parse_edge_ids(value: str) -> list[int]:
    return [int(item) for item in value.split(",") if item.strip()]


def _route_xy(graph, edge_ids: list[int]) -> np.ndarray:
    edge_map = graph.edge_by_id()
    segments = []
    for index, edge_id in enumerate(edge_ids):
        polyline = edge_map[edge_id].polyline_xy
        if index and np.linalg.norm(segments[-1][-1] - polyline[0]) > np.linalg.norm(
            segments[-1][-1] - polyline[-1]
        ):
            polyline = polyline[::-1]
        segments.append(polyline if index == 0 else polyline[1:])
    return np.vstack(segments)


def export_route_ab(
    map_path: Path,
    defaults_path: Path,
    geometry_edges: list[int],
    learned_edges: list[int],
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
    geometry_xy = _route_xy(graph, geometry_edges)
    learned_xy = _route_xy(graph, learned_edges)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(10, 14), constrained_layout=True)
    _map_background(axis, occupancy)
    _plot_graph(axis, graph, alpha=0.20, linewidth=0.5)
    axis.plot(
        geometry_xy[:, 0],
        geometry_xy[:, 1],
        color="#2962ff",
        linewidth=2.5,
        label="geometry-only Route Server result",
        zorder=8,
    )
    axis.plot(
        learned_xy[:, 0],
        learned_xy[:, 1],
        color="#ff6d00",
        linewidth=2.2,
        linestyle="--",
        label="Module2 learned-prior result",
        zorder=9,
    )
    axis.scatter(
        geometry_xy[0, 0],
        geometry_xy[0, 1],
        marker="o",
        s=70,
        c="#00c853",
        edgecolors="black",
        label="start",
        zorder=10,
    )
    axis.scatter(
        geometry_xy[-1, 0],
        geometry_xy[-1, 1],
        marker="*",
        s=130,
        c="#d50000",
        edgecolors="black",
        label="goal",
        zorder=10,
    )
    axis.set_title(
        "A21 Route A/B: Module3 retains the loop; Module2 scores existing edges"
    )
    axis.legend(loc="upper right", fontsize=8)
    figure.savefig(output_path, dpi=180)
    plt.close(figure)

    return {
        "classification": "engineering_visual_evidence",
        "image": output_path.name,
        "geometry_edge_ids": geometry_edges,
        "learned_edge_ids": learned_edges,
        "same_route": geometry_edges == learned_edges,
        "shared_edge_count": len(set(geometry_edges) & set(learned_edges)),
        "geometry_length_m": float(
            sum(graph.edge_by_id()[edge_id].length_m for edge_id in geometry_edges)
        ),
        "learned_length_m": float(
            sum(graph.edge_by_id()[edge_id].length_m for edge_id in learned_edges)
        ),
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, dest="map_path")
    parser.add_argument("--defaults", required=True, dest="defaults_path")
    parser.add_argument("--geometry-edges", required=True)
    parser.add_argument("--learned-edges", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    result = export_route_ab(
        Path(args.map_path),
        Path(args.defaults_path),
        _parse_edge_ids(args.geometry_edges),
        _parse_edge_ids(args.learned_edges),
        Path(args.output),
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
