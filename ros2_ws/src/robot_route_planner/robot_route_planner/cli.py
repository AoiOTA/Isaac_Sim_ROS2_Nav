"""Build and export an A21 structural graph from a ROS map YAML."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

from .defaults import load_engineering_defaults
from .diagnostics import graph_diagnostics
from .feasibility import apply_footprint_feasibility
from .gvg import build_gvg
from .map_io import load_occupancy_map
from .route_support import export_route_support_graph, save_route_support


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, dest="map_path")
    parser.add_argument("--defaults", required=True, dest="defaults_path")
    parser.add_argument("--geojson", required=True)
    parser.add_argument("--mapping", required=True)
    parser.add_argument("--summary", required=True)
    return parser


def main(argv=None) -> None:
    args = build_parser().parse_args(argv)
    defaults = load_engineering_defaults(args.defaults_path)
    occupancy = load_occupancy_map(
        args.map_path,
        unknown_is_occupied=bool(defaults["graph"]["unknown_is_occupied"]),
    )
    graph = build_gvg(
        occupancy,
        defaults["graph"],
        defaults["footprint"],
        defaults["route_cost"],
    )
    apply_footprint_feasibility(graph, occupancy, defaults["footprint"])
    export = export_route_support_graph(
        graph,
        support_spacing_m=float(defaults["graph"]["route_support_spacing_m"]),
    )
    save_route_support(export, args.geojson, args.mapping)
    summary = {
        "classification": "engineering_output_only",
        "graph_id": graph.graph_id,
        "revision": graph.revision,
        "map_version": graph.map_version,
        "node_count": len(graph.nodes),
        "directed_edge_count": len(graph.edges),
        "support_node_count": len(export.canonical_to_support_nodes)
        + len(
            {
                feature["properties"]["id"]
                for feature in export.geojson["features"]
                if feature["geometry"]["type"] == "Point"
            }
        )
        - len(export.canonical_to_support_nodes),
        "support_edge_count": len(export.support_to_canonical_edge),
        "traversability": dict(
            Counter(edge.static_traversability.name for edge in graph.edges)
        ),
        "total_undirected_length_m": sum(edge.length_m for edge in graph.edges) / 2.0,
        "choice_space": graph_diagnostics(graph),
    }
    Path(args.summary).write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
