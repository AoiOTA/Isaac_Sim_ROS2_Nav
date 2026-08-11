"""Export curved canonical edges as straight Nav2 Route Server segments."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from .models import Graph, Traversability


@dataclass(frozen=True)
class RouteSupportExport:
    geojson: dict
    canonical_to_support_edges: dict[int, list[int]]
    support_to_canonical_edge: dict[int, int]
    canonical_to_support_nodes: dict[int, int]


def _resample(points: np.ndarray, spacing_m: float) -> tuple[np.ndarray, np.ndarray]:
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment)))
    total = float(cumulative[-1])
    if total <= 0.0:
        return points[:1].copy(), np.asarray([0.0])
    distances = np.concatenate((np.arange(0.0, total, spacing_m), [total]))
    output = []
    for distance in distances:
        index = min(int(np.searchsorted(cumulative, distance, side="right") - 1), len(segment) - 1)
        fraction = 0.0 if segment[index] <= 0.0 else (
            distance - cumulative[index]
        ) / segment[index]
        output.append(points[index] + fraction * (points[index + 1] - points[index]))
    return np.asarray(output), distances


def export_route_support_graph(
    graph: Graph, *, support_spacing_m: float, frame_id: str = "map"
) -> RouteSupportExport:
    if support_spacing_m <= 0.0:
        raise ValueError("support spacing must be positive")
    node_positions: list[tuple[float, float]] = []
    coordinate_to_id: dict[tuple[float, float], int] = {}
    canonical_nodes: dict[int, int] = {}

    def support_node(point: np.ndarray) -> int:
        key = tuple(float(value) for value in np.round(point, 6))
        if key not in coordinate_to_id:
            coordinate_to_id[key] = len(node_positions)
            node_positions.append(key)
        return coordinate_to_id[key]

    for node in sorted(graph.nodes, key=lambda item: item.id):
        canonical_nodes[node.id] = support_node(np.asarray(node.position_xy))

    segments = []
    physical_samples: dict[
        tuple[int, int, tuple[float, ...]], tuple[np.ndarray, np.ndarray]
    ] = {}
    for edge in sorted(graph.edges, key=lambda item: item.id):
        if edge.static_traversability == Traversability.INFEASIBLE:
            continue
        forward = edge.from_node < edge.to_node or (
            edge.from_node == edge.to_node
            and tuple(edge.polyline_xy[0]) <= tuple(edge.polyline_xy[-1])
        )
        normalized = edge.polyline_xy if forward else edge.polyline_xy[::-1]
        physical_key = (
            min(edge.from_node, edge.to_node),
            max(edge.from_node, edge.to_node),
            tuple(float(value) for value in np.round(normalized, 6).ravel()),
        )
        if physical_key not in physical_samples:
            physical_samples[physical_key] = _resample(
                normalized, support_spacing_m
            )
        physical_points, physical_distances = physical_samples[physical_key]
        if forward:
            points = physical_points
            distances = physical_distances
        else:
            points = physical_points[::-1].copy()
            distances = physical_distances[-1] - physical_distances[::-1]
        ids = [support_node(point) for point in points]
        for index, (start, end) in enumerate(zip(ids[:-1], ids[1:])):
            if start == end:
                continue
            segments.append(
                {
                    "canonical_edge_id": edge.id,
                    "segment_index": index,
                    "start": start,
                    "end": end,
                    "start_xy": points[index].tolist(),
                    "end_xy": points[index + 1].tolist(),
                    "fraction_start": float(distances[index] / max(edge.length_m, np.finfo(float).eps)),
                    "fraction_end": float(distances[index + 1] / max(edge.length_m, np.finfo(float).eps)),
                    "length_m": float(distances[index + 1] - distances[index]),
                    "min_clearance_m": edge.min_clearance_m,
                    "bottleneck": edge.bottleneck,
                    "static_traversability": edge.static_traversability.name,
                }
            )
    feature_count = len(node_positions) + len(segments)
    if feature_count > 65535:
        raise ValueError(
            f"Route Server uint16 ID space exceeded by {feature_count} features"
        )
    features = []
    for node_id, point in enumerate(node_positions):
        features.append(
            {
                "type": "Feature",
                "properties": {"id": node_id, "frame": frame_id},
                "geometry": {"type": "Point", "coordinates": list(point)},
            }
        )
    canonical_to_support: dict[int, list[int]] = {}
    support_to_canonical: dict[int, int] = {}
    for offset, segment in enumerate(segments):
        support_edge_id = len(node_positions) + offset
        canonical_id = int(segment["canonical_edge_id"])
        canonical_to_support.setdefault(canonical_id, []).append(support_edge_id)
        support_to_canonical[support_edge_id] = canonical_id
        metadata = {
            "canonical_edge_id": canonical_id,
            "segment_index": int(segment["segment_index"]),
            "segment_fraction_start": float(segment["fraction_start"]),
            "segment_fraction_end": float(segment["fraction_end"]),
            "length_m": float(segment["length_m"]),
            "min_clearance_m": float(segment["min_clearance_m"]),
            "bottleneck": bool(segment["bottleneck"]),
            "static_traversability": str(segment["static_traversability"]),
        }
        features.append(
            {
                "type": "Feature",
                "properties": {
                    "id": support_edge_id,
                    "startid": int(segment["start"]),
                    "endid": int(segment["end"]),
                    "metadata": metadata,
                },
                "geometry": {
                    "type": "MultiLineString",
                    "coordinates": [[segment["start_xy"], segment["end_xy"]]],
                },
            }
        )
    geojson = {
        "type": "FeatureCollection",
        "name": graph.graph_id,
        "crs": {
            "type": "name",
            "properties": {"name": "urn:ogc:def:crs:EPSG::3857"},
        },
        "features": features,
    }
    return RouteSupportExport(
        geojson=geojson,
        canonical_to_support_edges=canonical_to_support,
        support_to_canonical_edge=support_to_canonical,
        canonical_to_support_nodes=canonical_nodes,
    )


def save_route_support(export: RouteSupportExport, geojson_path: str | Path, mapping_path: str | Path) -> None:
    Path(geojson_path).write_text(
        json.dumps(export.geojson, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    mapping = {
        "canonical_to_support_edges": {
            str(key): value for key, value in export.canonical_to_support_edges.items()
        },
        "support_to_canonical_edge": {
            str(key): value for key, value in export.support_to_canonical_edge.items()
        },
        "canonical_to_support_nodes": {
            str(key): value for key, value in export.canonical_to_support_nodes.items()
        },
    }
    Path(mapping_path).write_text(
        json.dumps(mapping, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


__all__ = ["RouteSupportExport", "export_route_support_graph", "save_route_support"]
