"""Stable canonical IDs across structural graph revisions."""

from __future__ import annotations

import math

import numpy as np

from .models import Edge, Graph


def _hausdorff(left: np.ndarray, right: np.ndarray) -> float:
    distances = np.linalg.norm(left[:, None, :] - right[None, :, :], axis=2)
    return float(max(distances.min(axis=1).max(), distances.min(axis=0).max()))


def _mean_distance_to_polyline(points: np.ndarray, polyline: np.ndarray) -> float:
    distances = np.linalg.norm(points[:, None, :] - polyline[None, :, :], axis=2)
    return float(distances.min(axis=1).mean())


def stabilize_graph_ids(graph: Graph, previous: Graph | None, settings: dict) -> Graph:
    if previous is None:
        return graph
    radius = float(settings["stable_node_match_radius_m"])
    candidates = []
    for node in graph.nodes:
        for old in previous.nodes:
            if node.node_type == old.node_type:
                distance = math.dist(node.position_xy, old.position_xy)
                if distance <= radius:
                    candidates.append((distance, node, old))
    used_new: set[int] = set()
    used_old: set[int] = set()
    node_mapping: dict[int, int] = {}
    for _, node, old in sorted(candidates, key=lambda item: (item[0], item[1].id, item[2].id)):
        if node.id in used_new or old.id in used_old:
            continue
        node_mapping[node.id] = old.id
        used_new.add(node.id)
        used_old.add(old.id)
    next_node_id = max((node.id for node in previous.nodes), default=0) + 1
    for node in sorted(graph.nodes, key=lambda item: item.id):
        original = node.id
        if original not in node_mapping:
            node_mapping[original] = next_node_id
            next_node_id += 1
        node.id = node_mapping[original]
    for edge in graph.edges:
        edge.from_node = node_mapping[edge.from_node]
        edge.to_node = node_mapping[edge.to_node]

    hausdorff_limit = float(settings["stable_edge_hausdorff_m"])
    ratio_limit = float(settings["stable_edge_length_ratio"])
    by_endpoints: dict[tuple[int, int], list[Edge]] = {}
    for edge in previous.edges:
        by_endpoints.setdefault((edge.from_node, edge.to_node), []).append(edge)
    matches = []
    for edge in graph.edges:
        for old in by_endpoints.get((edge.from_node, edge.to_node), []):
            ratio = abs(edge.length_m - old.length_m) / max(edge.length_m, old.length_m)
            distance = _hausdorff(edge.polyline_xy, old.polyline_xy)
            if ratio <= ratio_limit and distance <= hausdorff_limit:
                matches.append((distance, ratio, edge, old))
    used_new_edges: set[int] = set()
    used_old_edges: set[int] = set()
    edge_mapping: dict[int, int] = {}
    for _, _, edge, old in sorted(matches, key=lambda item: (item[0], item[1], item[2].id, item[3].id)):
        if edge.id in used_new_edges or old.id in used_old_edges:
            continue
        edge_mapping[edge.id] = old.id
        used_new_edges.add(edge.id)
        used_old_edges.add(old.id)
    next_edge_id = max((edge.id for edge in previous.edges), default=0) + 1
    for edge in sorted(graph.edges, key=lambda item: item.id):
        original = edge.id
        if original not in edge_mapping:
            predecessors = []
            for old in previous.edges:
                forward = _mean_distance_to_polyline(edge.polyline_xy, old.polyline_xy)
                reverse = _mean_distance_to_polyline(old.polyline_xy, edge.polyline_xy)
                if min(forward, reverse) <= hausdorff_limit:
                    predecessors.append(old.id)
            edge.predecessor_ids = tuple(sorted(predecessors))
            edge_mapping[original] = next_edge_id
            next_edge_id += 1
        edge.id = edge_mapping[original]
    graph.revision = previous.revision + 1
    return graph


__all__ = ["stabilize_graph_ids"]
