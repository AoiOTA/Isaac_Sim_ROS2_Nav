"""Reference A21 route-cost composition and Dijkstra implementation."""

from __future__ import annotations

import heapq
import math
from typing import Mapping

from .models import Edge, Graph, Traversability


def edge_cost(
    edge: Edge,
    settings: Mapping[str, float],
    *,
    prior_cost_delta_m: float = 0.0,
    prior_confidence: float = 0.0,
    runtime_penalty_m: float = 0.0,
    blocked: bool = False,
) -> float:
    if blocked or edge.static_traversability == Traversability.INFEASIBLE:
        return math.inf
    minimum = float(settings["minimum_clearance_m"])
    preferred = float(settings["preferred_clearance_m"])
    denominator = max(preferred - minimum, float(settings["numeric_epsilon"]))
    bottleneck = max(0.0, min(1.0, (preferred - edge.min_clearance_m) / denominator))
    geometry = edge.length_m * (
        1.0 + float(settings["clearance_penalty_weight"]) * bottleneck
    )
    prior_cap = float(settings["max_prior_cost_ratio_of_edge_length"]) * edge.length_m
    learned = max(0.0, min(float(prior_cost_delta_m), prior_cap)) * max(
        0.0, min(float(prior_confidence), 1.0)
    )
    return float(geometry + learned + max(0.0, float(runtime_penalty_m)))


def shortest_route(
    graph: Graph,
    start_node: int,
    goal_node: int,
    settings: Mapping[str, float],
    *,
    priors: Mapping[int, tuple[float, float]] | None = None,
    runtime: Mapping[int, tuple[float, bool]] | None = None,
) -> tuple[list[int], list[int], float]:
    prior_map = priors or {}
    runtime_map = runtime or {}
    outgoing: dict[int, list[Edge]] = {}
    for edge in graph.edges:
        outgoing.setdefault(edge.from_node, []).append(edge)
    distance = {int(start_node): 0.0}
    predecessor: dict[int, tuple[int, int]] = {}
    queue = [(0.0, int(start_node))]
    while queue:
        current_cost, node = heapq.heappop(queue)
        if current_cost != distance[node]:
            continue
        if node == int(goal_node):
            break
        for edge in outgoing.get(node, []):
            prior = prior_map.get(edge.id, (0.0, 0.0))
            dynamic = runtime_map.get(edge.id, (0.0, False))
            cost = edge_cost(
                edge,
                settings,
                prior_cost_delta_m=prior[0],
                prior_confidence=prior[1],
                runtime_penalty_m=dynamic[0],
                blocked=dynamic[1],
            )
            proposal = current_cost + cost
            if proposal < distance.get(edge.to_node, math.inf):
                distance[edge.to_node] = proposal
                predecessor[edge.to_node] = (node, edge.id)
                heapq.heappush(queue, (proposal, edge.to_node))
    if int(goal_node) not in distance:
        return [], [], math.inf
    nodes = [int(goal_node)]
    edges = []
    while nodes[-1] != int(start_node):
        previous, edge_id = predecessor[nodes[-1]]
        edges.append(edge_id)
        nodes.append(previous)
    return list(reversed(nodes)), list(reversed(edges)), float(distance[int(goal_node)])


__all__ = ["edge_cost", "shortest_route"]
