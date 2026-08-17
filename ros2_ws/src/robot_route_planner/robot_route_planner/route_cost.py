"""Reference A21 route-cost composition and Dijkstra implementation."""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass
from typing import Mapping

from .models import Edge, Graph, Traversability


@dataclass(frozen=True)
class EdgeCostBreakdown:
    structural_cost_m: float
    requested_module2_delta_m: float
    applied_module2_delta_m: float
    runtime_penalty_m: float
    final_cost_m: float
    blocked: bool


_ESTIMATED_CLEARANCE_KEYS = (
    ("estimated_minimum_clearance_m", "minimum_clearance_m"),
    ("estimated_preferred_clearance_m", "preferred_clearance_m"),
    ("estimated_clearance_penalty_weight", "clearance_penalty_weight"),
)


def resolve_route_cost_settings(
    route_cost_defaults: Mapping[str, float],
    localization_backend: str = "",
) -> dict[str, float]:
    """Return the effective route-cost settings for the localization backend.

    The AMCL estimated-localization chain carries ~0.15 m p95 pose error, so
    the engineering-defaults ``estimated_*`` keys widen the clearance
    economics.  Every other backend keeps the qualified A21 values, and a
    defaults file without estimated keys fails open to them as well.
    """
    settings = dict(route_cost_defaults)
    if localization_backend.strip().lower() == "amcl":
        for estimated_key, standard_key in _ESTIMATED_CLEARANCE_KEYS:
            if estimated_key in settings:
                settings[standard_key] = float(settings[estimated_key])
    return settings


def edge_cost_breakdown(
    edge: Edge,
    settings: Mapping[str, float],
    *,
    prior_cost_delta_m: float = 0.0,
    prior_confidence: float = 0.0,
    runtime_penalty_m: float = 0.0,
    blocked: bool = False,
) -> EdgeCostBreakdown:
    physically_blocked = bool(
        blocked or edge.static_traversability == Traversability.INFEASIBLE
    )
    minimum = float(settings["minimum_clearance_m"])
    preferred = float(settings["preferred_clearance_m"])
    denominator = max(preferred - minimum, float(settings["numeric_epsilon"]))
    bottleneck = max(
        0.0, min(1.0, (preferred - edge.min_clearance_m) / denominator)
    )
    structural = edge.length_m * (
        1.0 + float(settings["clearance_penalty_weight"]) * bottleneck
    )
    requested = max(0.0, float(prior_cost_delta_m))
    prior_cap = (
        float(settings["max_prior_cost_ratio_of_edge_length"]) * edge.length_m
    )
    applied = min(requested, prior_cap) * max(
        0.0, min(float(prior_confidence), 1.0)
    )
    runtime = max(0.0, float(runtime_penalty_m))
    final = math.inf if physically_blocked else structural + applied + runtime
    return EdgeCostBreakdown(
        structural_cost_m=float(structural),
        requested_module2_delta_m=requested,
        applied_module2_delta_m=float(applied),
        runtime_penalty_m=runtime,
        final_cost_m=float(final),
        blocked=physically_blocked,
    )


def edge_cost(
    edge: Edge,
    settings: Mapping[str, float],
    *,
    prior_cost_delta_m: float = 0.0,
    prior_confidence: float = 0.0,
    runtime_penalty_m: float = 0.0,
    blocked: bool = False,
) -> float:
    return edge_cost_breakdown(
        edge,
        settings,
        prior_cost_delta_m=prior_cost_delta_m,
        prior_confidence=prior_confidence,
        runtime_penalty_m=runtime_penalty_m,
        blocked=blocked,
    ).final_cost_m


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


__all__ = [
    "EdgeCostBreakdown",
    "edge_cost",
    "edge_cost_breakdown",
    "resolve_route_cost_settings",
    "shortest_route",
]
