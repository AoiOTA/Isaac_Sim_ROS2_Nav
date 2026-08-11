"""Small graph diagnostics for checking useful route choice space."""

from __future__ import annotations

from collections import defaultdict
import heapq
import math

from .models import Graph


def physical_edges(graph: Graph):
    """Return one direction of each bidirectional physical graph edge."""
    return [edge for edge in graph.edges if edge.from_node < edge.to_node]


def _adjacency(graph: Graph):
    adjacency = defaultdict(list)
    for index, edge in enumerate(physical_edges(graph)):
        adjacency[edge.from_node].append((edge.to_node, edge.length_m, index))
        adjacency[edge.to_node].append((edge.from_node, edge.length_m, index))
    return adjacency


def _components(graph: Graph, adjacency):
    unseen = {node.id for node in graph.nodes}
    components = []
    while unseen:
        component = {unseen.pop()}
        queue = list(component)
        while queue:
            current = queue.pop()
            for other, _, _ in adjacency[current]:
                if other in unseen:
                    unseen.remove(other)
                    component.add(other)
                    queue.append(other)
        components.append(component)
    return components


def _distances(start: int, allowed: set[int], adjacency):
    distances = {start: 0.0}
    queue = [(0.0, start)]
    while queue:
        distance, node = heapq.heappop(queue)
        if distance != distances[node]:
            continue
        for other, length, _ in adjacency[node]:
            if other not in allowed:
                continue
            proposal = distance + length
            if proposal < distances.get(other, math.inf):
                distances[other] = proposal
                heapq.heappush(queue, (proposal, other))
    return distances


def select_start_goal_probe(graph: Graph) -> tuple[int, int]:
    """Choose a reproducible, long endpoint pair in the largest component."""
    adjacency = _adjacency(graph)
    component = max(_components(graph, adjacency), key=lambda value: (len(value), -min(value)))
    endpoints = sorted(node for node in component if len(adjacency[node]) == 1)
    candidates = endpoints if len(endpoints) >= 2 else sorted(component)
    best = None
    for index, start in enumerate(candidates):
        distances = _distances(start, component, adjacency)
        for goal in candidates[index + 1 :]:
            candidate = (distances.get(goal, -math.inf), -start, -goal)
            if best is None or candidate > best[0]:
                best = (candidate, start, goal)
    if best is None:
        only = min(component)
        return only, only
    return best[1], best[2]


def count_simple_routes(
    graph: Graph,
    start_node: int,
    goal_node: int,
    *,
    cap: int = 64,
) -> tuple[int, bool]:
    """Count loopless physical-edge routes, stopping at a diagnostic cap."""
    if start_node == goal_node:
        return 1, False
    adjacency = _adjacency(graph)
    count = 0
    visited_nodes = {start_node}
    used_edges = set()

    def visit(node: int) -> None:
        nonlocal count
        if count >= cap:
            return
        if node == goal_node:
            count += 1
            return
        for other, _, edge_index in adjacency[node]:
            if other in visited_nodes or edge_index in used_edges:
                continue
            visited_nodes.add(other)
            used_edges.add(edge_index)
            visit(other)
            used_edges.remove(edge_index)
            visited_nodes.remove(other)

    visit(start_node)
    return count, count >= cap


def graph_diagnostics(
    graph: Graph,
    *,
    start_node: int | None = None,
    goal_node: int | None = None,
    route_count_cap: int = 64,
) -> dict:
    """Summarize sparsity without treating any value as a pass/fail gate."""
    adjacency = _adjacency(graph)
    components = _components(graph, adjacency)
    edges = physical_edges(graph)
    if start_node is None or goal_node is None:
        start_node, goal_node = select_start_goal_probe(graph)
    route_count, capped = count_simple_routes(
        graph, start_node, goal_node, cap=route_count_cap
    )
    return {
        "node_count": len(graph.nodes),
        "physical_edge_count": len(edges),
        "component_count": len(components),
        "cycle_count": len(edges) - len(graph.nodes) + len(components),
        "start_node": int(start_node),
        "goal_node": int(goal_node),
        "start_goal_route_count": route_count,
        "start_goal_alternative_route_count": max(0, route_count - 1),
        "start_goal_route_count_capped": capped,
    }


__all__ = [
    "count_simple_routes",
    "graph_diagnostics",
    "physical_edges",
    "select_start_goal_probe",
]
