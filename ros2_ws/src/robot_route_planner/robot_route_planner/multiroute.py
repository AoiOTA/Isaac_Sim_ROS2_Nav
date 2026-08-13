"""Deterministic loopless-route analysis for a physical A21 GVG."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import csv
import heapq
from itertools import combinations
import json
import math
from pathlib import Path
from typing import Iterable, Mapping

from .diagnostics import physical_edges
from .models import Graph, Traversability
from .route_cost import edge_cost_breakdown


@dataclass(frozen=True)
class RouteAlternative:
    cost_m: float
    node_ids: tuple[int, ...]
    edge_ids: tuple[int, ...]


@dataclass(frozen=True)
class MultiRouteQuery:
    query_id: str
    start_node: int
    goal_node: int
    euclidean_separation_m: float
    shortest_cost_m: float
    second_cost_ratio: float
    meaningful_route_count: int
    enumerated_route_count: int
    shortest_second_edge_overlap: float
    score: float
    alternatives: tuple[RouteAlternative, ...]


def _adjacency(
    graph: Graph, route_cost_settings: Mapping[str, float]
) -> dict[int, list[tuple[int, int, float]]]:
    adjacency: dict[int, list[tuple[int, int, float]]] = {}
    for edge in graph.edges:
        if edge.static_traversability != Traversability.FEASIBLE:
            continue
        cost = edge_cost_breakdown(edge, route_cost_settings).final_cost_m
        adjacency.setdefault(edge.from_node, []).append(
            (edge.to_node, edge.id, float(cost))
        )
    for values in adjacency.values():
        values.sort(key=lambda item: (item[0], item[1], item[2]))
    return adjacency


def _shortest_route(
    adjacency: Mapping[int, Iterable[tuple[int, int, float]]],
    start: int,
    goal: int,
    *,
    banned_nodes: frozenset[int] = frozenset(),
    banned_edges: frozenset[int] = frozenset(),
) -> RouteAlternative | None:
    if start in banned_nodes or goal in banned_nodes:
        return None
    queue = [(0.0, (start,), (), start)]
    best: dict[int, float] = {}
    while queue:
        cost, nodes, edges, node = heapq.heappop(queue)
        if cost > best.get(node, math.inf) + 1.0e-12:
            continue
        best[node] = cost
        if node == goal:
            return RouteAlternative(float(cost), nodes, edges)
        for target, edge_id, weight in adjacency.get(node, ()):
            if (
                target in banned_nodes
                or edge_id in banned_edges
                or target in nodes
                or not math.isfinite(weight)
            ):
                continue
            heapq.heappush(
                queue,
                (cost + weight, nodes + (target,), edges + (edge_id,), target),
            )
    return None


def k_shortest_loopless_routes(
    graph: Graph,
    start: int,
    goal: int,
    route_cost_settings: Mapping[str, float],
    *,
    k: int = 8,
) -> tuple[RouteAlternative, ...]:
    """Return deterministic Yen-style K shortest loopless physical routes."""

    if k < 1:
        raise ValueError("k must be positive")
    adjacency = _adjacency(graph, route_cost_settings)
    first = _shortest_route(adjacency, int(start), int(goal))
    if first is None:
        return ()
    accepted = [first]
    candidates: list[
        tuple[float, tuple[int, ...], tuple[int, ...], RouteAlternative]
    ] = []
    candidate_keys: set[tuple[int, ...]] = set()
    accepted_keys = {first.edge_ids}

    edge_weight = {
        edge_id: weight
        for values in adjacency.values()
        for _, edge_id, weight in values
    }
    for _ in range(1, k):
        previous = accepted[-1]
        for spur_index in range(len(previous.node_ids) - 1):
            root_nodes = previous.node_ids[: spur_index + 1]
            root_edges = previous.edge_ids[:spur_index]
            banned_edges = {
                route.edge_ids[spur_index]
                for route in accepted
                if len(route.node_ids) > spur_index
                and route.node_ids[: spur_index + 1] == root_nodes
            }
            spur = _shortest_route(
                adjacency,
                root_nodes[-1],
                goal,
                banned_nodes=frozenset(root_nodes[:-1]),
                banned_edges=frozenset(banned_edges),
            )
            if spur is None:
                continue
            edge_ids = root_edges + spur.edge_ids
            if edge_ids in accepted_keys or edge_ids in candidate_keys:
                continue
            node_ids = root_nodes[:-1] + spur.node_ids
            cost = sum(edge_weight[edge_id] for edge_id in edge_ids)
            candidate = RouteAlternative(float(cost), node_ids, edge_ids)
            heapq.heappush(candidates, (cost, node_ids, edge_ids, candidate))
            candidate_keys.add(edge_ids)
        if not candidates:
            break
        _, _, key, selected = heapq.heappop(candidates)
        candidate_keys.remove(key)
        accepted.append(selected)
        accepted_keys.add(selected.edge_ids)
    return tuple(accepted)


def _physical_edge_keys(
    route: RouteAlternative, edge_lookup: Mapping[int, object]
) -> frozenset[tuple[int, int]]:
    return frozenset(
        tuple(sorted((edge_lookup[edge_id].from_node, edge_lookup[edge_id].to_node)))
        for edge_id in route.edge_ids
    )


def analyze_multiroute_queries(
    graph: Graph,
    route_cost_settings: Mapping[str, float],
    *,
    k: int = 8,
    minimum_separation_m: float = 4.0,
    maximum_cost_ratio: float = 1.60,
    maximum_shortest_overlap: float = 0.75,
) -> tuple[MultiRouteQuery, ...]:
    """Rank real graph queries by useful alternative-route choice space."""

    node_lookup = graph.node_by_id()
    edge_lookup = graph.edge_by_id()
    queries: list[MultiRouteQuery] = []
    for start, goal in combinations(sorted(node_lookup), 2):
        separation = math.dist(
            node_lookup[start].position_xy, node_lookup[goal].position_xy
        )
        if separation < minimum_separation_m:
            continue
        alternatives = k_shortest_loopless_routes(
            graph, start, goal, route_cost_settings, k=k
        )
        if len(alternatives) < 2:
            continue
        shortest = alternatives[0]
        shortest_keys = _physical_edge_keys(shortest, edge_lookup)
        meaningful = [shortest]
        overlaps = []
        for alternative in alternatives[1:]:
            keys = _physical_edge_keys(alternative, edge_lookup)
            overlap = len(keys & shortest_keys) / max(
                1, min(len(keys), len(shortest_keys))
            )
            overlaps.append(overlap)
            if (
                alternative.cost_m / shortest.cost_m <= maximum_cost_ratio
                and overlap <= maximum_shortest_overlap
            ):
                meaningful.append(alternative)
        if len(meaningful) < 2:
            continue
        second_ratio = meaningful[1].cost_m / shortest.cost_m
        second_overlap = overlaps[alternatives[1:].index(meaningful[1])]
        route_priority = min(len(meaningful), 4)
        score = (
            1000.0 * route_priority
            + 25.0 * min(separation, 16.0)
            + 100.0 * (1.0 - second_overlap)
            - 100.0 * (second_ratio - 1.0)
        )
        queries.append(
            MultiRouteQuery(
                query_id=f"Q{start:02d}_{goal:02d}",
                start_node=start,
                goal_node=goal,
                euclidean_separation_m=float(separation),
                shortest_cost_m=shortest.cost_m,
                second_cost_ratio=float(second_ratio),
                meaningful_route_count=len(meaningful),
                enumerated_route_count=len(alternatives),
                shortest_second_edge_overlap=float(second_overlap),
                score=float(score),
                alternatives=tuple(meaningful),
            )
        )
    return tuple(
        sorted(
            queries,
            key=lambda item: (
                -min(item.meaningful_route_count, 4),
                item.second_cost_ratio,
                -item.euclidean_separation_m,
                item.start_node,
                item.goal_node,
            ),
        )
    )


def save_multiroute_analysis(
    queries: Iterable[MultiRouteQuery], json_path: str | Path, csv_path: str | Path
) -> None:
    records = []
    for query in queries:
        record = asdict(query)
        record["alternatives"] = [asdict(item) for item in query.alternatives]
        records.append(record)
    Path(json_path).write_text(
        json.dumps(records, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    fields = [
        "query_id",
        "start_node",
        "goal_node",
        "euclidean_separation_m",
        "shortest_cost_m",
        "second_cost_ratio",
        "meaningful_route_count",
        "enumerated_route_count",
        "shortest_second_edge_overlap",
        "score",
    ]
    with Path(csv_path).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for record in records:
            writer.writerow({field: record[field] for field in fields})


def select_stratified_queries(
    queries: Iterable[MultiRouteQuery], *, per_tier: int = 5
) -> tuple[MultiRouteQuery, ...]:
    """Select a balanced 2/3/4+-route shortlist from ranked queries."""

    if per_tier < 1:
        raise ValueError("per_tier must be positive")
    ranked = tuple(queries)
    selected = []
    for tier in (2, 3, 4):
        tier_queries = [
            query
            for query in ranked
            if min(query.meaningful_route_count, 4) == tier
        ]
        selected.extend(tier_queries[:per_tier])
    return tuple(selected)


__all__ = [
    "MultiRouteQuery",
    "RouteAlternative",
    "analyze_multiroute_queries",
    "k_shortest_loopless_routes",
    "save_multiroute_analysis",
    "select_stratified_queries",
]
