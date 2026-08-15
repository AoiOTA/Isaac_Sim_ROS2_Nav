"""ROS-independent graph models used by builders, tests, and adapters."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any

import numpy as np


class NodeType(IntEnum):
    ENDPOINT = 1
    JUNCTION = 2
    LOOP_ANCHOR = 3


class Traversability(IntEnum):
    UNKNOWN = 0
    FEASIBLE = 1
    INFEASIBLE = 2


@dataclass
class Node:
    id: int
    position_xy: tuple[float, float]
    degree: int
    node_type: NodeType
    clearance_m: float
    pixel_rc: tuple[int, int] | None = None


@dataclass
class Edge:
    id: int
    from_node: int
    to_node: int
    polyline_xy: np.ndarray
    length_m: float
    min_clearance_m: float
    mean_clearance_m: float
    p05_clearance_m: float
    nominal_width_m: float
    max_curvature_per_m: float
    bottleneck: bool
    static_traversability: Traversability = Traversability.UNKNOWN
    predecessor_ids: tuple[int, ...] = ()
    metadata: dict[str, Any] = field(default_factory=dict)


@dataclass
class Graph:
    graph_id: str
    revision: int
    map_version: str
    resolution_m: float
    nodes: list[Node]
    edges: list[Edge]

    def node_by_id(self) -> dict[int, Node]:
        return {node.id: node for node in self.nodes}

    def edge_by_id(self) -> dict[int, Edge]:
        return {edge.id: edge for edge in self.edges}


__all__ = [
    "Edge",
    "Graph",
    "Node",
    "NodeType",
    "Traversability",
]
