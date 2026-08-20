"""Physical validation adapter for Module2 cognitive graph candidates."""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
import re

import cv2
import numpy as np

from .feasibility import classify_edge, footprint_pose_is_free
from .map_io import OccupancyMap
from .models import Edge, Graph, Node, NodeType, Traversability


@dataclass(frozen=True)
class CognitiveGraphIdentity:
    reset_epoch: int
    recurrent_session_id: str
    map_version: str
    cognitive_tile_id: str
    tile_revision: int
    source_physical_graph_id: str
    source_physical_graph_revision: int
    model_id: str = ''


@dataclass(frozen=True)
class ValidatedCognitiveGraph:
    graph: Graph
    identity: CognitiveGraphIdentity
    source_sequence: int
    topology_revision: int
    value_sequence: int


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _duration_s(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def _inverse_rigid_transform(values) -> np.ndarray:
    transform = np.asarray(values, dtype=np.float64).reshape(3, 3)
    if not np.isfinite(transform).all():
        raise ValueError('transform is non-finite')
    rotation = transform[:2, :2]
    if (
        not np.allclose(transform[2], (0.0, 0.0, 1.0), atol=1.0e-8)
        or not np.allclose(rotation.T @ rotation, np.eye(2), atol=1.0e-6)
        or not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-6)
    ):
        raise ValueError('transform is not rigid SE(2)')
    return np.linalg.inv(transform)


def _to_map(inverse: np.ndarray, x: float, y: float) -> tuple[float, float]:
    point = inverse @ np.asarray([float(x), float(y), 1.0])
    if not np.isfinite(point).all() or not math.isclose(point[2], 1.0, abs_tol=1.0e-8):
        raise ValueError('transformed point is invalid')
    return float(point[0]), float(point[1])


def _edge(
    edge_id: int,
    source: int,
    target: int,
    points: np.ndarray,
    clearance: np.ndarray,
    occupancy: OccupancyMap,
    footprint: dict,
    metadata: dict,
) -> Edge:
    traversability = classify_edge(
        occupancy,
        points,
        footprint_polygon_m=np.asarray(footprint['polygon_m'], dtype=np.float64),
        footprint_padding_m=float(footprint['padding_m']),
        padded_inscribed_radius_m=float(footprint['padded_inscribed_radius_m']),
        sweep_sample_spacing_m=float(footprint['sweep_sample_spacing_m']),
    )
    if traversability != Traversability.FEASIBLE:
        raise ValueError(f'edge is not proven FEASIBLE ({traversability.name})')
    length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
    sampled_clearance = []
    for x, y in points:
        row, column = occupancy.world_to_pixel(float(x), float(y))
        if not (0 <= row < clearance.shape[0] and 0 <= column < clearance.shape[1]):
            raise ValueError('edge point is outside map')
        sampled_clearance.append(float(clearance[row, column]))
    minimum = min(sampled_clearance)
    mean = float(np.mean(sampled_clearance))
    p05 = float(np.percentile(sampled_clearance, 5.0))
    return Edge(
        edge_id,
        source,
        target,
        points,
        length,
        minimum,
        mean,
        p05,
        2.0 * minimum,
        0.0,
        minimum < float(footprint['padded_inscribed_radius_m']),
        Traversability.FEASIBLE,
        metadata=metadata,
    )


def validate_cognitive_graph_candidate(
    message,
    *,
    now_ns: int,
    expected: CognitiveGraphIdentity,
    last_source_sequence: int,
    occupancy: OccupancyMap,
    footprint: dict,
) -> ValidatedCognitiveGraph:
    """Reject any candidate that is not fresh, exact, rigid and physically free."""

    ttl_s = _duration_s(message.ttl)
    age_s = (int(now_ns) - _stamp_ns(message.header.stamp)) / 1.0e9
    if (
        message.schema_version != 'bio_nav_cognitive_place_graph_v1'
        or message.header.frame_id != 'module2_canvas'
        or not math.isfinite(ttl_s)
        or not 0.0 < ttl_s <= 0.5
        or not math.isfinite(age_s)
        or age_s < 0.0
        or age_s > ttl_s
    ):
        raise ValueError('candidate is stale or has invalid schema/frame')
    if int(message.source_sequence) <= int(last_source_sequence):
        raise ValueError('candidate source_sequence is not monotonic')
    observed = CognitiveGraphIdentity(
        int(message.reset_epoch),
        str(message.recurrent_session_id),
        str(message.map_version),
        str(message.cognitive_tile_id),
        int(message.tile_revision),
        str(message.source_physical_graph_id),
        int(message.source_physical_graph_revision),
        str(message.model_id),
    )
    identity_fields = (
        'reset_epoch', 'recurrent_session_id', 'map_version',
        'cognitive_tile_id', 'tile_revision', 'source_physical_graph_id',
        'source_physical_graph_revision',
    )
    if any(getattr(observed, name) != getattr(expected, name) for name in identity_fields):
        raise ValueError('candidate generation identity mismatch')
    if expected.model_id and observed.model_id != expected.model_id:
        raise ValueError('candidate model identity mismatch')
    if (
        not message.module2_healthy
        or not message.trusted_write
        or int(message.rejection_mask) != 0
        or not re.fullmatch(r'cpg-[0-9a-f]{24}', str(message.graph_id))
        or int(message.value_sequence) < 1
        or not observed.recurrent_session_id
        or not observed.cognitive_tile_id
        or not observed.model_id
    ):
        raise ValueError('candidate is unhealthy or incomplete')
    inverse = _inverse_rigid_transform(message.t_map_canvas)
    clearance = cv2.distanceTransform(
        occupancy.free.astype(np.uint8), cv2.DIST_L2, 5
    ) * occupancy.resolution_m
    nodes = []
    node_ids = {}
    for index, item in enumerate(message.nodes):
        external = str(item.node_id)
        if not external or external in node_ids:
            raise ValueError('duplicate or empty cognitive node id')
        position = _to_map(inverse, item.canvas_position.x, item.canvas_position.y)
        if not footprint_pose_is_free(
            occupancy,
            position,
            0.0,
            footprint_polygon_m=np.asarray(footprint['polygon_m'], dtype=np.float64),
            footprint_padding_m=float(footprint['padding_m']),
        ):
            raise ValueError('cognitive node footprint is occupied')
        row, column = occupancy.world_to_pixel(*position)
        node_id = index + 1
        node_ids[external] = node_id
        nodes.append(Node(
            node_id,
            position,
            0,
            NodeType.LOOP_ANCHOR,
            float(clearance[row, column]),
            (row, column),
        ))
    if len(nodes) < 2:
        raise ValueError('cognitive graph requires at least two nodes')
    edges = []
    directed_pairs = set()
    for item in message.edges:
        source_name = str(item.source_node_id)
        target_name = str(item.target_node_id)
        if (
            source_name not in node_ids
            or target_name not in node_ids
            or source_name == target_name
            or len(item.polyline_canvas) < 2
        ):
            raise ValueError('edge has bad references, self-loop, or zero geometry')
        points = np.asarray(
            [_to_map(inverse, point.x, point.y) for point in item.polyline_canvas],
            dtype=np.float64,
        )
        if float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()) <= 1.0e-6:
            raise ValueError('edge has zero length')
        directions = [(source_name, target_name, points)]
        if int(item.directionality) == int(item.DIRECTION_BIDIRECTIONAL):
            directions.append((target_name, source_name, points[::-1].copy()))
        elif int(item.directionality) != int(item.DIRECTION_DIRECTED):
            raise ValueError('edge directionality is invalid')
        for source_external, target_external, polyline in directions:
            pair = (source_external, target_external)
            if pair in directed_pairs:
                raise ValueError('duplicate directed edge')
            directed_pairs.add(pair)
            edges.append(_edge(
                len(edges) + 1,
                node_ids[source_external],
                node_ids[target_external],
                polyline,
                clearance,
                occupancy,
                footprint,
                {
                    'source': 'module2_cognitive',
                    'external_edge_id': str(item.edge_id),
                    'topology_revision': int(message.topology_revision),
                },
            ))
    adjacency = {node.id: set() for node in nodes}
    for edge in edges:
        adjacency[edge.from_node].add(edge.to_node)
        adjacency[edge.to_node].add(edge.from_node)
    visited = {nodes[0].id}
    pending = [nodes[0].id]
    while pending:
        current = pending.pop()
        for other in adjacency[current] - visited:
            visited.add(other)
            pending.append(other)
    if len(visited) != len(nodes):
        raise ValueError('cognitive graph is disconnected')
    for node in nodes:
        node.degree = len(adjacency[node.id])
        node.node_type = NodeType.ENDPOINT if node.degree == 1 else (
            NodeType.JUNCTION if node.degree >= 3 else NodeType.LOOP_ANCHOR
        )
    graph = Graph(
        f'{message.graph_id}:primary',
        max(int(message.topology_revision), expected.source_physical_graph_revision + 1),
        expected.map_version,
        occupancy.resolution_m,
        nodes,
        edges,
    )
    return ValidatedCognitiveGraph(
        graph,
        observed,
        int(message.source_sequence),
        int(message.topology_revision),
        int(message.value_sequence),
    )


def build_hybrid_graph(
    physical: Graph,
    cognitive: ValidatedCognitiveGraph,
    *,
    occupancy: OccupancyMap,
    footprint: dict,
) -> Graph:
    """Retain GVG and add only validated cognitive edges plus feasible connectors."""

    graph = copy.deepcopy(physical)
    clearance = cv2.distanceTransform(
        occupancy.free.astype(np.uint8), cv2.DIST_L2, 5
    ) * occupancy.resolution_m
    node_offset = max((node.id for node in graph.nodes), default=0)
    edge_offset = max((edge.id for edge in graph.edges), default=0)
    mapping = {}
    for item in cognitive.graph.nodes:
        clone = copy.deepcopy(item)
        clone.id = node_offset + item.id
        mapping[item.id] = clone.id
        graph.nodes.append(clone)
    for item in cognitive.graph.edges:
        clone = copy.deepcopy(item)
        clone.id = edge_offset + item.id
        clone.from_node = mapping[item.from_node]
        clone.to_node = mapping[item.to_node]
        graph.edges.append(clone)
    next_edge = max((edge.id for edge in graph.edges), default=0) + 1
    physical_nodes = physical.nodes
    for cognitive_node in cognitive.graph.nodes:
        nearest = min(
            physical_nodes,
            key=lambda item: math.dist(item.position_xy, cognitive_node.position_xy),
        )
        points = np.asarray(
            [nearest.position_xy, cognitive_node.position_xy], dtype=np.float64
        )
        connector = _edge(
            next_edge,
            nearest.id,
            mapping[cognitive_node.id],
            points,
            clearance,
            occupancy,
            footprint,
            {'source': 'module3_connector'},
        )
        reverse = copy.deepcopy(connector)
        reverse.id = next_edge + 1
        reverse.from_node, reverse.to_node = reverse.to_node, reverse.from_node
        reverse.polyline_xy = reverse.polyline_xy[::-1].copy()
        graph.edges.extend((connector, reverse))
        next_edge += 2
    graph.graph_id = f'{physical.graph_id}:hybrid:{cognitive.topology_revision}'
    graph.revision = max(physical.revision + 1, cognitive.topology_revision)
    return graph


__all__ = [
    'CognitiveGraphIdentity',
    'ValidatedCognitiveGraph',
    'build_hybrid_graph',
    'validate_cognitive_graph_candidate',
]
