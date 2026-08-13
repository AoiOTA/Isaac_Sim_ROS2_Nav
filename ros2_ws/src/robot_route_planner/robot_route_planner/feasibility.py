"""Conservative static footprint feasibility for canonical graph edges."""

from __future__ import annotations

import math

import cv2
import numpy as np

from .map_io import OccupancyMap
from .models import Graph, NodeType, Traversability


def _sample_polyline(points: np.ndarray, spacing_m: float) -> np.ndarray:
    segment = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment)))
    total = float(cumulative[-1])
    if total <= 0.0:
        return points[:1]
    distances = np.concatenate((np.arange(0.0, total, spacing_m), [total]))
    output = []
    for distance in distances:
        index = min(int(np.searchsorted(cumulative, distance, side="right") - 1), len(segment) - 1)
        fraction = 0.0 if segment[index] <= 0.0 else (
            distance - cumulative[index]
        ) / segment[index]
        position = points[index] + fraction * (points[index + 1] - points[index])
        tangent = points[index + 1] - points[index]
        output.append((position[0], position[1], math.atan2(tangent[1], tangent[0])))
    return np.asarray(output, dtype=np.float64)


def _polygon_is_free(
    occupancy: OccupancyMap,
    center_x: float,
    center_y: float,
    yaw: float,
    footprint_xy: np.ndarray,
) -> bool:
    rotation = np.asarray(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]],
        dtype=np.float64,
    )
    world = footprint_xy @ rotation.T + np.asarray([center_x, center_y])
    pixels = np.asarray(
        [occupancy.world_to_pixel(point[0], point[1]) for point in world],
        dtype=np.int32,
    )
    height, width = occupancy.free.shape
    if (
        np.any(pixels[:, 0] < 0)
        or np.any(pixels[:, 0] >= height)
        or np.any(pixels[:, 1] < 0)
        or np.any(pixels[:, 1] >= width)
    ):
        return False
    polygon = np.column_stack((pixels[:, 1], pixels[:, 0])).astype(np.int32)
    mask = np.zeros_like(occupancy.free, dtype=np.uint8)
    cv2.fillConvexPoly(mask, polygon, 1)
    return bool(np.all(occupancy.free[mask.astype(bool)]))


def footprint_pose_is_free(
    occupancy: OccupancyMap,
    center_xy: tuple[float, float],
    yaw_rad: float,
    *,
    footprint_polygon_m: np.ndarray,
    footprint_padding_m: float,
) -> bool:
    """Public conservative footprint predicate used by cognitive tiling."""

    polygon = np.asarray(footprint_polygon_m, dtype=np.float64)
    if polygon.ndim != 2 or polygon.shape[1] != 2:
        raise ValueError("footprint polygon must be [N,2]")
    radial = np.linalg.norm(polygon, axis=1)
    padded = polygon.copy()
    nonzero = radial > np.finfo(float).eps
    padded[nonzero] += (
        float(footprint_padding_m)
        * polygon[nonzero]
        / radial[nonzero, None]
    )
    return _polygon_is_free(
        occupancy,
        float(center_xy[0]),
        float(center_xy[1]),
        float(yaw_rad),
        padded,
    )


def _endpoints_disconnected_after_disk_erosion(
    occupancy: OccupancyMap,
    start_xy: np.ndarray,
    end_xy: np.ndarray,
    radius_m: float,
) -> bool:
    radius_cells = int(math.ceil(radius_m / occupancy.resolution_m))
    kernel_size = 2 * radius_cells + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, (kernel_size, kernel_size)
    )
    eroded = cv2.erode(occupancy.free.astype(np.uint8), kernel).astype(bool)
    _, labels = cv2.connectedComponents(eroded.astype(np.uint8))
    start = occupancy.world_to_pixel(float(start_xy[0]), float(start_xy[1]))
    end = occupancy.world_to_pixel(float(end_xy[0]), float(end_xy[1]))
    height, width = eroded.shape
    if not (
        0 <= start[0] < height
        and 0 <= start[1] < width
        and 0 <= end[0] < height
        and 0 <= end[1] < width
    ):
        return False
    start_label = int(labels[start])
    end_label = int(labels[end])
    return bool(start_label > 0 and end_label > 0 and start_label != end_label)


def classify_edge(
    occupancy: OccupancyMap,
    polyline_xy: np.ndarray,
    *,
    footprint_polygon_m: np.ndarray,
    footprint_padding_m: float,
    padded_inscribed_radius_m: float,
    sweep_sample_spacing_m: float,
) -> Traversability:
    points = np.asarray(polyline_xy, dtype=np.float64)
    polygon = np.asarray(footprint_polygon_m, dtype=np.float64)
    if len(points) < 2 or polygon.ndim != 2 or polygon.shape[1] != 2:
        raise ValueError("edge and footprint geometries are invalid")
    radial = np.linalg.norm(polygon, axis=1)
    padded = polygon.copy()
    nonzero = radial > np.finfo(float).eps
    padded[nonzero] += (
        footprint_padding_m * polygon[nonzero] / radial[nonzero, None]
    )
    poses = _sample_polyline(points, sweep_sample_spacing_m)
    if all(
        _polygon_is_free(occupancy, pose[0], pose[1], pose[2], padded)
        for pose in poses
    ):
        return Traversability.FEASIBLE
    if _endpoints_disconnected_after_disk_erosion(
        occupancy, points[0], points[-1], padded_inscribed_radius_m
    ):
        return Traversability.INFEASIBLE
    return Traversability.UNKNOWN


def apply_footprint_feasibility(
    graph: Graph, occupancy: OccupancyMap, settings: dict
) -> Graph:
    cache: dict[tuple[int, int], Traversability] = {}
    for edge in graph.edges:
        key = tuple(sorted((edge.from_node, edge.to_node)))
        if key not in cache:
            cache[key] = classify_edge(
                occupancy,
                edge.polyline_xy,
                footprint_polygon_m=np.asarray(settings["polygon_m"], dtype=np.float64),
                footprint_padding_m=float(settings["padding_m"]),
                padded_inscribed_radius_m=float(settings["padded_inscribed_radius_m"]),
                sweep_sample_spacing_m=float(settings["sweep_sample_spacing_m"]),
            )
        edge.static_traversability = cache[key]
    return graph


def retain_largest_feasible_component(graph: Graph) -> dict[str, int]:
    """Retain only the largest connected graph proven feasible by footprint sweep."""

    physical = [edge for edge in graph.edges if edge.from_node < edge.to_node]
    raw_nodes = len(graph.nodes)
    raw_edges = len(physical)
    raw_unknown = sum(
        edge.static_traversability == Traversability.UNKNOWN for edge in graph.edges
    )
    adjacency: dict[int, set[int]] = {}
    for edge in graph.edges:
        if edge.static_traversability != Traversability.FEASIBLE:
            continue
        adjacency.setdefault(edge.from_node, set()).add(edge.to_node)
        adjacency.setdefault(edge.to_node, set()).add(edge.from_node)
    unseen = set(adjacency)
    components = []
    while unseen:
        component = {unseen.pop()}
        queue = list(component)
        while queue:
            node = queue.pop()
            for other in adjacency.get(node, ()):
                if other in unseen:
                    unseen.remove(other)
                    component.add(other)
                    queue.append(other)
        components.append(component)
    if not components:
        raise RuntimeError("graph has no footprint-feasible component")
    retained = max(components, key=lambda value: (len(value), -min(value)))
    graph.edges = [
        edge
        for edge in graph.edges
        if edge.static_traversability == Traversability.FEASIBLE
        and edge.from_node in retained
        and edge.to_node in retained
    ]
    degree: dict[int, int] = {node: 0 for node in retained}
    for edge in graph.edges:
        if edge.from_node < edge.to_node:
            degree[edge.from_node] += 1
            degree[edge.to_node] += 1
    graph.nodes = [node for node in graph.nodes if node.id in retained]
    for node in graph.nodes:
        node.degree = degree[node.id]
        if node.degree == 1:
            node.node_type = NodeType.ENDPOINT
        elif node.degree >= 3:
            node.node_type = NodeType.JUNCTION
        else:
            node.node_type = NodeType.LOOP_ANCHOR
    retained_physical = sum(
        edge.from_node < edge.to_node for edge in graph.edges
    )
    return {
        "raw_node_count": raw_nodes,
        "raw_physical_edge_count": raw_edges,
        "raw_unknown_directed_edge_count": raw_unknown,
        "discarded_node_count": raw_nodes - len(graph.nodes),
        "discarded_physical_edge_count": raw_edges - retained_physical,
    }


__all__ = [
    "apply_footprint_feasibility",
    "classify_edge",
    "footprint_pose_is_free",
    "retain_largest_feasible_component",
]
