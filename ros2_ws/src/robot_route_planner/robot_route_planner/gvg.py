"""Deterministic GVD/GVG extraction from a static occupancy map."""

from __future__ import annotations

from collections import defaultdict
import heapq
import math
from typing import Iterable

import cv2
import numpy as np

from .map_io import OccupancyMap
from .models import Edge, Graph, Node, NodeType


_NEIGHBORS = tuple(
    (dr, dc)
    for dr in (-1, 0, 1)
    for dc in (-1, 0, 1)
    if not (dr == 0 and dc == 0)
)


def _adjacent(pixel: tuple[int, int], shape: tuple[int, int]):
    row, column = pixel
    for dr, dc in _NEIGHBORS:
        other = row + dr, column + dc
        if 0 <= other[0] < shape[0] and 0 <= other[1] < shape[1]:
            yield other


def _rdp(points: np.ndarray, epsilon: float) -> np.ndarray:
    if len(points) <= 2:
        return points.copy()
    start, end = points[0], points[-1]
    vector = end - start
    denominator = float(np.linalg.norm(vector))
    if denominator <= np.finfo(np.float64).eps:
        distance = np.linalg.norm(points - start, axis=1)
    else:
        distance = np.abs(np.cross(vector, points - start) / denominator)
    index = int(np.argmax(distance))
    if float(distance[index]) <= epsilon:
        return points[[0, -1]].copy()
    left = _rdp(points[: index + 1], epsilon)
    right = _rdp(points[index:], epsilon)
    return np.vstack((left[:-1], right))


def _curvature(points: np.ndarray) -> float:
    if len(points) < 3:
        return 0.0
    first = np.diff(points, axis=0)
    lengths = np.linalg.norm(first, axis=1)
    headings = np.unwrap(np.arctan2(first[:, 1], first[:, 0]))
    changes = np.abs(np.diff(headings))
    scale = np.maximum((lengths[:-1] + lengths[1:]) * 0.5, np.finfo(float).eps)
    return float(np.max(changes / scale)) if len(changes) else 0.0


def voronoi_layers(
    occupancy: OccupancyMap,
    *,
    obstacle_site_min_separation_cells: float,
    obstacle_source_min_angle_deg: float,
    ridge_distance_difference_cells: float,
    minimum_free_component_area_m2: float,
    topology_clearance_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return diagnostic ridge, navigable topology skeleton, and clearance.

    Pixel labels alone change continuously along the same obstacle wall.  The
    raw diagnostic ridge therefore requires angularly separated obstacle source
    vectors.  The graph backbone uses topology-preserving Guo-Hall thinning of
    exact-clearance-qualified free space so the connected navigable topology is
    not fragmented by discrete ridge gaps.
    """
    component_count, component_labels, stats, _ = cv2.connectedComponentsWithStats(
        occupancy.free.astype(np.uint8), 8
    )
    cleaned_free = np.zeros_like(occupancy.free, dtype=bool)
    for component in range(1, component_count):
        area_m2 = (
            int(stats[component, cv2.CC_STAT_AREA]) * occupancy.resolution_m**2
        )
        if area_m2 >= minimum_free_component_area_m2:
            cleaned_free |= component_labels == component
    free_u8 = np.where(cleaned_free, 255, 0).astype(np.uint8)
    distance_cells = cv2.distanceTransform(
        free_u8, cv2.DIST_L2, cv2.DIST_MASK_PRECISE
    )
    _, labels = cv2.distanceTransformWithLabels(
        free_u8,
        cv2.DIST_L2,
        cv2.DIST_MASK_5,
        labelType=cv2.DIST_LABEL_PIXEL,
    )
    source_by_label: dict[int, tuple[int, int]] = {}
    for row, column in np.argwhere(~cleaned_free):
        label = int(labels[row, column])
        if label > 0:
            source_by_label[label] = (int(row), int(column))

    candidate = np.zeros_like(cleaned_free, dtype=np.uint8)
    shape = cleaned_free.shape
    for row, column in np.argwhere(cleaned_free):
        neighbor_labels = sorted(
            {
                int(labels[other])
                for other in _adjacent((int(row), int(column)), shape)
                if int(labels[other]) > 0
            }
        )
        accepted = False
        for left_index, left_label in enumerate(neighbor_labels):
            left = source_by_label.get(left_label)
            if left is None:
                continue
            for right_label in neighbor_labels[left_index + 1 :]:
                right = source_by_label.get(right_label)
                if right is None:
                    continue
                if math.dist(left, right) < obstacle_site_min_separation_cells:
                    continue
                left_vector = np.asarray(
                    [left[0] - row, left[1] - column], dtype=np.float64
                )
                right_vector = np.asarray(
                    [right[0] - row, right[1] - column], dtype=np.float64
                )
                left_distance = float(np.linalg.norm(left_vector))
                right_distance = float(np.linalg.norm(right_vector))
                cosine = float(
                    np.dot(left_vector, right_vector)
                    / max(left_distance * right_distance, np.finfo(float).eps)
                )
                angle_deg = math.degrees(
                    math.acos(max(-1.0, min(1.0, cosine)))
                )
                if angle_deg < obstacle_source_min_angle_deg:
                    continue
                if abs(left_distance - right_distance) <= ridge_distance_difference_cells:
                    accepted = True
                    break
            if accepted:
                break
        if accepted:
            candidate[row, column] = 255
    topology_free = cleaned_free & (
        distance_cells * occupancy.resolution_m >= topology_clearance_m
    )
    skeleton = cv2.ximgproc.thinning(
        np.where(topology_free, 255, 0).astype(np.uint8),
        thinningType=cv2.ximgproc.THINNING_GUOHALL,
    )
    skeleton[~cleaned_free] = 0
    return (
        candidate.astype(bool),
        skeleton.astype(bool),
        distance_cells * occupancy.resolution_m,
    )


def voronoi_skeleton(
    occupancy: OccupancyMap,
    *,
    obstacle_site_min_separation_cells: float,
    obstacle_source_min_angle_deg: float,
    ridge_distance_difference_cells: float,
    minimum_free_component_area_m2: float,
    topology_clearance_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    _, skeleton, clearance = voronoi_layers(
        occupancy,
        obstacle_site_min_separation_cells=obstacle_site_min_separation_cells,
        obstacle_source_min_angle_deg=obstacle_source_min_angle_deg,
        ridge_distance_difference_cells=ridge_distance_difference_cells,
        minimum_free_component_area_m2=minimum_free_component_area_m2,
        topology_clearance_m=topology_clearance_m,
    )
    return skeleton, clearance


def _skeleton_degree(skeleton: np.ndarray) -> np.ndarray:
    binary = skeleton.astype(np.uint8)
    kernel = np.ones((3, 3), dtype=np.uint8)
    return cv2.filter2D(binary, cv2.CV_16U, kernel) - binary


def _farthest_skeleton_pixel(
    skeleton: np.ndarray, start: tuple[int, int]
) -> tuple[int, int]:
    distance = {start: 0.0}
    queue = [(0.0, start)]
    while queue:
        current_distance, pixel = heapq.heappop(queue)
        if current_distance != distance[pixel]:
            continue
        for other in _adjacent(pixel, skeleton.shape):
            if not skeleton[other]:
                continue
            weight = math.sqrt(2.0) if pixel[0] != other[0] and pixel[1] != other[1] else 1.0
            proposal = current_distance + weight
            if proposal < distance.get(other, math.inf):
                distance[other] = proposal
                heapq.heappush(queue, (proposal, other))
    return max(distance, key=lambda pixel: (distance[pixel], pixel))


def _node_regions(
    skeleton: np.ndarray,
    clearance_m: np.ndarray,
    *,
    junction_merge_radius_cells: int,
    loop_anchor_min_separation_m: float,
    resolution_m: float,
) -> list[dict[str, object]]:
    degree = _skeleton_degree(skeleton)
    junction = skeleton & (degree >= 3)
    radius = max(0, int(junction_merge_radius_cells))
    if radius:
        size = 2 * radius + 1
        merged = cv2.dilate(junction.astype(np.uint8), np.ones((size, size), np.uint8))
    else:
        merged = junction.astype(np.uint8)
    count, labels = cv2.connectedComponents(merged)
    regions: list[dict[str, object]] = []
    occupied_by_region: set[tuple[int, int]] = set()
    for label in range(1, count):
        pixels = [
            tuple(map(int, pixel))
            for pixel in np.argwhere(skeleton & (labels == label))
        ]
        actual = [pixel for pixel in pixels if junction[pixel]]
        if not actual:
            continue
        representative = max(actual, key=lambda pixel: (float(clearance_m[pixel]), -pixel[0], -pixel[1]))
        region = set(pixels)
        occupied_by_region.update(region)
        regions.append({"pixels": region, "representative": representative, "type": NodeType.JUNCTION})

    endpoints = [
        tuple(map(int, pixel))
        for pixel in np.argwhere(skeleton & (degree == 1))
        if tuple(map(int, pixel)) not in occupied_by_region
    ]
    for pixel in endpoints:
        regions.append({"pixels": {pixel}, "representative": pixel, "type": NodeType.ENDPOINT})
        occupied_by_region.add(pixel)

    component_count, components = cv2.connectedComponents(skeleton.astype(np.uint8))
    for component in range(1, component_count):
        component_pixels = [tuple(map(int, pixel)) for pixel in np.argwhere(components == component)]
        if any(pixel in occupied_by_region for pixel in component_pixels):
            continue
        if len(component_pixels) < 2:
            continue
        first = max(component_pixels, key=lambda pixel: (float(clearance_m[pixel]), -pixel[0], -pixel[1]))
        second = _farthest_skeleton_pixel(skeleton & (components == component), first)
        if math.dist(first, second) * resolution_m < loop_anchor_min_separation_m:
            continue
        for pixel in (first, second):
            regions.append({"pixels": {pixel}, "representative": pixel, "type": NodeType.LOOP_ANCHOR})
            occupied_by_region.add(pixel)
    return regions


def _trace_chains(
    skeleton: np.ndarray, regions: list[dict[str, object]]
) -> list[tuple[int, int, list[tuple[int, int]]]]:
    region_of: dict[tuple[int, int], int] = {}
    for index, region in enumerate(regions):
        for pixel in region["pixels"]:  # type: ignore[index]
            region_of[pixel] = index
    visited: set[frozenset[tuple[int, int]]] = set()
    chains: list[tuple[int, int, list[tuple[int, int]]]] = []
    for source_index, region in enumerate(regions):
        representative = region["representative"]  # type: ignore[index]
        for boundary in sorted(region["pixels"]):  # type: ignore[index]
            for neighbor in _adjacent(boundary, skeleton.shape):
                if not skeleton[neighbor] or region_of.get(neighbor) == source_index:
                    continue
                step = frozenset((boundary, neighbor))
                if step in visited:
                    continue
                path = [representative]
                if boundary != representative:
                    path.append(boundary)
                previous, current = boundary, neighbor
                visited.add(step)
                path.append(current)
                while current not in region_of:
                    choices = [
                        item
                        for item in _adjacent(current, skeleton.shape)
                        if skeleton[item] and item != previous
                    ]
                    if not choices:
                        break
                    unvisited = [item for item in choices if frozenset((current, item)) not in visited]
                    if not unvisited:
                        break
                    following = sorted(unvisited)[0]
                    visited.add(frozenset((current, following)))
                    previous, current = current, following
                    path.append(current)
                target_index = region_of.get(current)
                if target_index is None:
                    continue
                target_rep = regions[target_index]["representative"]  # type: ignore[index]
                if path[-1] != target_rep:
                    path.append(target_rep)
                if len(path) >= 2:
                    chains.append((source_index, target_index, path))
    return chains


def build_gvg(
    occupancy: OccupancyMap,
    settings: dict,
    footprint_settings: dict,
    route_cost_settings: dict,
    *,
    revision: int = 1,
) -> Graph:
    skeleton, clearance = voronoi_skeleton(
        occupancy,
        obstacle_site_min_separation_cells=float(settings["obstacle_site_min_separation_cells"]),
        obstacle_source_min_angle_deg=float(settings["obstacle_source_min_angle_deg"]),
        ridge_distance_difference_cells=float(settings["ridge_distance_difference_cells"]),
        minimum_free_component_area_m2=float(settings["minimum_free_component_area_m2"]),
        topology_clearance_m=float(footprint_settings["padded_inscribed_radius_m"]),
    )
    regions = _node_regions(
        skeleton,
        clearance,
        junction_merge_radius_cells=int(settings["junction_merge_radius_cells"]),
        loop_anchor_min_separation_m=float(settings["loop_anchor_min_separation_m"]),
        resolution_m=occupancy.resolution_m,
    )
    chains = _trace_chains(skeleton, regions)

    sorted_regions = sorted(
        enumerate(regions),
        key=lambda item: (
            occupancy.pixel_to_world(*item[1]["representative"])[0],  # type: ignore[arg-type]
            occupancy.pixel_to_world(*item[1]["representative"])[1],  # type: ignore[arg-type]
            int(item[1]["type"]),
        ),
    )
    region_to_id = {region_index: index + 1 for index, (region_index, _) in enumerate(sorted_regions)}
    physical_degree: dict[int, int] = defaultdict(int)
    for source, target, _ in chains:
        physical_degree[source] += 1
        physical_degree[target] += 1
    nodes = []
    for region_index, region in sorted_regions:
        pixel = region["representative"]  # type: ignore[index]
        nodes.append(
            Node(
                id=region_to_id[region_index],
                position_xy=occupancy.pixel_to_world(*pixel),
                degree=physical_degree[region_index],
                node_type=region["type"],  # type: ignore[arg-type]
                clearance_m=float(clearance[pixel]),
                pixel_rc=pixel,
            )
        )

    records = []
    for source, target, pixel_path in chains:
        world = np.asarray([occupancy.pixel_to_world(*pixel) for pixel in pixel_path], dtype=np.float64)
        simplified = _rdp(world, float(settings["rdp_epsilon_m"]))
        segment = np.linalg.norm(np.diff(simplified, axis=0), axis=1)
        length = float(segment.sum())
        pixel_clearance = np.asarray([clearance[pixel] for pixel in pixel_path], dtype=np.float64)
        if length <= 0.0 or len(pixel_clearance) == 0:
            continue
        min_clearance = float(pixel_clearance.min())
        endpoint_artifact = (
            (regions[source]["type"] == NodeType.ENDPOINT or regions[target]["type"] == NodeType.ENDPOINT)
            and length <= float(settings["spur_max_length_m"])
            and min_clearance
            < float(footprint_settings["padded_inscribed_radius_m"])
            * float(settings["spur_clearance_ratio_of_inscribed"])
        )
        if endpoint_artifact:
            continue
        stats = {
            "length": length,
            "min": min_clearance,
            "mean": float(pixel_clearance.mean()),
            "p05": float(np.percentile(pixel_clearance, 5.0)),
            "curvature": _curvature(simplified),
        }
        records.append((region_to_id[source], region_to_id[target], simplified, stats))

    directed = []
    for source, target, polyline, stats in records:
        directed.append((source, target, polyline, stats))
        directed.append((target, source, polyline[::-1].copy(), stats))
    directed.sort(key=lambda item: (item[0], item[1], tuple(np.round(item[2].reshape(-1), 6))))
    edges = []
    preferred_clearance = float(route_cost_settings["preferred_clearance_m"])
    for edge_id, (source, target, polyline, stats) in enumerate(directed, start=1):
        edges.append(
            Edge(
                id=edge_id,
                from_node=source,
                to_node=target,
                polyline_xy=polyline,
                length_m=stats["length"],
                min_clearance_m=stats["min"],
                mean_clearance_m=stats["mean"],
                p05_clearance_m=stats["p05"],
                nominal_width_m=2.0 * stats["p05"],
                max_curvature_per_m=stats["curvature"],
                bottleneck=stats["min"] < preferred_clearance,
            )
        )
    retained_degree: dict[int, int] = defaultdict(int)
    for source, target, _, _ in records:
        retained_degree[source] += 1
        retained_degree[target] += 1
    used_nodes = {edge.from_node for edge in edges} | {edge.to_node for edge in edges}
    nodes = [node for node in nodes if node.id in used_nodes]
    for node in nodes:
        node.degree = retained_degree[node.id]
        if node.degree == 1:
            node.node_type = NodeType.ENDPOINT
        elif node.degree >= 3:
            node.node_type = NodeType.JUNCTION
    graph_id = f"{occupancy.map_version}:{settings['algorithm_version']}"
    return Graph(graph_id, int(revision), occupancy.map_version, occupancy.resolution_m, nodes, edges)


__all__ = ["build_gvg", "voronoi_layers", "voronoi_skeleton"]
