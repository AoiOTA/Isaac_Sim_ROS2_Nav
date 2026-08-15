"""Publish and visualize one persistent structural-map change for A21."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .defaults import load_engineering_defaults
from .diagnostics import graph_diagnostics
from .feasibility import apply_footprint_feasibility
from .gvg import build_gvg
from .map_io import OccupancyMap, load_occupancy_map
from .route_ab_visualize import _route_xy
from .route_support import export_route_support_graph
from .stable_ids import stabilize_graph_ids
from .visualize import (
    _map_background,
    _plot_graph,
    _pixel_overlay,
    _route_from_geojson,
)


def _add_disk_obstacle(
    occupancy: OccupancyMap, center_xy: tuple[float, float], radius_m: float
) -> tuple[OccupancyMap, np.ndarray]:
    rows, columns = np.indices(occupancy.free.shape)
    x = occupancy.origin_xy_m[0] + (columns + 0.5) * occupancy.resolution_m
    y = occupancy.origin_xy_m[1] + (
        occupancy.free.shape[0] - rows - 0.5
    ) * occupancy.resolution_m
    disk = (x - center_xy[0]) ** 2 + (y - center_xy[1]) ** 2 <= radius_m**2
    changed = disk & occupancy.free
    free = occupancy.free.copy()
    free[disk] = False
    return (
        OccupancyMap(
            free=free,
            resolution_m=occupancy.resolution_m,
            origin_xy_m=occupancy.origin_xy_m,
            map_version=f"{occupancy.map_version}:structural_probe",
            yaml_path=occupancy.yaml_path,
        ),
        changed,
    )


def build_visual_evidence(
    map_path: Path,
    defaults_path: Path,
    center_xy: tuple[float, float],
    radius_m: float,
    output_image: Path,
) -> tuple[dict, OccupancyMap]:
    defaults = load_engineering_defaults(defaults_path)
    occupancy = load_occupancy_map(
        map_path,
        unknown_is_occupied=bool(defaults["graph"]["unknown_is_occupied"]),
    )
    modified, changed = _add_disk_obstacle(occupancy, center_xy, radius_m)
    start = time.perf_counter()
    before = apply_footprint_feasibility(
        build_gvg(
            occupancy,
            defaults["graph"],
            defaults["footprint"],
            defaults["route_cost"],
        ),
        occupancy,
        defaults["footprint"],
    )
    after = apply_footprint_feasibility(
        build_gvg(
            modified,
            defaults["graph"],
            defaults["footprint"],
            defaults["route_cost"],
            revision=before.revision + 1,
        ),
        modified,
        defaults["footprint"],
    )
    after = stabilize_graph_ids(after, before, defaults["graph"])
    build_time = time.perf_counter() - start
    before_metrics = graph_diagnostics(before)
    after_metrics = graph_diagnostics(after)
    mission_start = int(before_metrics["start_node"])
    mission_goal = int(before_metrics["goal_node"])
    def support_route(graph):
        support = export_route_support_graph(
            graph,
            support_spacing_m=float(defaults["graph"]["route_support_spacing_m"]),
        )
        support_edges, _, cost = _route_from_geojson(
            support.geojson,
            support.canonical_to_support_nodes[mission_start],
            support.canonical_to_support_nodes[mission_goal],
            set(),
        )
        canonical = []
        for edge_id in support_edges:
            edge = support.support_to_canonical_edge[edge_id]
            if not canonical or canonical[-1] != edge:
                canonical.append(edge)
        return canonical, cost

    before_route, before_cost = support_route(before)
    try:
        after_route, after_cost = support_route(after)
    except (KeyError, RuntimeError):
        after_route, after_cost = [], float("inf")

    figure, axes = plt.subplots(1, 2, figsize=(17, 10), constrained_layout=True)
    _map_background(axes[0], occupancy)
    _plot_graph(axes[0], before)
    if before_route:
        before_xy = _route_xy(before, before_route)
        axes[0].plot(
            before_xy[:, 0], before_xy[:, 1], color="#2962ff",
            linewidth=2.2, label="pre-change route", zorder=8,
        )
    axes[0].set_title(
        f"Before: revision {before.revision}, {before_metrics['physical_edge_count']} edges, "
        f"{before_metrics['cycle_count']} cycle"
    )
    _map_background(axes[1], modified)
    _pixel_overlay(axes[1], occupancy, changed, (1.0, 0.0, 0.0), 0.75)
    _plot_graph(axes[1], after)
    if after_route:
        after_xy = _route_xy(after, after_route)
        axes[1].plot(
            after_xy[:, 0], after_xy[:, 1], color="#00c853",
            linewidth=2.2, label="post-rebuild route", zorder=8,
        )
    axes[1].scatter(
        center_xy[0], center_xy[1], marker="X", s=90, c="#d50000",
        edgecolors="black", label="persistent structural obstacle", zorder=9,
    )
    axes[1].set_title(
        f"After full rebuild: revision {after.revision}, "
        f"{after_metrics['physical_edge_count']} edges, "
        f"{after_metrics['cycle_count']} cycle"
    )
    axes[1].legend(loc="upper right", fontsize=8)
    output_image.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_image, dpi=180)
    plt.close(figure)
    return (
        {
            "classification": "engineering_visual_evidence",
            "image": output_image.name,
            "obstacle_center_xy_m": list(center_xy),
            "obstacle_radius_m": radius_m,
            "changed_free_cells": int(np.count_nonzero(changed)),
            "changed_area_m2": float(
                np.count_nonzero(changed) * occupancy.resolution_m**2
            ),
            "before": before_metrics,
            "after": after_metrics,
            "mission_start_node": mission_start,
            "mission_goal_node": mission_goal,
            "before_route_edge_ids": before_route,
            "before_route_cost": before_cost,
            "after_route_edge_ids": after_route,
            "after_route_cost": after_cost,
            "new_route_exists": bool(after_route),
            "offline_full_rebuild_time_s": build_time,
        },
        modified,
    )


def publish_until_rebuilt(
    occupancy: OccupancyMap, timeout_s: float, publish_rate_hz: float
) -> dict:
    import rclpy
    from bio_nav_interfaces.msg import StructuralGraphStatus
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Bool

    rclpy.init()
    node = rclpy.create_node("attempt30_a21_structural_probe")
    map_pub = node.create_publisher(
        OccupancyGrid, "/bio_nav/structural_map", 10
    )
    complete_pub = node.create_publisher(
        Bool, "/bio_nav/route_goal_complete", 10
    )
    events = []

    def status_callback(message) -> None:
        events.append(
            {
                "wall_time_s": time.monotonic(),
                "state": int(message.state),
                "revision": int(message.graph_revision),
                "detail": str(message.detail),
            }
        )

    node.create_subscription(
        StructuralGraphStatus,
        "/bio_nav/structural_graph_status",
        status_callback,
        10,
    )
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = float(occupancy.resolution_m)
    message.info.width = int(occupancy.free.shape[1])
    message.info.height = int(occupancy.free.shape[0])
    message.info.origin.position.x = float(occupancy.origin_xy_m[0])
    message.info.origin.position.y = float(occupancy.origin_xy_m[1])
    message.info.origin.orientation.w = 1.0
    message.data = np.where(occupancy.free, 0, 100).astype(np.int8).ravel().tolist()

    completed = Bool()
    completed.data = True
    start = time.monotonic()
    next_publish = start
    complete_pub.publish(completed)
    while rclpy.ok() and time.monotonic() - start < timeout_s:
        now = time.monotonic()
        if now >= next_publish:
            message.header.stamp = node.get_clock().now().to_msg()
            map_pub.publish(message)
            next_publish += 1.0 / publish_rate_hz
        rclpy.spin_once(node, timeout_sec=0.05)
        if events and events[-1]["state"] == StructuralGraphStatus.READY \
                and events[-1]["revision"] >= 2:
            break
    node.destroy_node()
    rclpy.shutdown()
    rebuilding = next(
        (item for item in events if item["state"] == StructuralGraphStatus.REBUILDING),
        None,
    )
    ready = next(
        (
            item for item in events
            if item["state"] == StructuralGraphStatus.READY
            and item["revision"] >= 2
        ),
        None,
    )
    return {
        "runtime_completed": ready is not None,
        "elapsed_to_ready_s": None if ready is None else ready["wall_time_s"] - start,
        "runtime_rebuild_time_s": (
            None if rebuilding is None or ready is None
            else ready["wall_time_s"] - rebuilding["wall_time_s"]
        ),
        "events": [
            {key: value for key, value in item.items() if key != "wall_time_s"}
            for item in events
        ],
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True, dest="map_path")
    parser.add_argument("--defaults", required=True, dest="defaults_path")
    parser.add_argument("--center", nargs=2, type=float, required=True)
    parser.add_argument("--radius", type=float, required=True)
    parser.add_argument("--output-image", required=True)
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--publish", action="store_true")
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args(argv)
    evidence, occupancy = build_visual_evidence(
        Path(args.map_path),
        Path(args.defaults_path),
        tuple(args.center),
        args.radius,
        Path(args.output_image),
    )
    if args.publish:
        evidence["runtime"] = publish_until_rebuilt(
            occupancy, args.timeout, publish_rate_hz=2.0
        )
    output = Path(args.output_json)
    if not args.publish and output.is_file():
        previous = json.loads(output.read_text(encoding="utf-8"))
        if (
            previous.get("obstacle_center_xy_m") == evidence["obstacle_center_xy_m"]
            and previous.get("obstacle_radius_m") == evidence["obstacle_radius_m"]
            and "runtime" in previous
        ):
            evidence["runtime"] = previous["runtime"]
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    print(json.dumps(evidence, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
