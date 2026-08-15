from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

from robot_route_planner.benchmark_environment import (
    _directed_execution_query,
    generate_benchmark_bundle,
    load_benchmark_layout,
    rasterize_layout,
)
from robot_route_planner.defaults import load_engineering_defaults
from robot_route_planner.feasibility import (
    _polygon_is_free,
    apply_footprint_feasibility,
    retain_largest_feasible_component,
)
from robot_route_planner.gvg import build_gvg
from robot_route_planner.multiroute import (
    analyze_multiroute_queries,
    k_shortest_loopless_routes,
    select_stratified_queries,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
INTEGRATION_DEFAULTS = Path(
    "/home/lyb/Workspace/Bio_Nav/worktrees/integration/"
    "attempt30-a21-v310-srdr-rviz/ros2_ws/src/bio_nav_ros_bridge/"
    "config/engineering_defaults.yaml"
)
LAYOUT = (
    REPOSITORY_ROOT
    / "isaac_sim/configs/benchmarks/attempt30_a21_multiroute_v1.yaml"
)
LAYOUT_V3 = (
    REPOSITORY_ROOT
    / "isaac_sim/configs/benchmarks/attempt30_a21_multiroute_v3.yaml"
)
LAYOUT_V4 = (
    REPOSITORY_ROOT
    / "isaac_sim/configs/benchmarks/attempt30_a21_multiroute_v4.yaml"
)
ASSET_ROOT = Path("/home/lyb/isaacsim_assets/Assets/Isaac/6.0")
FOCUS_DYNAMIC_CONFIGS = tuple(
    REPOSITORY_ROOT
    / "isaac_sim/configs/benchmarks"
    / f"attempt30_a21_{query}_dynamic.yaml"
    for query in ("q02_58", "q01_50", "q36_04", "q14_45")
) + (
    REPOSITORY_ROOT
    / "isaac_sim/configs/benchmarks/attempt30_a21_multiroute_v4_benefit_dynamic.yaml",
)


def _graph():
    layout = load_benchmark_layout(LAYOUT)
    defaults = load_engineering_defaults(INTEGRATION_DEFAULTS)
    occupancy = rasterize_layout(layout, Path("/tmp/attempt30_a21_multiroute_v1.yaml"))
    graph = build_gvg(
        occupancy,
        defaults["graph"],
        defaults["footprint"],
        defaults["route_cost"],
    )
    apply_footprint_feasibility(graph, occupancy, defaults["footprint"])
    retain_largest_feasible_component(graph)
    return graph, defaults


def test_v1_baseline_meets_minimum_topology_and_query_targets():
    graph, defaults = _graph()
    queries = analyze_multiroute_queries(graph, defaults["route_cost"])
    physical_edge_count = len(graph.edges) // 2
    cycle_rank = physical_edge_count - len(graph.nodes) + 1

    assert len(graph.nodes) == 12
    assert len(graph.edges) == 40
    assert sum(node.degree >= 3 for node in graph.nodes) == 12
    assert sum(node.degree >= 4 for node in graph.nodes) == 4
    assert cycle_rank == 9
    assert all(edge.static_traversability.name == "FEASIBLE" for edge in graph.edges)
    assert len(queries) >= 15
    assert sum(query.meaningful_route_count >= 3 for query in queries) >= 2
    assert any(query.meaningful_route_count >= 4 for query in queries)


def test_yen_routes_are_loopless_unique_and_cost_sorted():
    graph, defaults = _graph()
    routes = k_shortest_loopless_routes(
        graph, 3, 10, defaults["route_cost"], k=8
    )

    assert len(routes) == 8
    assert list(routes) == sorted(routes, key=lambda item: (item.cost_m, item.node_ids, item.edge_ids))
    assert len({item.edge_ids for item in routes}) == len(routes)
    assert all(len(item.node_ids) == len(set(item.node_ids)) for item in routes)
    assert all(len(item.edge_ids) + 1 == len(item.node_ids) for item in routes)


def test_v3_is_asymmetric_rotated_and_retains_real_choice_space():
    layout = load_benchmark_layout(LAYOUT_V3)
    defaults = load_engineering_defaults(INTEGRATION_DEFAULTS)
    occupancy = rasterize_layout(layout, Path("/tmp/attempt30_a21_multiroute_v3.yaml"))
    graph = build_gvg(
        occupancy,
        defaults["graph"],
        defaults["footprint"],
        defaults["route_cost"],
    )
    apply_footprint_feasibility(graph, occupancy, defaults["footprint"])
    retain_largest_feasible_component(graph)
    queries = analyze_multiroute_queries(graph, defaults["route_cost"])

    interior = [item for item in layout.obstacles if item.role != "outer_boundary"]
    assert layout.status == "candidate"
    assert len({round(item.yaw_deg, 6) for item in interior}) >= 8
    assert len({tuple(item.size_xyz[:2]) for item in interior}) >= 12
    assert sum(node.degree >= 3 for node in graph.nodes) >= 10
    assert len(graph.edges) // 2 - len(graph.nodes) + 1 >= 6
    assert len(queries) >= 15
    assert any(query.meaningful_route_count == 2 for query in queries)
    assert any(query.meaningful_route_count == 3 for query in queries)
    assert any(query.meaningful_route_count >= 4 for query in queries)
    assert all(edge.static_traversability.name == "FEASIBLE" for edge in graph.edges)


def test_v4_wall_field_has_semantic_topology_and_balanced_shortlist():
    layout = load_benchmark_layout(LAYOUT_V4)
    defaults = load_engineering_defaults(INTEGRATION_DEFAULTS)
    occupancy = rasterize_layout(layout, Path("/tmp/attempt30_a21_multiroute_v4.yaml"))
    graph = build_gvg(
        occupancy,
        defaults["graph"],
        defaults["footprint"],
        defaults["route_cost"],
    )
    apply_footprint_feasibility(graph, occupancy, defaults["footprint"])
    retain_largest_feasible_component(graph)
    queries = analyze_multiroute_queries(graph, defaults["route_cost"])
    shortlist = select_stratified_queries(queries)

    roles = {item.role for item in layout.obstacles}
    assert layout.status == "frozen"
    assert {"structural_wall", "bottleneck_wall", "dead_end_wall"} <= roles
    assert len(layout.obstacles) >= 25
    assert sum(node.degree >= 3 for node in graph.nodes) >= 10
    assert len(graph.edges) // 2 - len(graph.nodes) + 1 >= 6
    assert len(queries) >= 15
    assert len(shortlist) == 15
    assert [min(query.meaningful_route_count, 4) for query in shortlist].count(2) == 5
    assert [min(query.meaningful_route_count, 4) for query in shortlist].count(3) == 5
    assert [min(query.meaningful_route_count, 4) for query in shortlist].count(4) == 5
    assert all(edge.static_traversability.name == "FEASIBLE" for edge in graph.edges)


def test_v4_focus_matrix_has_four_directed_eight_route_queries():
    layout = load_benchmark_layout(LAYOUT_V4)
    defaults = load_engineering_defaults(INTEGRATION_DEFAULTS)
    occupancy = rasterize_layout(layout, Path("/tmp/attempt30_a21_multiroute_v4_focus.yaml"))
    graph = build_gvg(
        occupancy,
        defaults["graph"],
        defaults["footprint"],
        defaults["route_cost"],
        revision=layout.revision,
    )
    apply_footprint_feasibility(graph, occupancy, defaults["footprint"])
    retain_largest_feasible_component(graph)

    queries = [
        _directed_execution_query(graph, start, goal, defaults["route_cost"])
        for start, goal in ((2, 58), (1, 50), (36, 4), (14, 45))
    ]
    assert [query.query_id for query in queries] == [
        "Q02_58", "Q01_50", "Q36_04", "Q14_45"
    ]
    assert all(query.enumerated_route_count == 8 for query in queries)
    assert all(query.meaningful_route_count >= 5 for query in queries)
    assert all(
        len({route.edge_ids for route in query.alternatives})
        == query.meaningful_route_count
        for query in queries
    )


@pytest.mark.parametrize("config_path", FOCUS_DYNAMIC_CONFIGS)
def test_v4_focus_dynamic_actor_sweeps_stay_in_static_free_space(config_path):
    import numpy as np
    import yaml

    layout = load_benchmark_layout(LAYOUT_V4)
    occupancy = rasterize_layout(
        layout, Path(f"/tmp/{config_path.stem}_map.yaml")
    )
    scenario = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    case_id, case = next(iter(scenario["cases"].items()))
    obstacle = case["obstacle"]
    half_x = obstacle["size"][0] / 2.0 + 0.05
    half_y = obstacle["size"][1] / 2.0 + 0.05
    actor_footprint = np.asarray([
        [half_x, half_y], [half_x, -half_y],
        [-half_x, -half_y], [-half_x, half_y],
    ])
    for start, end in zip(obstacle["waypoints"], obstacle["waypoints"][1:]):
        distance = np.linalg.norm(np.asarray(end[:2]) - np.asarray(start[:2]))
        for ratio in np.linspace(0.0, 1.0, max(2, int(distance / 0.025) + 1)):
            x = start[0] + ratio * (end[0] - start[0])
            y = start[1] + ratio * (end[1] - start[1])
            assert _polygon_is_free(occupancy, x, y, 0.0, actor_footprint), (
                config_path.name, case_id, x, y
            )


def test_generated_usd_has_flat_grid_reference_and_collision_on_every_box(tmp_path):
    pxr = pytest.importorskip("pxr")
    from pxr import Usd, UsdPhysics

    summary = generate_benchmark_bundle(
        LAYOUT, INTEGRATION_DEFAULTS, ASSET_ROOT, tmp_path
    )
    assert summary["minimum_targets_met"] is True
    assert summary["environment_frozen"] is False
    assert summary["graph"]["component_count"] == 1

    stage = Usd.Stage.Open(str(tmp_path / "attempt30_a21_multiroute_v1.usda"))
    assert stage
    assert stage.GetPrimAtPath("/World/GroundPlane/CollisionPlane").HasAPI(
        UsdPhysics.CollisionAPI
    )
    benchmark_root = stage.GetPrimAtPath("/World/BenchmarkGeometry")
    colliders = [
        prim for prim in Usd.PrimRange(benchmark_root) if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    layout = load_benchmark_layout(LAYOUT)
    assert len(colliders) == len(layout.obstacles)
    assert all(
        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True
        for prim in colliders
    )
    flat_grid = ASSET_ROOT / layout.base_asset_relative_path
    assert summary["base_environment"]["sha256"] == hashlib.sha256(flat_grid.read_bytes()).hexdigest()
