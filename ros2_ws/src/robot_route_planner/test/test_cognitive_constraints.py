from __future__ import annotations

from pathlib import Path

import numpy as np

from robot_route_planner.cognitive_constraints import (
    CognitiveConstraintsCache,
    build_cognitive_constraints,
    occupancy_grid_version,
)
from robot_route_planner.map_io import OccupancyMap


def _footprint() -> dict:
    return {
        "polygon_m": [
            [0.10, 0.08],
            [0.10, -0.08],
            [-0.10, -0.08],
            [-0.10, 0.08],
        ],
        "padding_m": 0.02,
        "padded_inscribed_radius_m": 0.10,
        "sweep_sample_spacing_m": 0.05,
    }


def test_cognitive_constraints_never_cross_a_physical_wall() -> None:
    free = np.ones((320, 320), dtype=bool)
    # 5 cm grid, map origin -8 m: this wall separates canvas columns 7 and 8.
    free[:, 160:162] = False
    occupancy = OccupancyMap(free, 0.05, (-8.0, -8.0), "test", Path("map.yaml"))
    value = build_cognitive_constraints(
        occupancy,
        map_version="a" * 64,
        graph_revision=7,
        footprint_settings=_footprint(),
    )
    transitions = {tuple(pair) for pair in value.verified_transitions.tolist()}
    assert value.reachable_state_mask.any()
    assert (7, 8) not in transitions
    assert (8, 7) not in transitions
    assert (6, 7) in transitions
    assert value.graph_revision == 7
    assert value.tile_revision == 7


def test_occupancy_grid_version_binds_geometry_and_exact_int8_bytes() -> None:
    data = np.asarray((0, 100, -1, 0), dtype=np.int8)
    first = occupancy_grid_version(
        width=2,
        height=2,
        resolution=0.05,
        origin_x=-1.0,
        origin_y=-2.0,
        data=data,
    )
    second = occupancy_grid_version(
        width=2,
        height=2,
        resolution=0.05,
        origin_x=-1.0,
        origin_y=-2.0,
        data=data.copy(),
    )
    changed = occupancy_grid_version(
        width=2,
        height=2,
        resolution=0.05,
        origin_x=-1.0,
        origin_y=-2.0,
        data=np.asarray((0, 100, 0, 0), dtype=np.int8),
    )
    assert first == second
    assert first != changed
    assert len(first) == 64


def test_cognitive_constraints_accept_stable_outdoor_region_identity() -> None:
    occupancy = OccupancyMap(
        np.ones((400, 400), dtype=bool),
        0.05,
        (92.0, 192.0),
        "outdoor",
        Path("map.yaml"),
    )
    transform = np.asarray(((1.0, 0.0, -100.0), (0.0, 1.0, -200.0), (0.0, 0.0, 1.0)))
    value = build_cognitive_constraints(
        occupancy,
        map_version="rivermark",
        graph_revision=3,
        footprint_settings=_footprint(),
        t_map_canvas=transform,
        cognitive_tile_id="rivermark:r00c00",
    )
    assert value.cognitive_tile_id == "rivermark:r00c00"
    assert np.array_equal(value.t_map_canvas, transform)
    assert value.reachable_state_mask.all()


def test_tile_cache_reuses_revision_bound_constraints_and_invalidates_values() -> None:
    occupancy = OccupancyMap(
        np.ones((320, 320), dtype=bool),
        0.05,
        (-8.0, -8.0),
        "outdoor",
        Path("map.yaml"),
    )
    value = build_cognitive_constraints(
        occupancy,
        map_version="map-a",
        graph_revision=4,
        footprint_settings=_footprint(),
        cognitive_tile_id="tile-07",
    )
    key = ("map-a", 4, "tile-07")
    cache = CognitiveConstraintsCache()

    assert cache.get(key) is None
    cache.put(key, value)
    assert cache.get(key) is value
    assert (cache.hits, cache.misses, len(cache.values)) == (1, 1, 1)

    cache.invalidate()
    assert cache.get(key) is None
    assert (cache.hits, cache.misses, len(cache.values)) == (1, 2, 0)
