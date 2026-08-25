from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from robot_route_planner.cognitive_constraints import (
    CognitiveConstraintsCache,
    build_cognitive_constraints,
    cognitive_constraints_payload,
    observed_adjacent_transition_report,
    occupancy_grid_version,
)
from robot_route_planner.cognitive_constraints_dump import (
    dump_fixed_scene_constraints,
)
from robot_route_planner.feasibility import classify_edge
from robot_route_planner.map_io import OccupancyMap, load_occupancy_map
from robot_route_planner.models import Traversability


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


def _jackal_footprint() -> dict:
    return {
        "polygon_m": [
            [0.255, 0.210],
            [0.255, -0.210],
            [-0.230, -0.210],
            [-0.230, 0.210],
        ],
        "padding_m": 0.005,
        "padded_inscribed_radius_m": 0.215,
        "sweep_sample_spacing_m": 0.025,
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


def test_kujiale_fixed_scene_override_uses_51_mask_and_physical_sweeps() -> None:
    repo = Path(__file__).resolve().parents[4]
    module2 = repo.parent / "bio_nav_module2"
    override = (
        module2 / "configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    )
    occupancy = load_occupancy_map(
        repo / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        unknown_is_occupied=True,
    )
    footprint = _jackal_footprint()

    default = build_cognitive_constraints(
        occupancy,
        map_version="live-map-version",
        graph_revision=1,
        footprint_settings=footprint,
    )
    fixed_scene = build_cognitive_constraints(
        occupancy,
        map_version="live-map-version",
        graph_revision=1,
        footprint_settings=footprint,
        fixed_scene_override_file=override,
    )

    assert int(default.reachable_state_mask.sum()) == 21
    assert len(default.verified_transitions) == 28
    assert default.cognitive_tile_id.startswith("canvas16:")
    for (source, target), path in zip(
        default.verified_transitions,
        default.transition_witnesses_map_xy,
        strict=True,
    ):
        source_row, source_column = divmod(int(source), 16)
        target_row, target_column = divmod(int(target), 16)
        assert np.allclose(
            path,
            (
                (source_column - 7.5, source_row - 7.5),
                (target_column - 7.5, target_row - 7.5),
            ),
        )
    assert int(fixed_scene.reachable_state_mask.sum()) == 51
    assert fixed_scene.cognitive_tile_id == "v6_kujiale_isaacgen_v1"
    assert np.array_equal(fixed_scene.t_map_canvas, np.eye(3))
    assert len(fixed_scene.verified_transitions) == 132
    assert fixed_scene.transition_witnesses_map_xy.shape == (132, 2, 2)
    adjacency = {
        int(state): set()
        for state in np.flatnonzero(fixed_scene.reachable_state_mask)
    }
    for (source, target), path in zip(
        fixed_scene.verified_transitions,
        fixed_scene.transition_witnesses_map_xy,
        strict=True,
    ):
        assert fixed_scene.reachable_state_mask[source]
        assert fixed_scene.reachable_state_mask[target]
        source_row, source_column = divmod(int(source), 16)
        target_row, target_column = divmod(int(target), 16)
        assert abs(source_row - target_row) + abs(source_column - target_column) == 1
        adjacency[int(source)].add(int(target))
        assert classify_edge(
            occupancy,
            path,
            footprint_polygon_m=np.asarray(footprint["polygon_m"]),
            footprint_padding_m=footprint["padding_m"],
            padded_inscribed_radius_m=footprint["padded_inscribed_radius_m"],
            sweep_sample_spacing_m=footprint["sweep_sample_spacing_m"],
        ) == Traversability.FEASIBLE
    reached = {40}
    queue = [40]
    while queue:
        state = queue.pop()
        for target in adjacency[state]:
            if target not in reached:
                reached.add(target)
                queue.append(target)
    assert reached == set(adjacency)
    assert {40, 200, 181, 117, 85}.issubset(reached)

    repeated = build_cognitive_constraints(
        occupancy,
        map_version="live-map-version",
        graph_revision=1,
        footprint_settings=footprint,
        fixed_scene_override_file=override,
    )
    assert np.array_equal(
        repeated.verified_transitions, fixed_scene.verified_transitions
    )
    assert np.array_equal(
        repeated.transition_witnesses_map_xy,
        fixed_scene.transition_witnesses_map_xy,
    )


def test_fixed_scene_payload_and_gt_report_do_not_add_transitions(
    tmp_path: Path,
) -> None:
    repo = Path(__file__).resolve().parents[4]
    module2 = repo.parent / "bio_nav_module2"
    enrollment = module2 / "configs/kujiale_0026_module1_enrollment_v310.yaml"
    output = tmp_path / "constraints.json"
    payload = dump_fixed_scene_constraints(
        map_yaml=repo / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        scene_config=enrollment,
        output=output,
        include_witnesses=True,
    )
    assert output.is_file()
    assert payload["map_id"] == "v6_kujiale_isaacgen_v1"
    assert len(payload["valid_state_ids"]) == 51
    assert len(payload["verified_transitions"]) == 132
    assert len(payload["transition_witnesses_map_xy"]) == 132

    occupancy = load_occupancy_map(
        repo / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        unknown_is_occupied=True,
    )
    value = build_cognitive_constraints(
        occupancy,
        map_version="map-version",
        graph_revision=1,
        footprint_settings=_jackal_footprint(),
        fixed_scene_override_file=enrollment,
    )
    transitions_before = value.verified_transitions.copy()
    report = observed_adjacent_transition_report(
        value,
        np.asarray((40, 56, 72, 88, 104, 104)),
        np.ones(6, dtype=bool),
    )
    assert report["observed_adjacent_occurrences"] == 4
    assert report["missing_adjacent_occurrences"] == 1
    assert report["missing_unique_transitions"] == [[88, 104]]
    assert np.array_equal(value.verified_transitions, transitions_before)

    direct_payload = cognitive_constraints_payload(
        value, include_witnesses=False
    )
    assert "transition_witnesses_map_xy" not in direct_payload


def test_fixed_scene_override_rejects_conflicting_explicit_tile_id() -> None:
    repo = Path(__file__).resolve().parents[4]
    occupancy = load_occupancy_map(
        repo / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
        unknown_is_occupied=True,
    )
    override = (
        repo.parent
        / "bio_nav_module2/configs/kujiale_0026_module1_visual_shadow_v310.yaml"
    )

    with pytest.raises(ValueError, match="canonical map_id"):
        build_cognitive_constraints(
            occupancy,
            map_version="occupancy-sha",
            graph_revision=1,
            footprint_settings=_jackal_footprint(),
            cognitive_tile_id="another-region",
            fixed_scene_override_file=override,
        )
