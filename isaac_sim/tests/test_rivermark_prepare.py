from __future__ import annotations

import numpy as np
import pytest

from isaac_sim.tools.rivermark_prepare import (
    _ground_slab_levels,
    _height_refined_occupancy,
    _parser,
    _seed_cell_is_free,
    _select_ground_relative_omap,
    _valid_ground_slab_seed,
    _write_map,
)


def _refine(
    height: np.ndarray,
    seed_xy: tuple[float, float],
    *,
    occupied_image: np.ndarray | None = None,
):
    center = (1.0, 1.0, 0.0)
    camera_z = center[2] + 150.0
    depth = camera_z - height
    raw = np.zeros(height.shape, dtype=np.float32)
    if occupied_image is not None:
        raw[occupied_image[::-1, :]] = 1.0
    return _height_refined_occupancy(
        raw,
        depth,
        center=center,
        seed_xy=seed_xy,
        window_m=2.0,
        resolution_m=0.1,
        max_step_m=0.06,
        max_slope=0.45,
        terrain_connect_step_m=0.25,
        terrain_connect_slope=2.5,
        barrier_thickness_cells=0,
    )


def test_ground_slab_levels_cover_the_eight_audited_anchors() -> None:
    assert _ground_slab_levels(5.734, 6.109, 0.05) == (
        5.75,
        5.8,
        5.85,
        5.9,
        5.95,
        6.0,
        6.05,
        6.1,
    )
    with pytest.raises(ValueError, match="positive spacing"):
        _ground_slab_levels(5.734, 6.109, 0.0)


def test_active_generator_defaults_match_runtime_scan_and_audited_ground() -> None:
    args = _parser().parse_args(["--output-dir", "/tmp/rivermark-test-output"])

    assert (args.scan_min_height_m, args.scan_max_height_m) == (0.05, 0.50)
    assert (args.ground_min_z_m, args.ground_max_z_m) == (5.734, 6.109)
    assert args.ground_slab_spacing_m == 0.05


def test_ground_relative_selection_uses_nearest_lower_tie_and_raw_y() -> None:
    low_raw = np.array([[0.0, 0.0], [1.0, 0.0]], dtype=np.float32)
    high_raw = np.array([[1.0, 1.0], [0.0, 1.0]], dtype=np.float32)
    height_image = np.array(
        [[5.75, 5.775], [5.80, 5.79]], dtype=np.float32
    )

    selected_raw, histogram = _select_ground_relative_omap(
        np.stack((low_raw, high_raw)),
        (5.75, 5.80),
        height_image,
        ground_min_z_m=5.734,
        ground_max_z_m=5.809,
    )
    selected_image = selected_raw[::-1, :]

    assert selected_image[0, 0] == 1.0  # low ground, low slab, flipped raw row
    assert selected_image[0, 1] == 0.0  # midpoint chooses the lower slab
    assert selected_image[1, 0] == 1.0
    assert selected_image[1, 1] == 1.0
    assert histogram == (2, 2)


def test_ground_relative_selection_leaves_invalid_and_out_of_range_unknown() -> None:
    slabs = np.zeros((2, 2, 2), dtype=np.float32)
    height_image = np.array(
        [[np.nan, 5.74], [5.81, np.inf]], dtype=np.float32
    )

    selected_raw, histogram = _select_ground_relative_omap(
        slabs,
        (5.75, 5.80),
        height_image,
        ground_min_z_m=5.75,
        ground_max_z_m=5.80,
    )

    assert np.all(selected_raw == 0.5)
    assert histogram == (0, 0)


def test_ground_range_edges_clamp_to_first_and_last_anchor() -> None:
    low_raw = np.zeros((1, 2), dtype=np.float32)
    high_raw = np.ones((1, 2), dtype=np.float32)
    height_image = np.array([[5.734, 6.109]], dtype=np.float32)

    selected_raw, histogram = _select_ground_relative_omap(
        np.stack((low_raw, high_raw)),
        (5.75, 6.10),
        height_image,
        ground_min_z_m=5.734,
        ground_max_z_m=6.109,
    )

    assert selected_raw.tolist() == [[0.0, 1.0]]
    assert histogram == (1, 1)


def test_invalid_or_colliding_slab_seed_is_rejected() -> None:
    assert _valid_ground_slab_seed(6.10, 6.05, 0.05, 0.50)
    assert _valid_ground_slab_seed(6.55, 6.05, 0.05, 0.50)
    assert not _valid_ground_slab_seed(6.09, 6.05, 0.05, 0.50)
    assert not _valid_ground_slab_seed(6.56, 6.05, 0.05, 0.50)
    assert not _valid_ground_slab_seed(6.10, np.nan, 0.05, 0.50)

    raw = np.zeros((2, 2), dtype=np.float32)
    assert _seed_cell_is_free(
        raw,
        seed_xy=(0.5, 0.5),
        center_xy=(0.5, 0.5),
        window_m=1.0,
        resolution_m=0.5,
    )
    raw[1, 1] = 1.0
    assert not _seed_cell_is_free(
        raw,
        seed_xy=(0.5, 0.5),
        center_xy=(0.5, 0.5),
        window_m=1.0,
        resolution_m=0.5,
    )


def test_curb_remains_blocked_when_sidewalk_is_reachable_through_ramp() -> None:
    height = np.zeros((20, 20), dtype=np.float32)
    height[:, 10:] = 0.10
    # A four-cell ramp connects both surfaces near the image bottom.
    for column, value in zip(range(8, 13), np.linspace(0.0, 0.10, 5)):
        height[15:, column] = value

    occupied = np.zeros(height.shape, dtype=bool)
    occupied[:, 10] = True
    refined_raw, diagnostic, _, reachable, steep, _ = _refine(
        height, seed_xy=(0.4, 1.0), occupied_image=occupied
    )
    refined_image = refined_raw[::-1, :]

    assert reachable[5, 15]
    assert steep[5, 10]
    assert not steep[17, 10]
    assert refined_image[5, 10] == 1.0
    assert refined_image[5, 15] == 0.0
    assert diagnostic["steep_boundary_cells"] > 0


def test_low_raised_island_has_blocked_curb_but_no_false_solid_obstacle() -> None:
    height = np.zeros((20, 20), dtype=np.float32)
    height[7:13, 7:13] = 0.10

    occupied = np.zeros(height.shape, dtype=bool)
    occupied[7:13, 7:13] = True
    refined_raw, _, _, reachable, _, nontraversable = _refine(
        height, seed_xy=(0.4, 1.0), occupied_image=occupied
    )
    refined_image = refined_raw[::-1, :]

    assert reachable[10, 10]
    assert not nontraversable[10, 10]
    assert refined_image[7, 10] == 1.0
    assert refined_image[10, 10] == 0.0
    assert refined_image[2, 2] == 0.0


def test_steep_boundary_is_retained_only_when_selected_slab_is_occupied() -> None:
    height = np.zeros((20, 20), dtype=np.float32)
    height[:, 10:] = 0.10

    refined_raw, diagnostic, _, _, steep, _ = _refine(
        height, seed_xy=(0.4, 1.0)
    )
    refined_image = refined_raw[::-1, :]

    assert steep[5, 10]
    assert refined_image[5, 10] != 1.0
    assert diagnostic["height_barrier_cells"] == 0


def test_write_map_is_byte_deterministic(tmp_path) -> None:
    raw = np.array([[0.0, 0.5], [1.0, 0.0]], dtype=np.float32)
    first = tmp_path / "first"
    second = tmp_path / "second"

    _write_map(first, "A", raw, resolution_m=0.05, origin_xy=(-52.0182, 111.603))
    _write_map(second, "A", raw, resolution_m=0.05, origin_xy=(-52.0182, 111.603))

    assert (first / "candidate_A_occupancy.pgm").read_bytes() == (
        second / "candidate_A_occupancy.pgm"
    ).read_bytes()
    assert (first / "candidate_A_occupancy.yaml").read_bytes() == (
        second / "candidate_A_occupancy.yaml"
    ).read_bytes()
