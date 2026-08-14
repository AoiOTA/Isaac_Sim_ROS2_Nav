from __future__ import annotations

import numpy as np

from isaac_sim.tools.rivermark_prepare import _height_refined_occupancy


def _refine(height: np.ndarray, seed_xy: tuple[float, float]):
    center = (1.0, 1.0, 0.0)
    camera_z = center[2] + 150.0
    depth = camera_z - height
    raw_free = np.zeros(height.shape, dtype=np.float32)
    return _height_refined_occupancy(
        raw_free,
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


def test_curb_remains_blocked_when_sidewalk_is_reachable_through_ramp() -> None:
    height = np.zeros((20, 20), dtype=np.float32)
    height[:, 10:] = 0.10
    # A four-cell ramp connects both surfaces near the image bottom.
    for column, value in zip(range(8, 13), np.linspace(0.0, 0.10, 5)):
        height[15:, column] = value

    refined_raw, diagnostic, _, reachable, steep, _ = _refine(
        height, seed_xy=(0.4, 1.0)
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

    refined_raw, _, _, reachable, _, nontraversable = _refine(
        height, seed_xy=(0.4, 1.0)
    )
    refined_image = refined_raw[::-1, :]

    assert reachable[10, 10]
    assert not nontraversable[10, 10]
    assert refined_image[7, 10] == 1.0
    assert refined_image[10, 10] == 0.0
    assert refined_image[2, 2] == 0.0
