from __future__ import annotations

from pathlib import Path

import numpy as np

from robot_route_planner.cognitive_pair import (
    _canvas_points_to_map,
    persistent_local_change,
)
from robot_route_planner.map_io import OccupancyMap


def test_persistent_local_change_is_copy_on_write_and_local() -> None:
    original = OccupancyMap(
        free=np.ones((40, 40), dtype=bool),
        resolution_m=0.05,
        origin_xy_m=(-1.0, -1.0),
        map_version="a",
        yaml_path=Path("map.yaml"),
    )
    changed = persistent_local_change(original, (0.0, 0.0), radius_m=0.20)
    assert original.free.all()
    assert not changed.free.all()
    assert changed.free[0, 0]
    assert np.count_nonzero(~changed.free) < changed.free.size // 4


def test_canvas_centers_are_transformed_back_into_nonzero_map_region() -> None:
    t_map_canvas = np.asarray(
        ((1.0, 0.0, 30.0), (0.0, 1.0, -120.0), (0.0, 0.0, 1.0))
    )
    mapped = _canvas_points_to_map(
        np.asarray(((0.0, 0.0), (7.5, -7.5))), t_map_canvas
    )
    assert np.allclose(mapped[0], (-30.0, 120.0))
    assert np.allclose(mapped[1], (-22.5, 112.5))
