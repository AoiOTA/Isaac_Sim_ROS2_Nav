from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from robot_route_planner.regions import (
    RegionSelector,
    load_region_config,
    rectangular_region_config,
)


ROOT = Path(__file__).resolve().parents[4]
RIVERMARK_DATA = ROOT / "data/rivermark_demo"


def test_map_to_canvas_transform_centers_region_and_rotates_axes() -> None:
    config = rectangular_region_config(
        scene_id="rivermark",
        map_frame="map",
        bounds_xy=(10.0, 20.0, 22.0, 32.0),
        stride_m=12.0,
        yaw_deg=90.0,
    )
    region = config.regions[0]
    center = np.asarray((*region.center_map_xy, 1.0))
    assert np.allclose(region.t_map_canvas @ center, (0.0, 0.0, 1.0))
    assert np.allclose(
        region.t_map_canvas @ np.asarray((region.center_map_xy[0], region.center_map_xy[1] + 1.0, 1.0)),
        (1.0, 0.0, 1.0),
        atol=1.0e-9,
    )


def test_region_selector_switches_core_without_losing_global_coordinates() -> None:
    config = rectangular_region_config(
        scene_id="rivermark",
        map_frame="map",
        bounds_xy=(0.0, 0.0, 24.0, 12.0),
        stride_m=12.0,
    )
    selector = RegionSelector(config, min_dwell_s=0.5)
    first = selector.select((3.0, 6.0), 0.0)
    assert first.region_id.endswith("r00c00")
    assert selector.select((18.0, 6.0), 0.2) == first
    second = selector.select((18.0, 6.0), 0.6)
    assert second.region_id.endswith("r00c01")


def test_rivermark_catalog_region_ids_are_a_subset_of_runtime_regions() -> None:
    config = load_region_config(RIVERMARK_DATA / "rivermark_regions.yaml")
    selection = yaml.safe_load(
        (RIVERMARK_DATA / "rivermark_srdr_tile_selection_v1.yaml").read_text(
            encoding="utf-8"
        )
    )
    runtime_ids = {region.region_id for region in config.regions}
    catalog_ids = set(selection["route_region_ids"]) | set(
        selection["ring_region_ids"]
    )

    assert catalog_ids
    assert catalog_ids <= runtime_ids
