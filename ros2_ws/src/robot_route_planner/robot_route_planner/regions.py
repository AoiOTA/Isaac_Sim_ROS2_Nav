"""Logical outdoor cognitive regions over one continuous global map."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class CognitiveRegion:
    region_id: str
    center_map_xy: tuple[float, float]
    yaw_deg: float
    core_polygon_map: tuple[tuple[float, float], ...]
    priority: int = 0

    @property
    def t_map_canvas(self) -> np.ndarray:
        """Return the homogeneous map-to-canvas transform."""

        yaw = math.radians(float(self.yaw_deg))
        cosine, sine = math.cos(yaw), math.sin(yaw)
        center_x, center_y = self.center_map_xy
        return np.asarray(
            [
                [cosine, sine, -(cosine * center_x + sine * center_y)],
                [-sine, cosine, sine * center_x - cosine * center_y],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

    def contains(self, xy: tuple[float, float]) -> bool:
        point_x, point_y = (float(xy[0]), float(xy[1]))
        inside = False
        previous = self.core_polygon_map[-1]
        for current in self.core_polygon_map:
            crosses = (current[1] > point_y) != (previous[1] > point_y)
            if crosses:
                x_crossing = (
                    (previous[0] - current[0])
                    * (point_y - current[1])
                    / (previous[1] - current[1])
                    + current[0]
                )
                inside ^= point_x < x_crossing
            previous = current
        return inside


@dataclass(frozen=True)
class RegionConfig:
    scene_id: str
    map_frame: str
    regions: tuple[CognitiveRegion, ...]


def load_region_config(path: str | Path) -> RegionConfig:
    source = Path(path).expanduser().resolve()
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("region config must be a mapping")
    regions = []
    identifiers: set[str] = set()
    for row in document.get("regions", []):
        identifier = str(row["id"]).strip()
        center = tuple(float(value) for value in row["center_map_xy"])
        polygon = tuple(
            tuple(float(value) for value in point)
            for point in row["core_polygon_map"]
        )
        if not identifier or identifier in identifiers:
            raise ValueError("region IDs must be non-empty and unique")
        if len(center) != 2 or len(polygon) < 3:
            raise ValueError(f"invalid geometry for region {identifier}")
        values = np.asarray((center, *polygon), dtype=np.float64)
        if not np.isfinite(values).all():
            raise ValueError(f"non-finite geometry for region {identifier}")
        identifiers.add(identifier)
        regions.append(
            CognitiveRegion(
                region_id=identifier,
                center_map_xy=center,
                yaw_deg=float(row.get("yaw_deg", 0.0)),
                core_polygon_map=polygon,
                priority=int(row.get("priority", 0)),
            )
        )
    if not regions:
        raise ValueError("region config requires at least one region")
    return RegionConfig(
        scene_id=str(document.get("scene_id", source.stem)),
        map_frame=str(document.get("map_frame", "map")),
        regions=tuple(regions),
    )


class RegionSelector:
    """Select a stable region from core polygons with a small dwell time."""

    def __init__(self, config: RegionConfig, *, min_dwell_s: float = 0.5) -> None:
        self.config = config
        self.min_dwell_s = max(0.0, float(min_dwell_s))
        self.current: CognitiveRegion | None = None
        self.last_switch_s = -math.inf

    def select(self, xy: tuple[float, float], now_s: float) -> CognitiveRegion:
        if self.current is not None and self.current.contains(xy):
            return self.current
        candidates = [region for region in self.config.regions if region.contains(xy)]
        if candidates:
            selected = min(candidates, key=lambda region: (-region.priority, region.region_id))
        else:
            selected = min(
                self.config.regions,
                key=lambda region: (
                    math.dist(xy, region.center_map_xy),
                    -region.priority,
                    region.region_id,
                ),
            )
        if (
            self.current is not None
            and selected != self.current
            and float(now_s) - self.last_switch_s < self.min_dwell_s
        ):
            return self.current
        if selected != self.current:
            self.current = selected
            self.last_switch_s = float(now_s)
        return selected


def rectangular_region_config(
    *,
    scene_id: str,
    map_frame: str,
    bounds_xy: tuple[float, float, float, float],
    stride_m: float = 12.0,
    yaw_deg: float = 0.0,
) -> RegionConfig:
    """Create non-overlapping cores whose 16 m canvases overlap by default."""

    minimum_x, minimum_y, maximum_x, maximum_y = map(float, bounds_xy)
    if maximum_x <= minimum_x or maximum_y <= minimum_y or stride_m <= 0.0:
        raise ValueError("invalid region bounds or stride")
    count_x = max(1, int(math.ceil((maximum_x - minimum_x) / stride_m)))
    count_y = max(1, int(math.ceil((maximum_y - minimum_y) / stride_m)))
    cell_x = (maximum_x - minimum_x) / count_x
    cell_y = (maximum_y - minimum_y) / count_y
    regions = []
    for row in range(count_y):
        y0, y1 = minimum_y + row * cell_y, minimum_y + (row + 1) * cell_y
        for column in range(count_x):
            x0, x1 = minimum_x + column * cell_x, minimum_x + (column + 1) * cell_x
            regions.append(
                CognitiveRegion(
                    region_id=f"{scene_id}:r{row:02d}c{column:02d}",
                    center_map_xy=(0.5 * (x0 + x1), 0.5 * (y0 + y1)),
                    yaw_deg=float(yaw_deg),
                    core_polygon_map=((x0, y0), (x1, y0), (x1, y1), (x0, y1)),
                )
            )
    return RegionConfig(scene_id=scene_id, map_frame=map_frame, regions=tuple(regions))


__all__ = [
    "CognitiveRegion",
    "RegionConfig",
    "RegionSelector",
    "load_region_config",
    "rectangular_region_config",
]
