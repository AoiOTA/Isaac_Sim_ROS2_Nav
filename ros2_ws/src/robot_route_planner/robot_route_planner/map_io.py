"""ROS map-server compatible occupancy map loading and coordinates."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass(frozen=True)
class OccupancyMap:
    free: np.ndarray
    resolution_m: float
    origin_xy_m: tuple[float, float]
    map_version: str
    yaml_path: Path

    def pixel_to_world(self, row: float, column: float) -> tuple[float, float]:
        height = int(self.free.shape[0])
        return (
            self.origin_xy_m[0] + (float(column) + 0.5) * self.resolution_m,
            self.origin_xy_m[1] + (height - float(row) - 0.5) * self.resolution_m,
        )

    def world_to_pixel(self, x_m: float, y_m: float) -> tuple[int, int]:
        height = int(self.free.shape[0])
        column = int(np.floor((float(x_m) - self.origin_xy_m[0]) / self.resolution_m))
        row_from_bottom = int(np.floor((float(y_m) - self.origin_xy_m[1]) / self.resolution_m))
        return height - row_from_bottom - 1, column


def load_occupancy_map(path: str | Path, *, unknown_is_occupied: bool) -> OccupancyMap:
    yaml_path = Path(path).expanduser().resolve()
    metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = Path(str(metadata["image"]))
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(image_path)
    normalized = image.astype(np.float64) / 255.0
    occupancy = normalized if int(metadata.get("negate", 0)) else 1.0 - normalized
    free = occupancy < float(metadata["free_thresh"])
    occupied = occupancy > float(metadata["occupied_thresh"])
    if not unknown_is_occupied:
        free = free | (~free & ~occupied)
    origin = metadata["origin"]
    return OccupancyMap(
        free=np.asarray(free, dtype=bool),
        resolution_m=float(metadata["resolution"]),
        origin_xy_m=(float(origin[0]), float(origin[1])),
        map_version=yaml_path.stem,
        yaml_path=yaml_path,
    )


__all__ = ["OccupancyMap", "load_occupancy_map"]
