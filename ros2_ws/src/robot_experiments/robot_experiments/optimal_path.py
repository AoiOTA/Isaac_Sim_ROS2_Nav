"""OccupancyGrid shortest-path reference used by navigation benchmarks."""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from pathlib import Path
from typing import Sequence

import numpy as np
import yaml


class OptimalPathError(ValueError):
    """Raised when a map or requested reference path is invalid."""


@dataclass(frozen=True)
class OccupancyGridReference:
    resolution: float
    origin_x: float
    origin_y: float
    blocked: np.ndarray

    @property
    def height(self) -> int:
        return int(self.blocked.shape[0])

    @property
    def width(self) -> int:
        return int(self.blocked.shape[1])

    def world_to_cell(
        self,
        point: Sequence[float],
    ) -> tuple[int, int]:
        if len(point) < 2:
            raise OptimalPathError("world point requires x and y")
        x = float(point[0])
        y = float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise OptimalPathError("world point must be finite")
        column = math.floor((x - self.origin_x) / self.resolution)
        row_from_bottom = math.floor((y - self.origin_y) / self.resolution)
        row = self.height - 1 - row_from_bottom
        if not 0 <= row < self.height or not 0 <= column < self.width:
            raise OptimalPathError(
                f"world point {(x, y)} is outside the OccupancyGrid"
            )
        return int(row), int(column)

    def shortest_path_length(
        self,
        start: Sequence[float],
        goal: Sequence[float],
    ) -> float:
        start_cell = self.world_to_cell(start)
        goal_cell = self.world_to_cell(goal)
        if bool(self.blocked[start_cell]):
            raise OptimalPathError("reference start is blocked after inflation")
        if bool(self.blocked[goal_cell]):
            raise OptimalPathError("reference goal is blocked after inflation")

        distances = np.full(self.blocked.shape, np.inf, dtype=np.float64)
        closed = np.zeros(self.blocked.shape, dtype=np.bool_)
        distances[start_cell] = 0.0
        queue: list[tuple[float, float, int, int]] = [
            (
                math.dist(start_cell, goal_cell),
                0.0,
                start_cell[0],
                start_cell[1],
            )
        ]
        diagonal = math.sqrt(2.0)
        neighbors = (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, diagonal),
            (-1, 1, diagonal),
            (1, -1, diagonal),
            (1, 1, diagonal),
        )

        while queue:
            _, distance, row, column = heapq.heappop(queue)
            if closed[row, column]:
                continue
            if distance != distances[row, column]:
                continue
            if (row, column) == goal_cell:
                return distance * self.resolution
            closed[row, column] = True

            for row_delta, column_delta, step_cost in neighbors:
                next_row = row + row_delta
                next_column = column + column_delta
                if (
                    not 0 <= next_row < self.height
                    or not 0 <= next_column < self.width
                    or closed[next_row, next_column]
                    or self.blocked[next_row, next_column]
                ):
                    continue
                if (
                    row_delta
                    and column_delta
                    and (
                        self.blocked[row + row_delta, column]
                        or self.blocked[row, column + column_delta]
                    )
                ):
                    continue
                candidate = distance + step_cost
                if candidate >= distances[next_row, next_column]:
                    continue
                distances[next_row, next_column] = candidate
                heuristic = math.hypot(
                    next_row - goal_cell[0],
                    next_column - goal_cell[1],
                )
                heapq.heappush(
                    queue,
                    (
                        candidate + heuristic,
                        candidate,
                        next_row,
                        next_column,
                    ),
                )
        raise OptimalPathError("no inflated OccupancyGrid path connects start and goal")


def _read_pgm(path: Path) -> tuple[np.ndarray, int]:
    with path.open("rb") as stream:
        magic = stream.readline().strip()
        if magic not in {b"P2", b"P5"}:
            raise OptimalPathError(f"unsupported PGM encoding {magic!r}")

        header_tokens: list[bytes] = []
        while len(header_tokens) < 3:
            line = stream.readline()
            if not line:
                raise OptimalPathError("truncated PGM header")
            content = line.split(b"#", 1)[0]
            header_tokens.extend(content.split())
        width, height, maximum = (
            int(header_tokens[0]),
            int(header_tokens[1]),
            int(header_tokens[2]),
        )
        if width <= 0 or height <= 0 or not 0 < maximum <= 65535:
            raise OptimalPathError("invalid PGM dimensions or maximum")

        if magic == b"P5":
            dtype = np.uint8 if maximum < 256 else np.dtype(">u2")
            pixels = np.frombuffer(
                stream.read(),
                dtype=dtype,
                count=width * height,
            )
        else:
            remaining = stream.read().split()
            pixels = np.asarray(
                [int(token) for token in remaining[: width * height]],
                dtype=np.uint16,
            )
        if pixels.size != width * height:
            raise OptimalPathError("truncated PGM pixel data")
        return pixels.reshape((height, width)), maximum


def _inflate(blocked: np.ndarray, resolution: float, clearance: float) -> np.ndarray:
    if clearance <= 0.0:
        return blocked.copy()
    radius_cells = math.ceil(clearance / resolution)
    result = blocked.copy()
    for row_delta in range(-radius_cells, radius_cells + 1):
        for column_delta in range(-radius_cells, radius_cells + 1):
            if (
                math.hypot(row_delta, column_delta) * resolution
                > clearance
            ):
                continue
            target_rows = slice(
                max(0, row_delta),
                min(blocked.shape[0], blocked.shape[0] + row_delta),
            )
            source_rows = slice(
                max(0, -row_delta),
                min(blocked.shape[0], blocked.shape[0] - row_delta),
            )
            target_columns = slice(
                max(0, column_delta),
                min(blocked.shape[1], blocked.shape[1] + column_delta),
            )
            source_columns = slice(
                max(0, -column_delta),
                min(blocked.shape[1], blocked.shape[1] - column_delta),
            )
            result[target_rows, target_columns] |= blocked[
                source_rows,
                source_columns,
            ]
    return result


def load_occupancy_grid_reference(
    map_yaml: str | Path,
    *,
    clearance_m: float,
    allow_unknown: bool = False,
) -> OccupancyGridReference:
    source = Path(map_yaml).expanduser().resolve()
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise OptimalPathError("map YAML must be a mapping")
    resolution = float(document.get("resolution", 0.0))
    origin = document.get("origin")
    if (
        not math.isfinite(resolution)
        or resolution <= 0.0
        or not isinstance(origin, list)
        or len(origin) != 3
    ):
        raise OptimalPathError("map resolution/origin is invalid")
    origin_values = tuple(float(value) for value in origin)
    if not all(math.isfinite(value) for value in origin_values):
        raise OptimalPathError("map origin must be finite")
    if abs(origin_values[2]) > 1.0e-12:
        raise OptimalPathError("rotated OccupancyGrid origins are unsupported")
    clearance = float(clearance_m)
    if not math.isfinite(clearance) or clearance < 0.0:
        raise OptimalPathError("clearance_m must be finite and non-negative")

    image_value = document.get("image")
    if not isinstance(image_value, str) or not image_value:
        raise OptimalPathError("map image path is required")
    image_path = (source.parent / image_value).resolve()
    pixels, maximum = _read_pgm(image_path)
    negate = int(document.get("negate", 0))
    if negate not in {0, 1}:
        raise OptimalPathError("map negate must be 0 or 1")
    occupied_threshold = float(document.get("occupied_thresh", 0.65))
    free_threshold = float(document.get("free_thresh", 0.196))
    if not (
        0.0 <= free_threshold < occupied_threshold <= 1.0
        and math.isfinite(occupied_threshold)
        and math.isfinite(free_threshold)
    ):
        raise OptimalPathError("map occupancy thresholds are invalid")

    scaled = pixels.astype(np.float64) / float(maximum)
    probability = scaled if negate else 1.0 - scaled
    occupied = probability >= occupied_threshold
    free = probability <= free_threshold
    traversable = ~occupied if allow_unknown else free
    blocked = _inflate(~traversable, resolution, clearance)
    return OccupancyGridReference(
        resolution=resolution,
        origin_x=origin_values[0],
        origin_y=origin_values[1],
        blocked=blocked,
    )
