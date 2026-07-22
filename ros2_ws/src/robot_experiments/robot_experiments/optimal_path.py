"""OccupancyGrid shortest-path reference used by navigation benchmarks."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
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

    def refined(self, factor: int = 2) -> "OccupancyGridReference":
        """Conservatively subdivide cells for the documented 0.025 m check."""
        if not isinstance(factor, int) or factor < 1:
            raise OptimalPathError("refinement factor must be a positive integer")
        if factor == 1:
            return self
        return OccupancyGridReference(
            resolution=self.resolution / factor,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            blocked=np.repeat(np.repeat(self.blocked, factor, axis=0), factor, axis=1),
        )


def _point_in_polygon(point: tuple[float, float], polygon: Sequence[tuple[float, float]]) -> bool:
    """Inclusive ray-cast test used only for conservative cell intersection."""
    x, y = point
    inside = False
    for (x0, y0), (x1, y1) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y0 > y) == (y1 > y):
            continue
        crossing = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
        if x <= crossing:
            inside = not inside
    return inside


def _orient(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(
    a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]
) -> bool:
    """Closed line-segment intersection, including collinear contact."""
    epsilon = 1.0e-12
    values = (_orient(a, b, c), _orient(a, b, d), _orient(c, d, a), _orient(c, d, b))
    if all(abs(value) <= epsilon for value in values):
        return not (
            max(a[0], b[0]) < min(c[0], d[0]) - epsilon
            or max(c[0], d[0]) < min(a[0], b[0]) - epsilon
            or max(a[1], b[1]) < min(c[1], d[1]) - epsilon
            or max(c[1], d[1]) < min(a[1], b[1]) - epsilon
        )
    return (values[0] * values[1] <= epsilon) and (values[2] * values[3] <= epsilon)


def _polygon_intersects_cell(
    polygon: Sequence[tuple[float, float]], *, x0: float, y0: float, resolution: float
) -> bool:
    """Exact convex-footprint vs occupied-square contact check."""
    corners = ((x0, y0), (x0 + resolution, y0), (x0 + resolution, y0 + resolution), (x0, y0 + resolution))
    if any(_point_in_polygon(corner, polygon) for corner in corners):
        return True
    if any(x0 <= x <= x0 + resolution and y0 <= y <= y0 + resolution for x, y in polygon):
        return True
    cell_edges = tuple(zip(corners, corners[1:] + corners[:1]))
    footprint_edges = tuple(zip(polygon, polygon[1:] + polygon[:1]))
    return any(_segments_intersect(a, b, c, d) for a, b in footprint_edges for c, d in cell_edges)


@dataclass(frozen=True)
class FootprintStateLatticeReference:
    """SE(2) A* reference using the real Jackal polygon, not a circumscribed circle.

    States are map cells plus heading bins.  Rotation has zero reported path
    length; translation uses swept footprint checks at half-cell intervals.
    This keeps the benchmark's length definition aligned with the plan while
    rejecting poses whose complete polygon intersects occupied or unknown map
    cells.
    """

    grid: OccupancyGridReference
    footprint: tuple[tuple[float, float], ...]
    heading_bins: int = 16

    def __post_init__(self) -> None:
        if self.heading_bins < 8 or self.heading_bins % 4:
            raise OptimalPathError("heading_bins must be a multiple of four and at least eight")
        if len(self.footprint) < 3:
            raise OptimalPathError("footprint requires at least three vertices")

    def _cell_center(self, row: int, column: int) -> tuple[float, float]:
        return (
            self.grid.origin_x + (column + 0.5) * self.grid.resolution,
            self.grid.origin_y + (self.grid.height - row - 0.5) * self.grid.resolution,
        )

    def _heading(self, index: int) -> float:
        return 2.0 * math.pi * (index % self.heading_bins) / self.heading_bins

    def _rotated_footprint(self, row: int, column: int, heading: int) -> tuple[tuple[float, float], ...]:
        x, y = self._cell_center(row, column)
        angle = self._heading(heading)
        cosine, sine = math.cos(angle), math.sin(angle)
        return tuple((x + cosine * px - sine * py, y + sine * px + cosine * py) for px, py in self.footprint)

    def _pose_is_clear(self, row: int, column: int, heading: int) -> bool:
        if not (0 <= row < self.grid.height and 0 <= column < self.grid.width):
            return False
        polygon = self._rotated_footprint(row, column, heading)
        min_x = min(point[0] for point in polygon)
        max_x = max(point[0] for point in polygon)
        min_y = min(point[1] for point in polygon)
        max_y = max(point[1] for point in polygon)
        min_column = math.floor((min_x - self.grid.origin_x) / self.grid.resolution)
        max_column = math.floor((max_x - self.grid.origin_x) / self.grid.resolution)
        min_row = self.grid.height - 1 - math.floor((max_y - self.grid.origin_y) / self.grid.resolution)
        max_row = self.grid.height - 1 - math.floor((min_y - self.grid.origin_y) / self.grid.resolution)
        if min_row < 0 or min_column < 0 or max_row >= self.grid.height or max_column >= self.grid.width:
            return False
        for blocked_row in range(min_row, max_row + 1):
            for blocked_column in range(min_column, max_column + 1):
                if not self.grid.blocked[blocked_row, blocked_column]:
                    continue
                x0 = self.grid.origin_x + blocked_column * self.grid.resolution
                y0 = self.grid.origin_y + (self.grid.height - blocked_row - 1) * self.grid.resolution
                if _polygon_intersects_cell(polygon, x0=x0, y0=y0, resolution=self.grid.resolution):
                    return False
        return True

    def shortest_path_length(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        *,
        goal_position_tolerance_m: float = 0.0,
        goal_orientation_tolerance_rad: float = 0.0,
    ) -> float:
        if len(start) < 2 or len(goal) < 2:
            raise OptimalPathError("state-lattice start and goal require x/y")
        start_cell, goal_cell = self.grid.world_to_cell(start), self.grid.world_to_cell(goal)
        start_yaw = float(start[2]) if len(start) >= 3 else 0.0
        goal_yaw = float(goal[2]) if len(goal) >= 3 else start_yaw
        if not (math.isfinite(start_yaw) and math.isfinite(goal_yaw)):
            raise OptimalPathError("state-lattice yaw must be finite")
        if (
            not math.isfinite(goal_position_tolerance_m)
            or not math.isfinite(goal_orientation_tolerance_rad)
            or goal_position_tolerance_m < 0.0
            or goal_orientation_tolerance_rad < 0.0
        ):
            raise OptimalPathError("goal tolerances must be finite and non-negative")
        to_heading = lambda yaw: int(round((yaw % (2.0 * math.pi)) * self.heading_bins / (2.0 * math.pi))) % self.heading_bins
        start_heading, goal_heading = to_heading(start_yaw), to_heading(goal_yaw)

        cached_clear = lru_cache(maxsize=None)(self._pose_is_clear)
        if not cached_clear(*start_cell, start_heading):
            raise OptimalPathError("reference start is blocked by the footprint")
        if not cached_clear(*goal_cell, goal_heading) and goal_position_tolerance_m <= 0.0:
            raise OptimalPathError("reference goal is blocked by the footprint")

        start_state = (*start_cell, start_heading)
        goal_state = (*goal_cell, goal_heading)
        goal_x, goal_y = float(goal[0]), float(goal[1])

        def reaches_goal(state: tuple[int, int, int]) -> bool:
            x, y = self._cell_center(state[0], state[1])
            if math.hypot(x - goal_x, y - goal_y) > goal_position_tolerance_m + 1.0e-12:
                return False
            heading_error = abs((self._heading(state[2]) - goal_yaw + math.pi) % (2.0 * math.pi) - math.pi)
            # A heading bin represents an interval, so include its quantization
            # half-width in the acceptance comparison.
            return heading_error <= goal_orientation_tolerance_rad + math.pi / self.heading_bins + 1.0e-12
        distances: dict[tuple[int, int, int], float] = {start_state: 0.0}
        serial = 0
        queue: list[tuple[float, int, float, tuple[int, int, int]]] = [(0.0, serial, 0.0, start_state)]
        # One-cell primitives preserve reachability for every map-cell parity;
        # the two-cell primitive supplies the intermediate 16-heading slopes
        # (for example 2:1 at 26.6 degrees) needed by narrow angled doors.
        while queue:
            _, _, distance, state = heapq.heappop(queue)
            if distance != distances.get(state):
                continue
            if state == goal_state if goal_position_tolerance_m <= 0.0 and goal_orientation_tolerance_rad <= 0.0 else reaches_goal(state):
                return distance
            row, column, heading = state
            neighbours: list[tuple[tuple[int, int, int], float]] = [
                ((row, column, (heading - 1) % self.heading_bins), 0.0),
                ((row, column, (heading + 1) % self.heading_bins), 0.0),
            ]
            angle = self._heading(heading)
            for scale in (1.0, 2.0):
                delta_column = int(round(scale * math.cos(angle)))
                delta_row = -int(round(scale * math.sin(angle)))
                if not (delta_row or delta_column):
                    continue
                candidate = (row + delta_row, column + delta_column, heading)
                step = math.hypot(delta_row, delta_column) * self.grid.resolution
                # Check the complete swept primitive, including intermediate poses.
                sweep_steps = max(abs(delta_row), abs(delta_column)) * 2
                clear = True
                for index in range(1, sweep_steps + 1):
                    sample_row = int(round(row + delta_row * index / sweep_steps))
                    sample_column = int(round(column + delta_column * index / sweep_steps))
                    if not cached_clear(sample_row, sample_column, heading):
                        clear = False
                        break
                if clear:
                    neighbours.append((candidate, step))
            for candidate, step in neighbours:
                if not cached_clear(*candidate):
                    continue
                candidate_distance = distance + step
                if candidate_distance >= distances.get(candidate, math.inf):
                    continue
                distances[candidate] = candidate_distance
                serial += 1
                candidate_x, candidate_y = self._cell_center(candidate[0], candidate[1])
                heuristic = max(0.0, math.hypot(candidate_x - goal_x, candidate_y - goal_y) - goal_position_tolerance_m)
                heapq.heappush(queue, (candidate_distance + heuristic, serial, candidate_distance, candidate))
        raise OptimalPathError("no footprint-aware SE(2) path connects start and goal")

    def refined(self, factor: int = 2) -> "FootprintStateLatticeReference":
        return FootprintStateLatticeReference(
            grid=self.grid.refined(factor),
            footprint=self.footprint,
            heading_bins=self.heading_bins,
        )

    def with_polygon_obstacles(
        self, polygons: Sequence[Sequence[Sequence[float]]]
    ) -> "FootprintStateLatticeReference":
        """Add physical obstacle footprints as occupied map cells conservatively."""
        blocked = self.grid.blocked.copy()
        for raw_polygon in polygons:
            polygon = tuple((float(point[0]), float(point[1])) for point in raw_polygon)
            if len(polygon) < 3 or not all(math.isfinite(value) for point in polygon for value in point):
                raise OptimalPathError("physical obstacle polygon is invalid")
            min_x, max_x = min(point[0] for point in polygon), max(point[0] for point in polygon)
            min_y, max_y = min(point[1] for point in polygon), max(point[1] for point in polygon)
            min_column = max(0, math.floor((min_x - self.grid.origin_x) / self.grid.resolution))
            max_column = min(self.grid.width - 1, math.floor((max_x - self.grid.origin_x) / self.grid.resolution))
            min_row = max(0, self.grid.height - 1 - math.floor((max_y - self.grid.origin_y) / self.grid.resolution))
            max_row = min(self.grid.height - 1, self.grid.height - 1 - math.floor((min_y - self.grid.origin_y) / self.grid.resolution))
            for row in range(min_row, max_row + 1):
                for column in range(min_column, max_column + 1):
                    x0 = self.grid.origin_x + column * self.grid.resolution
                    y0 = self.grid.origin_y + (self.grid.height - row - 1) * self.grid.resolution
                    if _polygon_intersects_cell(polygon, x0=x0, y0=y0, resolution=self.grid.resolution):
                        blocked[row, column] = True
        return FootprintStateLatticeReference(
            grid=OccupancyGridReference(
                resolution=self.grid.resolution,
                origin_x=self.grid.origin_x,
                origin_y=self.grid.origin_y,
                blocked=blocked,
            ),
            footprint=self.footprint,
            heading_bins=self.heading_bins,
        )


def load_footprint_state_lattice_reference(
    map_yaml: str | Path,
    *,
    footprint: Sequence[Sequence[float]],
    padding_m: float = 0.0,
    heading_bins: int = 16,
) -> FootprintStateLatticeReference:
    """Load unknown-blocked map data for the formal SE(2) reference solver."""
    padding = float(padding_m)
    if not math.isfinite(padding) or padding < 0.0:
        raise OptimalPathError("padding_m must be finite and non-negative")
    vertices: list[tuple[float, float]] = []
    for point in footprint:
        if len(point) != 2:
            raise OptimalPathError("footprint vertex requires exactly x/y")
        x, y = float(point[0]), float(point[1])
        if not math.isfinite(x) or not math.isfinite(y):
            raise OptimalPathError("footprint vertices must be finite")
        # Uniform padding is conservative and deliberately tiny (the planned
        # 5 mm numerical shell); collision itself remains polygon based.
        radius = math.hypot(x, y)
        scale = (radius + padding) / radius if radius > 1.0e-12 else 1.0
        vertices.append((x * scale, y * scale))
    grid = load_occupancy_grid_reference(map_yaml, clearance_m=0.0, allow_unknown=False)
    return FootprintStateLatticeReference(grid=grid, footprint=tuple(vertices), heading_bins=heading_bins)


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
