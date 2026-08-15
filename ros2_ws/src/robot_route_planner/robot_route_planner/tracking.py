"""Canonical-polyline projection and lookahead independent of Route Server path."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .models import Edge, Graph


@dataclass(frozen=True)
class Progress:
    edge_id: int
    edge_index: int
    arc_length_m: float
    lateral_error_m: float
    remaining_m: float
    projected_xy: tuple[float, float]
    lookahead_xy: tuple[float, float]
    use_final_goal: bool


def _project(
    point: np.ndarray, polyline: np.ndarray
) -> tuple[float, float, tuple[float, float]]:
    segments = np.diff(polyline, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    best = (math.inf, 0.0, polyline[0])
    for index, segment in enumerate(segments):
        squared = float(segment @ segment)
        fraction = 0.0 if squared <= 0.0 else float(
            np.clip((point - polyline[index]) @ segment / squared, 0.0, 1.0)
        )
        projected = polyline[index] + fraction * segment
        distance = float(np.linalg.norm(point - projected))
        arc = float(cumulative[index] + fraction * lengths[index])
        if (distance, -arc) < (best[0], -best[1]):
            best = (distance, arc, projected)
    return best[1], best[0], (float(best[2][0]), float(best[2][1]))


def _point_at(polyline: np.ndarray, distance: float) -> tuple[float, float]:
    lengths = np.linalg.norm(np.diff(polyline, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    target = float(np.clip(distance, 0.0, cumulative[-1]))
    index = min(int(np.searchsorted(cumulative, target, side="right") - 1), len(lengths) - 1)
    fraction = 0.0 if lengths[index] <= 0.0 else (
        target - cumulative[index]
    ) / lengths[index]
    point = polyline[index] + fraction * (polyline[index + 1] - polyline[index])
    return float(point[0]), float(point[1])


class RouteTracker:
    def __init__(
        self,
        graph: Graph,
        edge_ids: list[int],
        settings: dict,
        *,
        route_segments_xy: list[np.ndarray] | None = None,
    ) -> None:
        edge_map = graph.edge_by_id()
        if route_segments_xy is None:
            self.edges = [edge_map[edge_id] for edge_id in edge_ids]
        else:
            if len(route_segments_xy) != len(edge_ids):
                raise ValueError("route segments must match canonical edge IDs")
            self.edges = []
            for edge_id, points in zip(edge_ids, route_segments_xy):
                source = edge_map[edge_id]
                polyline = np.asarray(points, dtype=np.float64)
                length = float(
                    np.linalg.norm(np.diff(polyline, axis=0), axis=1).sum()
                )
                self.edges.append(
                    Edge(
                        id=source.id,
                        from_node=source.from_node,
                        to_node=source.to_node,
                        polyline_xy=polyline,
                        length_m=length,
                        min_clearance_m=source.min_clearance_m,
                        mean_clearance_m=source.mean_clearance_m,
                        p05_clearance_m=source.p05_clearance_m,
                        nominal_width_m=source.nominal_width_m,
                        max_curvature_per_m=source.max_curvature_per_m,
                        bottleneck=source.bottleneck,
                        static_traversability=source.static_traversability,
                        predecessor_ids=source.predecessor_ids,
                        metadata=source.metadata,
                    )
                )
        if not self.edges:
            raise ValueError("canonical route must contain at least one edge")
        self.settings = settings
        self.offsets = np.concatenate(
            ([0.0], np.cumsum([edge.length_m for edge in self.edges]))
        )
        self.progress_m = 0.0
        self.edge_index = 0

    def point_at_distance_ahead(self, distance_m: float) -> tuple[float, float]:
        """Return a point on this Route without mutating tracked progress."""

        global_arc = min(
            float(self.offsets[-1]),
            self.progress_m + max(0.0, float(distance_m)),
        )
        edge_index = min(
            int(np.searchsorted(self.offsets, global_arc, side="right") - 1),
            len(self.edges) - 1,
        )
        return _point_at(
            self.edges[edge_index].polyline_xy,
            global_arc - float(self.offsets[edge_index]),
        )

    def update(self, position_xy: tuple[float, float]) -> Progress:
        point = np.asarray(position_xy, dtype=np.float64)
        window = int(self.settings["projection_edge_window"])
        last_index = min(len(self.edges), self.edge_index + window)
        candidates = []
        for index in range(self.edge_index, last_index):
            local_arc, lateral, projected = _project(
                point, self.edges[index].polyline_xy
            )
            global_arc = float(self.offsets[index] + local_arc)
            if global_arc >= self.progress_m - float(self.settings["max_backtrack_m"]):
                candidates.append(
                    (lateral, -global_arc, index, global_arc, projected)
                )
        if candidates:
            lateral, _, candidate_index, candidate_arc, projected = min(candidates)
            hysteresis = float(self.settings["advance_hysteresis_m"])
            if candidate_index > self.edge_index:
                boundary = float(self.offsets[candidate_index])
                if candidate_arc < boundary + hysteresis:
                    candidate_index = self.edge_index
                    local_arc, lateral, projected = _project(
                        point, self.edges[candidate_index].polyline_xy
                    )
                    candidate_arc = float(self.offsets[candidate_index] + local_arc)
            self.edge_index = candidate_index
            self.progress_m = max(
                self.progress_m - float(self.settings["max_backtrack_m"]),
                candidate_arc,
            )
        else:
            lateral = math.inf
            projected = (math.nan, math.nan)
        total = float(self.offsets[-1])
        remaining = max(0.0, total - self.progress_m)
        # The support graph now attaches at footprint-feasible dense points and
        # the tracker uses the exact ordered support segments returned by Route
        # Server. Preserve the macro route until genuinely near its tail; an
        # entire short Route must not be bypassed merely because it fits inside
        # the lookahead horizon.
        final = remaining <= float(self.settings["final_goal_switch_distance_m"])
        lookahead = self.point_at_distance_ahead(
            float(self.settings["lookahead_m"])
        )
        return Progress(
            edge_id=self.edges[self.edge_index].id,
            edge_index=self.edge_index,
            arc_length_m=self.progress_m,
            lateral_error_m=float(lateral),
            remaining_m=remaining,
            projected_xy=projected,
            lookahead_xy=lookahead,
            use_final_goal=final,
        )


__all__ = ["Progress", "RouteTracker"]
