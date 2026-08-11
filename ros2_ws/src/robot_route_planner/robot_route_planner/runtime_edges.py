"""Runtime edge-state transitions for temporary local obstacles."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum


class RuntimeState(IntEnum):
    OPEN = 0
    SUSPECT = 1
    BLOCKED = 2
    UNKNOWN = 3


@dataclass
class RuntimeEdge:
    edge_id: int
    state: RuntimeState = RuntimeState.OPEN
    penalty_m: float = 0.0
    consecutive_failures: int = 0
    first_failure_s: float | None = None
    occupied_since_s: float | None = None
    clear_since_s: float | None = None
    last_observed_s: float | None = None
    state_changed_s: float = 0.0


class RuntimeEdgeManager:
    def __init__(self, settings: dict, route_cost_settings: dict) -> None:
        self.settings = settings
        self.route_cost_settings = route_cost_settings
        self.edges: dict[int, RuntimeEdge] = {}

    def state(self, edge_id: int) -> RuntimeEdge:
        return self.edges.setdefault(int(edge_id), RuntimeEdge(int(edge_id)))

    def observe_failure(self, edge_id: int, now_s: float, *, occupied_ahead: bool) -> RuntimeEdge:
        edge = self.state(edge_id)
        edge.last_observed_s = float(now_s)
        edge.clear_since_s = None
        edge.consecutive_failures += 1
        if edge.first_failure_s is None:
            edge.first_failure_s = float(now_s)
        if occupied_ahead and edge.occupied_since_s is None:
            edge.occupied_since_s = float(now_s)
        if not occupied_ahead:
            edge.occupied_since_s = None
        if edge.state != RuntimeState.BLOCKED:
            self._transition(
                edge,
                RuntimeState.SUSPECT,
                float(self.route_cost_settings["suspect_edge_penalty_m"]),
                now_s,
            )
        persistent = (
            edge.occupied_since_s is not None
            and now_s - edge.occupied_since_s
            >= float(self.settings["block_after_occupied_s"])
        )
        repeated = edge.consecutive_failures >= int(
            self.settings["block_after_consecutive_failures"]
        )
        if persistent and repeated:
            self._transition(
                edge,
                RuntimeState.BLOCKED,
                float(self.route_cost_settings["blocked_edge_penalty_m"]),
                now_s,
            )
        return edge

    def observe_clear(self, edge_id: int, now_s: float) -> RuntimeEdge:
        edge = self.state(edge_id)
        edge.last_observed_s = float(now_s)
        edge.occupied_since_s = None
        if edge.clear_since_s is None:
            edge.clear_since_s = float(now_s)
        edge.consecutive_failures = 0
        edge.first_failure_s = None
        if now_s - edge.clear_since_s >= float(self.settings["reopen_after_clear_s"]):
            self._transition(edge, RuntimeState.OPEN, 0.0, now_s)
        return edge

    def tick(self, now_s: float) -> list[RuntimeEdge]:
        changed = []
        for edge in self.edges.values():
            if (
                edge.last_observed_s is not None
                and now_s - edge.last_observed_s
                >= float(self.settings["unknown_after_unobserved_s"])
                and edge.state != RuntimeState.UNKNOWN
            ):
                self._transition(
                    edge,
                    RuntimeState.UNKNOWN,
                    float(self.route_cost_settings["unknown_edge_penalty_m"]),
                    now_s,
                )
                changed.append(edge)
        return changed

    @staticmethod
    def _transition(edge: RuntimeEdge, state: RuntimeState, penalty_m: float, now_s: float) -> None:
        if edge.state != state:
            edge.state_changed_s = float(now_s)
        edge.state = state
        edge.penalty_m = max(0.0, float(penalty_m))

    def route_cost_view(self) -> dict[int, tuple[float, bool]]:
        return {
            edge_id: (edge.penalty_m, edge.state == RuntimeState.BLOCKED)
            for edge_id, edge in self.edges.items()
        }


__all__ = ["RuntimeEdge", "RuntimeEdgeManager", "RuntimeState"]
