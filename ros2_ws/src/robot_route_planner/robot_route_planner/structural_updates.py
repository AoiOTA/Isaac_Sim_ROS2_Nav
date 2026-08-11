"""Detect persistent changes only from structural occupancy-map snapshots."""

from __future__ import annotations

import cv2
import numpy as np


class StructuralChangeMonitor:
    def __init__(self, baseline_free: np.ndarray, resolution_m: float, settings: dict) -> None:
        self.baseline = np.asarray(baseline_free, dtype=bool).copy()
        self.resolution_m = float(resolution_m)
        self.settings = settings
        self.last_candidate: np.ndarray | None = None
        self.first_stable_s: float | None = None
        self.stable_count = 0

    @staticmethod
    def _component_count(free: np.ndarray) -> int:
        count, _ = cv2.connectedComponents(np.asarray(free, dtype=np.uint8))
        return max(0, int(count) - 1)

    def observe(self, structural_free: np.ndarray, now_s: float) -> bool:
        candidate = np.asarray(structural_free, dtype=bool)
        if candidate.shape != self.baseline.shape:
            raise ValueError("structural map shape changed")
        changed_area = float(np.count_nonzero(candidate != self.baseline)) * self.resolution_m**2
        connectivity_changed = self._component_count(candidate) != self._component_count(self.baseline)
        material = (
            changed_area >= float(self.settings["changed_area_m2"])
            or connectivity_changed
        )
        if not material:
            self.last_candidate = None
            self.first_stable_s = None
            self.stable_count = 0
            return False
        if self.last_candidate is None or not np.array_equal(candidate, self.last_candidate):
            self.last_candidate = candidate.copy()
            self.first_stable_s = float(now_s)
            self.stable_count = 1
            return False
        self.stable_count += 1
        stable_time = now_s - float(self.first_stable_s)
        return bool(
            self.stable_count >= int(self.settings["stable_snapshot_count"])
            and stable_time >= float(self.settings["stable_for_s"])
        )

    def accept_rebuild(self) -> None:
        if self.last_candidate is None:
            raise RuntimeError("no stable structural candidate to accept")
        self.baseline = self.last_candidate.copy()
        self.last_candidate = None
        self.first_stable_s = None
        self.stable_count = 0


__all__ = ["StructuralChangeMonitor"]
