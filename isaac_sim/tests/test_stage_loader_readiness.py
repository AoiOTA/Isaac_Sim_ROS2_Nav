from __future__ import annotations

from pathlib import Path

import pytest

from isaac_sim.src.stage.stage_loader import (
    StageLoadError,
    wait_for_stable_context_stage,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


class FakeLayer:
    def __init__(self, path: Path) -> None:
        self.realPath = str(path)
        self.identifier = str(path)


class FakeStage:
    def __init__(self, path: Path) -> None:
        self.layer = FakeLayer(path)

    def GetRootLayer(self) -> FakeLayer:
        return self.layer


class FakeContext:
    def __init__(self, stage=None, pending: int = 0) -> None:
        self.stage = stage
        self.pending = pending

    def get_stage(self):
        return self.stage

    def get_stage_loading_status(self):
        return (0, 0, self.pending)


class FakeApp:
    def __init__(self, update) -> None:
        self.calls = 0
        self._update = update

    def update(self) -> None:
        self.calls += 1
        self._update(self.calls)


def test_instant_ready_stage_does_not_wait_or_update(tmp_path: Path):
    path = tmp_path / "ready.usda"
    stage = FakeStage(path)
    context = FakeContext(stage)
    app = FakeApp(lambda _calls: None)

    result = wait_for_stable_context_stage(
        context, app, path, 420.0, monotonic=FakeClock()
    )

    assert result is stage
    assert app.calls == 0


def test_blocking_update_crossing_deadline_passes_when_stage_returns_ready(
    tmp_path: Path,
):
    path = tmp_path / "cold.usda"
    stage = FakeStage(path)
    context = FakeContext()
    clock = FakeClock()

    def update(_calls: int) -> None:
        clock.now += 135.0
        context.stage = stage

    app = FakeApp(update)
    result = wait_for_stable_context_stage(
        context, app, path, 1.0, monotonic=clock
    )

    assert result is stage
    assert app.calls == 1


def test_several_updates_become_ready_within_default_bound(tmp_path: Path):
    path = tmp_path / "eventual.usda"
    stage = FakeStage(path)
    context = FakeContext()
    clock = FakeClock()

    def update(calls: int) -> None:
        clock.now += 100.0
        if calls == 4:
            context.stage = stage

    app = FakeApp(update)
    result = wait_for_stable_context_stage(
        context, app, path, 420.0, monotonic=clock
    )

    assert result is stage
    assert app.calls == 4


def test_missing_stage_past_bound_reports_actionable_timeout(tmp_path: Path):
    path = tmp_path / "missing.usda"
    context = FakeContext()
    clock = FakeClock()

    def update(_calls: int) -> None:
        clock.now += 140.0

    app = FakeApp(update)
    with pytest.raises(StageLoadError) as raised:
        wait_for_stable_context_stage(
            context, app, path, 420.0, monotonic=clock
        )

    message = str(raised.value)
    assert "elapsed=420.0s" in message
    assert "deadline=420.0s" in message
    assert "updates=3" in message
    assert "last_state=stage=None" in message
