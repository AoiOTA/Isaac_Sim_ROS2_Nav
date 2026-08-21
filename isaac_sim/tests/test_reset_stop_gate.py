from __future__ import annotations

from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from isaac_sim.src.bridge.reset_stop_gate import (
    ResetStopGate,
    ResetStopGateError,
    ResetStopGateState,
)


ROOT = Path(__file__).resolve().parents[2]


def test_hold_release_requires_current_completed_generation():
    state = ResetStopGateState()
    first = state.hold()
    assert state.held
    with pytest.raises(ResetStopGateError, match="release rejected"):
        state.release(first)

    state.mark_reset_complete(first)
    state.release(first)
    assert not state.held


def test_new_reset_rejects_stale_release_and_stays_held_on_failure():
    state = ResetStopGateState()
    first = state.hold()
    state.mark_reset_complete(first)
    second = state.hold()

    with pytest.raises(ResetStopGateError, match="release rejected"):
        state.release(first)
    assert state.held
    assert state.eligible_generation is None

    state.mark_reset_complete(second)
    state.release(second)
    assert not state.held


def test_release_never_carries_command_state_between_epochs():
    state = ResetStopGateState()
    generation = state.hold()
    assert vars(state) == {
        "generation": generation,
        "held": True,
        "eligible_generation": None,
    }
    state.mark_reset_complete(generation)
    state.release(generation)
    assert vars(state) == {
        "generation": generation,
        "held": False,
        "eligible_generation": None,
    }


def test_release_status_publication_failure_leaves_live_gate_held():
    state = ResetStopGateState()
    generation = state.hold()
    state.mark_reset_complete(generation)
    published_zero = []

    def fail_status(reason, *, state=None):
        del reason, state
        raise RuntimeError("status publisher failed")

    gate = SimpleNamespace(
        state=state,
        _lock=threading.RLock(),
        publish_zero=lambda: published_zero.append(True),
        _publish_status=fail_status,
    )
    with pytest.raises(RuntimeError, match="status publisher failed"):
        ResetStopGate.release(gate, generation, source="test")

    assert state.held
    assert state.eligible_generation == generation
    assert published_zero == [True]


def test_completion_status_publication_failure_does_not_make_gate_eligible():
    state = ResetStopGateState()
    generation = state.hold()
    gate = SimpleNamespace(
        state=state,
        _lock=threading.RLock(),
        publish_zero=lambda: None,
        _publish_status=lambda reason, state=None: (_ for _ in ()).throw(
            RuntimeError("completion status publisher failed")
        ),
    )
    with pytest.raises(RuntimeError, match="completion status publisher failed"):
        ResetStopGate.mark_reset_complete(gate, generation)

    assert state.held
    assert state.eligible_generation is None


def test_navigation_profile_has_one_final_external_command_authority():
    reset_source = (
        ROOT / "isaac_sim/src/bridge/reset_service.py"
    ).read_text(encoding="utf-8")
    gate_source = (
        ROOT / "isaac_sim/src/bridge/reset_stop_gate.py"
    ).read_text(encoding="utf-8")
    topics = (
        ROOT / "isaac_sim/configs/ros2_bridge/topics.yaml"
    ).read_text(encoding="utf-8")

    assert 'create_publisher(Twist, "/cmd_vel"' not in reset_source
    assert 'input_topic: str = "/cmd_vel"' in gate_source
    assert 'output_topic: str = "/cmd_vel_sim"' in gate_source
    assert "cmd_vel: /cmd_vel_sim" in topics
    navigation_source = (
        ROOT / "isaac_sim/apps/navigation_sim.py"
    ).read_text(encoding="utf-8")
    assert 'Twist, "/cmd_vel_diagnostic", 1' in navigation_source
    assert 'create_publisher(Twist, "/cmd_vel", 1)' not in navigation_source


def test_startup_and_trigger_resets_share_one_release_owner_contract():
    navigation_source = (
        ROOT / "isaac_sim/apps/navigation_sim.py"
    ).read_text(encoding="utf-8")
    bridge_source = (
        ROOT / "isaac_sim/src/bridge/reset_service.py"
    ).read_text(encoding="utf-8")

    assert "startup_stop_released" not in navigation_source
    assert "startup_reset_complete" not in navigation_source
    assert "external_recovery_release_required=(" in navigation_source
    assert (
        'config.simulation.navigation_mode == "localization"'
        in navigation_source
    )
    assert "and not diagnostic_command_mode" in navigation_source
    assert 'source="reset_transaction_complete"' in bridge_source
