from __future__ import annotations

from pathlib import Path

import pytest

from isaac_sim.src.bridge.reset_stop_gate import (
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
