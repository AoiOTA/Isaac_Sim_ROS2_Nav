from pathlib import Path

import pytest

from robot_experiments.v6_low_obstacle_causal import CausalContractError, load_manifest
from robot_experiments.v6_phase_f_active_ttl_probe import (
    ActiveTtlTimeline,
    PASS_STATE,
    PROBE_NOT_ARMED,
    build_probe_plan,
    execute_probe_lifecycle,
)


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "v6_kujiale_low_obstacle_causal.yaml"


class FakeAdapter:
    def __init__(self, arm, scenario="pass"):
        self.timeline = ActiveTtlTimeline(arm, margin_ns=200_000_000)
        self.scenario = scenario
        self.calls = []
        self._episode = {}

    def start_goal(self):
        self.calls.append("start_goal")
        self.timeline.observe_clock(1_000_000_000)
        self.timeline.start_goal()

    def wait_for_armed(self, _timeout_sec):
        self.calls.append("wait_for_armed")
        if self.scenario == "early_terminal":
            self.timeline.observe_terminal("collision", collision=True)
            return False
        if self.scenario == "noapply":
            return False
        self.timeline.observe_typed(
            source_sequence=7,
            validation_stamp_ns=900_000_000,
            validation_ttl_ns=500_000_000,
            trusted_write=True,
            healthy=True,
            observation_valid=True,
            obstacle_count=1,
        )
        for consumer in ("/global_costmap:cognitive", "/local_costmap:cognitive"):
            self.timeline.observe_layer(
                consumer=consumer,
                source_sequence=7,
                applied=True,
                raised_cell_count=4,
                active_cell_count=4,
                maximum_cost_increase=20,
                reason="",
            )
        if self.timeline.arm == "M3":
            self.timeline.observe_critic(
                source_sequence=7,
                applied=True,
                reason="cost_delta_applied=true;obstacle_applied=true",
            )
        return self.timeline.armed

    def stop_producer(self):
        self.calls.append("stop_producer")
        self.timeline.mark_producer_stopped()

    def wait_for_clear(self, _timeout_sec):
        self.calls.append("wait_sim_time_clear")
        self.timeline.observe_clock(self.timeline.expiry_target_ns)
        if self.scenario == "terminal_during_probe":
            self.timeline.observe_terminal("goal_terminal_during_probe")
            return False
        if self.scenario == "postexpiry_applied":
            self.timeline.observe_layer(
                consumer="/global_costmap:cognitive",
                source_sequence=7,
                applied=True,
                raised_cell_count=2,
                active_cell_count=2,
                maximum_cost_increase=10,
                reason="",
            )
            return False
        consumers = ["/global_costmap:cognitive", "/local_costmap:cognitive"]
        if self.scenario == "missing_clear":
            consumers.pop()
        for consumer in consumers:
            self.timeline.observe_layer(
                consumer=consumer,
                source_sequence=7,
                applied=False,
                raised_cell_count=0,
                active_cell_count=0,
                maximum_cost_increase=0,
                reason="rejection_reason=validation_stale",
            )
        if self.timeline.arm == "M3" and self.scenario != "missing_critic":
            self.timeline.observe_critic(
                source_sequence=7,
                applied=False,
                reason="cost_delta_applied=false;obstacle_rejected=validation_stale",
            )
        return self.timeline.clear_complete

    def cancel_goal_once(self, _reason):
        self.calls.append("cancel_goal_once")
        self.timeline.mark_cancel()

    def confirm_terminal_zero(self):
        self.calls.append("terminal_zero")
        confirmed = self.scenario != "no_terminal_zero"
        self._episode = {"state": "STOP", "terminal_zero_confirmed": confirmed}
        return confirmed

    def episode_result(self):
        return self._episode


@pytest.mark.parametrize("arm", ["M2", "M3"])
def test_fake_active_ttl_timeline_passes_and_preserves_adapter_order(arm):
    adapter = FakeAdapter(arm)
    result = execute_probe_lifecycle(
        adapter, arming_timeout_sec=3.0, probe_timeout_sec=3.0
    )

    assert result["state"] == PASS_STATE
    assert result["evaluator_fields"]["nominal_episode_ttl_clear_applicability"] == (
        "N/A_SEPARATE_ACTIVE_CONTROLLER_PROBE"
    )
    assert result["evaluator_fields"]["ttl_expiry_zero_write"] is True
    assert adapter.calls == [
        "start_goal",
        "wait_for_armed",
        "stop_producer",
        "wait_sim_time_clear",
        "cancel_goal_once",
        "terminal_zero",
    ]
    assert result["action"]["cancel_count"] == 1
    assert result["action"]["terminal_zero_confirmed"] is True
    if arm == "M3":
        assert result["evaluator_fields"]["critic_stale_active_probe"] == "STALE_REJECTED"


def test_no_positive_apply_is_probe_not_armed_and_cancels_active_goal():
    adapter = FakeAdapter("M2", "noapply")
    result = execute_probe_lifecycle(
        adapter, arming_timeout_sec=1.0, probe_timeout_sec=1.0
    )

    assert result["state"] == PROBE_NOT_ARMED
    assert "stop_producer" not in adapter.calls
    assert adapter.calls[-2:] == ["cancel_goal_once", "terminal_zero"]


def test_collision_or_goal_terminal_before_dropout_is_failure():
    adapter = FakeAdapter("M2", "early_terminal")
    result = execute_probe_lifecycle(
        adapter, arming_timeout_sec=1.0, probe_timeout_sec=1.0
    )

    assert result["state"] == "FAIL_EARLY_TERMINAL"
    assert result["action"]["terminal_before_dropout"] is True
    assert "stop_producer" not in adapter.calls


def test_missing_global_or_local_clear_is_failure():
    result = execute_probe_lifecycle(
        FakeAdapter("M2", "missing_clear"),
        arming_timeout_sec=1.0,
        probe_timeout_sec=1.0,
    )
    assert result["state"] == "FAIL_MISSING_LAYER_CLEAR"


def test_m3_missing_active_stale_critic_callback_is_failure():
    result = execute_probe_lifecycle(
        FakeAdapter("M3", "missing_critic"),
        arming_timeout_sec=1.0,
        probe_timeout_sec=1.0,
    )
    assert result["state"] == "FAIL_MISSING_CRITIC_STALE_REJECTION"


def test_post_expiry_applied_status_is_failure():
    result = execute_probe_lifecycle(
        FakeAdapter("M2", "postexpiry_applied"),
        arming_timeout_sec=1.0,
        probe_timeout_sec=1.0,
    )
    assert result["state"] == "FAIL_POST_EXPIRY_APPLIED"


def test_goal_terminal_after_dropout_before_clear_is_failure():
    result = execute_probe_lifecycle(
        FakeAdapter("M2", "terminal_during_probe"),
        arming_timeout_sec=1.0,
        probe_timeout_sec=1.0,
    )
    assert result["state"] == "FAIL_GOAL_TERMINATED_DURING_PROBE"


def test_terminal_zero_is_required_after_single_cancel():
    result = execute_probe_lifecycle(
        FakeAdapter("M2", "no_terminal_zero"),
        arming_timeout_sec=1.0,
        probe_timeout_sec=1.0,
    )
    assert result["state"] == "FAIL_TERMINAL_ZERO"
    assert result["action"]["cancel_count"] == 1


def test_sim_clock_controls_expiry_and_rejects_backward_jump():
    timeline = ActiveTtlTimeline("M2", margin_ns=200_000_000)
    timeline.observe_clock(1_000_000_000)
    timeline.start_goal()
    timeline.observe_typed(
        source_sequence=7,
        validation_stamp_ns=900_000_000,
        validation_ttl_ns=500_000_000,
        trusted_write=True,
        healthy=True,
        observation_valid=True,
        obstacle_count=1,
    )
    for consumer in ("global", "local"):
        timeline.observe_layer(
            consumer=consumer,
            source_sequence=7,
            applied=True,
            raised_cell_count=1,
            active_cell_count=1,
            maximum_cost_increase=1,
            reason="",
        )
    timeline.mark_producer_stopped()
    timeline.observe_clock(timeline.expiry_target_ns - 1)
    timeline.observe_layer(
        consumer="global",
        source_sequence=7,
        applied=False,
        raised_cell_count=0,
        active_cell_count=0,
        maximum_cost_increase=0,
        reason="validation_stale",
    )
    assert not timeline.clear_layers
    with pytest.raises(CausalContractError, match="clock moved backwards"):
        timeline.observe_clock(1)


def test_plan_is_m2_m3_only_and_keeps_exact_fixed_scene_contract(tmp_path):
    manifest = load_manifest(CONFIG)
    plan = build_probe_plan(manifest, tmp_path / "probe")

    assert [row["arm"] for row in plan["runs"]] == ["M2", "M3"]
    assert plan["nominal_episode_ttl"] == "N/A_SEPARATE_ACTIVE_CONTROLLER_PROBE"
    for row in plan["runs"]:
        assert row["start"]["id"] == "G1"
        assert row["goal"]["id"] == "G2"
        assert row["obstacle_layer_mode"] == "active"
        assert row["direct_rgbd_costmap_enabled"] is False
        assert row["module1_amcl_prior_enabled"] is False
        assert row["cognitive_place_graph_enabled"] is False
        assert row["commands"]["scene"][0].endswith("run_v6_r5_phase_b_kujiale.sh")
        assert row["commands"]["stack"][1] == row["arm"]
        assert row["commands"]["producer_stop"][1] == "stop-producer"
