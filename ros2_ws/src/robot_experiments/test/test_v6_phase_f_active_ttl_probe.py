import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_experiments import v6_phase_f_active_ttl_probe as probe_module
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
CRITIC_SOURCE = (
    PACKAGE.parent / "bio_nav_fusion" / "src" / "cognitive_risk_critic.cpp"
)


def _identity(sequence=7, *, epoch=3, session="session", map_version="map"):
    return {
        "reset_epoch": epoch,
        "recurrent_session_id": session,
        "map_version": map_version,
        "source_sequence": sequence,
    }


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
            **_identity(),
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
                **_identity(),
                applied=True,
                raised_cell_count=4,
                active_cell_count=4,
                maximum_cost_increase=20,
                reason="",
            )
        if self.timeline.arm == "M3":
            self.timeline.observe_critic(
                **_identity(),
                applied=True,
                reason="cost_delta_applied=true;obstacle_applied=true",
            )
        return self.timeline.armed

    def stop_producer(self):
        self.calls.append("stop_producer")
        if self.scenario == "stop_exception":
            raise CausalContractError("producer stop failed")
        self.timeline.mark_producer_stopped()

    def wait_for_clear(self, _timeout_sec):
        self.calls.append("wait_sim_time_clear")
        if self.scenario == "clear_exception":
            raise CausalContractError("clear wait failed")
        self.timeline.observe_clock(self.timeline.expiry_target_ns)
        if self.scenario == "terminal_during_probe":
            self.timeline.observe_terminal("goal_terminal_during_probe")
            return False
        if self.scenario == "postexpiry_applied":
            self.timeline.observe_layer(
                consumer="/global_costmap:cognitive",
                **_identity(),
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
                **_identity(),
                applied=False,
                raised_cell_count=0,
                active_cell_count=0,
                maximum_cost_increase=0,
                reason="rejection_reason=validation_stale",
            )
        if self.timeline.arm == "M3" and self.scenario != "missing_critic":
            self.timeline.observe_critic(
                **_identity(),
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
        state = "FAILED" if self.scenario == "episode_failed" else "STOP"
        stop_reason = (
            "navigation_failed"
            if self.scenario == "episode_failed"
            else "unexpected_stop"
            if self.scenario == "episode_unexpected_stop"
            else "active_ttl_probe_complete"
        )
        self._episode = {
            "state": state,
            "stop_reason": stop_reason,
            "collision": self.scenario == "late_collision",
            "terminal_zero_confirmed": confirmed,
        }
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
    assert result["evaluator_fields"]["ttl_reset_epoch"] == 3
    assert result["evaluator_fields"]["ttl_recurrent_session_id"] == "session"
    assert result["evaluator_fields"]["ttl_map_version"] == "map"
    assert result["evaluator_fields"]["ttl_source_sequence"] == 7
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
    else:
        assert result["positive_apply"]["critic"] is None
        assert result["evaluator_fields"]["critic_stale_active_probe"] == "NOT_APPLICABLE"
        assert result["evaluator_fields"]["critic_post_expiry_applied"] is None


def test_no_positive_apply_is_probe_not_armed_and_cancels_active_goal():
    adapter = FakeAdapter("M2", "noapply")
    result = execute_probe_lifecycle(
        adapter, arming_timeout_sec=1.0, probe_timeout_sec=1.0
    )

    assert result["state"] == PROBE_NOT_ARMED
    assert "stop_producer" not in adapter.calls
    assert adapter.calls[-2:] == ["cancel_goal_once", "terminal_zero"]


def _observe_positive(
    timeline, sequence=7, *, epoch=3, session="session", map_version="map"
):
    identity = _identity(
        sequence, epoch=epoch, session=session, map_version=map_version
    )
    timeline.observe_typed(
        **identity,
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
            **identity,
            applied=True,
            raised_cell_count=1,
            active_cell_count=1,
            maximum_cost_increase=1,
            reason="",
        )


def test_pre_goal_positive_callbacks_do_not_arm_new_goal_epoch():
    timeline = ActiveTtlTimeline("M2", margin_ns=200_000_000)
    _observe_positive(timeline)

    timeline.start_goal()

    assert timeline.goal_epoch == 1
    assert timeline.trusted_typed_seen is False
    assert timeline.positive_layers == set()
    assert timeline.armed is False


def test_post_goal_positive_callbacks_arm_current_goal_epoch():
    timeline = ActiveTtlTimeline("M2", margin_ns=200_000_000)
    timeline.start_goal()

    _observe_positive(timeline)

    assert timeline.goal_epoch == 1
    assert timeline.armed is True


def test_positive_evidence_from_different_sources_cannot_arm():
    timeline = ActiveTtlTimeline("M2", margin_ns=200_000_000)
    timeline.start_goal()
    for sequence in (7, 8):
        timeline.observe_typed(
            **_identity(sequence),
            validation_stamp_ns=900_000_000,
            validation_ttl_ns=500_000_000,
            trusted_write=True,
            healthy=True,
            observation_valid=True,
            obstacle_count=1,
        )
    timeline.observe_layer(
        consumer="global",
        **_identity(7),
        applied=True,
        raised_cell_count=1,
        active_cell_count=1,
        maximum_cost_increase=1,
        reason="",
    )
    timeline.observe_layer(
        consumer="local",
        **_identity(8),
        applied=True,
        raised_cell_count=1,
        active_cell_count=1,
        maximum_cost_increase=1,
        reason="",
    )

    assert timeline.armed is False


@pytest.mark.parametrize(
    "other_identity",
    [
        _identity(7, session="other-session"),
        _identity(7, epoch=4),
        _identity(7, map_version="other-map"),
    ],
)
def test_same_sequence_cross_identity_layer_cannot_arm(other_identity):
    timeline = ActiveTtlTimeline("M2", margin_ns=200_000_000)
    timeline.start_goal()
    timeline.observe_typed(
        **_identity(7),
        validation_stamp_ns=900_000_000,
        validation_ttl_ns=500_000_000,
        trusted_write=True,
        healthy=True,
        observation_valid=True,
        obstacle_count=1,
    )
    timeline.observe_layer(
        consumer="global",
        **_identity(7),
        applied=True,
        raised_cell_count=1,
        active_cell_count=1,
        maximum_cost_increase=1,
        reason="",
    )
    timeline.observe_layer(
        consumer="local",
        **other_identity,
        applied=True,
        raised_cell_count=1,
        active_cell_count=1,
        maximum_cost_increase=1,
        reason="",
    )

    assert timeline.armed is False


def test_m3_critic_must_match_typed_and_layer_source():
    timeline = ActiveTtlTimeline("M3", margin_ns=200_000_000)
    timeline.start_goal()
    _observe_positive(timeline, sequence=7)
    timeline.observe_critic(
        **_identity(7, session="other-session"),
        applied=True,
        reason="cost_delta_applied=true",
    )
    assert timeline.armed is False

    timeline.observe_critic(
        **_identity(7),
        applied=True,
        reason="cost_delta_applied=true",
    )
    assert timeline.armed is True


def test_post_stop_queued_source_cannot_replace_target_or_satisfy_clear():
    timeline = ActiveTtlTimeline("M2", margin_ns=200_000_000)
    timeline.start_goal()
    _observe_positive(timeline, sequence=7)
    timeline.mark_producer_stopped()
    target = (
        timeline.armed_source_identity,
        timeline.armed_validation_stamp_ns,
        timeline.armed_validation_ttl_ns,
        timeline.expiry_target_ns,
    )

    timeline.observe_typed(
        **_identity(7, epoch=4, session="queued-session"),
        validation_stamp_ns=1_500_000_000,
        validation_ttl_ns=900_000_000,
        trusted_write=True,
        healthy=True,
        observation_valid=True,
        obstacle_count=1,
    )
    assert (
        timeline.armed_source_identity,
        timeline.armed_validation_stamp_ns,
        timeline.armed_validation_ttl_ns,
        timeline.expiry_target_ns,
    ) == target

    timeline.observe_clock(timeline.expiry_target_ns)
    for consumer in ("global", "local"):
        timeline.observe_layer(
            consumer=consumer,
            **_identity(7, epoch=4, session="queued-session"),
            applied=False,
            raised_cell_count=0,
            active_cell_count=0,
            maximum_cost_increase=0,
            reason="validation_stale",
        )
    assert timeline.clear_layers == set()

    for consumer in ("global", "local"):
        timeline.observe_layer(
            consumer=consumer,
            **_identity(7),
            applied=False,
            raised_cell_count=0,
            active_cell_count=0,
            maximum_cost_increase=0,
            reason="validation_stale",
        )
    assert timeline.clear_complete is True


def test_post_stop_cross_identity_critic_cannot_satisfy_clear():
    timeline = ActiveTtlTimeline("M3", margin_ns=200_000_000)
    timeline.start_goal()
    _observe_positive(timeline)
    timeline.observe_critic(
        **_identity(7),
        applied=True,
        reason="cost_delta_applied=true",
    )
    timeline.mark_producer_stopped()
    frozen_identity = timeline.armed_source_identity
    frozen_expiry = timeline.expiry_target_ns
    timeline.observe_clock(frozen_expiry)
    for consumer in ("global", "local"):
        timeline.observe_layer(
            consumer=consumer,
            **_identity(7),
            applied=False,
            raised_cell_count=0,
            active_cell_count=0,
            maximum_cost_increase=0,
            reason="validation_stale",
        )
    timeline.observe_critic(
        **_identity(7, epoch=4, session="other-session"),
        applied=False,
        reason="cost_delta_applied=false;obstacle_rejected=validation_stale",
    )

    assert timeline.armed_source_identity == frozen_identity
    assert timeline.expiry_target_ns == frozen_expiry
    assert timeline.critic_stale_rejected is False
    assert timeline.clear_complete is False

    timeline.observe_critic(
        **_identity(7),
        applied=False,
        reason="cost_delta_applied=false;obstacle_rejected=validation_stale",
    )
    assert timeline.clear_complete is True


def _armed_m3_timeline():
    timeline = ActiveTtlTimeline("M3", margin_ns=200_000_000)
    timeline.start_goal()
    _observe_positive(timeline)
    timeline.observe_critic(
        **_identity(7),
        applied=True,
        reason="cost_delta_applied=true;obstacle_applied=true",
    )
    timeline.mark_producer_stopped()
    return timeline


def test_m3_real_offer_rejection_string_cannot_clear_critic_ttl():
    critic_source = CRITIC_SOURCE.read_text(encoding="utf-8")
    assert '"offer_rejected=" + reason +' in critic_source
    timeline = _armed_m3_timeline()
    timeline.observe_clock(timeline.expiry_target_ns)

    timeline.observe_critic(
        **_identity(7),
        applied=False,
        reason=(
            "offer_rejected=validation_stale;"
            "offer_reset_epoch=4;offer_session=queued-session"
        ),
    )

    assert timeline.critic_stale_rejected is False
    assert timeline.clear_complete is False


def test_m3_real_obstacle_rejection_string_needs_expiry_and_full_identity():
    critic_source = CRITIC_SOURCE.read_text(encoding="utf-8")
    assert '"obstacle_rejected=" + reason' in critic_source
    timeline = _armed_m3_timeline()

    timeline.observe_clock(timeline.expiry_target_ns - 1)
    timeline.observe_critic(
        **_identity(7),
        applied=False,
        reason="obstacle_rejected=validation_stale",
    )
    assert timeline.critic_stale_rejected is False

    timeline.observe_clock(timeline.expiry_target_ns)
    timeline.observe_critic(
        **_identity(7, session="other-session"),
        applied=False,
        reason="obstacle_rejected=validation_stale",
    )
    assert timeline.critic_stale_rejected is False

    timeline.observe_critic(
        **_identity(7),
        applied=False,
        reason="obstacle_rejected=validation_stale",
    )
    assert timeline.critic_stale_rejected is True


@pytest.mark.parametrize(
    ("applied", "reason"),
    [
        (True, "obstacle_rejected=validation_stale"),
        (False, "cost_delta_applied=true;obstacle_rejected=validation_stale"),
    ],
)
def test_m3_obstacle_stale_rejection_cannot_clear_with_applied_cost(applied, reason):
    timeline = _armed_m3_timeline()
    timeline.observe_clock(timeline.expiry_target_ns)

    timeline.observe_critic(
        **_identity(7),
        applied=applied,
        reason=reason,
    )

    assert timeline.critic_stale_rejected is False
    assert timeline.post_expiry_applied is True


def test_post_stop_queued_source_positive_after_expiry_fails_probe():
    timeline = ActiveTtlTimeline("M2", margin_ns=200_000_000)
    timeline.start_goal()
    _observe_positive(timeline, sequence=7)
    timeline.mark_producer_stopped()
    timeline.observe_clock(timeline.expiry_target_ns)

    timeline.observe_layer(
        consumer="global",
        **_identity(8),
        applied=True,
        raised_cell_count=1,
        active_cell_count=1,
        maximum_cost_increase=1,
        reason="",
    )

    assert timeline.post_expiry_applied is True
    assert timeline.clear_complete is False


@pytest.mark.parametrize("scenario", ["stop_exception", "clear_exception"])
def test_probe_exception_still_cancels_once_and_checks_terminal_zero(scenario):
    adapter = FakeAdapter("M2", scenario)

    with pytest.raises(CausalContractError):
        execute_probe_lifecycle(
            adapter, arming_timeout_sec=1.0, probe_timeout_sec=1.0
        )

    assert adapter.calls.count("cancel_goal_once") == 1
    assert adapter.timeline.cancel_count == 1
    assert adapter.calls[-1] == "terminal_zero"
    assert adapter.timeline.terminal_zero_confirmed is True


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


@pytest.mark.parametrize(
    ("scenario", "expected_state"),
    [
        ("late_collision", "FAIL_EPISODE_COLLISION"),
        ("episode_failed", "FAIL_EPISODE_NAVIGATION"),
        ("episode_unexpected_stop", "FAIL_EPISODE_TERMINAL"),
    ],
)
def test_final_episode_failure_overrides_ttl_pass(scenario, expected_state):
    result = execute_probe_lifecycle(
        FakeAdapter("M2", scenario),
        arming_timeout_sec=1.0,
        probe_timeout_sec=1.0,
    )

    assert result["state"] == expected_state
    assert result["state"] != PASS_STATE


def test_sim_clock_controls_expiry_and_rejects_backward_jump():
    timeline = ActiveTtlTimeline("M2", margin_ns=200_000_000)
    timeline.observe_clock(1_000_000_000)
    timeline.start_goal()
    timeline.observe_typed(
        **_identity(7),
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
            **_identity(7),
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
        **_identity(7),
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
    assert plan["selected_arm"] is None
    assert plan["qualification"] == "ENGINEERING_ONLY_NOT_FORMAL"
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


def test_selected_m2_plan_contains_only_repeat1_m2_and_no_m3_command(tmp_path):
    manifest = load_manifest(CONFIG)
    plan = build_probe_plan(
        manifest,
        tmp_path / "probe",
        selected_arm="M2",
    )

    assert plan["selected_arm"] == "M2"
    assert plan["qualification"] == "ENGINEERING_ONLY_NOT_FORMAL"
    assert plan["mode"] == "M2_ACTIVE_CONTROLLER_TTL_PROBE"
    assert [(row["arm"], row["repeat"]) for row in plan["runs"]] == [("M2", 1)]
    assert "M3" not in json.dumps(plan)


def test_selected_m3_plan_contains_only_repeat1_m3_and_no_m2_command(tmp_path):
    manifest = load_manifest(CONFIG)
    plan = build_probe_plan(
        manifest,
        tmp_path / "probe",
        selected_arm="M3",
    )

    assert plan["selected_arm"] == "M3"
    assert plan["qualification"] == "ENGINEERING_ONLY_NOT_FORMAL"
    assert plan["mode"] == "M3_ACTIVE_CONTROLLER_TTL_PROBE"
    assert [(row["arm"], row["repeat"]) for row in plan["runs"]] == [("M3", 1)]
    assert plan["runs"][0]["critic_mode"] == "active"
    assert "M2" not in json.dumps(plan)


def test_invalid_selected_arm_is_rejected_by_cli_parser():
    with pytest.raises(SystemExit) as raised:
        probe_module.build_parser().parse_args([
            "plan",
            "--config", str(CONFIG),
            "--output-root", "/tmp/ttl-probe",
            "--selected-arm", "M1",
        ])

    assert raised.value.code == 2


@pytest.mark.parametrize(
    ("m2_state", "cleanup_ok"),
    [("FAIL_MISSING_LAYER_CLEAR", True), (PASS_STATE, False)],
)
def test_m2_failure_or_cleanup_failure_does_not_start_m3(
    tmp_path, monkeypatch, m2_state, cleanup_ok
):
    manifest = load_manifest(CONFIG)
    dispatched = []

    monkeypatch.setattr(
        probe_module,
        "_start_process",
        lambda name, *_args, **_kwargs: SimpleNamespace(name=name),
    )
    monkeypatch.setattr(
        probe_module,
        "_wait_for_startup_ready",
        lambda *_args, **_kwargs: {"ready": True},
    )
    monkeypatch.setattr(
        probe_module,
        "_wait_for_cognitive_ready",
        lambda *_args, **_kwargs: {"ready": True},
    )
    monkeypatch.setattr(
        probe_module,
        "_stop_process",
        lambda process, _timeout: {"name": process.name, "stopped": True},
    )
    monkeypatch.setattr(
        probe_module,
        "_confirm_arm_cleanup",
        lambda *_args, **_kwargs: {"ok": cleanup_ok},
    )

    def fake_dispatch(_manifest, run, *_args, **_kwargs):
        dispatched.append(run.arm)
        return {"state": m2_state}

    monkeypatch.setattr(probe_module, "dispatch_live_probe", fake_dispatch)

    result = probe_module.run_probe_campaign(
        manifest,
        tmp_path / "campaign",
        arming_timeout_sec=1.0,
        probe_timeout_sec=1.0,
        shutdown_timeout_sec=1.0,
    )

    assert result["state"] == "FAILED"
    assert dispatched == ["M2"]
    assert [row["arm"] for row in result["runs"]] == ["M2"]


@pytest.mark.parametrize(
    ("selected_arm", "expected_arms"),
    [("M2", ["M2"]), ("M3", ["M3"]), (None, ["M2", "M3"])],
)
def test_campaign_selection_preserves_default_and_selected_dispatch(
    tmp_path, monkeypatch, selected_arm, expected_arms
):
    manifest = load_manifest(CONFIG)
    dispatched = []

    monkeypatch.setattr(
        probe_module,
        "_start_process",
        lambda name, *_args, **_kwargs: SimpleNamespace(name=name),
    )
    monkeypatch.setattr(
        probe_module,
        "_wait_for_startup_ready",
        lambda *_args, **_kwargs: {"ready": True},
    )
    monkeypatch.setattr(
        probe_module,
        "_wait_for_cognitive_ready",
        lambda *_args, **_kwargs: {"ready": True},
    )
    monkeypatch.setattr(
        probe_module,
        "_stop_process",
        lambda process, _timeout: {"name": process.name, "stopped": True},
    )
    monkeypatch.setattr(
        probe_module,
        "_confirm_arm_cleanup",
        lambda *_args, **_kwargs: {"ok": True},
    )

    def fake_dispatch(_manifest, run, *_args, **_kwargs):
        dispatched.append(run.arm)
        return {"state": PASS_STATE}

    monkeypatch.setattr(probe_module, "dispatch_live_probe", fake_dispatch)

    result = probe_module.run_probe_campaign(
        manifest,
        tmp_path / "campaign",
        arming_timeout_sec=1.0,
        probe_timeout_sec=1.0,
        shutdown_timeout_sec=1.0,
        selected_arm=selected_arm,
    )

    assert result["state"] == "PASS"
    assert result["selected_arm"] == selected_arm
    assert result["qualification"] == "ENGINEERING_ONLY_NOT_FORMAL"
    assert dispatched == expected_arms
    assert [row["arm"] for row in result["runs"]] == expected_arms
