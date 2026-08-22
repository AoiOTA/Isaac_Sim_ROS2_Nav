"""Deterministic tests for the active-reset probe state machine."""

import json
import math
from types import SimpleNamespace

import pytest

import robot_experiments.active_reset_probe as probe_module
from robot_experiments.active_reset_probe import (
    COMMAND_TOPICS,
    GROUND_TRUTH_TOPIC,
    ODOMETRY_TOPIC,
    ProbeConfig,
    ProbeMachine,
    _arguments,
    _twist_values,
    dispatch_reset_once,
    finalize_probe_output,
    teardown_ros_node,
    validate_topology_snapshot,
)


def _gate(reason, *, held, eligible=None, generation=2):
    return json.dumps({
        "generation": generation,
        "held": held,
        "eligible_generation": eligible,
        "reason": reason,
    })


def _receipt(*, seed=8601, generation=2, pose="long_route_start_g1", odometry="realistic"):
    return "ok; reset_receipt=" + json.dumps({
        "seed": seed,
        "generation": generation,
        "pose": pose,
        "odometry": odometry,
        "case_id": "",
        "variant_id": "",
    })


def _armed(machine=None):
    machine = machine or ProbeMachine()
    machine.gate_status(
        _gate("released:startup", held=False, generation=1), -0.01
    )
    machine.ground_truth(0.0, 0.0, 0.0)
    machine.endpoints_ready(0.0, {"ready": True})
    assert machine.phase == "PUBLISH_OLD_ONCE"
    machine.old_published(0.01)
    return machine


def _active(machine=None):
    machine = _armed(machine)
    machine.canonical(2, [51, 52, 30], 0.02)
    machine.progress(2, 0.03)
    machine.route_output("lookahead", 0.04)
    machine.route_output("goal_update", 0.05)
    for index in range(5):
        machine.command("/cmd_vel_sim", True, 0.06 + index * 0.01)
    machine.ground_truth(0.11, 0.0, 0.12)
    assert machine.phase == "CALL_RESET_ONCE"
    return machine


def _reset_to_quiet(machine=None):
    machine = _active(machine)
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.14)
    machine.command("/cmd_vel_sim", False, 0.145)
    machine.collision_event(False, 0.145)
    # Attempt6 observed one cross-topic in-flight publish triplet 7.6 ms after
    # HOLD and before the matching abort pair.  It is bounded evidence, not the
    # coordinator-owned retirement fence.
    machine.progress(2, 0.147)
    machine.route_output("lookahead", 0.148)
    machine.navigate_intent(0.149)
    machine.reset_event(0.15)
    # This can be a large teleport from the old active pose.  The stable-drift
    # gate starts with the first sample after reset_complete, not before the
    # reset landing is authoritative.
    machine.ground_truth(0.45, -5.35, 0.16)
    machine.gate_status(_gate("reset_complete", held=True, eligible=2), 0.18)
    machine.ground_truth(0.45, -5.35, 0.185)
    machine.odometry(0.0, 0.0, 0.185)
    machine.terminal_bool(False, 0.19)
    machine.terminal_result(json.dumps({
        "request_id": 2,
        "status": "aborted",
        "reason": "simulation_reset",
        "reset_epoch": 2,
    }), 0.20)
    machine.reset_response(True, _receipt(), 0.21)
    machine.ground_truth(0.455, -5.35, 0.215)
    machine.odometry(0.005, 0.0, 0.215)
    machine.command("/cmd_vel_sim", False, 0.215)
    machine.collision_event(False, 0.215)
    machine.gate_status(_gate("released:activation_gate", held=False), 0.22)
    assert machine.phase == "QUIET"
    return machine


def _fresh_wait(machine=None):
    machine = _reset_to_quiet(machine)
    machine.tick(1.21)
    assert machine.phase == "PUBLISH_FRESH_ONCE"
    machine.fresh_published(1.22)
    machine.canonical(4, [51, 52], 1.23)
    machine.progress(4, 1.24)
    return machine


def _provisional(machine=None):
    machine = _fresh_wait(machine)
    machine.ground_truth(0.69, -3.98, 1.3)
    machine.terminal_bool(True, 1.31)
    machine.terminal_result(json.dumps({
        "request_id": 4,
        "status": "succeeded",
        "reason": "final_goal_distance_confirmed",
        "reset_epoch": 2,
    }), 1.32)
    for topic in COMMAND_TOPICS:
        for stamp in (1.35, 1.55, 1.75, 1.95, 2.15, 2.32):
            machine.command(topic, False, stamp)
    machine.tick(2.33)
    assert machine.phase == "PROVISIONAL_COMPLETE"
    return machine


def test_endpoint_waits_for_ground_truth_and_all_contract_flags():
    """Endpoint readiness requires both data and every topology flag."""
    machine = ProbeMachine()
    machine.endpoints_ready(0.0, {"ready": False})
    assert machine.phase == "WAIT_ENDPOINTS"
    machine.ground_truth(0.0, 0.0, 0.1)
    machine.endpoints_ready(0.1, {"ready": False})
    assert machine.phase == "WAIT_ENDPOINTS"
    machine.endpoints_ready(0.2, {"ready": True})
    assert machine.phase == "PUBLISH_OLD_ONCE"


def test_startup_retained_route_and_reset_event_are_not_claimed():
    """Startup retained state cannot be mistaken for this episode."""
    machine = ProbeMachine()
    machine.canonical(99, [9], 0.0)
    machine.reset_event(0.0)
    assert machine.old_request_id is None
    assert not machine.reset_event_seen


def test_exactly_once_old_publish_and_reset_call_fail_closed():
    """A second publication or reset call is terminal."""
    machine = _armed()
    machine.old_published(0.02)
    assert machine.phase == "STOP"
    assert machine.stop_reason == "old_goal_publish_not_exactly_once"

    machine = _active()
    machine.reset_call_started(0.13)
    machine.reset_call_started(0.14)
    assert machine.phase == "STOP"
    assert machine.stop_reason == "reset_call_not_exactly_once"


def test_active_ready_timeout_and_terminal_before_reset_stop():
    """Missing active evidence and pre-reset termination both stop."""
    machine = _armed()
    machine.tick(6.02)
    assert machine.stop_reason == "active_ready_timeout"

    machine = _armed()
    machine.terminal_bool(False, 0.1)
    assert machine.stop_reason == "old_terminal_before_reset"

    machine = _active()
    machine.terminal_result(json.dumps({
        "request_id": 2, "status": "failed", "reason": "nav_failed",
    }), 0.125)
    assert machine.stop_reason == "old_terminal_before_reset"


def test_reset_call_delay_is_bounded_to_half_second():
    """Trigger must follow active-ready within half a second."""
    machine = _active()
    machine.reset_call_started(machine.timestamps["active_ready"] + 0.501)
    assert machine.stop_reason == "reset_call_after_active_ready_too_late"


def test_receipt_mismatch_is_fail_closed():
    """Seed and reset substrate mismatches are rejected."""
    machine = _active()
    machine.reset_call_started(0.13)
    machine.reset_response(True, _receipt(seed=7), 0.2)
    assert machine.phase == "STOP"
    assert machine.stop_reason.startswith("reset_receipt_invalid:")

    machine = _active()
    machine.reset_call_started(0.13)
    machine.reset_response(True, _receipt(odometry="ideal"), 0.2)
    assert machine.stop_reason == "reset_receipt_contract_mismatch"


def test_gate_order_and_hold_nonzero_are_fail_closed():
    """Gate order and zero output are strict HOLD contracts."""
    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("reset_complete", held=True, eligible=2), 0.14)
    machine.gate_status(_gate("hold", held=True), 0.15)
    assert machine.stop_reason == "gate_status_order_invalid"

    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.14)
    machine.command("/cmd_vel_sim", True, 0.15)
    assert machine.stop_reason == "hold_cmd_vel_sim_nonzero"

    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.14)
    machine.gate_status(_gate("hold", held=True), 0.15)
    assert machine.stop_reason == "gate_status_order_invalid"


def test_reset_teleport_is_allowed_but_post_landing_drift_is_not():
    """The reset jump is excluded while post-landing drift is bounded."""
    machine = _reset_to_quiet()
    assert machine.reset_stable_max_drift_m <= 0.005001

    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.14)
    machine.command("/cmd_vel_sim", False, 0.145)
    machine.collision_event(False, 0.145)
    machine.reset_event(0.15)
    machine.ground_truth(0.45, -5.35, 0.16)
    machine.gate_status(_gate("reset_complete", held=True, eligible=2), 0.18)
    machine.ground_truth(0.45, -5.35, 0.181)
    machine.ground_truth(0.48, -5.35, 0.182)
    machine.odometry(0.0, 0.0, 0.181)
    machine.odometry(0.005, 0.0, 0.215)
    machine.terminal_bool(False, 0.19)
    machine.terminal_result(json.dumps({
        "request_id": 2, "status": "aborted",
        "reason": "simulation_reset", "reset_epoch": 2,
    }), 0.20)
    machine.reset_response(True, _receipt(), 0.21)
    machine.command("/cmd_vel_sim", False, 0.215)
    machine.collision_event(False, 0.215)
    machine.gate_status(_gate("released:x", held=False), 0.22)
    assert machine.stop_reason.startswith("reset_ground_truth_coverage_failed:")


def test_reset_drift_remains_bounded_during_quiet():
    """Post-release quiet time remains part of landing stability."""
    machine = _reset_to_quiet()
    machine.ground_truth(0.48, -5.35, 0.5)
    assert machine.stop_reason == "reset_landing_drift_exceeded"


def test_old_route_output_must_be_silent_for_quiet_window():
    """Old-epoch route output after its terminal pair is forbidden."""
    machine = _reset_to_quiet()
    machine.route_output("goal_update", 0.5)
    assert machine.stop_reason == "old_output_after_retirement_fence:goal_update"


def test_fresh_failure_and_edge_mismatch_stop():
    """Fresh terminal failure or wrong route identity stops."""
    machine = _fresh_wait()
    machine.terminal_bool(False, 1.3)
    assert machine.stop_reason == "fresh_terminal_false"

    machine = _reset_to_quiet()
    machine.tick(1.21)
    machine.fresh_published(1.22)
    machine.canonical(4, [51, 99], 1.23)
    assert machine.stop_reason == "fresh_canonical_edges_mismatch"


def test_fresh_request_must_be_strictly_newer_and_terminals_bind_epoch_reason():
    """Request reuse and loose terminal documents are rejected."""
    machine = _reset_to_quiet()
    machine.tick(1.21)
    machine.fresh_published(1.22)
    machine.canonical(2, [51, 52], 1.23)
    assert machine.stop_reason == "fresh_request_id_not_newer"

    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.135)
    machine.terminal_result(json.dumps({
        "request_id": 2,
        "status": "aborted",
        "reason": "simulation_reset",
        "reset_epoch": 1,
    }), 0.14)
    assert machine.stop_reason == "old_result_contract_mismatch"

    machine = _fresh_wait()
    machine.terminal_result(json.dumps({
        "request_id": 4,
        "status": "succeeded",
        "reason": "ok",
        "reset_epoch": 2,
    }), 1.3)
    assert machine.stop_reason == "fresh_result_contract_mismatch"


def test_release_requires_best_effort_coverage_not_just_no_bad_sample():
    """Missing BEST_EFFORT samples fail closed at release."""
    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.14)
    machine.reset_event(0.15)
    machine.gate_status(_gate("reset_complete", held=True, eligible=2), 0.18)
    machine.gate_status(_gate("released:x", held=False), 0.22)
    assert machine.stop_reason.startswith("hold_cmd_vel_sim_coverage_failed:")


def test_duplicate_reset_event_and_failed_service_stop():
    """Duplicate event races and a disappearing/failed service are terminal."""
    machine = _active()
    machine.reset_call_started(0.13)
    machine.reset_event(0.14)
    machine.reset_event(0.15)
    assert machine.stop_reason == "duplicate_reset_event"

    machine = _active()
    machine.reset_call_started(0.13)
    machine.reset_response(False, "service unavailable", 0.14)
    assert machine.stop_reason == "reset_service_failed"
    assert machine.reset_call_detail["status"] == "response_failed"


def test_fresh_edges_are_always_exact_even_when_cli_list_is_explicit():
    """There is no support-equivalent escape hatch for fresh route identity."""
    machine = _reset_to_quiet(ProbeMachine(ProbeConfig(fresh_edges=(501, 502))))
    machine.tick(1.21)
    machine.fresh_published(1.22)
    machine.canonical(4, [501, 502], 1.23)
    assert machine.phase == "WAIT_SUCCESS"
    assert machine.fresh_edges_equivalence == "canonical_exact"


def test_fresh_success_then_four_chain_postzero_passes():
    """A complete fresh success and four zero tails can pass."""
    machine = _fresh_wait()
    machine.ground_truth(0.69, -3.98, 1.3)
    machine.terminal_bool(True, 1.31)
    machine.terminal_result(json.dumps({
        "request_id": 4,
        "status": "succeeded",
        "reason": "final_goal_distance_confirmed",
        "reset_epoch": 2,
    }), 1.32)
    assert machine.phase == "POSTZERO"
    for topic in COMMAND_TOPICS:
        for stamp in (1.35, 1.55, 1.75, 1.95, 2.15, 2.32):
            machine.command(topic, False, stamp)
    machine.tick(2.33)
    assert machine.phase == "PROVISIONAL_COMPLETE"
    assert machine.document()["verdict"] == "PROVISIONAL_PASS_REQUIRES_BAG_ORDER"


def test_postzero_nonzero_or_missing_chain_stops():
    """Any nonzero or missing final command stream stops."""
    machine = _fresh_wait()
    machine.ground_truth(0.685, -3.975535, 1.3)
    machine.terminal_bool(True, 1.31)
    machine.terminal_result(json.dumps({
        "request_id": 4, "status": "succeeded",
        "reason": "final_goal_distance_confirmed",
        "reset_epoch": 2,
    }), 1.32)
    for topic in COMMAND_TOPICS[:-1]:
        machine.command(topic, topic == "/cmd_vel", 1.4)
    machine.tick(2.33)
    assert machine.phase == "STOP"
    assert machine.stop_reason.startswith("postzero_contract_failed:")


def _snapshot(*, route_subscribers=None, cmd_publisher="/collision_monitor"):
    def endpoints(*names):
        return [
            {"node": name, "node_name": name.rsplit("/", 1)[-1],
             "node_namespace": "/", "gid": f"gid-{index}"}
            for index, name in enumerate(names)
        ]

    return {"topics": {
        "/cmd_vel": {
            "publishers": endpoints(cmd_publisher), "subscriptions": [],
        },
        "/cmd_vel_sim": {
            "publishers": endpoints("/isaac_navigation_sim"),
            "subscriptions": [],
        },
        "/bio_nav/route_goal": {
            "publishers": endpoints("/v6_active_reset_probe"),
            "subscriptions": endpoints(*(
                route_subscribers or
                ("/bio_nav_route_coordinator", "/rosbag2_recorder")
            )),
        },
    }}


def test_topology_identity_is_exact_and_changes_stop_at_checkpoint():
    """Counts without the expected identities cannot authorize a side effect."""
    expected = ("/bio_nav_route_coordinator", "/rosbag2_recorder")
    assert not validate_topology_snapshot(_snapshot(), expected)
    errors = validate_topology_snapshot(
        _snapshot(cmd_publisher="/unexpected_collision_monitor"), expected
    )
    assert errors and "/cmd_vel:publishers" in errors[0]

    machine = ProbeMachine()
    baseline = _snapshot()
    machine.topology_checked("prepublish", baseline, [], 0.0)
    machine.topology_checked("pre_reset", baseline, [], 0.1)
    changed = _snapshot(route_subscribers=("/bio_nav_route_coordinator",))
    machine.topology_checked("post_release", changed, [], 0.2)
    assert machine.stop_reason == "topology_changed:post_release"


def test_cli_has_exact_identity_defaults_and_no_support_equivalent_escape(tmp_path):
    """Attempt5 defaults are named identities and exact edge equality only."""
    args = _arguments(["--output", str(tmp_path / "probe.json")])
    assert args.expected_route_subscribers == (
        "/bio_nav_route_coordinator", "/rosbag2_recorder"
    )
    assert args.fresh_edges == (51, 52)
    with pytest.raises(SystemExit):
        _arguments([
            "--output", str(tmp_path / "probe.json"),
            "--allow-fresh-support-equivalent",
        ])


def test_entrypoint_exception_writes_atomic_terminal_stop(tmp_path, monkeypatch):
    """An adapter failure becomes a machine-readable STOP at the requested path."""
    output = tmp_path / "probe.json"

    def fail(_args):
        raise RuntimeError("injected")

    monkeypatch.setattr(probe_module, "_run_ros", fail)
    assert probe_module.main(["--output", str(output)]) == 20
    document = json.loads(output.read_text(encoding="utf-8"))
    assert document["phase"] == "STOP"
    assert document["verdict"] == "STOP"
    assert document["stop_reason"] == "entrypoint_exception:RuntimeError:injected"


@pytest.mark.parametrize(
    ("source", "kwargs", "reason"),
    (
        ("ground_truth", {"x": math.nan, "y": 0.0}, "ground_truth_sample_nonfinite"),
        ("odometry", {"x": 0.0, "y": math.inf}, "odometry_sample_nonfinite"),
        (
            "ground_truth",
            {"x": 0.0, "y": 0.0, "quaternion": (0.0, 0.0, math.nan, 1.0)},
            "ground_truth_sample_nonfinite",
        ),
        (
            "odometry",
            {"x": 0.0, "y": 0.0, "source_stamp_s": math.inf},
            "odometry_sample_nonfinite",
        ),
    ),
)
def test_pose_nonfinite_values_stop(source, kwargs, reason):
    """Every non-finite pose field is a terminal contract violation."""
    machine = ProbeMachine()
    getattr(machine, source)(now=0.0, **kwargs)
    assert machine.phase == "STOP"
    assert machine.stop_reason == reason


def test_pose_frames_and_official_odom_topic_are_exact():
    """Only the official topics and normalized map/odom frames are accepted."""
    assert GROUND_TRUTH_TOPIC == "/ground_truth/odom"
    assert ODOMETRY_TOPIC == "/odom"
    machine = ProbeMachine()
    machine.ground_truth(0.0, 0.0, 0.0, frame_id="/map")
    machine.odometry(0.0, 0.0, 0.1, frame_id="/odom")
    assert machine.pose_contract["ground_truth"]["normalized_frame"] == "map"
    assert machine.pose_contract["odometry"]["normalized_frame"] == "odom"

    wrong = ProbeMachine()
    wrong.ground_truth(0.0, 0.0, 0.0, frame_id="odom")
    assert wrong.stop_reason.startswith("ground_truth_frame_mismatch:")
    wrong = ProbeMachine()
    wrong.odometry(0.0, 0.0, 0.0, frame_id="map")
    assert wrong.stop_reason.startswith("odometry_frame_mismatch:")


def test_nonfinite_receive_time_and_twist_are_not_zero():
    """NaN/Inf receive times or Twist fields cannot masquerade as zero."""
    machine = ProbeMachine()
    machine.ground_truth(0.0, 0.0, math.nan)
    assert machine.stop_reason == "ground_truth_time_nonfinite"

    vector = SimpleNamespace(
        linear=SimpleNamespace(x=0.0, y=0.0, z=math.nan),
        angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
    )
    with pytest.raises(ValueError, match="twist_nonfinite"):
        _twist_values(vector)
    machine = ProbeMachine()
    machine.command(
        "/cmd_vel_sim", False, 0.0,
        values=(0.0, 0.0, math.inf, 0.0, 0.0, 0.0),
    )
    assert machine.stop_reason == "twist_nonfinite:/cmd_vel_sim"


@pytest.mark.parametrize("kind", ("canonical", "progress", "lookahead", "navigate"))
def test_pre_hold_and_post_hold_pre_retirement_outputs_are_inflight(kind):
    """HOLD starts a bounded interval; the exact terminal pair is the fence."""
    machine = _active()
    machine.reset_call_started(0.13)
    if kind == "canonical":
        machine.canonical(2, [51, 52, 30], 0.131)
    elif kind == "progress":
        machine.progress(2, 0.131)
    elif kind == "lookahead":
        machine.route_output("lookahead", 0.131)
    else:
        machine.navigate_intent(0.131)
    assert machine.phase == "OBSERVE_HOLD_ABORT"
    assert machine.pre_hold_inflight_outputs[-1]["received_monotonic_s"] == 0.131
    assert machine.pre_hold_inflight_outputs[-1]["type"] == (
        "navigate_intent" if kind == "navigate" else kind
    )

    machine.gate_status(_gate("hold", held=True), 0.14)
    if kind == "canonical":
        machine.canonical(2, [51, 52, 30], 0.141)
    elif kind == "progress":
        machine.progress(2, 0.141)
    elif kind == "lookahead":
        machine.route_output("lookahead", 0.141)
    else:
        machine.navigate_intent(0.141)
    assert machine.phase == "OBSERVE_HOLD_ABORT"
    expected_kind = "navigate_intent" if kind == "navigate" else kind
    assert machine.post_hold_pre_retirement_inflight[-1][
        "received_monotonic_s"
    ] == 0.141
    assert machine.post_hold_pre_retirement_inflight[-1]["type"] == expected_kind
    assert machine.post_hold_pre_retirement_counts[expected_kind] == 1


def test_attempt6_triplet_then_exact_terminal_pair_reaches_fresh_route():
    """Attempt6's bounded triplet retires at the abort pair, then quiet passes."""
    machine = _reset_to_quiet()
    inflight = machine.document()["old"]["post_hold_pre_retirement_inflight"]
    assert inflight["total"] == 3
    assert inflight["counts_by_type"] == {
        "progress": 1, "lookahead": 1, "navigate_intent": 1,
    }
    assert machine.timestamps["coordinator_retirement_fence"] == pytest.approx(0.20)
    assert machine.document()["old"]["coordinator_retirement_fence"][
        "hold_to_fence_s"
    ] == pytest.approx(0.06)
    machine.tick(1.20)
    assert machine.phase == "PUBLISH_FRESH_ONCE"


def test_output_after_terminal_pair_is_fail_stop():
    """Any old callback after pair completion crosses the retirement fence."""
    machine = _reset_to_quiet()
    machine.progress(2, 0.201)
    assert machine.stop_reason == "old_output_after_retirement_fence:progress"
    assert machine.post_retirement_outputs[-1]["request_id"] == 2


def test_old_terminal_pair_timeout_and_late_inflight_are_fail_stop():
    """The HOLD-to-retirement interval cannot exceed 0.25 seconds."""
    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.14)
    machine.tick(0.391)
    assert machine.stop_reason == "old_terminal_pair_timeout"

    late = _active()
    late.reset_call_started(0.13)
    late.gate_status(_gate("hold", held=True), 0.14)
    late.route_output("lookahead", 0.391)
    assert late.stop_reason == (
        "old_inflight_after_retirement_deadline:lookahead"
    )


def test_multiple_inflight_outputs_are_bounded_and_wrong_id_stops():
    """Multiple callbacks are observable inside the window, never for another ID."""
    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.14)
    machine.progress(2, 0.15)
    machine.progress(2, 0.20)
    machine.canonical(2, [51, 52, 30], 0.30)
    assert machine.phase == "OBSERVE_HOLD_ABORT"
    assert machine.post_hold_pre_retirement_counts == {
        "progress": 2, "canonical": 1,
    }

    wrong = _active()
    wrong.reset_call_started(0.13)
    wrong.gate_status(_gate("hold", held=True), 0.14)
    wrong.progress(99, 0.15)
    assert wrong.stop_reason == "old_inflight_request_id_mismatch:progress"


def test_old_terminal_wrong_pair_late_pair_and_duplicate_stop():
    """Abort pair identity, deadline, and exactly-once delivery are strict."""
    wrong = _active()
    wrong.reset_call_started(0.13)
    wrong.gate_status(_gate("hold", held=True), 0.14)
    wrong.terminal_bool(True, 0.15)
    assert wrong.stop_reason == "old_terminal_true"

    late = _active()
    late.reset_call_started(0.13)
    late.gate_status(_gate("hold", held=True), 0.14)
    late.terminal_bool(False, 0.391)
    assert late.stop_reason == "old_terminal_after_retirement_deadline:bool"

    duplicate = _active()
    duplicate.reset_call_started(0.13)
    duplicate.gate_status(_gate("hold", held=True), 0.14)
    duplicate.terminal_bool(False, 0.15)
    duplicate.terminal_bool(False, 0.16)
    assert duplicate.stop_reason == "duplicate_old_terminal_bool"

    result_first = _active()
    result_first.reset_call_started(0.13)
    result_first.gate_status(_gate("hold", held=True), 0.14)
    result_first.terminal_result(json.dumps({
        "request_id": 2,
        "status": "aborted",
        "reason": "simulation_reset",
        "reset_epoch": 2,
    }), 0.15)
    result_first.terminal_bool(False, 0.16)
    assert result_first.timestamps["coordinator_retirement_fence"] == 0.16
    result_first.terminal_result(json.dumps({
        "request_id": 2,
        "status": "aborted",
        "reason": "simulation_reset",
        "reset_epoch": 2,
    }), 0.17)
    assert result_first.stop_reason == "duplicate_or_invalid_old_result"


def test_dispatch_to_hold_latency_is_bounded_and_reported():
    """HOLD must arrive within 0.5 s and expose both receive boundaries."""
    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.63)
    assert machine.phase == "OBSERVE_HOLD_ABORT"
    assert machine.reset_call_detail["dispatch_to_hold_s"] == pytest.approx(0.5)
    assert machine.timestamps["gate_hold"] == pytest.approx(0.63)

    late = _active()
    late.reset_call_started(0.13)
    late.gate_status(_gate("hold", held=True), 0.631)
    assert late.stop_reason == "gate_hold_after_reset_dispatch_too_late"
    assert late.reset_call_detail["dispatch_to_hold_s"] == pytest.approx(0.501)

    missing = _active()
    missing.reset_call_started(0.13)
    missing.tick(0.631)
    assert missing.stop_reason == "gate_hold_observation_timeout"


def test_gate_baseline_hidden_generation_and_exact_episode_sequence():
    """The retained baseline and active generation are strict and contiguous."""
    machine = ProbeMachine()
    machine.gate_status(_gate("hold", held=True, generation=3), 0.0)
    assert machine.stop_reason == "gate_generation_before_reset_call"

    machine = ProbeMachine()
    baseline = _gate("released:startup", held=False, generation=1)
    machine.gate_status(baseline, 0.0)
    machine.gate_status(baseline, 0.1)
    assert machine.stop_reason == "gate_baseline_duplicate"

    machine = _active()
    machine.reset_call_started(0.13)
    machine.gate_status(_gate("hold", held=True), 0.14)
    machine.gate_status(_gate("reset_complete", held=True, eligible=2), 0.15)
    # An older retained generation is not ignorable once dispatch has begun.
    machine.gate_status(baseline, 0.16)
    assert machine.stop_reason == "gate_generation_regressed_unexpectedly"


def test_gate_extra_after_release_stops_even_after_provisional_completion():
    """Gate monitoring remains fail-stop through provisional completion."""
    machine = _provisional()
    machine.gate_status(_gate("released:duplicate", held=False), 2.34)
    assert machine.phase == "STOP"
    assert machine.stop_reason == "gate_status_extra_after_release"

    machine = _provisional()
    machine.command("/cmd_vel_sim", True, 2.34)
    assert machine.stop_reason == (
        "command_nonzero_after_provisional:/cmd_vel_sim"
    )


def test_slow_topology_consumes_delay_and_prevents_trigger_dispatch():
    """Synchronous topology latency is charged before call_async."""
    machine = _active()
    clock = {"now": 0.13}
    calls = []

    def topology():
        clock["now"] = 0.621
        return True

    future = dispatch_reset_once(
        machine,
        check_topology=topology,
        service_ready=lambda: True,
        call_async=lambda: calls.append("called"),
        done_callback=lambda _future: None,
        monotonic=lambda: clock["now"],
    )
    assert future is None
    assert calls == []
    assert machine.stop_reason == "reset_call_after_active_ready_too_late"


def test_normal_dispatch_records_actual_boundary_and_done_callback():
    """A timely dispatch records the actual call boundary and callback."""
    machine = _active()

    class Future:
        callback = None

        def add_done_callback(self, callback):
            self.callback = callback

    future = Future()
    result = dispatch_reset_once(
        machine,
        check_topology=lambda: True,
        service_ready=lambda: True,
        call_async=lambda: future,
        done_callback=lambda _future: None,
        monotonic=lambda: 0.13,
    )
    assert result is future
    assert future.callback is not None
    assert machine.timestamps["reset_call"] == 0.13

    failed = _active()
    dispatch_reset_once(
        failed,
        check_topology=lambda: True,
        service_ready=lambda: True,
        call_async=lambda: (_ for _ in ()).throw(RuntimeError("dispatch injected")),
        done_callback=lambda _future: None,
        monotonic=lambda: 0.13,
    )
    assert failed.stop_reason.startswith("reset_dispatch_exception:")
    assert failed.reset_call_detail["status"] == "dispatch_exception"
    assert failed.reset_call_detail["exception_type"] == "RuntimeError"


def test_provisional_requires_teardown_and_final_atomic_receipt(tmp_path):
    """Only completed teardown promotes provisional to PASS_REQUIRES_BAG."""
    machine = _provisional()

    class Node:
        def __init__(self):
            self.machine = machine

        def destroy_node(self):
            return None

    ros = SimpleNamespace(shutdown=lambda: None)
    lifecycle = teardown_ros_node(Node(), ros, monotonic=lambda: 3.0)
    status, document = finalize_probe_output(
        machine, tmp_path / "probe.json", lifecycle
    )
    assert status == 0
    assert document["phase"] == "PROVISIONAL_COMPLETE"
    assert document["finalized"] is True
    assert document["verdict"] == "PASS_REQUIRES_BAG"
    assert json.loads((tmp_path / "probe.json").read_text())["verdict"] == (
        "PASS_REQUIRES_BAG"
    )


@pytest.mark.parametrize("failure", ("destroy", "shutdown"))
def test_teardown_exception_forces_nonzero_final_stop(tmp_path, failure):
    """Destroy or shutdown failures force a persisted nonzero STOP."""
    machine = _provisional()

    class Node:
        def __init__(self):
            self.machine = machine

        def destroy_node(self):
            if failure == "destroy":
                raise RuntimeError("destroy injected")

    def shutdown():
        if failure == "shutdown":
            raise RuntimeError("shutdown injected")

    lifecycle = teardown_ros_node(
        Node(), SimpleNamespace(shutdown=shutdown), monotonic=lambda: 3.0
    )
    status, document = finalize_probe_output(
        machine, tmp_path / "probe.json", lifecycle
    )
    assert status == 20
    assert document["phase"] == "STOP"
    assert document["verdict"] == "STOP"


def test_final_persist_exception_is_nonzero_and_writes_emergency_stop(tmp_path):
    """A failed final write cannot return zero and emits emergency STOP."""
    machine = _provisional()
    output = tmp_path / "probe.json"

    def fail(_path, _document):
        raise OSError("write injected")

    status, document = finalize_probe_output(
        machine,
        output,
        {"node_destroyed": True, "rclpy_shutdown": True},
        writer=fail,
    )
    assert status == 20
    assert document["phase"] == "STOP"
    assert document["stop_reason"].startswith("final_json_write_exception:")
    assert json.loads(output.read_text())["verdict"] == "STOP"
    assert json.loads(
        (tmp_path / "probe.json.emergency-stop.json").read_text()
    )["verdict"] == "STOP"
