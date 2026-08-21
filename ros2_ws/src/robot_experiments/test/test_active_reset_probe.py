"""Deterministic tests for the active-reset probe state machine."""

import json

import pytest

import robot_experiments.active_reset_probe as probe_module
from robot_experiments.active_reset_probe import (
    COMMAND_TOPICS,
    ProbeConfig,
    ProbeMachine,
    _arguments,
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
    assert machine.stop_reason == "old_route_output_not_quiet"


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
    assert machine.phase == "PASS"
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
