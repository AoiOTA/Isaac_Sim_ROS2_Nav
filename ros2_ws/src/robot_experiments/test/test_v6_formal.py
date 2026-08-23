from collections import Counter, deque
import json
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from robot_experiments.v6_formal import (
    CAPTURE_SCHEMA,
    DISPATCH_SUBSCRIPTION_TOPICS,
    DynamicActionLedger,
    ENGINEERING_PILOT,
    EpisodeGuard,
    NOT_QUALIFIED,
    ReadinessFacts,
    V6ContractError,
    V6FormalNode,
    append_evidence_jsonl,
    authorize_manifest,
    cli,
    load_manifest,
)


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config"
MANIFESTS = {
    "kujiale_static": CONFIG / "v6_final_kujiale_static.yaml",
    "kujiale_dynamic": CONFIG / "v6_final_kujiale_dynamic.yaml",
    "kujiale_appearance": CONFIG / "v6_final_kujiale_appearance.yaml",
    "rivermark_static": CONFIG / "v6_final_rivermark_static.yaml",
    "rivermark_dynamic": CONFIG / "v6_final_rivermark_dynamic.yaml",
    "rivermark_appearance": CONFIG / "v6_final_rivermark_appearance.yaml",
}


def ready_facts() -> ReadinessFacts:
    return ReadinessFacts(**{name: True for name in ReadinessFacts.__dataclass_fields__})


def ready_guard(*legs: str) -> EpisodeGuard:
    guard = EpisodeGuard(mission_leg_ids=legs)
    guard.arm_reset(
        ready_facts(), 0, "session-0",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_receipt_generation(1)
    guard.record_reset_event()
    guard.record_bridge(1, "session-1", True)
    guard.record_startup_consensus(True)
    guard.record_initialpose(100)
    guard.record_amcl(101)
    guard.record_bridge(2, "session-2", False)
    guard.record_b5_diagnostic(
        state="normal", recovery_result="succeeded", seed_confirmation="succeeded",
        candidate_generation="epoch=2,session=session-2",
    )
    guard.record_prior(
        2, "session-2", trusted_write=True, module2_healthy=True,
        observation_valid=True, input_healthy=True,
    )
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_gate_status(1, True)
    guard.record_reset_gate_status(1, False)
    return guard


def gate_held_guard(*legs: str) -> EpisodeGuard:
    """Full readiness chain except the ResetStopGate release (live R5 race)."""
    guard = EpisodeGuard(mission_leg_ids=legs)
    guard.arm_reset(
        ready_facts(), 0, "session-0",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_receipt_generation(1)
    guard.record_reset_event()
    guard.record_bridge(1, "session-1", True)
    guard.record_startup_consensus(True)
    guard.record_initialpose(100)
    guard.record_amcl(101)
    guard.record_bridge(2, "session-2", False)
    guard.record_b5_diagnostic(
        state="normal", recovery_result="succeeded", seed_confirmation="succeeded",
        candidate_generation="epoch=2,session=session-2",
    )
    guard.record_prior(
        2, "session-2", trusted_write=True, module2_healthy=True,
        observation_valid=True, input_healthy=True,
    )
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_gate_status(1, True)
    return guard


def test_dispatcher_runtime_topic_firewall_is_zero_gt():
    assert DISPATCH_SUBSCRIPTION_TOPICS
    assert not [topic for topic in DISPATCH_SUBSCRIPTION_TOPICS if topic.startswith("/ground_truth/")]
    source = (PACKAGE / "robot_experiments" / "v6_formal.py").read_text()
    assert "create_subscription" in source
    assert '"/ground_truth/' not in source


def test_dispatcher_uses_route_coordinator_primary_goal_not_nav_action():
    source = (PACKAGE / "robot_experiments" / "v6_formal.py").read_text()
    assert 'PoseStamped, "/bio_nav/route_goal"' in source
    assert '"/bio_nav/route_goal_complete"' in source
    assert '"/bio_nav/route_goal_result"' in source
    assert '"/bio_nav/route_progress"' in source
    assert "NavigateToPose" not in source
    assert "navigate_to_pose" not in source


def test_reset_is_exactly_once_and_unknown_response_never_retries():
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(), 0, "session-0",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    guard.record_reset_call()
    guard.record_reset_response(None)
    assert guard.state == "STOP"
    assert guard.stop_reason == "reset_response_unknown"
    with pytest.raises(V6ContractError, match="reset_retry_forbidden"):
        guard.record_reset_call()
    assert guard.reset_calls == 1


def test_second_reset_event_is_immediate_stop():
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(), 0, "session-0",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_event()
    guard.record_reset_event()
    assert guard.state == "STOP"
    assert guard.stop_reason == "second_reset_event"


def test_multileg_goals_publish_in_order_and_stop_on_failure():
    guard = ready_guard("G2", "G3", "G4")
    guard.record_goal_publication("G2")
    guard.record_route_progress()
    guard.record_route_completion(True)
    assert guard.state == "LEG_SUCCEEDED"
    guard.record_goal_publication("G3")
    guard.record_route_progress()
    guard.record_route_completion(False)
    assert guard.state == "FAILED"
    assert guard.completed_leg_ids == ["G2"]
    with pytest.raises(V6ContractError, match="not_authorized"):
        guard.record_goal_publication("G4")
    assert guard.goal_publications == 2


def test_multileg_order_mismatch_stops_before_publish():
    guard = ready_guard("G2", "G3")
    with pytest.raises(V6ContractError, match="mission_leg_order"):
        guard.record_goal_publication("G3")
    assert guard.goal_publications == 0


def test_goal_waits_for_reset_gate_release_of_receipt_generation():
    guard = gate_held_guard("G2")
    assert not guard.reset_gate_released
    assert not guard.goal_ready
    guard.record_reset_gate_status(1, False)
    assert guard.reset_gate_released
    assert guard.goal_ready
    guard.record_goal_publication("G2")
    assert guard.state == "NAVIGATING"
    # The latched fact is not regressed by duplicate gate traffic.
    guard.record_reset_gate_status(1, False)
    assert guard.state == "NAVIGATING"


def test_goal_publication_refused_while_reset_gate_holds():
    guard = gate_held_guard("G2")
    with pytest.raises(V6ContractError, match="not_authorized"):
        guard.record_goal_publication("G2")
    assert guard.goal_publications == 0


def test_stale_reset_gate_release_does_not_arm_goal_readiness():
    guard = gate_held_guard("G2")
    guard.record_reset_gate_status(0, False)  # pre-reset generation release
    assert not guard.goal_ready
    guard.record_reset_gate_status(7, False)  # unknown future generation
    assert not guard.goal_ready
    with pytest.raises(V6ContractError, match="not_authorized"):
        guard.record_goal_publication("G2")
    assert guard.goal_publications == 0


def test_reset_gate_release_before_receipt_binding_is_not_lost():
    guard = EpisodeGuard(mission_leg_ids=("G2",))
    guard.arm_reset(
        ready_facts(), 0, "session-0",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_gate_status(1, False)
    assert not guard.reset_gate_released  # no receipt generation bound yet
    guard.record_reset_receipt_generation(1)
    assert guard.reset_gate_released


def steady_state_streams(guard: EpisodeGuard) -> None:
    """Trusted prior / AMCL / B5 traffic that keeps flowing during navigation."""
    guard.record_prior(
        2, "session-2", trusted_write=True, module2_healthy=True,
        observation_valid=True, input_healthy=True,
    )
    guard.record_amcl(102)
    guard.record_b5_diagnostic(
        state="normal", recovery_result="succeeded", seed_confirmation="succeeded",
        candidate_generation="epoch=2,session=session-2",
    )


def test_navigating_not_regressed_by_steady_state_streams():
    guard = ready_guard("G2", "G3")
    guard.record_goal_publication("G2")
    assert guard.state == "NAVIGATING"
    steady_state_streams(guard)
    assert guard.state == "NAVIGATING"
    guard.record_route_progress()
    steady_state_streams(guard)
    guard.record_route_completion(True)
    assert guard.route_progress_messages == 1
    assert guard.route_completion_messages == 1
    assert guard.state == "LEG_SUCCEEDED"
    assert guard.completed_leg_ids == ["G2"]


def test_leg_succeeded_not_regressed_by_steady_state_streams():
    guard = ready_guard("G2", "G3")
    guard.record_goal_publication("G2")
    guard.record_route_progress()
    guard.record_route_completion(True)
    assert guard.state == "LEG_SUCCEEDED"
    steady_state_streams(guard)
    assert guard.state == "LEG_SUCCEEDED"
    guard.record_goal_publication("G3")
    assert guard.state == "NAVIGATING"


def test_wrong_bridge_epoch_stops_before_goal():
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(), 0, "session-0",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_event()
    guard.record_bridge(3, "session-3", False)
    assert guard.stop_reason == "bridge_epoch_mismatch:3!=2"


def test_active_b5_readiness_requires_negative_window_not_prior_or_route():
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(), 0, "session-0",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    assert guard.state == "RESET_ARMED"


@pytest.mark.parametrize("name", ["prior", "candidate", "initialpose", "route"])
def test_active_b5_rejects_old_pre_reset_positive_preconditions(name):
    counts = {"prior": 0, "candidate": 0, "initialpose": 0, "route": 0}
    counts[name] = 1
    with pytest.raises(V6ContractError, match="negative window violated"):
        EpisodeGuard().arm_reset(
            ready_facts(), 0, "session-0", pre_reset_counts=counts
        )


def test_arm_reset_discovers_warm_baseline_and_rolls_epochs():
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(), 3, "session-3",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    assert guard.state == "RESET_ARMED"
    assert guard.bridge_epoch_baseline == 3
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_event()
    # Baseline diagnostics keep flowing and are ignored, not fatal.
    guard.record_bridge(3, "session-3", False)
    assert guard.stop_reason == ""
    guard.record_bridge(4, "session-4", True)
    assert guard.physical_epoch == 4
    guard.record_startup_consensus(True)
    guard.record_initialpose(100)
    guard.record_amcl(101)
    guard.record_bridge(5, "session-5", False)
    assert guard.bootstrap_epoch == 5
    guard.record_b5_diagnostic(
        state="normal", recovery_result="succeeded", seed_confirmation="succeeded",
        candidate_generation="epoch=5,session=session-5",
    )
    assert guard.b5_generation_witnessed


def test_arm_reset_rejects_missing_epoch_or_active_goal():
    with pytest.raises(V6ContractError, match="bridge epoch baseline"):
        EpisodeGuard().arm_reset(
            ready_facts(), None, "session-0",
            pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
        )
    guard = EpisodeGuard()
    guard.goal_publications = 1
    with pytest.raises(V6ContractError, match="reset_with_active_goal_forbidden"):
        guard.arm_reset(
            ready_facts(), 0, "session-0",
            pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
        )


def _bare_node():
    node = V6FormalNode.__new__(V6FormalNode)
    node.facts = ReadinessFacts()
    node.latest_bridge_epoch = None
    node.latest_bridge_session = ""
    node.pre_reset_counts = {"prior": 0, "candidate": 0, "initialpose": 0, "route": 0}
    node._cmd_window = deque()
    node._odom_window = deque()
    node.post_reset_odom_xy = []
    node.guard = EpisodeGuard()
    node._capture = lambda topic, message: None
    return node


def _twist(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(
        linear=SimpleNamespace(x=x, y=y), angular=SimpleNamespace(z=z)
    )


def test_pre_reset_still_requires_zero_commands_and_bounded_odom_span():
    node = _bare_node()
    now = time.monotonic()
    assert not node._pre_reset_still()  # no odom evidence at all
    node._odom_window.extend([
        (now - 0.8, 1.0, 2.0), (now - 0.4, 1.02, 2.01), (now - 0.1, 1.01, 2.0),
    ])
    assert node._pre_reset_still()
    node._cmd_window.append((now - 0.2, True))
    assert not node._pre_reset_still()
    node._cmd_window.clear()
    node._cmd_window.append((now - 2.0, True))  # stale evidence expires
    assert node._pre_reset_still()
    node._odom_window.append((now - 0.1, 1.5, 2.0))
    assert not node._pre_reset_still()


def test_track_command_stops_on_post_reset_pre_goal_motion():
    node = _bare_node()
    node._track_command("/cmd_vel_sim", _twist(x=0.2))
    assert node.guard.stop_reason == ""  # pre-reset commands only feed stillness
    node.guard.reset_calls = 1
    node._track_command("/cmd_vel_sim", _twist(x=0.2))
    assert node.guard.stop_reason == "post_reset_command_nonzero:/cmd_vel_sim"
    node.guard.goal_publications = 1
    node._track_command("/cmd_vel_sim", _twist(x=0.2))  # own goal: motion legal
    assert node.guard.stop_reason == "post_reset_command_nonzero:/cmd_vel_sim"


def test_track_route_signal_counts_pre_reset_and_stops_stale_post_reset():
    node = _bare_node()
    node._track_route_signal("route_progress")
    assert node.pre_reset_counts["route"] == 1
    assert node.guard.stop_reason == ""
    node.guard.reset_calls = 1
    node._track_route_signal("route_goal_complete")
    assert node.guard.stop_reason == "stale_route_goal_complete_after_reset"


def test_reset_gate_status_callback_scopes_to_reset_and_validates():
    node = _bare_node()
    released = SimpleNamespace(data=json.dumps({
        "generation": 1,
        "held": False,
        "eligible_generation": None,
        "reason": "released:activation_gate",
    }))
    node._reset_gate_status(released)
    assert node.guard.reset_gate_released_generation is None  # pre-reset ignored
    node.guard.reset_calls = 1
    node._reset_gate_status(released)
    assert node.guard.reset_gate_released_generation == 1
    node._reset_gate_status(SimpleNamespace(data="not json"))
    assert node.guard.stop_reason == "reset_gate_status_invalid"
    node = _bare_node()
    node.guard.reset_calls = 1
    node._reset_gate_status(SimpleNamespace(data=json.dumps({
        "generation": "1", "held": False,
    })))
    assert node.guard.stop_reason == "reset_gate_status_invalid"


def test_post_reset_odom_landing_and_span_contract():
    node = _bare_node()
    node._check_post_reset_odom()
    assert node.guard.stop_reason == "post_reset_odom_missing"

    node = _bare_node()
    # The first sample may straddle the reset boundary and is skipped.
    node.post_reset_odom_xy = [(9.9, 9.9), (0.01, 0.0), (0.02, 0.01), (0.0, 0.01)]
    node._check_post_reset_odom()
    assert node.guard.stop_reason == ""

    node = _bare_node()
    node.post_reset_odom_xy = [(0.0, 0.0), (0.5, 0.0), (0.5, 0.0)]
    node._check_post_reset_odom()
    assert node.guard.stop_reason.startswith("post_reset_odom_landing:")

    node = _bare_node()
    node.post_reset_odom_xy = [(0.0, 0.0), (0.01, 0.0), (0.30, 0.0)]
    node._check_post_reset_odom()
    assert node.guard.stop_reason.startswith("post_reset_odom_span:")


def test_b5_warm_baseline_matching_uses_bridge_epoch_and_session():
    node = _bare_node()
    assert not node._b5_matches_bridge_baseline("epoch=3,session=s3,map=v1")
    node.latest_bridge_epoch = 3
    node.latest_bridge_session = "s3"
    assert node._b5_matches_bridge_baseline("epoch=3,session=s3,map=v1,tile=t@1,graph=2")
    assert not node._b5_matches_bridge_baseline("epoch=4,session=s3")
    assert not node._b5_matches_bridge_baseline("epoch=3,session=other")
    assert not node._b5_matches_bridge_baseline("waiting_after_physical_reset")


def test_publisher_ownership_requires_sole_command_and_pose_publishers():
    node = _bare_node()
    counts = {"/odom": 1, "/cmd_vel": 1, "/cmd_vel_sim": 1, "/amcl_pose": 1}
    node.node = SimpleNamespace(count_publishers=lambda topic: counts[topic])
    assert node._publisher_ownership_violations() == ()
    counts["/cmd_vel_sim"] = 2
    assert node._publisher_ownership_violations() == ("/cmd_vel_sim=2",)
    counts["/cmd_vel_sim"] = 0
    assert node._publisher_ownership_violations() == ("/cmd_vel_sim=0",)


def test_b5_physical_epoch_seed_confirmation_bootstrap_rollover_sequence():
    guard = ready_guard()
    assert guard.physical_epoch == 1
    assert guard.bootstrap_epoch == 2
    assert guard.physical_session == "session-1"
    assert guard.bootstrap_session == "session-2"
    assert guard.goal_ready


def test_b5_path_never_depends_on_isaac_localization_seeded():
    source = (PACKAGE / "robot_experiments" / "v6_formal.py").read_text()
    assert "/simulation/localization_seeded" not in source


def test_initialpose_burst_and_scan_stamped_amcl_straddlers():
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(), 0, "session-0",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    guard.record_reset_call(); guard.record_reset_response(True); guard.record_reset_event()
    guard.record_bridge(1, "session-1", True)
    guard.record_startup_consensus(True)
    guard.record_initialpose(100)
    # AMCL stamps poses with the source scan time; samples straddling the
    # seed are stale-stamped stragglers and are skipped, not violations.
    guard.record_amcl(100)
    assert guard.stop_reason == ""
    assert not guard.post_initialpose_amcl_seen
    guard.record_amcl(101)
    assert guard.post_initialpose_amcl_seen

    # The reseed burst repeats the same calibrated pose inside the reset
    # epoch; repeats are accepted and the first seed stays the anchor.
    second = EpisodeGuard()
    second.arm_reset(
        ready_facts(), 0, "session-0",
        pre_reset_counts={"prior": 0, "candidate": 0, "initialpose": 0, "route": 0},
    )
    second.record_reset_call(); second.record_reset_response(True); second.record_reset_event()
    second.record_bridge(1, "session-1", True)
    second.record_startup_consensus(True)
    second.record_initialpose(100); second.record_initialpose(101)
    assert second.stop_reason == ""
    assert second.initialpose_messages == 2
    assert second.initialpose_stamp_ns == 100


@pytest.mark.parametrize(
    "kwargs, reason",
    [
        ({"trusted_write": True, "module2_healthy": True, "observation_valid": True, "input_healthy": True}, "active_prior_generation_mismatch"),
    ],
)
def test_wrong_active_prior_trust_or_session_stops(kwargs, reason):
    guard = ready_guard()
    guard.state = "WAITING_B5_CONFIRMATION"
    guard.stop_reason = ""
    guard.post_reset_prior_seen = False
    session = "wrong-session"
    guard.record_prior(2, session, **kwargs)
    assert guard.stop_reason == reason


def test_untrusted_intermediate_prior_is_skipped_not_a_stop():
    guard = ready_guard()
    guard.state = "WAITING_B5_CONFIRMATION"
    guard.stop_reason = ""
    guard.post_reset_prior_seen = False
    guard.record_prior(
        2, "session-2",
        trusted_write=False, module2_healthy=False,
        observation_valid=False, input_healthy=True,
    )
    assert guard.stop_reason == ""
    assert not guard.post_reset_prior_seen
    guard.record_prior(
        2, "session-2",
        trusted_write=True, module2_healthy=True,
        observation_valid=True, input_healthy=True,
    )
    assert guard.stop_reason == ""
    assert guard.post_reset_prior_seen


def test_b5_confirmation_failure_and_timeout_stop_before_goal():
    guard = EpisodeGuard()
    guard.record_b5_diagnostic(
        state="lost", recovery_result="seed_confirmation_failed", seed_confirmation="failed"
    )
    assert guard.stop_reason == "b5_seed_confirmation_failed"
    waiting = EpisodeGuard(state="WAITING_B5_CONFIRMATION")
    waiting.stop("post_reset_readiness_timeout")
    assert not waiting.goal_ready


@pytest.mark.parametrize("path", MANIFESTS.values())
def test_all_manifests_are_complete_candidates_but_not_formally_frozen(path):
    manifest = load_manifest(path)
    assert manifest.localization_seed_source == "b5_cognitive"
    assert len(manifest.episodes) == 20
    assert len(manifest.mission_legs) == 5
    assert manifest.frozen is False
    assert manifest.missing_required_values == ()
    assert manifest.estimated_policy == {
        "ekf_profile": "wheel_imu",
        "lidar_odometry_backend": "off",
        "lidar_odometry_validated": False,
        "rf2o_decision": "not_validated_off",
        "imu_calibration_profile": "isaac_v6_calibrated",
    }
    with pytest.raises(V6ContractError, match="scene_contract_frozen is false"):
        authorize_manifest(manifest, mode="formal")
    assert authorize_manifest(manifest, mode="pilot") == NOT_QUALIFIED


def test_candidate_assets_are_bound_and_only_unsupported_posegraph_is_null():
    for name, path in MANIFESTS.items():
        values = load_manifest(path).raw["required_runtime_values"]
        nulls = {key for key, value in values.items() if value is None}
        if name.startswith("rivermark"):
            assert values["scene_asset"] == "/home/lyb/Rivermark/rivermark.usd"
            assert values["posegraph_required"] is False
            assert nulls == {"posegraph_file"}
        else:
            assert values["scene_asset"].endswith(
                "/kujiale_0026/kujiale_0026_A_to_B_door_open.usd"
            )
            assert values["posegraph_required"] is True
            assert not nulls
        assert values["occupancy_map"].endswith(
            "v6_kujiale_isaacgen_v1.yaml"
            if name.startswith("kujiale")
            else "rivermark_selected.yaml"
        )


@pytest.mark.parametrize(
    "name, expected",
    [
        ("kujiale_static", ["G2", "G3", "G4", "G5", "G1"]),
        ("kujiale_dynamic", ["G2", "G3", "G4", "G5", "G1"]),
        ("kujiale_appearance", ["G2", "G3", "G4", "G5", "G1"]),
        ("rivermark_static", ["G1", "G2", "G3", "G4", "G5"]),
        ("rivermark_dynamic", ["G1", "G2", "G3", "G4", "G5"]),
        ("rivermark_appearance", ["G1", "G2", "G3", "G4", "G5"]),
    ],
)
def test_mission_is_full_ordered_route_not_final_only_shortcut(name, expected):
    manifest = load_manifest(MANIFESTS[name])
    assert [leg.goal_id for leg in manifest.mission_legs] == expected
    reset = manifest.reset_pose
    first = manifest.mission_legs[0]
    assert (float(reset["x"]), float(reset["y"])) != (first.x, first.y)
    assert all("goal" not in row for row in manifest.raw["episodes"])


@pytest.mark.parametrize(
    "name, first, last",
    [
        ("kujiale_static", 7201, 7220),
        ("kujiale_dynamic", 7301, 7320),
        ("kujiale_appearance", 7201, 7220),
        ("rivermark_static", 19301, 19320),
        ("rivermark_dynamic", 19401, 19420),
        ("rivermark_appearance", 19501, 19520),
    ],
)
def test_exact_seed_ranges(name, first, last):
    assert [row.seed for row in load_manifest(MANIFESTS[name]).episodes] == list(range(first, last + 1))


@pytest.mark.parametrize(
    "name, case_id, groups",
    [
        ("kujiale_dynamic", "full_route_three_stage", ["G2", "G3", "G1"]),
        ("rivermark_dynamic", "full_route_four_stage", ["G2", "G3", "G4", "G5"]),
    ],
)
def test_dynamic_cases_variants_and_trigger_groups(name, case_id, groups):
    manifest = load_manifest(MANIFESTS[name])
    assert {row.dynamic_case_id for row in manifest.episodes} == {case_id}
    assert Counter(row.variant_id for row in manifest.episodes) == {f"v{i}": 4 for i in range(1, 6)}
    assert [leg.dynamic_trigger_group for leg in manifest.mission_legs if leg.dynamic_trigger_group] == groups


@pytest.mark.parametrize("name", ["kujiale_static", "kujiale_appearance", "rivermark_static", "rivermark_appearance"])
def test_non_dynamic_scenes_never_arm_actor_services(name):
    assert not any(leg.dynamic_trigger_group for leg in load_manifest(MANIFESTS[name]).mission_legs)


@pytest.mark.parametrize("name", ["kujiale_appearance", "rivermark_appearance"])
def test_appearance_profiles_are_four_profiles_five_each(name):
    manifest = load_manifest(MANIFESTS[name])
    assert Counter(row.appearance_profile_id for row in manifest.episodes) == {
        "dim_warm": 5,
        "dim_cool": 5,
        "bright_warm": 5,
        "bright_cool": 5,
    }
    assert manifest.raw["required_runtime_values"]["appearance_config"]


def test_appearance_physical_obstacle_binding_matches_audited_launches():
    assert load_manifest(MANIFESTS["kujiale_appearance"]).raw[
        "required_runtime_values"
    ]["physical_obstacles_enabled"] is True
    assert load_manifest(MANIFESTS["rivermark_appearance"]).raw[
        "required_runtime_values"
    ]["physical_obstacles_enabled"] is False


def test_dynamic_actions_are_trigger_then_complete_exactly_once_no_retry():
    ledger = DynamicActionLedger()
    ledger.claim("G2", "trigger")
    ledger.record("G2", "trigger", "accepted", "armed")
    ledger.claim("G2", "complete")
    ledger.record("G2", "complete", "accepted", "retired")
    assert [(row["action"], row["result"]) for row in ledger.events] == [
        ("trigger", "claimed"), ("trigger", "accepted"),
        ("complete", "claimed"), ("complete", "accepted"),
    ]
    with pytest.raises(V6ContractError, match="retry forbidden"):
        ledger.claim("G2", "trigger")
    with pytest.raises(V6ContractError, match="before trigger"):
        DynamicActionLedger().claim("G3", "complete")


def test_pilot_lint_reports_not_qualified_and_does_not_create_output(tmp_path, capsys):
    output = tmp_path / "formal_ledger.jsonl"
    assert cli(["--manifest", str(MANIFESTS["kujiale_static"]), "--pilot", "--output-jsonl", str(output)]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["qualification"] == ENGINEERING_PILOT
    assert result["formal_qualification"] == NOT_QUALIFIED
    assert result["dispatch"] is False
    assert not output.exists()


def test_dispatch_pilot_requires_pilot(capsys):
    assert cli(["--manifest", str(MANIFESTS["kujiale_static"]), "--dispatch-pilot"]) == 2
    assert "--dispatch-pilot requires --pilot" in capsys.readouterr().err


@pytest.mark.parametrize(
    "policy",
    [
        {
            "ekf_profile": "wheel_imu",
            "lidar_odometry_backend": "rf2o",
            "lidar_odometry_validated": False,
            "rf2o_decision": "shadow",
        },
        {
            "ekf_profile": "wheel_imu_lidar",
            "lidar_odometry_backend": "rf2o",
            "lidar_odometry_validated": True,
            "rf2o_decision": "active_fused",
        },
    ],
)
def test_formal_rejects_rf2o_shadow_or_fused_policy(tmp_path, policy):
    raw = yaml.safe_load(MANIFESTS["kujiale_static"].read_text())
    raw["runtime"].update(policy)
    raw["scene_contract_frozen"] = True
    path = tmp_path / "policy.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False))

    manifest = load_manifest(path)
    with pytest.raises(V6ContractError, match="final Estimated policy mismatch"):
        authorize_manifest(manifest, mode="formal")


def test_engineering_policy_override_is_explicit_and_not_qualified(
    tmp_path, capsys
):
    raw = yaml.safe_load(MANIFESTS["kujiale_static"].read_text())
    raw["runtime"].update({
        "lidar_odometry_backend": "rf2o",
        "lidar_odometry_validated": False,
        "rf2o_decision": "shadow",
    })
    path = tmp_path / "shadow.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False))
    manifest = load_manifest(path)

    with pytest.raises(V6ContractError, match="requires explicit engineering override"):
        authorize_manifest(manifest, mode="pilot")
    assert authorize_manifest(
        manifest,
        mode="pilot",
        allow_engineering_policy_override=True,
    ) == NOT_QUALIFIED

    assert cli([
        "--manifest", str(path), "--pilot",
        "--allow-engineering-estimated-policy-override",
    ]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["formal_qualification"] == NOT_QUALIFIED
    assert result["engineering_estimated_policy_override"] is True


def test_evidence_jsonl_is_append_only_rows(tmp_path):
    path = tmp_path / "pilot.jsonl"
    append_evidence_jsonl(path, "route_goal_published", leg_id="G2")
    append_evidence_jsonl(path, "dynamic_action", group="G2", phase="armed")
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [row["event"] for row in rows] == ["route_goal_published", "dynamic_action"]
    assert all(row["wall_time_ns"] > 0 for row in rows)


def test_appearance_summary_captures_profile_light_and_material_counts():
    adapter = object.__new__(V6FormalNode)
    adapter.appearance_state = {
        "profile_id": "dim_warm",
        "overrides": {"light_intensity_scale": 0.4, "material_hue_shift_deg": 35.0},
        "applied_counts": {"lights": 3, "material_color_inputs": 7},
    }
    assert adapter._appearance_summary() == {
        "observed": True,
        "profile_id": "dim_warm",
        "light_intensity_scale": 0.4,
        "material_hue_shift_deg": 35.0,
        "lights_applied": 3,
        "material_inputs_applied": 7,
    }


def test_cognitive_capture_schema_covers_causal_control_and_pilot_facts():
    required = {
        "/bio_nav/module2/cognitive_place_graph",
        "/bio_nav/module3/cognitive_graph_validation_ack",
        "/bio_nav/navigation_graph",
        "/bio_nav/canonical_route",
        "/bio_nav/route_progress",
        "/bio_nav/module2/goal_planning_prior",
        "/bio_nav/risk_layer/status",
        "/bio_nav/cognitive_risk_critic/status",
        "/bio_nav/module3/cognitive_edge_outcome",
        "/cmd_vel",
        "/simulation/collision",
        "/simulation/collision_diagnostics",
        "/experiment/obstacles/state",
        "/experiment/appearance/state",
    }
    assert required <= CAPTURE_SCHEMA.keys()


def test_formal_dispatch_remains_fail_closed_while_candidates_unfrozen(capsys):
    assert cli(["--manifest", str(MANIFESTS["kujiale_static"])]) == 2
    assert "scene_contract_frozen is false" in capsys.readouterr().err
