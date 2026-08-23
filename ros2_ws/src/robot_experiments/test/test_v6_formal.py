from __future__ import annotations

import json
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_experiments.v6_formal import (
    CAPTURE_SCHEMA,
    DISPATCH_SUBSCRIPTION_TOPICS,
    ENGINEERING_PILOT,
    EpisodeGuard,
    FULL_HOUSE_LEGS,
    MissionLeg,
    NOT_QUALIFIED,
    PHASE1_RUNTIME,
    ReadinessFacts,
    V6ContractError,
    V6FormalNode,
    append_evidence_jsonl,
    authorize_manifest,
    cli,
    load_manifest,
)

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config"
MANIFESTS = {
    name: CONFIG / f"v6_final_kujiale_{name}.yaml"
    for name in ("static", "dynamic", "appearance")
}


def ready_facts() -> ReadinessFacts:
    return ReadinessFacts(
        **{name: True for name in ReadinessFacts.__dataclass_fields__}
    )


def ready_guard() -> EpisodeGuard:
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(),
        pre_reset_route_messages=0,
        localization_accepted_floor=4,
    )
    guard.record_reset_call()
    guard.record_reset_event()
    guard.record_reset_response(True)
    guard.record_reset_receipt_generation(7)
    guard.record_localization_status(5, "WAITING_FOR_SCAN", False)
    guard.record_localization_status(5, "WAITING_FOR_RESULT", False)
    guard.record_localization_status(5, "ACCEPTED", True)
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_gate_status(7, False)
    assert guard.goal_ready
    return guard


def test_dispatcher_topic_firewall_and_grid_capture_contract():
    assert not [
        topic
        for topic in DISPATCH_SUBSCRIPTION_TOPICS
        if topic.startswith("/ground_truth/")
    ]
    assert {
        "/flatscan",
        "/localization_result",
        "/bio_nav/localization/status",
        "/odom",
    } <= set(CAPTURE_SCHEMA)
    source = (PACKAGE / "robot_experiments" / "v6_formal.py").read_text()
    for retired in ("/amcl_pose", "/initialpose", "odom_static"):
        assert retired not in source


def test_phase1_goal_waits_for_new_grid_generation_and_gate_release():
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(),
        pre_reset_route_messages=0,
        localization_accepted_floor=8,
    )
    guard.record_reset_call()
    guard.record_reset_event()
    guard.record_reset_response(True)
    guard.record_reset_receipt_generation(3)
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_gate_status(3, False)

    guard.record_localization_status(8, "ACCEPTED", True)
    guard.record_localization_status(9, "ACCEPTED", True)
    assert not guard.goal_ready
    guard.record_localization_status(9, "WAITING_FOR_SCAN", False)
    guard.record_localization_status(10, "ACCEPTED", True)
    assert not guard.goal_ready
    guard.record_localization_status(9, "WAITING_FOR_RESULT", False)
    guard.record_localization_status(9, "ACCEPTED", True)
    assert guard.goal_ready
    assert guard.localization_generation == 9


def test_grid_rejection_stops_before_goal():
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(),
        pre_reset_route_messages=0,
        localization_accepted_floor=0,
    )
    guard.record_reset_call()
    guard.record_reset_event()
    guard.record_localization_status(1, "WAITING_FOR_SCAN", False)
    guard.record_localization_status(1, "REJECTED", False)
    assert guard.stop_reason == "grid_localization_rejected:1"


def test_reset_is_exactly_once_and_unknown_response_stops():
    guard = EpisodeGuard()
    guard.arm_reset(
        ready_facts(),
        pre_reset_route_messages=0,
        localization_accepted_floor=0,
    )
    guard.record_reset_call()
    guard.record_reset_response(None)
    assert guard.stop_reason == "reset_response_unknown"
    with pytest.raises(V6ContractError, match="reset_retry_forbidden"):
        guard.record_reset_call()


def test_full_house_goals_publish_in_exact_order():
    guard = ready_guard()
    for index, goal_id in enumerate(FULL_HOUSE_LEGS):
        guard.record_goal_publication(goal_id)
        guard.record_route_progress()
        guard.record_route_completion(True)
        expected = "SUCCEEDED" if index == 4 else "LEG_SUCCEEDED"
        assert guard.state == expected
    assert guard.completed_leg_ids == list(FULL_HOUSE_LEGS)


def test_goal_order_mismatch_stops_before_publish():
    guard = ready_guard()
    with pytest.raises(V6ContractError, match="mission_leg_order"):
        guard.record_goal_publication("G3")
    assert guard.goal_publications == 0


def test_route_completion_requires_progress():
    guard = ready_guard()
    guard.record_goal_publication("G2")
    guard.record_route_completion(True)
    assert guard.stop_reason == "route_completed_without_progress"


def test_goal_message_uses_valid_identity_orientation_placeholder():
    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(frame_id="", stamp=None)
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            )

    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter._types = {"PoseStamped": PoseStamped}
    adapter.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: "stamp")
        )
    )
    message = adapter._goal_message(MissionLeg("G2", "map", 1.0, 2.0))
    assert message.pose.orientation.w == 1.0
    assert message.pose.orientation.z == 0.0


@pytest.mark.parametrize("name", ["static", "dynamic", "appearance"])
def test_kujiale_manifests_share_phase1_runtime_and_xy_loop(name):
    manifest = load_manifest(MANIFESTS[name])
    assert manifest.raw["runtime"] | PHASE1_RUNTIME == manifest.raw["runtime"]
    assert [leg.goal_id for leg in manifest.mission_legs] == list(FULL_HOUSE_LEGS)
    assert all(
        set(row) == {"id", "frame_id", "x", "y"}
        for row in manifest.raw["mission"]["legs"]
    )
    assert "yaw_deg" in manifest.reset_pose
    assert len(manifest.episodes) == 20
    assert manifest.missing_required_values == ()
    assert authorize_manifest(manifest, mode="pilot") == NOT_QUALIFIED


def test_only_static_manifest_enables_phase1_and_other_intents_are_preserved():
    static = load_manifest(MANIFESTS["static"])
    dynamic = load_manifest(MANIFESTS["dynamic"])
    appearance = load_manifest(MANIFESTS["appearance"])
    assert static.phase1_enabled
    assert not dynamic.phase1_enabled
    assert not appearance.phase1_enabled
    assert dynamic.scenario_intent["dynamic_case_id"] == "full_route_three_stage"
    assert appearance.scenario_intent["appearance_profiles"] == [
        "dim_warm",
        "dim_cool",
        "bright_warm",
        "bright_cool",
    ]


def test_dynamic_or_appearance_phase1_dispatch_is_refused(tmp_path):
    for name in ("dynamic", "appearance"):
        assert cli(
            [
                "--manifest",
                str(MANIFESTS[name]),
                "--pilot",
                "--dispatch-pilot",
                "--output-jsonl",
                str(tmp_path / f"{name}.jsonl"),
            ]
        ) == 2


def test_formal_static_remains_unqualified_until_scene_is_frozen():
    manifest = load_manifest(MANIFESTS["static"])
    with pytest.raises(V6ContractError, match="scene_contract_frozen"):
        authorize_manifest(manifest, mode="formal")


def test_pilot_validation_prints_canonical_runtime_and_loop(capsys):
    assert cli(["--manifest", str(MANIFESTS["static"]), "--pilot"]) == 0
    row = json.loads(capsys.readouterr().out)
    assert row["qualification"] == ENGINEERING_PILOT
    assert row["runtime"]["localization_backend"] == "grid"
    assert row["runtime"]["nav2_profile"] == "stable"
    assert row["runtime"]["cognitive_profile"] == "M0"
    assert row["runtime"]["module2_enabled"] is False
    assert row["mission_leg_ids"] == list(FULL_HOUSE_LEGS)


def test_manifest_rejects_waypoint_yaw(tmp_path):
    raw = yaml.safe_load(MANIFESTS["static"].read_text())
    raw["mission"]["legs"][0]["yaw_deg"] = 0.0
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(V6ContractError, match="only id/frame_id/x/y"):
        load_manifest(path)


def test_pre_reset_still_and_post_reset_odom_checks():
    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter._cmd_window = deque()
    now = __import__("time").monotonic()
    adapter._odom_window = deque(
        [(now - 0.8, 1.0, 2.0), (now - 0.2, 1.02, 2.01)]
    )
    assert adapter._pre_reset_still()

    adapter.guard = EpisodeGuard()
    adapter.post_reset_odom_xy = [(9.0, 9.0), (0.01, 0.01), (0.02, 0.01)]
    adapter._check_post_reset_odom()
    assert not adapter.guard.stop_reason


def test_jsonl_appends_without_overwrite(tmp_path):
    path = tmp_path / "episode.jsonl"
    append_evidence_jsonl(path, "route_goal_published", leg_id="G2")
    append_evidence_jsonl(path, "mission_leg_result", leg_id="G2")
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    assert [row["event"] for row in rows] == [
        "route_goal_published",
        "mission_leg_result",
    ]
