from collections import Counter
from pathlib import Path

import pytest

from robot_experiments.v6_formal import (
    CAPTURE_SCHEMA,
    DISPATCH_SUBSCRIPTION_TOPICS,
    EpisodeGuard,
    NOT_QUALIFIED,
    ReadinessFacts,
    V6ContractError,
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
    assert '"/bio_nav/route_progress"' in source
    assert "NavigateToPose" not in source
    assert "navigate_to_pose" not in source


def test_reset_is_exactly_once_and_unknown_response_never_retries():
    guard = EpisodeGuard()
    guard.arm_reset(ready_facts(), 8)
    guard.record_reset_call()
    guard.record_reset_response(None)
    assert guard.state == "STOP"
    assert guard.stop_reason == "reset_response_unknown"
    assert guard.reset_calls == 1
    with pytest.raises(V6ContractError, match="reset_retry_forbidden"):
        guard.record_reset_call()
    assert guard.reset_calls == 1


def test_second_reset_event_is_immediate_stop():
    guard = EpisodeGuard()
    guard.arm_reset(ready_facts(), 3)
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_event()
    assert guard.state == "WAITING_RESET_EPOCH"
    guard.record_reset_event()
    assert guard.state == "STOP"
    assert guard.stop_reason == "second_reset_event"


def test_goal_requires_reset_epoch_plus_one_prior_and_localization_seed():
    guard = EpisodeGuard()
    guard.arm_reset(ready_facts(), 41)
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_event()
    guard.record_prior(42)
    assert not guard.goal_ready
    guard.record_localization_seeded()
    assert guard.goal_ready
    guard.record_goal_publication()
    guard.record_route_progress()
    guard.record_route_completion(True)
    assert guard.state == "SUCCEEDED"
    assert guard.reset_calls == guard.reset_events == guard.goal_publications == 1


def test_wrong_bridge_epoch_stops_before_goal():
    guard = EpisodeGuard()
    guard.arm_reset(ready_facts(), 10)
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_event()
    guard.record_prior(12)
    assert guard.state == "STOP"
    assert guard.stop_reason == "bridge_epoch_mismatch:12!=11"


def test_readiness_lists_every_missing_fact():
    facts = ready_facts()
    facts.constraints_seen = False
    facts.route_goal_subscriber_ready = False
    with pytest.raises(V6ContractError, match="constraints_seen"):
        EpisodeGuard().arm_reset(facts, 1)


@pytest.mark.parametrize("path", MANIFESTS.values())
def test_all_six_manifests_are_twenty_row_drafts_and_formal_refuses(path):
    manifest = load_manifest(path)
    assert len(manifest.episodes) == 20
    assert manifest.frozen is False
    assert manifest.missing_required_values
    with pytest.raises(V6ContractError, match="scene_contract_frozen is false"):
        authorize_manifest(manifest, mode="formal")
    assert authorize_manifest(manifest, mode="pilot") == NOT_QUALIFIED


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
    manifest = load_manifest(MANIFESTS[name])
    assert [row.seed for row in manifest.episodes] == list(range(first, last + 1))


@pytest.mark.parametrize("name", ["kujiale_dynamic", "rivermark_dynamic"])
def test_dynamic_variants_are_v1_through_v5_four_each(name):
    manifest = load_manifest(MANIFESTS[name])
    assert Counter(row.variant_id for row in manifest.episodes) == {
        "v1": 4,
        "v2": 4,
        "v3": 4,
        "v4": 4,
        "v5": 4,
    }


@pytest.mark.parametrize("name", ["kujiale_appearance", "rivermark_appearance"])
def test_appearance_profiles_are_four_profiles_five_each(name):
    manifest = load_manifest(MANIFESTS[name])
    assert Counter(row.appearance_profile_id for row in manifest.episodes) == {
        "dim_warm": 5,
        "dim_cool": 5,
        "bright_warm": 5,
        "bright_cool": 5,
    }


def test_pilot_reports_not_qualified_and_does_not_create_output(tmp_path, capsys):
    output = tmp_path / "formal_ledger.jsonl"
    assert cli([
        "--manifest", str(MANIFESTS["kujiale_static"]),
        "--mode", "pilot",
        "--output-jsonl", str(output),
    ]) == 0
    assert NOT_QUALIFIED in capsys.readouterr().out
    assert not output.exists()


def test_cognitive_capture_schema_covers_causal_and_control_paths():
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
    }
    assert required <= CAPTURE_SCHEMA.keys()


def test_formal_dispatch_is_explicitly_disabled_by_default(capsys):
    code = cli(["--manifest", str(MANIFESTS["kujiale_static"])])
    assert code == 2
    assert "scene_contract_frozen is false" in capsys.readouterr().err
