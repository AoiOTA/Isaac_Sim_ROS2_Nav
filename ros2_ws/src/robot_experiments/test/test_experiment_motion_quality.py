from dataclasses import replace
import hashlib
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
from nav2_msgs.msg import CollisionMonitorState
import yaml

import robot_experiments.experiment_runner as experiment_runner_module
from robot_experiments.attempt31_rivermark_qualification import _rate_group
from robot_experiments.experiment_runner import (
    _cognitive_admission_components,
    _edge_prior_statistics,
    _episode_validity,
    _localization_node_ownership_evidence,
    _mcap_inventory_evidence,
    _mcap_required_topic_coverage,
    _module2_readiness_required,
    _validate_condition_stack_parameters,
    _parse_obstacle_completion,
    _record_tracked_route_length,
    _result_with_terminal_zero,
    _route_prior_application_evidence,
    _strict_success_from_leg_count,
    validate_recorded_run_evidence,
    CommandSample,
    ExperimentRunner,
    OdometrySample,
    _dynamic_interaction_acceptance,
    _reset_dynamic_selection,
)
from robot_experiments.configuration import ConfigurationError
from robot_experiments.scenario import RunSelection, load_scenario


def test_campaign_provenance_distinguishes_untracked_from_tracked_dirty(tmp_path):
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    tracked = tmp_path / "tracked.txt"
    tracked.write_text("initial\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(tmp_path), "add", "tracked.txt"], check=True)
    subprocess.run(
        [
            "git", "-C", str(tmp_path), "-c", "user.name=Codex Test",
            "-c", "user.email=codex@example.invalid", "commit", "-qm", "fixture",
        ],
        check=True,
    )
    (tmp_path / "build.log").write_text("untracked\n", encoding="utf-8")

    provenance = experiment_runner_module._campaign_provenance(
        tmp_path, "missing-map", "missing-posegraph"
    )

    assert provenance["git_dirty"] is True
    assert provenance["git_tracked_dirty"] is False

    tracked.write_text("changed\n", encoding="utf-8")
    provenance = experiment_runner_module._campaign_provenance(
        tmp_path, "missing-map", "missing-posegraph"
    )
    assert provenance["git_dirty"] is True
    assert provenance["git_tracked_dirty"] is True


def test_tracked_route_length_replaces_untrimmed_canonical_edge_sum():
    routes = [{"request_id": 7, "planned_length_m": 14.65}]

    _record_tracked_route_length(routes, 7, 0.07, 11.87)
    _record_tracked_route_length(routes, 7, 11.88, 0.07)

    assert routes[0]["canonical_full_edge_length_m"] == pytest.approx(14.65)
    assert routes[0]["tracked_route_length_m"] == pytest.approx(11.95)
    assert routes[0]["planned_length_m"] == pytest.approx(11.95)


def test_edge_prior_statistics_preserve_nonzero_learned_cost_evidence():
    priors = [
        SimpleNamespace(cost_delta_m=0.0, learned_risk=0.0),
        SimpleNamespace(cost_delta_m=0.55, learned_risk=1.0),
        SimpleNamespace(cost_delta_m=0.07, learned_risk=0.58),
    ]

    assert _edge_prior_statistics(priors) == {
        "prior_count": 3,
        "positive_cost_count": 2,
        "total_cost_delta_m": pytest.approx(0.62),
        "maximum_cost_delta_m": pytest.approx(0.55),
        "maximum_learned_risk": pytest.approx(1.0),
    }


def _write_mcap_metadata(path: Path, topic_counts: dict[str, int]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump({
            "rosbag2_bagfile_information": {
                "topics_with_message_count": [
                    {
                        "topic_metadata": {"name": topic},
                        "message_count": count,
                    }
                    for topic, count in topic_counts.items()
                ]
            }
        }),
        encoding="utf-8",
    )


def test_required_topic_coverage_is_scene_aware_and_requires_messages(tmp_path):
    common = {
        topic: 1
        for topic in (
            *experiment_runner_module.COMMON_REQUIRED_RECORDED_TOPICS,
            *experiment_runner_module.ROUTE_GUIDED_REQUIRED_RECORDED_TOPICS,
        )
    }
    metadata = tmp_path / "telemetry" / "metadata.yaml"
    _write_mcap_metadata(metadata, {**common, "/amcl_pose": 3})

    indoor = _mcap_required_topic_coverage(
        metadata, scene="indoor", route_guided=True
    )
    outdoor_with_amcl = _mcap_required_topic_coverage(
        metadata, scene="outdoor", route_guided=True
    )
    _write_mcap_metadata(metadata, common)
    outdoor = _mcap_required_topic_coverage(
        metadata, scene="outdoor", route_guided=True
    )

    assert indoor["passed"]
    assert "/simulation/reset_event" not in indoor["required_topics"]
    assert "/simulation/reset_stop_gate/status" in indoor["required_topics"]
    assert "/bio_nav/canonical_route" in indoor["required_topics"]
    assert "/amcl_pose" in indoor["required_topics"]
    assert "/amcl_pose" not in outdoor["required_topics"]
    assert "/tf_static" not in outdoor["required_topics"]
    assert outdoor_with_amcl["observed_forbidden_topics"] == ["/amcl_pose"]
    assert not outdoor_with_amcl["passed"]
    assert outdoor["passed"]

    direct = _mcap_required_topic_coverage(
        metadata, scene="outdoor", route_guided=False
    )
    assert "/bio_nav/navigation_graph" not in direct["required_topics"]
    assert "/bio_nav/canonical_route" not in direct["required_topics"]
    assert "/bio_nav/route_progress" not in direct["required_topics"]


def test_required_topic_coverage_marks_recorder_errors_invalid(tmp_path):
    metadata = tmp_path / "metadata.yaml"
    counts = {
        topic: 1
        for topic in (
            *experiment_runner_module.COMMON_REQUIRED_RECORDED_TOPICS,
        )
    }
    _write_mcap_metadata(metadata, counts)

    coverage = _mcap_required_topic_coverage(
        metadata, scene="outdoor", recorder_error="recorder_exit_code:1"
    )

    assert not coverage["passed"]
    assert coverage["recorder_error"] == "recorder_exit_code:1"


def test_mcap_inventory_rejects_placeholder_and_requires_metadata_schema(tmp_path):
    root = tmp_path / "run"
    telemetry = root / "telemetry"
    telemetry.mkdir(parents=True)
    metadata = {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "relative_file_paths": ["telemetry_0.mcap"],
            "message_count": 1,
            "topics_with_message_count": [],
        }
    }
    (telemetry / "metadata.yaml").write_text(
        yaml.safe_dump(metadata), encoding="utf-8"
    )
    mcap = telemetry / "telemetry_0.mcap"
    mcap.write_bytes(b"mcap")
    assert not _mcap_inventory_evidence(root)["passed"]

    magic = experiment_runner_module.MCAP_MAGIC
    mcap.write_bytes(magic + b"payload" + magic)
    assert not _mcap_inventory_evidence(root)["passed"]


def _recorded_collision_contract_fixture(tmp_path, monkeypatch, *, collision=False):
    root = tmp_path / "recorded-run"
    root.mkdir()
    mandatory = {
        "TRIAL_DISPATCHED.json", "run_manifest.json", "run_summary.json",
        "events.jsonl", "ground_truth.csv.gz", "odom.csv.gz", "cmd_vel.csv.gz",
        "obstacles.csv.gz", "dynamic_obstacles.csv.gz", "leg_metrics.csv",
        "depth_frame.pgm", "depth_frame.json", "scan.csv", "scan.json",
        "scan_safety.csv", "scan_safety.json", "local_costmap.pgm",
        "local_costmap.json", "global_costmap.pgm", "global_costmap.json",
        "FINAL_TRIAL_METRICS.json", "telemetry/metadata.yaml",
    }
    for relative in mandatory:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("fixture\n", encoding="utf-8")
    coverage = {
        "required_topics": ["/simulation/collision"],
        "message_counts": {"/simulation/collision": 3},
        "forbidden_topics": [],
        "forbidden_message_counts": {},
        "observed_forbidden_topics": [],
        "passed": True,
    }
    inventory = {
        "passed": True,
        "topic_counts": {"/simulation/collision": 3},
        "topic_types": {"/simulation/collision": "std_msgs/msg/Bool"},
        "semantic": {
            "collision_true_count": int(collision),
            "route_complete_true_count": 5,
            "terminal_zero_count": 2,
            "terminal_nonzero_count": 0,
            "positive_requested_count": 1,
            "positive_applied_count": 1,
        },
    }
    monkeypatch.setattr(experiment_runner_module, "_mcap_inventory_evidence", lambda _root: inventory)
    monkeypatch.setattr(
        experiment_runner_module,
        "_mcap_required_topic_coverage",
        lambda *_args, **_kwargs: coverage,
    )
    monkeypatch.setattr(
        ExperimentRunner, "_checksums_are_verified", staticmethod(lambda _root: True)
    )
    route_costs = [{
        "request_id": 1,
        "edges": [{
            "requested_module2_delta_m": 0.2,
            "applied_module2_delta_m": 0.2,
        }],
    }]
    route_prior = _route_prior_application_evidence(route_costs, required=True)
    manifest = {
        "result": "failure" if collision else "success",
        "legs": [{"id": f"G{index}"} for index in range(1, 6)],
        "terminal_zero_confirmed": True,
        "reset_receipt": {"generation": 2},
        "observability": {"collision_status_seen": True},
        "route_edge_costs": route_costs,
    }
    summary = {
        "navigation_contract_success": not collision,
        "terminal_zero_confirmed": True,
        "reset_receipt": {"generation": 2},
        "reset_receipt_confirmed": True,
        "isaac_contact_sensor_collision_detected": collision,
        "physical_collision_free": not collision,
        "contact_sensor_evidence_confirmed": True,
        "fixed_map_to_odom_evidence_confirmed": True,
        "localization_node_ownership": {},
        "data_complete": True,
        "checksums_verified": True,
        "required_topic_coverage": coverage,
        "route_prior_application": route_prior,
        "route_prior_application_confirmed": True,
        "evidence": {
            "required_files": sorted(
                mandatory - {
                    "run_summary.json", "FINAL_TRIAL_METRICS.json",
                    "telemetry/metadata.yaml",
                }
            )
        },
        "final_trial_metric_gate": {"passed": True},
        "episode_validity": {
            "valid": True,
            "status": "valid",
            "invalid_reasons": [],
        },
        "strict_success": not collision,
    }
    (root / "FINAL_TRIAL_METRICS.json").write_text(
        json.dumps(summary["final_trial_metric_gate"]), encoding="utf-8"
    )
    return root, summary, manifest, inventory, coverage


def test_recorded_contact_validator_accepts_legacy_absent_and_future_consistent_fields(
    tmp_path, monkeypatch
):
    root, summary, manifest, _inventory, _coverage = (
        _recorded_collision_contract_fixture(tmp_path, monkeypatch)
    )

    assert validate_recorded_run_evidence(
        root, summary, manifest, scene="indoor", route_guided=True,
        route_prior_required=True, expected_leg_count=5,
    )["strict_success"] is True

    manifest["isaac_contact_sensor_collision_detected"] = False
    manifest["physical_collision_free"] = True
    assert validate_recorded_run_evidence(
        root, summary, manifest, scene="indoor", route_guided=True,
        route_prior_required=True, expected_leg_count=5,
    )["strict_success"] is True


@pytest.mark.parametrize(
    "mutation",
    ["missing_one", "wrong_type", "manifest_disagrees", "summary_wrong", "observability"],
)
def test_recorded_contact_validator_rejects_inconsistent_boolean_contract(
    tmp_path, monkeypatch, mutation
):
    root, summary, manifest, _inventory, _coverage = (
        _recorded_collision_contract_fixture(tmp_path, monkeypatch)
    )
    manifest.update({
        "isaac_contact_sensor_collision_detected": False,
        "physical_collision_free": True,
    })
    if mutation == "missing_one":
        manifest.pop("physical_collision_free")
    elif mutation == "wrong_type":
        manifest["isaac_contact_sensor_collision_detected"] = 0
    elif mutation == "manifest_disagrees":
        manifest["isaac_contact_sensor_collision_detected"] = True
        manifest["physical_collision_free"] = False
    elif mutation == "summary_wrong":
        summary["physical_collision_free"] = 1
    else:
        manifest["observability"]["collision_status_seen"] = False

    with pytest.raises(ConfigurationError, match="contact_sensor"):
        validate_recorded_run_evidence(
            root, summary, manifest, scene="indoor", route_guided=True,
            route_prior_required=True, expected_leg_count=5,
        )


@pytest.mark.parametrize(
    "mutation",
    ["true_count", "zero_count_for_collision", "wrong_type", "count", "inventory", "checksum"],
)
def test_recorded_contact_validator_rejects_mcap_or_checksum_tamper(
    tmp_path, monkeypatch, mutation
):
    collision = mutation == "zero_count_for_collision"
    root, summary, manifest, inventory, _coverage = _recorded_collision_contract_fixture(
        tmp_path, monkeypatch, collision=collision
    )
    if mutation == "true_count":
        inventory["semantic"]["collision_true_count"] = 1
    elif mutation == "zero_count_for_collision":
        inventory["semantic"]["collision_true_count"] = 0
    elif mutation == "wrong_type":
        inventory["topic_types"]["/simulation/collision"] = "std_msgs/msg/String"
    elif mutation == "count":
        inventory["topic_counts"]["/simulation/collision"] = 2
    elif mutation == "inventory":
        inventory["passed"] = False
    else:
        monkeypatch.setattr(
            ExperimentRunner, "_checksums_are_verified", staticmethod(lambda _root: False)
        )

    with pytest.raises(ConfigurationError, match="recorded run evidence invalid"):
        validate_recorded_run_evidence(
            root, summary, manifest, scene="indoor", route_guided=True,
            route_prior_required=True, expected_leg_count=5,
            require_strict_success=not collision,
        )

@pytest.mark.parametrize(
    "scenario_id",
    [
        f"{prefix}_{category}"
        for prefix in ("v6_final_kujiale", "final_rivermark")
        for category in ("static", "dynamic", "appearance")
    ],
)
def test_final_scenarios_force_module2_planning_readiness(scenario_id):
    assert _module2_readiness_required(scenario_id, "auto") is True
    assert _module2_readiness_required(scenario_id, "true") is True
    with pytest.raises(ConfigurationError, match="cannot disable"):
        _module2_readiness_required(scenario_id, "false")


def test_legacy_scenarios_keep_module2_readiness_default_off():
    assert _module2_readiness_required("static", "auto") is False
    assert _module2_readiness_required("static", "false") is False


def test_pilot_stack_attestation_allows_empty_formal_digest(tmp_path):
    _validate_condition_stack_parameters(
        condition_stack_id="indoor_static",
        stack_session_id="a" * 64,
        condition_stack_contract_path=tmp_path / "stack.contract.json",
        formal_freeze_digest="",
    )


def test_formal_digest_requires_complete_stack_attestation(tmp_path):
    with pytest.raises(ConfigurationError, match="requires condition stack"):
        _validate_condition_stack_parameters(
            condition_stack_id="",
            stack_session_id="",
            condition_stack_contract_path=None,
            formal_freeze_digest="b" * 64,
        )
    with pytest.raises(ConfigurationError, match="supplied together"):
        _validate_condition_stack_parameters(
            condition_stack_id="indoor_static",
            stack_session_id="",
            condition_stack_contract_path=tmp_path / "stack.contract.json",
            formal_freeze_digest="",
        )

    _validate_condition_stack_parameters(
        condition_stack_id="indoor_static",
        stack_session_id="a" * 64,
        condition_stack_contract_path=tmp_path / "stack.contract.json",
        formal_freeze_digest="b" * 64,
    )


@pytest.mark.parametrize(
    ("nodes", "passed", "owner_count", "forbidden"),
    [
        (["/ideal_localization_tf"], True, 1, []),
        (["ideal_localization_tf", "ideal_localization_tf"], False, 2, []),
        (["ideal_localization_tf", "amcl"], False, 1, ["amcl"]),
        (["ideal_localization_tf", "slam_toolbox"], False, 1, ["slam_toolbox"]),
        ([], False, 0, []),
    ],
)
def test_outdoor_localization_node_ownership_is_known_owner_fail_closed(
    nodes, passed, owner_count, forbidden
):
    evidence = _localization_node_ownership_evidence("outdoor", nodes)

    assert evidence["passed"] is passed
    assert evidence["required_owner_count"] == owner_count
    assert evidence["observed_forbidden_basenames"] == forbidden
    assert evidence["scope"] == "known_localization_nodes_not_arbitrary_tf_publishers"


def test_m3_route_prior_requires_positive_requested_and_applied_costs():
    records = [{
        "request_id": 7,
        "edges": [
            {
                "requested_module2_delta_m": 0.5,
                "applied_module2_delta_m": 0.0,
            },
            {
                "requested_module2_delta_m": 0.2,
                "applied_module2_delta_m": 0.2,
            },
        ],
    }]

    confirmed = _route_prior_application_evidence(records, required=True)
    missing = _route_prior_application_evidence([], required=True)
    not_required = _route_prior_application_evidence([], required=False)

    assert confirmed["positive_requested_count"] == 2
    assert confirmed["positive_applied_count"] == 1
    assert confirmed["confirmed"]
    assert not missing["confirmed"]
    assert not_required["confirmed"]


def test_episode_validity_separates_missing_evidence_from_product_failure():
    valid_product_failure = {
        "terminal_zero_confirmed": True,
        "reset_receipt": {"generation": 1},
        "reset_receipt_confirmed": True,
        "contact_sensor_evidence_confirmed": True,
        "fixed_map_to_odom_evidence_confirmed": True,
        "data_complete": True,
        "checksums_verified": True,
        "required_topic_coverage": {"required": True, "passed": True},
        "route_prior_application": {"required": True, "confirmed": True},
        "navigation_contract_success": False,
    }
    invalid = dict(valid_product_failure, checksums_verified=False)

    assert _episode_validity(valid_product_failure)["valid"]
    assert _episode_validity(invalid) == {
        "valid": False,
        "status": "invalid",
        "invalid_reasons": ["checksums_unverified"],
    }

    missing_outdoor_tf = dict(
        valid_product_failure,
        fixed_map_to_odom_evidence_confirmed=False,
    )
    assert _episode_validity(missing_outdoor_tf)["invalid_reasons"] == [
        "fixed_map_to_odom_evidence_missing"
    ]


def test_obstacle_completion_requires_the_exact_selected_actor_set():
    payload = json.dumps({"group": "G2", "retired": ["dynamic_box"]})
    assert _parse_obstacle_completion(
        payload,
        expected_group="G2",
        expected_ids={"dynamic_box"},
    ) == ("dynamic_box",)

    with pytest.raises(RuntimeError, match="retired IDs mismatch"):
        _parse_obstacle_completion(
            json.dumps({"group": "G2", "retired": []}),
            expected_group="G2",
            expected_ids={"dynamic_box"},
        )


def _retirement_stamp(value):
    seconds = int(value)
    return SimpleNamespace(
        sec=seconds,
        nanosec=int(round((value - seconds) * 1.0e9)),
    )


def _cognitive_admission_runner(*, components=None):
    runner = object.__new__(ExperimentRunner)
    runner._scenario = SimpleNamespace(map_version="map-v1")
    runner._cognitive_admission_components = (
        {
            "global_layer": {"mode": "active"},
            "local_layer": {"mode": "active"},
            "critic": {"mode": "active"},
        }
        if components is None
        else components
    )
    runner._clock_seconds = lambda: 12.0
    runner._latest_cognitive_layer_statuses = {}
    runner._latest_planning_prior_readiness = {
        "stamp_s": 11.0,
        "sequence": 9,
        "reset_epoch": 2,
        "recurrent_session_id": "session-2",
        "map_version": "map-v1",
        "module2_healthy": True,
        "input_healthy": True,
        "observation_valid": True,
        "trusted_write": True,
        "schema_version": "bio_nav_planning_prior_v4",
        "accepted": True,
        "place_entropy_normalized": 0.2,
        "context_uncertainty": 0.1,
    }
    runner._reset_receipt = {"generation": 2}
    runner._cognitive_admission_ready_timeout_sec = 0.25
    runner._wait_until = lambda predicate, _timeout: predicate()
    runner._reset_cognitive_admission_state(
        barrier_ros_s=10.0,
        barrier_monotonic=0.0,
        expected_reset_epoch=2,
        forbidden_recurrent_session_id="old-session",
    )
    return runner


def _cognitive_source(
    sequence, *, epoch=2, session="session-2", map_version="map-v1",
    validation_stamp=11.0,
):
    return SimpleNamespace(
        sequence=sequence,
        reset_epoch=epoch,
        recurrent_session_id=session,
        map_version=map_version,
        validation_stamp=_retirement_stamp(validation_stamp),
        input_healthy=True,
        module2_healthy=True,
        observation_valid=True,
        trusted_write=True,
    )


def _cognitive_status(
    role,
    sequence,
    *,
    rejected=False,
    fallback_reason=None,
    offered=True,
    epoch=2,
    session="session-2",
    map_version="map-v1",
    stamp=11.5,
    consumer=None,
):
    consumers = {
        "global_layer": "/global_costmap/global_costmap:cognitive_obstacle_layer",
        "local_layer": "/local_costmap/local_costmap:cognitive_obstacle_layer",
        "critic": "FollowPath.CognitiveRiskCritic",
    }
    if fallback_reason is None:
        fallback_reason = (
            "shadow;maximum_obstacle_cost_delta=0;obstacle_count=0;"
            "aggregation=max_per_step_mean_horizon"
            if role == "critic"
            else "validation_mode=2;source_age_ms=0;"
            "rejection_reason=shadow;confirmed_count=0"
        )
    return SimpleNamespace(
        consumer=consumer or consumers[role],
        mode="active",
        offered=offered,
        applied=False,
        rejected=rejected,
        fallback_reason=fallback_reason,
        source_sequence=sequence,
        reset_epoch=epoch,
        recurrent_session_id=session,
        map_version=map_version,
        stamp=_retirement_stamp(stamp),
        message_age_ms=25.0,
    )


def _publish_cognitive_sample(
    runner, sequence, *, status_overrides=None, source_overrides=None,
):
    source_overrides = source_overrides or {}
    runner._latest_planning_prior_readiness["sequence"] = sequence
    runner._cognitive_obstacle_callback(
        _cognitive_source(sequence, **source_overrides)
    )
    for role in runner._cognitive_admission_components:
        overrides = dict((status_overrides or {}).get(role, {}))
        status = _cognitive_status(role, sequence, **overrides)
        if role == "critic":
            runner._cognitive_critic_status_callback(status)
        else:
            runner._cognitive_layer_status_callback(status)


def test_cognitive_admission_blocks_odom_time_until_three_new_healthy_samples():
    runner = _cognitive_admission_runner()
    for sequence in range(1, 6):
        _publish_cognitive_sample(
            runner,
            sequence,
            status_overrides={
                role: {
                    "rejected": True,
                    "fallback_reason": "rejection_reason=odom_time",
                }
                for role in runner._cognitive_admission_components
            },
        )
    with pytest.raises(RuntimeError, match="timed out before route dispatch"):
        runner._wait_for_cognitive_admission_ready()

    for sequence in (6, 7):
        _publish_cognitive_sample(runner, sequence)
        with pytest.raises(RuntimeError, match="timed out before route dispatch"):
            runner._wait_for_cognitive_admission_ready()
    _publish_cognitive_sample(runner, 8)
    runner._wait_for_cognitive_admission_ready()

    evidence = runner._cognitive_admission_snapshot()
    assert evidence["ready"] is True
    assert all(
        item["consecutive_healthy_samples"] == 3
        for item in evidence["components"].values()
    )
    assert evidence["components"]["critic"]["latest"][
        "validation_stamp_vs_sim_time_skew_ms"
    ] == pytest.approx(1000.0)


@pytest.mark.parametrize(
    ("source_overrides", "status_overrides"),
    (
        ({"epoch": 1}, {"epoch": 1}),
        ({"session": "old-session"}, {"session": "old-session"}),
        ({"map_version": "old-map"}, {"map_version": "old-map"}),
        ({"validation_stamp": 9.0}, {"stamp": 9.5}),
    ),
    ids=("old-epoch", "old-session", "old-map", "stale"),
)
def test_cognitive_admission_rejects_old_identity_and_stale_samples(
    source_overrides, status_overrides,
):
    runner = _cognitive_admission_runner()
    _publish_cognitive_sample(
        runner,
        1,
        source_overrides=source_overrides,
        status_overrides={
            role: status_overrides
            for role in runner._cognitive_admission_components
        },
    )

    with pytest.raises(RuntimeError, match="timed out before route dispatch"):
        runner._wait_for_cognitive_admission_ready()
    assert not runner._cognitive_admission_snapshot()["ready"]


def test_cognitive_admission_accepts_healthy_zero_cost_no_cell_samples_only_at_three():
    runner = _cognitive_admission_runner()
    for sequence in (1, 2):
        _publish_cognitive_sample(runner, sequence)
    with pytest.raises(RuntimeError, match="timed out before route dispatch"):
        runner._wait_for_cognitive_admission_ready()

    _publish_cognitive_sample(runner, 3)
    runner._wait_for_cognitive_admission_ready()
    evidence = runner._cognitive_admission_snapshot()
    assert evidence["ready"] is True
    assert all(
        item["latest"]["applied"] is False
        and item["latest"]["rejected"] is False
        and item["latest"]["fallback_reason"]
        for item in evidence["components"].values()
    )


def test_cognitive_admission_latches_receipt_against_later_bad_status():
    runner = _cognitive_admission_runner()
    for sequence in (1, 2, 3):
        _publish_cognitive_sample(runner, sequence)
    runner._wait_for_cognitive_admission_ready()
    receipt = runner._cognitive_admission_snapshot()

    _publish_cognitive_sample(
        runner,
        4,
        status_overrides={
            role: {
                "rejected": True,
                "fallback_reason": "rejection_reason=odom_time",
            }
            for role in runner._cognitive_admission_components
        },
    )

    assert runner._cognitive_admission_snapshot() == receipt
    assert receipt["reset_generation"] == 2
    assert receipt["planning_prior"]["sequence"] == 3
    assert {
        role: item["latest"]["source_sequence"]
        for role, item in receipt["components"].items()
    } == {"critic": 3, "global_layer": 3, "local_layer": 3}


def test_cognitive_admission_requires_planning_and_source_sequence_match():
    runner = _cognitive_admission_runner()
    runner._latest_planning_prior_readiness["sequence"] = 99
    runner._cognitive_obstacle_callback(_cognitive_source(1))
    for role in runner._cognitive_admission_components:
        status = _cognitive_status(role, 1)
        if role == "critic":
            runner._cognitive_critic_status_callback(status)
        else:
            runner._cognitive_layer_status_callback(status)

    with pytest.raises(RuntimeError, match="timed out before route dispatch"):
        runner._wait_for_cognitive_admission_ready()


@pytest.mark.parametrize(
    ("role", "overrides"),
    (
        ("global_layer", {"consumer": "/fake/global_costmap:cognitive_obstacle_layer"}),
        ("local_layer", {"consumer": "/fake/local_costmap:cognitive_obstacle_layer"}),
        ("critic", {"consumer": "spoof.FollowPath.CognitiveRiskCritic"}),
        ("global_layer", {"offered": False}),
        ("local_layer", {"offered": False}),
        ("critic", {"offered": False}),
    ),
)
def test_cognitive_admission_rejects_spoofed_consumer_or_unoffered(
    role, overrides,
):
    runner = _cognitive_admission_runner()
    for sequence in (1, 2, 3):
        _publish_cognitive_sample(
            runner,
            sequence,
            status_overrides={role: overrides},
        )
    with pytest.raises(RuntimeError, match="timed out before route dispatch"):
        runner._wait_for_cognitive_admission_ready()


@pytest.mark.parametrize(
    "degraded",
    (
        "prior_sequence", "prior_untrusted", "prior_ood", "prior_nonfinite",
        "missing", "prior_missing", "obstacle_missing",
    ),
)
def test_cognitive_admission_rejects_each_degraded_critic_fallback(degraded):
    runner = _cognitive_admission_runner()
    for sequence in (1, 2, 3):
        _publish_cognitive_sample(
            runner,
            sequence,
            status_overrides={
                "critic": {
                    "fallback_reason": (
                        f"cost_delta_applied=false;zero_cost_delta;"
                        f"prior_suppressed={degraded};"
                        "maximum_obstacle_cost_delta=0;obstacle_count=0;"
                        "aggregation=max_per_step_mean_horizon"
                    ),
                }
            },
        )
    with pytest.raises(RuntimeError, match="timed out before route dispatch"):
        runner._wait_for_cognitive_admission_ready()


@pytest.mark.parametrize(
    ("planning", "source", "receipt", "required", "expected"),
    (
        ("session", None, None, True, "session"),
        ("session", "session", "session", True, "session"),
        (None, None, None, False, None),
    ),
)
def test_previous_cognitive_session_baseline_accepts_nonempty_consistent_values(
    planning, source, receipt, required, expected,
):
    assert ExperimentRunner._resolve_previous_cognitive_session(
        planning_prior=planning,
        cognitive_source=source,
        ready_receipt=receipt,
        required=required,
    ) == expected


def test_previous_cognitive_session_baseline_rejects_missing_or_mismatch():
    with pytest.raises(RuntimeError, match="baseline is unavailable"):
        ExperimentRunner._resolve_previous_cognitive_session(
            planning_prior=None,
            cognitive_source=None,
            ready_receipt=None,
            required=True,
        )
    with pytest.raises(RuntimeError, match="baseline mismatch"):
        ExperimentRunner._resolve_previous_cognitive_session(
            planning_prior="old",
            cognitive_source="new",
            ready_receipt="old",
            required=True,
        )


def test_cognitive_admission_timeout_cannot_reach_dispatch_and_records_reason():
    runner = _cognitive_admission_runner()
    dispatched = []
    with pytest.raises(RuntimeError, match="timed out before route dispatch"):
        runner._wait_for_cognitive_admission_ready()
        dispatched.append(True)

    evidence = runner._cognitive_admission_snapshot()
    assert dispatched == []
    assert evidence["status"] == "timeout"
    assert evidence["ready"] is False
    assert evidence["reason"].startswith(
        "cognitive_admission_readiness_timeout:"
    )
    summary = {
        "terminal_zero_confirmed": False,
        "reset_receipt": {},
        "reset_receipt_confirmed": False,
        "contact_sensor_evidence_confirmed": False,
        "fixed_map_to_odom_evidence_confirmed": True,
        "data_complete": False,
        "checksums_verified": False,
        "cognitive_admission_readiness": evidence,
    }
    assert "cognitive_admission_readiness_failed" in _episode_validity(summary)[
        "invalid_reasons"
    ]


def test_cognitive_admission_component_off_exemption_comes_from_profile():
    profile = {
        "controller_server": {
            "ros__parameters": {
                "FollowPath": {
                    "critics": ["CognitiveRiskCritic"],
                    "CognitiveRiskCritic": {"enabled": False},
                }
            }
        },
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {
                    "plugins": ["cognitive_obstacle_layer"],
                    "cognitive_obstacle_layer": {"enabled": False},
                }
            }
        },
        "local_costmap": {
            "local_costmap": {
                "ros__parameters": {
                    "plugins": ["cognitive_obstacle_layer"],
                    "cognitive_obstacle_layer": {"mode": "off"},
                }
            }
        },
    }
    assert _cognitive_admission_components(profile) == {}
    runner = _cognitive_admission_runner(components={})
    runner._wait_for_cognitive_admission_ready()
    assert runner._cognitive_admission_snapshot()["status"] == "exempt"


def test_v6_profile_requires_both_layers_and_critic():
    path = (
        Path(__file__).parents[2]
        / "robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml"
    )
    components = _cognitive_admission_components(
        yaml.safe_load(path.read_text(encoding="utf-8"))
    )
    assert components == {
        "global_layer": {"mode": "shadow", "maximum_age_s": 0.5},
        "local_layer": {"mode": "shadow", "maximum_age_s": 0.5},
        "critic": {"mode": "shadow", "maximum_age_s": 0.5},
    }


def _retirement_clearance_runner(*, rejected, fallback_reason):
    runner = object.__new__(ExperimentRunner)
    runner._obstacle_state_stamp_s = 11.0
    runner._obstacle_state = {
        "obstacles": [{"id": "dynamic_box", "state": "retired"}],
        "events": [],
    }
    runner._latest_cognitive_obstacles = SimpleNamespace(
        sequence=9,
        reset_epoch=2,
        recurrent_session_id="session-2",
        map_version="map-v1",
        header=SimpleNamespace(stamp=_retirement_stamp(11.0)),
        validation_stamp=_retirement_stamp(11.0),
        obstacles=[],
    )
    zero_status = lambda consumer: SimpleNamespace(
        consumer=consumer,
        mode="active",
        applied=False,
        rejected=rejected,
        active_cell_count=0,
        fallback_reason=fallback_reason,
        maximum_cost=0,
        raised_cell_count=0,
        maximum_cost_increase=0,
        source_sequence=9,
        reset_epoch=2,
        recurrent_session_id="session-2",
        map_version="map-v1",
        stamp=_retirement_stamp(11.0),
    )
    runner._latest_cognitive_layer_statuses = {
        "/global_costmap/global_costmap:cognitive_obstacle_layer": zero_status(
            "global"
        ),
        "/local_costmap/local_costmap:cognitive_obstacle_layer": zero_status(
            "local"
        ),
    }
    return runner


def _retirement_clearance_observed(runner):
    return runner._dynamic_retirement_clearance_observed(
        {"dynamic_box"}, 10.0, 8, {"global": 8, "local": 8}
    )


@pytest.mark.parametrize(
    ("rejected", "fallback_reason"),
    (
        (False, "rejection_reason=offered"),
        (True, "rejection_reason=no_costmap_cells"),
    ),
    ids=("accepted-offered-zero", "processed-no-cost-zero"),
)
def test_dynamic_retirement_clearance_accepts_two_valid_zero_receipts(
    rejected, fallback_reason
):
    runner = _retirement_clearance_runner(
        rejected=rejected,
        fallback_reason=fallback_reason,
    )

    assert _retirement_clearance_observed(runner)


def test_dynamic_retirement_clearance_accepts_mixed_global_offered_local_processed():
    runner = _retirement_clearance_runner(
        rejected=False,
        fallback_reason="rejection_reason=offered",
    )
    local_status = runner._latest_cognitive_layer_statuses[
        "/local_costmap/local_costmap:cognitive_obstacle_layer"
    ]
    local_status.rejected = True
    local_status.fallback_reason = "rejection_reason=no_costmap_cells"

    assert _retirement_clearance_observed(runner)


def test_dynamic_retirement_clearance_rejects_precompletion_header():
    runner = _retirement_clearance_runner(
        rejected=True,
        fallback_reason="rejection_reason=no_costmap_cells",
    )
    runner._latest_cognitive_obstacles.header.stamp = _retirement_stamp(9.0)
    runner._latest_cognitive_obstacles.validation_stamp = _retirement_stamp(11.0)

    assert not _retirement_clearance_observed(runner)


def test_dynamic_retirement_clearance_rejects_other_rejected_zero_receipt():
    runner = _retirement_clearance_runner(
        rejected=True,
        fallback_reason="rejection_reason=validation_stale",
    )

    assert not _retirement_clearance_observed(runner)


@pytest.mark.parametrize(
    "invalid_evidence",
    (
        "stale-obstacle-state",
        "actor-not-retired",
        "nonempty-source",
        "stale-validation",
        "old-source-sequence",
        "old-consumer-sequence",
        "identity-reset-epoch",
        "identity-session",
        "identity-map",
        "consumer-applied",
        "active-cells-nonzero",
        "maximum-cost-nonzero",
        "raised-cells-nonzero",
        "maximum-cost-increase-nonzero",
        "consumer-mode-mismatch",
        "consumer-status-stale",
        "missing-local-consumer",
    ),
)
def test_dynamic_retirement_clearance_preserves_all_other_gates(
    invalid_evidence,
):
    runner = _retirement_clearance_runner(
        rejected=False,
        fallback_reason="rejection_reason=offered",
    )
    source = runner._latest_cognitive_obstacles
    global_status = runner._latest_cognitive_layer_statuses[
        "/global_costmap/global_costmap:cognitive_obstacle_layer"
    ]

    if invalid_evidence == "stale-obstacle-state":
        runner._obstacle_state_stamp_s = 10.0
    elif invalid_evidence == "actor-not-retired":
        runner._obstacle_state["obstacles"][0]["state"] = "parked"
    elif invalid_evidence == "nonempty-source":
        source.obstacles = [object()]
    elif invalid_evidence == "stale-validation":
        source.validation_stamp = _retirement_stamp(10.0)
    elif invalid_evidence == "old-source-sequence":
        source.sequence = 8
    elif invalid_evidence == "old-consumer-sequence":
        global_status.source_sequence = 8
    elif invalid_evidence == "identity-reset-epoch":
        global_status.reset_epoch = 1
    elif invalid_evidence == "identity-session":
        global_status.recurrent_session_id = "other-session"
    elif invalid_evidence == "identity-map":
        global_status.map_version = "other-map"
    elif invalid_evidence == "consumer-applied":
        global_status.applied = True
    elif invalid_evidence == "active-cells-nonzero":
        global_status.active_cell_count = 1
    elif invalid_evidence == "maximum-cost-nonzero":
        global_status.maximum_cost = 1
    elif invalid_evidence == "raised-cells-nonzero":
        global_status.raised_cell_count = 1
    elif invalid_evidence == "maximum-cost-increase-nonzero":
        global_status.maximum_cost_increase = 1
    elif invalid_evidence == "consumer-mode-mismatch":
        global_status.mode = "fail_open"
    elif invalid_evidence == "consumer-status-stale":
        global_status.stamp = _retirement_stamp(10.0)
    elif invalid_evidence == "missing-local-consumer":
        runner._latest_cognitive_layer_statuses.pop(
            "/local_costmap/local_costmap:cognitive_obstacle_layer"
        )

    assert not _retirement_clearance_observed(runner)


def test_completion_clears_both_costmaps_before_returning():
    class Future:
        @staticmethod
        def result():
            return SimpleNamespace(
                success=True,
                message=json.dumps(
                    {
                        "group": "G2",
                        "retired": ["v6_dynamic_g2_crossing_box"],
                    }
                ),
            )

    class Client:
        @staticmethod
        def wait_for_service(*, timeout_sec):
            return timeout_sec > 0.0

        @staticmethod
        def call_async(_request):
            return Future()

    runner = object.__new__(ExperimentRunner)
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        map_version="v6_kujiale_isaacgen_v1",
    )
    runner._nav2_profile = "v6_low_obstacle_isolation"
    runner._selected_dynamic_trajectories = lambda: (
        {"id": "v6_dynamic_g2_crossing_box", "trigger_group": "G2"},
    )
    runner._selected_dynamic_groups_for_goal = lambda _goal_id: ["G2"]
    runner._obstacle_complete_clients = {"G2": Client()}
    runner._service_timeout_sec = 1.0
    runner._wait_future = lambda _future, _deadline: True
    runner._latest_cognitive_obstacles = SimpleNamespace(sequence=8)
    runner._latest_cognitive_layer_statuses = {}
    runner._clock_seconds = lambda: 10.0
    calls = []
    runner._clear_navigation_costmaps = lambda: calls.append("clear")
    runner._dynamic_retirement_clearance_observed = (
        lambda retired, _barrier, source_cursor, _status_cursors: (
            retired == {"v6_dynamic_g2_crossing_box"} and source_cursor == 8
        )
    )
    def wait_until(predicate, _timeout):
        calls.append("wait")
        return predicate()

    runner._wait_until = wait_until

    assert runner._complete_obstacle_group("G2") == (
        "v6_dynamic_g2_crossing_box",
    )
    assert calls == ["clear", "wait"]

    runner._wait_until = lambda _predicate, _timeout: False
    with pytest.raises(RuntimeError, match="did not clear"):
        runner._complete_obstacle_group("G2")


def test_noncanonical_dynamic_completion_does_not_enter_cognitive_clearance_gate():
    runner = object.__new__(ExperimentRunner)
    runner._nav2_profile = "dynamic_avoidance"
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        map_version="other-map",
    )
    assert not runner._requires_dynamic_retirement_clearance({"dynamic_box"})


def test_dynamic_manifest_requires_successful_completion_return(tmp_path):
    runner, _manifest, _summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=None,
        contact_sensor_collision=False,
    )
    scenario = load_scenario(
        Path(__file__).parents[1] / "config" / "dynamic.yaml"
    )
    runner._scenario = replace(
        scenario,
        obstacle_trajectories=({"id": "crossing_box", "motion": "crossing"},),
        success=replace(
            scenario.success,
            minimum_ground_truth_path_length_m=0.0,
            minimum_reverse_distance_m=0.0,
            maximum_reverse_distance_fraction=1.0,
            minimum_curved_distance_fraction=0.0,
            maximum_stopped_time_fraction=1.0,
        ),
    )
    runner._active_selection = RunSelection(7301, condition_id="dynamic")
    runner._depth_frame = {}
    runner._scan_frame = {}
    runner._local_costmap = object()
    runner._obstacle_samples = [{"id": "crossing_box", "min_clearance_m": 0.2}]
    runner._obstacle_events = [
        {"event": "armed", "obstacle_id": "crossing_box"},
        {"event": "motion_complete", "obstacle_id": "crossing_box"},
        {"event": "park", "obstacle_id": "crossing_box"},
    ]
    runner._leg_results = [{
        "id": "G1",
        "nav2_status": experiment_runner_module.GoalStatus.STATUS_ABORTED,
    }]

    failed = runner._build_manifest(
        run_index=3,
        seed=7301,
        nav2_succeeded=False,
        timed_out=False,
        nav2_status=experiment_runner_module.GoalStatus.STATUS_ABORTED,
        final_still=True,
        runner_error=None,
    )

    assert failed["dynamic_interaction"]["completed_ids"] == []
    assert failed["dynamic_interaction"]["retired_ids"] == []
    assert not failed["dynamic_interaction"]["complete"]
    assert "dynamic_obstacle_interaction_incomplete" in failed["failure_reason"]

    runner._completed_dynamic_obstacle_ids.update(("crossing_box",))
    runner._obstacle_events.append(
        {"event": "goal_reached_retire", "obstacle_id": "crossing_box"}
    )
    completed = runner._build_manifest(
        run_index=3,
        seed=7301,
        nav2_succeeded=True,
        timed_out=False,
        nav2_status=experiment_runner_module.GoalStatus.STATUS_SUCCEEDED,
        final_still=True,
        runner_error=None,
    )

    assert completed["dynamic_interaction"]["completed_ids"] == ["crossing_box"]
    assert completed["dynamic_interaction"]["retired_ids"] == ["crossing_box"]
    assert completed["dynamic_interaction"]["complete"]

    runner._clear_run_state()
    assert runner._completed_dynamic_obstacle_ids == set()


def test_strict_success_counts_single_goal_when_route_is_omitted():
    assert _strict_success_from_leg_count(
        "success", 1, 0, terminal_zero_confirmed=True
    )
    assert _strict_success_from_leg_count(
        "success", 5, 5, terminal_zero_confirmed=True
    )
    assert not _strict_success_from_leg_count(
        "success", 0, 0, terminal_zero_confirmed=True
    )
    assert not _strict_success_from_leg_count(
        "failure", 1, 0, terminal_zero_confirmed=True
    )
    assert not _strict_success_from_leg_count(
        "success", 5, 5, terminal_zero_confirmed=False
    )


def _twist(nonzero=False):
    return SimpleNamespace(
        linear=SimpleNamespace(x=0.2 if nonzero else 0.0, y=0.0),
        angular=SimpleNamespace(z=0.1 if nonzero else 0.0),
    )


def _terminal_zero_runner(
    monkeypatch,
    events,
    *,
    timeout_sec=0.65,
    odom_speed_at=None,
    barrier_source="route_goal_complete",
    barrier_at=0.0,
):
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(
        experiment_runner_module.time, "monotonic", lambda: clock.now
    )
    runner = object.__new__(ExperimentRunner)
    runner._clear_run_state()
    runner._scenario = SimpleNamespace(
        success=SimpleNamespace(
            final_still_timeout_sec=timeout_sec,
            final_still_duration_sec=0.15,
            final_linear_speed_mps=0.02,
            final_angular_speed_radps=0.05,
        )
    )
    runner._odom_max_age_sec = 0.5
    runner._raise_if_shutdown = lambda: None
    pending = list(events)

    def spin_once(timeout_sec):
        clock.now += timeout_sec
        if (
            barrier_source is not None
            and runner._terminal_zero_barrier_monotonic is None
            and clock.now >= 10.0 + barrier_at
        ):
            runner._mark_terminal_zero_barrier(barrier_source)
        speed = (
            odom_speed_at(clock.now - 10.0)
            if odom_speed_at is not None
            else 0.0
        )
        runner._odom_samples.append(
            OdometrySample(
                0.0, 0.0, 0.0, speed, 0.0, clock.now, clock.now
            )
        )
        while pending and clock.now >= 10.0 + pending[0][0]:
            _offset, nonzero = pending.pop(0)
            runner._actuator_command_callback(_twist(nonzero))

    runner._spin_once = spin_once
    runner._test_clock = clock
    runner._start_terminal_zero_observation()
    if barrier_source is not None and barrier_at == 0.0:
        runner._mark_terminal_zero_barrier(barrier_source)
    return runner


def test_terminal_zero_is_part_of_manifest_result_without_erasing_other_failures():
    assert _result_with_terminal_zero([], True) == ("success", [])
    assert _result_with_terminal_zero([], False) == (
        "failure",
        ["terminal_zero_not_confirmed"],
    )
    assert _result_with_terminal_zero(
        ["collision_detected", "timed_out"], False
    ) == (
        "failure",
        ["collision_detected", "timed_out", "terminal_zero_not_confirmed"],
    )


def test_terminal_zero_immediate_repeated_quiet_window_passes(monkeypatch):
    runner = _terminal_zero_runner(
        monkeypatch,
        ((0.05, False), (0.22, False)),
    )

    assert runner._wait_for_final_stillness()
    assert runner._terminal_zero_confirmed
    assert runner._terminal_zero_reason == "terminal_zero_confirmed"
    timing = runner._terminal_zero_timing()
    assert timing["barrier_source"] == "route_goal_complete"
    assert timing["first_zero_after_terminal_sec"] <= 0.10
    assert timing["last_zero_after_terminal_sec"] >= 0.20
    assert timing["confirming_zero_sample_count"] == 2


def test_terminal_zero_ignores_zeros_before_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    runner._test_clock.now = 10.02
    runner._actuator_command_callback(_twist())
    runner._test_clock.now = 10.18
    runner._actuator_command_callback(_twist())
    runner._test_clock.now = 10.20
    runner._mark_terminal_zero_barrier("route_goal_complete")

    assert not runner._terminal_zero_observation_complete(10.20, 0.15)
    assert runner._terminal_zero_reason == "terminal_zero_not_observed"
    assert runner._terminal_zero_timing()["observed_zero_sample_count"] == 0


def _expect_route_completion(runner, *, next_epoch, leg_id, final_leg):
    runner._route_goal_complete_epoch = next_epoch - 1
    runner._terminal_zero_expected_route_completion_epoch = next_epoch
    runner._terminal_zero_expected_route_leg_id = leg_id
    runner._terminal_zero_expected_route_leg_is_final = final_leg


def test_intermediate_success_clears_epoch_without_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G3", final_leg=False
    )
    runner._test_clock.now = 10.20

    runner._route_goal_complete_callback(SimpleNamespace(data=True))

    assert runner._route_goal_complete_epoch == 5
    assert runner._latest_route_goal_complete
    assert runner._terminal_zero_barrier_monotonic is None
    assert runner._terminal_zero_expected_route_completion_epoch is None
    assert runner._terminal_zero_expected_route_leg_id is None


def test_intermediate_false_records_episode_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G3", final_leg=False
    )
    runner._test_clock.now = 10.20

    runner._route_goal_complete_callback(SimpleNamespace(data=False))

    assert runner._terminal_zero_barrier_monotonic == pytest.approx(10.20)
    assert runner._terminal_zero_barrier_source == "route_goal_complete"
    assert runner._terminal_zero_barrier_leg_id == "G3"


def test_timeout_cancel_fresh_false_records_matching_leg_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=8, leg_id="G4", final_leg=False
    )
    runner._test_clock.now = 10.40

    runner._route_goal_complete_callback(SimpleNamespace(data=False))

    assert runner._route_goal_complete_epoch == 8
    assert runner._terminal_zero_barrier_monotonic == pytest.approx(10.40)
    assert runner._terminal_zero_barrier_leg_id == "G4"


def test_stale_duplicate_completion_is_ignored_after_epoch_consumed(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G3", final_leg=False
    )
    runner._route_goal_complete_callback(SimpleNamespace(data=True))
    runner._test_clock.now = 10.30

    runner._route_goal_complete_callback(SimpleNamespace(data=False))

    assert runner._route_goal_complete_epoch == 6
    assert runner._terminal_zero_barrier_monotonic is None


def test_fresh_final_success_records_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G1", final_leg=True
    )
    runner._test_clock.now = 10.20

    runner._route_goal_complete_callback(SimpleNamespace(data=True))

    assert runner._terminal_zero_barrier_monotonic == pytest.approx(10.20)
    assert runner._terminal_zero_barrier_source == "route_goal_complete"
    assert runner._terminal_zero_barrier_leg_id == "G1"


def test_terminal_zero_rejects_first_zero_later_than_100ms(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, ())
    runner._test_clock.now = 10.11
    runner._actuator_command_callback(_twist())
    runner._test_clock.now = 10.30
    runner._actuator_command_callback(_twist())

    assert not runner._terminal_zero_observation_complete(10.30, 0.15)
    assert runner._terminal_zero_reason == "terminal_first_zero_late"


def test_terminal_zero_rejects_any_nonzero_tail_after_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, ())
    for offset, nonzero in (
        (0.02, False),
        (0.08, True),
        (0.09, False),
        (0.30, False),
    ):
        runner._test_clock.now = 10.0 + offset
        runner._actuator_command_callback(_twist(nonzero))

    assert not runner._terminal_zero_observation_complete(10.30, 0.15)
    assert runner._terminal_zero_reason == "terminal_nonzero_after_barrier"


def test_terminal_zero_single_zero_then_timeout_does_not_erase_final_stillness(
    monkeypatch,
):
    runner = _terminal_zero_runner(
        monkeypatch, ((0.05, False),), timeout_sec=0.35
    )

    assert runner._wait_for_final_stillness()
    assert not runner._terminal_zero_confirmed
    assert runner._terminal_zero_reason == "terminal_zero_timeout"
    assert runner._terminal_zero_timing()["observed_zero_sample_count"] == 1


def test_terminal_zero_wait_restarts_full_odom_window_after_late_motion(
    monkeypatch,
):
    runner = _terminal_zero_runner(
        monkeypatch,
        ((0.30, False), (0.50, False), (0.60, False), (0.75, False)),
        timeout_sec=0.90,
        odom_speed_at=lambda offset: 0.2 if 0.25 <= offset < 0.50 else 0.0,
        barrier_at=0.25,
    )

    assert runner._wait_for_final_stillness()
    assert runner._test_clock.now >= 10.65
    assert runner._terminal_zero_confirmed


def test_direct_backend_action_return_barrier_uses_same_contract(monkeypatch):
    runner = _terminal_zero_runner(
        monkeypatch,
        ((0.05, False), (0.22, False)),
        barrier_source="navigate_action_return",
    )

    assert runner._wait_for_final_stillness()
    assert runner._terminal_zero_timing()["barrier_source"] == (
        "navigate_action_return"
    )


def test_cross_reset_completion_callback_cannot_mark_terminal_barrier(monkeypatch):
    runner = _terminal_zero_runner(monkeypatch, (), barrier_source=None)
    _expect_route_completion(
        runner, next_epoch=5, leg_id="G3", final_leg=False
    )

    runner._clear_run_state()
    runner._route_goal_complete_callback(SimpleNamespace(data=False))

    assert runner._route_goal_complete_epoch == 5
    assert runner._terminal_zero_barrier_monotonic is None
    assert runner._terminal_zero_expected_route_completion_epoch is None


def test_clear_run_state_resets_terminal_zero_observation_fields():
    runner = object.__new__(ExperimentRunner)
    runner._cognitive_admission_components = {
        "global_layer": {"mode": "active"}
    }
    runner._clear_run_state()
    runner._terminal_zero_observation_started_monotonic = 1.0
    runner._terminal_zero_barrier_monotonic = 1.05
    runner._terminal_zero_barrier_source = "route_goal_complete"
    runner._terminal_zero_barrier_leg_id = "G3"
    runner._terminal_zero_expected_route_completion_epoch = 9
    runner._terminal_zero_expected_route_leg_id = "G4"
    runner._terminal_zero_expected_route_leg_is_final = True
    runner._terminal_zero_confirmed_monotonic = 2.0
    runner._terminal_zero_first_zero_monotonic = 1.1
    runner._terminal_zero_last_zero_monotonic = 1.9
    runner._terminal_zero_confirming_sample_count = 4
    runner._terminal_zero_confirmed = True
    runner._terminal_zero_reason = "terminal_zero_confirmed"
    runner._cmd_vel_sim_last_receive_monotonic = 2.0
    runner._cmd_vel_sim_last_nonzero_monotonic = 1.2
    runner._cmd_vel_sim_zero_stamps = [1.3, 1.9]
    runner._cognitive_admission_sources[(2, "session", "map", 7)] = {
        "validation_stamp_s": 2.0,
        "received_at": 3.0,
    }
    runner._cognitive_admission_streaks["global_layer"] = 3
    runner._cognitive_admission_last_sequences["global_layer"] = 7

    runner._clear_run_state()

    assert runner._terminal_zero_observation_started_monotonic is None
    assert runner._terminal_zero_barrier_monotonic is None
    assert runner._terminal_zero_barrier_source == "not_observed"
    assert runner._terminal_zero_barrier_leg_id is None
    assert runner._terminal_zero_expected_route_completion_epoch is None
    assert runner._terminal_zero_expected_route_leg_id is None
    assert not runner._terminal_zero_expected_route_leg_is_final
    assert runner._terminal_zero_confirmed_monotonic is None
    assert runner._terminal_zero_first_zero_monotonic is None
    assert runner._terminal_zero_last_zero_monotonic is None
    assert runner._terminal_zero_confirming_sample_count == 0
    assert runner._cognitive_admission_sources == {}
    assert runner._cognitive_admission_streaks == {"global_layer": 0}
    assert runner._cognitive_admission_last_sequences == {"global_layer": -1}
    assert runner._cognitive_admission_barrier_ros_s is None
    assert runner._cognitive_admission_expected_reset_epoch is None
    assert runner._cognitive_admission_forbidden_recurrent_session_id is None
    assert not runner._terminal_zero_confirmed
    assert runner._terminal_zero_reason == "not_checked"
    assert runner._cmd_vel_sim_last_receive_monotonic is None
    assert runner._cmd_vel_sim_last_nonzero_monotonic is None
    assert runner._cmd_vel_sim_zero_stamps == []


def test_motion_quality_measures_reverse_curves_and_turn_reversals():
    samples = [
        CommandSample(0.30, 0.60, 0.0),
        CommandSample(0.30, 0.60, 0.1),
        CommandSample(-0.20, -0.50, 0.2),
        CommandSample(-0.20, -0.50, 0.3),
    ]
    metrics = ExperimentRunner._motion_quality_metrics(samples)
    assert metrics["translated_distance_m"] == pytest.approx(0.08)
    assert metrics["reverse_distance_m"] == pytest.approx(0.02)
    assert metrics["reverse_distance_fraction"] == pytest.approx(0.25)
    assert metrics["curved_distance_fraction"] == pytest.approx(1.0)
    assert metrics["angular_direction_changes"] == 1
    assert metrics["stopped_time_fraction"] == pytest.approx(0.0)


def test_motion_quality_ignores_large_timestamp_gaps():
    samples = [
        CommandSample(0.50, 1.00, 0.0),
        CommandSample(-0.50, -1.00, 1.0),
    ]
    metrics = ExperimentRunner._motion_quality_metrics(samples)
    assert metrics["observed_duration_sec"] == 0.0
    assert metrics["translated_distance_m"] == 0.0
    assert metrics["maximum_linear_acceleration_mps2"] == 0.0


def test_same_direction_overtake_requires_lateral_bypass_and_passing():
    ground_truth = [
        OdometrySample(-0.80, -1.00, 0.0, 0.5, 0.0, 1.00, 0.0),
        OdometrySample(-0.82, -0.20, 0.0, 0.5, 0.0, 1.10, 0.0),
        OdometrySample(-0.45, 0.70, 0.0, 0.5, 0.0, 1.20, 0.0),
    ]
    actor = [
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.45, -0.60, 0.5]},
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.10, "position": [-0.45, -0.10, 0.5]},
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.20, "position": [-0.45, 0.20, 0.5]},
    ]

    metrics = ExperimentRunner._same_direction_overtake_metrics(
        ground_truth, actor, "same_direction_slow_actor"
    )

    assert metrics["lateral_bypass_seen"]
    assert metrics["passed_while_moving"]
    assert metrics["passed_before_actor_yielded_right"]
    assert metrics["complete"]


def test_same_direction_waiting_is_not_an_overtake():
    ground_truth = [
        OdometrySample(-0.45, -1.10, 0.0, 0.0, 0.0, 1.00, 0.0),
        OdometrySample(-0.45, -1.10, 0.0, 0.0, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.45, -0.60, 0.5]},
        {"id": "same_direction_slow_actor", "state": "moving", "stamp_s": 1.10, "position": [-0.45, -0.58, 0.5]},
    ]

    metrics = ExperimentRunner._same_direction_overtake_metrics(
        ground_truth, actor, "same_direction_slow_actor"
    )

    assert not metrics["lateral_bypass_seen"]
    assert not metrics["passed_while_moving"]
    assert not metrics["passed_before_actor_yielded_right"]
    assert not metrics["complete"]


def test_local_bypass_requires_passing_to_the_actor_right():
    ground_truth = [
        OdometrySample(0.20, 0.35, 0.0, 0.4, 0.0, 1.00, 0.0),
        OdometrySample(0.25, 0.95, 0.0, 0.4, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "local_bypass_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.20, 0.45, 0.5]},
        {"id": "local_bypass_actor", "state": "moving", "stamp_s": 1.10, "position": [-0.10, 0.45, 0.5]},
    ]

    metrics = ExperimentRunner._local_right_bypass_metrics(
        ground_truth, actor, "local_bypass_actor"
    )

    assert metrics["right_side_bypass_seen"]
    assert metrics["passed_while_moving"]
    assert metrics["complete"]


def test_local_bypass_accepts_a_pass_after_the_planned_park():
    ground_truth = [
        OdometrySample(0.30, 0.40, 0.0, 0.3, 0.0, 1.00, 0.0),
        OdometrySample(0.32, 0.92, 0.0, 0.3, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "local_bypass_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.20, 0.45, 0.5]},
        {"id": "local_bypass_actor", "state": "parked", "stamp_s": 1.10, "position": [-0.10, 0.45, 0.5]},
    ]

    metrics = ExperimentRunner._local_right_bypass_metrics(
        ground_truth, actor, "local_bypass_actor"
    )

    assert metrics["planned_park_seen"]
    assert metrics["right_side_bypass_seen"]
    assert not metrics["passed_while_moving"]
    assert metrics["passed_after_planned_park"]
    assert metrics["complete"]


def test_g2_g3_exit_requires_following_then_left_exit_turn():
    ground_truth = [
        OdometrySample(-0.42, 1.60, 0.0, 0.4, 0.0, 1.00, 0.0),
        OdometrySample(-0.85, -0.55, 0.0, 0.4, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "g2_g3_exit_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.40, 1.00, 0.5]},
        {"id": "g2_g3_exit_actor", "state": "parked", "stamp_s": 1.10, "position": [-0.40, -0.70, 0.5]},
    ]

    metrics = ExperimentRunner._g2_g3_exit_metrics(
        ground_truth, actor, "g2_g3_exit_actor"
    )

    assert metrics["continuous_follow_seen"]
    assert metrics["outlet_left_turn_seen"]
    assert metrics["complete"]


def test_g2_g3_exit_accepts_calibrated_one_point_four_metre_following_gap():
    ground_truth = [
        OdometrySample(-0.42, 2.40, 0.0, 0.4, 0.0, 1.00, 0.0),
        OdometrySample(-0.85, -0.55, 0.0, 0.4, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "g2_g3_exit_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.40, 1.00, 0.5]},
        {"id": "g2_g3_exit_actor", "state": "parked", "stamp_s": 1.10, "position": [-0.40, -0.70, 0.5]},
    ]

    metrics = ExperimentRunner._g2_g3_exit_metrics(
        ground_truth, actor, "g2_g3_exit_actor"
    )

    assert metrics["continuous_follow_seen"]
    assert metrics["complete"]


def test_g2_g3_exit_rejects_a_following_gap_beyond_the_calibrated_window():
    ground_truth = [
        OdometrySample(-0.42, 2.41, 0.0, 0.4, 0.0, 1.00, 0.0),
        OdometrySample(-0.85, -0.55, 0.0, 0.4, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "g2_g3_exit_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.40, 1.00, 0.5]},
        {"id": "g2_g3_exit_actor", "state": "parked", "stamp_s": 1.10, "position": [-0.40, -0.70, 0.5]},
    ]

    metrics = ExperimentRunner._g2_g3_exit_metrics(
        ground_truth, actor, "g2_g3_exit_actor"
    )

    assert not metrics["continuous_follow_seen"]
    assert not metrics["complete"]


def test_valid_product_failure_is_preserved_and_blocks_successful_resume(tmp_path):
    root = tmp_path / "run-0002-seed-7301"
    root.mkdir()
    manifest = {
        "random_seed": 7301,
        "run_index": 2,
        "condition_id": "dynamic_appearance",
        "appearance": {"profile_id": "dim_warm"},
        "dynamic_selection": {"case_id": "full_route_three_stage", "variant_id": "v1"},
        "result": "failure",
        "terminal_zero_confirmed": True,
        "terminal_zero_reason": "terminal_zero_confirmed",
    }
    summary = {
        "data_complete": True,
        "checksums_verified": True,
        "strict_success": False,
        "terminal_zero_confirmed": True,
        "episode_validity": {"valid": True},
        "final_trial_metric_gate": {"passed": True},
    }
    (root / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (root / "run_summary.json").write_text(json.dumps(summary), encoding="utf-8")
    checksums = [
        f"{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}"
        for path in sorted(root.iterdir())
    ]
    (root / "checksums.sha256").write_text("\n".join(checksums) + "\n", encoding="utf-8")
    selection = SimpleNamespace(
        seed=7301,
        condition_id="dynamic_appearance",
        appearance_profile_id="dim_warm",
        case_id="full_route_three_stage",
        variant_id="v1",
    )
    runner = object.__new__(ExperimentRunner)

    runner._require_successful_resume = False
    assert runner._completed_resume_manifest(root, 2, selection) == manifest
    runner._require_successful_resume = True
    with pytest.raises(ConfigurationError, match="immutable valid product failure"):
        runner._completed_resume_manifest(root, 2, selection)


def _static_sat_evidence_run(
    tmp_path: Path,
    *,
    obstacle_position: tuple[float, float] | None,
    contact_sensor_collision: bool,
    nav2_succeeded: bool = True,
    collision_monitor_stop: bool = False,
    condition_stack_id: str = "",
    stack_session_id: str = "",
    formal_freeze_digest: str = "",
    path_deviation_percent: float | None = None,
    cognitive_admission_timeout: bool = False,
):
    scenario = load_scenario(
        Path(__file__).parents[1] / "config" / "static.yaml"
    )
    scenario = replace(
        scenario,
        obstacles={
            "layout_id": "sat_diagnostic_fixture",
            "static": [{"id": "low_box"}],
            "trajectories": [],
        },
        success=replace(
            scenario.success,
            minimum_ground_truth_path_length_m=0.0,
            minimum_reverse_distance_m=0.0,
            maximum_reverse_distance_fraction=1.0,
            minimum_curved_distance_fraction=0.0,
            maximum_stopped_time_fraction=1.0,
        ),
    )
    runner = object.__new__(ExperimentRunner)
    runner._clear_run_state()
    runner._scenario = scenario
    runner._active_selection = RunSelection(7301, condition_id="static")
    runner._spawn_pose = SimpleNamespace(
        name="mapping_start",
        usd=SimpleNamespace(as_dict=lambda: {}),
        map=SimpleNamespace(as_dict=lambda: {}),
    )
    runner._ground_truth_samples = [
        OdometrySample(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    ]
    if path_deviation_percent is not None:
        actual_length = 10.0 * (1.0 + path_deviation_percent / 100.0)
        runner._ground_truth_samples = [
            OdometrySample(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
            OdometrySample(actual_length, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0),
        ]
    runner._odom_samples = [
        OdometrySample(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    ]
    runner._leg_results = [{
        "id": "G1",
        "nav2_status": experiment_runner_module.GoalStatus.STATUS_SUCCEEDED,
    }]
    runner._robot_footprint = (
        (0.255, 0.210),
        (0.255, -0.210),
        (-0.230, -0.210),
        (-0.230, 0.210),
    )
    if obstacle_position is not None:
        runner._obstacle_state = {
            "obstacles": [{
                "id": "low_box",
                "position": [*obstacle_position, 0.08],
                "position_frame": "map",
                "size": [0.30, 0.30, 0.16],
                "retired": False,
            }],
            "events": [],
        }
    runner._collision_seen = False
    runner._collision_callback(SimpleNamespace(data=contact_sensor_collision))
    runner._collision_monitor_active = True
    runner._localization_seen = True
    runner._tf_ever_available = True
    runner._terminal_zero_confirmed = True
    runner._terminal_zero_reason = "terminal_zero_confirmed"
    runner._navigation_start_stamp_s = 0.5
    runner._navigation_end_stamp_s = 1.5
    runner._dynamic_runtime_contract = {"verified": True}
    runner._appearance_runtime_contract = {"verified": True}
    runner._appearance_state = None
    runner._appearance_config_hash = None
    runner._optimal_reference = None
    runner._optimal_reference_hash = None
    if path_deviation_percent is not None:
        runner._optimal_reference = {
            "total_length_m_0_05": 10.0,
            "legs": [{"id": "G1", "length_m_0_05": 10.0}],
        }
        runner._optimal_reference_hash = "reference"
    runner._navigation_graph = None
    runner._minimum_safety_scan_range_m = None
    runner._provenance = {}
    runner._robot_config_hash = "robot"
    runner._nav2_config_hash = "nav2"
    runner._scenario_runtime_hashes = {
        "robot_config": "robot",
        "nav2_config": "nav2",
    }
    runner._nav2_profile = "stable"
    runner._clear_slam_localization_buffer = True
    runner._reset_map_base_translation_tolerance_m = 0.05
    runner._experiment_arm = ""
    runner._navigation_execution_backend = "navigate_to_pose"
    runner._condition_stack_id = condition_stack_id
    runner._stack_session_id = stack_session_id
    runner._formal_freeze_digest = formal_freeze_digest
    runner._reset_receipt = {"generation": 1}
    runner._record_bag = False
    runner._fail_stop_metric_contract = None
    if cognitive_admission_timeout:
        runner._cognitive_admission_components = {
            "global_layer": {"mode": "active", "maximum_age_s": 0.5},
            "local_layer": {"mode": "active", "maximum_age_s": 0.5},
            "critic": {"mode": "active", "maximum_age_s": 0.5},
        }
        runner._reset_cognitive_admission_state(
            barrier_ros_s=10.0,
            barrier_monotonic=0.0,
            expected_reset_epoch=2,
        )
        runner._cognitive_admission_result_status = "timeout"
        runner._cognitive_admission_result_reason = (
            "cognitive_admission_readiness_timeout:critic=0,global_layer=0,local_layer=0"
        )
    if collision_monitor_stop:
        runner._collision_lock_timeout_sec = 0.0
        runner._lookup_fresh_map_to_odom = lambda: None
        runner._collision_lock_callback(
            SimpleNamespace(action_type=CollisionMonitorState.STOP)
        )
        runner._update_health()

    nav2_status = (
        experiment_runner_module.GoalStatus.STATUS_SUCCEEDED
        if nav2_succeeded
        else experiment_runner_module.GoalStatus.STATUS_ABORTED
    )
    manifest = runner._build_manifest(
        run_index=2,
        seed=7301,
        nav2_succeeded=nav2_succeeded,
        timed_out=False,
        nav2_status=nav2_status,
        final_still=True,
        runner_error=None,
    )
    manifest["dynamic_selection"] = {"case_id": None, "variant_id": None}

    root = tmp_path / "run-0002-seed-7301"
    root.mkdir()
    if not cognitive_admission_timeout:
        (root / "TRIAL_DISPATCHED.json").write_text("{}\n", encoding="utf-8")

    def write_files(names):
        for name in names:
            (root / name).write_text("fixture\n", encoding="utf-8")
        return True

    runner._write_depth_snapshot = lambda _root: write_files(
        ("depth_frame.pgm", "depth_frame.json")
    )
    runner._write_costmap_snapshot = lambda _root, name, _grid: write_files(
        (f"{name}.pgm", f"{name}.json")
    )
    runner._write_scan_snapshot = lambda _root, stem, _frame: write_files(
        (f"{stem}.csv", f"{stem}.json")
    )
    summary = runner._write_run_evidence(
        manifest, 7301, 2, root, bag_complete=False
    )
    return runner, manifest, summary, root


def test_cognitive_admission_timeout_is_manifested_as_startup_invalid(tmp_path):
    _runner, manifest, summary, root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=None,
        contact_sensor_collision=False,
        cognitive_admission_timeout=True,
    )

    assert not (root / "TRIAL_DISPATCHED.json").exists()
    assert manifest["cognitive_admission_readiness"]["status"] == "timeout"
    assert manifest["cognitive_admission_readiness"]["ready"] is False
    assert summary["startup_invalid_reason"].startswith(
        "cognitive_admission_readiness_timeout:"
    )
    assert "cognitive_admission_readiness_failed" in summary[
        "episode_validity"
    ]["invalid_reasons"]


def test_run_evidence_records_condition_stack_attestation(tmp_path):
    session_id = "a" * 64
    freeze_digest = "b" * 64
    _runner, manifest, summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=None,
        contact_sensor_collision=False,
        condition_stack_id="indoor_static",
        stack_session_id=session_id,
        formal_freeze_digest=freeze_digest,
    )

    assert manifest["condition_stack_id"] == "indoor_static"
    assert manifest["stack_session_id"] == session_id
    assert manifest["formal_freeze_digest"] == freeze_digest
    assert manifest["scenario_runtime_hashes"] == {
        "robot_config": "robot",
        "nav2_config": "nav2",
    }
    assert manifest["condition_stack_attestation"]["confirmed"] is True
    assert summary["condition_stack_id"] == "indoor_static"
    assert summary["stack_session_id"] == session_id
    assert summary["formal_freeze_digest"] == freeze_digest
    assert summary["episode_validity"]["valid"] is True


def test_exactly_twenty_percent_executed_path_deviation_is_a_failure(tmp_path):
    _runner, manifest, summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=None,
        contact_sensor_collision=False,
        path_deviation_percent=20.0,
    )

    assert manifest["metrics"]["path_deviation_percent"] == pytest.approx(20.0)
    assert "ground_truth_path_deviation_exceeds_20_percent" in manifest[
        "failure_reason"
    ]
    assert summary["strict_success"] is False
    assert summary["episode_validity"]["valid"] is True


def test_stack_local_episode_sequence_claims_fresh_cold_then_hot_receipts(tmp_path):
    sequence_path = tmp_path / "episode.sequence.json"
    sequence_path.write_text(json.dumps({
        "schema": "bio_nav.v6_stack_episode_sequence.v1",
        "stack_session_id": "a" * 64,
        "last_sequence": 0,
        "startup_reset_generation_baseline": 1,
    }), encoding="utf-8")
    runner = object.__new__(ExperimentRunner)
    runner._stack_session_id = "a" * 64
    runner._condition_stack_contract = {
        "episode_sequence_path": str(sequence_path),
        "t2_selector_path": "/module3/scripts/run_v6_kujiale_low_obstacles.sh",
        "t2_selector_sha256": "b" * 64,
    }

    receipts = [
        runner._claim_stack_episode_sequence(
            pre_reset_generation=rep,
            reset_generation=rep + 1,
        )
        for rep in range(1, 4)
    ]

    assert [receipt["sequence"] for receipt in receipts] == [1, 2, 3]
    assert all(receipt["baseline"] == 1 for receipt in receipts)
    assert all(receipt["stack_session_id"] == "a" * 64 for receipt in receipts)
    state = json.loads(sequence_path.read_text())
    assert state["last_sequence"] == 3


def test_stack_episode_sequence_failure_does_not_consume_receipt(tmp_path):
    sequence_path = tmp_path / "episode.sequence.json"
    initial = {
        "schema": "bio_nav.v6_stack_episode_sequence.v1",
        "stack_session_id": "a" * 64,
        "last_sequence": 0,
        "startup_reset_generation_baseline": 1,
    }
    sequence_path.write_text(json.dumps(initial), encoding="utf-8")
    runner = object.__new__(ExperimentRunner)
    runner._stack_session_id = "a" * 64
    runner._condition_stack_contract = {
        "episode_sequence_path": str(sequence_path),
        "t2_selector_path": "/selector",
        "t2_selector_sha256": "b" * 64,
    }

    with pytest.raises(ConfigurationError, match="fresh-start sequence"):
        runner._claim_stack_episode_sequence(
            pre_reset_generation=2,
            reset_generation=3,
        )

    assert json.loads(sequence_path.read_text()) == initial


def test_attested_episode_snapshots_stack_contract_into_evidence_root(tmp_path):
    contract = tmp_path / "runtime" / "stack.contract.json"
    contract.parent.mkdir()
    contract.write_text('{"schema":"bio_nav.v6_stack_contract.v1"}\n', encoding="utf-8")
    runner = object.__new__(ExperimentRunner)
    runner._output_directory = tmp_path / "evidence"
    runner._scenario = load_scenario(
        Path(__file__).parents[1] / "config" / "static.yaml"
    )
    runner._condition_stack_contract_path = contract
    runner._record_bag = False

    root = runner._begin_run_evidence(1, 7301)

    assert (root / "stack_contract.json").read_bytes() == contract.read_bytes()


def test_sat_overlap_is_diagnostic_when_contact_sensor_is_clear(tmp_path):
    runner, manifest, summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=(1.25, 0.0),
        contact_sensor_collision=False,
    )

    geometric = manifest["static_geometric_contact"]
    assert geometric["observed"]
    assert geometric["maximum_sat_overlap_m"] > 0.001
    assert geometric["exceeds_acceptance_overlap"]
    assert geometric["diagnostic_only"]
    assert not runner._collision_detected
    assert manifest["result"] == "success"
    assert manifest["failure_reason"] == ""
    assert summary["strict_success"]
    assert summary["physical_collision_free"]
    assert not summary["isaac_contact_sensor_collision_detected"]


def test_contact_sensor_collision_fails_even_when_sat_overlap_is_zero(tmp_path):
    runner, manifest, summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=(5.0, 0.0),
        contact_sensor_collision=True,
    )

    geometric = manifest["static_geometric_contact"]
    assert geometric["observed"]
    assert geometric["maximum_sat_overlap_m"] == 0.0
    assert runner._isaac_contact_sensor_collision_detected
    assert manifest["result"] == "failure"
    assert manifest["failure_reason"] == "collision_detected"
    assert not summary["strict_success"]
    assert not summary["physical_collision_free"]
    assert summary["isaac_contact_sensor_collision_detected"]


def test_collision_monitor_stop_is_navigation_failure_not_physical_collision(
    tmp_path,
):
    runner, manifest, summary, _root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=(5.0, 0.0),
        contact_sensor_collision=False,
        nav2_succeeded=False,
        collision_monitor_stop=True,
    )

    assert runner._collision_monitor_locked
    assert manifest["result"] == "failure"
    assert "nav2_action_failed" in manifest["failure_reason"]
    assert "collision_monitor_locked" in manifest["failure_reason"]
    assert "collision_detected" not in manifest["failure_reason"]
    assert not summary["strict_success"]
    assert summary["physical_collision_free"]
    assert not summary["isaac_contact_sensor_collision_detected"]


@pytest.mark.parametrize(
    ("obstacle_position", "warning_expected"),
    [
        (None, False),
        ((1.25, 0.0), True),
    ],
)
def test_sat_diagnostic_presence_does_not_change_complete_resume_eligibility(
    tmp_path, obstacle_position, warning_expected
):
    runner, manifest, summary, root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=obstacle_position,
        contact_sensor_collision=False,
    )
    geometric = manifest["static_geometric_contact"]
    assert geometric["observed"] is warning_expected
    assert bool(manifest["warning_reason"]) is warning_expected
    assert summary["warning_reason"] == manifest["warning_reason"]
    assert summary["data_complete"]
    assert summary["checksums_verified"]
    assert summary["strict_success"]
    assert summary["physical_collision_free"]
    selection = SimpleNamespace(
        seed=7301,
        condition_id="static",
        appearance_profile_id=None,
        case_id=None,
        variant_id=None,
    )
    runner._require_successful_resume = True

    assert runner._completed_resume_manifest(root, 2, selection) == manifest


def test_successful_resume_blocks_failed_final_metric_gate(tmp_path):
    runner, manifest, summary, root = _static_sat_evidence_run(
        tmp_path,
        obstacle_position=None,
        contact_sensor_collision=False,
    )
    summary["final_trial_metric_gate"] = {"applicable": True, "passed": False}
    ExperimentRunner._finalize_checksums(root, summary, manifest)
    selection = SimpleNamespace(
        seed=7301,
        condition_id="static",
        appearance_profile_id=None,
        case_id=None,
        variant_id=None,
    )
    runner._require_successful_resume = True

    with pytest.raises(ConfigurationError, match="immutable valid product failure"):
        runner._completed_resume_manifest(root, 2, selection)


def test_sat_diagnostics_do_not_change_collision_rate_statistics():
    records = []
    for seed in range(20):
        summary = {
            "strict_success": True,
            "physical_collision_free": True,
            "data_complete": True,
            "checksums_verified": True,
            "path_deviation_percent": 0.0,
            "legs": [{"id": f"G{index}"} for index in range(1, 6)],
        }
        if seed % 2:
            summary["static_geometric_contact"] = {
                "observed": True,
                "contact_detected": True,
                "maximum_sat_overlap_m": 0.12,
                "diagnostic_only": True,
            }
        records.append({"summary": summary, "manifest": {}})

    result = _rate_group(
        records,
        name="static",
        required_rate_percent=95.0,
        require_path_deviation=True,
    )

    assert result["strict_successes"] == 20
    assert result["collision_free_runs"] == 20
    assert result["collision_free_rate_percent"] == 100.0
    assert result["complete_evidence_runs"] == 20
    assert result["passed"]


def test_checksum_finalization_updates_summary_and_covers_final_bytes(tmp_path):
    root = tmp_path / "run-0001-seed-19301"
    root.mkdir()
    summary = {"checksums_verified": False, "strict_success": True}
    (root / "run_summary.json").write_text(json.dumps(summary), encoding="utf-8")
    (root / "evidence.json").write_text('{"complete": true}\n', encoding="utf-8")

    ExperimentRunner._finalize_checksums(root, summary)

    stored_summary = json.loads((root / "run_summary.json").read_text(encoding="utf-8"))
    assert summary["checksums_verified"] is True
    assert stored_summary["checksums_verified"] is True
    assert ExperimentRunner._checksums_are_verified(root)

    (root / "unlisted-required.json").write_text("{}\n", encoding="utf-8")
    assert not ExperimentRunner._checksums_are_verified(root)


def test_shared_checksum_verifier_rejects_missing_inventory_entry(tmp_path):
    root = tmp_path / "run"
    root.mkdir()
    first = root / "run_summary.json"
    second = root / "run_manifest.json"
    first.write_text("{}\n", encoding="utf-8")
    second.write_text("{}\n", encoding="utf-8")
    (root / "checksums.sha256").write_text(
        f"{hashlib.sha256(first.read_bytes()).hexdigest()}  {first.name}\n",
        encoding="utf-8",
    )

    assert not ExperimentRunner._checksums_are_verified(root)


def test_checksum_finalization_covers_final_acceptance_summary_and_manifest(tmp_path):
    root = tmp_path / "run-0001-seed-19301"
    root.mkdir()
    summary = {
        "navigation_contract_success": True,
        "strict_success": False,
        "terminal_zero_confirmed": True,
        "reset_receipt": {"generation": 1},
        "reset_receipt_confirmed": True,
        "physical_collision_free": True,
        "contact_sensor_evidence_confirmed": True,
        "fixed_map_to_odom_evidence_confirmed": True,
        "data_complete": True,
        "checksums_verified": False,
        "required_topic_coverage": {"required": True, "passed": True},
        "route_prior_application": {"required": True, "confirmed": True},
        "route_prior_application_confirmed": True,
    }
    manifest = {}
    (root / "run_summary.json").write_text(json.dumps(summary), encoding="utf-8")
    (root / "run_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    ExperimentRunner._finalize_checksums(root, summary, manifest)

    assert summary["strict_success"] is True
    assert summary["episode_validity"]["valid"] is True
    assert manifest["episode_validity"]["valid"] is True
    assert ExperimentRunner._checksums_are_verified(root)
    entries = dict(
        line.split("  ", 1)
        for line in (root / "checksums.sha256").read_text().splitlines()
    )
    for filename in ("run_summary.json", "run_manifest.json"):
        assert entries[hashlib.sha256((root / filename).read_bytes()).hexdigest()] == filename


def test_g5_g1_crossing_requires_left_side_pass_while_actor_exists():
    ground_truth = [
        OdometrySample(-0.85, -1.42, 0.0, 0.4, 0.0, 1.00, 0.0),
        OdometrySample(-0.86, -1.90, 0.0, 0.4, 0.0, 1.10, 0.0),
    ]
    actor = [
        {"id": "g5_g1_crossing_actor", "state": "moving", "stamp_s": 1.00, "position": [-0.45, -1.45, 0.5]},
        {"id": "g5_g1_crossing_actor", "state": "parked", "stamp_s": 1.10, "position": [-0.20, -1.45, 0.5]},
    ]

    metrics = ExperimentRunner._g5_g1_left_bypass_metrics(
        ground_truth, actor, "g5_g1_crossing_actor"
    )

    assert metrics["left_side_bypass_seen"]
    assert metrics["passed_while_present"]
    assert metrics["complete"]


def test_focused_dynamic_case_skips_unselected_intermediate_goal_groups():
    runner = object.__new__(ExperimentRunner)
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        obstacle_trajectories=(
            {"id": "local", "motion": "local_bypass", "trigger_group": "G2"},
            {"id": "exit", "motion": "g2_g3_exit", "trigger_group": "G3"},
            {"id": "door", "motion": "g5_g1_crossing", "trigger_group": "G1"},
        ),
    )
    runner._active_selection = SimpleNamespace(case_id="g2_g3_exit")

    assert runner._selected_dynamic_groups_for_goal("G2") == []
    assert runner._selected_dynamic_groups_for_goal("G3") == ["G3"]

    runner._active_selection = SimpleNamespace(case_id="full_route_three_stage")
    assert runner._selected_dynamic_groups_for_goal("G2") == ["G2"]
    assert runner._selected_dynamic_groups_for_goal("G3") == ["G3"]
    assert runner._selected_dynamic_groups_for_goal("G1") == ["G1"]

    runner._scenario.obstacle_trajectories = (
        {"id": "oncoming", "motion": "oncoming", "trigger_group": "G2"},
        {"id": "crossing", "motion": "crossing", "trigger_group": "G3"},
        {"id": "following", "motion": "same_direction_slow", "trigger_group": "G4"},
        {"id": "block", "motion": "temporary_block", "trigger_group": "G5"},
    )
    runner._active_selection = SimpleNamespace(case_id="full_route_four_stage")
    assert runner._selected_dynamic_groups_for_goal("G2") == ["G2"]
    assert runner._selected_dynamic_groups_for_goal("G3") == ["G3"]
    assert runner._selected_dynamic_groups_for_goal("G4") == ["G4"]
    assert runner._selected_dynamic_groups_for_goal("G5") == ["G5"]


def test_single_dynamic_g2_crossing_maps_crossing_to_its_trigger_group():
    runner = object.__new__(ExperimentRunner)
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        obstacle_trajectories=(
            {"id": "low_box", "motion": "crossing", "trigger_group": "G2"},
        ),
    )
    runner._active_selection = SimpleNamespace(case_id="single_dynamic_g2_crossing")

    assert runner._selected_dynamic_groups_for_goal("G2") == ["G2"]


def test_seeds_only_dynamic_scenario_selects_all_trajectories_and_passes_guard():
    runner = object.__new__(ExperimentRunner)
    runner._scenario = SimpleNamespace(
        scenario_type="dynamic",
        obstacle_trajectories=(
            {"id": "first", "motion": "crossing", "trigger_group": "G2"},
            {"id": "second", "motion": "oncoming", "trigger_group": "G3"},
        ),
    )
    runner._active_selection = RunSelection(1)

    assert runner._selected_dynamic_groups_for_goal("G2") == ["G2"]
    assert runner._selected_dynamic_groups_for_goal("G3") == ["G3"]
    runner._validate_dynamic_episode_selection()


@pytest.mark.parametrize(
    ("case_id", "trajectories"),
    [
        (
            "unknown_dynamic_case",
            ({"id": "low_box", "motion": "crossing", "trigger_group": "G2"},),
        ),
        ("single_dynamic_g2_crossing", ()),
    ],
)
def test_dynamic_episode_rejects_empty_selection_before_reset(case_id, trajectories):
    runner = object.__new__(ExperimentRunner)
    runner._clock_ready = True
    runner._clock_timeout_sec = 1.0
    runner._wait_until = lambda predicate, _timeout: predicate()
    runner._verify_dynamic_runtime_contract = lambda: None
    runner._verify_appearance_runtime_contract = lambda: None
    runner._verify_collision_monitor_active = lambda: None
    runner._authorization_only = False
    runner._scenario = SimpleNamespace(
        scenario_id="dynamic_selection_test",
        scenario_type="dynamic",
        obstacle_trajectories=trajectories,
        run_matrix=(RunSelection(1, case_id, "v1"),),
        seeds=(),
    )
    runner._run_indices = None
    runner._require_pregoal_authorization = False
    reset_calls = []
    runner._reset_simulation = lambda *args: reset_calls.append(args)

    with pytest.raises(ConfigurationError, match="no trigger groups or expected actor IDs"):
        runner.run_all()

    assert reset_calls == []


def test_runner_has_no_actor_lifecycle_costmap_clear_workaround():
    assert not hasattr(
        ExperimentRunner, "_request_pending_dynamic_trail_clears"
    )


def test_global_costmap_readiness_rejects_default_window_and_covers_all_goals():
    runner = object.__new__(ExperimentRunner)
    runner._spawn_pose = SimpleNamespace(map=SimpleNamespace(position=(21.2, 120.0)))
    runner._scenario = SimpleNamespace(
        route=(
            SimpleNamespace(position=(1.5, 131.8)),
            SimpleNamespace(position=(-42.6, 180.6)),
        ),
        goal=SimpleNamespace(position=(-42.6, 180.6)),
    )
    metadata = SimpleNamespace(
        resolution=0.05,
        size_x=100,
        size_y=100,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
    )
    runner._global_costmap = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"), metadata=metadata
    )
    assert not runner._global_costmap_covers_mission()

    metadata.size_x = metadata.size_y = 1600
    metadata.origin.position.x = -52.0182
    metadata.origin.position.y = 111.603
    assert runner._global_costmap_covers_mission()


def test_collision_free_policy_keeps_low_clearance_as_warning():
    actor_ids = {"local", "exit", "door"}
    status = _dynamic_interaction_acceptance(
        scenario_type="dynamic",
        expected_ids=actor_ids,
        triggered_ids=actor_ids,
        completed_ids=actor_ids,
        retired_ids=actor_ids,
        clearance_by_actor={"local": 0.0, "exit": 0.69, "door": 0.23},
        evidence_complete=True,
    )

    assert status["complete"] is True
    assert status["minimum_clearance_complete"] is True
    assert status["clearance_warning_below_0_10m"] is True
    assert status["minimum_clearance_requirement_m"] == 0.0
    assert status["acceptance_policy"] == "physical_collision_free"


def test_dynamic_acceptance_rejects_empty_expected_actor_ids():
    status = _dynamic_interaction_acceptance(
        scenario_type="dynamic",
        expected_ids=set(),
        triggered_ids=set(),
        completed_ids=set(),
        retired_ids=set(),
        clearance_by_actor={},
        evidence_complete=True,
    )

    assert status["complete"] is False
    assert status["minimum_clearance_complete"] is False
    assert status["reason"] == "expected_dynamic_actor_ids_empty"


def test_static_appearance_profile_does_not_select_dynamic_obstacle_case():
    appearance = RunSelection(
        9201, "static", "v1", "dim_warm", "rivermark_appearance"
    )
    dynamic = RunSelection(9101, "full_route_four_stage", "v1")

    assert _reset_dynamic_selection("static", appearance) == (None, None)
    assert _reset_dynamic_selection("dynamic", dynamic) == (
        "full_route_four_stage", "v1"
    )


@pytest.mark.parametrize(
    ("triggered", "clearance", "evidence_complete"),
    [
        ({"local", "exit"}, {"local": 0.2, "exit": 0.2, "door": 0.2}, True),
        ({"local", "exit", "door"}, {"local": 0.2, "exit": 0.2}, True),
        ({"local", "exit", "door"}, {"local": 0.2, "exit": 0.2, "door": 0.2}, False),
    ],
)
def test_collision_free_policy_still_requires_complete_interaction_evidence(
    triggered, clearance, evidence_complete
):
    actor_ids = {"local", "exit", "door"}
    status = _dynamic_interaction_acceptance(
        scenario_type="dynamic",
        expected_ids=actor_ids,
        triggered_ids=triggered,
        completed_ids=actor_ids,
        retired_ids=actor_ids,
        clearance_by_actor=clearance,
        evidence_complete=evidence_complete,
    )

    assert status["complete"] is False
