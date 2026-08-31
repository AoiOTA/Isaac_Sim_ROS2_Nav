from collections import deque
from dataclasses import replace
import errno
import hashlib
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import tempfile
import time
from types import SimpleNamespace

import pytest
import robot_experiments.v6_formal as v6_formal_module
import robot_experiments.experiment_runner as experiment_runner_module
import yaml
from robot_experiments.scenario import load_scenario
from robot_experiments.report import write_run_report

from robot_experiments.v6_formal import (
    DISPATCH_SUBSCRIPTION_TOPICS,
    DynamicActionLedger,
    DynamicScheduleEntry,
    ENGINEERING_PILOT,
    EpisodeGuard,
    MissionLeg,
    NOT_QUALIFIED,
    ReadinessFacts,
    V6ContractError,
    V6FormalNode,
    authorize_manifest,
    cli,
    evaluate_formal_campaign,
    evaluate_indoor_campaign,
    execute_formal_campaign,
    execute_indoor_campaign,
    formal_dispatch_plan,
    freeze_indoor_campaign_from_pilot,
    freeze_formal_manifest_from_pilot,
    load_indoor_campaign_manifest,
    load_formal_campaign_manifest,
    load_manifest,
    validate_condition_stack_contract,
)


PACKAGE = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
CONFIG = PACKAGE / "config"
MANIFEST = CONFIG / "v6_r3_phase2_kujiale_baseline.yaml"
PHASE_B_MANIFEST = CONFIG / "v6_r5_phase_b_kujiale_exact_baseline.yaml"
LEGACY_MANIFESTS = tuple(
    CONFIG / (
        f"v6_final_{world}_{category}.yaml"
        if world == "kujiale"
        else f"final_{world}_{category}.yaml"
    )
    for world in ("kujiale", "rivermark")
    for category in ("static", "dynamic", "appearance")
)
REAL_NAS_GUARD = v6_formal_module._validate_nas_mount
REAL_TRACKED_GUARD = v6_formal_module._repository_tracked_dirty
MODULE3_HEAD = subprocess.run(
    ["git", "-C", str(REPO), "rev-parse", "HEAD"],
    check=True,
    capture_output=True,
    text=True,
).stdout.strip()


@pytest.fixture(autouse=True)
def _stub_formal_nas_mount(monkeypatch):
    monkeypatch.setattr(
        v6_formal_module,
        "_validate_nas_mount",
        lambda path: {"target": str(path), "filesystem": "test", "source": "test"},
    )
    # Freeze validation is exercised against the real canonical repositories,
    # whose source files are intentionally dirty while this test change is
    # under development. Dedicated tests cover the tracked-dirty predicate.
    monkeypatch.setattr(v6_formal_module, "_repository_tracked_dirty", lambda _path: False)


def test_formal_nas_guard_resolves_nearest_existing_mount_parent():
    evidence = REAL_NAS_GUARD(
        Path("/mnt/nas_home/.codex-nonexistent/formal-output")
    )
    assert evidence["target"] == "/mnt/nas_home"
    assert evidence["filesystem"].lower() == "cifs"


def test_freeze_tracked_dirty_check_ignores_untracked_and_catches_index(tmp_path):
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

    assert REAL_TRACKED_GUARD(tmp_path) is False

    tracked.write_text("staged\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(tmp_path), "add", "tracked.txt"], check=True)
    assert REAL_TRACKED_GUARD(tmp_path) is True


def _raw() -> dict:
    return yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))


def _write_manifest(tmp_path: Path, raw: dict) -> Path:
    path = tmp_path / "manifest.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return path


def _formal_raw(tmp_path: Path, *, authorization: str = "NOT_AUTHORIZED") -> dict:
    rows = []
    for scene, world in (("indoor", "kujiale"), ("outdoor", "rivermark")):
        for category in ("static", "dynamic", "appearance"):
            scenario = (
                CONFIG / f"v6_final_{world}_{category}.yaml"
                if world == "kujiale"
                else CONFIG / f"final_{world}_{category}.yaml"
            )
            rows.append({
                "id": f"{scene}_{category}",
                "scene": scene,
                "category": category,
                "scenario_file": str(scenario),
                "output_directory": str(tmp_path / f"{scene}-{category}"),
                "runner_arguments": [
                    "nav2_profile:=v6_low_obstacle_isolation",
                    f"nav2_config_file:={v6_formal_module._canonical_nav2_config()}",
                    "navigation_execution_backend:=route_guided",
                    "require_module2_planning_ready:=true",
                ],
            })
    by_id = {row["id"]: row for row in rows}
    ordered = [by_id[name] for name in v6_formal_module.FORMAL_CONDITION_IDS]
    def file_entry(path: Path) -> dict[str, str]:
        path = path.resolve()
        return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    scenario_entries = {}
    scenario_configs = {}
    for row in ordered:
        scenario_path = Path(row["scenario_file"])
        scenario_entries[row["id"]] = file_entry(scenario_path)
        scenario = load_scenario(scenario_path)
        config_paths = v6_formal_module._scenario_runtime_config_paths(
            scenario, row["runner_arguments"]
        )
        scenario_configs[row["id"]] = [
            file_entry(path) for path in sorted(config_paths)
        ]
    repository_paths = {
        "integration": Path(
            "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
            "bio_nav_integration"
        ),
        "module2": Path(
            "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
            "bio_nav_module2"
        ),
        "module3": Path(
            "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
            "bio_nav_module3"
        ),
    }
    repositories = {
        name: {
            "path": str(path.resolve()),
            "head": subprocess.run(
                ["git", "-C", str(path), "rev-parse", "HEAD"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip(),
        }
        for name, path in repository_paths.items()
    }
    runner_entrypoint = REPO / "scripts" / "run_experiment.sh"
    frozen_assets = {}
    for name in sorted(v6_formal_module.FORMAL_FROZEN_ASSET_KEYS):
        if name == "rivermark_catalog_constraints_tree":
            path = tmp_path / "frozen-assets" / name
            path.mkdir(parents=True, exist_ok=True)
            (path / "region_02.json").write_text("{}\n", encoding="utf-8")
            frozen_assets[name] = {
                "path": str(path.resolve()),
                "sha256": v6_formal_module._constraints_tree_sha256(path),
            }
            continue
        path = tmp_path / "frozen-assets" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"{name}\n", encoding="utf-8")
        frozen_assets[name] = file_entry(path)
    return {
        "schema_version": "bio_nav_v6_formal_campaign_v1",
        "intended_use": "formal_qualification",
        "execution_authorization": authorization,
        "runs_per_condition": 20,
        "runner_entrypoint": str(runner_entrypoint),
        "freeze": {
            "repositories": repositories,
            "driver_version": v6_formal_module._current_driver_version(),
            "kernel_release": os.uname().release,
            "scenarios": scenario_entries,
            "scenario_configs": scenario_configs,
            "frozen_assets": frozen_assets,
            "runner_entrypoint": file_entry(runner_entrypoint),
            "experiment_runner": file_entry(
                PACKAGE / "robot_experiments" / "experiment_runner.py"
            ),
            "v6_formal": file_entry(PACKAGE / "robot_experiments" / "v6_formal.py"),
        },
        "conditions": ordered,
    }


def _write_formal_manifest(
    tmp_path: Path, *, authorization: str = "NOT_AUTHORIZED"
) -> Path:
    path = tmp_path / "formal.yaml"
    path.write_text(
        yaml.safe_dump(_formal_raw(tmp_path, authorization=authorization)),
        encoding="utf-8",
    )
    return path


def _write_formal_run(
    condition,
    run_index: int,
    *,
    strict_success: bool,
    formal_freeze_digest: str,
    valid: bool = True,
    stack_session_id: str = "a" * 64,
    collision_detected: bool = False,
    route_completion_count: int = 5,
    route_completion_values: tuple[bool, ...] | None = None,
    path_deviation_percent: float = 10.0,
    cognitive_mutation: str | None = None,
    reset_generation: int | None = None,
) -> Path:
    generation = run_index if reset_generation is None else reset_generation
    identity = condition.episode_identities[run_index - 1]
    seed = identity["seed"]
    root = (
        condition.output_directory
        / condition.scenario_id
        / f"run-{run_index:04d}-seed-{seed}"
    )
    root.mkdir(parents=True)
    telemetry = root / "telemetry"
    required_topics = (
        *experiment_runner_module.COMMON_REQUIRED_RECORDED_TOPICS,
        *experiment_runner_module.ROUTE_GUIDED_REQUIRED_RECORDED_TOPICS,
        *experiment_runner_module.SCENE_REQUIRED_RECORDED_TOPICS[condition.scene],
    )
    import rosbag2_py
    from bio_nav_interfaces.msg import (
        CanonicalRoute, CognitiveObstacleArray, NavigationGraph, PlanningPrior,
        RiskLayerStatus, RouteEdgeCost, RouteEdgeCostArray, RouteProgress,
    )
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage

    message_factories = {
        "/clock": Clock,
        "/ground_truth/odom": Odometry,
        "/odom": Odometry,
        "/bio_nav/module1/odom": Odometry,
        "/tf": TFMessage,
        "/simulation/collision": Bool,
        "/simulation/reset_stop_gate/status": String,
        "/bio_nav/navigation_graph": NavigationGraph,
        "/bio_nav/canonical_route": CanonicalRoute,
        "/bio_nav/route_progress": RouteProgress,
        "/amcl_pose": PoseWithCovarianceStamped,
    }
    messages = {topic: message_factories[topic]() for topic in required_topics if topic != "/cmd_vel_sim"}
    scenario = load_scenario(condition.scenario_file)
    semantic_map_version = scenario.map_version
    cognitive_content_map_id = (
        "cf9eb6dce097b3a58b82c3b52b7a12f5d77ef6901c00d31029a1eda8038e63fc"
    )
    navigation_graph = messages["/bio_nav/navigation_graph"]
    navigation_graph.graph_id = f"{semantic_map_version}:gvg_v1"
    navigation_graph.revision = 1
    navigation_graph.map_version = (
        "wrong-semantic-map"
        if cognitive_mutation == "semantic_map_mismatch"
        else semantic_map_version
    )
    messages["/simulation/collision"] = Bool(data=collision_detected)
    route_cost_message = RouteEdgeCostArray()
    route_cost = RouteEdgeCost()
    route_cost.requested_module2_delta_m = 0.5
    route_cost.applied_module2_delta_m = 0.5
    route_cost_message.costs = [route_cost]
    messages["/bio_nav/route_edge_costs"] = route_cost_message
    terminal_values = route_completion_values or tuple(
        True for _ in range(route_completion_count)
    )
    if not terminal_values:
        raise ValueError("route completion fixture requires at least one terminal")
    messages["/bio_nav/route_goal_complete"] = Bool(data=terminal_values[0])
    cognitive_identity = {
        "reset_epoch": generation,
        "recurrent_session_id": f"recurrent-session-{generation}",
        "map_version": cognitive_content_map_id,
    }
    source_graph_id = (
        f"{semantic_map_version}:gvg_v1"
        if cognitive_mutation == "graph_provenance_current"
        else "wrong-physical-graph"
        if cognitive_mutation == "graph_provenance_mismatch"
        else ""
    )
    source_graph_revision = 1 if source_graph_id else 0
    topology_revision = 1 if source_graph_id else 0
    component_consumers = {
        "global_layer": "/global_costmap/global_costmap:cognitive_obstacle_layer",
        "local_layer": "/local_costmap/local_costmap:cognitive_obstacle_layer",
        "critic": "FollowPath.CognitiveRiskCritic",
    }
    cognitive_receipt = {
        "required": True,
        "required_components": ["global_layer", "local_layer"],
        "deferred_postdispatch_components": ["critic"],
        "minimum_consecutive_samples": 3,
        "barrier_ros_s": 9.0,
        "expected_reset_epoch": generation,
        "forbidden_previous_recurrent_session_id": (
            f"recurrent-session-{generation - 1}" if generation > 1 else "pre-reset"
        ),
        "periodic_planning_identity": {
            "sequence": 3,
            **cognitive_identity,
        },
        "status": "ready",
        "ready": True,
        "reason": "three_consecutive_current_healthy_source_and_layer_samples",
        "reset_generation": generation,
        "ready_ros_s": 13.2,
        "periodic_planning_health": {
            "stamp_s": 13.000000998,
            "sequence": 3,
            **cognitive_identity,
            "module2_healthy": True,
            "input_healthy": True,
            "observation_valid": True,
            "trusted_write": False,
            "schema_version": "bio_nav_planning_prior_v4",
            "model_id": "model-v1",
            "cognitive_tile_id": "tile-v1",
            "tile_revision": 1,
            "graph_revision": 1,
            "source_physical_graph_id": source_graph_id,
            "source_physical_graph_revision": source_graph_revision,
            "topology_revision": topology_revision,
            "risk_model_sha256": "risk-sha256",
            "qualification_receipt_sha256": "qualification-sha256",
            "accepted": True,
            "place_entropy_normalized": 0.20000000298023224,
            "context_uncertainty": 0.10000000149011612,
            "context_trusted": False,
            "trust_rejection_mask": 0,
            "risk_healthy": False,
            "risk_rejection_mask": 0,
            "healthy": True,
        },
        "identity_namespaces": {
            "semantic_navigation_map_version": semantic_map_version,
            "cognitive_content_map_id": cognitive_content_map_id,
            "source_physical_graph_id": source_graph_id,
            "source_physical_graph_revision": source_graph_revision,
            "topology_revision": topology_revision,
            "active_effect_scope": "obstacle_only",
            "navigation_graph_id": f"{semantic_map_version}:gvg_v1",
            "navigation_graph_revision": 1,
            "navigation_graph_map_version": semantic_map_version,
            "semantic_navigation_map_match": True,
            "cognitive_content_map_match": True,
            "graph_provenance_status": (
                "legacy_empty" if not source_graph_id else "current"
            ),
            "graph_provenance_match": (
                cognitive_mutation != "graph_provenance_mismatch"
            ),
            "passed": True,
        },
        "components": {},
    }
    for role in ("global_layer", "local_layer"):
        consumer = component_consumers[role]
        cognitive_receipt["components"][role] = {
            "expected_mode": "active",
            "maximum_age_s": 0.5,
            "consecutive_healthy_samples": 3,
            "latest": {
                "consumer": consumer,
                "role": role,
                "mode": "active",
                "offered": True,
                "applied": False,
                "rejected": False,
                "fallback_reason": "validation_mode=2;source_age_ms=0.000998;"
                "rejection_reason=offered;confirmed_count=0",
                "risk_model_sha256": "risk-sha256",
                "qualification_receipt_sha256": "qualification-sha256",
                "admission_rejection_reason": None,
                "source_sequence": 3,
                **cognitive_identity,
                "status_stamp_s": 13.025,
                "message_age_ms": 24.99900245666504,
                "validation_stamp_s": 13.000000998,
                "consecutive_healthy_samples": 3,
            },
            "source": {
                "sequence": 3,
                **cognitive_identity,
                "frame_id": "base_link",
                "cognitive_tile_id": "tile-v1",
                "tile_revision": 1,
                "graph_revision": 1,
                "schema_version": "bio_nav_cognitive_obstacles_v1",
                "model_id": "model-v1",
                "risk_model_sha256": "risk-sha256",
                "qualification_receipt_sha256": "qualification-sha256",
                "ttl_ns": 500000000,
                "validation_ttl_ns": 500000000,
                "source_age_ns": 998,
                "validation_mode": 2,
                "validation_stamp_s": 13.000000998,
                "healthy": True,
            },
            "periodic_planning_health": dict(
                cognitive_receipt["periodic_planning_health"]
            ),
        }
    if cognitive_mutation == "async_lag":
        cognitive_receipt["periodic_planning_health"] = {
            **cognitive_receipt["periodic_planning_health"],
            "stamp_s": 13.100000998,
            "sequence": 4,
        }
        cognitive_receipt["periodic_planning_identity"] = {
            "sequence": 4,
            **cognitive_identity,
        }
        for role in ("global_layer", "local_layer"):
            component = cognitive_receipt["components"][role]
            component["consecutive_healthy_samples"] = 4
            component["latest"].update({
                "source_sequence": 4,
                "status_stamp_s": 13.125,
                "validation_stamp_s": 13.100000998,
                "consecutive_healthy_samples": 4,
            })
            component["source"].update({
                "sequence": 4,
                "validation_stamp_s": 13.100000998,
            })
            component["periodic_planning_health"] = dict(
                cognitive_receipt["periodic_planning_health"]
            )
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(telemetry), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    for topic, message in messages.items():
        type_name = (
            "bio_nav_interfaces/msg/RouteEdgeCostArray"
            if topic == "/bio_nav/route_edge_costs"
            else "std_msgs/msg/Bool"
            if topic == "/bio_nav/route_goal_complete"
            else experiment_runner_module.RECORDED_TOPIC_TYPES[topic]
        )
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0,
            name=topic,
            type=type_name,
            serialization_format="cdr",
            offered_qos_profiles=[],
        ))
    for topic, type_name in (
        (
            "/bio_nav/module2/cognitive_obstacles",
            "bio_nav_interfaces/msg/CognitiveObstacleArray",
        ),
        (
            "/bio_nav/cognitive_obstacle_layer/status",
            "bio_nav_interfaces/msg/RiskLayerStatus",
        ),
        (
            "/bio_nav/cognitive_risk_critic/status",
            "bio_nav_interfaces/msg/RiskLayerStatus",
        ),
        (
            "/bio_nav/module2/planning_prior",
            "bio_nav_interfaces/msg/PlanningPrior",
        ),
    ):
        writer.create_topic(rosbag2_py.TopicMetadata(
            id=0,
            name=topic,
            type=type_name,
            serialization_format="cdr",
            offered_qos_profiles=[],
        ))
    writer.create_topic(rosbag2_py.TopicMetadata(
        id=0,
        name="/cmd_vel_sim",
        type="geometry_msgs/msg/Twist",
        serialization_format="cdr",
        offered_qos_profiles=[],
    ))
    stamp = 1
    for topic, message in messages.items():
        if topic == "/bio_nav/route_goal_complete":
            continue
        writer.write(topic, serialize_message(message), stamp)
        stamp += 1
    sequences = (1, 2, 3, 4) if cognitive_mutation == "async_lag" else (1, 2, 3)
    for sequence in sequences:
        semantic_sec = (
            8
            if cognitive_mutation == "pre_barrier" and sequence == 1
            else 13 if sequence == 4 else 10 + sequence
        )
        semantic_nanosec = 100000000 if sequence == 4 else 0
        offered_epoch = (
            generation + 1 if cognitive_mutation == "generation_mismatch" else generation
        )
        offered_session = (
            cognitive_receipt["forbidden_previous_recurrent_session_id"]
            if cognitive_mutation == "session_reuse"
            else cognitive_identity["recurrent_session_id"]
        )
        planning = PlanningPrior()
        planning.stamp.sec = semantic_sec
        planning.stamp.nanosec = semantic_nanosec + 998
        planning.sequence = (
            99
            if cognitive_mutation == "planning_mismatch" and sequence == 2
            else sequence
        )
        planning.reset_epoch = offered_epoch
        planning.recurrent_session_id = offered_session
        planning.map_version = cognitive_identity["map_version"]
        planning.schema_version = "bio_nav_planning_prior_v4"
        planning.model_id = (
            "unrelated-model"
            if cognitive_mutation == "planning_chain_tamper" and sequence == 2
            else "model-v1"
        )
        planning.cognitive_tile_id = "tile-v1"
        planning.tile_revision = 1
        planning.graph_revision = 1
        planning.source_physical_graph_id = source_graph_id
        planning.source_physical_graph_revision = source_graph_revision
        planning.topology_revision = topology_revision
        planning.risk_model_sha256 = "risk-sha256"
        planning.qualification_receipt_sha256 = "qualification-sha256"
        planning.module2_healthy = True
        planning.input_healthy = True
        planning.observation_valid = True
        planning.trusted_write = False
        planning.context_trusted = False
        planning.trust_rejection_mask = 0
        planning.risk_healthy = False
        planning.risk_rejection_mask = 0
        planning.place_entropy_normalized = (
            0.7
            if cognitive_mutation == "planning_field_tamper" and sequence == 3
            else 0.2
        )
        planning.context_uncertainty = 0.1
        writer.write(
            "/bio_nav/module2/planning_prior",
            serialize_message(planning),
            semantic_sec * 1_000_000_000 + semantic_nanosec + 2_000_000,
        )
        stamp += 1
        source = CognitiveObstacleArray()
        source.header.frame_id = "base_link"
        source.header.stamp.sec = semantic_sec
        source.header.stamp.nanosec = semantic_nanosec
        source.sequence = sequence
        source.reset_epoch = offered_epoch
        source.recurrent_session_id = offered_session
        source.map_version = (
            "0" * 64
            if cognitive_mutation == "content_map_mismatch" and sequence == 2
            else cognitive_identity["map_version"]
        )
        source.cognitive_tile_id = "tile-v1"
        source.tile_revision = 1
        source.graph_revision = 1
        source.schema_version = (
            "tampered_schema"
            if cognitive_mutation == "source_schema_tamper" and sequence == 2
            else "bio_nav_cognitive_obstacles_v1"
        )
        source.model_id = "model-v1"
        source.risk_model_sha256 = "risk-sha256"
        source.qualification_receipt_sha256 = "qualification-sha256"
        source.ttl.nanosec = 500000000
        source.validation_ttl.nanosec = 500000000
        source.reliability = 0.9
        source.ood_probability = 0.1
        source.validation_stamp.sec = semantic_sec
        source.validation_stamp.nanosec = semantic_nanosec + 998
        source.source_age.nanosec = 998
        source.validation_mode = CognitiveObstacleArray.VALIDATION_STATIC_DEPTH_REVALIDATED
        source.validation_sensor_mask = CognitiveObstacleArray.VALIDATION_SENSOR_DEPTH
        source.source_odom_stamp.sec = semantic_sec
        source.source_odom_stamp.nanosec = (
            100000001
            if cognitive_mutation == "odom_over_100ms" and sequence == 2
            else semantic_nanosec + 16665668
        )
        source.validation_odom_stamp.sec = semantic_sec
        source.validation_odom_stamp.nanosec = (
            100000002
            if cognitive_mutation == "odom_over_100ms" and sequence == 2
            else semantic_nanosec + 16666666
        )
        source.input_healthy = True
        source.module2_healthy = True
        source.observation_valid = True
        source.trusted_write = True
        writer.write(
            "/bio_nav/module2/cognitive_obstacles",
            serialize_message(source),
            semantic_sec * 1_000_000_000 + semantic_nanosec + 3_000_000,
        )
        stamp += 1
        for endpoint in (
            source.source_odom_stamp,
            source.validation_odom_stamp,
        ):
            odom = Odometry()
            odom.header.stamp = endpoint
            writer.write("/odom", serialize_message(odom), stamp)
            stamp += 1
        for role, consumer in component_consumers.items():
            if role == "critic":
                continue
            status = RiskLayerStatus()
            status.stamp.sec = semantic_sec
            status.stamp.nanosec = semantic_nanosec + 25000000
            status.consumer = (
                "spoof.FollowPath.CognitiveRiskCritic"
                if cognitive_mutation == "component_spoof"
                and sequence == 2 and role == "critic"
                else consumer
            )
            status.mode = "active"
            status.offered = not (
                cognitive_mutation == "offered_false"
                and sequence == 2 and role == "critic"
            )
            status.applied = False
            status.rejected = False
            status.source_sequence = sequence
            status.reset_epoch = offered_epoch
            status.recurrent_session_id = offered_session
            status.map_version = cognitive_identity["map_version"]
            status.message_age_ms = (
                30.0
                if cognitive_mutation == "age_mismatch"
                and sequence == 2 and role == "critic"
                else 24.999002
            )
            status.fallback_reason = (
                "shadow;maximum_obstacle_cost_delta=0;obstacle_count=0;"
                "aggregation=max_per_step_mean_horizon"
                if cognitive_mutation == "shadow_in_active"
                and sequence == 2 and role == "critic"
                else
                "cost_delta_applied=false;zero_cost_delta;"
                f"prior_suppressed={cognitive_mutation.removeprefix('degraded_')};"
                "maximum_obstacle_cost_delta=0;obstacle_count=0;"
                "aggregation=max_per_step_mean_horizon"
                if cognitive_mutation is not None
                and cognitive_mutation.startswith("degraded_")
                and sequence == 2 and role == "critic"
                else
                "cost_delta_applied=false;zero_cost_delta;"
                "obstacle_applied=false;obstacle_suppressed=zero_cost_delta;"
                "prior_accepted=true;context_applied=false;"
                "context_suppressed=zero_cost_delta;novelty_applied=false;"
                "novelty_suppressed=zero_cost_delta;"
                "uncertainty_applied=false;"
                "uncertainty_suppressed=zero_cost_delta;"
                "direction_applied=false;direction_suppressed=zero_cost_delta;"
                f"accepted_source_sequence={sequence};"
                "maximum_obstacle_cost_delta=0;obstacle_count=0;"
                "aggregation=max_per_step_mean_horizon"
                if role == "critic"
                else "validation_mode=2;source_age_ms=0.000998;"
                "rejection_reason=offered;confirmed_count=0"
            )
            status.risk_model_sha256 = (
                "unrelated-risk"
                if cognitive_mutation in {
                    "critic_receipt_tamper", "layer_receipt_tamper",
                }
                and sequence == 2
                and (
                    role == "critic"
                    if cognitive_mutation == "critic_receipt_tamper"
                    else role == "global_layer"
                )
                else "risk-sha256"
            )
            status.qualification_receipt_sha256 = "qualification-sha256"
            writer.write(
                (
                    "/bio_nav/cognitive_risk_critic/status"
                    if role == "critic"
                    else "/bio_nav/cognitive_obstacle_layer/status"
                ),
                serialize_message(status),
                semantic_sec * 1_000_000_000
                + semantic_nanosec + 25_000_000,
            )
            stamp += 1
    if cognitive_mutation == "bad_after_latch":
        bad_planning = PlanningPrior()
        bad_planning.stamp.sec = 13
        bad_planning.stamp.nanosec = 250000000
        bad_planning.sequence = 4
        bad_planning.reset_epoch = cognitive_identity["reset_epoch"]
        bad_planning.recurrent_session_id = cognitive_identity[
            "recurrent_session_id"
        ]
        bad_planning.map_version = cognitive_identity["map_version"]
        bad_planning.schema_version = "bio_nav_planning_prior_v4"
        bad_planning.model_id = "model-v1"
        bad_planning.cognitive_tile_id = "tile-v1"
        bad_planning.tile_revision = 1
        bad_planning.graph_revision = 1
        bad_planning.risk_model_sha256 = "risk-sha256"
        bad_planning.qualification_receipt_sha256 = "qualification-sha256"
        bad_planning.input_healthy = True
        bad_planning.observation_valid = True
        bad_planning.trusted_write = True
        bad_planning.module2_healthy = False
        writer.write(
            "/bio_nav/module2/planning_prior",
            serialize_message(bad_planning),
            13_250_000_000,
        )
        stamp += 1
    if cognitive_mutation == "post_latch_bad_then_same_seq_healthy":
        source.schema_version = "tampered_schema"
        writer.write(
            "/bio_nav/module2/cognitive_obstacles",
            serialize_message(source),
            13_190_000_000,
        )
        source.schema_version = "bio_nav_cognitive_obstacles_v1"
        writer.write(
            "/bio_nav/module2/cognitive_obstacles",
            serialize_message(source),
            13_195_000_000,
        )
    if cognitive_mutation == "post_latch_bad_planning_then_healthy":
        planning.module2_healthy = False
        writer.write(
            "/bio_nav/module2/planning_prior",
            serialize_message(planning),
            13_190_000_000,
        )
        planning.module2_healthy = True
        writer.write(
            "/bio_nav/module2/planning_prior",
            serialize_message(planning),
            13_195_000_000,
        )
    post_sequence = 4 if cognitive_mutation == "async_lag" else 3
    post_validation_stamp_s = 13.100000998 if post_sequence == 4 else 13.000000998
    critic_status = RiskLayerStatus()
    critic_status.stamp.sec = 13
    critic_status.stamp.nanosec = 300000000
    critic_status.consumer = (
        "spoof.FollowPath.CognitiveRiskCritic"
        if cognitive_mutation == "component_spoof"
        else component_consumers["critic"]
    )
    critic_status.mode = "active"
    critic_status.offered = cognitive_mutation != "offered_false"
    critic_status.applied = False
    critic_status.rejected = False
    critic_status.source_sequence = post_sequence
    critic_status.reset_epoch = generation
    critic_status.recurrent_session_id = cognitive_identity["recurrent_session_id"]
    critic_status.map_version = cognitive_identity["map_version"]
    critic_status.message_age_ms = (
        350.0
        if cognitive_mutation == "age_mismatch"
        else (13.3 - post_validation_stamp_s) * 1000.0
    )
    critic_status.fallback_reason = (
        "shadow;maximum_obstacle_cost_delta=0;obstacle_count=0;"
        "aggregation=max_per_step_mean_horizon"
        if cognitive_mutation == "shadow_in_active"
        else
        "cost_delta_applied=false;zero_cost_delta;"
        f"prior_suppressed={cognitive_mutation.removeprefix('degraded_')};"
        "maximum_obstacle_cost_delta=0;obstacle_count=0;"
        "aggregation=max_per_step_mean_horizon"
        if cognitive_mutation is not None
        and cognitive_mutation.startswith("degraded_")
        else
        "cost_delta_applied=false;zero_cost_delta;"
        "obstacle_applied=false;obstacle_suppressed=zero_cost_delta;"
        "active_effect_scope=obstacle_only;prior_required=false;"
        f"accepted_source_sequence={post_sequence};"
        "maximum_obstacle_cost_delta=0;obstacle_count=0;"
        "aggregation=max_per_step_mean_horizon"
    )
    critic_status.risk_model_sha256 = (
        "unrelated-risk"
        if cognitive_mutation == "critic_receipt_tamper"
        else "risk-sha256"
    )
    critic_status.qualification_receipt_sha256 = "qualification-sha256"
    if cognitive_mutation == "post_latch_bad_status_then_healthy":
        healthy_reason = critic_status.fallback_reason
        critic_status.fallback_reason = "prior_untrusted"
        writer.write(
            "/bio_nav/cognitive_risk_critic/status",
            serialize_message(critic_status),
            13_290_000_000,
        )
        critic_status.fallback_reason = healthy_reason
    if cognitive_mutation not in {"postdispatch_critic_missing", "postdispatch_critic_late"}:
        writer.write(
            "/bio_nav/cognitive_risk_critic/status",
            serialize_message(critic_status),
            13_300_000_000,
        )
    nonzero = Twist()
    nonzero.linear.x = 0.2
    writer.write("/cmd_vel_sim", serialize_message(nonzero), 13_310_000_000)
    if cognitive_mutation == "postdispatch_critic_late":
        critic_status.stamp.nanosec = 400000000
        critic_status.message_age_ms = (13.4 - post_validation_stamp_s) * 1000.0
        writer.write(
            "/bio_nav/cognitive_risk_critic/status",
            serialize_message(critic_status),
            13_400_000_000,
        )
    terminal_record_ns = 13_500_000_000
    for terminal_value in terminal_values:
        writer.write(
            "/bio_nav/route_goal_complete",
            serialize_message(Bool(data=terminal_value)),
            terminal_record_ns,
        )
        terminal_record_ns += 1_000_000
    writer.write("/cmd_vel_sim", serialize_message(Twist()), 13_600_000_000)
    writer.write("/cmd_vel_sim", serialize_message(Twist()), 13_610_000_000)
    del writer
    coverage = experiment_runner_module._mcap_required_topic_coverage(
        telemetry / "metadata.yaml",
        scene=condition.scene,
        route_guided=True,
    )
    coverage["required"] = True
    route_costs = [{
        "request_id": 1,
        "edges": [{
            "requested_module2_delta_m": 0.5,
            "applied_module2_delta_m": 0.5,
        }],
    }]
    route_prior = experiment_runner_module._route_prior_application_evidence(
        route_costs, required=True
    )
    critic_invalid = bool(
        cognitive_mutation in {
            "component_spoof", "offered_false", "age_mismatch",
            "shadow_in_active", "critic_receipt_tamper",
            "post_latch_bad_status_then_healthy",
            "postdispatch_critic_missing", "postdispatch_critic_late",
        }
        or (
            cognitive_mutation is not None
            and cognitive_mutation.startswith("degraded_")
        )
    )
    postdispatch_critic_evidence = {
        "required": True,
        "passed": not critic_invalid,
        "reason": (
            "healthy_critic_processed_before_motion"
            if not critic_invalid
            else "postdispatch_critic_fixture_invalid"
        ),
        "active_effect_scope": "obstacle_only",
        "expected_mode": "active",
        "maximum_age_s": 0.5,
        "first_healthy_critic": (
            {
                "runtime_order": 1,
                "ros_stamp_s": 13.3,
                "received_monotonic": 1.0,
                "source_sequence": post_sequence,
                "reset_epoch": generation,
                "recurrent_session_id": cognitive_identity[
                    "recurrent_session_id"
                ],
                "cognitive_content_map_id": cognitive_identity["map_version"],
            }
            if not critic_invalid else None
        ),
        "first_nonzero_cmd_vel": {
            "runtime_order": 2,
            "ros_stamp_s": 13.31,
            "received_monotonic": 1.01,
        },
        "same_cycle_tolerance_ns": 1000000,
        "ordered_before_command": not critic_invalid,
        "same_cycle_allowed": False,
        "rejection_or_degraded_seen": bool(
            critic_invalid
            and cognitive_mutation not in {
                "postdispatch_critic_missing", "postdispatch_critic_late",
            }
        ),
    }
    declared_required_files = {
        "TRIAL_DISPATCHED.json",
        "run_manifest.json",
        "events.jsonl",
        "ground_truth.csv.gz",
        "odom.csv.gz",
        "cmd_vel.csv.gz",
        "obstacles.csv.gz",
        "dynamic_obstacles.csv.gz",
        "leg_metrics.csv",
        "depth_frame.pgm",
        "depth_frame.json",
        "scan.csv",
        "scan.json",
        "scan_safety.csv",
        "scan_safety.json",
        "local_costmap.pgm",
        "local_costmap.json",
        "global_costmap.pgm",
        "global_costmap.json",
    }
    if identity["appearance_profile_id"] is not None:
        declared_required_files |= {
            "appearance_rgb_before_goal.ppm",
            "appearance_rgb_before_goal.json",
        }
    for name in declared_required_files - {"run_manifest.json"}:
        target = root / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text("fixture\n", encoding="utf-8")
    (root / "TRIAL_DISPATCHED.json").write_text(json.dumps({
        "schema": "bio_nav.trial_dispatched.v1",
        "run_index": run_index,
        "ros_stamp_s": 13.3,
        "cognitive_admission_readiness": cognitive_receipt,
        "postdispatch_critic_evidence": postdispatch_critic_evidence,
    }), encoding="utf-8")
    (root / "FINAL_TRIAL_METRICS.json").write_text(
        '{"passed": true}\n', encoding="utf-8"
    )
    summary = {
        "navigation_contract_success": strict_success,
        "strict_success": strict_success,
        "terminal_zero_confirmed": True,
        "reset_receipt": {"generation": generation},
        "reset_receipt_confirmed": True,
        "physical_collision_free": not collision_detected,
        "isaac_contact_sensor_collision_detected": collision_detected,
        "contact_sensor_evidence_confirmed": True,
        "fixed_map_to_odom_evidence_confirmed": True,
        "localization_node_ownership": {
            **experiment_runner_module._localization_node_ownership_evidence(
                condition.scene,
                ["ideal_localization_tf"] if condition.scene == "outdoor" else [],
            ),
            "graph_error": None,
        },
        "data_complete": True,
        "checksums_verified": True,
        "episode_validity": {
            "valid": valid,
            "status": "valid" if valid else "invalid",
            "invalid_reasons": [] if valid else ["fixture_invalid"],
        },
        "final_trial_metric_gate": {"passed": True},
        "required_topic_coverage": coverage,
        "route_prior_application": route_prior,
        "route_prior_application_confirmed": True,
        "evidence": {
            "required_files": sorted(declared_required_files)
        },
        "condition_stack_id": condition.condition_id,
        "stack_session_id": stack_session_id,
        "formal_freeze_digest": formal_freeze_digest,
        "path_deviation_percent": path_deviation_percent,
        "cognitive_admission_readiness": cognitive_receipt,
        "postdispatch_critic_evidence": postdispatch_critic_evidence,
    }
    episode = {
        "scenario_id": condition.scenario_id,
        "run_index": run_index,
        "random_seed": seed,
        "condition_id": identity["condition_id"],
        "dynamic_selection": {
            "case_id": identity["dynamic_case_id"],
            "variant_id": identity["dynamic_variant_id"],
        },
        "appearance": {"profile_id": identity["appearance_profile_id"]},
        "result": "success" if strict_success else "failure",
        "terminal_zero_confirmed": True,
        "legs": [{"id": f"G{index}"} for index in range(1, 6)],
        "route_edge_costs": route_costs,
        "observability": {
            "collision_status_seen": True,
            "map_to_odom_seen": True,
        },
        "condition_stack_id": condition.condition_id,
        "stack_session_id": stack_session_id,
        "formal_freeze_digest": formal_freeze_digest,
        "reset_receipt": {"generation": generation},
        "metrics": {"path_deviation_percent": path_deviation_percent},
        "cognitive_admission_readiness": cognitive_receipt,
        "postdispatch_critic_evidence": postdispatch_critic_evidence,
    }
    robot_hash, nav2_hash, runtime_hashes = (
        v6_formal_module._expected_scenario_runtime_hashes(condition)
    )
    scenario = load_scenario(condition.scenario_file)
    episode.update({
        "map_version": scenario.map_version,
        "posegraph_version": scenario.posegraph_version,
        "robot_config_hash": robot_hash,
        "nav2_config_hash": nav2_hash,
        "scenario_runtime_hashes": runtime_hashes,
        "dynamic_runtime_contract": {
            "verified": True,
            "config_sha256": runtime_hashes["dynamic_config"],
        },
        "spawn_pose_name": scenario.spawn_pose_name,
        "usd_start_pose": {},
        "map_start_pose": {},
        "goal_pose": {},
        "obstacle_trajectories": [],
        "physics_dt": scenario.physics_dt,
        "rtf": scenario.rtf,
        "failure_reason": "" if strict_success else "fixture_product_failure",
        "provenance": {
            "git_head": MODULE3_HEAD,
            "git_dirty": True,
            "git_tracked_dirty": False,
            "map_and_posegraph_hashes": {},
        },
    })
    episode["optimal_reference_hash"] = runtime_hashes.get("optimal_reference")
    (root / "run_summary.json").write_text(json.dumps(summary), encoding="utf-8")
    (root / "run_manifest.json").write_text(json.dumps(episode), encoding="utf-8")
    _refresh_checksums(root)
    return root


def _refresh_checksums(root: Path) -> None:
    entries = [
        f"{hashlib.sha256(item.read_bytes()).hexdigest()}  {item.relative_to(root)}"
        for item in sorted(
            path
            for path in root.rglob("*")
            if path.is_file() and path.name != "checksums.sha256"
        )
    ]
    (root / "checksums.sha256").write_text(
        "\n".join(entries) + "\n", encoding="utf-8"
    )


def _live_stack_contract(
    tmp_path: Path,
    *,
    condition_id: str = "indoor_static",
    pid: int | None = None,
    **overrides,
) -> Path:
    pid = os.getpid() if pid is None else pid
    stat = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    scene, condition = condition_id.split("_", 1)
    module3_root = Path(
        "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
        "bio_nav_module3"
    )
    t2_selector = module3_root / "scripts" / (
        "run_v6_rivermark.sh"
        if scene == "outdoor"
        else "run_v6_kujiale_low_obstacles.sh"
    )
    payload = {
        "schema": "bio_nav.v6_stack_contract.v1",
        "condition_id": condition_id,
        "scene": scene,
        "condition": condition,
        "arm": "M3",
        "domain": 150,
        "startup_profile": (
            "module2_causal_obstacle_outdoor"
            if scene == "outdoor"
            else "module2_causal_obstacle_active"
        ),
        "pid": pid,
        "pgid": int(stat[2]),
        "start_ticks": int(stat[19]),
        "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "integration_head": subprocess.run(
            [
                "git", "-C",
                "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
                "bio_nav_integration",
                "rev-parse", "HEAD",
            ],
            check=True, capture_output=True, text=True,
        ).stdout.strip(),
        "module2_head": subprocess.run(
            [
                "git", "-C",
                "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
                "bio_nav_module2",
                "rev-parse", "HEAD",
            ],
            check=True, capture_output=True, text=True,
        ).stdout.strip(),
        "module3_head": subprocess.run(
            [
                "git", "-C",
                "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
                "bio_nav_module3",
                "rev-parse", "HEAD",
            ],
            check=True, capture_output=True, text=True,
        ).stdout.strip(),
        "integration_dirty": False,
        "module2_dirty": False,
        "module3_dirty": False,
        "driver_version": v6_formal_module._current_driver_version(),
        "kernel_release": os.uname().release,
        "t2_selector_path": str(t2_selector.resolve()),
        "t2_selector_sha256": hashlib.sha256(t2_selector.read_bytes()).hexdigest(),
        "episode_sequence_path": str((tmp_path / "episode.sequence.json").resolve()),
    }
    payload.update(overrides)
    payload["stack_session_id"] = v6_formal_module._stack_session_id(payload)
    path = tmp_path / "stack.contract.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _write_sufficient_pilot_inputs(
    tmp_path: Path,
    *,
    first_collision: bool = False,
    first_route_completion_count: int = 5,
):
    (tmp_path / "nas").mkdir(exist_ok=True)
    raw = _formal_raw(tmp_path)
    reference_path = tmp_path / "reference-formal.json"
    reference_path.write_text(json.dumps(raw), encoding="utf-8")
    reference = load_formal_campaign_manifest(reference_path)
    pilot_formal_digest = ""
    pilot_manifest = {
        "schema_version": "bio_nav_v6_sufficient_pilot_manifest_v1",
        "intended_use": "sufficient_pilot",
        "runner_entrypoint": raw["runner_entrypoint"],
        "freeze": raw["freeze"],
        "conditions": [
            {
                key: row[key]
                for key in (
                    "id", "scene", "category", "scenario_file", "runner_arguments"
                )
            }
            for row in raw["conditions"]
        ],
    }
    pilot_manifest_path = tmp_path / "pilot-manifest.json"
    pilot_manifest_path.write_text(json.dumps(pilot_manifest), encoding="utf-8")
    aggregate_rows = []
    for condition in reference.conditions:
        contract_root = tmp_path / "pilot-stack-contracts" / condition.condition_id
        contract_root.mkdir(parents=True)
        contract_path = _live_stack_contract(
            contract_root, condition_id=condition.condition_id
        )
        contract = json.loads(contract_path.read_text())
        stack_tuple_digest = v6_formal_module._stack_tuple_digest(contract)
        episode_rows = []
        for rep in range(1, 4):
            root = _write_formal_run(
                condition,
                rep,
                strict_success=True,
                formal_freeze_digest=pilot_formal_digest,
                stack_session_id=contract["stack_session_id"],
                collision_detected=bool(
                    first_collision
                    and condition.condition_id == "indoor_static"
                    and rep == 1
                ),
                route_completion_count=(
                    first_route_completion_count
                    if condition.condition_id == "indoor_static" and rep == 1
                    else 5
                ),
                reset_generation=rep + 1,
            )
            manifest_path = root / "run_manifest.json"
            summary_path = root / "run_summary.json"
            stack_snapshot_path = root / "stack_contract.json"
            shutil.copy2(contract_path, stack_snapshot_path)
            episode = json.loads(manifest_path.read_text())
            summary = json.loads(summary_path.read_text())
            scenario = load_scenario(condition.scenario_file)
            episode["robot_config_hash"] = hashlib.sha256(
                scenario.resolve_path(scenario.robot_config_file).read_bytes()
            ).hexdigest()
            episode["nav2_config_hash"] = hashlib.sha256(
                v6_formal_module._effective_nav2_config(
                    condition.runner_arguments
                ).read_bytes()
            ).hexdigest()
            episode["scenario_runtime_hashes"] = {
                **v6_formal_module._expected_scenario_runtime_hashes(condition)[2]
            }
            map_keys = (
                ("outdoor_map_yaml", "outdoor_map_pgm")
                if condition.scene == "outdoor"
                else ("indoor_map_yaml", "indoor_map_pgm")
            )
            episode["provenance"] = {
                "git_head": reference.freeze["repositories"]["module3"]["head"],
                "git_dirty": False,
                "git_tracked_dirty": False,
                "map_and_posegraph_hashes": {
                    name: reference.freeze["frozen_assets"][name]["sha256"]
                    for name in map_keys
                },
            }
            episode["reset_receipt"] = {"generation": rep + 1}
            summary["reset_receipt"] = {"generation": rep + 1}
            stack_episode_receipt = {
                "schema": "bio_nav.v6_stack_episode_receipt.v1",
                "sequence": rep,
                "baseline": 1,
                "stack_session_id": contract["stack_session_id"],
                "sequence_path": contract["episode_sequence_path"],
                "t2_selector_path": contract["t2_selector_path"],
                "t2_selector_sha256": contract["t2_selector_sha256"],
            }
            episode["stack_episode_receipt"] = stack_episode_receipt
            summary["condition_stack_attestation"] = {
                "required": True,
                "condition_stack_id": condition.condition_id,
                "stack_session_id": contract["stack_session_id"],
                "formal_freeze_digest": pilot_formal_digest,
                "stack_episode_receipt": stack_episode_receipt,
                "confirmed": True,
            }
            if "stack_contract.json" not in summary["evidence"]["required_files"]:
                summary["evidence"]["required_files"].append("stack_contract.json")
            manifest_path.write_text(json.dumps(episode), encoding="utf-8")
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            _refresh_checksums(root)
            episode_rows.append({
                "rep": rep,
                "boundary": "cold" if rep == 1 else "hot_reset",
                "summary_path": str(summary_path),
                "manifest_path": str(manifest_path),
                "stack_contract_path": str(stack_snapshot_path),
                "stack_tuple_digest": stack_tuple_digest,
            })
        aggregate_rows.append({
            "id": condition.condition_id,
            "scene": condition.scene,
            "category": condition.category,
            "episodes": episode_rows,
        })
    aggregate_path = tmp_path / "pilot-aggregate.json"
    aggregate_path.write_text(json.dumps({
        "schema_version": "bio_nav_v6_sufficient_pilot_aggregate_v1",
        "pilot_manifest": str(pilot_manifest_path),
        "conditions": aggregate_rows,
    }), encoding="utf-8")
    return pilot_manifest_path, aggregate_path, reference


def _write_authorized_formal_manifest(tmp_path: Path) -> Path:
    pilot_manifest, aggregate, _reference = _write_sufficient_pilot_inputs(tmp_path)
    output = tmp_path / "authorized-formal.json"
    original_nas_root = v6_formal_module.FORMAL_NAS_ROOT
    v6_formal_module.FORMAL_NAS_ROOT = tmp_path / "nas"
    try:
        freeze_formal_manifest_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate,
            output_manifest_path=output,
            formal_output_root=tmp_path / "nas" / "authorized-formal-root",
        )
    finally:
        v6_formal_module.FORMAL_NAS_ROOT = original_nas_root
    raw = json.loads(output.read_text())
    raw["execution_authorization"] = "AUTHORIZED"
    output.write_text(json.dumps(raw), encoding="utf-8")
    return output


def _fake_validator_promotion(freeze, pilot_runtime=None):
    pilot_runtime = pilot_runtime or {
        "repositories": freeze["repositories"],
        "driver_version": freeze["driver_version"],
        "kernel_release": freeze["kernel_release"],
    }
    module3_head = freeze["repositories"]["module3"]["head"]
    return {
        "schema": "bio_nav.v6_validator_only_head_promotion.v1",
        "pilot_runtime": json.loads(json.dumps(pilot_runtime)),
        "final_repositories": json.loads(json.dumps(freeze["repositories"])),
        "module3_diff": {
            "from_head": pilot_runtime["repositories"]["module3"]["head"],
            "to_head": module3_head,
            "from_is_ancestor": True,
            "name_status": [],
            "canonical_diff_sha256": "0" * 64,
        },
        "loaded_validator": {
            "module": "robot_experiments.experiment_runner",
            "symbol": "validate_recorded_run_evidence",
            "source_path": str(REPO / "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"),
            "source_sha256": "0" * 64,
            "git_blob_oid": "fixture",
            "current_head": module3_head,
        },
    }


def _mock_validator_promotion(monkeypatch):
    monkeypatch.setattr(
        v6_formal_module,
        "_validate_validator_only_head_promotion",
        lambda value, *, freeze: dict(value),
    )
    monkeypatch.setattr(
        v6_formal_module,
        "_build_validator_only_head_promotion",
        lambda *, freeze, pilot_runtime: _fake_validator_promotion(
            freeze, pilot_runtime
        ),
    )


def _fake_qualification_tooling_promotion(freeze, parent_freeze):
    return {
        "schema": "bio_nav.v6_indoor_qualification_tooling_promotion.v1",
        "from_head": parent_freeze["repositories"]["module3"]["head"],
        "to_head": freeze["repositories"]["module3"]["head"],
        "from_is_ancestor": True,
        "name_status": [],
        "canonical_diff_sha256": "0" * 64,
        "v6_formal": {},
        "wrapper": {},
        "unchanged_files": {},
        "parent_validator_promotion_sha256": "0" * 64,
    }


def _mock_qualification_tooling_promotion(monkeypatch):
    monkeypatch.setattr(
        v6_formal_module,
        "_build_qualification_tooling_promotion",
        lambda *, freeze, parent_freeze: _fake_qualification_tooling_promotion(
            freeze, parent_freeze
        ),
    )
    monkeypatch.setattr(
        v6_formal_module,
        "_validate_qualification_tooling_promotion",
        lambda value, *, freeze, parent_freeze: dict(value),
    )
    monkeypatch.setattr(
        v6_formal_module,
        "_validate_historical_validator_promotion",
        lambda value, *, parent_freeze: dict(value),
    )


def _expected_indoor_stable_arguments():
    return [
        "nav2_profile:=v6_low_obstacle_isolation",
        f"nav2_config_file:={v6_formal_module._canonical_nav2_config()}",
        f"spawn_poses_file:={v6_formal_module._canonical_indoor_spawn_manifest()}",
        "navigation_execution_backend:=route_guided",
        "require_module2_planning_ready:=true",
        "module2_planning_ready_timeout_sec:=120.0",
        "clear_slam_localization_buffer:=false",
        "reset_map_base_translation_tolerance_m:=0.1",
    ]


def _write_indoor_pilot_inputs(tmp_path: Path, monkeypatch):
    _mock_validator_promotion(monkeypatch)
    pilot_manifest, aggregate_path, _reference = _write_sufficient_pilot_inputs(
        tmp_path
    )
    pilot = json.loads(pilot_manifest.read_text())
    pilot["schema_version"] = v6_formal_module.INDOOR_PILOT_MANIFEST_SCHEMA
    pilot["intended_use"] = "indoor_pilot"
    pilot["conditions"] = [
        next(row for row in pilot["conditions"] if row["id"] == condition_id)
        for condition_id in v6_formal_module.INDOOR_CONDITION_IDS
    ]
    for row in pilot["conditions"]:
        row["runner_arguments"].extend([
            f"spawn_poses_file:={v6_formal_module._canonical_indoor_spawn_manifest()}",
            "module2_planning_ready_timeout_sec:=120.0",
            "clear_slam_localization_buffer:=false",
            "reset_map_base_translation_tolerance_m:=0.1",
        ])
    pilot["freeze"]["scenarios"] = {
        condition_id: pilot["freeze"]["scenarios"][condition_id]
        for condition_id in v6_formal_module.INDOOR_CONDITION_IDS
    }
    pilot["freeze"]["scenario_configs"] = {
        condition_id: pilot["freeze"]["scenario_configs"][condition_id]
        for condition_id in v6_formal_module.INDOOR_CONDITION_IDS
    }
    experiment_launch = REPO / "ros2_ws/src/robot_experiments/launch/experiment.launch.py"
    experiment_launch_entry = {
        "path": str(experiment_launch.resolve()),
        "sha256": hashlib.sha256(experiment_launch.read_bytes()).hexdigest(),
    }
    for rows in pilot["freeze"]["scenario_configs"].values():
        rows.append(dict(experiment_launch_entry))
    pilot["freeze"]["frozen_assets"] = {
        name: pilot["freeze"]["frozen_assets"][name]
        for name in v6_formal_module.INDOOR_FROZEN_ASSET_KEYS
    }
    for name, path in (
        ("indoor_map_yaml", REPO / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml"),
        ("indoor_map_pgm", REPO / "data/maps/occupancy/v6_kujiale_isaacgen_v1.pgm"),
    ):
        pilot["freeze"]["frozen_assets"][name] = {
            "path": str(path.resolve()),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
    spawn_manifest = v6_formal_module._canonical_indoor_spawn_manifest()
    physical_contracts = {}
    for row in pilot["conditions"]:
        scenario_path = Path(row["scenario_file"])
        scenario = load_scenario(scenario_path)
        static_ids, dynamic_ids = v6_formal_module.INDOOR_EXPECTED_OBSTACLES[
            row["id"]
        ]
        assert scenario.dynamic_config_file is not None

        def frozen_file(path: Path) -> dict[str, str]:
            path = path.resolve()
            return {
                "path": str(path),
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }

        physical_contracts[row["id"]] = {
            "static_obstacle_ids": list(static_ids),
            "static_obstacle_count": len(static_ids),
            "dynamic_actor_ids": list(dynamic_ids),
            "dynamic_actor_count": len(dynamic_ids),
            "physical_config": frozen_file(
                scenario.resolve_path(scenario.dynamic_config_file)
            ),
            "scenario": frozen_file(scenario_path),
            "spawn_manifest": frozen_file(spawn_manifest),
        }
    pilot["freeze"]["physical_contracts"] = physical_contracts
    pilot["freeze"]["validator_only_head_promotion"] = _fake_validator_promotion(
        pilot["freeze"]
    )
    indoor_manifest = tmp_path / "indoor-pilot-manifest.json"
    indoor_manifest.write_text(json.dumps(pilot), encoding="utf-8")
    aggregate = json.loads(aggregate_path.read_text())
    aggregate["schema_version"] = v6_formal_module.INDOOR_PILOT_AGGREGATE_SCHEMA
    aggregate["pilot_manifest"] = str(indoor_manifest)
    aggregate["validator_only_head_promotion"] = pilot["freeze"][
        "validator_only_head_promotion"
    ]
    aggregate["conditions"] = [
        next(row for row in aggregate["conditions"] if row["id"] == condition_id)
        for condition_id in v6_formal_module.INDOOR_CONDITION_IDS
    ]
    map_hashes = {
        pilot["freeze"]["frozen_assets"][name]["sha256"]
        for name in ("indoor_map_yaml", "indoor_map_pgm")
    }
    for condition in aggregate["conditions"]:
        for row in condition["episodes"]:
            manifest_path = Path(row["manifest_path"])
            episode = json.loads(manifest_path.read_text())
            episode["provenance"]["map_and_posegraph_hashes"] = {
                f"map-{index}": digest
                for index, digest in enumerate(sorted(map_hashes))
            }
            manifest_path.write_text(json.dumps(episode), encoding="utf-8")
            _refresh_checksums(manifest_path.parent)
    indoor_aggregate = tmp_path / "indoor-pilot-aggregate.json"
    indoor_aggregate.write_text(json.dumps(aggregate), encoding="utf-8")
    return indoor_manifest, indoor_aggregate


def _write_indoor_continuation_inputs(tmp_path: Path, monkeypatch):
    _mock_qualification_tooling_promotion(monkeypatch)
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    parent_path = tmp_path / "nas" / "parent-indoor.json"
    parent_output = tmp_path / "nas" / "parent-output"
    parent = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=parent_path,
        indoor_output_root=parent_output,
    )
    parent_raw = json.loads(parent_path.read_text())
    parent_raw["freeze"] = json.loads(json.dumps(parent.freeze))
    parent_path.write_text(json.dumps(parent_raw), encoding="utf-8")
    contract_root = tmp_path / "parent-contract"
    contract_root.mkdir()
    stack_path = _live_stack_contract(
        contract_root, condition_id="indoor_static"
    )
    stack = json.loads(stack_path.read_text())
    session = stack["stack_session_id"]
    run_root = _write_formal_run(
        parent.conditions[0],
        1,
        strict_success=False,
        collision_detected=True,
        formal_freeze_digest=parent.freeze_digest,
        stack_session_id=session,
        path_deviation_percent=None,
        reset_generation=2,
    )
    manifest_path = run_root / "run_manifest.json"
    summary_path = run_root / "run_summary.json"
    episode = json.loads(manifest_path.read_text())
    summary = json.loads(summary_path.read_text())
    receipt = {
        "schema": "bio_nav.v6_stack_episode_receipt.v1",
        "sequence": 1,
        "baseline": 1,
        "stack_session_id": session,
        "sequence_path": stack["episode_sequence_path"],
        "t2_selector_path": stack["t2_selector_path"],
        "t2_selector_sha256": stack["t2_selector_sha256"],
    }
    episode["stack_episode_receipt"] = receipt
    episode["reset_receipt"]["generation"] = 2
    episode["metrics"]["path_deviation_percent"] = None
    episode["provenance"]["git_head"] = parent.freeze["repositories"]["module3"]["head"]
    summary["reset_receipt"]["generation"] = 2
    summary["path_deviation_percent"] = None
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    (run_root / "stack_contract.json").write_bytes(stack_path.read_bytes())
    (run_root / "TRIAL_DISPATCHED.json").write_text(json.dumps({
        "schema": "bio_nav.trial_dispatched.v1",
        "scenario_id": parent.conditions[0].scenario_id,
        "run_index": 1,
        "seed": 8601,
        "condition_stack_id": "indoor_static",
        "stack_session_id": session,
        "formal_freeze_digest": parent.freeze_digest,
        "navigation_execution_backend": "route_guided",
        "stack_episode_receipt": receipt,
    }), encoding="utf-8")
    _refresh_checksums(run_root)
    continuation_file_names = {
        "manifest": "run_manifest.json",
        "summary": "run_summary.json",
        "trial": "TRIAL_DISPATCHED.json",
        "checksums": "checksums.sha256",
        "stack_contract": "stack_contract.json",
    }
    expectations = {
        "parent_manifest": str(parent_path),
        "parent_manifest_sha256": hashlib.sha256(parent_path.read_bytes()).hexdigest(),
        "parent_freeze_digest": parent.freeze_digest,
        "parent_output_root": str(parent_output),
        "preserved_run_root": str(run_root),
        "stack_session_id": session,
        "files": {
            name: hashlib.sha256((run_root / filename).read_bytes()).hexdigest()
            for name, filename in continuation_file_names.items()
        },
    }
    monkeypatch.setattr(
        v6_formal_module,
        "_indoor_continuation_parent_expectations",
        lambda: json.loads(json.dumps(expectations)),
    )
    successor_path = tmp_path / "nas" / "successor-indoor.json"
    successor_output = tmp_path / "nas" / "successor-output"
    successor = v6_formal_module.create_indoor_continuation_campaign(
        parent_manifest_path=parent_path,
        output_manifest_path=successor_path,
        successor_output_root=successor_output,
    )
    return parent_path, parent, run_root, successor_path, successor


def _write_continuation_successor_run(
    condition,
    run_index,
    *,
    freeze_digest,
    stack_session_id,
    strict_success=True,
    collision_detected=False,
    path_deviation_percent=10.0,
):
    root = _write_formal_run(
        condition,
        run_index,
        strict_success=strict_success,
        collision_detected=collision_detected,
        formal_freeze_digest=freeze_digest,
        stack_session_id=stack_session_id,
        path_deviation_percent=path_deviation_percent,
    )
    manifest_path = root / "run_manifest.json"
    episode = json.loads(manifest_path.read_text())
    episode["stack_episode_receipt"] = {
        "schema": "bio_nav.v6_stack_episode_receipt.v1",
        "sequence": run_index - 1,
        "baseline": 1,
        "stack_session_id": stack_session_id,
        "sequence_path": str(root / "episode.sequence.json"),
        "t2_selector_path": str(REPO / "scripts/run_v6_kujiale_low_obstacles.sh"),
        "t2_selector_sha256": hashlib.sha256(
            (REPO / "scripts/run_v6_kujiale_low_obstacles.sh").read_bytes()
        ).hexdigest(),
    }
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    _refresh_checksums(root)
    return root


def _write_production_pilot_root(tmp_path: Path, monkeypatch):
    source_root = tmp_path / "source"
    source_root.mkdir()
    _pilot_manifest, aggregate_path, _reference = _write_sufficient_pilot_inputs(
        source_root
    )
    aggregate = json.loads(aggregate_path.read_text())
    nas_root = tmp_path / "nas"
    pilot_root = nas_root / "sufficient-pilot"
    pilot_root.mkdir(parents=True)
    module3_root = Path(
        "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
        "bio_nav_module3"
    )
    asset_root = tmp_path / "runtime-assets"
    for relative in (
        "weights/module1_mamba_metric_sensor_warm_v8.pt",
        "weights/module2_srdr_v310_seed20260822.pt",
        "weights/module2_srdr_v310_kujiale_0026_visual_heads_shadow_v1.pt",
        "weights/kujiale_0026_visual_heads_run4_v310.pt",
        "third_party/dinov2/weights/dinov2_vits14_pretrain.pth",
    ):
        path = asset_root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(relative, encoding="utf-8")
    snapshot = tmp_path / "snapshot"
    snapshot.mkdir()
    for name in ("manifest.json", "m_sr.npy", "m_dr.npy", "transition.npy", "valid_state_mask.npy"):
        (snapshot / name).write_text(name, encoding="utf-8")
    catalog = tmp_path / "catalog"
    (catalog / "constraints").mkdir(parents=True)
    (catalog / "catalog.json").write_text("{}\n", encoding="utf-8")
    (catalog / "constraints/region_02.json").write_text("{}\n", encoding="utf-8")
    rivermark_usd = tmp_path / "rivermark.usd"
    rivermark_usd.write_text("usd", encoding="utf-8")
    monkeypatch.setenv(
        "BIO_NAV_INTEGRATION_ROOT",
        "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration",
    )
    monkeypatch.setenv(
        "BIO_NAV_MODULE2_ROOT",
        "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module2",
    )
    monkeypatch.setenv("BIO_NAV_MODULE3_ROOT", str(module3_root))
    monkeypatch.setattr(
        v6_formal_module,
        "__file__",
        str(
            module3_root
            / "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py"
        ),
    )
    monkeypatch.setenv("BIO_NAV_MODULE2_ASSET_ROOT", str(asset_root))
    monkeypatch.setenv("BIO_NAV_ROUTE_PRIOR_SNAPSHOT", str(snapshot))
    monkeypatch.setenv("BIO_NAV_ROUTE_PRIOR_CATALOG", str(catalog))
    monkeypatch.setenv("RIVERMARK_USD", str(rivermark_usd))
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", nas_root)
    map_hashes = {
        "indoor": {
            hashlib.sha256(
                (module3_root / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml").read_bytes()
            ).hexdigest(),
            hashlib.sha256(
                (module3_root / "data/maps/occupancy/v6_kujiale_isaacgen_v1.pgm").read_bytes()
            ).hexdigest(),
        },
        "outdoor": {
            hashlib.sha256(
                (module3_root / "data/rivermark_demo/rivermark_selected.yaml").read_bytes()
            ).hexdigest(),
            hashlib.sha256(
                (module3_root / "data/rivermark_demo/rivermark_selected.pgm").read_bytes()
            ).hexdigest(),
        },
    }
    for condition in aggregate["conditions"]:
        scenario_id = load_scenario(
            module3_root
            / "ros2_ws/src/robot_experiments/config"
            / v6_formal_module.PILOT_SCENARIO_FILENAMES[condition["id"]]
        ).scenario_id
        for episode_row in condition["episodes"]:
            rep = episode_row["rep"]
            source_run = Path(episode_row["summary_path"]).parent
            destination = (
                pilot_root / condition["id"] / f"rep{rep}" / scenario_id / source_run.name
            )
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copytree(source_run, destination)
            contract_source = Path(episode_row["stack_contract_path"])
            shutil.copy2(contract_source, destination / "stack_contract.json")
            summary_path = destination / "run_summary.json"
            manifest_path = destination / "run_manifest.json"
            summary = json.loads(summary_path.read_text())
            manifest = json.loads(manifest_path.read_text())
            if "stack_contract.json" not in summary["evidence"]["required_files"]:
                summary["evidence"]["required_files"].append("stack_contract.json")
            manifest["provenance"]["map_and_posegraph_hashes"] = {
                f"map-{index}": digest
                for index, digest in enumerate(sorted(map_hashes[condition["scene"]]))
            }
            summary_path.write_text(json.dumps(summary), encoding="utf-8")
            manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
            _refresh_checksums(destination)
            stem = (
                f"{scenario_id}-run-{rep:04d}-seed-{manifest['random_seed']}-"
                f"20260831T120000.{rep:06d}Z"
            )
            write_run_report(manifest, destination.parents[1], stem)
    return pilot_root


def _write_indoor_production_pilot_root(tmp_path: Path, monkeypatch):
    _mock_validator_promotion(monkeypatch)
    pilot_root = _write_production_pilot_root(tmp_path, monkeypatch)
    for condition_id in set(v6_formal_module.FORMAL_CONDITION_IDS) - set(
        v6_formal_module.INDOOR_CONDITION_IDS
    ):
        shutil.rmtree(pilot_root / condition_id)
    repositories = {
        name: v6_formal_module._repository_freeze_entry_with_tree(path)
        for name, path in {
            "integration": REPO.parent / "bio_nav_integration",
            "module2": REPO.parent / "bio_nav_module2",
            "module3": REPO,
        }.items()
    }
    for condition_id in v6_formal_module.INDOOR_CONDITION_IDS:
        run_roots = [
            next((pilot_root / condition_id / f"rep{rep}").glob("*/run-*"))
            for rep in range(1, 4)
        ]
        stack = json.loads((run_roots[0] / "stack_contract.json").read_text())
        stack["schema"] = v6_formal_module.STACK_CONTRACT_SCHEMA
        stack["contract_path"] = str(
            (pilot_root / condition_id / "runtime/stack.contract.json").resolve()
        )
        for name, entry in repositories.items():
            stack[f"{name}_head"] = entry["head"]
            stack[f"{name}_tree"] = entry["tree"]
        stack.update({
            "viewport_arm": "not_applicable",
            "disable_viewport_updates_requested": "not_applicable",
            "disable_viewport_updates_observed": "not_applicable",
            "viewport_startup_attestation_sha256": "not_applicable",
            "viewport_runtime_attestation_path": "not_applicable",
            "viewport_runtime_attestation_sha256": "not_applicable",
            "viewport_instance_uuid": "not_applicable",
            "viewport_pid": "not_applicable",
            "viewport_pgid": "not_applicable",
            "viewport_start_ticks": "not_applicable",
            "viewport_winner_manifest_sha256": "not_applicable",
        })
        stack["stack_session_id"] = v6_formal_module._stack_session_id(stack)
        stack_bytes = json.dumps(stack).encode()
        stack_digest = hashlib.sha256(stack_bytes).hexdigest()
        for rep, run_root in enumerate(run_roots, start=1):
            stack_path = run_root / "stack_contract.json"
            stack_path.write_bytes(stack_bytes)
            manifest_path = run_root / "run_manifest.json"
            summary_path = run_root / "run_summary.json"
            manifest = json.loads(manifest_path.read_text())
            summary = json.loads(summary_path.read_text())
            receipt = manifest["stack_episode_receipt"]
            receipt.update({
                "sequence": rep,
                "startup_kind": "cold" if rep == 1 else "hot_reset",
                "stack_session_id": stack["stack_session_id"],
                "condition_stack_contract_path": stack["contract_path"],
                "condition_stack_contract_sha256": stack_digest,
                "viewport_arm": "not_applicable",
                "disable_viewport_updates_requested": "not_applicable",
                "disable_viewport_updates_observed": "not_applicable",
                "viewport_startup_attestation_sha256": "not_applicable",
                "viewport_runtime_attestation_sha256": "not_applicable",
                "viewport_instance_uuid": "not_applicable",
                "viewport_pid": "not_applicable",
                "viewport_pgid": "not_applicable",
                "viewport_start_ticks": "not_applicable",
                "viewport_winner_manifest_sha256": "not_applicable",
            })
            manifest["stack_session_id"] = stack["stack_session_id"]
            summary["stack_session_id"] = stack["stack_session_id"]
            summary["condition_stack_attestation"].update({
                "stack_session_id": stack["stack_session_id"],
                "stack_episode_receipt": receipt,
                "condition_stack_contract_path": stack["contract_path"],
                "condition_stack_contract_sha256": stack_digest,
            })
            manifest_path.write_text(json.dumps(manifest))
            summary_path.write_text(json.dumps(summary))
            _refresh_checksums(run_root)
            sidecars = [path for path in run_root.parents[1].iterdir() if path.is_file()]
            stem = sidecars[0].stem
            for path in sidecars:
                path.unlink()
            write_run_report(manifest, run_root.parents[1], stem)
    return pilot_root


def _write_outdoor_production_pilot_root(tmp_path: Path, monkeypatch):
    pilot_root = _write_production_pilot_root(tmp_path, monkeypatch)
    for condition_id in set(v6_formal_module.FORMAL_CONDITION_IDS) - set(
        v6_formal_module.OUTDOOR_CONDITION_IDS
    ):
        shutil.rmtree(pilot_root / condition_id)
    winner = tmp_path / "viewport-winner.json"
    winner.write_text(json.dumps({
        "schema": "startup-ab-fixture", "winner": {"viewport_arm": "B"}
    }))
    monkeypatch.setenv(
        "BIO_NAV_RIVERMARK_STARTUP_WINNER_MANIFEST", str(winner.resolve())
    )
    winner_digest = hashlib.sha256(winner.read_bytes()).hexdigest()
    repositories = {
        name: v6_formal_module._repository_freeze_entry_with_tree(path)
        for name, path in {
            "integration": REPO.parent / "bio_nav_integration",
            "module2": REPO.parent / "bio_nav_module2",
            "module3": REPO,
        }.items()
    }
    for condition_id in v6_formal_module.OUTDOOR_CONDITION_IDS:
        run_roots = [
            next((pilot_root / condition_id / f"rep{rep}").glob("*/run-*"))
            for rep in range(1, 4)
        ]
        runtime_root = tmp_path / "outdoor-runtime" / condition_id
        runtime_root.mkdir(parents=True)
        runtime_attestation = runtime_root / "viewport_runtime_attestation.json"
        own_pid = os.getpid()
        fields = Path(f"/proc/{own_pid}/stat").read_text().rsplit(")", 1)[1].split()
        runtime_payload = {
            "schema": "bio_nav.v6_viewport_runtime_attestation.v1",
            "instance_uuid": (
                "550e8400-e29b-41d4-a716-44665544"
                f"{v6_formal_module.OUTDOOR_CONDITION_IDS.index(condition_id) + 1:04d}"
            ),
            "pid": own_pid,
            "pgid": int(fields[2]),
            "start_ticks": int(fields[19]),
            "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
            "argv_sha256": hashlib.sha256(
                Path(f"/proc/{own_pid}/cmdline").read_bytes()
            ).hexdigest(),
            "producer_executable": str(Path(f"/proc/{own_pid}/exe").resolve()),
            "start_wall_time_ns": time.time_ns(),
            "module3": repositories["module3"],
            "producer_script_realpath": str(
                (REPO / "isaac_sim/apps/navigation_sim.py").resolve()
            ),
            "producer_script_sha256": hashlib.sha256(
                (REPO / "isaac_sim/apps/navigation_sim.py").read_bytes()
            ).hexdigest(),
            "viewport_arm": "B",
            "readbacks": [
                {"phase": phase, "requested_disabled": True,
                 "observed_enabled": False, "match": True}
                for phase in ("post_construction", "pre_ready")
            ],
            "scene": f"rivermark:{condition_id.split('_', 1)[1]}",
            "run_root": str(runtime_root.resolve()),
            "launcher_path": str((REPO / "scripts/run_v6_rivermark.sh").resolve()),
            "winner_manifest": {"path": str(winner.resolve()), "sha256": winner_digest},
            "command_contract": {
                "navigation_source": str(
                    (REPO / "isaac_sim/apps/navigation_sim.py").resolve()
                ),
                "producer_executable": str(Path(f"/proc/{own_pid}/exe").resolve()),
                "argv_sha256": hashlib.sha256(
                    Path(f"/proc/{own_pid}/cmdline").read_bytes()
                ).hexdigest(),
                "disable_viewport_updates": True,
                "viewport_arm_identity": "B",
                "viewport_runtime_attestation": str(runtime_attestation.resolve()),
                "viewport_winner_manifest": str(winner.resolve()),
                "viewport_winner_manifest_sha256": winner_digest,
                "viewport_run_root": str(runtime_root.resolve()),
                "viewport_scene": f"rivermark:{condition_id.split('_', 1)[1]}",
                "viewport_launcher": str(
                    (REPO / "scripts/run_v6_rivermark.sh").resolve()
                ),
            },
        }
        runtime_attestation.write_text(json.dumps(runtime_payload))
        runtime_attestation.chmod(0o600)
        runtime_digest = hashlib.sha256(runtime_attestation.read_bytes()).hexdigest()
        stack = json.loads((run_roots[0] / "stack_contract.json").read_text())
        stack["schema"] = v6_formal_module.STACK_CONTRACT_SCHEMA
        stack["contract_path"] = str((runtime_root / "stack.contract.json").resolve())
        for name, entry in repositories.items():
            stack[f"{name}_head"] = entry["head"]
            stack[f"{name}_tree"] = entry["tree"]
        stack.update({
            "viewport_arm": "B",
            "disable_viewport_updates_requested": True,
            "disable_viewport_updates_observed": True,
            "viewport_startup_attestation_sha256": runtime_digest,
            "viewport_runtime_attestation_path": str(runtime_attestation.resolve()),
            "viewport_runtime_attestation_sha256": runtime_digest,
            "viewport_instance_uuid": runtime_payload["instance_uuid"],
            "viewport_pid": runtime_payload["pid"],
            "viewport_pgid": runtime_payload["pgid"],
            "viewport_start_ticks": runtime_payload["start_ticks"],
            "viewport_winner_manifest_sha256": winner_digest,
        })
        stack["stack_session_id"] = v6_formal_module._stack_session_id(stack)
        stack_bytes = json.dumps(stack).encode()
        stack_digest = hashlib.sha256(stack_bytes).hexdigest()
        for rep, run_root in enumerate(run_roots, start=1):
            rep_root = run_root.parents[1]
            stack_path = run_root / "stack_contract.json"
            stack_path.write_bytes(stack_bytes)
            viewport_copy = run_root / "viewport_runtime_attestation.json"
            viewport_copy.write_bytes(runtime_attestation.read_bytes())
            viewport_copy.chmod(0o600)
            manifest_path = run_root / "run_manifest.json"
            summary_path = run_root / "run_summary.json"
            manifest = json.loads(manifest_path.read_text())
            summary = json.loads(summary_path.read_text())
            receipt = manifest["stack_episode_receipt"]
            receipt.update({
                "sequence": rep,
                "startup_kind": "cold" if rep == 1 else "hot_reset",
                "stack_session_id": stack["stack_session_id"],
                "condition_stack_contract_path": stack["contract_path"],
                "condition_stack_contract_sha256": stack_digest,
                "viewport_arm": "B",
                "disable_viewport_updates_requested": True,
                "disable_viewport_updates_observed": True,
                "viewport_startup_attestation_sha256": runtime_digest,
                "viewport_runtime_attestation_sha256": runtime_digest,
                "viewport_instance_uuid": runtime_payload["instance_uuid"],
                "viewport_pid": runtime_payload["pid"],
                "viewport_pgid": runtime_payload["pgid"],
                "viewport_start_ticks": runtime_payload["start_ticks"],
                "viewport_winner_manifest_sha256": winner_digest,
            })
            manifest["stack_session_id"] = stack["stack_session_id"]
            summary["stack_session_id"] = stack["stack_session_id"]
            summary["condition_stack_attestation"].update({
                "stack_session_id": stack["stack_session_id"],
                "stack_episode_receipt": receipt,
                "condition_stack_contract_path": stack["contract_path"],
                "condition_stack_contract_sha256": stack_digest,
                "viewport_arm": "B",
                "disable_viewport_updates_requested": True,
                "disable_viewport_updates_observed": True,
                "viewport_startup_attestation_sha256": runtime_digest,
            })
            manifest_path.write_text(json.dumps(manifest))
            summary_path.write_text(json.dumps(summary))
            _refresh_checksums(run_root)
            sidecars = [path for path in rep_root.iterdir() if path.is_file()]
            stem = sidecars[0].stem
            for path in sidecars:
                path.unlink()
            write_run_report(manifest, rep_root, stem)
    return pilot_root


def ready_facts() -> ReadinessFacts:
    return ReadinessFacts(
        **{name: True for name in ReadinessFacts.__dataclass_fields__}
    )


def ready_guard(*legs: str) -> EpisodeGuard:
    guard = EpisodeGuard(mission_leg_ids=legs)
    guard.arm_reset(ready_facts())
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_receipt_generation(1)
    guard.record_reset_event()
    guard.record_initialpose(100)
    guard.record_amcl(101)
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_gate_status(1, False)
    assert guard.goal_ready
    return guard


def test_r3_phase2_manifest_is_the_only_dispatch_candidate():
    manifest = load_manifest(MANIFEST)

    assert manifest.scene_id == "v6_kujiale_clearance_r2"
    assert [leg.goal_id for leg in manifest.mission_legs] == [
        "G2", "G3", "G4", "G5", "G1"
    ]
    assert manifest.dynamic_schedule == ()
    assert manifest.runtime["canonical_odom"] == {
        "topic": "/odom",
        "owner": "isaac_compute_odometry",
        "tf": "odom->base_link",
    }
    assert manifest.runtime["global_localization"] == {
        "pose_topic": "/amcl_pose",
        "owner": "amcl",
        "tf": "map->odom",
    }
    assert manifest.runtime["module1_odom"] == {
        "topic": "/bio_nav/module1/odom",
        "owner": "wheel_imu_ekf",
        "publish_tf": False,
    }
    assert manifest.runtime["recovery_enabled"] is False
    assert manifest.runtime["module2_navigation_write_enabled"] is False
    assert manifest.runtime["cognitive_place_graph_enabled"] is False
    assert manifest.runtime["route_backend"] == "gvg"
    assert manifest.runtime["low_obstacles_enabled"] is False
    assert manifest.runtime["dynamic_actors_enabled"] is False
    assert manifest.runtime["goal_checker"] == "position_xy"

    text = MANIFEST.read_text(encoding="utf-8")
    for forbidden in ("B5", "M3", "primary", "rf2o"):
        assert forbidden not in text


def test_r5_phase_b_manifest_binds_original_scene_and_shadow_baseline():
    manifest = load_manifest(PHASE_B_MANIFEST)

    assert manifest.scene_id == "kujiale_0026_A_to_B_door_open"
    assert manifest.runtime["cognitive_profile"] == "M0"
    assert manifest.runtime["module1_mode"] == "shadow"
    assert manifest.runtime["module2_navigation_write_enabled"] is False
    assert manifest.runtime["cognitive_place_graph_enabled"] is False
    assert manifest.runtime["route_backend"] == "gvg"
    assert manifest.runtime["low_obstacles_enabled"] is False
    assert manifest.runtime["dynamic_actors_enabled"] is False
    assert manifest.assets["scene_asset"].endswith(
        "/kujiale_0026/kujiale_0026_A_to_B_door_open.usd"
    )
    assert manifest.assets["occupancy_map"].endswith(
        "/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml"
    )
    assert manifest.assets["spawn_manifest"].endswith(
        "/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
    )
    assert manifest.assets["route_graph"].endswith(
        "/v6_kujiale_isaacgen_v1_gvg_v1.geojson"
    )
    assert [leg.goal_id for leg in manifest.mission_legs] == [
        "G2", "G3", "G4", "G5", "G1"
    ]


def test_r5_phase_b_rejects_nonexact_scene_asset(tmp_path):
    raw = yaml.safe_load(PHASE_B_MANIFEST.read_text(encoding="utf-8"))
    raw["assets"]["scene_asset"] = "/tmp/modified_scene.usd"
    with pytest.raises(V6ContractError, match="accepted Phase B asset"):
        load_manifest(_write_manifest(tmp_path, raw))


@pytest.mark.parametrize("path", LEGACY_MANIFESTS)
def test_legacy_campaign_manifests_are_rejected_by_r3_dispatcher(path):
    with pytest.raises(V6ContractError, match="schema_version"):
        load_manifest(path)


def test_r3_phase2_is_pilot_only(capsys):
    manifest = load_manifest(MANIFEST)
    assert authorize_manifest(manifest, mode="pilot") == NOT_QUALIFIED
    with pytest.raises(V6ContractError, match="engineering pilot only"):
        authorize_manifest(manifest, mode="formal")

    assert cli(["--manifest", str(MANIFEST), "--pilot"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["qualification"] == ENGINEERING_PILOT
    assert payload["formal_qualification"] == NOT_QUALIFIED
    assert payload["dispatch"] is False

    assert cli(["--manifest", str(MANIFEST)]) == 2
    assert "engineering pilot only" in capsys.readouterr().err


def test_formal_manifest_dry_run_freezes_six_conditions_and_120_runs(
    tmp_path, capsys
):
    path = _write_formal_manifest(tmp_path)

    assert cli(["--formal-manifest", str(path)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["execution_authorization"] == "NOT_AUTHORIZED"
    assert payload["dispatch"] is False
    assert payload["aggregate"]["expected_episodes"] == 120
    assert payload["aggregate"]["present_episodes"] == 0
    assert payload["aggregate"]["strict_successes"] == 0
    assert [
        row["id"] for row in payload["aggregate"]["conditions"]
    ] == list(v6_formal_module.FORMAL_CONDITION_IDS)
    assert set(payload["resume_points"].values()) == {1}
    assert len(payload["dispatch_plan"]) == 6
    assert all("run_indices:=1" in row["command"] for row in payload["dispatch_plan"])
    assert all(
        row["command"][0] == str(REPO / "scripts" / "run_experiment.sh")
        and row["condition_stack_contract_required"] is True
        for row in payload["dispatch_plan"]
    )
    assert all(
        row["stack_boundary"] == "cold"
        and row["requires_existing_condition_stack"] is True
        for row in payload["dispatch_plan"]
    )

    campaign = load_formal_campaign_manifest(path)
    dynamic = next(
        condition
        for condition in campaign.conditions
        if condition.condition_id == "indoor_dynamic"
    )
    assert len(dynamic.episode_identities) == 20
    assert len({row["seed"] for row in dynamic.episode_identities}) == 4
    assert len({
        (
            row["seed"],
            row["dynamic_case_id"],
            row["dynamic_variant_id"],
        )
        for row in dynamic.episode_identities
    }) == 20


@pytest.mark.parametrize("mutation", ["missing", "relative", "duplicate", "wrong"])
def test_campaign_loader_requires_unique_canonical_absolute_nav2_override(
    tmp_path, mutation
):
    raw = _formal_raw(tmp_path)
    arguments = raw["conditions"][0]["runner_arguments"]
    index = next(
        i for i, value in enumerate(arguments) if value.startswith("nav2_config_file:=")
    )
    if mutation == "missing":
        arguments.pop(index)
    elif mutation == "relative":
        arguments[index] = "nav2_config_file:=nav2_v6_low_obstacle_isolation.yaml"
    elif mutation == "duplicate":
        arguments.append(arguments[index])
    else:
        arguments[index] = f"nav2_config_file:={tmp_path / 'wrong.yaml'}"
    path = tmp_path / "formal.json"
    path.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(V6ContractError, match="nav2_config_file|duplicate"):
        load_formal_campaign_manifest(path)


def test_sufficient_pilot_freezer_writes_not_authorized_formal_manifest(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate, reference = _write_sufficient_pilot_inputs(tmp_path)
    output = tmp_path / "frozen-formal.json"
    formal_root = tmp_path / "nas" / "formal-campaign"
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    frozen = freeze_formal_manifest_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=output,
        formal_output_root=formal_root,
    )

    assert output.is_file()
    assert frozen.authorization == "NOT_AUTHORIZED"
    assert frozen.freeze_digest == reference.freeze_digest
    assert [condition.condition_id for condition in frozen.conditions] == list(
        v6_formal_module.FORMAL_CONDITION_IDS
    )
    assert all(
        condition.output_directory == formal_root / condition.condition_id
        for condition in frozen.conditions
    )
    dry_run = json.loads(output.read_text())
    assert dry_run["runs_per_condition"] == 20
    assert dry_run["execution_authorization"] == "NOT_AUTHORIZED"
    assert load_formal_campaign_manifest(output).freeze_digest == frozen.freeze_digest
    assert formal_root.is_dir()
    assert frozen.pilot_freeze_provenance is not None
    assert len(frozen.pilot_freeze_provenance["episodes"]) == 18
    assert frozen.pilot_freeze_provenance["pilot_manifest"]["path"] == str(
        pilot_manifest
    )


def test_sufficient_pilot_aggregate_generator_publishes_18_verified_runs(
    tmp_path, monkeypatch
):
    pilot_root = _write_production_pilot_root(tmp_path, monkeypatch)
    manifest_output = tmp_path / "nas" / "pilot-manifest.json"
    aggregate_output = tmp_path / "nas" / "pilot-aggregate.json"

    result = v6_formal_module.aggregate_sufficient_pilot(
        pilot_root=pilot_root,
        output_manifest=manifest_output,
        output_aggregate=aggregate_output,
    )

    assert result["qualification"] == "SUFFICIENT_PILOT_READY"
    assert result["strict_successes"] == 18
    assert result["dispatch"] is False
    aggregate = json.loads(aggregate_output.read_text())
    assert [row["id"] for row in aggregate["conditions"]] == list(
        v6_formal_module.FORMAL_CONDITION_IDS
    )
    assert all(len(row["episodes"]) == 3 for row in aggregate["conditions"])
    assert all(
        Path(episode["stack_contract_path"]).name == "stack_contract.json"
        for row in aggregate["conditions"]
        for episode in row["episodes"]
    )
    assert result["pilot_manifest"]["sha256"] == hashlib.sha256(
        manifest_output.read_bytes()
    ).hexdigest()
    frozen = freeze_formal_manifest_from_pilot(
        pilot_manifest_path=manifest_output,
        pilot_aggregate_path=aggregate_output,
        output_manifest_path=tmp_path / "nas" / "formal.json",
        formal_output_root=tmp_path / "nas" / "formal-root",
    )
    assert frozen.authorization == "NOT_AUTHORIZED"
    assert frozen.pilot_freeze_provenance is not None


def test_sufficient_pilot_aggregate_cli_never_dispatches(
    tmp_path, monkeypatch, capsys
):
    pilot_root = _write_production_pilot_root(tmp_path, monkeypatch)
    manifest_output = tmp_path / "nas" / "cli-pilot-manifest.json"
    aggregate_output = tmp_path / "nas" / "cli-pilot-aggregate.json"

    assert cli([
        "--aggregate-pilot-root", str(pilot_root),
        "--output-pilot-manifest", str(manifest_output),
        "--output-pilot-aggregate", str(aggregate_output),
    ]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["qualification"] == "SUFFICIENT_PILOT_READY"
    assert payload["strict_successes"] == 18
    assert payload["dispatch"] is False


def _validator_promotion_repository(tmp_path: Path, *, mutation: str | None = None):
    repository = tmp_path / f"promotion-{mutation or 'valid'}"
    repository.mkdir()
    subprocess.run(["git", "init", "-q", str(repository)], check=True)
    experiment_path = repository / (
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
    )
    formal_path = repository / (
        "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py"
    )
    motion_test = repository / "ros2_ws/src/robot_experiments/test/test_experiment_motion_quality.py"
    formal_test = repository / "ros2_ws/src/robot_experiments/test/test_v6_formal.py"
    for path in (experiment_path, formal_path, motion_test, formal_test):
        path.parent.mkdir(parents=True, exist_ok=True)
    experiment_path.write_text(
        "def protected_writer():\n    return 'writer-v1'\n\n"
        "def validate_recorded_run_evidence():\n    return 'validator-v1'\n",
        encoding="utf-8",
    )
    formal_common = (
        "import os\n\n"
        "GLOBAL_VALUE = 1\n\n"
        "@dataclass(frozen=True)\n"
        "class V6FormalNode:\n    live_contract = 'unchanged'\n\n"
        "def _validate_formal_freeze(value):\n    return value\n\n"
        "def load_indoor_campaign_manifest(path):\n    return path\n\n"
        "def freeze_indoor_campaign_from_pilot(value):\n    return value\n\n"
        "def _build_indoor_pilot_manifest(root):\n    return root\n\n"
    )
    formal_tail_v1 = (
        "def aggregate_indoor_pilot(value: str) -> str:\n    return 'v1'\n\n"
        "def execute_indoor_campaign(manifest):\n    return manifest\n\n"
        "def cli(argv=None):\n    return argv\n"
    )
    formal_path.write_text(
        "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v1'\n"
        "INDOOR_PILOT_AGGREGATE_SCHEMA = 'bio_nav_v6_indoor_pilot_aggregate_v1'\n"
        "INDOOR_CAMPAIGN_SCHEMA_VERSION = 'bio_nav_v6_indoor_campaign_v1'\n\n"
        + formal_common
        + formal_tail_v1,
        encoding="utf-8",
    )
    motion_test.write_text("def test_old():\n    assert True\n", encoding="utf-8")
    formal_test.write_text("def test_old():\n    assert True\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(repository), "add", "."], check=True)
    subprocess.run(
        [
            "git", "-C", str(repository), "-c", "user.name=Codex Test", "-c",
            "user.email=codex@example.invalid", "commit", "-qm", "pilot",
        ],
        check=True,
    )
    from_head = subprocess.run(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    experiment_path.write_text(
        "def protected_writer():\n    return 'writer-v1'\n\n"
        "def validate_recorded_run_evidence():\n    return 'validator-v2'\n",
        encoding="utf-8",
    )
    new_helpers = (
        "\ndef _validator_only_git_output(repository, arguments, *, binary):\n    return None\n"
        "\ndef _validator_only_diff_evidence(repository, from_head, to_head):\n    return None\n"
        "\ndef _validator_only_ast_guard(repository, from_head, to_head):\n    return None\n"
        "\ndef _validator_only_loaded_identity(module3_root, current_head):\n    return None\n"
        "\ndef _validate_validator_only_head_promotion(value, *, freeze):\n    return None\n"
        "\ndef _build_validator_only_head_promotion(*, freeze, pilot_runtime):\n    return None\n"
        "\ndef _derive_indoor_pilot_runtime(pilot_root, *, repositories):\n    return None\n"
        "\ndef _pilot_freeze_from_validator_promotion(freeze, promotion):\n    return None\n"
        "\ndef _revalidate_indoor_pilot_freeze_provenance("
        "provenance, *, conditions, freeze, freeze_digest):\n    return None\n"
    )
    formal_tail_v2 = (
        "def aggregate_indoor_pilot(value: str) -> str:\n    return 'v2'\n\n"
        "def execute_indoor_campaign(manifest):\n    return 'validated'\n\n"
        "def cli(argv=None):\n    return argv\n"
    )
    formal_path.write_text(
        "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v2'\n"
        "INDOOR_PILOT_AGGREGATE_SCHEMA = 'bio_nav_v6_indoor_pilot_aggregate_v2'\n"
        "INDOOR_CAMPAIGN_SCHEMA_VERSION = 'bio_nav_v6_indoor_campaign_v2'\n\n"
        + formal_common
        + formal_tail_v2
        + new_helpers,
        encoding="utf-8",
    )
    motion_test.write_text("def test_new():\n    assert True\n", encoding="utf-8")
    formal_test.write_text("def test_new():\n    assert True\n", encoding="utf-8")
    if mutation == "product_symbol":
        experiment_path.write_text(
            experiment_path.read_text().replace("writer-v1", "writer-v2"),
            encoding="utf-8",
        )
    elif mutation == "function_default_call":
        formal_path.write_text(
            formal_path.read_text().replace(
                "def aggregate_indoor_pilot(value: str) -> str:",
                "def aggregate_indoor_pilot(value: str = import_time_call()) -> str:",
            ),
            encoding="utf-8",
        )
    elif mutation == "function_decorator":
        formal_path.write_text(
            formal_path.read_text().replace(
                "def aggregate_indoor_pilot(value: str) -> str:",
                "@changed\ndef aggregate_indoor_pilot(value: str) -> str:",
            ),
            encoding="utf-8",
        )
    elif mutation == "function_annotation":
        formal_path.write_text(
            formal_path.read_text().replace(
                "def aggregate_indoor_pilot(value: str) -> str:",
                "def aggregate_indoor_pilot(value: bytes) -> bytes:",
            ),
            encoding="utf-8",
        )
    elif mutation == "class_frozen":
        formal_path.write_text(
            formal_path.read_text().replace("@dataclass(frozen=True)", "@dataclass(frozen=False)"),
            encoding="utf-8",
        )
    elif mutation == "class_base":
        formal_path.write_text(
            formal_path.read_text().replace("class V6FormalNode:", "class V6FormalNode(BaseNode):"),
            encoding="utf-8",
        )
    elif mutation == "class_decorator":
        formal_path.write_text(
            formal_path.read_text().replace("@dataclass(frozen=True)", "@changed\n@dataclass(frozen=True)"),
            encoding="utf-8",
        )
    elif mutation == "class_field":
        formal_path.write_text(
            formal_path.read_text().replace(
                "    live_contract = 'unchanged'", "    live_contract = 'unchanged'\n    new_field = 1"
            ),
            encoding="utf-8",
        )
    elif mutation == "class_method":
        formal_path.write_text(
            formal_path.read_text().replace(
                "    live_contract = 'unchanged'",
                "    live_contract = 'unchanged'\n\n    def new_method(self):\n        return 1",
            ),
            encoding="utf-8",
        )
    elif mutation == "import_change":
        formal_path.write_text(
            formal_path.read_text().replace("import os", "import sys"), encoding="utf-8"
        )
    elif mutation == "global_change":
        formal_path.write_text(
            formal_path.read_text().replace("GLOBAL_VALUE = 1", "GLOBAL_VALUE = 2"),
            encoding="utf-8",
        )
    elif mutation == "schema_call":
        formal_path.write_text(
            formal_path.read_text().replace(
                "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v2'",
                "INDOOR_PILOT_MANIFEST_SCHEMA = import_time_call()",
            ),
            encoding="utf-8",
        )
    elif mutation == "schema_multi_target":
        formal_path.write_text(
            formal_path.read_text().replace(
                "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v2'",
                "INDOOR_PILOT_MANIFEST_SCHEMA = os.environ['PROMOTION_SIDE_EFFECT'] = "
                "'bio_nav_v6_indoor_pilot_manifest_v2'",
            ),
            encoding="utf-8",
        )
    elif mutation == "schema_attribute_target":
        formal_path.write_text(
            formal_path.read_text().replace(
                "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v2'",
                "holder.INDOOR_PILOT_MANIFEST_SCHEMA = "
                "'bio_nav_v6_indoor_pilot_manifest_v2'",
            ),
            encoding="utf-8",
        )
    elif mutation == "schema_subscript_target":
        formal_path.write_text(
            formal_path.read_text().replace(
                "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v2'",
                "holder['INDOOR_PILOT_MANIFEST_SCHEMA'] = "
                "'bio_nav_v6_indoor_pilot_manifest_v2'",
            ),
            encoding="utf-8",
        )
    elif mutation == "schema_tuple_target":
        formal_path.write_text(
            formal_path.read_text().replace(
                "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v2'",
                "(INDOOR_PILOT_MANIFEST_SCHEMA, other) = "
                "('bio_nav_v6_indoor_pilot_manifest_v2', 'x')",
            ),
            encoding="utf-8",
        )
    elif mutation == "schema_renamed_target":
        formal_path.write_text(
            formal_path.read_text().replace(
                "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v2'",
                "RENAMED_INDOOR_PILOT_MANIFEST_SCHEMA = "
                "'bio_nav_v6_indoor_pilot_manifest_v2'",
            ),
            encoding="utf-8",
        )
    elif mutation == "schema_annassign_target":
        formal_path.write_text(
            formal_path.read_text().replace(
                "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v2'",
                "INDOOR_PILOT_MANIFEST_SCHEMA: str = "
                "'bio_nav_v6_indoor_pilot_manifest_v2'",
            ),
            encoding="utf-8",
        )
    elif mutation == "schema_type_comment":
        formal_path.write_text(
            formal_path.read_text().replace(
                "INDOOR_PILOT_MANIFEST_SCHEMA = 'bio_nav_v6_indoor_pilot_manifest_v2'",
                "INDOOR_PILOT_MANIFEST_SCHEMA = "
                "'bio_nav_v6_indoor_pilot_manifest_v2'  # type: str",
            ),
            encoding="utf-8",
        )
    elif mutation == "cli_change":
        formal_path.write_text(
            formal_path.read_text().replace("def cli(argv=None):\n    return argv", "def cli(argv=None):\n    return 'changed'"),
            encoding="utf-8",
        )
    elif mutation == "helper_default_call":
        formal_path.write_text(
            formal_path.read_text().replace(
                "def _validator_only_git_output(repository, arguments, *, binary):",
                "def _validator_only_git_output(repository, arguments=import_time_call(), *, binary):",
            ),
            encoding="utf-8",
        )
    elif mutation == "helper_decorator":
        formal_path.write_text(
            formal_path.read_text().replace(
                "def _validator_only_git_output(repository, arguments, *, binary):",
                "@changed\ndef _validator_only_git_output(repository, arguments, *, binary):",
            ),
            encoding="utf-8",
        )
    elif mutation in {"runner_path", "writer_path", "scenario_path", "navigation_path"}:
        relative = {
            "runner_path": "scripts/run_experiment.sh",
            "writer_path": "ros2_ws/src/robot_experiments/robot_experiments/writer.py",
            "scenario_path": "ros2_ws/src/robot_experiments/config/scenario.yaml",
            "navigation_path": "ros2_ws/src/robot_navigation/config/nav2.yaml",
        }[mutation]
        path = repository / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("changed\n", encoding="utf-8")
    elif mutation == "rename":
        renamed = formal_test.with_name("test_v6_formal_renamed.py")
        formal_test.rename(renamed)
    elif mutation == "delete":
        motion_test.unlink()
    subprocess.run(["git", "-C", str(repository), "add", "-A"], check=True)
    subprocess.run(
        [
            "git", "-C", str(repository), "-c", "user.name=Codex Test", "-c",
            "user.email=codex@example.invalid", "commit", "-qm", "promotion",
        ],
        check=True,
    )
    to_head = subprocess.run(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    repositories = {
        name: {"path": str(repository), "head": to_head}
        for name in ("integration", "module2", "module3")
    }
    freeze = {
        "repositories": repositories,
        "driver_version": "driver",
        "kernel_release": "kernel",
    }
    pilot_runtime = {
        "repositories": {
            **repositories,
            "module3": {"path": str(repository), "head": from_head},
        },
        "driver_version": "driver",
        "kernel_release": "kernel",
    }
    return repository, from_head, to_head, freeze, pilot_runtime


def _promotion_loaded_identity(repository: Path, head: str):
    return {
        "module": "robot_experiments.experiment_runner",
        "symbol": "validate_recorded_run_evidence",
        "source_path": str(repository / "validator.py"),
        "source_sha256": "1" * 64,
        "git_blob_oid": "blob",
        "current_head": head,
    }


def _qualification_promotion_repository(tmp_path: Path, *, extra_path=False):
    repository = tmp_path / ("qualification-extra" if extra_path else "qualification")
    repository.mkdir()
    subprocess.run(["git", "init", "-q", str(repository)], check=True)
    paths = {
        "v6_formal": "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
        "test": "ros2_ws/src/robot_experiments/test/test_v6_formal.py",
        "wrapper": "scripts/run_v6_formal_episode.sh",
        "experiment_runner": "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py",
        "selector": "scripts/run_v6_kujiale_low_obstacles.sh",
        "launch": "ros2_ws/src/robot_experiments/launch/experiment.launch.py",
    }
    for name, relative in paths.items():
        path = repository / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"{name}-v1\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(repository), "add", "."], check=True)
    subprocess.run(
        [
            "git", "-C", str(repository), "-c", "user.name=Codex Test", "-c",
            "user.email=codex@example.invalid", "commit", "-qm", "parent",
        ],
        check=True,
    )
    from_head = subprocess.run(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    for name in ("v6_formal", "test", "wrapper"):
        (repository / paths[name]).write_text(f"{name}-v2\n", encoding="utf-8")
    if extra_path:
        extra = repository / "ros2_ws/src/robot_navigation/config/nav2.yaml"
        extra.parent.mkdir(parents=True, exist_ok=True)
        extra.write_text("changed\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(repository), "add", "-A"], check=True)
    subprocess.run(
        [
            "git", "-C", str(repository), "-c", "user.name=Codex Test", "-c",
            "user.email=codex@example.invalid", "commit", "-qm", "tooling",
        ],
        check=True,
    )
    to_head = subprocess.run(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    shared = {
        "integration": {"path": "/integration", "head": "i" * 40},
        "module2": {"path": "/module2", "head": "2" * 40},
    }
    parent_freeze = {
        "repositories": {
            **shared,
            "module3": {"path": str(repository), "head": from_head},
        },
        "validator_only_head_promotion": {"schema": "fixture"},
    }
    freeze = {
        "repositories": {
            **shared,
            "module3": {"path": str(repository), "head": to_head},
        },
        "v6_formal": {
            "path": str((repository / paths["v6_formal"]).resolve()),
            "sha256": hashlib.sha256(
                (repository / paths["v6_formal"]).read_bytes()
            ).hexdigest(),
        },
    }
    return repository, freeze, parent_freeze


def test_validator_only_head_promotion_accepts_exact_offline_symbol_delta(
    tmp_path, monkeypatch
):
    repository, _from, to_head, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path)
    )
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: _promotion_loaded_identity(repository, head),
    )

    promotion = v6_formal_module._build_validator_only_head_promotion(
        freeze=freeze, pilot_runtime=pilot_runtime
    )

    assert promotion["module3_diff"]["from_is_ancestor"] is True
    assert {row["status"] for row in promotion["module3_diff"]["name_status"]} == {"M"}
    assert len(promotion["module3_diff"]["name_status"]) == 4
    assert promotion["loaded_validator"]["current_head"] == to_head


def test_qualification_tooling_promotion_accepts_only_bound_tooling_delta(
    tmp_path, monkeypatch
):
    _repository, freeze, parent_freeze = _qualification_promotion_repository(tmp_path)
    monkeypatch.setattr(
        v6_formal_module, "_qualification_tooling_ast_guard",
        lambda _repository, _from, _to: None,
    )

    promotion = v6_formal_module._build_qualification_tooling_promotion(
        freeze=freeze, parent_freeze=parent_freeze
    )

    assert {row["path"] for row in promotion["name_status"]} == {
        "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
        "ros2_ws/src/robot_experiments/test/test_v6_formal.py",
        "scripts/run_v6_formal_episode.sh",
    }
    assert {row["status"] for row in promotion["name_status"]} == {"M"}
    assert set(promotion["unchanged_files"]) == {
        "experiment_runner", "t2_selector", "experiment_launch"
    }
    promotion["wrapper"]["sha256"] = "0" * 64
    with pytest.raises(V6ContractError, match="source identity"):
        v6_formal_module._validate_qualification_tooling_promotion(
            promotion, freeze=freeze, parent_freeze=parent_freeze
        )


def test_qualification_tooling_promotion_rejects_nonallowlist_path(
    tmp_path, monkeypatch
):
    _repository, freeze, parent_freeze = _qualification_promotion_repository(
        tmp_path, extra_path=True
    )
    monkeypatch.setattr(
        v6_formal_module, "_qualification_tooling_ast_guard",
        lambda _repository, _from, _to: None,
    )

    with pytest.raises(V6ContractError, match="diff mismatch"):
        v6_formal_module._build_qualification_tooling_promotion(
            freeze=freeze, parent_freeze=parent_freeze
        )


@pytest.mark.parametrize(
    "mutation",
    [
        "product_symbol", "runner_path", "writer_path", "scenario_path",
        "navigation_path", "rename", "delete",
    ],
)
def test_validator_only_head_promotion_rejects_product_or_path_delta(
    tmp_path, monkeypatch, mutation
):
    repository, _from, _to, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path, mutation=mutation)
    )
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: _promotion_loaded_identity(repository, head),
    )

    with pytest.raises(V6ContractError, match="disallowed|protected top-level AST"):
        v6_formal_module._build_validator_only_head_promotion(
            freeze=freeze, pilot_runtime=pilot_runtime
        )


@pytest.mark.parametrize(
    "mutation",
    ["function_default_call", "function_decorator", "function_annotation"],
)
def test_validator_only_ast_guard_rejects_callable_header_changes(
    tmp_path, monkeypatch, mutation
):
    repository, _from, _to, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path, mutation=mutation)
    )
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: _promotion_loaded_identity(repository, head),
    )

    with pytest.raises(V6ContractError, match="callable signature/decorators"):
        v6_formal_module._build_validator_only_head_promotion(
            freeze=freeze, pilot_runtime=pilot_runtime
        )


@pytest.mark.parametrize(
    "mutation",
    [
        "class_frozen", "class_base", "class_decorator", "class_field",
        "class_method", "import_change", "global_change", "cli_change",
    ],
)
def test_validator_only_ast_guard_rejects_class_import_global_or_cli_changes(
    tmp_path, monkeypatch, mutation
):
    repository, _from, _to, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path, mutation=mutation)
    )
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: _promotion_loaded_identity(repository, head),
    )

    with pytest.raises(V6ContractError, match="protected top-level AST"):
        v6_formal_module._build_validator_only_head_promotion(
            freeze=freeze, pilot_runtime=pilot_runtime
        )


@pytest.mark.parametrize("mutation", ["helper_default_call", "helper_decorator"])
def test_validator_only_ast_guard_rejects_new_helper_decorator_or_default(
    tmp_path, monkeypatch, mutation
):
    repository, _from, _to, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path, mutation=mutation)
    )
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: _promotion_loaded_identity(repository, head),
    )

    with pytest.raises(V6ContractError, match="helper kind/decorator|helper signature/default"):
        v6_formal_module._build_validator_only_head_promotion(
            freeze=freeze, pilot_runtime=pilot_runtime
        )


@pytest.mark.parametrize(
    "mutation",
    [
        "schema_call", "schema_multi_target", "schema_attribute_target",
        "schema_subscript_target", "schema_tuple_target", "schema_renamed_target",
        "schema_annassign_target", "schema_type_comment",
    ],
)
def test_validator_only_ast_guard_rejects_schema_assignment_shell_changes(
    tmp_path, monkeypatch, mutation
):
    repository, _from, _to, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path, mutation=mutation)
    )
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: _promotion_loaded_identity(repository, head),
    )

    with pytest.raises(V6ContractError, match="schema assignment|protected top-level AST"):
        v6_formal_module._build_validator_only_head_promotion(
            freeze=freeze, pilot_runtime=pilot_runtime
        )


@pytest.mark.parametrize("tamper", ["hash", "record", "loaded"])
def test_validator_only_head_promotion_rejects_tampered_record(
    tmp_path, monkeypatch, tamper
):
    repository, _from, _to, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path)
    )
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: _promotion_loaded_identity(repository, head),
    )
    monkeypatch.setattr(
        v6_formal_module, "_repository_tracked_dirty", REAL_TRACKED_GUARD
    )
    promotion = v6_formal_module._build_validator_only_head_promotion(
        freeze=freeze, pilot_runtime=pilot_runtime
    )
    if tamper == "hash":
        promotion["module3_diff"]["canonical_diff_sha256"] = "0" * 64
    elif tamper == "record":
        promotion["module3_diff"]["name_status"][0]["path"] = "scripts/run_experiment.sh"
    else:
        promotion["loaded_validator"]["source_sha256"] = "0" * 64

    with pytest.raises(V6ContractError, match="diff record|loaded validator"):
        v6_formal_module._validate_validator_only_head_promotion(
            promotion, freeze=freeze
        )


def test_validator_only_head_promotion_rejects_current_checkout_drift(
    tmp_path, monkeypatch
):
    repository, _from, _to, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path)
    )
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: _promotion_loaded_identity(repository, head),
    )
    monkeypatch.setattr(
        v6_formal_module, "_repository_tracked_dirty", REAL_TRACKED_GUARD
    )
    promotion = v6_formal_module._build_validator_only_head_promotion(
        freeze=freeze, pilot_runtime=pilot_runtime
    )
    protected = repository / (
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
    )
    protected.write_text(protected.read_text() + "# drift\n", encoding="utf-8")

    with pytest.raises(V6ContractError, match="current checkout drift"):
        v6_formal_module._validate_validator_only_head_promotion(
            promotion, freeze=freeze
        )


def test_validator_only_head_promotion_rejects_nonancestor_from_head(
    tmp_path, monkeypatch
):
    repository, _from, to_head, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path)
    )
    subprocess.run(
        ["git", "-C", str(repository), "checkout", "--orphan", "unrelated"],
        check=True, capture_output=True,
    )
    subprocess.run(
        ["git", "-C", str(repository), "commit", "--allow-empty", "-qm", "unrelated"],
        check=True,
        env={**os.environ, "GIT_AUTHOR_NAME": "Codex Test", "GIT_AUTHOR_EMAIL": "codex@example.invalid", "GIT_COMMITTER_NAME": "Codex Test", "GIT_COMMITTER_EMAIL": "codex@example.invalid"},
    )
    unrelated = subprocess.run(
        ["git", "-C", str(repository), "rev-parse", "HEAD"],
        check=True, capture_output=True, text=True,
    ).stdout.strip()
    subprocess.run(
        ["git", "-C", str(repository), "checkout", "-q", to_head], check=True
    )
    pilot_runtime["repositories"]["module3"]["head"] = unrelated
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: _promotion_loaded_identity(repository, head),
    )

    with pytest.raises(V6ContractError, match="not an ancestor"):
        v6_formal_module._build_validator_only_head_promotion(
            freeze=freeze, pilot_runtime=pilot_runtime
        )


def test_validator_only_head_promotion_rejects_loaded_module_bytes_drift(
    tmp_path, monkeypatch
):
    repository, _from, _to, freeze, pilot_runtime = (
        _validator_promotion_repository(tmp_path)
    )
    state = {"sha": "1" * 64}
    monkeypatch.setattr(
        v6_formal_module, "_validator_only_loaded_identity",
        lambda _root, head: {
            **_promotion_loaded_identity(repository, head),
            "source_sha256": state["sha"],
        },
    )
    promotion = v6_formal_module._build_validator_only_head_promotion(
        freeze=freeze, pilot_runtime=pilot_runtime
    )
    state["sha"] = "2" * 64

    with pytest.raises(V6ContractError, match="loaded validator identity"):
        v6_formal_module._validate_validator_only_head_promotion(
            promotion, freeze=freeze
        )


def test_indoor_pilot_runtime_derivation_rejects_mixed_stack_snapshots(tmp_path):
    pilot_root = tmp_path / "pilot"
    template_root = tmp_path / "template"
    template_root.mkdir()
    template_path = _live_stack_contract(template_root)
    template = json.loads(template_path.read_text())
    for condition_id in v6_formal_module.INDOOR_CONDITION_IDS:
        for rep in range(1, 4):
            destination = (
                pilot_root / condition_id / f"rep{rep}" / "scenario"
                / f"run-{rep:04d}" / "stack_contract.json"
            )
            destination.parent.mkdir(parents=True)
            payload = dict(template)
            if condition_id == "indoor_dynamic" and rep == 2:
                payload["module3_head"] = "0" * 40
            payload["stack_session_id"] = v6_formal_module._stack_session_id(payload)
            destination.write_text(json.dumps(payload), encoding="utf-8")
    repositories = {
        name: {"path": str(REPO), "head": template[f"{name}_head"]}
        for name in ("integration", "module2", "module3")
    }

    with pytest.raises(V6ContractError, match="mixed runtime tuple"):
        v6_formal_module._derive_indoor_pilot_runtime(
            pilot_root, repositories=repositories
        )


def test_indoor_pilot_aggregate_publishes_only_nine_indoor_runs(
    tmp_path, monkeypatch
):
    pilot_root = _write_indoor_production_pilot_root(tmp_path, monkeypatch)
    manifest_output = tmp_path / "nas" / "indoor-pilot-manifest.json"
    aggregate_output = tmp_path / "nas" / "indoor-pilot-aggregate.json"

    result = v6_formal_module.aggregate_indoor_pilot(
        pilot_root=pilot_root,
        output_manifest=manifest_output,
        output_aggregate=aggregate_output,
    )

    assert result["qualification"] == "INDOOR_PILOT_READY"
    assert result["formal_qualification"] == NOT_QUALIFIED
    assert result["strict_successes"] == 9
    aggregate = json.loads(aggregate_output.read_text())
    pilot_manifest = json.loads(manifest_output.read_text())
    assert aggregate["schema_version"] == v6_formal_module.INDOOR_PILOT_AGGREGATE_SCHEMA
    assert "validator_only_head_promotion" not in aggregate
    assert "validator_only_head_promotion" not in pilot_manifest["freeze"]
    assert result["validator_only_head_promotion"] is None
    assert [row["id"] for row in aggregate["conditions"]] == list(
        v6_formal_module.INDOOR_CONDITION_IDS
    )
    assert sum(len(row["episodes"]) for row in aggregate["conditions"]) == 9
    assert all(
        row["runner_arguments"] == _expected_indoor_stable_arguments()
        for row in pilot_manifest["conditions"]
    )
    experiment_launch = (
        REPO / "ros2_ws/src/robot_experiments/launch/experiment.launch.py"
    ).resolve()
    assert hashlib.sha256(experiment_launch.read_bytes()).hexdigest() == (
        "e11de8da7b00e75d1ad99ec09adf74aeccd8471b3e28c5a653a10c01d0a1bef5"
    )
    for rows in pilot_manifest["freeze"]["scenario_configs"].values():
        launch_rows = [row for row in rows if row["path"] == str(experiment_launch)]
        assert launch_rows == [{
            "path": str(experiment_launch),
            "sha256": "e11de8da7b00e75d1ad99ec09adf74aeccd8471b3e28c5a653a10c01d0a1bef5",
        }]


def test_same_exact_tuple_skips_validator_promotion(monkeypatch):
    freeze = {
        "repositories": {"module3": {"path": "/m3", "head": "a", "tree": "t"}},
        "driver_version": "driver",
        "kernel_release": "kernel",
    }
    monkeypatch.setattr(
        v6_formal_module,
        "_build_validator_only_head_promotion",
        lambda **_kwargs: pytest.fail("same tuple must not enter promotion"),
    )
    assert v6_formal_module._validator_promotion_if_needed(
        freeze=freeze,
        pilot_runtime=json.loads(json.dumps(freeze)),
    ) is None


def test_outdoor_pilot_aggregate_and_freeze_publish_nine_b_attested_runs(
    tmp_path, monkeypatch
):
    pilot_root = _write_outdoor_production_pilot_root(tmp_path, monkeypatch)
    manifest_output = tmp_path / "nas" / "outdoor-pilot-manifest.json"
    aggregate_output = tmp_path / "nas" / "outdoor-pilot-aggregate.json"
    result = v6_formal_module.aggregate_outdoor_pilot(
        pilot_root=pilot_root,
        output_manifest=manifest_output,
        output_aggregate=aggregate_output,
    )
    assert result["qualification"] == "OUTDOOR_PILOT_READY"
    assert result["strict_successes"] == 9
    aggregate = json.loads(aggregate_output.read_text())
    assert [row["id"] for row in aggregate["conditions"]] == list(
        v6_formal_module.OUTDOOR_CONDITION_IDS
    )
    campaign_path = tmp_path / "nas" / "outdoor-campaign.json"
    campaign = v6_formal_module.freeze_outdoor_campaign_from_pilot(
        pilot_manifest_path=manifest_output,
        pilot_aggregate_path=aggregate_output,
        output_manifest_path=campaign_path,
        outdoor_output_root=tmp_path / "nas" / "outdoor-runs",
    )
    assert len(campaign.pilot_freeze_provenance["episodes"]) == 9
    assert campaign.freeze["frozen_assets"][
        "rivermark_viewport_startup_winner_manifest"
    ]["sha256"]


def test_outdoor_thresholds_are_static_19_dynamic_appearance_18():
    assert v6_formal_module.OUTDOOR_SUCCESS_THRESHOLDS == {
        "outdoor_static": 19,
        "outdoor_dynamic": 18,
        "outdoor_appearance": 18,
    }


def test_outdoor_and_combine_wrapper_cli_contracts_are_exposed():
    wrapper = (REPO / "scripts/run_v6_formal_episode.sh").read_text()
    for token in (
        "--aggregate-outdoor-pilot", "--freeze-outdoor-pilot",
        "--execute-outdoor", "--combine-qualified-halves",
    ):
        assert token in wrapper
    parser = v6_formal_module.build_parser()
    parsed = parser.parse_args(["--combine-qualified-halves", "/indoor", "/outdoor"])
    assert parsed.combine_qualified_halves == ["/indoor", "/outdoor"]


def test_different_head_keeps_existing_validator_promotion(monkeypatch):
    freeze = {
        "repositories": {"module3": {"path": "/m3", "head": "b"}},
        "driver_version": "driver",
        "kernel_release": "kernel",
    }
    expected = {"schema": "bio_nav.v6_validator_only_head_promotion.v1"}
    monkeypatch.setattr(
        v6_formal_module,
        "_build_validator_only_head_promotion",
        lambda **_kwargs: expected,
    )
    assert v6_formal_module._validator_promotion_if_needed(
        freeze=freeze,
        pilot_runtime={
            **freeze,
            "repositories": {"module3": {"path": "/m3", "head": "a"}},
        },
    ) == expected


def test_viewport_winner_manifest_requires_selected_b(tmp_path):
    path = tmp_path / "winner.json"
    path.write_text(json.dumps({
        "winner": {
            "viewport_arm": "B",
        }
    }))
    parsed = v6_formal_module._viewport_winner_manifest(path)
    assert parsed["viewport_arm"] == "B"
    assert parsed["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    path.write_text(json.dumps({
        "winner": {
            "viewport_arm": "A",
        }
    }))
    with pytest.raises(V6ContractError, match="select arm B"):
        v6_formal_module._viewport_winner_manifest(path)


@pytest.mark.parametrize("mutation", ["missing", "mismatch"])
def test_outdoor_stack_rejects_missing_or_mismatched_viewport_attestation(
    tmp_path, mutation, monkeypatch
):
    pilot_root = _write_outdoor_production_pilot_root(tmp_path, monkeypatch)
    _pilot, conditions, _digest, freeze = (
        v6_formal_module._build_outdoor_pilot_manifest(pilot_root)
    )
    condition = conditions[0]
    identity = condition.episode_identities[0]
    run_root = (
        pilot_root / condition.condition_id / "rep1" / condition.scenario_id
        / f"run-0001-seed-{identity['seed']}"
    )
    path = run_root / "stack_contract.json"
    runtime = run_root / "viewport_runtime_attestation.json"
    if mutation == "missing":
        runtime.unlink()
    else:
        payload = json.loads(runtime.read_text())
        payload["readbacks"][1]["observed_enabled"] = True
        runtime.write_text(json.dumps(payload))
        runtime.chmod(0o600)
    with pytest.raises(V6ContractError, match="viewport runtime"):
        v6_formal_module._load_stack_contract_snapshot(
            path, expected_condition_id="outdoor_static", freeze=freeze
        )


def test_combine_rejects_mixed_repository_tuple_before_evidence(monkeypatch):
    common = {
        "repositories": {
            "integration": {"path": "/i", "head": "a", "tree": "t"},
            "module2": {"path": "/m2", "head": "b", "tree": "u"},
            "module3": {"path": "/m3", "head": "c", "tree": "v"},
        },
        "driver_version": "driver", "kernel_release": "kernel",
        "frozen_assets": {}, "runner_entrypoint": {},
        "experiment_runner": {}, "v6_formal": {},
    }
    indoor = SimpleNamespace(freeze=json.loads(json.dumps(common)))
    outdoor = SimpleNamespace(freeze=json.loads(json.dumps(common)))
    outdoor.freeze["repositories"]["module3"]["head"] = "different"
    monkeypatch.setattr(v6_formal_module, "load_indoor_campaign_manifest", lambda _p: indoor)
    monkeypatch.setattr(v6_formal_module, "load_outdoor_campaign_manifest", lambda _p: outdoor)
    monkeypatch.setattr(v6_formal_module, "evaluate_indoor_campaign", lambda _m: {
        "qualification": "INDOOR_QUALIFICATION_PASS", "present_episodes": 60,
        "valid_episodes": 60,
    })
    monkeypatch.setattr(v6_formal_module, "evaluate_outdoor_campaign", lambda _m: {
        "qualification": "OUTDOOR_QUALIFICATION_PASS", "present_episodes": 60,
        "valid_episodes": 60,
    })
    with pytest.raises(V6ContractError, match=r"HEAD\+tree tuple mismatch"):
        v6_formal_module.combine_qualified_halves("indoor", "outdoor")


def test_half_evidence_digest_rereads_exactly_sixty_runs(tmp_path, monkeypatch):
    conditions = []
    for condition_index, condition_id in enumerate(v6_formal_module.INDOOR_CONDITION_IDS):
        scenario_id = f"scenario-{condition_index}"
        output = tmp_path / condition_id
        identities = tuple(
            {"seed": 1000 + condition_index * 20 + index} for index in range(20)
        )
        for run_index, identity in enumerate(identities, start=1):
            root = output / scenario_id / f"run-{run_index:04d}-seed-{identity['seed']}"
            root.mkdir(parents=True)
            stack_path = root / "stack_contract.json"
            stack_path.write_text(f"{condition_id}:{run_index}:stack")
            stack_digest = hashlib.sha256(stack_path.read_bytes()).hexdigest()
            session = hashlib.sha256(condition_id.encode()).hexdigest()
            contract_path = str((tmp_path / "runtime" / condition_id / "stack.contract.json").resolve())
            receipt = {
                "sequence": run_index,
                "startup_kind": "cold" if run_index == 1 else "hot_reset",
                "stack_session_id": session,
                "condition_stack_contract_path": contract_path,
                "condition_stack_contract_sha256": stack_digest,
            }
            (root / "run_manifest.json").write_text(json.dumps({
                "stack_episode_receipt": receipt,
                "stack_session_id": session,
                "reset_receipt": {"generation": run_index + 1},
            }))
            (root / "run_summary.json").write_text(json.dumps({
                "stack_session_id": session,
                "reset_receipt": {"generation": run_index + 1},
                "condition_stack_attestation": {
                    "stack_episode_receipt": receipt,
                    "condition_stack_contract_path": contract_path,
                    "condition_stack_contract_sha256": stack_digest,
                },
            }))
            (root / "checksums.sha256").write_text("fixture")
        conditions.append(v6_formal_module.FormalCondition(
            condition_id, "indoor", condition_id.split("_", 1)[1],
            tmp_path / "scenario.yaml", output, (), scenario_id, identities,
        ))
    calls = []
    monkeypatch.setattr(
        v6_formal_module, "_load_stack_contract_snapshot",
        lambda path, expected_condition_id, **_kwargs: (
            calls.append(path) or {
                "schema": v6_formal_module.STACK_CONTRACT_SCHEMA,
                "stack_session_id": hashlib.sha256(expected_condition_id.encode()).hexdigest(),
                "contract_path": str((tmp_path / "runtime" / expected_condition_id / "stack.contract.json").resolve()),
            },
            hashlib.sha256(str(path).encode()).hexdigest(),
        ),
    )
    digest = v6_formal_module._half_evidence_digest(
        SimpleNamespace(conditions=tuple(conditions), freeze={})
    )
    assert len(digest) == 64
    assert len(calls) == 60
    calls[-1].unlink()
    with pytest.raises(V6ContractError, match="core file is missing"):
        v6_formal_module._half_evidence_digest(
            SimpleNamespace(conditions=tuple(conditions), freeze={})
        )


def _strict_half_campaign(campaign):
    freeze = json.loads(json.dumps(campaign.freeze))
    for entry in freeze["repositories"].values():
        entry["tree"] = subprocess.run(
            ["git", "-C", entry["path"], "rev-parse", f"{entry['head']}^{{tree}}"],
            check=True, capture_output=True, text=True,
        ).stdout.strip()
    return replace(campaign, freeze=freeze)


def _write_strict_half_episode(campaign, condition, run_index, *, mutation=None):
    stack = json.loads(
        _live_stack_contract(
            condition.output_directory.parent,
            condition_id=condition.condition_id,
        ).read_text()
    )
    stack["schema"] = v6_formal_module.STACK_CONTRACT_SCHEMA
    stack["contract_path"] = str(
        (condition.output_directory.parent / condition.condition_id / "stack.contract.json").resolve()
    )
    for name, entry in campaign.freeze["repositories"].items():
        stack[f"{name}_head"] = entry["head"]
        stack[f"{name}_tree"] = entry["tree"]
    stack.update({
        "viewport_arm": "not_applicable",
        "disable_viewport_updates_requested": "not_applicable",
        "disable_viewport_updates_observed": "not_applicable",
        "viewport_startup_attestation_sha256": "not_applicable",
        "viewport_runtime_attestation_path": "not_applicable",
        "viewport_runtime_attestation_sha256": "not_applicable",
        "viewport_instance_uuid": "not_applicable",
        "viewport_pid": "not_applicable",
        "viewport_pgid": "not_applicable",
        "viewport_start_ticks": "not_applicable",
        "viewport_winner_manifest_sha256": "not_applicable",
    })
    if mutation == "mixed_stack":
        stack["domain"] += run_index
    stack["stack_session_id"] = v6_formal_module._stack_session_id(stack)
    root = _write_formal_run(
        condition,
        run_index,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
        stack_session_id=stack["stack_session_id"],
        reset_generation=run_index + 1,
    )
    stack_path = root / "stack_contract.json"
    stack_path.write_text(json.dumps(stack))
    stack_digest = hashlib.sha256(stack_path.read_bytes()).hexdigest()
    manifest_path = root / "run_manifest.json"
    summary_path = root / "run_summary.json"
    manifest = json.loads(manifest_path.read_text())
    summary = json.loads(summary_path.read_text())
    receipt = {
        "schema": "bio_nav.v6_stack_episode_receipt.v1",
        "baseline": 1,
        "sequence_path": stack["episode_sequence_path"],
        "t2_selector_path": stack["t2_selector_path"],
        "t2_selector_sha256": stack["t2_selector_sha256"],
        "sequence": 4 if mutation == "sequence4" and run_index == 1 else run_index,
        "startup_kind": (
            "cold" if run_index == 1 else "hot_reset"
        ),
        "stack_session_id": stack["stack_session_id"],
        "condition_stack_contract_path": stack["contract_path"],
        "condition_stack_contract_sha256": (
            "0" * 64 if mutation == "contract_digest" else stack_digest
        ),
        "viewport_arm": "not_applicable",
        "disable_viewport_updates_requested": "not_applicable",
        "disable_viewport_updates_observed": "not_applicable",
        "viewport_startup_attestation_sha256": "not_applicable",
        "viewport_runtime_attestation_sha256": "not_applicable",
        "viewport_instance_uuid": "not_applicable",
        "viewport_pid": "not_applicable",
        "viewport_pgid": "not_applicable",
        "viewport_start_ticks": "not_applicable",
        "viewport_winner_manifest_sha256": "not_applicable",
    }
    manifest["stack_episode_receipt"] = receipt
    if mutation == "startup_kind" and run_index == 2:
        receipt["startup_kind"] = "cold"
    manifest["stack_session_id"] = stack["stack_session_id"]
    summary["stack_session_id"] = stack["stack_session_id"]
    if mutation == "session" and run_index == 2:
        summary["stack_session_id"] = "f" * 64
    summary.setdefault("condition_stack_attestation", {}).update({
        "stack_session_id": summary["stack_session_id"],
        "stack_episode_receipt": receipt,
        "condition_stack_contract_path": stack["contract_path"],
        "condition_stack_contract_sha256": receipt[
            "condition_stack_contract_sha256"
        ],
    })
    if "stack_contract.json" not in summary["evidence"]["required_files"]:
        summary["evidence"]["required_files"].append("stack_contract.json")
    manifest_path.write_text(json.dumps(manifest))
    summary_path.write_text(json.dumps(summary))
    _refresh_checksums(root)
    return root


@pytest.mark.parametrize(
    "mutation", ["sequence4", "session", "contract_digest", "mixed_stack", "startup_kind"]
)
def test_half60_group_contract_rejects_sequence_session_digest_stack_or_kind(
    tmp_path, monkeypatch, mutation
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "strict-half.json",
        indoor_output_root=tmp_path / "nas" / "strict-half-runs",
    )
    campaign = _strict_half_campaign(campaign)
    condition = campaign.conditions[0]
    _write_strict_half_episode(campaign, condition, 1, mutation=mutation)
    _write_strict_half_episode(campaign, condition, 2, mutation=mutation)
    result = evaluate_indoor_campaign(campaign)
    row = result["conditions"][0]
    assert row["qualification"] == "INCOMPLETE"
    assert row["blockers"]


def test_half60_group_contract_accepts_exact_cold_then_hot_prefix(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = _strict_half_campaign(freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "strict-prefix.json",
        indoor_output_root=tmp_path / "nas" / "strict-prefix-runs",
    ))
    condition = campaign.conditions[0]
    _write_strict_half_episode(campaign, condition, 1)
    _write_strict_half_episode(campaign, condition, 2)
    row = evaluate_indoor_campaign(campaign)["conditions"][0]
    assert row["blockers"] == []
    assert row["valid_episodes"] == 2
    assert row["next_run_index"] == 3


def test_indoor_pilot_aggregate_rejects_non_indoor_root(tmp_path, monkeypatch):
    pilot_root = _write_indoor_production_pilot_root(tmp_path, monkeypatch)
    (pilot_root / "outdoor_static").mkdir()

    with pytest.raises(V6ContractError, match="exactly the three indoor"):
        v6_formal_module.aggregate_indoor_pilot(
            pilot_root=pilot_root,
            output_manifest=tmp_path / "nas" / "manifest.json",
            output_aggregate=tmp_path / "nas" / "aggregate.json",
        )


def test_indoor_pilot_aggregate_rejects_unknown_top_level_directory(
    tmp_path, monkeypatch
):
    pilot_root = _write_indoor_production_pilot_root(tmp_path, monkeypatch)
    (pilot_root / "stale_old_campaign").mkdir()

    with pytest.raises(V6ContractError, match="exactly the three indoor"):
        v6_formal_module.aggregate_indoor_pilot(
            pilot_root=pilot_root,
            output_manifest=tmp_path / "nas" / "manifest.json",
            output_aggregate=tmp_path / "nas" / "aggregate.json",
        )


def test_indoor_pilot_aggregate_rejects_extra_rep(tmp_path, monkeypatch):
    pilot_root = _write_indoor_production_pilot_root(tmp_path, monkeypatch)
    (pilot_root / "indoor_static" / "rep4").mkdir()

    with pytest.raises(V6ContractError, match="exactly rep1-rep3"):
        v6_formal_module.aggregate_indoor_pilot(
            pilot_root=pilot_root,
            output_manifest=tmp_path / "nas" / "manifest.json",
            output_aggregate=tmp_path / "nas" / "aggregate.json",
        )


def test_indoor_pilot_aggregate_rejects_unknown_rep_sibling(
    tmp_path, monkeypatch
):
    pilot_root = _write_indoor_production_pilot_root(tmp_path, monkeypatch)
    (pilot_root / "indoor_static" / "rep1" / "stale_old_campaign").mkdir()

    with pytest.raises(V6ContractError, match="rep topology mismatch"):
        v6_formal_module.aggregate_indoor_pilot(
            pilot_root=pilot_root,
            output_manifest=tmp_path / "nas" / "manifest.json",
            output_aggregate=tmp_path / "nas" / "aggregate.json",
        )


@pytest.mark.parametrize(
    "mutation",
    [
        "unknown_file",
        "single_file",
        "different_stem",
        "multiple_pairs",
        "symlink",
        "socket",
        "invalid_stem",
        "json_projection",
        "json_noncanonical",
        "csv_tamper",
    ],
)
def test_indoor_pilot_aggregate_rejects_invalid_official_sidecars(
    tmp_path, monkeypatch, mutation
):
    pilot_root = _write_indoor_production_pilot_root(tmp_path, monkeypatch)
    rep_root = pilot_root / "indoor_static" / "rep1"
    json_path = next(rep_root.glob("*.json"))
    csv_path = next(rep_root.glob("*.csv"))
    owned_socket = None
    owned_socket_directory = None
    if mutation == "unknown_file":
        (rep_root / "unknown.txt").write_text("stale\n", encoding="utf-8")
    elif mutation == "single_file":
        csv_path.unlink()
    elif mutation == "different_stem":
        csv_path.rename(rep_root / f"{csv_path.stem}-different.csv")
    elif mutation == "multiple_pairs":
        shutil.copy2(json_path, rep_root / f"{json_path.stem}-copy.json")
        shutil.copy2(csv_path, rep_root / f"{csv_path.stem}-copy.csv")
    elif mutation == "symlink":
        target = tmp_path / "sidecar.json"
        target.write_bytes(json_path.read_bytes())
        json_path.unlink()
        json_path.symlink_to(target)
    elif mutation == "socket":
        csv_path.unlink()
        owned_socket_directory = Path(
            tempfile.mkdtemp(prefix="bnsock-", dir="/tmp")
        )
        short_socket_path = owned_socket_directory / "s"
        owned_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        owned_socket.bind(str(short_socket_path))
        os.link(short_socket_path, csv_path)
    elif mutation == "invalid_stem":
        prefix = "v6_final_kujiale_static-run-0001-seed-8601-invalid"
        json_path.rename(rep_root / f"{prefix}.json")
        csv_path.rename(rep_root / f"{prefix}.csv")
    elif mutation == "json_projection":
        payload = json.loads(json_path.read_text())
        payload["random_seed"] += 1
        json_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    elif mutation == "json_noncanonical":
        json_path.write_text(json_path.read_text().rstrip() + "  \n", encoding="utf-8")
    else:
        csv_path.write_text(csv_path.read_text() + "tamper\n", encoding="utf-8")
    try:
        with pytest.raises(V6ContractError):
            v6_formal_module.aggregate_indoor_pilot(
                pilot_root=pilot_root,
                output_manifest=tmp_path / "nas" / "manifest.json",
                output_aggregate=tmp_path / "nas" / "aggregate.json",
            )
    finally:
        if owned_socket is not None:
            owned_socket.close()
        if owned_socket_directory is not None:
            (owned_socket_directory / "s").unlink(missing_ok=True)
            owned_socket_directory.rmdir()


def test_indoor_pilot_aggregate_rejects_extra_run_directory(
    tmp_path, monkeypatch
):
    pilot_root = _write_indoor_production_pilot_root(tmp_path, monkeypatch)
    scenario_root = (
        pilot_root / "indoor_static" / "rep1" / "v6_final_kujiale_static"
    )
    (scenario_root / "run-9999-seed-9999").mkdir()

    with pytest.raises(V6ContractError, match="rep topology mismatch"):
        v6_formal_module.aggregate_indoor_pilot(
            pilot_root=pilot_root,
            output_manifest=tmp_path / "nas" / "manifest.json",
            output_aggregate=tmp_path / "nas" / "aggregate.json",
        )


def test_indoor_freezer_and_dry_run_are_scope_separated(tmp_path, monkeypatch, capsys):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    output = tmp_path / "indoor-campaign.json"
    output_root = tmp_path / "nas" / "indoor-60"

    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=output,
        indoor_output_root=output_root,
    )

    assert len(campaign.pilot_freeze_provenance["episodes"]) == 9
    assert [row.condition_id for row in campaign.conditions] == list(
        v6_formal_module.INDOOR_CONDITION_IDS
    )
    assert all(len(row.episode_identities) == 20 for row in campaign.conditions)
    frozen_raw = json.loads(output.read_text())
    assert frozen_raw["freeze"]["validator_only_head_promotion"] == json.loads(
        pilot_manifest.read_text()
    )["freeze"]["validator_only_head_promotion"]
    assert campaign.freeze["validator_only_head_promotion"] == frozen_raw["freeze"][
        "validator_only_head_promotion"
    ]
    physical = frozen_raw["freeze"]["physical_contracts"]
    assert physical["indoor_static"]["static_obstacle_ids"] == ["v6_low_box_solo"]
    assert physical["indoor_static"]["static_obstacle_count"] == 1
    assert physical["indoor_dynamic"]["dynamic_actor_ids"] == [
        "v6_dynamic_g2_crossing_box"
    ]
    assert physical["indoor_dynamic"]["dynamic_actor_count"] == 1
    assert all(
        set(row) >= {"physical_config", "scenario", "spawn_manifest"}
        for row in physical.values()
    )
    assert {
        row["spawn_manifest"]["sha256"] for row in physical.values()
    } == {"df06635cf706f407f5d58e8ebcf7788b0bea3fe1914589e768ade754a0098d70"}
    with pytest.raises(V6ContractError, match="formal_manifest keys|formal schema_version"):
        load_formal_campaign_manifest(output)
    full_root = tmp_path / "full"
    full_root.mkdir()
    full = _write_formal_manifest(full_root)
    with pytest.raises(V6ContractError, match="indoor_manifest keys|indoor schema_version"):
        load_indoor_campaign_manifest(full)
    assert cli(["--indoor-manifest", str(output)]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["aggregate"]["expected_episodes"] == 60
    assert payload["aggregate"]["present_episodes"] == 0
    assert payload["aggregate"]["formal_qualification"] == NOT_QUALIFIED
    assert payload["qualification"] == "INCOMPLETE"
    assert payload["dispatch"] is False
    assert len(payload["dispatch_plan"]) == 1
    assert payload["dispatch_plan"][0]["condition_id"] == "indoor_static"


def test_indoor_continuation_creator_preserves_external_static_run1(
    tmp_path, monkeypatch
):
    parent_path, parent, run1, successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )

    raw = json.loads(successor_path.read_text())
    continuation = successor.freeze["indoor_continuation"]
    assert raw["schema_version"] == v6_formal_module.INDOOR_CONTINUATION_SCHEMA_VERSION
    assert continuation["reason"] == "tooling_invalid_after_valid_episode"
    assert continuation["parent_manifest"]["path"] == str(parent_path)
    assert continuation["parent_manifest"]["freeze_digest"] == parent.freeze_digest
    assert continuation["preserved_run"]["root"] == str(run1)
    assert continuation["preserved_run"]["classification"] == "valid_product_failure"
    assert continuation["preserved_run"]["collision_detected"] is True
    assert continuation["preserved_run"]["path_deviation_percent"] is None
    assert successor.conditions[0].output_directory != parent.conditions[0].output_directory
    assert not (
        successor.conditions[0].output_directory
        / successor.conditions[0].scenario_id
        / "run-0001-seed-8601"
    ).exists()

    result = evaluate_indoor_campaign(successor)
    static = result["conditions"][0]
    assert static["runs"][0]["status"] == "product_failure"
    assert static["valid_episodes"] == 1
    assert static["strict_successes"] == 0
    assert static["next_run_index"] == 2
    plan = v6_formal_module.indoor_dispatch_plan(successor, result)
    assert plan[0]["condition_id"] == "indoor_static"
    assert plan[0]["run_index"] == 2
    assert plan[0]["stack_boundary"] == "continuation_cold_restart"


def test_indoor_continuation_accepts_preserved_failure_without_recorded_route_terminal(
    tmp_path, monkeypatch
):
    _parent_path, _parent, _run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    original_inventory = experiment_runner_module._mcap_inventory_evidence

    def failure_inventory(root):
        evidence = original_inventory(root)
        evidence["semantic"]["route_complete_true_count"] = 0
        evidence["semantic"]["terminal_zero_count"] = 0
        evidence["semantic"]["terminal_nonzero_count"] = 0
        return evidence

    monkeypatch.setattr(
        experiment_runner_module, "_mcap_inventory_evidence", failure_inventory
    )

    result = evaluate_indoor_campaign(successor)

    assert result["conditions"][0]["runs"][0]["status"] == "product_failure"
    assert result["conditions"][0]["next_run_index"] == 2


@pytest.mark.parametrize(
    "tamper",
    [
        "parent_manifest", "run_summary", "collision_record", "classification",
        "sequence", "session", "root", "dynamic_condition",
    ],
)
def test_indoor_continuation_revalidates_parent_and_preserved_run1(
    tmp_path, monkeypatch, tamper
):
    parent_path, _parent, run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    record = successor.freeze["indoor_continuation"]
    if tamper == "parent_manifest":
        parent_path.write_text(parent_path.read_text() + "\n", encoding="utf-8")
    elif tamper == "run_summary":
        summary = run1 / "run_summary.json"
        summary.write_text(summary.read_text() + "\n", encoding="utf-8")
    elif tamper == "collision_record":
        record["preserved_run"]["collision_detected"] = False
    elif tamper == "classification":
        record["preserved_run"]["classification"] = "strict_success"
    elif tamper == "sequence":
        record["preserved_run"]["sequence"] = 2
    elif tamper == "session":
        record["preserved_run"]["stack_session_id"] = "0" * 64
    elif tamper == "root":
        record["preserved_run"]["root"] = str(tmp_path / "wrong")
    else:
        record["condition_id"] = "indoor_dynamic"

    with pytest.raises(V6ContractError):
        evaluate_indoor_campaign(successor)


def test_indoor_continuation_rejects_run1_copy_or_retry(tmp_path, monkeypatch):
    _parent_path, _parent, _run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    retry_root = (
        successor.conditions[0].output_directory
        / successor.conditions[0].scenario_id
        / "run-0001-seed-8601"
    )
    retry_root.mkdir(parents=True)

    with pytest.raises(V6ContractError, match="copy or retry preserved run1"):
        evaluate_indoor_campaign(successor)


def test_indoor_continuation_creator_is_no_clobber(tmp_path, monkeypatch):
    parent_path, _parent, _run1, successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    original = successor_path.read_bytes()

    with pytest.raises(V6ContractError, match="new non-symlink outputs"):
        v6_formal_module.create_indoor_continuation_campaign(
            parent_manifest_path=parent_path,
            output_manifest_path=successor_path,
            successor_output_root=successor.conditions[0].output_directory.parent,
        )

    assert successor_path.read_bytes() == original


def test_indoor_continuation_cannot_chain_a_second_continuation(
    tmp_path, monkeypatch
):
    _parent_path, _parent, _run1, successor_path, _successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )

    with pytest.raises(V6ContractError, match="approved campaign|parent schema"):
        v6_formal_module.create_indoor_continuation_campaign(
            parent_manifest_path=successor_path,
            output_manifest_path=tmp_path / "nas" / "third-campaign.json",
            successor_output_root=tmp_path / "nas" / "third-output",
        )


def test_indoor_continuation_creator_rejects_symlink_outputs(tmp_path, monkeypatch):
    parent_path, _parent, _run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    output_link = tmp_path / "nas" / "linked-successor.json"
    output_link.symlink_to(parent_path)
    with pytest.raises(V6ContractError, match="symlink|new non-symlink outputs"):
        v6_formal_module.create_indoor_continuation_campaign(
            parent_manifest_path=parent_path,
            output_manifest_path=output_link,
            successor_output_root=tmp_path / "nas" / "fresh-root",
        )
    root_link = tmp_path / "nas" / "linked-root"
    root_link.symlink_to(successor.freeze["indoor_continuation"]["parent_output_root"])
    with pytest.raises(V6ContractError, match="symlink|new non-symlink outputs"):
        v6_formal_module.create_indoor_continuation_campaign(
            parent_manifest_path=parent_path,
            output_manifest_path=tmp_path / "nas" / "fresh-successor.json",
            successor_output_root=root_link,
        )


def test_indoor_continuation_creator_rejects_dangling_manifest_and_root_links(
    tmp_path, monkeypatch
):
    parent_path, _parent, _run1, _successor_path, _successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    dangling_manifest_target = tmp_path / "nas" / "missing-manifest-target.json"
    dangling_manifest = tmp_path / "nas" / "dangling-manifest.json"
    dangling_manifest.symlink_to(dangling_manifest_target)
    untouched_root = tmp_path / "nas" / "untouched-root"
    with pytest.raises(V6ContractError, match="symlink"):
        v6_formal_module.create_indoor_continuation_campaign(
            parent_manifest_path=parent_path,
            output_manifest_path=dangling_manifest,
            successor_output_root=untouched_root,
        )
    assert not dangling_manifest_target.exists()
    assert not untouched_root.exists()

    dangling_root_target = tmp_path / "nas" / "missing-root-target"
    dangling_root = tmp_path / "nas" / "dangling-root"
    dangling_root.symlink_to(dangling_root_target)
    untouched_manifest = tmp_path / "nas" / "untouched-manifest.json"
    with pytest.raises(V6ContractError, match="symlink"):
        v6_formal_module.create_indoor_continuation_campaign(
            parent_manifest_path=parent_path,
            output_manifest_path=untouched_manifest,
            successor_output_root=dangling_root,
        )
    assert not dangling_root_target.exists()
    assert not untouched_manifest.exists()


def test_indoor_continuation_rejects_preserved_run_directory_symlink(
    tmp_path, monkeypatch
):
    _parent_path, _parent, run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    moved = run1.with_name("run-0001-seed-8601-moved-copy")
    run1.rename(moved)
    run1.symlink_to(moved, target_is_directory=True)

    with pytest.raises(V6ContractError, match="symlink"):
        evaluate_indoor_campaign(successor)


def test_indoor_continuation_creator_rejects_dotdot_escape(tmp_path, monkeypatch):
    parent_path, _parent, _run1, _successor_path, _successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    escaped_manifest = tmp_path / "nas" / "subdir" / ".." / "escaped.json"
    escaped_root = tmp_path / "nas" / "fresh-root-after-escape"

    with pytest.raises(V6ContractError, match="lexical path contract"):
        v6_formal_module.create_indoor_continuation_campaign(
            parent_manifest_path=parent_path,
            output_manifest_path=escaped_manifest,
            successor_output_root=escaped_root,
        )

    assert not escaped_root.exists()


def test_indoor_continuation_cli_creates_without_dispatch(
    tmp_path, monkeypatch, capsys
):
    parent_path, _parent, _run1, _successor_path, _successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    output = tmp_path / "nas" / "cli-successor.json"
    output_root = tmp_path / "nas" / "cli-successor-root"

    assert cli([
        "--continue-indoor-parent", str(parent_path),
        "--continuation-output-manifest", str(output),
        "--continuation-output-root", str(output_root),
    ]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["qualification"] == "INDOOR_CONTINUATION_READY"
    assert payload["formal_qualification"] == NOT_QUALIFIED
    assert payload["indoor_progress"] == "1/60"
    assert payload["dispatch"] is False
    assert payload["reason"] == "tooling_invalid_after_valid_episode"
    assert payload["manifest"] == str(output)


@pytest.mark.parametrize(
    "mutation",
    [
        "missing_root", "duplicate_mode", "duplicate_manifest", "duplicate_root",
        "other_mode", "other_output", "pilot", "timeout", "episode", "unknown",
        "three_equals", "mixed_equals", "abbreviated", "double_dash",
    ],
)
def test_indoor_continuation_cli_rejects_any_nonexact_raw_argv_without_outputs(
    tmp_path, mutation
):
    output = tmp_path / "successor.json"
    output_root = tmp_path / "successor-root"
    argv = [
        "--continue-indoor-parent", "/approved-parent.json",
        "--continuation-output-manifest", str(output),
        "--continuation-output-root", str(output_root),
    ]
    if mutation == "missing_root":
        argv = argv[:-2]
    elif mutation == "duplicate_mode":
        argv.extend(["--continue-indoor-parent", "/second-parent.json"])
    elif mutation == "duplicate_manifest":
        argv.extend(["--continuation-output-manifest", str(tmp_path / "second.json")])
    elif mutation == "duplicate_root":
        argv.extend(["--continuation-output-root", str(tmp_path / "second-root")])
    elif mutation == "other_mode":
        argv.extend(["--formal-manifest", "/formal.json"])
    elif mutation == "other_output":
        argv.extend(["--output-manifest", str(tmp_path / "other.json")])
    elif mutation == "pilot":
        argv.append("--pilot")
    elif mutation == "timeout":
        argv.extend(["--readiness-timeout-sec", "1"])
    elif mutation == "episode":
        argv.extend(["--episode-index", "1"])
    elif mutation == "unknown":
        argv.append("--unknown")
    elif mutation == "three_equals":
        argv = [
            "--continue-indoor-parent=/approved-parent.json",
            f"--continuation-output-manifest={output}",
            f"--continuation-output-root={output_root}",
        ]
    elif mutation == "mixed_equals":
        argv = [
            "--continue-indoor-parent=/approved-parent.json",
            "--continuation-output-manifest", str(output),
            "--continuation-output-root", str(output_root),
        ]
    elif mutation == "abbreviated":
        argv[0] = "--continue-indoor-par"
    else:
        argv.append("--")

    with pytest.raises(SystemExit) as exc_info:
        cli(argv)

    assert exc_info.value.code == 2
    assert not output.exists()
    assert not output_root.exists()


def test_indoor_continuation_extra_static_failure_exhausts_budget(
    tmp_path, monkeypatch
):
    _parent_path, _parent, _run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    _write_continuation_successor_run(
        successor.conditions[0],
        2,
        freeze_digest=successor.freeze_digest,
        stack_session_id="b" * 64,
        strict_success=False,
        path_deviation_percent=20.0,
    )

    result = evaluate_indoor_campaign(successor)

    assert result["conditions"][0]["runs"][1]["status"] == "product_failure"
    assert "indoor_static:early_fail_unreachable" in result["blockers"]
    assert result["conditions"][0]["next_run_index"] is None


def test_indoor_continuation_rejects_third_static_stack_session(
    tmp_path, monkeypatch
):
    _parent_path, _parent, _run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    _write_continuation_successor_run(
        successor.conditions[0],
        2,
        freeze_digest=successor.freeze_digest,
        stack_session_id="b" * 64,
    )
    _write_continuation_successor_run(
        successor.conditions[0],
        3,
        freeze_digest=successor.freeze_digest,
        stack_session_id="c" * 64,
    )

    result = evaluate_indoor_campaign(successor)

    assert "continuation_stack_session_mismatch" in result["conditions"][0]["blockers"]
    assert result["conditions"][0]["next_run_index"] is None


@pytest.mark.parametrize("field", ["sequence", "generation"])
def test_indoor_continuation_rejects_successor_sequence_or_generation_drift(
    tmp_path, monkeypatch, field
):
    _parent_path, _parent, _run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    root = _write_continuation_successor_run(
        successor.conditions[0],
        2,
        freeze_digest=successor.freeze_digest,
        stack_session_id="b" * 64,
    )
    manifest_path = root / "run_manifest.json"
    summary_path = root / "run_summary.json"
    episode = json.loads(manifest_path.read_text())
    summary = json.loads(summary_path.read_text())
    if field == "sequence":
        episode["stack_episode_receipt"]["sequence"] = 2
    else:
        episode["reset_receipt"]["generation"] = 3
        summary["reset_receipt"]["generation"] = 3
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    _refresh_checksums(root)

    result = evaluate_indoor_campaign(successor)

    assert result["conditions"][0]["runs"][1]["status"] == "invalid_evidence"
    assert result["conditions"][0]["next_run_index"] is None


@pytest.mark.parametrize("next_run_index", [1, 21])
def test_indoor_continuation_plan_rejects_run1_retry_or_run21(
    tmp_path, monkeypatch, next_run_index
):
    _parent_path, _parent, _run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    aggregate = evaluate_indoor_campaign(successor)
    aggregate["conditions"][0]["next_run_index"] = next_run_index

    with pytest.raises(V6ContractError, match="cannot retry run1|beyond run20"):
        v6_formal_module.indoor_dispatch_plan(successor, aggregate)


def test_indoor_continuation_final_pass_has_dedicated_qualification(
    tmp_path, monkeypatch
):
    _parent_path, _parent, _run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    for run_index in range(2, 21):
        _write_continuation_successor_run(
            successor.conditions[0],
            run_index,
            freeze_digest=successor.freeze_digest,
            stack_session_id="b" * 64,
            path_deviation_percent=10.0,
        )
    for condition, session in zip(successor.conditions[1:], ("d" * 64, "e" * 64)):
        for run_index in range(1, 21):
            _write_formal_run(
                condition,
                run_index,
                strict_success=run_index <= 18,
                formal_freeze_digest=successor.freeze_digest,
                stack_session_id=session,
            )

    result = evaluate_indoor_campaign(successor)

    assert result["qualification"] == (
        "INDOOR_QUALIFICATION_PASS_WITH_STATIC_CONTINUATION"
    )
    assert result["campaign_status"] == result["qualification"]
    assert result["qualification"] != "INDOOR_QUALIFICATION_PASS"
    assert result["formal_qualification"] == NOT_QUALIFIED
    static = result["conditions"][0]
    assert static["strict_successes"] == 19
    assert static["valid_episodes"] == 20
    assert static["product_failures"] == 1
    assert static["continuation"]["collision_count"] == 1
    assert static["continuation"]["cold_starts"] == 2
    assert static["continuation"]["hot_resets"] == 18
    assert all(row["strict_successes"] == 18 for row in result["conditions"][1:])
    assert result["continuation"]["static_result"] == (
        "19/20 strict + 1 collision product failure"
    )
    assert result["continuation"]["parent_freeze_digest"] == (
        successor.freeze["indoor_continuation"]["parent_manifest"]["freeze_digest"]
    )
    assert result["continuation"]["successor_freeze_digest"] == successor.freeze_digest


def test_indoor_loader_rejects_physical_obstacle_identity_drift(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    output = tmp_path / "indoor.json"
    freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=output,
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    raw = json.loads(output.read_text())
    raw["freeze"]["physical_contracts"]["indoor_static"][
        "static_obstacle_ids"
    ] = ["wrong"]
    output.write_text(json.dumps(raw), encoding="utf-8")

    with pytest.raises(V6ContractError, match="physical obstacle identity mismatch"):
        load_indoor_campaign_manifest(output)


@pytest.mark.parametrize("mutation", ["missing", "relative", "duplicate", "generic"])
def test_indoor_freezer_requires_unique_canonical_v6_spawn_override(
    tmp_path, monkeypatch, mutation
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    pilot = json.loads(pilot_manifest.read_text())
    arguments = pilot["conditions"][0]["runner_arguments"]
    index = next(
        i for i, value in enumerate(arguments) if value.startswith("spawn_poses_file:=")
    )
    if mutation == "missing":
        arguments.pop(index)
    elif mutation == "relative":
        arguments[index] = "spawn_poses_file:=kujiale.spawn.yaml"
    elif mutation == "duplicate":
        arguments.append(arguments[index])
    else:
        arguments[index] = (
            "spawn_poses_file:="
            + str(
                REPO
                / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml"
            )
        )
    pilot_manifest.write_text(json.dumps(pilot), encoding="utf-8")
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(V6ContractError, match="spawn_poses_file|duplicate|audited stable"):
        freeze_indoor_campaign_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate,
            output_manifest_path=tmp_path / "indoor.json",
            indoor_output_root=tmp_path / "nas" / "indoor-60",
        )


@pytest.mark.parametrize(
    "stable_key",
    [
        "nav2_profile", "nav2_config_file", "spawn_poses_file",
        "navigation_execution_backend", "require_module2_planning_ready",
        "module2_planning_ready_timeout_sec", "clear_slam_localization_buffer",
        "reset_map_base_translation_tolerance_m",
    ],
)
@pytest.mark.parametrize("mutation", ["missing", "wrong", "duplicate"])
def test_indoor_freezer_requires_each_exact_audited_stable_t3_argument(
    tmp_path, monkeypatch, stable_key, mutation
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    pilot = json.loads(pilot_manifest.read_text())
    arguments = pilot["conditions"][0]["runner_arguments"]
    index = next(
        i for i, value in enumerate(arguments) if value.startswith(f"{stable_key}:=")
    )
    if mutation == "missing":
        arguments.pop(index)
    elif mutation == "wrong":
        arguments[index] = f"{stable_key}:=wrong"
    else:
        arguments.append(arguments[index])
    pilot_manifest.write_text(json.dumps(pilot), encoding="utf-8")
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(V6ContractError, match="audited stable|duplicate"):
        freeze_indoor_campaign_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate,
            output_manifest_path=tmp_path / "indoor.json",
            indoor_output_root=tmp_path / "nas" / "indoor-60",
        )


def test_indoor_freezer_rejects_extra_unaudited_stable_t3_argument(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    pilot = json.loads(pilot_manifest.read_text())
    pilot["conditions"][0]["runner_arguments"].append("extra_stable_override:=true")
    pilot_manifest.write_text(json.dumps(pilot), encoding="utf-8")
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(V6ContractError, match="audited stable"):
        freeze_indoor_campaign_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate,
            output_manifest_path=tmp_path / "indoor.json",
            indoor_output_root=tmp_path / "nas" / "indoor-60",
        )


def test_indoor_freezer_rejects_failed_clear_slam_true_default_regression(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    pilot = json.loads(pilot_manifest.read_text())
    arguments = pilot["conditions"][0]["runner_arguments"]
    index = arguments.index("clear_slam_localization_buffer:=false")
    arguments[index] = "clear_slam_localization_buffer:=true"
    pilot_manifest.write_text(json.dumps(pilot), encoding="utf-8")
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(V6ContractError, match="audited stable"):
        freeze_indoor_campaign_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate,
            output_manifest_path=tmp_path / "indoor.json",
            indoor_output_root=tmp_path / "nas" / "indoor-60",
        )


@pytest.mark.parametrize("mutation", ["missing", "wrong_sha", "wrong_path"])
def test_indoor_freezer_requires_frozen_experiment_launch_source(
    tmp_path, monkeypatch, mutation
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    pilot = json.loads(pilot_manifest.read_text())
    rows = pilot["freeze"]["scenario_configs"]["indoor_static"]
    index = next(
        i for i, row in enumerate(rows)
        if row["path"].endswith("/launch/experiment.launch.py")
    )
    if mutation == "missing":
        rows.pop(index)
    elif mutation == "wrong_sha":
        rows[index]["sha256"] = "0" * 64
    else:
        rows[index]["path"] = str(REPO / "ros2_ws/src/robot_experiments/launch/wrong.py")
    pilot_manifest.write_text(json.dumps(pilot), encoding="utf-8")
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(
        V6ContractError,
        match="scenario config set mismatch|sha256|frozen file|readable file",
    ):
        freeze_indoor_campaign_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate,
            output_manifest_path=tmp_path / "indoor.json",
            indoor_output_root=tmp_path / "nas" / "indoor-60",
        )


def test_indoor_freezer_never_clobbers_existing_outputs(tmp_path, monkeypatch):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    output = tmp_path / "indoor-campaign.json"
    output.write_text("owner\n", encoding="utf-8")

    with pytest.raises(V6ContractError, match="new file"):
        freeze_indoor_campaign_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate,
            output_manifest_path=output,
            indoor_output_root=tmp_path / "nas" / "indoor-60",
        )

    assert output.read_text() == "owner\n"


@pytest.mark.parametrize("provenance_value", [None, True])
def test_indoor_freezer_requires_explicit_clean_tracked_provenance(
    tmp_path, monkeypatch, provenance_value
):
    pilot_manifest, aggregate_path = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    aggregate = json.loads(aggregate_path.read_text())
    manifest_path = Path(aggregate["conditions"][0]["episodes"][0]["manifest_path"])
    episode = json.loads(manifest_path.read_text())
    if provenance_value is None:
        episode["provenance"].pop("git_tracked_dirty")
    else:
        episode["provenance"]["git_tracked_dirty"] = provenance_value
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    _refresh_checksums(manifest_path.parent)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(V6ContractError, match="source/config provenance mismatch"):
        freeze_indoor_campaign_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate_path,
            output_manifest_path=tmp_path / "indoor.json",
            indoor_output_root=tmp_path / "nas" / "indoor-60",
        )


def test_indoor_freezer_allows_untracked_diagnostics_when_tracked_clean(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate_path = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    aggregate = json.loads(aggregate_path.read_text())
    manifest_path = Path(aggregate["conditions"][0]["episodes"][0]["manifest_path"])
    episode = json.loads(manifest_path.read_text())
    episode["provenance"]["git_dirty"] = True
    episode["provenance"]["git_tracked_dirty"] = False
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    _refresh_checksums(manifest_path.parent)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate_path,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )

    assert campaign.freeze_digest


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("head", "source/config provenance mismatch"),
        ("session", "frozen tuple/session mismatch"),
        ("generation", "primary evidence failed"),
    ],
)
def test_indoor_freezer_rejects_mixed_head_session_or_generation(
    tmp_path, monkeypatch, mutation, message
):
    pilot_manifest, aggregate_path = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    aggregate = json.loads(aggregate_path.read_text())
    row = aggregate["conditions"][0]["episodes"][1]
    manifest_path = Path(row["manifest_path"])
    summary_path = Path(row["summary_path"])
    episode = json.loads(manifest_path.read_text())
    summary = json.loads(summary_path.read_text())
    if mutation == "head":
        episode["provenance"]["git_head"] = "0" * 40
    elif mutation == "session":
        episode["stack_session_id"] = "b" * 64
        summary["stack_session_id"] = "b" * 64
    else:
        episode["reset_receipt"]["generation"] = 9
        summary["reset_receipt"]["generation"] = 9
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    _refresh_checksums(manifest_path.parent)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(V6ContractError, match=message):
        freeze_indoor_campaign_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate_path,
            output_manifest_path=tmp_path / "indoor.json",
            indoor_output_root=tmp_path / "nas" / "indoor-60",
        )


def test_sufficient_pilot_aggregate_rejects_missing_exact_run_path(
    tmp_path, monkeypatch
):
    pilot_root = _write_production_pilot_root(tmp_path, monkeypatch)
    missing = next((pilot_root / "indoor_static" / "rep2").rglob("run_summary.json"))
    missing.unlink()
    manifest_output = tmp_path / "nas" / "missing-manifest.json"
    aggregate_output = tmp_path / "nas" / "missing-aggregate.json"

    with pytest.raises(V6ContractError):
        v6_formal_module.aggregate_sufficient_pilot(
            pilot_root=pilot_root,
            output_manifest=manifest_output,
            output_aggregate=aggregate_output,
        )

    assert not manifest_output.exists()
    assert not aggregate_output.exists()


def test_pilot_pair_publish_rolls_back_second_link_oserror_and_retries(
    tmp_path, monkeypatch
):
    manifest_output = tmp_path / "pilot-manifest.json"
    aggregate_output = tmp_path / "pilot-aggregate.json"
    real_link = v6_formal_module.os.link
    calls = 0

    def fail_second_link(source, target):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise OSError(errno.EIO, "injected second-link failure")
        return real_link(source, target)

    monkeypatch.setattr(v6_formal_module.os, "link", fail_second_link)

    with pytest.raises(V6ContractError, match="pair publish failed"):
        v6_formal_module._publish_no_clobber_json_pair(
            manifest_output,
            {"kind": "manifest"},
            aggregate_output,
            {"kind": "aggregate"},
        )

    assert not manifest_output.exists()
    assert not aggregate_output.exists()

    monkeypatch.setattr(v6_formal_module.os, "link", real_link)
    v6_formal_module._publish_no_clobber_json_pair(
        manifest_output,
        {"kind": "manifest"},
        aggregate_output,
        {"kind": "aggregate"},
    )
    assert json.loads(manifest_output.read_text()) == {"kind": "manifest"}
    assert json.loads(aggregate_output.read_text()) == {"kind": "aggregate"}


@pytest.mark.parametrize(
    ("fault", "message"),
    [
        ("strict", "primary evidence failed"),
        ("config", "source/config provenance mismatch"),
        ("runtime_hashes", "source/config provenance mismatch"),
        ("session", "frozen tuple/session mismatch"),
        ("sequence", "stack episode sequence/T2 receipt mismatch"),
        ("counts", "primary evidence failed"),
        ("collision", "primary evidence failed"),
        ("completion", "primary evidence failed"),
        ("boundary", "cold/hot episode order mismatch"),
        ("order", "condition order/identity mismatch"),
    ],
)
def test_sufficient_pilot_freezer_fails_closed_and_writes_nothing(
    tmp_path, monkeypatch, fault, message
):
    pilot_manifest, aggregate_path, _reference = _write_sufficient_pilot_inputs(
        tmp_path,
        first_collision=(fault == "collision"),
        first_route_completion_count=(1 if fault == "completion" else 5),
    )
    aggregate = json.loads(aggregate_path.read_text())
    first_episode = aggregate["conditions"][0]["episodes"][0]
    if fault == "strict":
        summary_path = Path(first_episode["summary_path"])
        summary = json.loads(summary_path.read_text())
        summary["strict_success"] = False
        summary_path.write_text(json.dumps(summary), encoding="utf-8")
        _refresh_checksums(summary_path.parent)
    elif fault == "config":
        manifest_path = Path(first_episode["manifest_path"])
        episode = json.loads(manifest_path.read_text())
        episode["nav2_config_hash"] = "0" * 64
        manifest_path.write_text(json.dumps(episode), encoding="utf-8")
        _refresh_checksums(manifest_path.parent)
    elif fault in {"runtime_hashes", "session", "sequence"}:
        manifest_path = Path(first_episode["manifest_path"])
        summary_path = Path(first_episode["summary_path"])
        episode = json.loads(manifest_path.read_text())
        summary = json.loads(summary_path.read_text())
        if fault == "runtime_hashes":
            episode["scenario_runtime_hashes"]["nav2_config"] = "0" * 64
        elif fault == "session":
            episode["stack_session_id"] = "b" * 64
            summary["stack_session_id"] = "b" * 64
        else:
            episode["stack_episode_receipt"]["sequence"] = 2
            summary["condition_stack_attestation"]["stack_episode_receipt"][
                "sequence"
            ] = 2
        manifest_path.write_text(json.dumps(episode), encoding="utf-8")
        summary_path.write_text(json.dumps(summary), encoding="utf-8")
        _refresh_checksums(manifest_path.parent)
    elif fault == "boundary":
        first_episode["boundary"] = "hot_reset"
        aggregate_path.write_text(json.dumps(aggregate), encoding="utf-8")
    elif fault == "counts":
        metadata_path = Path(first_episode["summary_path"]).parent / "telemetry" / "metadata.yaml"
        metadata = yaml.safe_load(metadata_path.read_text())
        metadata["rosbag2_bagfile_information"]["message_count"] += 1
        metadata_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")
        _refresh_checksums(metadata_path.parents[1])
    elif fault in {"collision", "completion"}:
        pass
    else:
        aggregate["conditions"][0], aggregate["conditions"][1] = (
            aggregate["conditions"][1], aggregate["conditions"][0]
        )
        aggregate_path.write_text(json.dumps(aggregate), encoding="utf-8")
    output = tmp_path / "must-not-exist.json"
    formal_root = tmp_path / "nas" / "formal"
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(V6ContractError, match=message):
        freeze_formal_manifest_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate_path,
            output_manifest_path=output,
            formal_output_root=formal_root,
        )

    assert not output.exists()
    assert not formal_root.exists()


def test_sufficient_pilot_freezer_rejects_existing_formal_root(tmp_path, monkeypatch):
    pilot_manifest, aggregate, _reference = _write_sufficient_pilot_inputs(tmp_path)
    formal_root = tmp_path / "nas" / "formal"
    formal_root.mkdir(parents=True)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(V6ContractError, match="must be new"):
        freeze_formal_manifest_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate,
            output_manifest_path=tmp_path / "output.json",
            formal_output_root=formal_root,
        )


def test_sufficient_pilot_freezer_rejects_normalized_stack_tuple_drift(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate_path, _reference = _write_sufficient_pilot_inputs(
        tmp_path
    )
    aggregate = json.loads(aggregate_path.read_text())
    episodes = aggregate["conditions"][0]["episodes"]
    contract_path = Path(episodes[0]["stack_contract_path"])
    contract = json.loads(contract_path.read_text())
    contract["driver_version"] = "stale-driver"
    contract["stack_session_id"] = v6_formal_module._stack_session_id(contract)
    contract_path.write_text(json.dumps(contract), encoding="utf-8")
    tuple_digest = v6_formal_module._stack_tuple_digest(contract)
    for row in episodes:
        row["stack_tuple_digest"] = tuple_digest
        manifest_path = Path(row["manifest_path"])
        summary_path = Path(row["summary_path"])
        episode = json.loads(manifest_path.read_text())
        summary = json.loads(summary_path.read_text())
        episode["stack_session_id"] = contract["stack_session_id"]
        summary["stack_session_id"] = contract["stack_session_id"]
        manifest_path.write_text(json.dumps(episode), encoding="utf-8")
        summary_path.write_text(json.dumps(summary), encoding="utf-8")
        _refresh_checksums(manifest_path.parent)
    aggregate_path.write_text(json.dumps(aggregate), encoding="utf-8")
    formal_root = tmp_path / "nas" / "formal"
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    with pytest.raises(V6ContractError, match="stack contract frozen tuple mismatch"):
        freeze_formal_manifest_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate_path,
            output_manifest_path=tmp_path / "output.json",
            formal_output_root=formal_root,
        )

    assert not formal_root.exists()


def test_sufficient_pilot_freezer_cli_never_dispatches(tmp_path, monkeypatch, capsys):
    pilot_manifest, aggregate, _reference = _write_sufficient_pilot_inputs(tmp_path)
    output = tmp_path / "formal.json"
    formal_root = tmp_path / "nas" / "formal"
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    assert cli([
        "--pilot-manifest", str(pilot_manifest),
        "--pilot-aggregate", str(aggregate),
        "--output-manifest", str(output),
        "--formal-output-root", str(formal_root),
    ]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["qualification"] == "FORMAL_READY_MANIFEST"
    assert payload["execution_authorization"] == "NOT_AUTHORIZED"
    assert payload["formal_progress"] == "0/120"
    assert payload["dispatch"] is False


def test_sufficient_pilot_freezer_atomic_publish_never_clobbers(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate, _reference = _write_sufficient_pilot_inputs(tmp_path)
    output = tmp_path / "formal.json"
    formal_root = tmp_path / "nas" / "formal"
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")

    def raced_link(_source, target):
        Path(target).write_text("racer\n", encoding="utf-8")
        raise FileExistsError

    monkeypatch.setattr(v6_formal_module.os, "link", raced_link)

    with pytest.raises(V6ContractError, match="already exists"):
        freeze_formal_manifest_from_pilot(
            pilot_manifest_path=pilot_manifest,
            pilot_aggregate_path=aggregate,
            output_manifest_path=output,
            formal_output_root=formal_root,
        )

    assert output.read_text() == "racer\n"
    assert not formal_root.exists()


def test_formal_loader_rejects_post_freeze_pilot_evidence_drift(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate, _reference = _write_sufficient_pilot_inputs(tmp_path)
    output = tmp_path / "formal.json"
    formal_root = tmp_path / "nas" / "formal"
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    frozen = freeze_formal_manifest_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=output,
        formal_output_root=formal_root,
    )
    summary_path = Path(
        frozen.pilot_freeze_provenance["episodes"][0]["summary"]["path"]
    )
    summary_path.write_text(summary_path.read_text() + "\n", encoding="utf-8")

    with pytest.raises(V6ContractError, match="sha256 mismatch"):
        load_formal_campaign_manifest(output)


def test_formal_manifest_requires_source_runner_and_route_prior_contract(tmp_path):
    raw = _formal_raw(tmp_path)
    raw["runner_entrypoint"] = str(tmp_path / "missing-runner")
    with pytest.raises(V6ContractError, match="runner_entrypoint"):
        load_formal_campaign_manifest(_write_manifest(tmp_path, raw))

    for argument, message in (
        ("navigation_execution_backend:=navigate_to_pose", "route_guided"),
        ("require_module2_planning_ready:=false", "planning readiness"),
    ):
        raw = _formal_raw(tmp_path)
        name = argument.split(":=", 1)[0]
        raw["conditions"][0]["runner_arguments"] = [
            argument if item.startswith(f"{name}:=") else item
            for item in raw["conditions"][0]["runner_arguments"]
        ]
        with pytest.raises(V6ContractError, match=message):
            load_formal_campaign_manifest(_write_manifest(tmp_path, raw))


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("repo_head", "repository head mismatch"),
        ("driver", "driver_version mismatch"),
        ("file_hash", "sha256 mismatch"),
        ("asset_keys", "freeze.frozen_assets keys"),
    ],
)
def test_formal_freeze_rejects_tuple_or_file_drift(tmp_path, mutation, message):
    raw = _formal_raw(tmp_path)
    if mutation == "repo_head":
        raw["freeze"]["repositories"]["module3"]["head"] = "0" * 40
    elif mutation == "driver":
        raw["freeze"]["driver_version"] = "stale-driver"
    elif mutation == "file_hash":
        raw["freeze"]["v6_formal"]["sha256"] = "0" * 64
    else:
        raw["freeze"]["frozen_assets"].pop("dino_checkpoint")

    with pytest.raises(V6ContractError, match=message):
        load_formal_campaign_manifest(_write_manifest(tmp_path, raw))


def test_formal_freeze_rejects_rivermark_constraint_tree_drift(tmp_path):
    raw = _formal_raw(tmp_path)
    tree = Path(
        raw["freeze"]["frozen_assets"]["rivermark_catalog_constraints_tree"]["path"]
    )
    (tree / "region_02.json").write_text('{"changed": true}\n', encoding="utf-8")

    with pytest.raises(V6ContractError, match="sha256 mismatch"):
        load_formal_campaign_manifest(_write_manifest(tmp_path, raw))


def test_formal_execution_requires_flag_and_authorized_manifest(tmp_path, capsys):
    path = _write_formal_manifest(tmp_path)

    assert cli(["--formal-manifest", str(path), "--execute-formal"]) == 2
    assert "requires --condition-stack-id" in capsys.readouterr().err

    assert cli([
        "--formal-manifest",
        str(path),
        "--execute-formal",
        "--condition-stack-id",
        "indoor_static",
        "--condition-stack-contract",
        "/missing/stack.contract.json",
    ]) == 2
    assert "manifest is NOT_AUTHORIZED" in capsys.readouterr().err

    assert cli(["--manifest", str(MANIFEST), "--execute-formal"]) == 2
    assert "require --formal-manifest" in capsys.readouterr().err


def test_authorized_manifest_without_pilot_provenance_is_rejected(tmp_path):
    path = _write_formal_manifest(tmp_path, authorization="AUTHORIZED")

    with pytest.raises(V6ContractError, match="requires complete Pilot"):
        load_formal_campaign_manifest(path)


def test_formal_execution_rejects_wrong_stack_without_subprocess(
    tmp_path, monkeypatch
):
    campaign = load_formal_campaign_manifest(
        _write_authorized_formal_manifest(tmp_path)
    )
    calls = []
    monkeypatch.setattr(
        v6_formal_module.subprocess,
        "run",
        lambda *args, **kwargs: calls.append((args, kwargs)),
    )

    with pytest.raises(V6ContractError, match="unknown formal condition stack"):
        execute_formal_campaign(
            campaign,
            condition_stack_id="wrong_stack",
            condition_stack_contract="/missing/stack.contract.json",
        )

    assert calls == []


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"integration_head": "0" * 40}, "frozen tuple mismatch"),
        ({"driver_version": "stale-driver"}, "frozen tuple mismatch"),
    ],
)
def test_formal_execution_rejects_stack_freeze_mismatch(
    tmp_path, monkeypatch, override, message
):
    campaign = load_formal_campaign_manifest(
        _write_authorized_formal_manifest(tmp_path)
    )
    contract = _live_stack_contract(tmp_path, **override)
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    monkeypatch.setattr(
        v6_formal_module.subprocess,
        "run",
        lambda *args, **kwargs: pytest.fail("subprocess must not run"),
    )

    with pytest.raises(V6ContractError, match=message):
        execute_formal_campaign(
            campaign,
            condition_stack_id="indoor_static",
            condition_stack_contract=contract,
        )


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"boot_id": "stale-boot"}, "boot_id is stale"),
        ({"start_ticks": 1}, "start_ticks is stale"),
    ],
)
def test_stack_contract_rejects_stale_process_identity(
    tmp_path, overrides, message
):
    contract = _live_stack_contract(tmp_path, **overrides)

    with pytest.raises(V6ContractError, match=message):
        validate_condition_stack_contract(
            contract, expected_condition_id="indoor_static"
        )


def test_stack_contract_rejects_wrong_condition(tmp_path):
    contract = _live_stack_contract(tmp_path)

    with pytest.raises(V6ContractError, match="condition mismatch"):
        validate_condition_stack_contract(
            contract, expected_condition_id="outdoor_static"
        )


def test_stack_contract_rejects_dead_process(tmp_path):
    process = subprocess.Popen(["sleep", "30"])
    contract = _live_stack_contract(tmp_path, pid=process.pid)
    process.terminate()
    process.wait(timeout=5)

    with pytest.raises(V6ContractError, match="process is not live"):
        validate_condition_stack_contract(
            contract, expected_condition_id="indoor_static"
        )


def test_formal_execution_dispatches_one_episode_and_returns(
    tmp_path, monkeypatch
):
    campaign = load_formal_campaign_manifest(
        _write_authorized_formal_manifest(tmp_path)
    )
    contract = _live_stack_contract(tmp_path)
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    contract_payload = json.loads(contract.read_text())
    calls = []

    def fake_run(command, *, check):
        calls.append((command, check))
        _write_formal_run(
            campaign.conditions[0],
            1,
            strict_success=True,
            formal_freeze_digest=campaign.freeze_digest,
            stack_session_id=contract_payload["stack_session_id"],
        )

    monkeypatch.setattr(v6_formal_module.subprocess, "run", fake_run)

    aggregate = execute_formal_campaign(
        campaign,
        condition_stack_id="indoor_static",
        condition_stack_contract=contract,
    )

    assert len(calls) == 1
    assert calls[0][1] is True
    assert "run_indices:=1" in calls[0][0]
    assert f"condition_stack_id:=indoor_static" in calls[0][0]
    assert any(
        argument == f"stack_session_id:={contract_payload['stack_session_id']}"
        for argument in calls[0][0]
    )
    assert f"formal_freeze_digest:={campaign.freeze_digest}" in calls[0][0]
    assert f"condition_stack_contract_path:={contract.resolve()}" in calls[0][0]
    assert aggregate["present_episodes"] == 1


@pytest.mark.parametrize("failure", ["tampered_record", "checkout_drift"])
def test_indoor_execute_revalidates_promotion_before_evaluate_plan_or_subprocess(
    tmp_path, monkeypatch, failure
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    promotion = campaign.freeze["validator_only_head_promotion"]
    if failure == "tampered_record":
        promotion["module3_diff"]["canonical_diff_sha256"] = "f" * 64
    calls = {"validator": 0, "evaluate": 0, "plan": 0, "subprocess": 0}

    def reject_promotion(value, *, freeze):
        calls["validator"] += 1
        assert value is promotion
        raise V6ContractError(failure)

    def forbidden(name):
        def fail(*_args, **_kwargs):
            calls[name] += 1
            raise AssertionError(f"{name} ran before promotion validation")
        return fail

    monkeypatch.setattr(
        v6_formal_module, "_validate_validator_only_head_promotion", reject_promotion
    )
    monkeypatch.setattr(v6_formal_module, "evaluate_indoor_campaign", forbidden("evaluate"))
    monkeypatch.setattr(v6_formal_module, "indoor_dispatch_plan", forbidden("plan"))
    monkeypatch.setattr(v6_formal_module.subprocess, "run", forbidden("subprocess"))

    with pytest.raises(V6ContractError, match=failure):
        v6_formal_module.execute_indoor_campaign(
            campaign,
            condition_stack_id="indoor_static",
            condition_stack_contract=tmp_path / "never-read.json",
        )

    assert calls == {"validator": 1, "evaluate": 0, "plan": 0, "subprocess": 0}


def test_indoor_execution_dispatches_exactly_one_episode(tmp_path, monkeypatch):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    contract = _live_stack_contract(tmp_path, condition_id="indoor_static")
    contract_payload = json.loads(contract.read_text())
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    calls = []

    dynamic_root = tmp_path / "dynamic"
    dynamic_root.mkdir()
    dynamic_contract = _live_stack_contract(
        dynamic_root, condition_id="indoor_dynamic"
    )
    with pytest.raises(V6ContractError, match="no unique pending"):
        execute_indoor_campaign(
            campaign,
            condition_stack_id="indoor_dynamic",
            condition_stack_contract=dynamic_contract,
        )

    def fake_run(command, *, check):
        calls.append((command, check))
        _write_formal_run(
            campaign.conditions[0],
            1,
            strict_success=True,
            formal_freeze_digest=campaign.freeze_digest,
            stack_session_id=contract_payload["stack_session_id"],
        )

    monkeypatch.setattr(v6_formal_module.subprocess, "run", fake_run)

    result = execute_indoor_campaign(
        campaign,
        condition_stack_id="indoor_static",
        condition_stack_contract=contract,
    )

    assert len(calls) == 1
    command = calls[0][0]
    assert "run_indices:=1" in command
    assert (
        f"spawn_poses_file:={v6_formal_module._canonical_indoor_spawn_manifest()}"
        in command
    )
    named_arguments = [item for item in command if ":=" in item]
    names = [item.split(":=", 1)[0] for item in named_arguments]
    stable_names = {
        item.split(":=", 1)[0] for item in _expected_indoor_stable_arguments()
    }
    dynamic_names = {
        "record_evidence", "record_bag", "fail_stop", "run_indices", "resume",
        "condition_stack_id", "stack_session_id", "formal_freeze_digest",
        "condition_stack_contract_path",
    }
    assert set(names) == stable_names | dynamic_names
    assert all(names.count(name) == 1 for name in stable_names | dynamic_names)
    assert command.count(str(campaign.conditions[0].scenario_file)) == 1
    assert command.count(str(campaign.conditions[0].output_directory)) == 1
    assert result["present_episodes"] == 1
    assert result["formal_qualification"] == NOT_QUALIFIED


def test_indoor_continuation_programmatic_dispatch_starts_at_run2_and_locks_epoch2_session(
    tmp_path, monkeypatch
):
    _parent_path, _parent, _run1, _successor_path, successor = (
        _write_indoor_continuation_inputs(tmp_path, monkeypatch)
    )
    contract_root = tmp_path / "continuation-live"
    contract_root.mkdir()
    contract = _live_stack_contract(contract_root, condition_id="indoor_static")
    contract_payload = json.loads(contract.read_text())
    parent_session = successor.freeze["indoor_continuation"]["preserved_run"][
        "stack_session_id"
    ]
    assert contract_payload["stack_session_id"] != parent_session
    third_root = tmp_path / "third-live"
    third_root.mkdir()
    third_contract = _live_stack_contract(third_root, condition_id="indoor_static")
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    calls = []

    def fake_run(command, *, check):
        calls.append((command, check))
        _write_continuation_successor_run(
            successor.conditions[0],
            2,
            freeze_digest=successor.freeze_digest,
            stack_session_id=contract_payload["stack_session_id"],
        )

    monkeypatch.setattr(v6_formal_module.subprocess, "run", fake_run)

    result = execute_indoor_campaign(
        successor,
        condition_stack_id="indoor_static",
        condition_stack_contract=contract,
    )

    assert len(calls) == 1
    assert "run_indices:=2" in calls[0][0]
    assert "run_indices:=1" not in calls[0][0]
    assert result["present_episodes"] == 2
    assert result["conditions"][0]["stack_session_id"] == contract_payload[
        "stack_session_id"
    ]
    with pytest.raises(V6ContractError, match="live indoor stack session differs"):
        execute_indoor_campaign(
            successor,
            condition_stack_id="indoor_static",
            condition_stack_contract=third_contract,
        )
    assert len(calls) == 1
    stale_payload = json.loads(contract.read_text())
    stale_payload["pid"] = 99999999
    stale_payload["stack_session_id"] = v6_formal_module._stack_session_id(
        stale_payload
    )
    stale_contract = tmp_path / "stale-stack.json"
    stale_contract.write_text(json.dumps(stale_payload), encoding="utf-8")
    with pytest.raises(V6ContractError, match="process is not live"):
        execute_indoor_campaign(
            successor,
            condition_stack_id="indoor_static",
            condition_stack_contract=stale_contract,
        )
    assert len(calls) == 1


def test_indoor_execution_keeps_valid_failure_and_advances_identity(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    contract = _live_stack_contract(tmp_path, condition_id="indoor_static")
    contract_payload = json.loads(contract.read_text())
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")

    def fake_run(_command, *, check):
        assert check is True
        _write_formal_run(
            campaign.conditions[0],
            1,
            strict_success=False,
            formal_freeze_digest=campaign.freeze_digest,
            stack_session_id=contract_payload["stack_session_id"],
            path_deviation_percent=20.0,
        )

    monkeypatch.setattr(v6_formal_module.subprocess, "run", fake_run)

    result = execute_indoor_campaign(
        campaign,
        condition_stack_id="indoor_static",
        condition_stack_contract=contract,
    )

    static = result["conditions"][0]
    assert static["runs"][0]["status"] == "product_failure"
    assert static["valid_episodes"] == 1
    assert static["next_run_index"] == 2


def test_complete_indoor_campaign_never_reports_formal_pass(tmp_path, monkeypatch):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    for condition in campaign.conditions:
        for run_index in range(1, 21):
            allowed_failure = (
                condition.condition_id == "indoor_static" and run_index == 20
            ) or (
                condition.condition_id in {"indoor_dynamic", "indoor_appearance"}
                and run_index in {19, 20}
            )
            _write_formal_run(
                condition,
                run_index,
                strict_success=not allowed_failure,
                formal_freeze_digest=campaign.freeze_digest,
                path_deviation_percent=(20.0 if allowed_failure else 10.0),
            )

    result = evaluate_indoor_campaign(campaign)

    assert result["strict_successes"] == 55
    assert result["valid_episodes"] == 60
    assert result["qualification"] == "INDOOR_QUALIFICATION_PASS"
    assert result["formal_qualification"] == NOT_QUALIFIED
    static = result["conditions"][0]
    assert static["strict_successes"] == 19
    assert static["product_failures"] == 1
    assert static["path_deviation_percent"]["count"] == 19
    assert static["path_deviation_percent"]["max"] == 10.0


def test_indoor_threshold_does_not_pass_before_all_twenty_valid_runs(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    static = campaign.conditions[0]
    for run_index in range(1, 20):
        _write_formal_run(
            static,
            run_index,
            strict_success=True,
            formal_freeze_digest=campaign.freeze_digest,
        )

    result = evaluate_indoor_campaign(campaign)

    static_result = result["conditions"][0]
    assert static_result["strict_successes"] == 19
    assert static_result["valid_episodes"] == 19
    assert static_result["qualification"] == "INCOMPLETE"
    assert static_result["next_run_index"] == 20
    assert result["campaign_status"] == "IN_PROGRESS"


def test_indoor_valid_failures_continue_until_budget_is_unreachable(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    static = campaign.conditions[0]
    _write_formal_run(
        static,
        1,
        strict_success=False,
        formal_freeze_digest=campaign.freeze_digest,
        path_deviation_percent=20.0,
    )
    within_budget = evaluate_indoor_campaign(campaign)
    assert within_budget["conditions"][0]["runs"][0]["status"] == "product_failure"
    assert within_budget["conditions"][0]["next_run_index"] == 2
    assert within_budget["blockers"] == []

    _write_formal_run(
        static,
        2,
        strict_success=False,
        formal_freeze_digest=campaign.freeze_digest,
        path_deviation_percent=21.0,
    )
    unreachable = evaluate_indoor_campaign(campaign)
    assert unreachable["campaign_status"] == "EARLY_FAIL_UNREACHABLE"
    assert unreachable["conditions"][0]["next_run_index"] is None
    assert unreachable["conditions"][0]["product_failures"] == 2


def test_indoor_invalid_evidence_stops_without_entering_valid_denominator(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=False,
        valid=False,
        formal_freeze_digest=campaign.freeze_digest,
    )

    result = evaluate_indoor_campaign(campaign)

    assert result["campaign_status"] == "STOP_INVALID"
    assert result["valid_episodes"] == 0
    assert result["conditions"][0]["next_run_index"] is None


def test_indoor_final_rejects_summary_success_tampering_over_manifest_failure(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
    )
    manifest_path = root / "run_manifest.json"
    summary_path = root / "run_summary.json"
    episode = json.loads(manifest_path.read_text())
    summary = json.loads(summary_path.read_text())
    episode["result"] = "failure"
    summary["navigation_contract_success"] = True
    summary["strict_success"] = True
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    _refresh_checksums(root)

    result = evaluate_indoor_campaign(campaign)

    run = result["conditions"][0]["runs"][0]
    assert run["status"] == "invalid_evidence"
    assert "navigation_contract_mismatch" in run["detail"]
    assert result["campaign_status"] == "STOP_INVALID"


def test_indoor_final_rejects_one_of_five_route_forged_to_strict_success(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=False,
        route_completion_count=1,
        formal_freeze_digest=campaign.freeze_digest,
    )
    manifest_path = root / "run_manifest.json"
    summary_path = root / "run_summary.json"
    episode = json.loads(manifest_path.read_text())
    summary = json.loads(summary_path.read_text())
    episode["result"] = "success"
    summary["navigation_contract_success"] = True
    summary["strict_success"] = True
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    _refresh_checksums(root)

    result = evaluate_indoor_campaign(campaign)

    run = result["conditions"][0]["runs"][0]
    assert run["status"] == "invalid_evidence"
    assert "strict_recorded_navigation_completion_missing" in run["detail"]
    assert result["strict_successes"] == 0
    assert result["campaign_status"] == "STOP_INVALID"


def test_indoor_final_keeps_one_of_five_route_as_valid_product_failure(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=False,
        route_completion_count=1,
        formal_freeze_digest=campaign.freeze_digest,
    )

    result = evaluate_indoor_campaign(campaign)

    run = result["conditions"][0]["runs"][0]
    assert run["status"] == "product_failure"
    assert result["valid_episodes"] == 1
    assert result["strict_successes"] == 0
    assert result["blockers"] == []


def test_indoor_final_cross_checks_contact_sensor_product_failure(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=False,
        collision_detected=True,
        formal_freeze_digest=campaign.freeze_digest,
    )

    result = evaluate_indoor_campaign(campaign)

    run = result["conditions"][0]["runs"][0]
    assert run["status"] == "product_failure"
    assert result["valid_episodes"] == 1
    assert result["blockers"] == []


@pytest.mark.parametrize(
    ("summary_metric", "episode_metric", "expected_status"),
    [
        (None, None, "product_failure"),
        (float("nan"), float("nan"), "invalid_evidence"),
        (float("inf"), float("inf"), "invalid_evidence"),
        (None, 10.0, "invalid_evidence"),
        (10.0, None, "invalid_evidence"),
        (10.0, 11.0, "invalid_evidence"),
        (20.0, 20.0, "product_failure"),
        (21.0, 21.0, "product_failure"),
        (19.9, 19.9, "strict_success"),
    ],
)
def test_indoor_static_metric_pair_classification(
    tmp_path, monkeypatch, summary_metric, episode_metric, expected_status
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
    )
    summary_path = root / "run_summary.json"
    manifest_path = root / "run_manifest.json"
    summary = json.loads(summary_path.read_text())
    episode = json.loads(manifest_path.read_text())
    summary["path_deviation_percent"] = summary_metric
    episode["metrics"]["path_deviation_percent"] = episode_metric
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    _refresh_checksums(root)

    result = evaluate_indoor_campaign(campaign)

    run = result["conditions"][0]["runs"][0]
    assert run["status"] == expected_status
    if expected_status == "invalid_evidence":
        assert result["campaign_status"] == "STOP_INVALID"
    else:
        assert result["valid_episodes"] == 1


def test_indoor_static_null_metric_still_requires_reference_hash(
    tmp_path, monkeypatch
):
    pilot_manifest, aggregate = _write_indoor_pilot_inputs(tmp_path, monkeypatch)
    monkeypatch.setattr(v6_formal_module, "FORMAL_NAS_ROOT", tmp_path / "nas")
    campaign = freeze_indoor_campaign_from_pilot(
        pilot_manifest_path=pilot_manifest,
        pilot_aggregate_path=aggregate,
        output_manifest_path=tmp_path / "indoor.json",
        indoor_output_root=tmp_path / "nas" / "indoor-60",
    )
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=False,
        formal_freeze_digest=campaign.freeze_digest,
        path_deviation_percent=None,
    )
    manifest_path = root / "run_manifest.json"
    episode = json.loads(manifest_path.read_text())
    episode["optimal_reference_hash"] = "0" * 64
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    _refresh_checksums(root)

    result = evaluate_indoor_campaign(campaign)

    assert result["conditions"][0]["runs"][0]["status"] == "invalid_evidence"


def test_formal_execution_rejects_live_session_change_before_subprocess(
    tmp_path, monkeypatch
):
    campaign = load_formal_campaign_manifest(
        _write_authorized_formal_manifest(tmp_path)
    )
    _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
        stack_session_id="a" * 64,
    )
    contract = _live_stack_contract(tmp_path)
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    calls = []
    monkeypatch.setattr(
        v6_formal_module.subprocess,
        "run",
        lambda *args, **kwargs: calls.append((args, kwargs)),
    )

    with pytest.raises(V6ContractError, match="differs from recorded episodes"):
        execute_formal_campaign(
            campaign,
            condition_stack_id="indoor_static",
            condition_stack_contract=contract,
        )

    assert calls == []


def test_formal_execution_preserves_valid_product_failure_and_continues(
    tmp_path, monkeypatch
):
    campaign = load_formal_campaign_manifest(
        _write_authorized_formal_manifest(tmp_path)
    )
    contract = _live_stack_contract(tmp_path)
    contract_payload = json.loads(contract.read_text())
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    calls = []

    def fake_run(command, *, check):
        calls.append((command, check))
        _write_formal_run(
            campaign.conditions[0],
            1,
            strict_success=False,
            formal_freeze_digest=campaign.freeze_digest,
            stack_session_id=contract_payload["stack_session_id"],
        )

    monkeypatch.setattr(v6_formal_module.subprocess, "run", fake_run)

    aggregate = execute_formal_campaign(
        campaign,
        condition_stack_id="indoor_static",
        condition_stack_contract=contract,
    )

    assert len(calls) == 1
    static = aggregate["conditions"][0]
    assert static["runs"][0]["status"] == "product_failure"
    assert static["next_run_index"] == 2


def test_formal_execution_requires_exactly_one_new_valid_target(
    tmp_path, monkeypatch
):
    campaign = load_formal_campaign_manifest(
        _write_authorized_formal_manifest(tmp_path)
    )
    contract = _live_stack_contract(tmp_path)
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    monkeypatch.setattr(v6_formal_module.subprocess, "run", lambda *args, **kwargs: None)

    with pytest.raises(V6ContractError, match="exactly one valid target"):
        execute_formal_campaign(
            campaign,
            condition_stack_id="indoor_static",
            condition_stack_contract=contract,
        )


def test_formal_shell_requires_and_forwards_condition_stack_id():
    source = (REPO / "scripts" / "run_v6_formal_episode.sh").read_text()
    assert "--aggregate-pilot PILOT_ROOT OUT_MANIFEST OUT_AGGREGATE" in source
    assert '--aggregate-pilot-root "$1"' in source
    assert "--freeze-pilot PILOT_MANIFEST PILOT_AGGREGATE" in source
    assert '--pilot-aggregate "$2"' in source
    assert '--output-manifest "$3"' in source
    assert "formal execution requires stack ID and contract path" in source
    assert '--condition-stack-contract "$5"' in source
    assert "--aggregate-indoor-pilot PILOT_ROOT OUT_MANIFEST OUT_AGGREGATE" in source
    assert '--aggregate-indoor-pilot-root "$1"' in source
    assert "--freeze-indoor-pilot PILOT_MANIFEST PILOT_AGGREGATE" in source
    assert '--indoor-pilot-aggregate "$2"' in source
    assert "--execute-indoor" in source
    assert "indoor execution requires stack ID and contract path" in source
    assert '--indoor-manifest "$manifest"' in source
    assert "--continue-indoor PARENT_MANIFEST SUCCESSOR_MANIFEST SUCCESSOR_OUTPUT_ROOT" in source
    assert '--continue-indoor-parent "$1"' in source
    assert '--continuation-output-manifest "$2"' in source
    assert '--continuation-output-root "$3"' in source


def test_run_experiment_forwards_effective_spawn_exactly_once(tmp_path):
    project = tmp_path / "project"
    scripts = project / "scripts"
    (scripts / "lib").mkdir(parents=True)
    source = (REPO / "scripts" / "run_experiment.sh").read_text()
    source = source.replace(
        "source_ros --require-workspace --require-integration-underlay",
        ": # test harness skips ROS setup",
    )
    runner = scripts / "run_experiment.sh"
    runner.write_text(source, encoding="utf-8")
    runner.chmod(0o755)
    common = (REPO / "scripts/lib/common.sh").read_text()
    common = common.replace(
        "declare -ag ISAAC_NAV_LOCK_FDS=()\nvalidate_runtime_environment\n",
        "declare -ag ISAAC_NAV_LOCK_FDS=()\n",
    )
    (scripts / "lib/common.sh").write_text(common, encoding="utf-8")
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        '#!/usr/bin/env bash\nprintf "%s\\n" "$@" > "${CAPTURE_ARGS}"\n',
        encoding="utf-8",
    )
    fake_ros2.chmod(0o755)
    scenario = tmp_path / "scenario.yaml"
    scenario.write_text("scenario\n", encoding="utf-8")
    output = tmp_path / "output"
    output.mkdir()
    generic_spawn = tmp_path / "generic.spawn.yaml"
    canonical_spawn = tmp_path / "v6.spawn.yaml"
    generic_spawn.write_text("generic\n", encoding="utf-8")
    canonical_spawn.write_text("v6\n", encoding="utf-8")
    capture = tmp_path / "args.txt"
    environment = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "CAPTURE_ARGS": str(capture),
        "ISAAC_NAV_SPAWN_POSES": str(generic_spawn),
    }

    subprocess.run(
        [str(runner), str(scenario), str(output)],
        check=True,
        env=environment,
    )
    default_args = capture.read_text().splitlines()
    assert [arg for arg in default_args if arg.startswith("spawn_poses_file:=")] == [
        f"spawn_poses_file:={generic_spawn}"
    ]

    subprocess.run(
        [
            str(runner),
            str(scenario),
            str(output),
            f"spawn_poses_file:={canonical_spawn}",
        ],
        check=True,
        env=environment,
    )
    override_args = capture.read_text().splitlines()
    assert [arg for arg in override_args if arg.startswith("spawn_poses_file:=")] == [
        f"spawn_poses_file:={canonical_spawn}"
    ]
    assert f"spawn_poses_file:={generic_spawn}" not in override_args

    duplicate = subprocess.run(
        [
            str(runner),
            str(scenario),
            str(output),
            f"spawn_poses_file:={canonical_spawn}",
            f"spawn_poses_file:={generic_spawn}",
        ],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )
    assert duplicate.returncode != 0
    assert "at most once" in duplicate.stderr


def test_formal_aggregate_resumes_after_valid_strict_episode(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    _write_formal_run(
        first, 1, strict_success=True, formal_freeze_digest=campaign.freeze_digest
    )

    aggregate = evaluate_formal_campaign(campaign)
    plans = formal_dispatch_plan(campaign, aggregate)

    first_result = aggregate["conditions"][0]
    assert first_result["strict_successes"] == 1
    assert first_result["valid_episodes"] == 1
    assert first_result["next_run_index"] == 2
    assert plans[0]["condition_id"] == first.condition_id
    assert plans[0]["run_index"] == 2
    assert plans[0]["stack_boundary"] == "hot_reset"
    assert plans[0]["stack_session"] == first.condition_id


@pytest.mark.parametrize(
    ("strict_success", "valid", "expected_status"),
    [
        (False, True, "product_failure"),
        (False, False, "invalid_evidence"),
    ],
)
def test_formal_aggregate_continues_valid_failure_and_stops_invalid(
    tmp_path, strict_success, valid, expected_status
):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    _write_formal_run(
        first,
        1,
        strict_success=strict_success,
        valid=valid,
        formal_freeze_digest=campaign.freeze_digest,
    )

    aggregate = evaluate_formal_campaign(campaign)
    first_result = aggregate["conditions"][0]

    assert first_result["runs"][0]["status"] == expected_status
    if valid:
        assert first_result["next_run_index"] == 2
        assert aggregate["blockers"] == []
    else:
        assert first_result["next_run_index"] is None
        assert aggregate["blockers"] == [
            f"{first.condition_id}:run-1:invalid_evidence"
        ]


def test_formal_aggregate_rejects_changed_stack_session(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    _write_formal_run(
        first, 1, strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
        stack_session_id="a" * 64,
    )
    _write_formal_run(
        first, 2, strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
        stack_session_id="b" * 64,
    )

    aggregate = evaluate_formal_campaign(campaign)

    assert aggregate["conditions"][0]["stack_session_id"] is None
    assert "indoor_static:stack_session_mismatch" in aggregate["blockers"]


@pytest.mark.parametrize("generation", [None, 1, 4])
def test_formal_aggregate_requires_contiguous_reset_generation(
    tmp_path, generation
):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    _write_formal_run(
        first, 1, strict_success=True, formal_freeze_digest=campaign.freeze_digest
    )
    root = _write_formal_run(
        first, 2, strict_success=True, formal_freeze_digest=campaign.freeze_digest
    )
    manifest_path = root / "run_manifest.json"
    episode = json.loads(manifest_path.read_text())
    if generation is None:
        episode["reset_receipt"] = {}
    else:
        episode["reset_receipt"]["generation"] = generation
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    _refresh_checksums(root)

    aggregate = evaluate_formal_campaign(campaign)

    expected = (
        "reset_generation_missing"
        if generation is None
        else "reset_generation_discontinuous"
    )
    assert f"indoor_static:{expected}" in aggregate["blockers"]


def test_formal_checksum_requires_core_and_mcap_coverage(tmp_path):
    root = tmp_path / "run"
    root.mkdir()
    (root / "unrelated.txt").write_text("ok\n", encoding="utf-8")
    _refresh_checksums(root)

    assert not v6_formal_module._checksums_verified(root)


def test_primary_mcap_reader_normalizes_metadata_zero_message_topics(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
    )
    metadata_path = root / "telemetry" / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    metadata["rosbag2_bagfile_information"]["topics_with_message_count"].append({
        "topic_metadata": {
            "name": "/recorded_but_unused",
                "type": "std_msgs/msg/String",
                "serialization_format": "cdr",
                "offered_qos_profiles": [],
                "type_description_hash": "",
        },
        "message_count": 0,
    })
    metadata_path.write_text(yaml.safe_dump(metadata), encoding="utf-8")

    inventory = experiment_runner_module._mcap_inventory_evidence(root)

    assert inventory["passed"] is True
    assert inventory["topic_counts"]["/recorded_but_unused"] == 0


def test_formal_primary_mcap_replays_two_phase_cognitive_contract(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
    )
    summary = json.loads((root / "run_summary.json").read_text())
    episode = json.loads((root / "run_manifest.json").read_text())

    evidence = experiment_runner_module.validate_recorded_run_evidence(
        root,
        summary,
        episode,
        scene="indoor",
        route_guided=True,
        route_prior_required=True,
        expected_leg_count=5,
        cognitive_admission_required=True,
    )

    replay = evidence["cognitive_admission_replay"]
    assert replay["passed"] is True
    assert set(replay["components"]) == {"global_layer", "local_layer"}
    assert replay["postdispatch_critic_evidence"]["passed"] is True
    assert all(
        row["consecutive_healthy_samples"] == 3
        for row in replay["components"].values()
    )
    inventory = evidence["inventory"]["semantic"]["cognitive_admission"]
    assert all(
        source["validation_odom_stamp_ns"]
        - source["source_odom_stamp_ns"] == 998
        for source in inventory["sources"]
    )


def test_formal_binary_mcap_accepts_lagging_critic_coherent_tuple(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
        cognitive_mutation="async_lag",
    )
    summary = json.loads((root / "run_summary.json").read_text())
    episode = json.loads((root / "run_manifest.json").read_text())

    evidence = experiment_runner_module.validate_recorded_run_evidence(
        root,
        summary,
        episode,
        scene="indoor",
        route_guided=True,
        route_prior_required=True,
        expected_leg_count=5,
        cognitive_admission_required=True,
    )

    assert evidence["cognitive_admission_replay"]["passed"] is True


def test_formal_binary_mcap_accepts_current_graph_provenance(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
        cognitive_mutation="graph_provenance_current",
    )
    summary = json.loads((root / "run_summary.json").read_text())
    episode = json.loads((root / "run_manifest.json").read_text())

    evidence = experiment_runner_module.validate_recorded_run_evidence(
        root,
        summary,
        episode,
        scene="indoor",
        route_guided=True,
        route_prior_required=True,
        expected_leg_count=5,
        cognitive_admission_required=True,
    )

    namespaces = evidence["cognitive_admission_replay"]["identity_namespaces"]
    assert namespaces["graph_provenance_status"] == "current"
    assert namespaces["source_physical_graph_id"].endswith(":gvg_v1")


@pytest.mark.parametrize("terminals", ((False,), (True, False)))
def test_product_failure_terminal_false_starts_zero_tail(tmp_path, terminals):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=False,
        formal_freeze_digest=campaign.freeze_digest,
        route_completion_values=terminals,
    )
    summary = json.loads((root / "run_summary.json").read_text())
    episode = json.loads((root / "run_manifest.json").read_text())

    evidence = experiment_runner_module.validate_recorded_run_evidence(
        root,
        summary,
        episode,
        scene="indoor",
        route_guided=True,
        route_prior_required=True,
        expected_leg_count=5,
        require_strict_success=False,
        cognitive_admission_required=True,
    )

    assert evidence["inventory"]["semantic"]["route_complete_false_count"] == 1
    assert evidence["inventory"]["semantic"]["terminal_zero_count"] == 2


@pytest.mark.parametrize(
    "mutation",
    (
        "planning_mismatch", "component_spoof", "offered_false",
        "degraded_prior_sequence", "degraded_prior_untrusted",
        "degraded_prior_ood", "degraded_prior_nonfinite", "degraded_missing",
        "degraded_prior_missing", "degraded_obstacle_missing",
        "age_mismatch", "pre_barrier",
        "generation_mismatch", "session_reuse", "odom_over_100ms",
        "bad_after_latch",
        "source_schema_tamper", "planning_field_tamper",
        "shadow_in_active",
        "post_latch_bad_then_same_seq_healthy",
        "planning_chain_tamper", "critic_receipt_tamper",
        "layer_receipt_tamper",
        "post_latch_bad_planning_then_healthy",
        "post_latch_bad_status_then_healthy",
        "postdispatch_critic_missing", "postdispatch_critic_late",
        "semantic_map_mismatch", "content_map_mismatch",
        "graph_provenance_mismatch",
    ),
)
def test_formal_binary_mcap_replay_rejects_adversarial_cognitive_chain(
    tmp_path, mutation,
):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
        cognitive_mutation=mutation,
    )
    summary = json.loads((root / "run_summary.json").read_text())
    episode = json.loads((root / "run_manifest.json").read_text())

    with pytest.raises(
        experiment_runner_module.ConfigurationError,
        match="recorded_cognitive_admission_invalid",
    ):
        experiment_runner_module.validate_recorded_run_evidence(
            root,
            summary,
            episode,
            scene="indoor",
            route_guided=True,
            route_prior_required=True,
            expected_leg_count=5,
            cognitive_admission_required=True,
        )


@pytest.mark.parametrize(
    "mutation",
    ("required_false", "component_set", "missing_field", "summary_mismatch"),
)
def test_formal_primary_mcap_fails_closed_on_cognitive_receipt_tamper(
    tmp_path, mutation,
):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
    )
    summary = json.loads((root / "run_summary.json").read_text())
    episode = json.loads((root / "run_manifest.json").read_text())
    if mutation == "required_false":
        summary["cognitive_admission_readiness"]["required"] = False
        episode["cognitive_admission_readiness"]["required"] = False
    elif mutation == "component_set":
        summary["cognitive_admission_readiness"]["required_components"].pop()
        episode["cognitive_admission_readiness"]["required_components"].pop()
    elif mutation == "missing_field":
        summary["cognitive_admission_readiness"]["components"]["global_layer"][
            "latest"
        ].pop("fallback_reason")
        episode["cognitive_admission_readiness"]["components"]["global_layer"][
            "latest"
        ].pop("fallback_reason")
    else:
        summary["cognitive_admission_readiness"]["components"]["global_layer"][
            "latest"
        ]["message_age_ms"] = 26.0

    with pytest.raises(
        experiment_runner_module.ConfigurationError,
        match="cognitive_admission",
    ):
        experiment_runner_module.validate_recorded_run_evidence(
            root,
            summary,
            episode,
            scene="indoor",
            route_guided=True,
            route_prior_required=True,
            expected_leg_count=5,
            cognitive_admission_required=True,
        )


def test_formal_checksum_rejects_unlisted_regular_file(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
    )
    (root / "unlisted.txt").write_text("not in checksum\n", encoding="utf-8")

    assert not v6_formal_module._checksums_verified(root)


def test_formal_aggregate_requires_final_metric_gate(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    root = _write_formal_run(
        first, 1, strict_success=True, formal_freeze_digest=campaign.freeze_digest
    )
    summary_path = root / "run_summary.json"
    summary = json.loads(summary_path.read_text())
    summary["final_trial_metric_gate"]["passed"] = False
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    _refresh_checksums(root)

    aggregate = evaluate_formal_campaign(campaign)

    assert aggregate["conditions"][0]["runs"][0]["status"] == "invalid_evidence"


def test_formal_aggregate_rejects_run_freeze_digest_mismatch(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest="0" * 64,
    )

    aggregate = evaluate_formal_campaign(campaign)

    assert aggregate["conditions"][0]["runs"][0]["status"] == "invalid_evidence"


def test_formal_condition_thresholds_pass_only_when_authorized(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    for condition in campaign.conditions:
        required = v6_formal_module.FORMAL_SUCCESS_THRESHOLDS[
            condition.condition_id
        ]
        for run_index in range(1, 21):
            _write_formal_run(
                condition,
                run_index,
                strict_success=run_index <= required,
                formal_freeze_digest=campaign.freeze_digest,
            )

    aggregate = evaluate_formal_campaign(campaign)

    assert aggregate["strict_successes"] == 110
    assert all(row["qualification"] == "PASS" for row in aggregate["conditions"])
    assert aggregate["execution_authorization"] == "NOT_AUTHORIZED"
    assert aggregate["formal_qualification"] == "INCOMPLETE"
    authorized = evaluate_formal_campaign(
        replace(campaign, authorization="AUTHORIZED")
    )
    assert authorized["formal_qualification"] == "PASS"


@pytest.mark.parametrize(
    ("condition_index", "failure_count"),
    ((0, 2), (2, 3), (3, 3)),
    ids=("static-second", "dynamic-third", "appearance-third"),
)
def test_formal_early_unreachable_uses_condition_failure_budget(
    tmp_path, condition_index, failure_count,
):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    condition = campaign.conditions[condition_index]
    for run_index in range(1, failure_count + 1):
        _write_formal_run(
            condition,
            run_index,
            strict_success=False,
            formal_freeze_digest=campaign.freeze_digest,
        )

    aggregate = evaluate_formal_campaign(campaign)
    row = aggregate["conditions"][condition_index]

    assert row["product_failures"] == failure_count
    assert row["next_run_index"] is None
    assert row["blockers"] == ["early_fail_unreachable"]


def test_runtime_contract_rejects_nonbaseline_navigation_features(tmp_path):
    for key, value in (
        ("recovery_enabled", True),
        ("module2_navigation_write_enabled", True),
        ("cognitive_place_graph_enabled", True),
        ("route_backend", "primary"),
        ("low_obstacles_enabled", True),
        ("dynamic_actors_enabled", True),
    ):
        raw = _raw()
        raw["runtime"][key] = value
        with pytest.raises(V6ContractError, match=f"runtime.{key}"):
            load_manifest(_write_manifest(tmp_path, raw))


def test_mission_legs_are_xy_only_and_schedule_is_separate(tmp_path):
    manifest = load_manifest(MANIFEST)
    assert all(
        set(row) == {"id", "frame_id", "x", "y"}
        for row in manifest.raw["mission"]["legs"]
    )

    raw = _raw()
    raw["mission"]["legs"][0]["yaw_deg"] = 45.0
    with pytest.raises(V6ContractError, match="only id/frame_id/x/y"):
        load_manifest(_write_manifest(tmp_path, raw))


@pytest.mark.parametrize(
    "schedule, message",
    [
        ([{"leg_id": "missing", "group": "actor_a"}], "not a mission leg"),
        (
            [
                {"leg_id": "G2", "group": "actor_a"},
                {"leg_id": "G2", "group": "actor_b"},
            ],
            "must be unique",
        ),
        (
            [
                {"leg_id": "G2", "group": "actor_a"},
                {"leg_id": "G3", "group": "actor_a"},
            ],
            "must be unique",
        ),
    ],
)
def test_dynamic_schedule_validates_leg_and_uniqueness(tmp_path, schedule, message):
    raw = _raw()
    raw["dynamic_schedule"] = schedule
    with pytest.raises(V6ContractError, match=message):
        load_manifest(_write_manifest(tmp_path, raw))


def test_dynamic_schedule_parses_independently_from_xy_goals(tmp_path):
    raw = _raw()
    raw["dynamic_schedule"] = [{"leg_id": "G3", "group": "actor_a"}]
    manifest = load_manifest(_write_manifest(tmp_path, raw))
    assert manifest.dynamic_schedule == (DynamicScheduleEntry("G3", "actor_a"),)
    assert not hasattr(manifest.mission_legs[1], "dynamic_trigger_group")


def test_baseline_readiness_has_no_candidate_bridge_or_prior_dependency():
    fields = set(ReadinessFacts.__dataclass_fields__)
    assert fields == {
        "reset_service_ready",
        "reset_event_publisher_ready",
        "reset_subscriber_roster_ready",
        "route_goal_subscriber_ready",
        "clock_seen",
        "scan_seen",
        "map_seen",
        "navigation_graph_seen",
        "estimated_odom_seen",
    }
    assert not any("module2" in topic for topic in DISPATCH_SUBSCRIPTION_TOPICS)
    assert "/bio_nav/localization/candidates" not in DISPATCH_SUBSCRIPTION_TOPICS


def test_goal_requires_fresh_initialpose_then_amcl_nav_tf_and_gate_release():
    guard = EpisodeGuard(mission_leg_ids=("G2",))
    guard.arm_reset(ready_facts())
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_receipt_generation(7)
    guard.record_reset_event()
    guard.record_amcl(99)
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_gate_status(7, False)
    assert not guard.goal_ready

    guard.record_initialpose(100)
    guard.record_amcl(100)
    assert not guard.goal_ready
    guard.record_amcl(101)
    assert guard.localization_ready
    assert guard.goal_ready


def test_stale_reset_gate_release_does_not_authorize_goal():
    guard = EpisodeGuard(mission_leg_ids=("G2",))
    guard.arm_reset(ready_facts())
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_event()
    guard.record_initialpose(100)
    guard.record_amcl(101)
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_receipt_generation(4)
    guard.record_reset_gate_status(3, False)
    assert not guard.goal_ready
    guard.record_reset_gate_status(4, False)
    assert guard.goal_ready


def _tf_message(parent: str, child: str):
    return SimpleNamespace(
        transforms=[
            SimpleNamespace(
                header=SimpleNamespace(frame_id=parent),
                child_frame_id=child,
            )
        ]
    )


def _stamped_pose(stamp_ns: int):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            )
        )
    )


def _tf_epoch_adapter() -> V6FormalNode:
    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter.guard = EpisodeGuard(mission_leg_ids=("G2",))
    adapter.map_odom_tf_seen = False
    adapter.odom_base_tf_seen = False
    adapter._capture = lambda *_args, **_kwargs: None
    return adapter


def _record_adapter_navigation_ready(adapter: V6FormalNode) -> None:
    adapter.guard.record_navigation_ready(
        nav2_active=True,
        tf_active=adapter.map_odom_tf_seen and adapter.odom_base_tf_seen,
    )


def test_reset_epoch_requires_both_tf_edges_to_be_observed_again():
    adapter = _tf_epoch_adapter()
    adapter._tf(_tf_message("map", "odom"))
    adapter._tf(_tf_message("odom", "base_link"))
    assert adapter.map_odom_tf_seen and adapter.odom_base_tf_seen

    adapter.guard.arm_reset(ready_facts())
    adapter.guard.record_reset_call()
    adapter.guard.record_reset_response(True)
    adapter._reset_gate_status(
        SimpleNamespace(data='{"generation":7,"held":false}')
    )
    adapter._reset_event(SimpleNamespace())
    adapter.guard.record_reset_receipt_generation(7)
    adapter._initialpose(_stamped_pose(100))
    adapter._amcl_pose(_stamped_pose(101))

    _record_adapter_navigation_ready(adapter)
    assert not adapter.guard.goal_ready
    assert not adapter.guard.tf_active

    adapter._tf(_tf_message("map", "odom"))
    _record_adapter_navigation_ready(adapter)
    assert not adapter.guard.goal_ready
    assert not adapter.guard.tf_active

    adapter._tf(_tf_message("odom", "base_footprint"))
    _record_adapter_navigation_ready(adapter)
    assert adapter.guard.tf_active
    assert adapter.guard.goal_ready


def test_invalid_reset_events_do_not_rewrite_tf_epoch_observations():
    out_of_order = _tf_epoch_adapter()
    out_of_order._tf(_tf_message("map", "odom"))
    out_of_order._reset_event(SimpleNamespace())
    assert out_of_order.guard.stop_reason == "reset_event_without_call"
    assert out_of_order.map_odom_tf_seen

    duplicate = _tf_epoch_adapter()
    duplicate.guard.arm_reset(ready_facts())
    duplicate.guard.record_reset_call()
    duplicate.guard.record_reset_response(True)
    duplicate._reset_event(SimpleNamespace())
    duplicate._tf(_tf_message("map", "odom"))
    duplicate._reset_event(SimpleNamespace())
    assert duplicate.guard.stop_reason == "second_reset_event"
    assert duplicate.map_odom_tf_seen
    duplicate._tf(_tf_message("odom", "base_link"))
    _record_adapter_navigation_ready(duplicate)
    assert not duplicate.guard.goal_ready


def test_reset_is_exactly_once():
    guard = EpisodeGuard()
    guard.arm_reset(ready_facts())
    guard.record_reset_call()
    guard.record_reset_response(None)
    assert guard.stop_reason == "reset_response_unknown"
    with pytest.raises(V6ContractError, match="reset_retry_forbidden"):
        guard.record_reset_call()


def test_multileg_order_and_xy_goal_message():
    guard = ready_guard("G2", "G3")
    guard.record_goal_publication("G2")
    guard.record_route_progress()
    guard.record_route_completion(True)
    with pytest.raises(V6ContractError, match="mission_leg_order"):
        guard.record_goal_publication("G4")

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
    goal = adapter._goal_message(MissionLeg("G2", "map", 1.0, 2.0))
    assert (goal.pose.position.x, goal.pose.position.y) == (1.0, 2.0)
    assert goal.pose.orientation.z == 0.0
    assert goal.pose.orientation.w == 1.0


def _mission_leg_adapter(*, spin_result: bool, route_success: bool):
    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter.guard = ready_guard("G2")
    adapter.canonical_route_count = 0
    adapter.route_goal_results = []
    adapter._navigation_terminal_observed = False
    events = []
    adapter._call_dynamic_action = lambda group, action, timeout: (
        events.append((action, group)) or True
    )
    adapter._goal_message = lambda leg: f"goal:{leg.goal_id}"
    adapter.route_goal_publisher = SimpleNamespace(
        publish=lambda message: events.append(("publish", message))
    )
    adapter._write = lambda event, **payload: events.append((event, payload))

    def spin_until(_predicate, _timeout):
        if spin_result:
            adapter.guard.record_route_progress()
            adapter.guard.record_route_completion(route_success)
            adapter.canonical_route_count += 1
        return spin_result

    adapter._spin_until = spin_until
    return adapter, events


@pytest.mark.parametrize(
    "spin_result, route_success, expected_state",
    [(True, True, "SUCCEEDED"), (True, False, "FAILED"), (False, False, "STOP")],
)
def test_dynamic_action_triggers_before_goal_and_completes_after_leg_terminal(
    spin_result, route_success, expected_state
):
    adapter, events = _mission_leg_adapter(
        spin_result=spin_result, route_success=route_success
    )
    adapter._run_mission_leg(
        index=0,
        leg=MissionLeg("G2", "map", 1.0, 2.0),
        dynamic_group="actor_a",
        reset_timeout_sec=1.0,
        navigation_timeout_sec=2.0,
    )

    labels = [row[0] for row in events]
    assert labels.index("trigger") < labels.index("publish")
    assert labels.index("publish") < labels.index("complete")
    assert adapter.guard.state == expected_state


def test_dynamic_action_ledger_is_exactly_once():
    ledger = DynamicActionLedger()
    ledger.claim("actor_a", "trigger")
    ledger.claim("actor_a", "complete")
    with pytest.raises(V6ContractError, match="retry forbidden"):
        ledger.claim("actor_a", "complete")
    with pytest.raises(V6ContractError, match="completion before trigger"):
        DynamicActionLedger().claim("actor_b", "complete")


class _Twist:
    def __init__(self, *, nonzero: bool = False):
        self.linear = SimpleNamespace(x=0.2 if nonzero else 0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class _CancelGoal:
    class Request:
        pass


def _terminal_adapter(
    monkeypatch,
    *,
    state: str,
    downstream_events: tuple[tuple[float, bool], ...],
    cancel_done_after: float | None = 0.0,
    timeout_sec: float = 0.65,
):
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(v6_formal_module.time, "monotonic", lambda: clock.now)

    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter.guard = ready_guard("G2")
    adapter.guard.record_goal_publication("G2")
    adapter.guard.record_route_progress()
    if state == "SUCCEEDED":
        adapter.guard.record_route_completion(True)
    elif state == "FAILED":
        adapter.guard.record_route_completion(False)
    else:
        adapter.guard.stop("collision")
    adapter._terminal_cancel_requested = False
    adapter._terminal_cancel_future = None
    adapter._terminal_started_monotonic = None
    adapter._navigation_terminal_observed = state in {"SUCCEEDED", "FAILED"}
    adapter._terminal_zero_settled = False
    adapter._terminal_zero_confirmed = False
    adapter._terminal_zero_reason = "not_required"
    adapter._terminal_topic_summary = {}
    adapter._cmd_vel_sim_last_receive_monotonic = None
    adapter._cmd_vel_sim_last_nonzero_monotonic = None
    adapter._cmd_vel_sim_zero_stamps = deque()
    adapter._cmd_window = deque()
    adapter._types = {"CancelGoal": _CancelGoal, "Twist": _Twist}
    adapter.node = SimpleNamespace()
    adapter.TERMINAL_ZERO_TIMEOUT_SEC = timeout_sec
    adapter.TERMINAL_ZERO_PERIOD_SEC = 0.05
    adapter.TERMINAL_ZERO_QUIET_SEC = 0.15
    adapter.TERMINAL_ZERO_CADENCE_TOLERANCE_SEC = 0.10
    adapter._capture = lambda *_args, **_kwargs: None

    lifecycle = []
    adapter.navigate_cancel_client = SimpleNamespace(
        call_async=lambda request: (
            lifecycle.append((clock.now, "cancel"))
            or SimpleNamespace(
                done=lambda: cancel_done_after is not None
                and clock.now >= 10.0 + cancel_done_after
            )
        )
    )
    adapter.terminal_zero_publisher = SimpleNamespace(
        publish=lambda message: lifecycle.append((clock.now, "zero_publish"))
    )
    adapter._write = lambda event, **payload: lifecycle.append((clock.now, event))
    pending = list(downstream_events)

    def spin_once(_node, *, timeout_sec):
        clock.now += timeout_sec
        while pending and clock.now >= 10.0 + pending[0][0]:
            _offset, nonzero = pending.pop(0)
            adapter._track_command("/cmd_vel_sim", _Twist(nonzero=nonzero))

    adapter._rclpy = SimpleNamespace(ok=lambda: True, spin_once=spin_once)
    return adapter, clock, lifecycle


def test_success_terminal_settle_publishes_20hz_without_cancel(monkeypatch):
    adapter, _clock, lifecycle = _terminal_adapter(
        monkeypatch,
        state="SUCCEEDED",
        downstream_events=((0.05, False), (0.20, False)),
    )
    adapter._start_terminal_settle(cancel_navigation=False, reason="SUCCEEDED")

    assert adapter._settle_terminal_zero()
    labels = [label for _stamp, label in lifecycle]
    assert "cancel" not in labels
    publish_stamps = [stamp for stamp, label in lifecycle if label == "zero_publish"]
    assert len(publish_stamps) >= 4
    assert all(
        later - earlier == pytest.approx(0.05)
        for earlier, later in zip(publish_stamps, publish_stamps[1:])
    )


@pytest.mark.parametrize(
    "state, cancel_navigation",
    [("SUCCEEDED", False), ("FAILED", True), ("STOP", True)],
)
def test_result_routes_every_terminal_state_through_zero_settle(
    state, cancel_navigation
):
    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter.guard = EpisodeGuard(state=state, stop_reason="failure" if state != "SUCCEEDED" else "")
    adapter.qualification = ENGINEERING_PILOT
    adapter.reset_receipt = None
    adapter.route_goal_results = []
    adapter.dynamic_actions = DynamicActionLedger()
    adapter.obstacle_state_messages = []
    adapter.collision = False
    adapter._terminal_zero_confirmed = True
    adapter._terminal_zero_reason = "terminal_zero_confirmed"
    calls = []
    adapter._start_terminal_settle = lambda **kwargs: calls.append(
        ("start", kwargs)
    )
    adapter._settle_terminal_zero = lambda: calls.append(("settle", {})) or True
    adapter._write = lambda *_args, **_kwargs: None

    result = adapter.result()

    assert result["state"] == state
    assert calls == [
        (
            "start",
            {
                "cancel_navigation": cancel_navigation,
                "reason": "failure" if state != "SUCCEEDED" else "SUCCEEDED",
            },
        ),
        ("settle", {}),
    ]


def test_success_terminal_settle_timeout_becomes_stop(monkeypatch):
    adapter, _clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="SUCCEEDED",
        downstream_events=((0.05, False),),
        timeout_sec=0.35,
    )
    adapter._start_terminal_settle(cancel_navigation=False, reason="SUCCEEDED")

    assert not adapter._settle_terminal_zero()
    assert adapter.guard.state == "STOP"
    assert adapter.guard.stop_reason == "terminal_zero_timeout_after_success"


def test_failed_terminal_cancels_before_zero_settle(monkeypatch):
    adapter, _clock, lifecycle = _terminal_adapter(
        monkeypatch,
        state="FAILED",
        downstream_events=((0.05, False), (0.20, False)),
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="route_failed")

    assert adapter._settle_terminal_zero()
    labels = [label for _stamp, label in lifecycle]
    assert labels.index("cancel") < labels.index("zero_publish")


def test_single_downstream_zero_plus_silence_never_passes(monkeypatch):
    adapter, _clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="STOP",
        downstream_events=((0.05, False),),
        timeout_sec=0.35,
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="collision")
    assert not adapter._settle_terminal_zero()
    assert not adapter._terminal_zero_confirmed


def test_repeated_downstream_zero_covers_quiet_window_and_passes(monkeypatch):
    adapter, clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="STOP",
        downstream_events=((0.05, False), (0.22, False)),
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="collision")
    assert adapter._settle_terminal_zero()
    assert clock.now <= 10.32


def test_repeated_zero_then_silence_fails_latest_cadence_check(monkeypatch):
    adapter, _clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="STOP",
        downstream_events=((0.05, False), (0.22, False)),
        cancel_done_after=0.45,
        timeout_sec=0.60,
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="collision")
    assert not adapter._settle_terminal_zero()


def test_downstream_nonzero_resets_zero_window(monkeypatch):
    adapter, clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="STOP",
        downstream_events=(
            (0.05, False),
            (0.15, False),
            (0.18, True),
            (0.30, False),
            (0.48, False),
        ),
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="collision")
    assert adapter._settle_terminal_zero()
    assert clock.now >= 10.48


def test_terminal_capture_writer_stall_cannot_starve_zero_observation(monkeypatch):
    adapter, clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="SUCCEEDED",
        downstream_events=(
            (0.05, False),
            (0.10, False),
            (0.12, True),
            (0.20, False),
            (0.38, False),
        ),
    )
    adapter._capture = V6FormalNode._capture.__get__(adapter, V6FormalNode)
    writes = []

    def stalled_write(event, **payload):
        writes.append((event, payload, adapter._terminal_zero_confirmed))
        clock.now += 1.0

    adapter._write = stalled_write
    adapter._start_terminal_settle(cancel_navigation=False, reason="SUCCEEDED")

    assert adapter._settle_terminal_zero()
    assert adapter._terminal_zero_confirmed
    summary_event, payload, confirmed_before_stall = writes[0]
    assert summary_event == "terminal_topic_summary"
    assert confirmed_before_stall
    assert payload["topics"]["/cmd_vel_sim"]["count"] == 5
    assert payload["topics"]["/cmd_vel_sim"]["last_message"]["linear"]["x"] == 0.0
    assert writes[-1][0] == "terminal_zero_confirmed"


def test_command_observation_and_terminal_publish_qos_are_depth_one():
    source = (PACKAGE / "robot_experiments" / "v6_formal.py").read_text()
    observation = source[
        source.index("command_observation_qos = QoSProfile("):
        source.index("terminal_zero_qos = QoSProfile(")
    ]
    assert "depth=1" in observation
    assert "ReliabilityPolicy.RELIABLE" in observation
    assert "DurabilityPolicy.VOLATILE" in observation
    for topic in ("/cmd_vel", "/cmd_vel_nav", "/cmd_vel_sim"):
        assert f'lambda m: self._track_command("{topic}", m), command_observation_qos' in source

    terminal = source[
        source.index("terminal_zero_qos = QoSProfile("):
        source.index("self.reset_client =")
    ]
    assert "depth=1" in terminal
    assert 'Twist, "/cmd_vel_nav", terminal_zero_qos' in terminal
    assert 'create_publisher(\n            Twist, "/cmd_vel_sim"' not in source


def test_reset_stop_gate_and_omnigraph_command_queues_remain_depth_one():
    gate = (REPO / "isaac_sim/src/bridge/reset_stop_gate.py").read_text()
    command_qos = gate[
        gate.index("command_qos = QoSProfile("):
        gate.index("status_qos = QoSProfile(")
    ]
    assert "HistoryPolicy.KEEP_LAST" in command_qos
    assert "depth=1" in command_qos

    graph = (REPO / "isaac_sim/graphs/control_graph.py").read_text()
    assert '("SubscribeTwist.inputs:queueSize", 1)' in graph


def test_dispatcher_ground_truth_firewall():
    assert DISPATCH_SUBSCRIPTION_TOPICS
    assert not [
        topic for topic in DISPATCH_SUBSCRIPTION_TOPICS
        if topic.startswith("/ground_truth/")
    ]
