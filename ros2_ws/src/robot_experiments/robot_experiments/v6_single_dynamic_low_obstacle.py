"""Single dynamic low-obstacle M1/M3 experiment with an M2 fallback."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import yaml

from robot_experiments import v6_low_obstacle_causal as causal


SCHEMA_VERSION = "bio_nav_v6_single_dynamic_low_obstacle_v1"
DYNAMIC_STATE_TOPIC = "/experiment/obstacles/state"
ARM_MODES = {"M1": "M1", "M3": "M3", "M2-fallback": "M2"}
VISIBLE_ACTOR_STATES = {"moving", "dwell", "clearing", "parked", "safety_yield"}
FORBIDDEN_ACTOR_STATES = {"safety_yield", "guard_aborted"}


class DynamicLowObstacleError(RuntimeError):
    pass


@dataclass(frozen=True)
class DynamicLowObstacleExperiment:
    path: Path
    module3_root: Path
    identity: Mapping[str, Any]
    criteria: Mapping[str, Any]
    arm_modes: Mapping[str, str]
    actor: Mapping[str, Any]


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise DynamicLowObstacleError(f"{name} must be a mapping")
    return value


def _resolve(
    value: Any, *, config_path: Path, module3_root: Path, name: str
) -> Path:
    try:
        return causal._resolve_phase_f_resource(
            value, manifest_path=config_path, module3_root=module3_root
        )
    except causal.CausalContractError as exc:
        raise DynamicLowObstacleError(f"{name}: {exc}") from exc


def load_experiment(path: str | Path) -> DynamicLowObstacleExperiment:
    config_path = Path(path).expanduser().resolve()
    raw = _mapping(yaml.safe_load(config_path.read_text(encoding="utf-8")), "config")
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise DynamicLowObstacleError(f"schema_version must be {SCHEMA_VERSION}")
    module3_root = causal._source_module3_root(config_path)
    if module3_root is None:
        configured_root = os.environ.get(causal.MODULE3_ROOT_ENV, "")
        module3_root = Path(configured_root).expanduser().resolve() if configured_root else None
    if module3_root is None or not module3_root.is_dir():
        raise DynamicLowObstacleError(
            f"{causal.MODULE3_ROOT_ENV} must identify the Module3 source root"
        )

    identity = dict(_mapping(raw.get("identity"), "identity"))
    fixed = {
        "scene_id": "v6_kujiale_single_dynamic_low_obstacle",
        "seed": 8601,
        "start": {"id": "G1", "reset_pose_name": "long_route_start_g1"},
        "dynamic_actors_enabled": True,
        "dynamic_case_id": "single_dynamic_low_box",
        "dynamic_variant_id": "v1",
        "actor_id": "v6_dynamic_low_box_solo",
        "trigger_group": "G2",
        "direct_rgbd_costmap_enabled": False,
        "module1_amcl_prior_enabled": False,
        "cognitive_place_graph_enabled": False,
    }
    for key, expected in fixed.items():
        if identity.get(key) != expected:
            raise DynamicLowObstacleError(f"identity.{key} must be {expected!r}")
    goal = _mapping(identity.get("goal"), "identity.goal")
    if goal.get("id") != "G2" or goal.get("frame_id") != "map":
        raise DynamicLowObstacleError("identity.goal must be the map-frame G2 goal")
    if int(identity.get("ros_domain_id", -1)) not in range(233):
        raise DynamicLowObstacleError("identity.ros_domain_id must be in [0,232]")
    if float(identity.get("timeout_sec", 0.0)) <= 0.0:
        raise DynamicLowObstacleError("identity.timeout_sec must be positive")

    for key in (
        "scene_asset", "occupancy_map", "spawn_manifest", "route_graph",
        "obstacle_config", "navigation_overlay",
    ):
        identity[key] = str(
            _resolve(identity.get(key), config_path=config_path,
                     module3_root=module3_root, name=f"identity.{key}")
        )
    identity["obstacle_manifest"] = identity["obstacle_config"]

    try:
        if str(module3_root) not in sys.path:
            sys.path.insert(0, str(module3_root))
        from isaac_sim.src.experiment.scenario import load_dynamic_scenario
        scenario = load_dynamic_scenario(identity["obstacle_config"])
    except (ImportError, OSError, ValueError, yaml.YAMLError) as exc:
        raise DynamicLowObstacleError(f"invalid dynamic obstacle config: {exc}") from exc
    if scenario.enabled is not False or scenario.coordinate_frame != "map":
        raise DynamicLowObstacleError("dynamic config must be default-off and authored in map")
    selected = scenario.selected_cases(str(identity["dynamic_case_id"]))
    if len(selected) != 1:
        raise DynamicLowObstacleError("dynamic experiment must select exactly one actor")
    case = selected[0]
    variant = case.variant(str(identity["dynamic_variant_id"]))
    expected = {
        "id": "v6_dynamic_low_box_solo",
        "size": (0.30, 0.30, 0.16),
        "start": (-1.25, -0.35, 0.08),
        "end": (-0.45, -0.35, 0.08),
        "speed": 0.25,
        "max_acceleration": 0.50,
    }
    actual = {
        "id": case.obstacle.obstacle_id,
        "size": tuple(case.obstacle.size),
        "start": tuple(case.waypoints[0]),
        "end": tuple(case.waypoints[-1]),
        "speed": case.obstacle.speed,
        "max_acceleration": case.max_acceleration,
    }
    if actual != expected or case.obstacle.post_motion != "park":
        raise DynamicLowObstacleError("single dynamic actor geometry/motion differs from the frozen design")
    gate = case.gate
    if (
        case.trigger_group != "G2" or gate.axis != "y"
        or not math.isclose(gate.threshold, -2.60)
        or gate.direction != "positive"
        or not math.isclose(gate.min_speed_mps, 0.20)
        or tuple(gate.x_range or ()) != (0.30, 0.80)
        or variant.seed != 8601
    ):
        raise DynamicLowObstacleError("single dynamic actor gate/variant differs from the frozen design")

    raw_arms = _mapping(raw.get("arms"), "arms")
    if set(raw_arms) != set(ARM_MODES):
        raise DynamicLowObstacleError("arms must be exactly M1, M3, and M2-fallback")
    for label, mode in ARM_MODES.items():
        row = _mapping(raw_arms[label], f"arms.{label}")
        if row.get("module3_mode") != mode:
            raise DynamicLowObstacleError(f"arms.{label}.module3_mode must be {mode}")

    criteria = dict(_mapping(raw.get("criteria"), "criteria"))
    for key in (
        "typed_spatial_match_tolerance_m", "minimum_actor_position_span_m",
        "minimum_candidate_position_span_m", "old_position_clear_timeout_sec",
        "spin_duration_max_sec",
    ):
        if float(criteria.get(key, 0.0)) <= 0.0:
            raise DynamicLowObstacleError(f"criteria.{key} must be positive")
    actor = {
        "id": actual["id"],
        "center": list(actual["start"][:2]),
        "start": list(actual["start"]),
        "end": list(actual["end"]),
        "size": list(actual["size"]),
        "z_bounds": [0.0, 0.16],
        "robot_radius_m": 0.33,
    }
    return DynamicLowObstacleExperiment(
        config_path, module3_root, identity, criteria, dict(ARM_MODES), actor
    )


def _causal_manifest(
    experiment: DynamicLowObstacleExperiment, arm_label: str
) -> tuple[causal.CausalManifest, causal.RunContract]:
    if arm_label not in experiment.arm_modes:
        raise DynamicLowObstacleError(f"unsupported arm: {arm_label}")
    mode = experiment.arm_modes[arm_label]
    arm = causal.ArmContract(
        name=mode,
        module2_uds_enabled=True,
        integration_bridge_enabled=True,
        integration_process_required=True,
        localization_contract="same_estimated_autonomy",
        module3_mode=mode,
        obstacle_layer_mode="shadow" if mode == "M1" else "active",
        critic_mode="active" if mode == "M3" else ("shadow" if mode == "M1" else "off"),
    )
    run = causal.RunContract(
        run_id=f"v6-dynamic-low-{arm_label.lower().replace('-', '_')}",
        repeat=1,
        arm=mode,
    )
    manifest = causal.CausalManifest(
        path=experiment.path,
        module3_root=experiment.module3_root,
        identity=experiment.identity,
        localization_contract={
            "startup_profile": "estimated_autonomy",
            "preserve_when_module2_disabled": True,
        },
        freshness={
            "typed_obstacle_ttl_sec": 0.5,
            "post_producer_stop_observation_margin_sec": 1.0,
        },
        criteria=experiment.criteria,
        arms={mode: arm},
        runs=(run,),
    )
    return manifest, run


def build_plan(
    experiment: DynamicLowObstacleExperiment,
    arm_label: str,
    output_root: str | Path,
) -> dict[str, Any]:
    manifest, run = _causal_manifest(experiment, arm_label)
    run_dir = Path(output_root).expanduser().resolve() / run.run_id
    socket_path = f"/tmp/bnv6dyn-{run.arm.lower()}-{os.getpid()}.sock"
    root = experiment.module3_root
    commands = {
        "scene": (
            str(root / "scripts/run_v6_r5_phase_b_kujiale.sh"),
            "--domain", str(experiment.identity["ros_domain_id"]),
            "isaac", "--dynamic-obstacle-config",
            str(experiment.identity["obstacle_config"]), "--dynamic-obstacles",
        ),
        "stack": (
            str(root / "scripts/run_v6_low_obstacle_phase_f_stack.sh"),
            run.arm, "--domain", str(experiment.identity["ros_domain_id"]),
            "--run-dir", str(run_dir), "--socket", socket_path,
        ),
        "episode": (
            "ros2", "run", "robot_experiments", "v6_single_dynamic_low_obstacle",
            "dispatch-episode", "--config", str(experiment.path),
            "--arm", arm_label, "--output-jsonl", str(run_dir / "episode.jsonl"),
        ),
        "recorder": _rosbag_command(experiment, run_dir / "bag"),
    }
    return {
        "qualification": "ENGINEERING_NOT_RUN",
        "arm_label": arm_label,
        "module3_mode": run.arm,
        "fallback_only": arm_label == "M2-fallback",
        "recommended_order": ["M1", "M3"],
        "fallback_arm": "M2-fallback",
        "run_id": run.run_id,
        "run_directory": str(run_dir),
        "module2_socket": socket_path,
        "commands": commands,
        "dynamic_actor": dict(experiment.actor),
        "trigger": {
            "group": "G2", "axis": "y", "threshold": -2.60,
            "direction": "positive", "minimum_speed_mps": 0.20,
            "x_range": [0.30, 0.80],
        },
        "isolation": {
            "direct_rgbd_costmap_enabled": False,
            "module1_amcl_prior_enabled": False,
            "cognitive_place_graph_enabled": False,
            "producer_fault_injection": False,
        },
    }


def _rosbag_topics() -> tuple[str, ...]:
    return tuple(dict.fromkeys((
        *causal.DISPATCHER_TOPICS,
        *causal.PASSIVE_EVALUATOR_TOPICS,
        *causal.ISOLATION_AUDIT_TOPICS,
        DYNAMIC_STATE_TOPIC,
    )))


def _rosbag_command(
    experiment: DynamicLowObstacleExperiment, bag_dir: Path
) -> tuple[str, ...]:
    qos = (
        experiment.module3_root / "ros2_ws/src/robot_experiments/config"
        / causal.PHASE_F_QOS_CONFIG
    )
    return (
        "ros2", "bag", "record", "--storage", "mcap",
        "--qos-profile-overrides-path", str(qos),
        "--include-unpublished-topics", "--output", str(bag_dir),
        *_rosbag_topics(),
    )


def actor_timeline(
    records: Sequence[causal.RecordedMessage], actor_id: str
) -> list[dict[str, Any]]:
    timeline: list[dict[str, Any]] = []
    for record in records:
        if record.topic != DYNAMIC_STATE_TOPIC:
            continue
        try:
            document = json.loads(str(causal._field(record.message, "data", "")))
        except (json.JSONDecodeError, TypeError, ValueError):
            continue
        obstacles = document.get("obstacles", []) if isinstance(document, Mapping) else []
        for obstacle in obstacles if isinstance(obstacles, list) else []:
            if not isinstance(obstacle, Mapping) or obstacle.get("id") != actor_id:
                continue
            position = obstacle.get("position")
            size = obstacle.get("size")
            if (
                obstacle.get("position_frame") != "map"
                or not isinstance(position, list) or len(position) != 3
                or not isinstance(size, list) or len(size) != 3
            ):
                continue
            events = document.get("events", [])
            timeline.append({
                "stamp_ns": causal._message_stamp_ns(record),
                "state": str(obstacle.get("state", "")),
                "position": [float(value) for value in position],
                "size": [float(value) for value in size],
                "velocity_mps": float(obstacle.get("velocity_mps", 0.0)),
                "events": [
                    dict(value) for value in events
                    if isinstance(value, Mapping)
                    and value.get("obstacle_id") in {None, actor_id}
                ] if isinstance(events, list) else [],
            })
    timeline.sort(key=lambda row: row["stamp_ns"])
    return timeline


def _actor_resolver(
    timeline: Sequence[Mapping[str, Any]], fallback: Mapping[str, Any]
):
    def resolve(stamp_ns: int) -> Mapping[str, Any] | None:
        if not timeline:
            return None
        row = min(timeline, key=lambda value: abs(int(value["stamp_ns"]) - stamp_ns))
        if abs(int(row["stamp_ns"]) - stamp_ns) > causal.DEFAULT_SYNC_TOLERANCE_NS:
            return None
        if row["state"] not in VISIBLE_ACTOR_STATES:
            return None
        position = row["position"]
        size = row["size"]
        return {
            "id": fallback["id"],
            "center": [float(position[0]), float(position[1])],
            "size": [float(value) for value in size],
            "z_bounds": [
                float(position[2]) - 0.5 * float(size[2]),
                float(position[2]) + 0.5 * float(size[2]),
            ],
            "robot_radius_m": float(fallback["robot_radius_m"]),
        }
    return resolve


def _costmap_max(message: Any, center: Sequence[float], size: Sequence[float]) -> int | None:
    try:
        width = int(causal._field(message, "info.width"))
        height = int(causal._field(message, "info.height"))
        resolution = float(causal._field(message, "info.resolution"))
        origin_x = float(causal._field(message, "info.origin.position.x"))
        origin_y = float(causal._field(message, "info.origin.position.y"))
        data = causal._field(message, "data")
    except (TypeError, ValueError):
        return None
    if width <= 0 or height <= 0 or resolution <= 0.0 or data is None:
        return None
    values: list[int] = []
    half_x, half_y = 0.5 * float(size[0]), 0.5 * float(size[1])
    for ix in range(5):
        for iy in range(5):
            x = float(center[0]) - half_x + ix * half_x / 2.0
            y = float(center[1]) - half_y + iy * half_y / 2.0
            column = int(math.floor((x - origin_x) / resolution))
            row = int(math.floor((y - origin_y) / resolution))
            if 0 <= column < width and 0 <= row < height:
                value = int(data[row * width + column])
                values.append(max(0, value))
    return max(values) if values else None


def old_position_clearance(
    records: Sequence[causal.RecordedMessage],
    timeline: Sequence[Mapping[str, Any]],
    actor: Mapping[str, Any],
    maximum_cost: int,
) -> dict[str, Any]:
    start = actor["start"]
    size = actor["size"]
    vacated = next((
        row for row in timeline
        if row["state"] in VISIBLE_ACTOR_STATES
        and math.dist(row["position"][:2], start[:2]) >= float(size[0])
    ), None)
    result: dict[str, Any] = {"vacated_stamp_ns": None, "consumers": {}}
    if vacated is None:
        return result
    vacated_stamp = int(vacated["stamp_ns"])
    result["vacated_stamp_ns"] = vacated_stamp
    for topic, name in (
        ("/global_costmap/costmap", "global"),
        ("/local_costmap/costmap", "local"),
    ):
        rows = sorted(
            (record for record in records if record.topic == topic),
            key=causal._message_stamp_ns,
        )
        before = [
            _costmap_max(row.message, start, size)
            for row in rows if causal._message_stamp_ns(row) <= vacated_stamp
        ]
        clear_stamp = next((
            causal._message_stamp_ns(row) for row in rows
            if causal._message_stamp_ns(row) >= vacated_stamp
            and (value := _costmap_max(row.message, start, size)) is not None
            and value <= maximum_cost
        ), None)
        result["consumers"][name] = {
            "occupied_before_vacated": any(
                value is not None and value > maximum_cost for value in before
            ),
            "clear_stamp_ns": clear_stamp,
            "clear_latency_sec": (
                (clear_stamp - vacated_stamp) * 1.0e-9
                if clear_stamp is not None else None
            ),
        }
    return result


def record_dynamic_evidence(
    experiment: DynamicLowObstacleExperiment,
    arm_label: str,
    bag_dir: str | Path,
    episode_jsonl: str | Path,
    output: str | Path,
) -> dict[str, Any]:
    manifest, run = _causal_manifest(experiment, arm_label)
    records = list(causal.read_rosbag_records(bag_dir, topics=_rosbag_topics()))
    timeline = actor_timeline(records, str(experiment.identity["actor_id"]))
    resolver = _actor_resolver(timeline, experiment.actor)
    episode_result = causal._episode_result_from_jsonl(
        Path(episode_jsonl).expanduser().resolve()
    )
    evidence = causal.build_recorded_evidence(
        manifest, run, records, episode_result,
        physical_obstacle=experiment.actor,
        obstacle_at_stamp=resolver,
        dynamic_actors_enabled=True,
    )
    positions = [row["position"][:2] for row in timeline]
    actor_span = max(
        (math.dist(lhs, rhs) for lhs in positions for rhs in positions),
        default=0.0,
    )
    event_names = sorted({
        str(event.get("event", ""))
        for row in timeline for event in row["events"] if event.get("event")
    })
    evidence["requested_arm_label"] = arm_label
    evidence["producer_stop_invoked"] = False
    evidence["dynamic_actor"] = {
        "id": experiment.actor["id"],
        "states": sorted({row["state"] for row in timeline}),
        "events": event_names,
        "position_span_m": actor_span,
        "timeline": timeline,
    }
    evidence["old_position_clearance"] = old_position_clearance(
        records, timeline, experiment.actor,
        int(experiment.criteria["old_position_cost_max"]),
    )
    destination = Path(output).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return evidence


def _candidate_position_span(samples: Any) -> float:
    points: list[tuple[float, float]] = []
    if isinstance(samples, list):
        for sample in samples:
            if not isinstance(sample, Mapping):
                continue
            for obstacle in sample.get("typed_obstacles", []):
                if isinstance(obstacle, Mapping) and obstacle.get("accepted") is True:
                    points.append((float(obstacle["x"]), float(obstacle["y"])))
    return max((math.dist(a, b) for a in points for b in points), default=0.0)


def _maximum_spin_duration(commands: Any, criteria: Mapping[str, Any]) -> float:
    if not isinstance(commands, list):
        return 0.0
    rows = sorted(
        (row for row in commands if isinstance(row, Mapping)),
        key=lambda row: int(row.get("stamp_ns", 0)),
    )
    maximum = current = 0.0
    previous_stamp: int | None = None
    for row in rows:
        stamp = int(row.get("stamp_ns", 0))
        spinning = (
            abs(float(row.get("linear_x", 0.0)))
            <= float(criteria["spin_linear_speed_max_mps"])
            and abs(float(row.get("angular_z", 0.0)))
            >= float(criteria["spin_angular_speed_min_rps"])
        )
        delta = (stamp - previous_stamp) * 1.0e-9 if previous_stamp is not None else 0.0
        current = current + delta if spinning and 0.0 <= delta <= 0.5 else 0.0
        maximum = max(maximum, current)
        previous_stamp = stamp
    return maximum


def _has_nav2_error105(value: Any) -> bool:
    if isinstance(value, Mapping):
        for key, item in value.items():
            if key in {"code", "error_code", "result_code", "status_code"}:
                try:
                    if int(item) == 105:
                        return True
                except (TypeError, ValueError):
                    pass
            if _has_nav2_error105(item):
                return True
        return False
    if isinstance(value, list):
        return any(_has_nav2_error105(item) for item in value)
    if isinstance(value, str):
        compact = value.lower().replace(" ", "")
        return "error105" in compact or "error_code=105" in compact
    return False


def evaluate_evidence(
    experiment: DynamicLowObstacleExperiment,
    arm_label: str,
    evidence: Mapping[str, Any],
) -> dict[str, Any]:
    manifest, run = _causal_manifest(experiment, arm_label)
    reasons: list[str] = []
    if evidence.get("requested_arm_label") != arm_label:
        reasons.append("arm_label_mismatch")
    if evidence.get("producer_stop_invoked") is not False:
        reasons.append("producer_fault_injection_not_allowed")
    dynamic = _mapping(evidence.get("dynamic_actor"), "dynamic_actor")
    states = set(dynamic.get("states", []))
    events = set(dynamic.get("events", []))
    if not {"armed", "moving", "parked"}.issubset(states):
        reasons.append("actor_lifecycle_incomplete")
    if states & FORBIDDEN_ACTOR_STATES or events & {"safety_yield", "near_contact_abort"}:
        reasons.append("actor_yield_or_guard_abort")
    actor_span = float(dynamic.get("position_span_m", 0.0))
    if actor_span < float(experiment.criteria["minimum_actor_position_span_m"]):
        reasons.append("actor_position_span_too_small")

    samples = evidence.get("synchronized_samples")
    try:
        spatial = causal._scan_and_spatial_metrics(
            samples,
            float(experiment.criteria["typed_spatial_match_tolerance_m"]),
            physical_obstacle=experiment.actor,
            validate_obstacles=True,
        )
    except (causal.CausalContractError, KeyError, TypeError, ValueError) as exc:
        spatial = {}
        reasons.append(f"dynamic_spatial_evidence_invalid:{exc}")
    if spatial.get("scan_invisible_rgbd_pairs", 0) <= 0:
        reasons.append("scan_zero_rgbd_positive_pair_missing")
    candidate_span = _candidate_position_span(samples)
    if candidate_span < float(experiment.criteria["minimum_candidate_position_span_m"]):
        reasons.append("candidate_position_span_too_small")

    layer = _mapping(evidence.get("layer"), "layer")
    global_layer = _mapping(layer.get("global"), "layer.global")
    local_layer = _mapping(layer.get("local"), "layer.local")
    critic = _mapping(evidence.get("critic"), "critic")
    if arm_label == "M1":
        for name, row in (("global", global_layer), ("local", local_layer)):
            if (
                int(row.get("applied_count", 0)) != 0
                or int(row.get("active_cells", 0)) != 0
                or int(row.get("max_cost_increase", 0)) != 0
            ):
                reasons.append(f"m1_{name}_layer_write_nonzero")
        if bool(critic.get("applied", False)):
            reasons.append("m1_critic_applied")
    else:
        for name, row in (("global", global_layer), ("local", local_layer)):
            if int(row.get("applied_count", 0)) <= 0 or int(row.get("cells", 0)) <= 0:
                reasons.append(f"active_{name}_layer_not_applied")
        if arm_label == "M3" and (
            not bool(critic.get("applied", False))
            or int(critic.get("applied_count", 0)) <= 0
            or int(critic.get("cost_delta_nonzero_count", 0)) <= 0
        ):
            reasons.append("m3_critic_not_applied")
        if arm_label == "M2-fallback" and bool(critic.get("applied", False)):
            reasons.append("m2_fallback_critic_applied")

        action = _mapping(evidence.get("action"), "action")
        passive = _mapping(evidence.get("passive"), "passive")
        if (
            action.get("state") != "SUCCEEDED"
            or action.get("terminal_zero_confirmed") is not True
            or passive.get("success") is not True
            or passive.get("collision") is not False
        ):
            reasons.append("active_navigation_not_successful_safe_terminal_zero")
        if _has_nav2_error105({"action": action, "route": evidence.get("route")}):
            reasons.append("nav2_error105")
        commands = evidence.get("cmd_vel")
        if isinstance(commands, list) and any(
            float(row.get("linear_x", 0.0))
            < float(experiment.criteria["reverse_speed_limit_mps"])
            for row in commands if isinstance(row, Mapping)
        ):
            reasons.append("pathological_reverse")
        spin_duration = _maximum_spin_duration(commands, experiment.criteria)
        if spin_duration > float(experiment.criteria["spin_duration_max_sec"]):
            reasons.append("pathological_spin")

        clearance = _mapping(
            evidence.get("old_position_clearance"), "old_position_clearance"
        )
        consumers = _mapping(clearance.get("consumers"), "old_position_clearance.consumers")
        for name in ("global", "local"):
            row = _mapping(consumers.get(name), f"old_position_clearance.{name}")
            latency = row.get("clear_latency_sec")
            if row.get("occupied_before_vacated") is not True:
                reasons.append(f"{name}_old_position_never_observed_occupied")
            elif latency is None or float(latency) > float(
                experiment.criteria["old_position_clear_timeout_sec"]
            ):
                reasons.append(f"{name}_old_position_not_cleared_in_time")

    action = _mapping(evidence.get("action"), "action")
    if action.get("terminal_zero_confirmed") is not True:
        reasons.append("terminal_zero_not_confirmed")
    return {
        "qualification": "ENGINEERING_ONLY_NOT_FORMAL",
        "verdict": "PASS" if not reasons else "FAIL",
        "arm_label": arm_label,
        "module3_mode": run.arm,
        "reasons": reasons,
        "metrics": {
            "actor_position_span_m": actor_span,
            "candidate_position_span_m": candidate_span,
            "scan_invisible_rgbd_pairs": spatial.get("scan_invisible_rgbd_pairs"),
            "source_recall": spatial.get("source_recall"),
            "candidate_precision": spatial.get("candidate_precision"),
            "old_position_clearance": evidence.get("old_position_clearance"),
            "maximum_spin_duration_sec": _maximum_spin_duration(
                evidence.get("cmd_vel"), experiment.criteria
            ),
        },
    }


def _write_or_print(payload: Any, output: str | None) -> None:
    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if output:
        destination = Path(output).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(text, encoding="utf-8")
    else:
        print(text, end="")


def run_experiment(
    experiment: DynamicLowObstacleExperiment,
    arm_label: str,
    output_root: str | Path,
    *,
    shutdown_timeout_sec: float = causal.DEFAULT_SHUTDOWN_TIMEOUT_SEC,
) -> dict[str, Any]:
    manifest, run = _causal_manifest(experiment, arm_label)
    plan = build_plan(experiment, arm_label, output_root)
    run_dir = Path(plan["run_directory"])
    prepared_plan = {
        "mode": "single_dynamic_low_obstacle",
        "runs": [{
            "run_id": run.run_id,
            "repeat": run.repeat,
            "arm": run.arm,
            "run_directory": str(run_dir),
            "setup": {"module2_socket": plan["module2_socket"]},
            "commands": plan["commands"],
        }],
    }

    def record_hook(
        _manifest: causal.CausalManifest,
        _run: causal.RunContract,
        bag_dir: Path,
        episode_jsonl: Path,
        output: Path,
    ) -> dict[str, Any]:
        return record_dynamic_evidence(
            experiment, arm_label, bag_dir, episode_jsonl, output
        )

    summary = causal.run_campaign(
        manifest, None, output_root, pilot=False,
        shutdown_timeout_sec=shutdown_timeout_sec,
        prepared_plan=prepared_plan,
        recorder_command=lambda _manifest, bag_dir: _rosbag_command(
            experiment, bag_dir
        ),
        evidence_recorder=record_hook,
        stop_producer_after_episode=False,
        classify_baseline_collision=False,
    )
    status = summary["runs"][0]
    evidence_path = run_dir / f"{run.run_id}.json"
    if evidence_path.is_file():
        evidence = _mapping(
            json.loads(evidence_path.read_text(encoding="utf-8")), "evidence"
        )
        status["evaluation"] = evaluate_evidence(experiment, arm_label, evidence)
        status["producer_stop_invoked"] = False
        if (
            status["evaluation"]["verdict"] == "PASS"
            and status.get("cleanup", {}).get("ok") is True
        ):
            status["state"] = "ENGINEERING_OUTCOME_RECORDED"
            summary["state"] = "FINISHED"
    (run_dir / "run_status.json").write_text(
        json.dumps(status, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    Path(summary["output_root"]).joinpath("campaign_result.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("plan", "run"):
        child = subparsers.add_parser(command)
        child.add_argument("--config", required=True)
        child.add_argument("--arm", required=True, choices=tuple(ARM_MODES))
        child.add_argument("--output-root", required=True)
        child.add_argument("--output")
    dispatch = subparsers.add_parser("dispatch-episode")
    dispatch.add_argument("--config", required=True)
    dispatch.add_argument("--arm", required=True, choices=tuple(ARM_MODES))
    dispatch.add_argument("--output-jsonl", required=True)
    dispatch.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    dispatch.add_argument("--reset-timeout-sec", type=float, default=120.0)
    dispatch.add_argument("--navigation-timeout-sec", type=float, default=180.0)
    record = subparsers.add_parser("record-evidence")
    record.add_argument("--config", required=True)
    record.add_argument("--arm", required=True, choices=tuple(ARM_MODES))
    record.add_argument("--bag-dir", required=True)
    record.add_argument("--episode-jsonl", required=True)
    record.add_argument("--output", required=True)
    evaluate = subparsers.add_parser("evaluate")
    evaluate.add_argument("--config", required=True)
    evaluate.add_argument("--arm", required=True, choices=tuple(ARM_MODES))
    evaluate.add_argument("--evidence", required=True)
    evaluate.add_argument("--output")
    return parser


def cli(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        experiment = load_experiment(args.config)
        if args.command == "plan":
            _write_or_print(build_plan(experiment, args.arm, args.output_root), args.output)
            return 0
        if args.command == "dispatch-episode":
            manifest, run = _causal_manifest(experiment, args.arm)
            result = causal.dispatch_episode(
                manifest, run, args.output_jsonl,
                readiness_timeout_sec=args.readiness_timeout_sec,
                reset_timeout_sec=args.reset_timeout_sec,
                navigation_timeout_sec=args.navigation_timeout_sec,
            )
            print(json.dumps(result, sort_keys=True))
            return 0 if result.get("state") == "SUCCEEDED" else 2
        if args.command == "record-evidence":
            evidence = record_dynamic_evidence(
                experiment, args.arm, args.bag_dir, args.episode_jsonl, args.output
            )
            print(json.dumps({"state": "RECORDED", "output": args.output,
                              "samples": len(evidence["synchronized_samples"])}, sort_keys=True))
            return 0
        if args.command == "evaluate":
            evidence = _mapping(
                json.loads(Path(args.evidence).read_text(encoding="utf-8")), "evidence"
            )
            result = evaluate_evidence(experiment, args.arm, evidence)
            _write_or_print(result, args.output)
            return 0 if result["verdict"] == "PASS" else 2
        result = run_experiment(experiment, args.arm, args.output_root)
        _write_or_print(result, args.output)
        return 0 if (
            result.get("runs")
            and result["runs"][0].get("evaluation", {}).get("verdict") == "PASS"
        ) else 2
    except (
        OSError, ValueError, json.JSONDecodeError, yaml.YAMLError,
        causal.CausalContractError, DynamicLowObstacleError,
    ) as exc:
        print(f"INVALID: {exc}", file=os.sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(cli())


if __name__ == "__main__":
    main()
