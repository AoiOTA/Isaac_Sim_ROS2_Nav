"""Plan and evaluate the V6 estimated-state calibration matrix.

This module deliberately has no ROS execution adapter.  ``run`` emits NOT_RUN
records so a static plan can never be mistaken for completed calibration.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
from typing import Any, Mapping

import yaml


SCHEMA_VERSION = 1
ARM_ORDER = ("off", "shadow", "fused")
PRIMITIVE_ORDER = ("straight_3m", "ccw_360", "cw_360", "s_route")
PASSIVE_EVALUATOR_TOPICS = (
    "/odom", "/wheel/odom", "/lidar/odom", "/imu/data",
    "/amcl_pose", "/ground_truth/odom",
)
DISPATCHER_TOPICS = ("/clock", "/odom", "/tf", "/amcl_pose")
RECORDED_ESTIMATE_STREAMS = (
    "odom", "wheel_odom", "lidar_odom", "imu_data", "amcl_pose",
)
STREAM_MINIMUM_FREQUENCY_HZ = {
    "odom": 30.0,
    "lidar_odom": 15.0,
    "amcl_pose": 1.0,
}
SHADOW_MAXIMUM_ALIGNED_ATE_P95_M = 0.15


@dataclass
class RivermarkRouteAcceptance:
    """Pure fail-closed request handshake for the future live adapter."""

    request_id: int
    published_at_sec: float
    acceptance_timeout_sec: float
    route_timeout_sec: float
    canonical_route_seen: bool = False
    route_progress_seen: bool = False
    accepted_at_sec: float | None = None
    failed_at_sec: float | None = None
    failure_reason: str | None = None

    @property
    def state(self) -> str:
        if self.failure_reason is not None:
            return "FAIL"
        if self.accepted_at_sec is not None:
            return "ACCEPTED"
        return "AWAITING_ACCEPTANCE"

    @property
    def route_timeout_started_at_sec(self) -> float | None:
        return self.accepted_at_sec

    @property
    def route_deadline_sec(self) -> float | None:
        if self.accepted_at_sec is None:
            return None
        return self.accepted_at_sec + self.route_timeout_sec

    @property
    def resend_allowed(self) -> bool:
        return False

    def observe(self, topic: str, request_id: int, now_sec: float) -> bool:
        if self.state != "AWAITING_ACCEPTANCE" or int(request_id) != self.request_id:
            return False
        if now_sec >= self.published_at_sec + self.acceptance_timeout_sec:
            self.poll(now_sec)
            return False
        if topic == "/bio_nav/canonical_route":
            self.canonical_route_seen = True
        elif topic == "/bio_nav/route_progress":
            self.route_progress_seen = True
        else:
            return False
        if self.canonical_route_seen and self.route_progress_seen:
            self.accepted_at_sec = float(now_sec)
            return True
        return False

    def poll(self, now_sec: float) -> str:
        if (
            self.state == "AWAITING_ACCEPTANCE"
            and now_sec >= self.published_at_sec + self.acceptance_timeout_sec
        ):
            self.failed_at_sec = float(now_sec)
            self.failure_reason = "route_acceptance_timeout_no_resend"
        return self.state


def _default_config_path() -> Path:
    source = Path(__file__).parents[1] / "config" / "v6_estimated_calibration.yaml"
    if source.is_file():
        return source
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        installed = Path(prefix) / "share" / "robot_experiments" / "config" / "v6_estimated_calibration.yaml"
        if installed.is_file():
            return installed
    return source


DEFAULT_CONFIG = _default_config_path()


class CalibrationContractError(ValueError):
    """Raised when the frozen calibration plan or an input report is invalid."""


def load_config(path: str | Path = DEFAULT_CONFIG) -> dict[str, Any]:
    document = yaml.safe_load(Path(path).expanduser().resolve().read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("schema_version") != SCHEMA_VERSION:
        raise CalibrationContractError("calibration schema_version must be 1")
    if document.get("repeats") != 3:
        raise CalibrationContractError("calibration repeats must be exactly 3")
    arms = document.get("arms")
    primitives = document.get("indoor_primitives")
    if [arm.get("id") for arm in arms or ()] != list(ARM_ORDER):
        raise CalibrationContractError("arm order must be off, shadow, fused")
    if [primitive.get("id") for primitive in primitives or ()] != list(PRIMITIVE_ORDER):
        raise CalibrationContractError("primitive order must be 3m, CCW, CW, S")
    if tuple(document["execution"].get("dispatcher_topics", ())) != DISPATCHER_TOPICS:
        raise CalibrationContractError("dispatcher topic contract changed")
    if any(topic.startswith("/ground_truth/") for topic in DISPATCHER_TOPICS):
        raise CalibrationContractError("dispatcher must not consume ground truth")
    if tuple(document["execution"].get("passive_evaluator_topics", ())) != PASSIVE_EVALUATOR_TOPICS:
        raise CalibrationContractError("passive evaluator topic contract changed")
    if (
        document["execution"].get("dispatcher_result_file") != "dispatcher_result.json"
        or document["execution"].get("dispatcher_success_status") != "SUCCEEDED"
        or document["execution"].get("collision_field") != "collision_detected"
    ):
        raise CalibrationContractError("dispatcher result/collision contract changed")
    route_acceptance = document["execution"].get("rivermark_route_acceptance", {})
    if route_acceptance != {
        "canonical_route_topic": "/bio_nav/canonical_route",
        "route_progress_topic": "/bio_nav/route_progress",
        "acceptance_timeout_sec": 15.0,
        "resend_count": 0,
        "route_timeout_starts_after_acceptance": True,
    }:
        raise CalibrationContractError("Rivermark route acceptance contract changed")
    if document["execution"].get("reset_count_per_episode") != 1 or document["execution"].get("retry_count") != 0:
        raise CalibrationContractError("each episode requires exactly one reset and no retry")
    fused = arms[-1]
    if fused.get("requires_shadow_validation") is not True:
        raise CalibrationContractError("fused arm must require shadow validation")
    environment = document.get("indoor_environment", {})
    expected_environment = Path(
        "/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Grid/default_environment.usd"
    )
    if Path(environment.get("environment_usd", "")) != expected_environment:
        raise CalibrationContractError("indoor calibration must use the official Grid default_environment")
    for field in ("environment_usd", "spawn_poses_file", "map_yaml", "static_obstacle_config"):
        if not Path(environment.get(field, "")).is_file():
            raise CalibrationContractError(f"indoor calibration {field} is missing")
    if environment.get("spawn_pose_name") != "mapping_start" or environment.get("map_version") != "flat20_v1":
        raise CalibrationContractError("flat20 spawn/map contract changed")
    supplement = document.get("supplement", {})
    if supplement.get("revision") != "v6_estimated_calibration_flat20_supplement_r1":
        raise CalibrationContractError("flat20 supplemental revision changed")
    if supplement.get("exclusion_mapping") != {
        "prior_narrow_indoor_calibration": "excluded_environment_mismatch",
        "prior_primary_evidence": "unchanged_not_reclassified",
        "rivermark_calibration_rows": "unchanged_separate_environment",
    }:
        raise CalibrationContractError("flat20 evidence exclusion mapping changed")
    frequency = document.get("thresholds", {}).get("engineering_recommendation", {}).get("minimum_frequency_hz")
    if frequency != STREAM_MINIMUM_FREQUENCY_HZ:
        raise CalibrationContractError("stream-specific frequency contract changed")
    return document


def build_manifest(config: Mapping[str, Any]) -> dict[str, Any]:
    """Create the exact 45-episode, arm-grouped matrix."""
    episodes = []
    seed = int(config["reset_seed_base"])
    index = 0
    for arm in config["arms"]:
        arm_id = arm["id"]
        for primitive in config["indoor_primitives"]:
            for repeat in range(1, int(config["repeats"]) + 1):
                index += 1
                episodes.append(_episode(
                    index=index,
                    identifier=f"{arm_id}-indoor-{primitive['id']}-r{repeat:02d}",
                    arm=arm,
                    environment="indoor",
                    scenario_id=primitive["id"],
                    repeat=repeat,
                    seed=seed + index,
                    execution={
                        "type": "cmd_vel_primitive",
                        "segments": primitive["segments"],
                        **dict(config["indoor_environment"]),
                    },
                ))
        route = config["rivermark"]
        for repeat in range(1, int(config["repeats"]) + 1):
            index += 1
            episodes.append(_episode(
                index=index,
                identifier=f"{arm_id}-rivermark-static-g1-r{repeat:02d}",
                arm=arm,
                environment="rivermark",
                scenario_id=route["id"],
                repeat=repeat,
                seed=seed + index,
                execution={
                    "type": "nav2_route_goal",
                    "scenario": route["scenario"],
                    "start": route["start"],
                    "wrapper": route["wrapper"],
                    "goal": route["goal"],
                    "acceptance_handshake": dict(
                        config["execution"]["rivermark_route_acceptance"]
                    ),
                },
            ))
    if len(episodes) != 45:
        raise CalibrationContractError(f"expected 45 episodes, got {len(episodes)}")
    return {
        "schema_version": SCHEMA_VERSION,
        "campaign_id": config["campaign_id"],
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "PLANNED_NOT_EXECUTED",
        "episode_count": len(episodes),
        "grouped_by_arm": True,
        "single_isaac_session_reusable": True,
        "dispatcher_topics": list(DISPATCHER_TOPICS),
        "passive_evaluator_topics": list(PASSIVE_EVALUATOR_TOPICS),
        "ground_truth_firewall": True,
        "thresholds": config["thresholds"],
        "supplement": config["supplement"],
        "indoor_environment": config["indoor_environment"],
        "episodes": episodes,
    }


def _episode(*, index, identifier, arm, environment, scenario_id, repeat, seed, execution):
    return {
        "index": index,
        "episode_id": identifier,
        "arm": arm["id"],
        "environment": environment,
        "scenario_id": scenario_id,
        "repeat": repeat,
        "reset_seed": seed,
        "reset_count": 1,
        "retry_count": 0,
        "launch_arguments": dict(arm["launch_arguments"]),
        "requires_shadow_validation": bool(arm.get("requires_shadow_validation", False)),
        "validated_argument_is_not_evidence": arm["id"] == "fused",
        "execution": execution,
        "dispatcher_topics": list(DISPATCHER_TOPICS),
        "passive_evaluator_topics": list(PASSIVE_EVALUATOR_TOPICS),
    }


def assess_shadow_promotion(evaluation: Mapping[str, Any] | None) -> dict[str, Any]:
    """Gate fused dispatch using completed shadow reports, never a launch boolean."""
    failures = []
    rows = [] if evaluation is None else [row for row in evaluation.get("episodes", ()) if row.get("arm") == "shadow"]
    if len(rows) != 15:
        failures.append("shadow_requires_15_evaluated_episodes")
    for row in rows:
        metrics = row.get("stream_metrics", {}).get("lidar_odom")
        if row.get("status") != "EVALUATED" or not isinstance(metrics, dict):
            failures.append(f"{row.get('episode_id', 'unknown')}:not_evaluated")
            continue
        diagnostics = metrics.get("input", {})
        covariance = diagnostics.get("covariance", {})
        scale = metrics.get("scale", {})
        association = metrics.get("association", {})
        checks = {
            "backward_stamps": diagnostics.get("backward_stamps") == 0,
            "pose_jumps": diagnostics.get("pose_jump_count") == 0,
            "yaw_jumps": diagnostics.get("yaw_jump_count") == 0,
            "frequency": _at_least(diagnostics.get("frequency_hz"), 15.0),
            "matched_samples": _at_least(association.get("matched_count"), 1),
            "time_offset": _at_most_abs(association.get("best_estimate_time_offset_ms"), 100.0),
            "covariance_finite": covariance.get("finite_fraction") == 1.0,
            "covariance_symmetric": covariance.get("symmetric_fraction") == 1.0,
            "covariance_psd": covariance.get("positive_semidefinite_fraction") == 1.0,
        }
        scenario = row.get("scenario_id")
        endpoint = (metrics.get("endpoint") or {}).get("aligned", {})
        if scenario == "straight_3m":
            checks.update({
                "linear_scale_denominator": scale.get("linear_denominator_valid") is True,
                "linear_scale": _between(scale.get("linear"), 0.90, 1.10),
                "longitudinal_error": _at_most_abs(endpoint.get("longitudinal_error_m"), 0.10),
                "lateral_error": _at_most_abs(endpoint.get("lateral_error_m"), 0.10),
            })
        elif scenario in ("ccw_360", "cw_360"):
            checks.update({
                "yaw_scale_denominator": scale.get("yaw_denominator_valid") is True,
                "yaw_scale": _between(scale.get("yaw"), 0.90, 1.10),
                "yaw_closure": _at_most_abs(endpoint.get("yaw_error_rad"), math.radians(5.0)),
                "position_closure": _at_most_abs(endpoint.get("position_error_m"), 0.20),
            })
        elif scenario == "s_route":
            checks.update({
                "aligned_ate": _at_most_abs(
                    metrics.get("aligned_ate", {}).get("xy_m", {}).get("p95_abs"),
                    SHADOW_MAXIMUM_ALIGNED_ATE_P95_M,
                ),
                "aligned_yaw": _at_most_abs(
                    metrics.get("aligned_ate", {}).get("yaw_rad", {}).get("p95_abs"),
                    math.radians(5.0),
                ),
            })
        elif scenario == "rivermark_static_start_to_g1":
            checks.update({
                "rivermark_lidar_ate": _at_most_abs(
                    metrics.get("absolute_ate", {}).get("xy_m", {}).get("p95_abs"), 0.35
                ),
                "rivermark_lidar_yaw": _at_most_abs(
                    metrics.get("absolute_ate", {}).get("yaw_rad", {}).get("p95_abs"),
                    math.radians(8.0),
                ),
            })
        else:
            checks["known_scenario"] = False
        failures.extend(f"{row['episode_id']}:{name}" for name, passed in checks.items() if not passed)
    return {
        "passed": not failures,
        "requires_explicit_promotion_flag": True,
        "evaluated_shadow_episode_count": len(rows),
        "failure_reasons": failures,
    }


def build_dispatch_plan(
    manifest: Mapping[str, Any],
    *,
    explicit_fused_promotion: bool = False,
    shadow_evaluation: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    shadow_gate = assess_shadow_promotion(shadow_evaluation)
    fused_allowed = explicit_fused_promotion and shadow_gate["passed"]
    episodes = []
    for episode in manifest["episodes"]:
        blocked = episode["arm"] == "fused" and not fused_allowed
        episodes.append({
            **episode,
            "dispatch_status": "BLOCKED" if blocked else "PLANNED",
            "dispatch_reason": (
                "fused_requires_shadow_gate_and_explicit_promotion" if blocked else "ready_for_runtime_adapter"
            ),
        })
    return {
        "schema_version": SCHEMA_VERSION,
        "campaign_id": manifest["campaign_id"],
        "status": "DISPATCH_PLAN_ONLY",
        "explicit_fused_promotion": explicit_fused_promotion,
        "shadow_gate": shadow_gate,
        "fused_dispatch_allowed": fused_allowed,
        "episodes": episodes,
    }


def evaluate_campaign(manifest: Mapping[str, Any], results_root: str | Path) -> dict[str, Any]:
    """Aggregate passive-evaluator reports without manufacturing missing runs."""
    root = Path(results_root).expanduser().resolve()
    rows = []
    for episode in manifest["episodes"]:
        dispatcher_path = root / episode["episode_id"] / "dispatcher_result.json"
        dispatcher_result, dispatcher_failure = _load_dispatcher_result(
            dispatcher_path, episode["episode_id"]
        )
        if dispatcher_failure is not None:
            rows.append({
                **_episode_identity(episode),
                "status": "INVALID" if dispatcher_path.is_file() else "NOT_RUN",
                "reason": dispatcher_failure,
                "threshold_assessment": {
                    "passed": False,
                    "failure_reasons": [dispatcher_failure],
                },
                "dispatcher_result": dispatcher_result,
            })
            continue
        report_path = root / episode["episode_id"] / "estimated_state_metrics.json"
        if not report_path.is_file():
            rows.append({**_episode_identity(episode), "status": "NOT_RUN", "reason": "metrics_report_missing"})
            continue
        report = json.loads(report_path.read_text(encoding="utf-8"))
        stream = "amcl_pose" if episode["environment"] == "rivermark" else "odom"
        stream_metrics = report.get("estimates", {})
        metrics = stream_metrics.get(stream)
        if not isinstance(metrics, dict):
            rows.append({**_episode_identity(episode), "status": "NOT_RUN", "reason": f"{stream}_metrics_missing"})
            continue
        threshold = assess_episode_thresholds(episode, metrics, stream=stream)
        rows.append({
            **_episode_identity(episode),
            "status": "EVALUATED",
            "selected_stream": stream,
            "selected_metrics": metrics,
            "stream_metrics": {
                name: value for name, value in stream_metrics.items()
                if name in RECORDED_ESTIMATE_STREAMS and isinstance(value, dict)
            },
            "stream_frequency_assessment": assess_stream_frequencies(stream_metrics),
            "threshold_assessment": threshold,
            "source_report": str(report_path),
            "dispatcher_result": dispatcher_result,
            "dispatcher_result_source": str(dispatcher_path),
        })
    if any(row["status"] == "INVALID" for row in rows):
        campaign_status = "INVALID"
    elif all(row["status"] == "EVALUATED" for row in rows):
        campaign_status = "EVALUATED"
    else:
        campaign_status = "INCOMPLETE_NOT_RUN"
    summary = {
        "schema_version": SCHEMA_VERSION,
        "campaign_id": manifest["campaign_id"],
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": campaign_status,
        "episode_count": len(rows),
        "thresholds": manifest["thresholds"],
        "evaluated_count": sum(row["status"] == "EVALUATED" for row in rows),
        "threshold_pass_count": sum(row.get("threshold_assessment", {}).get("passed") is True for row in rows),
        "episodes": rows,
        "aggregates": _aggregate_rows(rows),
        "cw_ccw_bias": _cw_ccw_bias(rows),
    }
    summary["shadow_promotion_gate"] = assess_shadow_promotion(summary)
    summary["fused_comparison"] = assess_fused_comparison(rows)
    return summary


def assess_stream_frequencies(stream_metrics: Mapping[str, Any]) -> dict[str, Any]:
    result = {}
    for stream, minimum_hz in STREAM_MINIMUM_FREQUENCY_HZ.items():
        metrics = stream_metrics.get(stream)
        frequency = metrics.get("input", {}).get("frequency_hz") if isinstance(metrics, dict) else None
        result[stream] = {
            "minimum_hz": minimum_hz,
            "measured_hz": frequency,
            "passed": _at_least(frequency, minimum_hz),
            "missing": not isinstance(metrics, dict),
        }
    return result


def _load_dispatcher_result(
    path: Path, episode_id: str
) -> tuple[dict[str, Any] | None, str | None]:
    if not path.is_file():
        return None, "dispatcher_result_missing"
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None, "dispatcher_result_invalid"
    if not isinstance(document, dict) or document.get("episode_id") != episode_id:
        return document if isinstance(document, dict) else None, "dispatcher_result_episode_mismatch"
    collision = document.get("collision_detected")
    if not isinstance(collision, bool):
        return document, "dispatcher_collision_missing"
    if collision:
        return document, "collision_detected"
    if document.get("status") != "SUCCEEDED":
        return document, "dispatcher_not_succeeded"
    return document, None


def assess_episode_thresholds(
    episode: Mapping[str, Any],
    metrics: Mapping[str, Any],
    *,
    stream: str | None = None,
) -> dict[str, Any]:
    failures = []
    diagnostics = metrics.get("input", {})
    selected_stream = stream or ("amcl_pose" if episode["environment"] == "rivermark" else "odom")
    minimum_frequency_hz = STREAM_MINIMUM_FREQUENCY_HZ[selected_stream]
    if not _at_least(diagnostics.get("frequency_hz"), minimum_frequency_hz):
        failures.append(f"{selected_stream}_frequency_below_{minimum_frequency_hz:g}hz")
    if diagnostics.get("backward_stamps") != 0:
        failures.append("backward_stamp")
    if diagnostics.get("pose_jump_count") != 0 or diagnostics.get("yaw_jump_count") != 0:
        failures.append("pose_jump")
    scenario = episode["scenario_id"]
    if scenario == "straight_3m":
        endpoint = (metrics.get("endpoint") or {}).get("aligned", {})
        if not _at_most_abs(endpoint.get("longitudinal_error_m"), 0.10):
            failures.append("longitudinal_error")
        if not _at_most_abs(endpoint.get("lateral_error_m"), 0.10):
            failures.append("lateral_error")
    elif scenario in ("ccw_360", "cw_360"):
        endpoint = (metrics.get("endpoint") or {}).get("aligned", {})
        if not _at_most_abs(endpoint.get("yaw_error_rad"), math.radians(5.0)):
            failures.append("yaw_closure")
        if not _at_most_abs(endpoint.get("position_error_m"), 0.20):
            failures.append("position_closure")
    elif scenario == "s_route":
        aligned = metrics.get("aligned_ate", {})
        if not _at_most_abs(aligned.get("xy_m", {}).get("p95_abs"), 0.15):
            failures.append("aligned_ate_p95")
        if not _at_most_abs(aligned.get("yaw_rad", {}).get("p95_abs"), math.radians(5.0)):
            failures.append("aligned_yaw_p95")
    else:
        absolute = metrics.get("absolute_ate", {})
        if not _at_most_abs(absolute.get("xy_m", {}).get("p95_abs"), 0.35):
            failures.append("amcl_absolute_ate_p95")
        if not _at_most_abs(absolute.get("yaw_rad", {}).get("p95_abs"), math.radians(8.0)):
            failures.append("amcl_absolute_yaw_p95")
    return {
        "passed": not failures,
        "failure_reasons": failures,
        "covariance_coverage_is_diagnostic_only": True,
        "threshold_class": "plan_reference_and_engineering_recommendation",
        "selected_stream": selected_stream,
        "minimum_frequency_hz": minimum_frequency_hz,
    }


def assess_fused_comparison(rows) -> dict[str, Any]:
    values = {arm: [] for arm in ARM_ORDER}
    for row in rows:
        metrics = row.get("selected_metrics", {})
        value = metrics.get("aligned_ate", {}).get("xy_m", {}).get("p95_abs")
        if row.get("status") == "EVALUATED" and isinstance(value, (int, float)) and math.isfinite(value):
            values[row["arm"]].append(float(value))
    if any(not values[arm] for arm in ARM_ORDER):
        return {"status": "NOT_AVAILABLE", "passed": False, "reason": "all three arms require evaluated metrics"}
    means = {arm: sum(items) / len(items) for arm, items in values.items()}
    references = (means["off"], means["shadow"])
    improved = means["fused"] <= min(references) * 0.95
    no_regression = means["fused"] <= max(references) * 1.10
    return {
        "status": "AVAILABLE",
        "passed": improved and no_regression,
        "mean_aligned_ate_p95_m": means,
        "at_least_one_clear_improvement": improved,
        "no_regression": no_regression,
    }


def _cw_ccw_bias(rows):
    result = {}
    for arm in ARM_ORDER:
        direction = {}
        for scenario in ("ccw_360", "cw_360"):
            values = [
                row.get("selected_metrics", {}).get("scale", {}).get("yaw_change_bias_rad")
                for row in rows
                if row.get("arm") == arm and row.get("scenario_id") == scenario and row.get("status") == "EVALUATED"
            ]
            values = [float(value) for value in values if isinstance(value, (int, float)) and math.isfinite(value)]
            direction[scenario] = sum(values) / len(values) if values else None
        if direction["ccw_360"] is not None and direction["cw_360"] is not None:
            direction["signed_asymmetry_rad"] = direction["ccw_360"] + direction["cw_360"]
        else:
            direction["signed_asymmetry_rad"] = None
        result[arm] = direction
    return result


def _aggregate_rows(rows):
    result = []
    for arm in ARM_ORDER:
        for scenario in (*PRIMITIVE_ORDER, "rivermark_static_start_to_g1"):
            selected = [row for row in rows if row.get("arm") == arm and row.get("scenario_id") == scenario]
            evaluated = [row for row in selected if row.get("status") == "EVALUATED"]
            result.append({
                "arm": arm,
                "scenario_id": scenario,
                "planned_run_count": len(selected),
                "evaluated_run_count": len(evaluated),
                "threshold_pass_count": sum(row.get("threshold_assessment", {}).get("passed") is True for row in evaluated),
                "mean_absolute_ate_p95_m": _mean_path(evaluated, ("absolute_ate", "xy_m", "p95_abs")),
                "mean_aligned_ate_p95_m": _mean_path(evaluated, ("aligned_ate", "xy_m", "p95_abs")),
                "mean_rpe_1s_p95_m": _mean_path(evaluated, ("rpe_fixed_1s", "xy_m", "p95_abs")),
                "mean_rpe_1m_p95_m": _mean_path(evaluated, ("rpe_fixed_1m", "xy_m", "p95_abs")),
                "mean_planar_nees": _mean_path(evaluated, ("planar_nees", "summary", "mean_abs")),
            })
    return result


def _mean_path(rows, path):
    values = []
    for row in rows:
        value = row.get("selected_metrics", {})
        for component in path:
            value = value.get(component) if isinstance(value, dict) else None
        if isinstance(value, (int, float)) and math.isfinite(value):
            values.append(float(value))
    return sum(values) / len(values) if values else None


def write_manifest_bundle(manifest: Mapping[str, Any], output_dir: str | Path) -> None:
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    _write_json(output / "calibration_manifest.json", manifest)
    columns = ("index", "episode_id", "arm", "environment", "scenario_id", "repeat", "reset_seed", "reset_count", "retry_count")
    with (output / "calibration_manifest.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for episode in manifest["episodes"]:
            writer.writerow({name: episode[name] for name in columns})
    _write_json(output / "plot_input_manifest.json", {
        "schema_version": SCHEMA_VERSION,
        "status": "INPUT_SCHEMA_ONLY",
        "episode_id_field": "episode_id",
        "group_fields": ["arm", "environment", "scenario_id", "repeat"],
        "metric_families": ["absolute_ate", "aligned_ate", "rpe_fixed_1s", "rpe_fixed_1m", "endpoint", "scale", "planar_nees", "covariance_consistency"],
        "episode_match_csv_pattern": "<results_root>/<episode_id>/estimated_state_matches.csv",
    })


def _episode_identity(episode):
    return {name: episode[name] for name in ("episode_id", "arm", "environment", "scenario_id", "repeat")}


def _at_least(value, threshold):
    return isinstance(value, (int, float)) and math.isfinite(value) and value >= threshold


def _at_most_abs(value, threshold):
    return isinstance(value, (int, float)) and math.isfinite(value) and abs(value) <= threshold


def _between(value, minimum, maximum):
    return isinstance(value, (int, float)) and math.isfinite(value) and minimum <= value <= maximum


def _write_json(path: Path, value: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    subparsers = parser.add_subparsers(dest="command", required=True)
    for command in ("manifest", "plan", "run"):
        child = subparsers.add_parser(command)
        child.add_argument("--output-dir", required=True)
        if command in ("plan", "run"):
            child.add_argument("--allow-fused-promotion", action="store_true")
            child.add_argument("--shadow-evaluation")
    evaluate = subparsers.add_parser("evaluate")
    evaluate.add_argument("--results-root", required=True)
    evaluate.add_argument("--output-dir", required=True)
    arguments = parser.parse_args(argv)
    manifest = build_manifest(load_config(arguments.config))
    output = Path(arguments.output_dir).expanduser().resolve()
    write_manifest_bundle(manifest, output)
    if arguments.command == "manifest":
        return 0
    if arguments.command == "evaluate":
        evaluation = evaluate_campaign(manifest, arguments.results_root)
        _write_json(output / "calibration_evaluation.json", evaluation)
        with (output / "calibration_evaluation.csv").open("w", encoding="utf-8", newline="") as stream:
            fieldnames = ("episode_id", "arm", "environment", "scenario_id", "repeat", "status", "threshold_passed", "reason")
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for row in evaluation["episodes"]:
                writer.writerow({
                    **{name: row.get(name) for name in fieldnames},
                    "threshold_passed": row.get("threshold_assessment", {}).get("passed"),
                })
        return 0
    shadow_evaluation = None
    if arguments.shadow_evaluation:
        shadow_evaluation = json.loads(Path(arguments.shadow_evaluation).read_text(encoding="utf-8"))
    plan = build_dispatch_plan(
        manifest,
        explicit_fused_promotion=arguments.allow_fused_promotion,
        shadow_evaluation=shadow_evaluation,
    )
    if arguments.command == "run":
        plan["status"] = "NOT_RUN"
        plan["reason"] = "runtime_adapter_not_implemented"
        for episode in plan["episodes"]:
            episode["run_status"] = "NOT_RUN"
            episode["run_reason"] = "runtime_adapter_not_implemented"
    _write_json(output / ("calibration_run.json" if arguments.command == "run" else "calibration_dispatch_plan.json"), plan)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
