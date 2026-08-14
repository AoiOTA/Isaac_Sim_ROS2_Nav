"""Fail-closed Final Rivermark 3x20 navigation qualification.

The report separates end-to-end navigation outcomes from compute-only timing.
Static runs must encounter four physical boxes.  Dynamic runs must prove a
measurable threat from every actor, not merely that an actor existed.
"""

from __future__ import annotations

import argparse
import ast
from bisect import bisect_left
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
from statistics import median
from typing import Any, Iterable, Mapping

import yaml

from robot_experiments.attempt31_rivermark_qualification import (
    _load_records,
    _module2_runtime_consumption,
    _rate_group,
)


ACTIVE_DYNAMIC_STATES = frozenset(
    {"moving", "dwell", "clearing", "safety_yield", "parked"}
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _percentile(values: Iterable[float], percentile: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percentile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _read_gzip_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise ValueError(f"missing evidence stream: {path}")
    with gzip.open(path, "rt", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _position(value: str) -> tuple[float, float]:
    try:
        parsed = ast.literal_eval(value)
        x, y = float(parsed[0]), float(parsed[1])
    except (ValueError, SyntaxError, TypeError, IndexError) as exc:
        raise ValueError(f"invalid obstacle position {value!r}") from exc
    if not math.isfinite(x) or not math.isfinite(y):
        raise ValueError(f"non-finite obstacle position {value!r}")
    return x, y


def _optional_float(value: object) -> float | None:
    if value in (None, ""):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _nearest_ground_truth(
    samples: list[tuple[float, float, float]],
    timestamps: list[float],
    stamp_s: float,
) -> tuple[float, float, float] | None:
    index = bisect_left(timestamps, stamp_s)
    candidates = [
        candidate
        for candidate in (index - 1, index)
        if 0 <= candidate < len(samples)
    ]
    if not candidates:
        return None
    nearest = min(candidates, key=lambda item: abs(samples[item][0] - stamp_s))
    result = samples[nearest]
    return result if abs(result[0] - stamp_s) <= 0.15 else None


def _trace_metrics(
    evidence_root: Path,
    actor_ids: Iterable[str],
    *,
    exposure_distance_m: float,
) -> dict[str, dict[str, Any]]:
    ground_truth_rows = _read_gzip_csv(evidence_root / "ground_truth.csv.gz")
    ground_truth = [
        (float(row["stamp_s"]), float(row["x"]), float(row["y"]))
        for row in ground_truth_rows
    ]
    ground_truth.sort()
    timestamps = [item[0] for item in ground_truth]
    obstacle_rows = _read_gzip_csv(
        evidence_root / "dynamic_obstacles.csv.gz"
    )
    results: dict[str, dict[str, Any]] = {}
    for actor_id in actor_ids:
        aligned: list[dict[str, Any]] = []
        states: set[str] = set()
        manager_clearances: list[float] = []
        for row in obstacle_rows:
            if row.get("id") != actor_id:
                continue
            stamp_s = _optional_float(row.get("stamp_s"))
            if stamp_s is None or not row.get("position"):
                continue
            robot = _nearest_ground_truth(ground_truth, timestamps, stamp_s)
            if robot is None:
                continue
            actor_x, actor_y = _position(row["position"])
            state = str(row.get("state", ""))
            states.add(state)
            manager_clearance = _optional_float(row.get("min_clearance_m"))
            if manager_clearance is not None:
                manager_clearances.append(manager_clearance)
            aligned.append(
                {
                    "stamp_s": stamp_s,
                    "state": state,
                    "distance_m": math.hypot(actor_x - robot[1], actor_y - robot[2]),
                    "speed_mps": _optional_float(row.get("velocity_mps")) or 0.0,
                    "progress": _optional_float(row.get("progress")) or 0.0,
                }
            )
        aligned.sort(key=lambda item: item["stamp_s"])
        active = [
            item for item in aligned if item["state"] in ACTIVE_DYNAMIC_STATES
        ]
        closing_speeds: list[float] = []
        time_to_collision: list[float] = []
        exposure_duration = 0.0
        for previous, current in zip(active, active[1:]):
            elapsed = current["stamp_s"] - previous["stamp_s"]
            if elapsed <= 0.0 or elapsed > 0.25:
                continue
            closing = (
                previous["distance_m"] - current["distance_m"]
            ) / elapsed
            closing_speeds.append(closing)
            if (
                closing > 0.05
                and current["distance_m"] <= 8.0
            ):
                time_to_collision.append(current["distance_m"] / closing)
            if (
                (previous["distance_m"] + current["distance_m"]) / 2.0
                <= exposure_distance_m
            ):
                exposure_duration += elapsed
        results[actor_id] = {
            "aligned_sample_count": len(aligned),
            "active_sample_count": len(active),
            "states": sorted(states),
            "minimum_center_distance_m": min(
                (item["distance_m"] for item in active), default=None
            ),
            "minimum_manager_clearance_m": min(
                manager_clearances, default=None
            ),
            "peak_speed_mps": max(
                (item["speed_mps"] for item in active), default=0.0
            ),
            "maximum_progress": max(
                (item["progress"] for item in active), default=0.0
            ),
            "maximum_relative_closing_speed_mps": max(
                closing_speeds, default=0.0
            ),
            "minimum_time_to_collision_sec": min(
                time_to_collision, default=None
            ),
            "exposure_distance_m": exposure_distance_m,
            "exposure_duration_sec": exposure_duration,
        }
    return results


def _static_encounter_coverage(
    records: list[dict[str, Any]], contract: Mapping[str, Any]
) -> dict[str, Any]:
    expected_ids = tuple(contract["expected_obstacle_ids"])
    maximum_distance = float(contract["maximum_encounter_center_distance_m"])
    minimum_clearance = float(contract["minimum_manager_clearance_m"])
    maximum_speed = float(contract["maximum_stationary_speed_mps"])
    runs: list[dict[str, Any]] = []
    for record in records:
        root = Path(record["summary_path"]).parent
        metrics = _trace_metrics(
            root, expected_ids, exposure_distance_m=maximum_distance
        )
        actor_gates: dict[str, dict[str, bool]] = {}
        for actor_id, actor in metrics.items():
            actor_gates[actor_id] = {
                "telemetry_observed": actor["aligned_sample_count"] > 0,
                "parked_state_observed": "parked" in actor["states"],
                "stationary_speed": actor["peak_speed_mps"] <= maximum_speed,
                "physical_encounter": (
                    actor["minimum_center_distance_m"] is not None
                    and actor["minimum_center_distance_m"] <= maximum_distance
                ),
                "non_negative_manager_clearance": (
                    actor["minimum_manager_clearance_m"] is not None
                    and actor["minimum_manager_clearance_m"] >= minimum_clearance
                ),
            }
        passed = bool(actor_gates) and all(
            all(gates.values()) for gates in actor_gates.values()
        )
        runs.append(
            {
                "run_index": record["run_index"],
                "seed": record["seed"],
                "actors": metrics,
                "actor_gates": actor_gates,
                "passed": passed,
            }
        )
    return {
        "expected_obstacle_ids": list(expected_ids),
        "maximum_encounter_center_distance_m": maximum_distance,
        "minimum_manager_clearance_m": minimum_clearance,
        "runs": runs,
        "passed_runs": sum(item["passed"] for item in runs),
        "passed": len(runs) == 20 and all(item["passed"] for item in runs),
    }


def _dynamic_threat_coverage(
    records: list[dict[str, Any]], contract: Mapping[str, Any]
) -> dict[str, Any]:
    actor_contracts = contract["actors"]
    actor_ids = tuple(actor_contracts)
    exposure_distance = float(contract["exposure_distance_m"])
    minimum_exposure = float(contract["minimum_exposure_duration_sec"])
    maximum_distance = float(contract["maximum_encounter_center_distance_m"])
    maximum_ttc = float(contract["maximum_time_to_collision_sec"])
    minimum_closing = float(contract["minimum_relative_closing_speed_mps"])
    minimum_clearance = float(contract["minimum_manager_clearance_m"])
    runs: list[dict[str, Any]] = []
    for record in records:
        root = Path(record["summary_path"]).parent
        metrics = _trace_metrics(
            root, actor_ids, exposure_distance_m=exposure_distance
        )
        actor_gates: dict[str, dict[str, bool]] = {}
        for actor_id, actor in metrics.items():
            specific = actor_contracts[actor_id]
            actor_gates[actor_id] = {
                "active_telemetry_observed": actor["active_sample_count"] > 0,
                "motion_state_observed": bool(
                    {"moving", "dwell", "clearing", "safety_yield"}
                    & set(actor["states"])
                ),
                "peak_speed": actor["peak_speed_mps"]
                >= float(specific["minimum_peak_speed_mps"]),
                "trajectory_progress": actor["maximum_progress"]
                >= float(specific["minimum_progress"]),
                "close_pairing": (
                    actor["minimum_center_distance_m"] is not None
                    and actor["minimum_center_distance_m"] <= maximum_distance
                ),
                "threat_exposure": actor["exposure_duration_sec"]
                >= minimum_exposure,
                "relative_closing_speed": (
                    actor["maximum_relative_closing_speed_mps"]
                    >= minimum_closing
                ),
                "time_to_collision": (
                    actor["minimum_time_to_collision_sec"] is not None
                    and actor["minimum_time_to_collision_sec"] <= maximum_ttc
                ),
                "non_negative_manager_clearance": (
                    actor["minimum_manager_clearance_m"] is not None
                    and actor["minimum_manager_clearance_m"] >= minimum_clearance
                ),
            }
        passed = bool(actor_gates) and all(
            all(gates.values()) for gates in actor_gates.values()
        )
        runs.append(
            {
                "run_index": record["run_index"],
                "seed": record["seed"],
                "actors": metrics,
                "actor_gates": actor_gates,
                "passed": passed,
            }
        )
    return {
        "expected_actor_ids": list(actor_ids),
        "runs": runs,
        "passed_runs": sum(item["passed"] for item in runs),
        "passed": len(runs) == 20 and all(item["passed"] for item in runs),
    }


def _timing_distribution(
    path: Path,
    *,
    baseline_field: str,
    treatment_field: str,
    quality_fields: tuple[str, ...],
) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    baseline = [float(row[baseline_field]) / 1.0e6 for row in rows]
    treatment = [float(row[treatment_field]) / 1.0e6 for row in rows]
    speedups = [left / right for left, right in zip(baseline, treatment)]
    quality = [
        max(float(row[field]) for field in quality_fields)
        for row in rows
    ]
    return {
        "sample_count": len(rows),
        "baseline_latency_p50_ms": _percentile(baseline, 0.50),
        "baseline_latency_p95_ms": _percentile(baseline, 0.95),
        "treatment_latency_p50_ms": _percentile(treatment, 0.50),
        "treatment_latency_p95_ms": _percentile(treatment, 0.95),
        "median_speedup_ratio": median(speedups),
        "maximum_quality_error": max(quality),
    }


def _compute_only_metrics(contract_summary: str | Path | None) -> dict[str, Any]:
    result: dict[str, Any] = {
        "scope": "compute_only",
        "gating_for_navigation_qualification": False,
        "prohibited_interpretation": (
            "navigation_success_or_end_to_end_speed_improvement"
        ),
    }
    if contract_summary is None:
        result["status"] = "NOT_SUPPLIED"
        return result
    source = Path(contract_summary).expanduser().resolve()
    payload = json.loads(source.read_text(encoding="utf-8"))
    result.update(
        {
            "status": "REPORTED_AS_SECONDARY",
            "source": str(source),
            "source_sha256": _sha256(source),
            "adaptation_compute_latency": _timing_distribution(
                source.parent / "convergence_paired.csv",
                baseline_field="classic_time_ns",
                treatment_field="cognitive_time_ns",
                quality_fields=("quality_max_abs_error",),
            ),
            "map_update_compute_latency": _timing_distribution(
                source.parent / "map_update_paired.csv",
                baseline_field="full_time_ns",
                treatment_field="incremental_time_ns",
                quality_fields=("sr_max_abs_error", "dr_max_abs_error"),
            ),
            "legacy_relative_percentages": {
                "adaptation_median_percent": payload.get(
                    "convergence", {}
                ).get("median_improvement_percent"),
                "map_update_median_percent": payload.get(
                    "map_update", {}
                ).get("median_improvement_percent"),
            },
            "timer_scope": {
                "adaptation": payload.get("convergence", {}).get("timer_scope"),
                "map_update": payload.get("map_update", {}).get("timer_scope"),
            },
        }
    )
    return result


def summarize(
    *,
    static_root: str | Path,
    dynamic_root: str | Path,
    appearance_root: str | Path,
    metric_contract: str | Path,
    contract_summary: str | Path | None = None,
) -> dict[str, Any]:
    contract_path = Path(metric_contract).expanduser().resolve()
    contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
    if contract.get("schema") != "bio_nav.final_rivermark_metric_contract.v1":
        raise ValueError("unsupported Final Rivermark metric contract")
    primary = contract["primary_navigation_metrics"]
    static_records = _load_records(static_root)
    dynamic_records = _load_records(dynamic_root)
    appearance_records = _load_records(appearance_root)
    static = _rate_group(
        static_records,
        name="static",
        required_rate_percent=100.0,
        require_path_deviation=True,
    )
    dynamic = _rate_group(
        dynamic_records,
        name="dynamic",
        required_rate_percent=100.0,
        require_dynamic_interaction=True,
    )
    appearance = _rate_group(
        appearance_records,
        name="appearance",
        required_rate_percent=100.0,
        require_appearance=True,
    )
    static_coverage = _static_encounter_coverage(
        static_records, primary["static"]
    )
    dynamic_threat = _dynamic_threat_coverage(
        dynamic_records, primary["dynamic"]
    )
    module2_runtime = _module2_runtime_consumption(dynamic_records)
    result = {
        "schema": "bio_nav.final_rivermark_qualification.v1",
        "status": "QUALIFICATION_EVALUATION",
        "metric_contract": {
            "path": str(contract_path),
            "sha256": _sha256(contract_path),
            "schema": contract["schema"],
        },
        "primary_navigation_metrics": {
            "static": static,
            "static_obstacle_encounter_coverage": static_coverage,
            "dynamic": dynamic,
            "dynamic_threat_coverage": dynamic_threat,
            "appearance": appearance,
        },
        "module2_runtime_consumption": module2_runtime,
        "module2_causality": {
            "scope": "external_v4_matched_four_arm",
            "gating_for_rivermark": False,
            "claim_from_3x20": False,
        },
        "secondary_compute_metrics": _compute_only_metrics(contract_summary),
    }
    result["passed"] = all(
        (
            static["passed"],
            static_coverage["passed"],
            dynamic["passed"],
            dynamic_threat["passed"],
            appearance["passed"],
            module2_runtime["passed"],
        )
    )
    result["status"] = "PASS" if result["passed"] else "STOP"
    return result


def _checksums_valid(root: Path) -> bool:
    manifest = root / "checksums.sha256"
    if not manifest.is_file():
        return False
    for line in manifest.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        try:
            digest, relative = line.split("  ", 1)
            candidate = root / relative
            candidate.resolve().relative_to(root.resolve())
        except (ValueError, OSError):
            return False
        if not candidate.is_file() or _sha256(candidate) != digest:
            return False
    return True


def pilot_check(
    *, group: str, root: str | Path, metric_contract: str | Path
) -> dict[str, Any]:
    if group not in {"static", "dynamic", "appearance"}:
        raise ValueError("pilot group must be static, dynamic or appearance")
    contract_path = Path(metric_contract).expanduser().resolve()
    contract_sha256 = _sha256(contract_path)
    records = _load_records(root)
    if len(records) != 1:
        raise ValueError(f"Final {group} pilot requires exactly one run")
    record = records[0]
    summary = record["summary"]
    evidence_root = Path(record["summary_path"]).parent
    metric_gate = summary.get("final_trial_metric_gate", {})
    gates = {
        "scenario_identity": record["scenario_id"]
        == f"final_rivermark_{group}",
        "trial_dispatched": (
            evidence_root / "TRIAL_DISPATCHED.json"
        ).is_file(),
        "strict_success": summary.get("strict_success") is True,
        "physical_collision_free": summary.get("physical_collision_free")
        is True,
        "data_complete": summary.get("data_complete") is True,
        "checksums_declared": summary.get("checksums_verified") is True,
        "checksums_recomputed": _checksums_valid(evidence_root),
        "five_waypoint_contract": tuple(
            str(leg.get("id"))
            for leg in summary.get("legs", [])
            if isinstance(leg, Mapping)
        )
        == ("G1", "G2", "G3", "G4", "G5"),
        "final_metric_gate": bool(
            isinstance(metric_gate, Mapping)
            and metric_gate.get("applicable") is True
            and metric_gate.get("passed") is True
            and metric_gate.get("contract_sha256") == contract_sha256
        ),
    }
    return {
        "schema": "bio_nav.final_rivermark_pilot.v1",
        "group": group,
        "run_index": record["run_index"],
        "seed": record["seed"],
        "evidence_root": str(evidence_root),
        "metric_contract": {
            "path": str(contract_path),
            "sha256": contract_sha256,
        },
        "gates": gates,
        "metric_gate": metric_gate,
        "passed": all(gates.values()),
        "status": "PASS" if all(gates.values()) else "STOP",
    }


def _write_csv(result: Mapping[str, Any], target: Path) -> tuple[Path, Path]:
    group_path = target.with_name("group_summary.csv")
    run_path = target.with_name("interaction_metrics.csv")
    primary = result["primary_navigation_metrics"]
    with group_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            ("group", "passed", "strict_successes", "collision_free_runs")
        )
        for group in ("static", "dynamic", "appearance"):
            item = primary[group]
            writer.writerow(
                (
                    group,
                    item["passed"],
                    item["strict_successes"],
                    item["collision_free_runs"],
                )
            )
    with run_path.open("w", newline="", encoding="utf-8") as stream:
        fields = (
            "group",
            "run_index",
            "seed",
            "actor_id",
            "passed",
            "minimum_center_distance_m",
            "minimum_manager_clearance_m",
            "peak_speed_mps",
            "maximum_progress",
            "maximum_relative_closing_speed_mps",
            "minimum_time_to_collision_sec",
            "exposure_duration_sec",
        )
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for group, key in (
            ("static", "static_obstacle_encounter_coverage"),
            ("dynamic", "dynamic_threat_coverage"),
        ):
            for run in primary[key]["runs"]:
                for actor_id, actor in run["actors"].items():
                    writer.writerow(
                        {
                            "group": group,
                            "run_index": run["run_index"],
                            "seed": run["seed"],
                            "actor_id": actor_id,
                            "passed": all(
                                run["actor_gates"][actor_id].values()
                            ),
                            **{
                                field: actor.get(field)
                                for field in fields
                                if field in actor
                            },
                        }
                    )
    return group_path, run_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--static-root", required=True)
    parser.add_argument("--dynamic-root", required=True)
    parser.add_argument("--appearance-root", required=True)
    parser.add_argument("--metric-contract", required=True)
    parser.add_argument("--contract-summary")
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    result = summarize(
        static_root=arguments.static_root,
        dynamic_root=arguments.dynamic_root,
        appearance_root=arguments.appearance_root,
        metric_contract=arguments.metric_contract,
        contract_summary=arguments.contract_summary,
    )
    target = Path(arguments.output).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    group_path, run_path = _write_csv(result, target)
    checksum_path = target.with_name("checksums.sha256")
    checksum_path.write_text(
        "\n".join(
            f"{_sha256(path)}  {path.name}"
            for path in (target, group_path, run_path)
        )
        + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "output": str(target),
                "status": result["status"],
                "checksums": str(checksum_path),
            }
        )
    )
    raise SystemExit(0 if result["passed"] else 2)


def pilot_main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate one isolated Final Rivermark pilot run"
    )
    parser.add_argument("--group", required=True)
    parser.add_argument("--root", required=True)
    parser.add_argument("--metric-contract", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    result = pilot_check(
        group=arguments.group,
        root=arguments.root,
        metric_contract=arguments.metric_contract,
    )
    target = Path(arguments.output).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    target.with_name("checksums.sha256").write_text(
        f"{_sha256(target)}  {target.name}\n", encoding="utf-8"
    )
    print(json.dumps({"output": str(target), "status": result["status"]}))
    raise SystemExit(0 if result["passed"] else 2)


if __name__ == "__main__":
    main()
