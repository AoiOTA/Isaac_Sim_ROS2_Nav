"""Fail-closed Attempt31 Rivermark qualification summarizer."""

from __future__ import annotations

import argparse
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
from statistics import median
from typing import Any, Mapping


APPEARANCE_PROFILES = ("dim_warm", "dim_cool", "bright_warm", "bright_cool")
MISSION_WAYPOINTS = ("G1", "G2", "G3", "G4", "G5")


def _load_records(root: str | Path) -> list[dict[str, Any]]:
    directory = Path(root).expanduser().resolve()
    paths = sorted(directory.rglob("run_summary.json"))
    if not paths:
        raise ValueError(f"no run_summary.json under {directory}")
    records: list[dict[str, Any]] = []
    identities: set[tuple[str, int, int]] = set()
    for summary_path in paths:
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        manifest_path = summary_path.with_name("run_manifest.json")
        if not isinstance(summary, Mapping) or not manifest_path.is_file():
            raise ValueError(f"incomplete run evidence at {summary_path.parent}")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(manifest, Mapping):
            raise ValueError(f"invalid manifest at {manifest_path}")
        identity = (
            str(manifest.get("scenario_id", "")),
            int(manifest.get("run_index", 0)),
            int(manifest.get("random_seed", -1)),
        )
        if identity in identities:
            raise ValueError(f"duplicate run identity {identity}")
        identities.add(identity)
        records.append(
            {
                "summary_path": str(summary_path),
                "summary": dict(summary),
                "manifest": dict(manifest),
                "scenario_id": identity[0],
                "run_index": identity[1],
                "seed": identity[2],
            }
        )
    return records


def _rate_group(
    records: list[dict[str, Any]],
    *,
    name: str,
    required_rate_percent: float,
    require_path_deviation: bool = False,
    require_dynamic_interaction: bool = False,
    require_appearance: bool = False,
) -> dict[str, Any]:
    if len(records) != 20:
        raise ValueError(f"{name} requires exactly 20 runs, got {len(records)}")
    summaries = [item["summary"] for item in records]
    strict_successes = sum(bool(item.get("strict_success")) for item in summaries)
    collision_free = sum(bool(item.get("physical_collision_free")) for item in summaries)
    complete = sum(
        bool(item.get("data_complete")) and bool(item.get("checksums_verified"))
        for item in summaries
    )
    five_waypoint_runs = sum(
        tuple(
            str(leg.get("id")) for leg in item.get("legs", [])
            if isinstance(leg, Mapping)
        ) == MISSION_WAYPOINTS
        for item in summaries
    )
    rate = strict_successes / len(records) * 100.0
    collision_free_rate = collision_free / len(records) * 100.0
    result: dict[str, Any] = {
        "run_count": len(records),
        "strict_successes": strict_successes,
        "success_rate_percent": rate,
        "collision_free_runs": collision_free,
        "collision_free_rate_percent": collision_free_rate,
        "complete_evidence_runs": complete,
        "five_waypoint_runs": five_waypoint_runs,
        "required_rate_percent": required_rate_percent,
    }
    gates = {
        "run_count": len(records) == 20,
        "success_rate": rate >= required_rate_percent,
        "collision_free_rate": collision_free_rate >= required_rate_percent,
        "evidence_complete": complete == len(records),
        "five_waypoint_contract": five_waypoint_runs == len(records),
    }
    if require_path_deviation:
        deviations = [item.get("path_deviation_percent") for item in summaries]
        finite = [
            float(value)
            for value in deviations
            if isinstance(value, (int, float)) and math.isfinite(float(value))
        ]
        result["path_deviation_percent"] = {
            "values": finite,
            "maximum": max(finite) if finite else None,
            "median": median(finite) if finite else None,
            "required_maximum": 20.0,
        }
        gates["static_path_deviation"] = (
            len(finite) == len(records) and max(finite) <= 20.0
        )
    if require_dynamic_interaction:
        dynamic_complete = sum(
            bool(item.get("dynamic_interaction_complete")) for item in summaries
        )
        expected_four = sum(
            len(item["manifest"].get("dynamic_interaction", {}).get("expected_ids", []))
            == 4
            for item in records
        )
        four_stage_case = sum(
            item["manifest"].get("dynamic_selection", {}).get("case_id")
            == "full_route_four_stage"
            for item in records
        )
        close_four = 0
        for item in records:
            interaction = item["manifest"].get("dynamic_interaction", {})
            expected_ids = interaction.get("expected_ids", [])
            clearances = interaction.get("minimum_clearance_m_by_actor", {})
            if (
                isinstance(expected_ids, list)
                and len(expected_ids) == 4
                and len(set(expected_ids)) == 4
                and isinstance(clearances, Mapping)
                and all(
                    isinstance(clearances.get(identifier), (int, float))
                    and math.isfinite(float(clearances[identifier]))
                    and 0.0 <= float(clearances[identifier]) <= 1.5
                    for identifier in expected_ids
                )
            ):
                close_four += 1
        result["dynamic_interaction_complete_runs"] = dynamic_complete
        result["dynamic_interaction_complete_rate_percent"] = (
            dynamic_complete / len(records) * 100.0
        )
        result["four_actor_contract_runs"] = expected_four
        result["four_stage_case_runs"] = four_stage_case
        result["four_actor_close_interaction_runs"] = close_four
        result["four_actor_close_interaction_rate_percent"] = (
            close_four / len(records) * 100.0
        )
        result["maximum_pairing_clearance_m"] = 1.5
        gates["dynamic_interaction"] = (
            result["dynamic_interaction_complete_rate_percent"]
            >= required_rate_percent
        )
        gates["four_actor_contract"] = expected_four == len(records)
        gates["four_stage_case"] = four_stage_case == len(records)
        gates["four_actor_close_interaction"] = (
            result["four_actor_close_interaction_rate_percent"]
            >= required_rate_percent
        )
    if require_appearance:
        profiles = Counter(
            str(item.get("appearance_profile_id")) for item in summaries
        )
        applied = 0
        for item in records:
            appearance = item["manifest"].get("appearance", {})
            state = appearance.get("state", {}) if isinstance(appearance, Mapping) else {}
            counts = state.get("applied_counts", {}) if isinstance(state, Mapping) else {}
            if (
                isinstance(appearance, Mapping)
                and appearance.get("ready") is True
                and isinstance(counts, Mapping)
                and int(counts.get("lights", 0)) > 0
                and int(counts.get("material_color_inputs", 0)) > 0
            ):
                applied += 1
        result["appearance_profile_counts"] = dict(sorted(profiles.items()))
        result["appearance_applied_runs"] = applied
        gates["appearance_balance"] = profiles == Counter(
            {profile: 5 for profile in APPEARANCE_PROFILES}
        )
        gates["appearance_applied"] = applied == len(records)
    result["gates"] = gates
    result["passed"] = all(gates.values())
    return result


def _module2_runtime_consumption(
    records: list[dict[str, Any]],
) -> dict[str, Any]:
    """Prove bounded runtime consumption without making a causal claim."""
    if len(records) != 20:
        raise ValueError(
            f"Module2 runtime consumption requires exactly 20 runs, got {len(records)}"
        )
    run_results: list[dict[str, Any]] = []
    model_ids: set[str] = set()
    for item in records:
        manifest = item["manifest"]
        prior_samples = manifest.get("planning_prior_samples", [])
        canonical_routes = manifest.get("canonical_routes", [])
        cost_snapshots = manifest.get("route_edge_costs", [])
        if not all(
            isinstance(value, list)
            for value in (prior_samples, canonical_routes, cost_snapshots)
        ):
            raise ValueError(
                f"invalid Module2 runtime evidence for run {item['run_index']}"
            )

        healthy_samples = 0
        tile_ids: set[str] = set()
        run_model_ids: set[str] = set()
        for sample in prior_samples:
            if not isinstance(sample, Mapping):
                continue
            if sample.get("module2_healthy") is True:
                healthy_samples += 1
            tile_id = sample.get("cognitive_tile_id")
            if isinstance(tile_id, str) and tile_id:
                tile_ids.add(tile_id)
            model_id = sample.get("model_id")
            if isinstance(model_id, str) and model_id:
                run_model_ids.add(model_id)
                model_ids.add(model_id)

        positive_applied = 0
        applied_total = 0.0
        cap_violations = 0
        cost_identity_violations = 0
        for snapshot in cost_snapshots:
            edges = snapshot.get("edges", []) if isinstance(snapshot, Mapping) else []
            for edge in edges:
                if not isinstance(edge, Mapping):
                    continue
                requested = float(edge.get("requested_module2_delta_m", 0.0))
                applied = float(edge.get("applied_module2_delta_m", 0.0))
                structural = float(edge.get("structural_cost_m", 0.0))
                runtime = float(edge.get("runtime_penalty_m", 0.0))
                final = float(edge.get("final_cost_m", 0.0))
                if applied > 0.0:
                    positive_applied += 1
                    applied_total += applied
                if applied > requested + 1e-6:
                    cap_violations += 1
                if abs(structural + runtime + applied - final) > 1e-5:
                    cost_identity_violations += 1

        aligned_snapshots = 0
        route_snapshots_affected = 0
        for route, snapshot in zip(canonical_routes, cost_snapshots):
            if not isinstance(route, Mapping) or not isinstance(snapshot, Mapping):
                continue
            if route.get("request_id") == snapshot.get("request_id"):
                aligned_snapshots += 1
            route_edge_ids = {
                int(edge_id) for edge_id in route.get("edge_ids", [])
                if isinstance(edge_id, int)
            }
            if any(
                isinstance(edge, Mapping)
                and edge.get("edge_id") in route_edge_ids
                and float(edge.get("applied_module2_delta_m", 0.0)) > 0.0
                for edge in snapshot.get("edges", [])
            ):
                route_snapshots_affected += 1

        model_identity_valid = bool(run_model_ids) and all(
            not model_id.startswith("geometry_only:")
            for model_id in run_model_ids
        )
        run_results.append({
            "run_index": item["run_index"],
            "healthy_prior_samples": healthy_samples,
            "prior_samples": len(prior_samples),
            "cognitive_tile_count": len(tile_ids),
            "positive_applied_edge_records": positive_applied,
            "applied_delta_total_m": applied_total,
            "route_snapshot_count": len(canonical_routes),
            "aligned_route_cost_snapshots": aligned_snapshots,
            "route_snapshots_with_applied_prior": route_snapshots_affected,
            "cap_violations": cap_violations,
            "cost_identity_violations": cost_identity_violations,
            "model_ids": sorted(run_model_ids),
            "model_identity_valid": model_identity_valid,
        })

    gates = {
        "twenty_runs": len(run_results) == 20,
        "healthy_samples_every_run": all(
            item["healthy_prior_samples"] > 0 for item in run_results
        ),
        "region_switch_every_run": all(
            item["cognitive_tile_count"] > 1 for item in run_results
        ),
        "positive_delta_applied_every_run": all(
            item["positive_applied_edge_records"] > 0 for item in run_results
        ),
        "selected_route_cost_affected_every_run": all(
            item["route_snapshots_with_applied_prior"] > 0
            for item in run_results
        ),
        "request_cost_snapshots_aligned": all(
            item["route_snapshot_count"] > 0
            and item["aligned_route_cost_snapshots"] == item["route_snapshot_count"]
            for item in run_results
        ),
        "bounded_cost_composition": all(
            item["cap_violations"] == 0
            and item["cost_identity_violations"] == 0
            for item in run_results
        ),
        "module2_model_identity": all(
            item["model_identity_valid"] for item in run_results
        ),
    }
    return {
        "scope": "runtime_consumption_only",
        "causal_claim": False,
        "run_count": len(run_results),
        "runs_with_healthy_prior_samples": sum(
            item["healthy_prior_samples"] > 0 for item in run_results
        ),
        "runs_with_region_switch": sum(
            item["cognitive_tile_count"] > 1 for item in run_results
        ),
        "runs_with_positive_applied_delta": sum(
            item["positive_applied_edge_records"] > 0 for item in run_results
        ),
        "runs_with_selected_route_cost_affected": sum(
            item["route_snapshots_with_applied_prior"] > 0
            for item in run_results
        ),
        "minimum_healthy_prior_samples": min(
            item["healthy_prior_samples"] for item in run_results
        ),
        "minimum_cognitive_tile_count": min(
            item["cognitive_tile_count"] for item in run_results
        ),
        "minimum_positive_applied_edge_records": min(
            item["positive_applied_edge_records"] for item in run_results
        ),
        "minimum_route_snapshots_with_applied_prior": min(
            item["route_snapshots_with_applied_prior"] for item in run_results
        ),
        "total_applied_delta_m": sum(
            item["applied_delta_total_m"] for item in run_results
        ),
        "model_ids": sorted(model_ids),
        "gates": gates,
        "runs": run_results,
        "passed": all(gates.values()),
    }


def summarize(
    *,
    static_root: str | Path,
    dynamic_root: str | Path,
    appearance_root: str | Path,
    contract_summary: str | Path,
) -> dict[str, Any]:
    contracts = json.loads(Path(contract_summary).read_text(encoding="utf-8"))
    static_records = _load_records(static_root)
    dynamic_records = _load_records(dynamic_root)
    appearance_records = _load_records(appearance_root)

    def evidence_index(records: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [
            {
                "scenario_id": item["scenario_id"],
                "run_index": item["run_index"],
                "seed": item["seed"],
                "summary_path": item["summary_path"],
            }
            for item in records
        ]

    convergence = contracts.get("convergence", {})
    map_update = contracts.get("map_update", {})
    contract_gates = {
        "convergence_20_cases": convergence.get("case_count") == 20,
        "convergence_quality": convergence.get("quality_pass") is True,
        "convergence_20_percent": (
            isinstance(convergence.get("minimum_improvement_percent"), (int, float))
            and float(convergence["minimum_improvement_percent"]) >= 20.0
            and convergence.get("target_pass") is True
        ),
        "map_update_20_repeats": map_update.get("repeat_count") == 20,
        "map_update_quality": map_update.get("quality_pass") is True,
        "map_update_30_percent": (
            isinstance(map_update.get("minimum_improvement_percent"), (int, float))
            and float(map_update["minimum_improvement_percent"]) >= 30.0
            and map_update.get("target_pass") is True
        ),
        "parent_child_smoke": bool(
            contracts.get("runtime_parent_child_smoke", {}).get("pass")
        ),
    }
    result = {
        "schema": "bio_nav.attempt31.rivermark_qualification.v3",
        "status": "QUALIFICATION_EVALUATION",
        "static": _rate_group(
            static_records, name="static", required_rate_percent=95.0,
            require_path_deviation=True,
        ),
        "dynamic": _rate_group(
            dynamic_records, name="dynamic", required_rate_percent=90.0,
            require_dynamic_interaction=True,
        ),
        "appearance": _rate_group(
            appearance_records, name="appearance", required_rate_percent=90.0,
            require_appearance=True,
        ),
        # Per user scope, Rivermark evaluates the 3x20 outdoor navigation
        # campaign only.  Module2 four-arm causality is owned by a later V4
        # campaign and must not be inferred from these outdoor runs.
        "module2_causality": {
            "scope": "external_v4",
            "status": "DEFERRED_TO_V4",
            "gating_for_rivermark": False,
        },
        # This only proves that the bounded V3.10 prior reached and changed
        # Route Server cost snapshots.  OFF/SR/DR/SRDR causal attribution is
        # deliberately deferred to the external V4 campaign above.
        "module2_runtime_consumption": _module2_runtime_consumption(
            dynamic_records
        ),
        "evidence_index": {
            "static": evidence_index(static_records),
            "dynamic": evidence_index(dynamic_records),
            "appearance": evidence_index(appearance_records),
        },
        "learning_and_update_contracts": {
            "source": str(Path(contract_summary).resolve()),
            "convergence": contracts.get("convergence"),
            "map_update": contracts.get("map_update"),
            "runtime_parent_child_smoke": contracts.get("runtime_parent_child_smoke"),
            "gates": contract_gates,
            "passed": all(contract_gates.values()),
        },
    }
    result["passed"] = all(
        result[key]["passed"]
        for key in (
            "static", "dynamic", "appearance",
            "learning_and_update_contracts", "module2_runtime_consumption",
        )
    )
    result["status"] = "PASS" if result["passed"] else "STOP"
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--static-root", required=True)
    parser.add_argument("--dynamic-root", required=True)
    parser.add_argument("--appearance-root", required=True)
    parser.add_argument("--contract-summary", required=True)
    parser.add_argument("--output", required=True)
    arguments = parser.parse_args()
    result = summarize(
        static_root=arguments.static_root,
        dynamic_root=arguments.dynamic_root,
        appearance_root=arguments.appearance_root,
        contract_summary=arguments.contract_summary,
    )
    target = Path(arguments.output).expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    csv_path = target.with_suffix(".csv")
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("group", "passed", "success_rate_percent", "collision_free_rate_percent"))
        for group in ("static", "dynamic", "appearance"):
            item = result[group]
            writer.writerow((
                group,
                item["passed"],
                item["success_rate_percent"],
                item["collision_free_rate_percent"],
            ))
    checksum_path = target.with_name("checksums.sha256")
    checksum_lines = []
    for artifact in (target, csv_path):
        digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
        checksum_lines.append(f"{digest}  {artifact.name}")
    checksum_path.write_text("\n".join(checksum_lines) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(target),
        "checksums": str(checksum_path),
        "status": result["status"],
    }))
    raise SystemExit(0 if result["passed"] else 2)


if __name__ == "__main__":
    main()
