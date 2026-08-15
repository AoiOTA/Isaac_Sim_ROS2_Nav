#!/usr/bin/env python3
"""Summarize matched Baseline/SR-only/DR-only/SRDR closed-loop evidence."""

from __future__ import annotations

import argparse
from collections import defaultdict
import csv
import json
import math
from pathlib import Path
from statistics import mean, median

import matplotlib.pyplot as plt
import numpy as np

from robot_route_planner.defaults import load_engineering_defaults
from robot_route_planner.map_io import load_occupancy_map
from robot_route_planner.visualize import _map_background


ARMS = ("baseline", "sr_only", "dr_only", "srdr")
ARM_LABELS = {
    "baseline": "Baseline",
    "sr_only": "SR-only",
    "dr_only": "DR-only",
    "srdr": "SRDR",
}
COLORS = {
    "baseline": "#546e7a",
    "sr_only": "#00a152",
    "dr_only": "#f57c00",
    "srdr": "#8e24aa",
}


def _initial_route(value: dict) -> list[int]:
    history = value.get("route_history", [])
    return [int(item) for item in history[0]["edge_ids"]] if history else []


def _is_suffix(candidate: list[int], route: list[int]) -> bool:
    return len(candidate) <= len(route) and candidate == route[len(route) - len(candidate):]


def _row(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    diagnostics = value.get("srdr_edge_diagnostic_history", [])
    edge_priors = value.get("edge_prior_history", [])
    planning_priors = value.get("planning_prior_history", [])
    arm = str(value.get("experiment_arm", ""))
    if arm not in ARMS:
        raise RuntimeError(f"{path}: invalid experiment_arm {arm!r}")
    positive_sr = max(
        (int(item.get("positive_sr_count", 0)) for item in diagnostics), default=0
    )
    positive_dr = max(
        (int(item.get("positive_dr_count", 0)) for item in diagnostics), default=0
    )
    simultaneous = any(
        int(item.get("positive_sr_count", 0)) > 0
        and int(item.get("positive_dr_count", 0)) > 0
        for item in diagnostics
    )
    return {
        "path": str(path),
        "arm": arm,
        "query_id": str(value.get("query_id", "")),
        "variant_id": str(value.get("dynamic_variant_id", "")),
        "dynamic_case_id": str(value.get("dynamic_case_id", "")),
        "ros_domain_id": int(value.get("ros_domain_id", 0)),
        "completed": bool(value.get("completed", False)),
        "failed": bool(value.get("failed", False)),
        "physical_collision": bool(value.get("physical_collision", False)),
        "sampled_static_footprint_collisions": int(
            value.get("sampled_static_footprint_collisions", 0)
        ),
        "elapsed_s": float(value.get("elapsed_s", 0.0)),
        "travelled_distance_m": float(value.get("travelled_distance_m", 0.0)),
        "final_goal_error_m": float(value.get("final_goal_error_m", 0.0)),
        "initial_route_edge_ids": _initial_route(value),
        "route_edge_id_history": [
            [int(edge_id) for edge_id in item["edge_ids"]]
            for item in value.get("route_history", [])
        ],
        "route_update_count": len(value.get("route_history", [])),
        "planning_prior_count": len(planning_priors),
        "healthy_planning_prior_count": sum(
            bool(item.get("module2_healthy", False)) for item in planning_priors
        ),
        "edge_prior_count": len(edge_priors),
        "healthy_edge_prior_count": sum(
            bool(item.get("healthy", False)) for item in edge_priors
        ),
        "positive_sr_edge_count": positive_sr,
        "positive_dr_edge_count": positive_dr,
        "simultaneous_sr_dr": simultaneous,
        "obstacle_center_min_distance_m": float(
            value.get("obstacle_center_min_distance_m", float("nan"))
        ),
        "obstacle_center_exposure_s_under_1m": float(
            value.get("obstacle_center_exposure_s_under_1m", 0.0)
        ),
        "obstacle_center_exposure_s_under_1_5m": float(
            value.get("obstacle_center_exposure_s_under_1_5m", 0.0)
        ),
        "trajectory_xy": [item[:2] for item in value.get("trajectory_xy_zw", [])],
    }


def _bootstrap_median_ci(
    values: list[float], *, seed_text: str, resamples: int = 20_000
) -> list[float] | None:
    if not values:
        return None
    data = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(data)):
        return None
    seed = sum((index + 1) * byte for index, byte in enumerate(seed_text.encode()))
    generator = np.random.default_rng(seed)
    indices = generator.integers(0, len(data), size=(resamples, len(data)))
    samples = np.median(data[indices], axis=1)
    return [float(item) for item in np.quantile(samples, [0.025, 0.975])]


def _one_sided_sign_test_p(values: list[float], *, beneficial_sign: int) -> float | None:
    non_ties = [value for value in values if not math.isclose(value, 0.0, abs_tol=1e-12)]
    if not non_ties:
        return None
    beneficial = sum(value * beneficial_sign > 0.0 for value in non_ties)
    count = len(non_ties)
    return sum(math.comb(count, index) for index in range(beneficial, count + 1)) / (2**count)


def _paired_effects(rows: list[dict]) -> list[dict]:
    by_pair = {(row["query_id"], row["variant_id"], row["arm"]): row for row in rows}
    records = []
    for query_id in sorted({row["query_id"] for row in rows}):
        variants = sorted({row["variant_id"] for row in rows if row["query_id"] == query_id})
        for arm in ARMS[1:]:
            pairs = [
                (
                    by_pair[(query_id, variant, "baseline")],
                    by_pair[(query_id, variant, arm)],
                )
                for variant in variants
                if (query_id, variant, "baseline") in by_pair
                and (query_id, variant, arm) in by_pair
            ]
            if not pairs:
                continue
            clearance_deltas = [
                candidate["obstacle_center_min_distance_m"]
                - reference["obstacle_center_min_distance_m"]
                for reference, candidate in pairs
            ]
            exposure_deltas = [
                candidate["obstacle_center_exposure_s_under_1_5m"]
                - reference["obstacle_center_exposure_s_under_1_5m"]
                for reference, candidate in pairs
            ]
            distance_deviations = [candidate["distance_deviation_pct"] for _, candidate in pairs]
            records.append({
                "query_id": query_id,
                "arm": arm,
                "matched_pair_count": len(pairs),
                "route_change_count": sum(candidate["route_changed_vs_baseline"] for _, candidate in pairs),
                "route_change_rate": mean(candidate["route_changed_vs_baseline"] for _, candidate in pairs),
                "success_rate_delta": mean(
                    int(candidate["completed"] and not candidate["failed"])
                    - int(reference["completed"] and not reference["failed"])
                    for reference, candidate in pairs
                ),
                "collision_free_rate_delta": mean(
                    int(not candidate["physical_collision"] and candidate["sampled_static_footprint_collisions"] == 0)
                    - int(not reference["physical_collision"] and reference["sampled_static_footprint_collisions"] == 0)
                    for reference, candidate in pairs
                ),
                "median_distance_deviation_pct": median(distance_deviations),
                "distance_deviation_pct_bootstrap_95_ci": _bootstrap_median_ci(
                    distance_deviations, seed_text=f"{query_id}:{arm}:distance"
                ),
                "all_distance_deviations_within_20pct": all(
                    candidate["distance_within_20pct"] for _, candidate in pairs
                ),
                "median_min_actor_distance_delta_m": median(clearance_deltas),
                "min_actor_distance_delta_m_bootstrap_95_ci": _bootstrap_median_ci(
                    clearance_deltas, seed_text=f"{query_id}:{arm}:clearance"
                ),
                "min_actor_distance_one_sided_sign_p": _one_sided_sign_test_p(
                    clearance_deltas, beneficial_sign=1
                ),
                "median_exposure_delta_s_under_1_5m": median(exposure_deltas),
                "exposure_delta_s_bootstrap_95_ci": _bootstrap_median_ci(
                    exposure_deltas, seed_text=f"{query_id}:{arm}:exposure"
                ),
                "exposure_reduction_one_sided_sign_p": _one_sided_sign_test_p(
                    exposure_deltas, beneficial_sign=-1
                ),
                "dynamic_risk_benefit_count": sum(
                    clearance_delta > 0.0 and exposure_delta < 0.0
                    for clearance_delta, exposure_delta in zip(clearance_deltas, exposure_deltas)
                ),
            })
    return records


def _factor_gate(row: dict) -> bool:
    if row["arm"] == "baseline":
        return (
            row["edge_prior_count"] == 0
            and row["positive_sr_edge_count"] == 0
            and row["positive_dr_edge_count"] == 0
        )
    if row["healthy_edge_prior_count"] < 1:
        return False
    if row["arm"] == "sr_only":
        return row["positive_sr_edge_count"] > 0 and row["positive_dr_edge_count"] == 0
    if row["arm"] == "dr_only":
        return row["positive_sr_edge_count"] == 0 and row["positive_dr_edge_count"] > 0
    return row["simultaneous_sr_dr"]


def _aggregate(rows: list[dict]) -> list[dict]:
    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for row in rows:
        groups[(row["query_id"], row["arm"])].append(row)
    records = []
    for (query_id, arm), items in sorted(groups.items()):
        records.append({
            "query_id": query_id,
            "arm": arm,
            "n": len(items),
            "success_rate": mean(
                row["completed"] and not row["failed"] for row in items
            ),
            "collision_free_rate": mean(
                not row["physical_collision"]
                and row["sampled_static_footprint_collisions"] == 0
                for row in items
            ),
            "factor_gate_rate": mean(row["factor_gate_pass"] for row in items),
            "route_changed_vs_baseline_rate": mean(
                row["route_changed_vs_baseline"] for row in items
            ),
            "distance_within_20pct_rate": mean(
                row["distance_within_20pct"] for row in items
            ),
            "mean_distance_deviation_pct": mean(
                row["distance_deviation_pct"] for row in items
            ),
            "median_travelled_distance_m": median(
                row["travelled_distance_m"] for row in items
            ),
            "median_elapsed_s": median(row["elapsed_s"] for row in items),
            "median_obstacle_center_min_distance_m": median(
                row["obstacle_center_min_distance_m"] for row in items
            ),
            "total_obstacle_exposure_s_under_1_5m": sum(
                row["obstacle_center_exposure_s_under_1_5m"] for row in items
            ),
        })
    return records


def _render(rows: list[dict], occupancy, output: Path) -> None:
    query_ids = sorted({row["query_id"] for row in rows})
    figure, axes = plt.subplots(
        len(query_ids), 3, figsize=(18, 6 * len(query_ids)), squeeze=False,
        constrained_layout=True,
    )
    for row_index, query_id in enumerate(query_ids):
        selected = [row for row in rows if row["query_id"] == query_id]
        trajectory_axis, risk_axis, distance_axis = axes[row_index]
        _map_background(trajectory_axis, occupancy)
        for arm in ARMS:
            arm_rows = [row for row in selected if row["arm"] == arm]
            for index, row in enumerate(arm_rows):
                trajectory = np.asarray(row["trajectory_xy"], dtype=float)
                if trajectory.size:
                    trajectory_axis.plot(
                        trajectory[:, 0], trajectory[:, 1],
                        color=COLORS[arm], alpha=0.35, linewidth=1.3,
                        label=ARM_LABELS[arm] if index == 0 else None,
                    )
        trajectory_axis.set_title(f"{query_id}: actual trajectories")
        trajectory_axis.legend(loc="upper left")

        positions = np.arange(len(ARMS))
        risk_data = [
            [row["obstacle_center_min_distance_m"] for row in selected if row["arm"] == arm]
            for arm in ARMS
        ]
        risk_axis.boxplot(risk_data, positions=positions, widths=0.6)
        risk_axis.set_xticks(positions, [ARM_LABELS[item] for item in ARMS])
        risk_axis.set_ylabel("minimum actor-centre distance [m]")
        risk_axis.set_title(f"{query_id}: dynamic-risk separation")
        risk_axis.grid(axis="y", alpha=0.25)

        distance_data = [
            [row["distance_deviation_pct"] for row in selected if row["arm"] == arm]
            for arm in ARMS
        ]
        distance_axis.boxplot(distance_data, positions=positions, widths=0.6)
        distance_axis.axhspan(-20.0, 20.0, color="#66bb6a", alpha=0.12)
        distance_axis.axhline(0.0, color="#455a64", linewidth=1.0)
        distance_axis.set_xticks(positions, [ARM_LABELS[item] for item in ARMS])
        distance_axis.set_ylabel("actual-distance deviation vs matched Baseline [%]")
        distance_axis.set_title(f"{query_id}: user-defined ±20% gate")
        distance_axis.grid(axis="y", alpha=0.25)
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-root", type=Path, required=True)
    parser.add_argument("--map", dest="map_path", type=Path, required=True)
    parser.add_argument("--defaults", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()

    paths = sorted(args.runtime_root.rglob("closed_loop.json"))
    rows = [_row(path) for path in paths]
    if not rows:
        raise RuntimeError("no four-arm closed-loop evidence found")
    baseline = {
        (row["query_id"], row["variant_id"]): row
        for row in rows if row["arm"] == "baseline"
    }
    duplicates = len(baseline) != sum(row["arm"] == "baseline" for row in rows)
    if duplicates:
        raise RuntimeError("duplicate matched Baseline trials")
    for row in rows:
        key = (row["query_id"], row["variant_id"])
        if key not in baseline:
            raise RuntimeError(f"missing matched Baseline for {key}")
        reference = baseline[key]
        denominator = max(reference["travelled_distance_m"], 1.0e-9)
        row["baseline_travelled_distance_m"] = reference["travelled_distance_m"]
        row["distance_deviation_pct"] = 100.0 * (
            row["travelled_distance_m"] - denominator
        ) / denominator
        row["distance_within_20pct"] = abs(row["distance_deviation_pct"]) <= 20.0
        baseline_route = reference["initial_route_edge_ids"]
        row["route_changed_vs_baseline"] = any(
            not _is_suffix(route, baseline_route)
            for route in row["route_edge_id_history"]
            if route
        )
        row["factor_gate_pass"] = _factor_gate(row)

    public_rows = [
        {
            key: value
            for key, value in row.items()
            if key not in {"path", "trajectory_xy", "route_edge_id_history"}
        }
        for row in rows
    ]
    aggregate = _aggregate(rows)
    paired_effects = _paired_effects(rows)
    paired_by_arm = {item["arm"]: item for item in paired_effects}
    payload = {
        "classification": "engineering_evidence_not_qualification",
        "comparison": "matched_baseline_sr_only_dr_only_srdr",
        "benefit_definition": {
            "benefit_signals": [
                "route selection changes",
                "success and collision-free behavior do not degrade",
                "dynamic-obstacle exposure decreases or clearance increases",
            ],
            "distance_role": "constraint only; shorter distance is not counted as benefit",
            "distance_acceptance_rule": "absolute deviation from matched Baseline <= 20 percent",
        },
        "rows": public_rows,
        "aggregate": aggregate,
        "paired_effects": paired_effects,
        "campaign_gate": {
            "all_four_arms_present_per_pair": all(
                {row["arm"] for row in rows if (row["query_id"], row["variant_id"]) == key}
                == set(ARMS)
                for key in baseline
            ),
            "all_completed": all(row["completed"] and not row["failed"] for row in rows),
            "all_collision_free": all(
                not row["physical_collision"]
                and row["sampled_static_footprint_collisions"] == 0
                for row in rows
            ),
            "all_factor_gates_pass": all(row["factor_gate_pass"] for row in rows),
            "all_distance_deviations_within_20pct": all(
                row["distance_within_20pct"] for row in rows
            ),
            "dr_only_and_srdr_route_change_observed": all(
                paired_by_arm.get(arm, {}).get("route_change_count", 0) > 0
                for arm in ("dr_only", "srdr")
            ),
            "dr_only_and_srdr_dynamic_risk_benefit_observed": all(
                paired_by_arm.get(arm, {}).get("dynamic_risk_benefit_count", 0) > 0
                for arm in ("dr_only", "srdr")
            ),
        },
    }
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    args.output_prefix.with_suffix(".json").write_text(
        json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    fieldnames = list(public_rows[0])
    with args.output_prefix.with_suffix(".csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(
            stream, fieldnames=fieldnames, lineterminator="\n"
        )
        writer.writeheader()
        for row in public_rows:
            writer.writerow({
                **row,
                "initial_route_edge_ids": " ".join(map(str, row["initial_route_edge_ids"])),
            })

    defaults = load_engineering_defaults(args.defaults)
    occupancy = load_occupancy_map(
        args.map_path,
        unknown_is_occupied=bool(defaults["graph"]["unknown_is_occupied"]),
    )
    _render(rows, occupancy, args.output_prefix.with_suffix(".png"))


if __name__ == "__main__":
    main()
