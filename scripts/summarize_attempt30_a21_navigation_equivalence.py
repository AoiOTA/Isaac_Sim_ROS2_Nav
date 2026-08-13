#!/usr/bin/env python3
"""Summarize fresh Classic-versus-Cognitive real navigation evidence."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import re

import matplotlib.pyplot as plt
import numpy as np

from robot_route_planner.defaults import load_engineering_defaults
from robot_route_planner.map_io import load_occupancy_map
from robot_route_planner.visualize import _map_background


METHODS = ("cognitive", "classic_iterative")
CLASSIC_ERROR = re.compile(r"(?:^|:)error=([0-9.eE+-]+)$")


def _load(path: Path, method: str) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    healthy = [item for item in value.get("edge_prior_history", []) if item.get("healthy")]
    if not healthy:
        raise RuntimeError(f"{path}: no healthy edge prior")
    first = healthy[0]
    classic_errors = []
    for item in healthy:
        match = CLASSIC_ERROR.search(str(item.get("model_id", "")))
        if match:
            classic_errors.append(float(match.group(1)))
    costs = {
        int(item["edge_id"]): float(item["cost_delta_m"])
        for item in first.get("priors", [])
    }
    diagnostics = value.get("srdr_edge_diagnostic_history", [])
    return {
        "path": str(path),
        "method": method,
        "variant": str(value.get("dynamic_variant_id", "")),
        "completed": bool(value.get("completed")),
        "failed": bool(value.get("failed")),
        "physical_collision": bool(value.get("physical_collision")),
        "footprint_collisions": int(value.get("sampled_static_footprint_collisions", 0)),
        "elapsed_s": float(value.get("elapsed_s", 0.0)),
        "distance_m": float(value.get("travelled_distance_m", 0.0)),
        "final_goal_error_m": float(value.get("final_goal_error_m", 0.0)),
        "minimum_actor_distance_m": float(value.get("obstacle_center_min_distance_m", 0.0)),
        "exposure_s_under_1_5m": float(value.get("obstacle_center_exposure_s_under_1_5m", 0.0)),
        "route": [int(item) for item in value.get("route_history", [{}])[0].get("edge_ids", [])],
        "model_id": str(first.get("model_id", "")),
        "classic_internal_field_error_max": (
            max(classic_errors) if classic_errors else None
        ),
        "costs": costs,
        "positive_sr": max((int(item.get("positive_sr_count", 0)) for item in diagnostics), default=0),
        "positive_dr": max((int(item.get("positive_dr_count", 0)) for item in diagnostics), default=0),
        "trajectory": np.asarray(
            [item[:2] for item in value.get("trajectory_xy_zw", [])], dtype=float
        ),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign-root", type=Path, required=True)
    parser.add_argument("--map", dest="map_path", type=Path, required=True)
    parser.add_argument("--defaults", type=Path, required=True)
    parser.add_argument("--output-prefix", type=Path, required=True)
    arguments = parser.parse_args()

    rows = []
    for method in METHODS:
        paths = sorted((arguments.campaign_root / method).rglob("closed_loop.json"))
        rows.extend(_load(path, method) for path in paths)
    variants = sorted({row["variant"] for row in rows})
    by_key = {(row["method"], row["variant"]): row for row in rows}
    if len(rows) != 2 * len(variants) or not variants:
        raise RuntimeError("every variant must contain exactly two methods")

    pairs = []
    for variant in variants:
        cognitive = by_key[("cognitive", variant)]
        classic = by_key[("classic_iterative", variant)]
        common = sorted(set(cognitive["costs"]) & set(classic["costs"]))
        cost_error = max(
            (abs(cognitive["costs"][edge] - classic["costs"][edge]) for edge in common),
            default=float("inf"),
        )
        denominator = max(cognitive["distance_m"], 1.0e-9)
        distance_deviation = 100.0 * (classic["distance_m"] - denominator) / denominator
        pair = {
            "variant": variant,
            "cognitive_model_id": cognitive["model_id"],
            "classic_model_id": classic["model_id"],
            "edge_cost_max_abs_error_m": cost_error,
            "classic_internal_field_error_max": classic[
                "classic_internal_field_error_max"
            ],
            "route_equal": cognitive["route"] == classic["route"],
            "route_edge_ids": cognitive["route"],
            "cognitive_distance_m": cognitive["distance_m"],
            "classic_distance_m": classic["distance_m"],
            "classic_distance_deviation_percent": distance_deviation,
            "both_completed": all(item["completed"] and not item["failed"] for item in (cognitive, classic)),
            "both_collision_free": all(
                not item["physical_collision"] and item["footprint_collisions"] == 0
                for item in (cognitive, classic)
            ),
            "both_factor_isolated": all(
                item["positive_sr"] > 0 and item["positive_dr"] > 0
                for item in (cognitive, classic)
            ),
            "minimum_actor_distance_delta_m": classic["minimum_actor_distance_m"] - cognitive["minimum_actor_distance_m"],
            "exposure_delta_s_under_1_5m": classic["exposure_s_under_1_5m"] - cognitive["exposure_s_under_1_5m"],
        }
        pair["pass"] = bool(
            pair["both_completed"]
            and pair["both_collision_free"]
            and pair["both_factor_isolated"]
            and pair["route_equal"]
            and pair["classic_internal_field_error_max"] is not None
            and pair["classic_internal_field_error_max"] <= 1.0e-7
            and abs(pair["classic_distance_deviation_percent"]) <= 20.0
            and classic["model_id"].startswith("classic_iterative_equivalent:")
            and not cognitive["model_id"].startswith("classic_iterative_equivalent:")
        )
        pairs.append(pair)

    payload = {
        "schema": "attempt30_a21_contract_5_3_13_navigation_equivalence_v1",
        "classification": "engineering_evidence_not_qualification",
        "methods": {
            "classic": "iterative regularized value adaptation",
            "cognitive": "cached V3.10 M_SR/M_DR readout",
        },
        "same_effect_criteria": {
            "classic_same_ingress_internal_field_error_max": 1.0e-7,
            "cross_run_edge_cost_max_abs_error_m": "reported_not_gated_distinct_live_ingress",
            "selected_route_equal": True,
            "both_goal_checker_complete": True,
            "both_collision_free": True,
            "actual_distance_deviation_limit_percent": 20.0,
        },
        "pairs": pairs,
        "pass": bool(pairs and all(item["pass"] for item in pairs)),
    }
    arguments.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_prefix.with_suffix(".json").write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    fields = [key for key in pairs[0] if key != "route_edge_ids"]
    with arguments.output_prefix.with_suffix(".csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows({key: row[key] for key in fields} for row in pairs)

    defaults = load_engineering_defaults(arguments.defaults)
    occupancy = load_occupancy_map(
        arguments.map_path,
        unknown_is_occupied=bool(defaults["graph"]["unknown_is_occupied"]),
    )
    figure, axes = plt.subplots(1, len(variants), figsize=(6 * len(variants), 6), squeeze=False)
    for axis, variant in zip(axes[0], variants):
        _map_background(axis, occupancy)
        for method, color in (("cognitive", "#8e24aa"), ("classic_iterative", "#00897b")):
            row = by_key[(method, variant)]
            if row["trajectory"].size:
                axis.plot(
                    row["trajectory"][:, 0], row["trajectory"][:, 1],
                    color=color, linewidth=1.8, label=method,
                )
        axis.set_title(f"{variant}: same Route and actual execution")
        axis.legend()
    figure.suptitle("5.3.13 Part B: Classic/Cognitive real navigation equivalence")
    figure.tight_layout()
    figure.savefig(arguments.output_prefix.with_suffix(".png"), dpi=180)
    plt.close(figure)
    if not payload["pass"]:
        raise SystemExit("navigation equivalence gate failed")
    print("NAVIGATION_EQUIVALENCE_PASS", len(pairs))


if __name__ == "__main__":
    main()
