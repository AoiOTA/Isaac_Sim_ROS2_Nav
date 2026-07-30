#!/usr/bin/env python3
"""Generate identity-bound optional BioNav fusion profiles.

Checked-in profiles are fail-closed templates.  A runnable planning, risk, or
combined overlay must be generated with the exact snapshot/map and/or
risk-adapter identities selected by the Integration repository.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path

import yaml

WAREHOUSE_NEW_OCCUPANCY_VERSION = (
    "566b92f58042919f9be0612281f1936910465784a32b788c60263e6f1825b2ce"
)
ZERO_SHA256 = "0" * 64
VARIANTS = ("planning_only", "risk_only", "combined")
PROFILE_NAMES = {
    "planning_only": "bio_nav_planning_only",
    "risk_only": "bio_nav_risk_only",
    "combined": "bio_nav_tiebreak_risk",
}


def _sha256(value: str, field: str) -> str:
    normalized = str(value).strip().lower()
    if (
        len(normalized) != 64
        or any(character not in "0123456789abcdef" for character in normalized)
    ):
        raise ValueError(f"{field} must be a lowercase SHA-256")
    return normalized


def build_profile(
    dynamic_profile: Path,
    *,
    variant: str,
    module3_map_sha256: str = "",
    planning_qualification_sha256: str = "",
    risk_model_sha256: str = "",
    risk_qualification_sha256: str = "",
) -> dict:
    if variant not in VARIANTS:
        raise ValueError(f"unsupported fusion profile variant: {variant}")
    planning_enabled = variant in {"planning_only", "combined"}
    risk_enabled = variant in {"risk_only", "combined"}
    if planning_enabled:
        module3_map_sha256 = _sha256(
            module3_map_sha256, "module3_map_sha256"
        )
        planning_qualification_sha256 = _sha256(
            planning_qualification_sha256,
            "planning_qualification_sha256",
        )
    if risk_enabled:
        risk_model_sha256 = _sha256(
            risk_model_sha256, "risk_model_sha256"
        )
        risk_qualification_sha256 = _sha256(
            risk_qualification_sha256, "risk_qualification_sha256"
        )

    payload = yaml.safe_load(dynamic_profile.read_text(encoding="utf-8"))
    result = deepcopy(payload)
    global_parameters = result["global_costmap"]["global_costmap"]["ros__parameters"]
    if risk_enabled:
        plugins = list(global_parameters["plugins"])
        if "cognitive_risk_layer" not in plugins:
            plugins.insert(
                plugins.index("inflation_layer"), "cognitive_risk_layer"
            )
        global_parameters["plugins"] = plugins
        global_parameters["cognitive_risk_layer"] = {
            "plugin": "bio_nav_fusion::CognitiveRiskLayer",
            "enabled": True,
            "prior_topic": "/bio_nav/module2/planning_prior",
            "reset_topic": "/simulation/reset_event",
            "max_message_age_s": 0.75,
            "minimum_reliability": 0.2,
            "maximum_cost": 80,
            # This is the live OccupancyGrid hash published by the Integration
            # bridge for warehouse_new, not the human-facing map alias or the
            # offline PGM-loader digest.
            "expected_map_version": WAREHOUSE_NEW_OCCUPANCY_VERSION,
            "expected_risk_model_sha256": risk_model_sha256,
            "expected_qualification_sha256": risk_qualification_sha256,
            "initial_reset_epoch": 0,
        }
    if planning_enabled:
        result["planner_server"] = {
            "ros__parameters": {
                "planner_plugins": ["GridBased"],
                "GridBased": {
                    "plugin": "bio_nav_fusion::BioNavGridBased",
                    "planner_profile": PROFILE_NAMES[variant],
                    "prior_service": "/bio_nav/get_goal_planning_prior",
                    "planning_prior_topic": "/bio_nav/module2/planning_prior",
                    "service_timeout_ms": 100,
                    "allow_unknown": True,
                    "expected_module3_map_sha256": module3_map_sha256,
                    "expected_qualification_sha256":
                        planning_qualification_sha256,
                },
                # The wrapper calls this stock plugin directly whenever
                # cognition is absent, stale, invalid, or cannot find a path.
                "GridBased_fallback": {
                    "tolerance": 0.10,
                    "downsample_costmap": False,
                    "allow_unknown": True,
                    "max_iterations": 1000000,
                    "max_on_approach_iterations": 1000,
                    "max_planning_time": 2.0,
                    "cost_travel_multiplier": 1.2,
                    "use_final_approach_orientation": False,
                    "smoother": {
                        "max_iterations": 1000,
                        "w_smooth": 0.30,
                        "w_data": 0.20,
                        "tolerance": 1.0e-10,
                        "do_refinement": True,
                    },
                },
            }
        }
    return result


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dynamic-profile",
        type=Path,
        default=root
        / "ros2_ws/src/robot_navigation/config/nav2_dynamic_avoidance.yaml",
    )
    parser.add_argument(
        "--variant",
        choices=VARIANTS,
        default="combined",
    )
    parser.add_argument("--module3-map-sha256", default="")
    parser.add_argument("--planning-qualification-sha256", default="")
    parser.add_argument("--risk-model-sha256", default="")
    parser.add_argument("--risk-qualification-sha256", default="")
    parser.add_argument(
        "--safe-template",
        action="store_true",
        help="write a fail-closed all-zero identity template",
    )
    parser.add_argument(
        "--output",
        type=Path,
    )
    args = parser.parse_args()
    if args.safe_template:
        args.module3_map_sha256 = ZERO_SHA256
        args.planning_qualification_sha256 = ZERO_SHA256
        args.risk_model_sha256 = ZERO_SHA256
        args.risk_qualification_sha256 = ZERO_SHA256
    payload = build_profile(
        args.dynamic_profile.resolve(),
        variant=args.variant,
        module3_map_sha256=args.module3_map_sha256,
        planning_qualification_sha256=args.planning_qualification_sha256,
        risk_model_sha256=args.risk_model_sha256,
        risk_qualification_sha256=args.risk_qualification_sha256,
    )
    target = (
        args.output.resolve()
        if args.output
        else (
            root
            / "ros2_ws/src/robot_navigation/config"
            / f"nav2_{PROFILE_NAMES[args.variant]}.yaml"
        )
    )
    target.write_text(
        "# Generated by scripts/generate_bionav_fusion_profile.py.\n"
        "# Optional identity-bound profile; stable/dynamic_avoidance remain defaults.\n"
        + (
            "# FAIL-CLOSED TEMPLATE: regenerate with exact runtime identities.\n"
            if args.safe_template
            else ""
        )
        + yaml.safe_dump(payload, sort_keys=False),
        encoding="utf-8",
    )
    print(target)


if __name__ == "__main__":
    main()
