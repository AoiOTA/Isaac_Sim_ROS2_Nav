#!/usr/bin/env python3
"""Generate an identity-bound Attempt-21 LocalRiskGrid Nav2 overlay.

The checked-in RGB-D profile stays Shadow-only.  A non-Shadow profile is
created only for a controlled static A/B campaign and is bound to an explicit
authorization receipt SHA.  It never changes the stable or dynamic defaults.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import yaml


WAREHOUSE_NEW_OCCUPANCY_VERSION = (
    "566b92f58042919f9be0612281f1936910465784a32b788c60263e6f1825b2ce"
)


def effective_global_plugins(
    overlay_profile: Path, overlay_parameters: dict
) -> list[str]:
    """Resolve the complete plugin list inherited by a ROS parameter overlay."""

    explicit = overlay_parameters.get("plugins")
    if explicit is not None:
        return list(explicit)
    defaults_path = overlay_profile.parent / "nav2_params.yaml"
    if not defaults_path.is_file():
        raise ValueError(
            "overlay has no global plugins and adjacent nav2_params.yaml is absent"
        )
    defaults = yaml.safe_load(defaults_path.read_text(encoding="utf-8"))
    plugins = (
        defaults.get("global_costmap", {})
        .get("global_costmap", {})
        .get("ros__parameters", {})
        .get("plugins")
    )
    if not plugins or "depth_voxel_layer" not in plugins:
        raise ValueError(
            "effective static Global Costmap must retain depth_voxel_layer"
        )
    return list(plugins)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    digest.update(path.read_bytes())
    return digest.hexdigest()


def require_sha256(value: str, field: str) -> str:
    normalized = str(value).strip().lower()
    if len(normalized) != 64 or any(
        character not in "0123456789abcdef" for character in normalized
    ):
        raise ValueError(f"{field} must be a lowercase SHA-256")
    return normalized


def build_profile(
    stable_profile: Path,
    *,
    risk_model_sha256: str,
    risk_qualification_sha256: str,
    shadow_only: bool,
    minimum_projection_range_m: float = 0.0,
    maximum_cost: int = 80,
) -> dict:
    model_sha = require_sha256(risk_model_sha256, "risk_model_sha256")
    qualification_sha = require_sha256(
        risk_qualification_sha256, "risk_qualification_sha256"
    )
    minimum_range = float(minimum_projection_range_m)
    if not 0.0 <= minimum_range <= 8.0:
        raise ValueError("minimum_projection_range_m must be in [0, 8]")
    bounded_cost = int(maximum_cost)
    if not 1 <= bounded_cost <= 80:
        raise ValueError("maximum_cost must be in [1, 80]")
    source = yaml.safe_load(stable_profile.read_text(encoding="utf-8"))
    result = deepcopy(source)
    parameters = (
        result.setdefault("global_costmap", {})
        .setdefault("global_costmap", {})
        .setdefault("ros__parameters", {})
    )
    plugins = effective_global_plugins(stable_profile, parameters)
    if "local_rgbd_risk_layer" not in plugins:
        plugins.insert(plugins.index("inflation_layer"), "local_rgbd_risk_layer")
    parameters["plugins"] = plugins
    parameters["local_rgbd_risk_layer"] = {
        "plugin": "bio_nav_fusion::LocalRiskGridLayer",
        "enabled": True,
        "shadow_only": bool(shadow_only),
        "risk_topic": "/bio_nav/module2/local_risk_grid",
        "reset_topic": "/simulation/reset_event",
        "max_message_age_s": 0.5,
        "transform_tolerance_s": 0.05,
        "minimum_reliability": 0.6,
        "maximum_ood_probability": 0.4,
        "activation_threshold": 0.5,
        "clear_threshold": 0.4,
        "minimum_projection_range_m": minimum_range,
        "maximum_cost": bounded_cost,
        "expected_map_version": WAREHOUSE_NEW_OCCUPANCY_VERSION,
        "expected_model_sha256": model_sha,
        "expected_qualification_sha256": qualification_sha,
        "initial_reset_epoch": 0,
    }
    return result


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--stable-profile",
        type=Path,
        default=None,
        help=(
            "base profile to preserve byte-for-semantics; controlled static "
            "modes default to nav2_attempt21_static_collection.yaml"
        ),
    )
    parser.add_argument("--risk-model-sha256", required=True)
    parser.add_argument("--risk-qualification-sha256", required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument(
        "--controlled-static-ab",
        action="store_true",
        help="enable bounded Global Costmap writes for the static A/B only",
    )
    modes.add_argument(
        "--static-opt-in",
        action="store_true",
        help="generate the explicit static-only opt-in profile after full static PASS",
    )
    parser.add_argument(
        "--authorization-sha256",
        default="",
        help="required SHA-256 of the A/B authorization or final static delivery",
    )
    parser.add_argument(
        "--minimum-projection-range-m",
        type=float,
        default=0.0,
        help=(
            "exclude closer LocalRiskGrid cells from Global Costmap writes; "
            "the existing Local Costmap and Collision Monitor retain near-field ownership"
        ),
    )
    parser.add_argument(
        "--maximum-cost",
        type=int,
        default=80,
        help="nonlethal Global Costmap ceiling (1..80)",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    authorization_sha = ""
    if args.controlled_static_ab or args.static_opt_in:
        authorization_sha = require_sha256(
            args.authorization_sha256, "authorization_sha256"
        )
    elif args.authorization_sha256:
        parser.error(
            "--authorization-sha256 requires --controlled-static-ab or --static-opt-in"
        )

    base_profile = args.stable_profile
    if base_profile is None:
        base_profile = root / "ros2_ws/src/robot_navigation/config" / (
            "nav2_attempt21_static_collection.yaml"
            if args.controlled_static_ab or args.static_opt_in
            else "nav2_stable.yaml"
        )
    base_profile = base_profile.resolve()
    output = args.output.resolve()
    receipt_path = output.with_suffix(output.suffix + ".receipt.json")
    if output.exists() or receipt_path.exists():
        raise FileExistsError(f"refusing to overwrite frozen profile {output}")
    payload = build_profile(
        base_profile,
        risk_model_sha256=args.risk_model_sha256,
        risk_qualification_sha256=args.risk_qualification_sha256,
        shadow_only=not (args.controlled_static_ab or args.static_opt_in),
        minimum_projection_range_m=args.minimum_projection_range_m,
        maximum_cost=args.maximum_cost,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        "# Generated by generate_attempt21_rgbd_risk_profile.py.\n"
        "# stable and dynamic_avoidance remain unchanged.\n"
        + yaml.safe_dump(payload, sort_keys=False),
        encoding="utf-8",
    )
    receipt = {
        "schema": "bio_nav_attempt21_rgbd_risk_profile_receipt_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "profile": str(output),
        "profile_sha256": sha256_file(output),
        "base_profile": str(base_profile),
        "base_profile_sha256": sha256_file(base_profile),
        "base_profile_semantics_preserved": True,
        "effective_global_plugins": payload["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]["plugins"],
        "global_depth_voxel_layer_preserved": (
            "depth_voxel_layer"
            in payload["global_costmap"]["global_costmap"]["ros__parameters"][
                "plugins"
            ]
        ),
        "shadow_only": not (args.controlled_static_ab or args.static_opt_in),
        "qualification_scope": (
            "static_hazard_opt_in_active"
            if args.static_opt_in
            else "controlled_static_ab_only"
            if args.controlled_static_ab
            else "shadow_only"
        ),
        "controlled_ab_costmap_write_enabled": bool(args.controlled_static_ab),
        "static_opt_in_costmap_write_enabled": bool(args.static_opt_in),
        "authorization_sha256": authorization_sha,
        "ab_authorization_sha256": (
            authorization_sha if args.controlled_static_ab else ""
        ),
        "static_delivery_sha256": (
            authorization_sha if args.static_opt_in else ""
        ),
        "risk_model_sha256": require_sha256(
            args.risk_model_sha256, "risk_model_sha256"
        ),
        "risk_qualification_sha256": require_sha256(
            args.risk_qualification_sha256, "risk_qualification_sha256"
        ),
        "maximum_cost": args.maximum_cost,
        "minimum_projection_range_m": args.minimum_projection_range_m,
        "local_costmap_modified": False,
        "stable_default_modified": False,
        "dynamic_default_modified": False,
        "general_active_fusion_authorized": False,
    }
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(receipt, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
