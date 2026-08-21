"""Frozen L0--L3 localization causal campaign planner.

This is deliberately a dispatcher contract, not a simulation framework.  It
expands the core 60 rows, describes the external ROS/Isaac actions, and refuses
``run`` until a real adapter is supplied.  Ground Truth is absent from every
dispatcher topic and is consumed only by the offline evaluator.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from .v6_localization_causal_evaluator import evaluate_campaign


SCHEMA_VERSION = "bio_nav_v6_localization_causal_manifest_v1"
QUALIFICATION = "ENGINEERING_CAUSAL_NOT_RUN"
GT_PREFIX = "/" + "ground_truth/"
ARMS = ("L0", "L1", "L2", "L3")
CORE_CASES = ("S0", "S3", "W0")
COUNTERBALANCED_ORDERS = (
    ("L0", "L1", "L2", "L3"),
    ("L3", "L2", "L1", "L0"),
    ("L1", "L3", "L0", "L2"),
    ("L2", "L0", "L3", "L1"),
    ("L0", "L2", "L3", "L1"),
)
DISPATCHER_TOPICS = (
    "/odom",
    "/amcl_pose",
    "/initialpose",
    "/cmd_vel",
    "/plan",
    "/bio_nav/localization/status",
    "/bio_nav/localization/request_manual_rescue",
    "/simulation/localization_seeded",
)
PASSIVE_EVALUATOR_TOPICS = (
    "/ground_truth/odom",
    "/simulation/collision",
    "/simulation/collision_diagnostics",
)

if any(topic.startswith(GT_PREFIX) for topic in DISPATCHER_TOPICS):
    raise RuntimeError("localization causal dispatcher Ground Truth firewall violated")


class CausalContractError(RuntimeError):
    """The frozen manifest is invalid."""


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise CausalContractError(f"{name} must be a mapping")
    return value


def _expected_counts(arm: str, case: str) -> dict[str, Any]:
    mode = {"L0": "off", "L1": "shadow", "L2": "startup", "L3": "active"}[arm]
    if case == "S0":
        return {
            "integration_mode": mode,
            "integration_initialpose_writes": 1 if arm in {"L2", "L3"} else 0,
            "total_initialpose_writes": 1,
            "manual_rescue_requests": 0,
        }
    if case == "S3" and arm == "L3":
        return {
            "integration_mode": mode,
            "integration_initialpose_writes": 1,
            "total_initialpose_writes": 1,
            "manual_rescue_requests": 1,
        }
    return {
        "integration_mode": mode,
        "integration_initialpose_writes": 0,
        "total_initialpose_writes": 0,
        "manual_rescue_requests": 0,
    }


def _expand_runs(raw: Mapping[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    cases = _mapping(raw.get("cases"), "cases")
    core: list[dict[str, Any]] = []
    for case_name in CORE_CASES:
        case = _mapping(cases.get(case_name), f"cases.{case_name}")
        seeds = case.get("seeds")
        if not isinstance(seeds, list) or len(seeds) != 5 or any(
            isinstance(seed, bool) or not isinstance(seed, int) for seed in seeds
        ):
            raise CausalContractError(f"cases.{case_name}.seeds must contain five integers")
        for repeat, (seed, order) in enumerate(zip(seeds, COUNTERBALANCED_ORDERS), start=1):
            for sequence, arm in enumerate(order, start=1):
                core.append(
                    {
                        "run_id": f"v6-loc-{case_name.lower()}-{seed}-{arm.lower()}",
                        "case": case_name,
                        "seed": seed,
                        "repeat": repeat,
                        "sequence_in_block": sequence,
                        "arm": arm,
                        "expected": _expected_counts(arm, case_name),
                    }
                )
    preflight_raw = raw.get("engineering_preflights")
    if not isinstance(preflight_raw, list) or len(preflight_raw) != 2:
        raise CausalContractError("engineering_preflights must define S1 and S2")
    preflight = []
    for row in preflight_raw:
        item = _mapping(row, "engineering_preflight")
        if item.get("case") not in {"S1", "S2"} or not isinstance(item.get("seed"), int):
            raise CausalContractError("engineering preflights require integer-seed S1/S2 rows")
        preflight.append(dict(item))
    if {row["case"] for row in preflight} != {"S1", "S2"}:
        raise CausalContractError("engineering_preflights must contain exactly S1 and S2")
    return core, preflight


def load_manifest(path: str | Path) -> dict[str, Any]:
    config_path = Path(path).expanduser().resolve()
    raw_value = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    raw = dict(_mapping(raw_value, "manifest"))
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise CausalContractError(f"schema_version must be {SCHEMA_VERSION}")
    common = _mapping(raw.get("common_runtime"), "common_runtime")
    fixed = {
        "odometry_mode": "estimated",
        "cognitive_profile": "M0",
        "module2_planning_influence": False,
        "cognitive_graph_mode": "gvg",
        "direct_rgbd_costmap_enabled": False,
        "use_rviz": False,
        "structure_tf_source": "isaac",
        "goal_id": "G2",
        "initial_physical_pose": "long_route_start_g2",
        "automatic_rescue_enabled": False,
    }
    for key, expected in fixed.items():
        if common.get(key) != expected:
            raise CausalContractError(f"common_runtime.{key} must be {expected!r}")
    arms = _mapping(raw.get("arms"), "arms")
    for name in ARMS:
        _mapping(arms.get(name), f"arms.{name}")
    arm_fixed = {
        "L0": (False, "conventional", "off", 0, False),
        "L1": (True, "estimated_shadow", "shadow", 0, False),
        "L2": (True, "estimated_shadow", "startup", 1, False),
        "L3": (True, "estimated_autonomy", "active", 1, True),
    }
    for name, expected in arm_fixed.items():
        arm = arms[name]
        actual = (
            arm.get("integration_enabled"),
            arm.get("localization_profile"),
            arm.get("integration_mode"),
            arm.get("startup_initialpose_writes"),
            arm.get("manual_rescue_allowed_after_lost"),
        )
        if actual != expected:
            raise CausalContractError(f"arms.{name} violates the frozen localization contract")
    capture = _mapping(raw.get("capture"), "capture")
    if tuple(capture.get("dispatcher_topics", ())) != DISPATCHER_TOPICS:
        raise CausalContractError("capture.dispatcher_topics changed")
    if tuple(capture.get("passive_evaluator_topics", ())) != PASSIVE_EVALUATOR_TOPICS:
        raise CausalContractError("capture.passive_evaluator_topics changed")
    if any(str(topic).startswith(GT_PREFIX) for topic in capture["dispatcher_topics"]):
        raise CausalContractError("dispatcher must not subscribe to Ground Truth")
    core, preflight = _expand_runs(raw)
    if len(core) != 60 or len({row["run_id"] for row in core}) != 60:
        raise CausalContractError("core campaign must expand to 60 unique rows")
    for case_name, expected in {
        "S0": [8701, 8702, 8703, 8704, 8705],
        "S3": [8731, 8732, 8733, 8734, 8735],
        "W0": [8741, 8742, 8743, 8744, 8745],
    }.items():
        if list(_mapping(raw["cases"][case_name], case_name)["seeds"]) != expected:
            raise CausalContractError(f"{case_name} seeds changed")
    raw["config_path"] = str(config_path)
    raw["qualification"] = QUALIFICATION
    raw["core_runs"] = core
    raw["engineering_preflight_runs"] = preflight
    return raw


def _case_steps(run: Mapping[str, Any]) -> list[dict[str, Any]]:
    arm = run["arm"]
    case = run["case"]
    if case == "S0":
        seed_source = "integration_cognitive" if arm in {"L2", "L3"} else "runner_conventional_coarse"
        return [
            {
                "action": "startup_seed_once",
                "source": seed_source,
                "coarse_offset": (
                    None if arm in {"L2", "L3"}
                    else {"x_m": 1.0, "y_m": -0.5, "yaw_deg": 20.0}
                ),
            },
            {"action": "observe_absolute_convergence", "hold_s": 2.0},
        ]
    if case == "S3":
        steps = [
            {"action": "cancel_goal_and_assert_no_active_goal"},
            {"action": "publish_zero_and_hold", "hold_s": 1.0},
            {"action": "pause_simulation"},
            {"action": "set_kidnap_parameters", "pose": "long_route_start_g5", "armed": True},
            {"action": "call_trigger_once", "service": "/simulation/kidnap", "retry": False},
            {"action": "assert_no_odom_ekf_amcl_reset_or_initialpose_side_effect"},
            {"action": "set_kidnap_parameters", "pose": "long_route_start_g5", "armed": False},
            {"action": "resume_simulation"},
            {"action": "observe_lost", "position_m": 0.75, "yaw_deg": 20.0, "hold_s": 1.0},
        ]
        if arm == "L3":
            steps.append(
                {
                    "action": "request_manual_rescue_once_after_lost",
                    "service": "/bio_nav/localization/request_manual_rescue",
                }
            )
        else:
            steps.append({"action": "assert_no_manual_rescue"})
        steps.append({"action": "observe_absolute_recovery", "hold_s": 2.0})
        return steps
    return [
        {"action": "begin_with_healthy_amcl"},
        {"action": "do_not_request_manual_rescue"},
        {"action": "assert_no_initialpose_during_observation"},
    ]


def build_plan(manifest: Mapping[str, Any]) -> dict[str, Any]:
    common = dict(manifest["common_runtime"])
    arms = manifest["arms"]
    rows = []
    for run in manifest["core_runs"]:
        rows.append(
            {
                **run,
                "runtime": {**common, **dict(arms[run["arm"]])},
                "dispatcher_topics": list(DISPATCHER_TOPICS),
                "passive_evaluator_topics": list(PASSIVE_EVALUATOR_TOPICS),
                "steps": _case_steps(run),
            }
        )
    return {
        "schema_version": SCHEMA_VERSION,
        "qualification": QUALIFICATION,
        "state": "PLANNED_NOT_RUN",
        "core_run_count": len(rows),
        "engineering_preflight_count": len(manifest["engineering_preflight_runs"]),
        "ground_truth_policy": "passive_evaluator_only",
        "runs": rows,
        "engineering_preflights": manifest["engineering_preflight_runs"],
    }


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def cli(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("manifest", "plan", "evaluate", "run"))
    parser.add_argument("--config", required=True)
    parser.add_argument("--evidence-dir")
    args = parser.parse_args(argv)
    manifest = load_manifest(args.config)
    if args.command == "manifest":
        output = manifest
        code = 0
    elif args.command == "plan":
        output = build_plan(manifest)
        code = 0
    elif args.command == "evaluate":
        if not args.evidence_dir:
            parser.error("evaluate requires --evidence-dir")
        output = evaluate_campaign(manifest, args.evidence_dir)
        code = 0 if output["verdict"] == "PASS_CRITERIA" else 2
    else:
        output = {
            "qualification": QUALIFICATION,
            "state": "NOT_RUN",
            "reason": "no live ROS/Isaac adapter is installed; use plan for the bounded external sequence",
            "core_run_count": 60,
        }
        code = 2
    print(json.dumps(output, indent=2, sort_keys=True, default=_json_default))
    return code


def main(argv: Sequence[str] | None = None) -> int:
    return cli(argv)


if __name__ == "__main__":
    raise SystemExit(main())
