"""V6 estimated-autonomy episode dispatcher and engineering pilot adapter.

This module deliberately does not share the legacy experiment runner.  The
dispatcher owns reset and RouteCoordinator goal sequencing only.  Ground Truth
is reserved for the independent ``estimated_state_evaluator``/recorder.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import dataclass, field, replace
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import shlex
import statistics
import subprocess
import sys
import tempfile
import time
from typing import Any, Mapping

import yaml

from robot_experiments.reset_receipt import (
    ResetReceiptError,
    parse_reset_receipt,
)
from robot_experiments.configuration import ConfigurationError
from robot_experiments.report import (
    REPRODUCIBILITY_FIELDS,
    ReportValidationError,
    _csv_value,
    validate_manifest,
)
from robot_experiments.scenario import load_scenario


SCHEMA_VERSION = "bio_nav_v6_r3_phase2_pilot_manifest_v1"
FORMAL_CAMPAIGN_SCHEMA_VERSION = "bio_nav_v6_formal_campaign_v1"
SUFFICIENT_PILOT_MANIFEST_SCHEMA = "bio_nav_v6_sufficient_pilot_manifest_v1"
SUFFICIENT_PILOT_AGGREGATE_SCHEMA = "bio_nav_v6_sufficient_pilot_aggregate_v1"
INDOOR_PILOT_MANIFEST_SCHEMA = "bio_nav_v6_indoor_pilot_manifest_v2"
INDOOR_PILOT_AGGREGATE_SCHEMA = "bio_nav_v6_indoor_pilot_aggregate_v2"
INDOOR_CAMPAIGN_SCHEMA_VERSION = "bio_nav_v6_indoor_campaign_v2"
INDOOR_CONTINUATION_SCHEMA_VERSION = "bio_nav.v6_indoor_continuation.v1"
OUTDOOR_PILOT_MANIFEST_SCHEMA = "bio_nav_v6_outdoor_pilot_manifest_v1"
OUTDOOR_PILOT_AGGREGATE_SCHEMA = "bio_nav_v6_outdoor_pilot_aggregate_v1"
OUTDOOR_CAMPAIGN_SCHEMA_VERSION = "bio_nav_v6_outdoor_campaign_v1"
COMBINED_HALVES_SCHEMA_VERSION = "bio_nav_v6_combined_qualified_halves_v1"
# The externally reviewed migration has been consumed.  False is steady state:
# v6_formal may not change in later evaluator-only promotions.
EVALUATOR_PROMOTION_MIGRATION = False
FORMAL_NAS_ROOT = Path("/mnt/nas_home")
PILOT_SCENARIO_FILENAMES = {
    "indoor_static": "v6_final_kujiale_static.yaml",
    "outdoor_static": "final_rivermark_static.yaml",
    "outdoor_dynamic": "final_rivermark_dynamic.yaml",
    "outdoor_appearance": "final_rivermark_appearance.yaml",
    "indoor_dynamic": "v6_final_kujiale_dynamic.yaml",
    "indoor_appearance": "v6_final_kujiale_appearance.yaml",
}
FORMAL_CONDITION_IDS = (
    "indoor_static",
    "indoor_dynamic",
    "indoor_appearance",
    "outdoor_static",
    "outdoor_dynamic",
    "outdoor_appearance",
)
INDOOR_CONDITION_IDS = (
    "indoor_static",
    "indoor_dynamic",
    "indoor_appearance",
)
OUTDOOR_CONDITION_IDS = (
    "outdoor_static",
    "outdoor_dynamic",
    "outdoor_appearance",
)
FORMAL_RUNS_PER_CONDITION = 20
INDOOR_SUCCESS_THRESHOLDS = {
    "indoor_static": 19,
    "indoor_dynamic": 18,
    "indoor_appearance": 18,
}
OUTDOOR_SUCCESS_THRESHOLDS = {
    "outdoor_static": 19,
    "outdoor_dynamic": 18,
    "outdoor_appearance": 18,
}
FORMAL_SUCCESS_THRESHOLDS = {
    condition_id: (19 if condition_id.endswith("_static") else 18)
    for condition_id in FORMAL_CONDITION_IDS
}
INDOOR_FAILURE_BUDGETS = {
    condition_id: FORMAL_RUNS_PER_CONDITION - threshold
    for condition_id, threshold in INDOOR_SUCCESS_THRESHOLDS.items()
}
INDOOR_EXPECTED_OBSTACLES = {
    "indoor_static": (["v6_low_box_solo"], []),
    "indoor_dynamic": ([], ["v6_dynamic_g2_crossing_box"]),
    "indoor_appearance": (["v6_low_box_solo"], []),
}
FORMAL_EXECUTION_AUTHORIZED = "AUTHORIZED"
FORMAL_EXECUTION_NOT_AUTHORIZED = "NOT_AUTHORIZED"
FORMAL_FROZEN_ASSET_KEYS = frozenset({
    "module1_checkpoint",
    "module2_srdr_checkpoint",
    "module2_visual_heads_shadow_checkpoint",
    "selected_run4_visual_heads_checkpoint",
    "dino_checkpoint",
    "indoor_route_prior_manifest",
    "indoor_route_prior_m_sr",
    "indoor_route_prior_m_dr",
    "indoor_route_prior_transition",
    "indoor_route_prior_valid_state_mask",
    "rivermark_usd",
    "rivermark_catalog",
    "rivermark_catalog_constraints_tree",
    "indoor_map_yaml",
    "indoor_map_pgm",
    "outdoor_map_yaml",
    "outdoor_map_pgm",
})
INDOOR_FROZEN_ASSET_KEYS = frozenset({
    "module1_checkpoint",
    "module2_srdr_checkpoint",
    "module2_visual_heads_shadow_checkpoint",
    "selected_run4_visual_heads_checkpoint",
    "dino_checkpoint",
    "indoor_route_prior_manifest",
    "indoor_route_prior_m_sr",
    "indoor_route_prior_m_dr",
    "indoor_route_prior_transition",
    "indoor_route_prior_valid_state_mask",
    "indoor_map_yaml",
    "indoor_map_pgm",
})
OUTDOOR_FROZEN_ASSET_KEYS = frozenset({
    "module1_checkpoint",
    "module2_srdr_checkpoint",
    "module2_visual_heads_shadow_checkpoint",
    "selected_run4_visual_heads_checkpoint",
    "dino_checkpoint",
    "rivermark_usd",
    "rivermark_catalog",
    "rivermark_catalog_constraints_tree",
    "rivermark_viewport_startup_winner_manifest",
    "outdoor_map_yaml",
    "outdoor_map_pgm",
})
COMMON_HALF_ASSET_KEYS = frozenset({
    "module1_checkpoint",
    "module2_srdr_checkpoint",
    "module2_visual_heads_shadow_checkpoint",
    "selected_run4_visual_heads_checkpoint",
    "dino_checkpoint",
})
NOT_QUALIFIED = "NOT_QUALIFIED"
ENGINEERING_PILOT = "ENGINEERING_PILOT"
GT_PREFIX = "/" + "ground_truth/"
PRE_RESET_NEGATIVE_WINDOW_S = 1.0
# Cold episode boundary: reset is only armed after the stack is provably
# idle and still.  The same 1.0 s quiet window doubles as the stillness
# observation window.
PRE_RESET_STILL_SPAN_M = 0.10
COMMAND_ZERO_TOLERANCE = 1.0e-3
# Post-reset odometry must land at the re-zeroed odom origin and stay
# bounded until the first goal (no stale drive replay, no estimator jump).
POST_RESET_ODOM_LANDING_M = 0.10
POST_RESET_ODOM_SPAN_M = 0.10
# Per-attempt budget of the post-reset Nav2/TF readiness poll; the overall
# fail-closed budget stays the reset timeout.
NAV2_PROBE_ATTEMPT_TIMEOUT_SEC = 5.0
SOLE_PUBLISHER_TOPICS = ("/odom",)
R3_PHASE2_RUNTIME = {
    "canonical_odom": {
        "topic": "/odom",
        "owner": "isaac_compute_odometry",
        "tf": "odom->base_link",
    },
    "global_localization": {
        "pose_topic": "/amcl_pose",
        "owner": "amcl",
        "tf": "map->odom",
    },
    "module1_odom": {
        "topic": "/bio_nav/module1/odom",
        "owner": "wheel_imu_ekf",
        "publish_tf": False,
    },
    "recovery_enabled": False,
    "module2_navigation_write_enabled": False,
    "cognitive_place_graph_enabled": False,
    "route_backend": "gvg",
    "low_obstacles_enabled": False,
    "dynamic_actors_enabled": False,
    "goal_checker": "position_xy",
}
R5_PHASE_B_SCENE_ID = "kujiale_0026_A_to_B_door_open"
R5_PHASE_B_RUNTIME = {
    **R3_PHASE2_RUNTIME,
    "cognitive_profile": "M0",
    "module1_mode": "shadow",
}
R3_PHASE2_ASSET_KEYS = {
    "scene_asset",
    "occupancy_map",
    "spawn_manifest",
    "route_graph",
    "navigation_config",
}
R5_PHASE_B_ASSET_KEYS = R3_PHASE2_ASSET_KEYS | {
    "module1_shadow_config",
    "module1_shadow_checkpoint",
}

# Runtime subscriptions are a reviewable firewall.  Keep Ground Truth in the
# passive evaluator, never in this dispatcher.
DISPATCH_SUBSCRIPTION_TOPICS = (
    "/clock",
    "/scan",
    "/odom",
    "/amcl_pose",
    "/initialpose",
    "/tf",
    "/tf_static",
    "/map",
    "/simulation/reset_event",
    "/simulation/reset_stop_gate/status",
    "/bio_nav/navigation_graph",
    "/bio_nav/canonical_route",
    "/bio_nav/route_progress",
    "/bio_nav/route_goal_complete",
    "/bio_nav/route_goal_result",
    "/cmd_vel",
    "/cmd_vel_nav",
    "/cmd_vel_sim",
    "/simulation/collision",
    "/simulation/collision_diagnostics",
    "/experiment/obstacles/state",
)

CAPTURE_SCHEMA = {
    "/bio_nav/navigation_graph": "NavigationGraph",
    "/bio_nav/canonical_route": "CanonicalRoute",
    "/bio_nav/route_progress": "RouteProgress",
    "/cmd_vel": "Twist",
    "/cmd_vel_nav": "Twist",
    "/cmd_vel_sim": "Twist",
    "/simulation/collision": "Bool",
    "/simulation/collision_diagnostics": "String",
    "/bio_nav/route_goal_result": "String",
    "/experiment/obstacles/state": "String",
    "/experiment/appearance/state": "String",
}

if any(topic.startswith(GT_PREFIX) for topic in DISPATCH_SUBSCRIPTION_TOPICS):
    raise RuntimeError("V6 dispatcher Ground Truth firewall violated")


class V6ContractError(RuntimeError):
    """A fail-closed V6 manifest or episode contract violation."""


def append_evidence_jsonl(path: Path, event: str, **payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    row = {"event": event, "wall_time_ns": time.time_ns(), **payload}
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(row, sort_keys=True) + "\n")


@dataclass(frozen=True)
class Episode:
    seed: int
    variant_id: str
    appearance_profile_id: str | None
    reset_pose_name: str
    dynamic_case_id: str


@dataclass(frozen=True)
class MissionLeg:
    goal_id: str
    frame_id: str
    x: float
    y: float


@dataclass(frozen=True)
class DynamicScheduleEntry:
    leg_id: str
    group: str


@dataclass(frozen=True)
class Manifest:
    path: Path
    raw: Mapping[str, Any]
    scene_id: str
    category: str
    runtime: Mapping[str, Any]
    assets: Mapping[str, str]
    reset_pose: Mapping[str, Any]
    mission_legs: tuple[MissionLeg, ...]
    dynamic_schedule: tuple[DynamicScheduleEntry, ...]
    episodes: tuple[Episode, ...]


@dataclass(frozen=True)
class FormalCondition:
    condition_id: str
    scene: str
    category: str
    scenario_file: Path
    output_directory: Path
    runner_arguments: tuple[str, ...]
    scenario_id: str
    episode_identities: tuple[Mapping[str, Any], ...]


@dataclass(frozen=True)
class FormalCampaignManifest:
    path: Path
    authorization: str
    runner_entrypoint: Path
    freeze: Mapping[str, Any]
    freeze_digest: str
    pilot_freeze_provenance: Mapping[str, Any] | None
    conditions: tuple[FormalCondition, ...]


@dataclass(frozen=True)
class IndoorCampaignManifest:
    path: Path
    runner_entrypoint: Path
    freeze: Mapping[str, Any]
    freeze_digest: str
    pilot_freeze_provenance: Mapping[str, Any]
    conditions: tuple[FormalCondition, ...]


@dataclass(frozen=True)
class OutdoorCampaignManifest:
    path: Path
    runner_entrypoint: Path
    freeze: Mapping[str, Any]
    freeze_digest: str
    pilot_freeze_provenance: Mapping[str, Any]
    conditions: tuple[FormalCondition, ...]


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _constraints_tree_sha256(path: Path) -> str:
    if not path.is_dir():
        raise V6ContractError("Rivermark catalog constraints path is not a directory")
    rows = [
        f"{file.relative_to(path)}\0{_file_sha256(file)}"
        for file in sorted(item for item in path.rglob("*") if item.is_file())
    ]
    if not rows:
        raise V6ContractError("Rivermark catalog constraints tree is empty")
    return hashlib.sha256("\n".join(rows).encode()).hexdigest()


def _validate_frozen_file(
    value: Any, path: str, *, expected_path: Path | None = None
) -> dict[str, str]:
    entry = _mapping(value, path)
    _require_exact_keys(entry, {"path", "sha256"}, path)
    candidate = Path(str(entry.get("path", ""))).expanduser()
    if not candidate.is_absolute():
        raise V6ContractError(f"{path}.path must be absolute")
    candidate = candidate.resolve()
    if expected_path is not None and candidate != expected_path.resolve():
        raise V6ContractError(f"{path}.path mismatch")
    if not candidate.is_file() or not os.access(candidate, os.R_OK):
        raise V6ContractError(f"{path}.path is not a readable file")
    digest = str(entry.get("sha256", ""))
    if digest != _file_sha256(candidate):
        raise V6ContractError(f"{path}.sha256 mismatch")
    return {"path": str(candidate), "sha256": digest}


def _current_driver_version() -> str:
    try:
        return Path("/proc/driver/nvidia/version").read_text().splitlines()[0]
    except (OSError, IndexError) as exc:
        raise V6ContractError("NVIDIA driver version is unavailable") from exc


def _validate_nas_mount(path: Path) -> dict[str, str]:
    probe = path.resolve()
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    try:
        output = subprocess.run(
            ["findmnt", "-T", str(probe), "-n", "-o", "TARGET,FSTYPE,SOURCE"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise V6ContractError("formal NAS mount is unavailable") from exc
    fields = output.split(maxsplit=2)
    if len(fields) != 3:
        raise V6ContractError("formal NAS mount identity is malformed")
    target, filesystem, source = fields
    if (
        Path(target).resolve() != FORMAL_NAS_ROOT.resolve()
        or filesystem.lower() in {"overlay", "ext4", "xfs", "btrfs", "tmpfs"}
        or not source
    ):
        raise V6ContractError("formal output root is not on the current NAS mount")
    return {"target": target, "filesystem": filesystem, "source": source}


def _repository_tracked_dirty(path: Path) -> bool:
    return any(
        subprocess.run(command).returncode != 0
        for command in (
            ["git", "-C", str(path), "diff", "--quiet", "HEAD", "--"],
            ["git", "-C", str(path), "diff", "--cached", "--quiet", "--"],
        )
    )


def _validate_formal_freeze(
    value: Any,
    *,
    conditions: tuple[FormalCondition, ...],
    scenario_configs: Mapping[str, set[Path]],
    runner_entrypoint: Path,
    condition_ids: tuple[str, ...] = FORMAL_CONDITION_IDS,
    frozen_asset_keys: frozenset[str] = FORMAL_FROZEN_ASSET_KEYS,
    expected_physical_contracts: Mapping[str, Mapping[str, Any]] | None = None,
) -> tuple[dict[str, Any], str]:
    freeze = _mapping(value, "freeze")
    required_freeze_keys = {
        "repositories",
        "driver_version",
        "kernel_release",
        "scenarios",
        "scenario_configs",
        "frozen_assets",
        "runner_entrypoint",
        "experiment_runner",
        "v6_formal",
    }
    if expected_physical_contracts is not None:
        required_freeze_keys.add("physical_contracts")
        if isinstance(value, Mapping) and "validator_only_head_promotion" in value:
            required_freeze_keys.add("validator_only_head_promotion")
        if isinstance(value, Mapping) and "evaluator_hardening_promotion" in value:
            required_freeze_keys.update({
                "validator_only_head_promotion", "evaluator_hardening_promotion",
            })
        continuation_present = isinstance(value, Mapping) and (
            "indoor_continuation" in value
            or "qualification_tooling_promotion" in value
        )
        if continuation_present:
            required_freeze_keys.update({
                "validator_only_head_promotion",
                "indoor_continuation",
                "qualification_tooling_promotion",
            })
    _require_exact_keys(
        freeze,
        required_freeze_keys,
        "freeze",
    )
    repositories = _mapping(freeze.get("repositories"), "freeze.repositories")
    _require_exact_keys(
        repositories, {"integration", "module2", "module3"}, "freeze.repositories"
    )
    normalized_repositories: dict[str, dict[str, str]] = {}
    for name, value_entry in repositories.items():
        entry = _mapping(value_entry, f"freeze.repositories.{name}")
        if set(entry) not in ({"path", "head"}, {"path", "head", "tree"}):
            raise V6ContractError(
                f"freeze.repositories.{name} must bind path/head and optional tree"
            )
        repo_path = Path(str(entry.get("path", ""))).expanduser()
        if not repo_path.is_absolute():
            raise V6ContractError(f"freeze.repositories.{name}.path must be absolute")
        repo_path = repo_path.resolve()
        try:
            actual_head = subprocess.run(
                ["git", "-C", str(repo_path), "rev-parse", "HEAD"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        except (OSError, subprocess.CalledProcessError) as exc:
            raise V6ContractError(f"freeze repository is invalid: {name}") from exc
        if entry.get("head") != actual_head:
            raise V6ContractError(f"freeze repository head mismatch: {name}")
        actual_tree = subprocess.run(
            ["git", "-C", str(repo_path), "rev-parse", "HEAD^{tree}"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        if "tree" in entry and entry.get("tree") != actual_tree:
            raise V6ContractError(f"freeze repository tree mismatch: {name}")
        if _repository_tracked_dirty(repo_path):
            raise V6ContractError(f"freeze repository tracked state is dirty: {name}")
        normalized_repositories[name] = {"path": str(repo_path), "head": actual_head}
        if "tree" in entry:
            normalized_repositories[name]["tree"] = actual_tree
    driver_version = str(freeze.get("driver_version", ""))
    if driver_version != _current_driver_version():
        raise V6ContractError("freeze driver_version mismatch")
    kernel_release = str(freeze.get("kernel_release", ""))
    if kernel_release != os.uname().release:
        raise V6ContractError("freeze kernel_release mismatch")
    condition_ids = {condition.condition_id for condition in conditions}
    scenarios = _mapping(freeze.get("scenarios"), "freeze.scenarios")
    _require_exact_keys(scenarios, condition_ids, "freeze.scenarios")
    normalized_scenarios = {
        condition.condition_id: _validate_frozen_file(
            scenarios[condition.condition_id],
            f"freeze.scenarios.{condition.condition_id}",
            expected_path=condition.scenario_file,
        )
        for condition in conditions
    }
    configs = _mapping(freeze.get("scenario_configs"), "freeze.scenario_configs")
    _require_exact_keys(configs, condition_ids, "freeze.scenario_configs")
    normalized_configs: dict[str, list[dict[str, str]]] = {}
    for condition_id in condition_ids:
        rows = configs[condition_id]
        if not isinstance(rows, list):
            raise V6ContractError(f"freeze.scenario_configs.{condition_id} must be a list")
        normalized_rows = [
            _validate_frozen_file(
                row, f"freeze.scenario_configs.{condition_id}[{index}]"
            )
            for index, row in enumerate(rows)
        ]
        if {Path(row["path"]) for row in normalized_rows} != scenario_configs[condition_id]:
            raise V6ContractError(f"freeze scenario config set mismatch: {condition_id}")
        normalized_configs[condition_id] = sorted(
            normalized_rows, key=lambda row: row["path"]
        )
    asset_rows = _mapping(freeze.get("frozen_assets"), "freeze.frozen_assets")
    _require_exact_keys(asset_rows, frozen_asset_keys, "freeze.frozen_assets")
    normalized_assets = {}
    for name in sorted(frozen_asset_keys):
        if name != "rivermark_catalog_constraints_tree":
            normalized_assets[name] = _validate_frozen_file(
                asset_rows[name], f"freeze.frozen_assets.{name}"
            )
            continue
        entry = _mapping(asset_rows[name], f"freeze.frozen_assets.{name}")
        _require_exact_keys(entry, {"path", "sha256"}, f"freeze.frozen_assets.{name}")
        tree_path = Path(str(entry.get("path", ""))).expanduser()
        if not tree_path.is_absolute():
            raise V6ContractError(f"freeze.frozen_assets.{name}.path must be absolute")
        tree_path = tree_path.resolve()
        digest = _constraints_tree_sha256(tree_path)
        if entry.get("sha256") != digest:
            raise V6ContractError(f"freeze.frozen_assets.{name}.sha256 mismatch")
        normalized_assets[name] = {"path": str(tree_path), "sha256": digest}
    normalized_physical_contracts = None
    if expected_physical_contracts is not None:
        physical_rows = _mapping(
            freeze.get("physical_contracts"), "freeze.physical_contracts"
        )
        _require_exact_keys(
            physical_rows, set(condition_ids), "freeze.physical_contracts"
        )
        normalized_physical_contracts = {}
        for condition_id in condition_ids:
            path = f"freeze.physical_contracts.{condition_id}"
            row = _mapping(physical_rows[condition_id], path)
            _require_exact_keys(
                row,
                {
                    "static_obstacle_ids",
                    "static_obstacle_count",
                    "dynamic_actor_ids",
                    "dynamic_actor_count",
                    "physical_config",
                    "scenario",
                    "spawn_manifest",
                },
                path,
            )
            expected = expected_physical_contracts[condition_id]
            static_ids = row.get("static_obstacle_ids")
            dynamic_ids = row.get("dynamic_actor_ids")
            if (
                static_ids != expected["static_obstacle_ids"]
                or row.get("static_obstacle_count") != len(static_ids or [])
                or dynamic_ids != expected["dynamic_actor_ids"]
                or row.get("dynamic_actor_count") != len(dynamic_ids or [])
            ):
                raise V6ContractError(
                    f"indoor physical obstacle identity mismatch: {condition_id}"
                )
            normalized_physical_contracts[condition_id] = {
                "static_obstacle_ids": list(static_ids),
                "static_obstacle_count": len(static_ids),
                "dynamic_actor_ids": list(dynamic_ids),
                "dynamic_actor_count": len(dynamic_ids),
                "physical_config": _validate_frozen_file(
                    row.get("physical_config"),
                    f"{path}.physical_config",
                    expected_path=expected["physical_config"],
                ),
                "scenario": _validate_frozen_file(
                    row.get("scenario"),
                    f"{path}.scenario",
                    expected_path=expected["scenario"],
                ),
                "spawn_manifest": _validate_frozen_file(
                    row.get("spawn_manifest"),
                    f"{path}.spawn_manifest",
                    expected_path=expected["spawn_manifest"],
                ),
            }
    code_root = Path(__file__).resolve().parent
    normalized = {
        "repositories": normalized_repositories,
        "driver_version": driver_version,
        "kernel_release": kernel_release,
        "scenarios": normalized_scenarios,
        "scenario_configs": normalized_configs,
        "frozen_assets": normalized_assets,
        "runner_entrypoint": _validate_frozen_file(
            freeze.get("runner_entrypoint"),
            "freeze.runner_entrypoint",
            expected_path=runner_entrypoint,
        ),
        "experiment_runner": _validate_frozen_file(
            freeze.get("experiment_runner"),
            "freeze.experiment_runner",
            expected_path=code_root / "experiment_runner.py",
        ),
        "v6_formal": _validate_frozen_file(
            freeze.get("v6_formal"),
            "freeze.v6_formal",
            expected_path=Path(__file__).resolve(),
        ),
    }
    if normalized_physical_contracts is not None:
        normalized["physical_contracts"] = normalized_physical_contracts
        if "validator_only_head_promotion" in freeze:
            normalized["validator_only_head_promotion"] = dict(_mapping(
                freeze.get("validator_only_head_promotion"),
                "freeze.validator_only_head_promotion",
            ))
        if "evaluator_hardening_promotion" in freeze:
            normalized["evaluator_hardening_promotion"] = dict(_mapping(
                freeze.get("evaluator_hardening_promotion"),
                "freeze.evaluator_hardening_promotion",
            ))
            _validate_evaluator_hardening_chain(
                normalized["evaluator_hardening_promotion"], freeze=normalized
            )
        if continuation_present:
            normalized["indoor_continuation"] = dict(_mapping(
                freeze.get("indoor_continuation"),
                "freeze.indoor_continuation",
            ))
            normalized["qualification_tooling_promotion"] = dict(_mapping(
                freeze.get("qualification_tooling_promotion"),
                "freeze.qualification_tooling_promotion",
            ))
    canonical = json.dumps(normalized, sort_keys=True, separators=(",", ":"))
    return normalized, hashlib.sha256(canonical.encode()).hexdigest()


def _canonical_nav2_config() -> Path:
    return (
        Path(__file__).resolve().parents[4]
        / "ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml"
    ).resolve()


def _canonical_indoor_spawn_manifest() -> Path:
    return (
        Path(__file__).resolve().parents[4]
        / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
    ).resolve()


def _effective_indoor_spawn_manifest(
    arguments: tuple[str, ...] | list[str],
) -> Path:
    overrides = [
        value
        for name, value in (argument.split(":=", 1) for argument in arguments)
        if name == "spawn_poses_file"
    ]
    if len(overrides) != 1:
        raise V6ContractError(
            "indoor runner_arguments must contain exactly one spawn_poses_file override"
        )
    candidate = Path(overrides[0]).expanduser()
    if not candidate.is_absolute():
        raise V6ContractError("spawn_poses_file override must be absolute")
    candidate = candidate.resolve()
    if candidate != _canonical_indoor_spawn_manifest():
        raise V6ContractError(
            "spawn_poses_file override must select canonical V6 IsaacGen spawn"
        )
    if not candidate.is_file():
        raise V6ContractError("canonical V6 IsaacGen spawn manifest is missing")
    return candidate


def _indoor_physical_contract_expectations(
    conditions: tuple[FormalCondition, ...],
) -> dict[str, dict[str, Any]]:
    expectations = {}
    for condition in conditions:
        scenario = load_scenario(condition.scenario_file)
        static_ids = sorted(
            str(row["id"])
            for row in scenario.obstacles.get("static", [])
            if isinstance(row, Mapping) and isinstance(row.get("id"), str)
        )
        dynamic_ids = sorted(
            str(row["id"])
            for row in scenario.obstacle_trajectories
            if isinstance(row, Mapping) and isinstance(row.get("id"), str)
        )
        if (static_ids, dynamic_ids) != INDOOR_EXPECTED_OBSTACLES.get(
            condition.condition_id
        ):
            raise V6ContractError(
                f"indoor scenario obstacle contract mismatch: {condition.condition_id}"
            )
        if scenario.dynamic_config_file is None:
            raise V6ContractError(
                f"indoor physical config is missing: {condition.condition_id}"
            )
        expectations[condition.condition_id] = {
            "static_obstacle_ids": static_ids,
            "dynamic_actor_ids": dynamic_ids,
            "physical_config": scenario.resolve_path(scenario.dynamic_config_file),
            "scenario": condition.scenario_file,
            "spawn_manifest": _canonical_indoor_spawn_manifest(),
        }
    return expectations


def _effective_nav2_config(arguments: tuple[str, ...] | list[str]) -> Path:
    overrides = [
        value
        for name, value in (argument.split(":=", 1) for argument in arguments)
        if name == "nav2_config_file"
    ]
    if len(overrides) != 1:
        raise V6ContractError(
            "runner_arguments must contain exactly one nav2_config_file override"
        )
    candidate = Path(overrides[0]).expanduser()
    if not candidate.is_absolute():
        raise V6ContractError("nav2_config_file override must be absolute")
    candidate = candidate.resolve()
    if candidate != _canonical_nav2_config():
        raise V6ContractError(
            "nav2_config_file override must select canonical low-obstacle config"
        )
    if not candidate.is_file():
        raise V6ContractError("canonical nav2_config_file override is missing")
    return candidate


def _scenario_runtime_config_paths(
    scenario: Any, runner_arguments: tuple[str, ...] | list[str]
) -> set[Path]:
    return {
        scenario.resolve_path(path)
        for path in (
            scenario.robot_config_file,
            scenario.dynamic_config_file,
            scenario.appearance_config_file,
            scenario.optimal_reference_file,
        )
        if path is not None
    } | {_effective_nav2_config(runner_arguments)}


def _expected_scenario_runtime_hashes(
    condition: FormalCondition,
) -> tuple[str, str, dict[str, str]]:
    scenario = load_scenario(condition.scenario_file)
    robot_path = scenario.resolve_path(scenario.robot_config_file)
    nav2_path = _effective_nav2_config(condition.runner_arguments)
    runtime_hashes = {
        name: _file_sha256(path)
        for name, path in (
            ("robot_config", robot_path),
            ("nav2_config", nav2_path),
            (
                "dynamic_config",
                scenario.resolve_path(scenario.dynamic_config_file)
                if scenario.dynamic_config_file is not None
                else None,
            ),
            (
                "appearance_config",
                scenario.resolve_path(scenario.appearance_config_file)
                if scenario.appearance_config_file is not None
                else None,
            ),
            (
                "optimal_reference",
                scenario.resolve_path(scenario.optimal_reference_file)
                if scenario.optimal_reference_file is not None
                else None,
            ),
        )
        if path is not None
    }
    return _file_sha256(robot_path), _file_sha256(nav2_path), runtime_hashes


def _validate_indoor_static_reference_contract(
    condition: FormalCondition, freeze: Mapping[str, Any]
) -> None:
    scenario = load_scenario(condition.scenario_file)
    if scenario.optimal_reference_file is None:
        raise V6ContractError("indoor static optimal reference is missing")
    reference_path = scenario.resolve_path(scenario.optimal_reference_file)
    try:
        reference = _mapping(
            json.loads(reference_path.read_text(encoding="utf-8")),
            "indoor_static_reference",
        )
    except (OSError, json.JSONDecodeError) as exc:
        raise V6ContractError("indoor static optimal reference is unreadable") from exc
    physical = freeze["physical_contracts"][condition.condition_id]
    map_identity = _mapping(reference.get("map"), "indoor_static_reference.map")
    scenario_identity = _mapping(
        reference.get("scenario"), "indoor_static_reference.scenario"
    )
    total = reference.get("total_length_m_0_05")
    legs = reference.get("legs")
    if not (
        reference.get("converged") is True
        and isinstance(total, (int, float))
        and not isinstance(total, bool)
        and math.isfinite(float(total))
        and float(total) > 0.0
        and isinstance(legs, list)
        and len(legs) == 5
        and all(
            isinstance(row, Mapping)
            and isinstance(row.get("length_m_0_05"), (int, float))
            and not isinstance(row.get("length_m_0_05"), bool)
            and math.isfinite(float(row["length_m_0_05"]))
            and float(row["length_m_0_05"]) > 0.0
            for row in legs
        )
        and map_identity.get("id") == scenario.map_version
        and map_identity.get("yaml_sha256")
        == freeze["frozen_assets"]["indoor_map_yaml"]["sha256"]
        and map_identity.get("image_sha256")
        == freeze["frozen_assets"]["indoor_map_pgm"]["sha256"]
        and scenario_identity.get("id") == condition.scenario_id
        and scenario_identity.get("obstacle_id") == "v6_low_box_solo"
        and scenario_identity.get("obstacle_sha256")
        == physical["physical_config"]["sha256"]
        and scenario_identity.get("spawn_pose_name") == scenario.spawn_pose_name
        and scenario_identity.get("spawn_sha256")
        == physical["spawn_manifest"]["sha256"]
    ):
        raise V6ContractError(
            "indoor static optimal reference identity/metrics mismatch"
        )


def load_formal_campaign_manifest(path: str | Path) -> FormalCampaignManifest:
    manifest_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    raw = _mapping(raw, "formal_manifest")
    required_manifest_keys = {
        "schema_version",
        "intended_use",
        "execution_authorization",
        "runs_per_condition",
        "runner_entrypoint",
        "freeze",
        "conditions",
    }
    if frozenset(raw) not in {
        frozenset(required_manifest_keys),
        frozenset(required_manifest_keys | {"pilot_freeze_provenance"}),
    }:
        raise V6ContractError("formal_manifest keys are invalid")
    if raw.get("schema_version") != FORMAL_CAMPAIGN_SCHEMA_VERSION:
        raise V6ContractError(
            f"formal schema_version must be {FORMAL_CAMPAIGN_SCHEMA_VERSION}"
        )
    if raw.get("intended_use") != "formal_qualification":
        raise V6ContractError("formal intended_use must be formal_qualification")
    authorization = str(raw.get("execution_authorization", ""))
    if authorization not in {
        FORMAL_EXECUTION_AUTHORIZED,
        FORMAL_EXECUTION_NOT_AUTHORIZED,
    }:
        raise V6ContractError(
            "execution_authorization must be AUTHORIZED or NOT_AUTHORIZED"
        )
    if raw.get("runs_per_condition") != FORMAL_RUNS_PER_CONDITION:
        raise V6ContractError("runs_per_condition must be exactly 20")
    runner_entrypoint = Path(str(raw.get("runner_entrypoint", ""))).expanduser()
    if not runner_entrypoint.is_absolute():
        runner_entrypoint = manifest_path.parent / runner_entrypoint
    runner_entrypoint = runner_entrypoint.resolve()
    if not runner_entrypoint.is_file() or not os.access(
        runner_entrypoint, os.R_OK | os.X_OK
    ):
        raise V6ContractError("runner_entrypoint must be a readable executable file")
    condition_rows = raw.get("conditions")
    if not isinstance(condition_rows, list) or len(condition_rows) != len(
        FORMAL_CONDITION_IDS
    ):
        raise V6ContractError("formal conditions must contain exactly six rows")
    conditions: list[FormalCondition] = []
    scenario_configs: dict[str, set[Path]] = {}
    output_directories: set[Path] = set()
    for index, value in enumerate(condition_rows):
        row = _mapping(value, f"conditions[{index}]")
        _require_exact_keys(
            row,
            {
                "id",
                "scene",
                "category",
                "scenario_file",
                "output_directory",
                "runner_arguments",
            },
            f"conditions[{index}]",
        )
        condition_id = str(row.get("id", ""))
        scene = str(row.get("scene", ""))
        category = str(row.get("category", ""))
        if condition_id != f"{scene}_{category}":
            raise V6ContractError(
                f"conditions[{index}] id must equal scene_category"
            )
        scenario_file = Path(str(row.get("scenario_file", ""))).expanduser()
        if not scenario_file.is_absolute():
            scenario_file = manifest_path.parent / scenario_file
        scenario_file = scenario_file.resolve()
        if not scenario_file.is_file():
            raise V6ContractError(
                f"conditions[{index}].scenario_file is missing: {scenario_file}"
            )
        scenario = load_scenario(scenario_file)
        expected_scenario_type = "static" if category == "appearance" else category
        if scenario.scenario_type != expected_scenario_type:
            raise V6ContractError(
                f"conditions[{index}] category differs from scenario type"
            )
        if category == "appearance" and scenario.appearance_config_file is None:
            raise V6ContractError(
                f"conditions[{index}] appearance scenario has no appearance config"
            )
        actual_scene = (
            "outdoor"
            if "rivermark" in f"{scenario.scenario_id} {scenario.map_version}".lower()
            else "indoor"
        )
        if actual_scene != scene:
            raise V6ContractError(
                f"conditions[{index}] scene differs from scenario identity"
            )
        selections = scenario.run_matrix or tuple(
            {
                "seed": seed,
                "condition_id": None,
                "dynamic_case_id": None,
                "dynamic_variant_id": None,
                "appearance_profile_id": None,
            }
            for seed in scenario.seeds
        )
        episode_identities: list[dict[str, Any]] = []
        for selection in selections:
            if isinstance(selection, Mapping):
                identity = dict(selection)
            else:
                identity = {
                    "seed": selection.seed,
                    "condition_id": selection.condition_id,
                    "dynamic_case_id": selection.case_id,
                    "dynamic_variant_id": selection.variant_id,
                    "appearance_profile_id": selection.appearance_profile_id,
                }
            episode_identities.append(identity)
        identity_keys = {
            (
                identity.get("seed"),
                identity.get("condition_id"),
                identity.get("dynamic_case_id"),
                identity.get("dynamic_variant_id"),
                identity.get("appearance_profile_id"),
            )
            for identity in episode_identities
        }
        if len(episode_identities) != FORMAL_RUNS_PER_CONDITION or len(
            identity_keys
        ) != FORMAL_RUNS_PER_CONDITION:
            raise V6ContractError(
                f"conditions[{index}] scenario must contain 20 unique run identities"
            )
        output_directory = Path(
            str(row.get("output_directory", ""))
        ).expanduser()
        if not output_directory.is_absolute():
            raise V6ContractError(
                f"conditions[{index}].output_directory must be absolute"
            )
        output_directory = output_directory.resolve()
        if output_directory in output_directories:
            raise V6ContractError("formal output directories must be unique")
        output_directories.add(output_directory)
        arguments = row.get("runner_arguments")
        if not isinstance(arguments, list) or not all(
            isinstance(argument, str) and ":=" in argument
            for argument in arguments
        ):
            raise V6ContractError(
                f"conditions[{index}].runner_arguments must be ROS name:=value strings"
            )
        reserved = {
            "scenario_file",
            "output_directory",
            "run_indices",
            "resume",
            "record_evidence",
            "record_bag",
            "fail_stop",
            "condition_stack_id",
            "stack_session_id",
            "condition_stack_contract_path",
        }
        if any(argument.split(":=", 1)[0] in reserved for argument in arguments):
            raise V6ContractError(
                f"conditions[{index}].runner_arguments overrides dispatcher ownership"
            )
        argument_pairs = [argument.split(":=", 1) for argument in arguments]
        if len({name for name, _value in argument_pairs}) != len(argument_pairs):
            raise V6ContractError(
                f"conditions[{index}].runner_arguments contains duplicate names"
            )
        argument_map = dict(argument_pairs)
        if argument_map.get("navigation_execution_backend") != "route_guided":
            raise V6ContractError(
                f"conditions[{index}] must freeze route_guided execution"
            )
        if argument_map.get("require_module2_planning_ready") != "true":
            raise V6ContractError(
                f"conditions[{index}] must require Module2 planning readiness"
            )
        scenario_configs[condition_id] = _scenario_runtime_config_paths(
            scenario, arguments
        )
        conditions.append(
            FormalCondition(
                condition_id=condition_id,
                scene=scene,
                category=category,
                scenario_file=scenario_file,
                output_directory=output_directory,
                runner_arguments=tuple(arguments),
                scenario_id=scenario.scenario_id,
                episode_identities=tuple(episode_identities),
            )
        )
    if tuple(condition.condition_id for condition in conditions) != FORMAL_CONDITION_IDS:
        raise V6ContractError(
            "formal conditions must use the frozen indoor/outdoor execution order"
        )
    condition_tuple = tuple(conditions)
    freeze, freeze_digest = _validate_formal_freeze(
        raw.get("freeze"),
        conditions=condition_tuple,
        scenario_configs=scenario_configs,
        runner_entrypoint=runner_entrypoint,
    )
    pilot_freeze_provenance = raw.get("pilot_freeze_provenance")
    if pilot_freeze_provenance is not None:
        pilot_freeze_provenance = _mapping(
            pilot_freeze_provenance, "pilot_freeze_provenance"
        )
        _require_exact_keys(
            pilot_freeze_provenance,
            {"schema", "pilot_manifest", "pilot_aggregate", "episodes"},
            "pilot_freeze_provenance",
        )
        if pilot_freeze_provenance.get("schema") != "bio_nav.v6_pilot_freeze_provenance.v1":
            raise V6ContractError("pilot freeze provenance schema mismatch")
        episodes = pilot_freeze_provenance.get("episodes")
        if not isinstance(episodes, list) or len(episodes) != 18:
            raise V6ContractError("pilot freeze provenance must index 18 episodes")
        _revalidate_pilot_freeze_provenance(
            pilot_freeze_provenance,
            conditions=condition_tuple,
            freeze=freeze,
            freeze_digest=freeze_digest,
        )
    elif authorization == FORMAL_EXECUTION_AUTHORIZED:
        raise V6ContractError(
            "AUTHORIZED formal manifest requires complete Pilot freeze provenance"
        )
    return FormalCampaignManifest(
        path=manifest_path,
        authorization=authorization,
        runner_entrypoint=runner_entrypoint,
        freeze=freeze,
        freeze_digest=freeze_digest,
        pilot_freeze_provenance=pilot_freeze_provenance,
        conditions=condition_tuple,
    )


def _validator_only_git_output(
    repository: Path, arguments: list[str], *, binary: bool
) -> str | bytes:
    try:
        result = subprocess.run(
            ["git", "-C", str(repository), *arguments],
            check=True,
            capture_output=True,
            text=not binary,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise V6ContractError(
            f"validator-only promotion git command failed: {' '.join(arguments)}"
        ) from exc
    return result.stdout


def _validator_only_diff_evidence(
    repository: Path, from_head: str, to_head: str
) -> tuple[list[dict[str, str]], str]:
    raw_name_status = str(_validator_only_git_output(
        repository,
        ["diff", "--name-status", "--no-renames", from_head, to_head, "--"],
        binary=False,
    ))
    rows: list[dict[str, str]] = []
    for line in raw_name_status.splitlines():
        fields = line.split("\t")
        if len(fields) != 2:
            raise V6ContractError("validator-only promotion name-status is malformed")
        rows.append({"status": fields[0], "path": fields[1]})
    canonical_diff = _validator_only_git_output(
        repository,
        [
            "diff", "--binary", "--full-index", "--no-ext-diff", "--no-renames",
            from_head, to_head, "--",
        ],
        binary=True,
    )
    assert isinstance(canonical_diff, bytes)
    return rows, hashlib.sha256(canonical_diff).hexdigest()


def _validator_only_ast_guard(
    repository: Path, from_head: str, to_head: str
) -> None:
    import ast
    import copy

    experiment_runner_path = (
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
    )
    v6_formal_path = "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py"
    body_only = {
        experiment_runner_path: {
            "FunctionDef:validate_recorded_run_evidence",
        },
        v6_formal_path: {
            "FunctionDef:_validate_formal_freeze",
            "FunctionDef:load_indoor_campaign_manifest",
            "FunctionDef:freeze_indoor_campaign_from_pilot",
            "FunctionDef:_build_indoor_pilot_manifest",
            "FunctionDef:aggregate_indoor_pilot",
            "FunctionDef:execute_indoor_campaign",
        },
    }
    allowed_assignments = {
        experiment_runner_path: set(),
        v6_formal_path: {
            "Assign:INDOOR_PILOT_MANIFEST_SCHEMA",
            "Assign:INDOOR_PILOT_AGGREGATE_SCHEMA",
            "Assign:INDOOR_CAMPAIGN_SCHEMA_VERSION",
        },
    }
    schema_values = {
        "Assign:INDOOR_PILOT_MANIFEST_SCHEMA": (
            "bio_nav_v6_indoor_pilot_manifest_v1",
            "bio_nav_v6_indoor_pilot_manifest_v2",
        ),
        "Assign:INDOOR_PILOT_AGGREGATE_SCHEMA": (
            "bio_nav_v6_indoor_pilot_aggregate_v1",
            "bio_nav_v6_indoor_pilot_aggregate_v2",
        ),
        "Assign:INDOOR_CAMPAIGN_SCHEMA_VERSION": (
            "bio_nav_v6_indoor_campaign_v1",
            "bio_nav_v6_indoor_campaign_v2",
        ),
    }
    new_helpers = {
        "FunctionDef:_validator_only_git_output": (
            ("repository", "arguments"), ("binary",)
        ),
        "FunctionDef:_validator_only_diff_evidence": (
            ("repository", "from_head", "to_head"), ()
        ),
        "FunctionDef:_validator_only_ast_guard": (
            ("repository", "from_head", "to_head"), ()
        ),
        "FunctionDef:_validator_only_loaded_identity": (
            ("module3_root", "current_head"), ()
        ),
        "FunctionDef:_validate_validator_only_head_promotion": (
            ("value",), ("freeze",)
        ),
        "FunctionDef:_build_validator_only_head_promotion": (
            (), ("freeze", "pilot_runtime")
        ),
        "FunctionDef:_derive_indoor_pilot_runtime": (
            ("pilot_root",), ("repositories",)
        ),
        "FunctionDef:_pilot_freeze_from_validator_promotion": (
            ("freeze", "promotion"), ()
        ),
        "FunctionDef:_revalidate_indoor_pilot_freeze_provenance": (
            ("provenance",), ("conditions", "freeze", "freeze_digest")
        ),
    }

    def node_key(node: ast.AST, anonymous_index: int) -> str:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            return f"{type(node).__name__}:{node.name}"
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            names = [target.id for target in targets if isinstance(target, ast.Name)]
            if names:
                return f"{type(node).__name__}:{','.join(names)}"
        return f"{type(node).__name__}:#{anonymous_index}"

    for relative_path in (experiment_runner_path, v6_formal_path):
        sources = []
        for head in (from_head, to_head):
            value = _validator_only_git_output(
                repository, ["show", f"{head}:{relative_path}"], binary=False
            )
            assert isinstance(value, str)
            sources.append(value)
        parsed = [
            ast.parse(source, filename=relative_path, type_comments=True)
            for source in sources
        ]
        keyed_nodes = []
        protected_rows = []
        for source, tree in zip(sources, parsed):
            anonymous_counts: dict[str, int] = {}
            rows = []
            nodes: dict[str, ast.AST] = {}
            for node in tree.body:
                kind = type(node).__name__
                anonymous_index = anonymous_counts.get(kind, 0)
                key = node_key(node, anonymous_index)
                if key.endswith(f":#{anonymous_index}"):
                    anonymous_counts[kind] = anonymous_index + 1
                if key in nodes:
                    raise V6ContractError(
                        f"validator-only promotion duplicated top-level symbol: {key}"
                    )
                nodes[key] = node
                if (
                    key in body_only[relative_path]
                    or key in allowed_assignments[relative_path]
                    or (relative_path == v6_formal_path and key in new_helpers)
                ):
                    continue
                segment = ast.get_source_segment(source, node)
                rows.append((key, segment, ast.dump(node, include_attributes=False)))
            keyed_nodes.append(nodes)
            protected_rows.append(rows)
        if protected_rows[0] != protected_rows[1]:
            raise V6ContractError(
                f"validator-only promotion changed protected top-level AST: {relative_path}"
            )
        for key in body_only[relative_path]:
            before = keyed_nodes[0].get(key)
            after = keyed_nodes[1].get(key)
            if not isinstance(before, ast.FunctionDef) or not isinstance(
                after, ast.FunctionDef
            ):
                raise V6ContractError(
                    f"validator-only promotion callable is missing or changed kind: {key}"
                )
            headers = []
            for node in (before, after):
                header = copy.deepcopy(node)
                header.body = [ast.Pass()]
                headers.append(ast.dump(header, include_attributes=False))
            if headers[0] != headers[1]:
                raise V6ContractError(
                    f"validator-only promotion changed callable signature/decorators: {key}"
                )
        for key in allowed_assignments[relative_path]:
            before = keyed_nodes[0].get(key)
            after = keyed_nodes[1].get(key)
            if not isinstance(before, ast.Assign) or not isinstance(after, ast.Assign):
                raise V6ContractError(
                    f"validator-only promotion schema assignment is missing: {key}"
                )
            target_name = key.split(":", 1)[1]
            if not (
                len(before.targets) == 1
                and isinstance(before.targets[0], ast.Name)
                and before.targets[0].id == target_name
                and len(after.targets) == 1
                and isinstance(after.targets[0], ast.Name)
                and after.targets[0].id == target_name
                and isinstance(before.value, ast.Constant)
                and isinstance(before.value.value, str)
                and before.value.value == schema_values[key][0]
                and isinstance(after.value, ast.Constant)
                and isinstance(after.value.value, str)
                and after.value.value == schema_values[key][1]
            ):
                raise V6ContractError(
                    f"validator-only promotion schema assignment is not v1-to-v2: {key}"
                )
            before_shell = copy.deepcopy(before)
            after_shell = copy.deepcopy(after)
            before_shell.value = ast.Constant(value=None)
            after_shell.value = ast.Constant(value=None)
            if ast.dump(before_shell, include_attributes=False) != ast.dump(
                after_shell, include_attributes=False
            ):
                raise V6ContractError(
                    f"validator-only promotion schema assignment shell changed: {key}"
                )
        if relative_path == v6_formal_path:
            for key, (positional, keyword_only) in new_helpers.items():
                if key in keyed_nodes[0]:
                    raise V6ContractError(
                        f"validator-only promotion helper already existed at Pilot HEAD: {key}"
                    )
                node = keyed_nodes[1].get(key)
                if not isinstance(node, ast.FunctionDef) or node.decorator_list:
                    raise V6ContractError(
                        f"validator-only promotion helper kind/decorator mismatch: {key}"
                    )
                arguments = node.args
                actual_positional = tuple(
                    argument.arg
                    for argument in (*arguments.posonlyargs, *arguments.args)
                )
                actual_keyword_only = tuple(
                    argument.arg for argument in arguments.kwonlyargs
                )
                if (
                    actual_positional != positional
                    or actual_keyword_only != keyword_only
                    or arguments.vararg is not None
                    or arguments.kwarg is not None
                    or arguments.defaults
                    or any(value is not None for value in arguments.kw_defaults)
                ):
                    raise V6ContractError(
                        f"validator-only promotion helper signature/default mismatch: {key}"
                    )


def _validator_only_loaded_identity(
    module3_root: Path, current_head: str
) -> dict[str, str]:
    from robot_experiments.experiment_runner import validate_recorded_run_evidence

    relative_path = (
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
    )
    expected_path = (module3_root / relative_path).resolve()
    loaded_path = Path(validate_recorded_run_evidence.__code__.co_filename).resolve()
    if (
        validate_recorded_run_evidence.__module__
        != "robot_experiments.experiment_runner"
        or loaded_path != expected_path
    ):
        raise V6ContractError("validator-only promotion loaded validator source mismatch")
    tracked_source = _validator_only_git_output(
        module3_root, ["show", f"{current_head}:{relative_path}"], binary=True
    )
    assert isinstance(tracked_source, bytes)
    try:
        loaded_source = loaded_path.read_bytes()
    except OSError as exc:
        raise V6ContractError(
            "validator-only promotion loaded validator source is unreadable"
        ) from exc
    if loaded_source != tracked_source:
        raise V6ContractError(
            "validator-only promotion loaded validator bytes differ from current HEAD"
        )
    blob = str(_validator_only_git_output(
        module3_root,
        ["rev-parse", f"{current_head}:{relative_path}"],
        binary=False,
    )).strip()
    return {
        "module": validate_recorded_run_evidence.__module__,
        "symbol": validate_recorded_run_evidence.__name__,
        "source_path": str(loaded_path),
        "source_sha256": hashlib.sha256(loaded_source).hexdigest(),
        "git_blob_oid": blob,
        "current_head": current_head,
    }


def _evaluator_only_ast_guard(
    repository: Path, from_head: str, to_head: str
) -> None:
    import ast
    import copy

    experiment_path = (
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
    )
    formal_path = "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py"
    allowed_bodies = {
        experiment_path: {"FunctionDef:_recorded_cognitive_admission_evidence"},
        formal_path: {
            "FunctionDef:_validate_formal_freeze",
            "FunctionDef:load_indoor_campaign_manifest",
            "FunctionDef:freeze_indoor_campaign_from_pilot",
            "FunctionDef:_revalidate_indoor_pilot_freeze_provenance",
            "FunctionDef:_validate_validator_only_head_promotion",
            "FunctionDef:_validate_historical_validator_promotion",
        },
    }

    def keyed(tree: ast.Module) -> dict[str, ast.AST]:
        rows: dict[str, ast.AST] = {}
        anonymous: dict[str, int] = {}
        for node in tree.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                key = f"{type(node).__name__}:{node.name}"
            elif isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(
                node.targets[0], ast.Name
            ):
                key = f"Assign:{node.targets[0].id}"
            else:
                kind = type(node).__name__
                index = anonymous.get(kind, 0)
                anonymous[kind] = index + 1
                key = f"{kind}:#{index}"
            if key in rows:
                raise V6ContractError(
                    f"evaluator promotion duplicated top-level symbol: {key}"
                )
            rows[key] = node
        return rows

    parsed: dict[str, tuple[list[str], list[dict[str, ast.AST]]]] = {}
    for relative_path in (experiment_path, formal_path):
        sources = []
        for head in (from_head, to_head):
            source = _validator_only_git_output(
                repository, ["show", f"{head}:{relative_path}"], binary=False
            )
            assert isinstance(source, str)
            sources.append(source)
        nodes = [
            keyed(ast.parse(source, filename=relative_path, type_comments=True))
            for source in sources
        ]
        parsed[relative_path] = (sources, nodes)

    formal_sources, formal_nodes = parsed[formal_path]
    del formal_sources

    def marker_value(nodes: dict[str, ast.AST]):
        marker = nodes.get("Assign:EVALUATOR_PROMOTION_MIGRATION")
        if marker is None:
            return None
        if not (
            isinstance(marker, ast.Assign)
            and len(marker.targets) == 1
            and isinstance(marker.targets[0], ast.Name)
            and marker.targets[0].id == "EVALUATOR_PROMOTION_MIGRATION"
            and isinstance(marker.value, ast.Constant)
        ):
            raise V6ContractError("evaluator promotion migration marker is invalid")
        return marker.value.value

    before_marker = marker_value(formal_nodes[0])
    after_marker = marker_value(formal_nodes[1])
    active_markers = {True, "retire_after_aggregate"}
    if before_marker is None and after_marker in active_markers:
        transition = "migration"
    elif before_marker in active_markers and after_marker is False:
        transition = "hardening"
    elif before_marker is False and after_marker is False:
        transition = "steady"
    else:
        raise V6ContractError(
            "evaluator promotion marker transition is not one-way"
        )

    for relative_path in (experiment_path, formal_path):
        sources, nodes = parsed[relative_path]
        protected = []
        for source, keyed_nodes in zip(sources, nodes):
            rows = []
            for key, node in keyed_nodes.items():
                if key in allowed_bodies[relative_path] or (
                    relative_path == formal_path
                    and key in {
                        "FunctionDef:_evaluator_only_ast_guard",
                        "FunctionDef:_validate_evaluator_hardening_chain",
                        "Assign:EVALUATOR_PROMOTION_MIGRATION",
                    }
                ):
                    continue
                rows.append((
                    key,
                    ast.get_source_segment(source, node),
                    ast.dump(node, include_attributes=False),
                ))
            protected.append(rows)
        if protected[0] != protected[1]:
            raise V6ContractError(
                f"evaluator promotion changed protected AST: {relative_path}"
            )
        for key in allowed_bodies[relative_path]:
            before = nodes[0].get(key)
            after = nodes[1].get(key)
            if before is None and after is None:
                continue
            if not isinstance(before, ast.FunctionDef) or not isinstance(
                after, ast.FunctionDef
            ):
                raise V6ContractError(
                    f"evaluator promotion callable missing or changed kind: {key}"
                )
            headers = []
            for node in (before, after):
                header = copy.deepcopy(node)
                header.body = [ast.Pass()]
                headers.append(ast.dump(header, include_attributes=False))
            if headers[0] != headers[1]:
                raise V6ContractError(
                    f"evaluator promotion callable signature changed: {key}"
                )
        if relative_path == formal_path:
            before_helper = nodes[0].get("FunctionDef:_evaluator_only_ast_guard")
            after_helper = nodes[1].get("FunctionDef:_evaluator_only_ast_guard")
            if not isinstance(after_helper, ast.FunctionDef) or after_helper.decorator_list:
                raise V6ContractError("evaluator promotion AST guard helper mismatch")
            if transition == "migration":
                if before_helper is not None:
                    raise V6ContractError(
                        "evaluator promotion AST guard existed before migration"
                    )
            elif not isinstance(before_helper, ast.FunctionDef):
                raise V6ContractError(
                    "evaluator promotion AST guard missing before hardening"
                )
            arguments = after_helper.args
            if (
                tuple(
                    item.arg for item in (*arguments.posonlyargs, *arguments.args)
                ) != ("repository", "from_head", "to_head")
                or arguments.kwonlyargs
                or arguments.vararg is not None
                or arguments.kwarg is not None
                or arguments.defaults
            ):
                raise V6ContractError(
                    "evaluator promotion AST guard helper signature mismatch"
                )
            if isinstance(before_helper, ast.FunctionDef):
                headers = []
                for node in (before_helper, after_helper):
                    header = copy.deepcopy(node)
                    header.body = [ast.Pass()]
                    headers.append(ast.dump(header, include_attributes=False))
                if headers[0] != headers[1]:
                    raise V6ContractError(
                        "evaluator promotion AST guard signature changed"
                    )
            chain_helper = nodes[1].get(
                "FunctionDef:_validate_evaluator_hardening_chain"
            )
            before_chain_helper = nodes[0].get(
                "FunctionDef:_validate_evaluator_hardening_chain"
            )
            if transition == "hardening":
                if before_chain_helper is not None or not isinstance(
                    chain_helper, ast.FunctionDef
                ):
                    raise V6ContractError(
                        "evaluator hardening chain helper mismatch"
                    )
                arguments = chain_helper.args
                if (
                    tuple(
                        item.arg
                        for item in (*arguments.posonlyargs, *arguments.args)
                    ) != ("value",)
                    or tuple(item.arg for item in arguments.kwonlyargs)
                    != ("freeze", "pilot_manifest_sha256", "pilot_aggregate_sha256")
                        or arguments.vararg is not None
                        or arguments.kwarg is not None
                        or arguments.defaults
                        or len(arguments.kw_defaults) != 3
                        or arguments.kw_defaults[0] is not None
                        or any(
                            item is None for item in arguments.kw_defaults[1:]
                        )
                ):
                    raise V6ContractError(
                        "evaluator hardening chain helper signature mismatch"
                    )
            elif not (
                (before_chain_helper is None and chain_helper is None)
                or (
                    isinstance(before_chain_helper, ast.FunctionDef)
                    and isinstance(chain_helper, ast.FunctionDef)
                    and ast.dump(before_chain_helper, include_attributes=False)
                    == ast.dump(chain_helper, include_attributes=False)
                )
            ):
                    raise V6ContractError(
                        "evaluator hardening chain helper changed outside hardening"
                    )
    diff_rows, _digest = _validator_only_diff_evidence(
        repository, from_head, to_head
    )
    diff_paths = {row["path"] for row in diff_rows}
    migration_paths = {
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py",
        "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
        "ros2_ws/src/robot_experiments/test/test_v6_formal.py",
    }
    hardening_paths = {
        "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
        "ros2_ws/src/robot_experiments/test/test_v6_formal.py",
    }
    steady_paths = {
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py",
        "ros2_ws/src/robot_experiments/test/test_v6_formal.py",
    }
    if (
        any(row["status"] != "M" for row in diff_rows)
        or (transition == "migration" and diff_paths != migration_paths)
        or (transition == "hardening" and diff_paths != hardening_paths)
        or (
            transition == "steady"
            and (
                not diff_paths <= steady_paths
                or "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
                not in diff_paths
            )
        )
    ):
        raise V6ContractError(
            "validator-only evaluator promotion path transition mismatch"
        )


def _validate_validator_only_head_promotion(
    value: Any, *, freeze: Mapping[str, Any]
) -> dict[str, Any]:
    promotion = dict(_mapping(value, "validator_only_head_promotion"))
    _require_exact_keys(
        promotion,
        {
            "schema", "pilot_runtime", "final_repositories", "module3_diff",
            "loaded_validator",
        },
        "validator_only_head_promotion",
    )
    if promotion.get("schema") != "bio_nav.v6_validator_only_head_promotion.v1":
        raise V6ContractError("validator-only head promotion schema mismatch")
    final_repositories = _mapping(
        promotion.get("final_repositories"),
        "validator_only_head_promotion.final_repositories",
    )
    if final_repositories != freeze["repositories"]:
        raise V6ContractError("validator-only promotion final repository tuple mismatch")
    pilot_runtime = _mapping(
        promotion.get("pilot_runtime"), "validator_only_head_promotion.pilot_runtime"
    )
    _require_exact_keys(
        pilot_runtime, {"repositories", "driver_version", "kernel_release"},
        "validator_only_head_promotion.pilot_runtime",
    )
    pilot_repositories = _mapping(
        pilot_runtime.get("repositories"),
        "validator_only_head_promotion.pilot_runtime.repositories",
    )
    _require_exact_keys(
        pilot_repositories, {"integration", "module2", "module3"},
        "validator_only_head_promotion.pilot_runtime.repositories",
    )
    for name in ("integration", "module2", "module3"):
        pilot_entry = _mapping(
            pilot_repositories.get(name),
            f"validator_only_head_promotion.pilot_runtime.repositories.{name}",
        )
        if set(pilot_entry) != {"path", "head", "tree"}:
            raise V6ContractError(
                "validator-only promotion repository identity keys are invalid"
            )
        final_entry = _mapping(freeze["repositories"][name], f"freeze.repositories.{name}")
        if set(final_entry) != {"path", "head", "tree"}:
            raise V6ContractError(
                "validator-only promotion final repository identity keys are invalid"
            )
        if Path(str(pilot_entry["path"])).resolve() != Path(str(final_entry["path"])).resolve():
            raise V6ContractError("validator-only promotion repository path mismatch")
        repository = Path(str(final_entry["path"])).resolve()
        current_head = str(_validator_only_git_output(
            repository, ["rev-parse", "HEAD"], binary=False
        )).strip()
        current_tree = str(_validator_only_git_output(
            repository, ["rev-parse", "HEAD^{tree}"], binary=False
        )).strip()
        pilot_tree = str(_validator_only_git_output(
            repository, ["rev-parse", f"{pilot_entry['head']}^{{tree}}"], binary=False
        )).strip()
        final_tree = str(_validator_only_git_output(
            repository, ["rev-parse", f"{final_entry['head']}^{{tree}}"], binary=False
        )).strip()
        if (
            current_head != final_entry["head"]
            or current_tree != final_entry["tree"]
            or pilot_tree != pilot_entry["tree"]
            or final_tree != final_entry["tree"]
            or _repository_tracked_dirty(repository)
        ):
            raise V6ContractError(
                f"validator-only promotion current checkout drift: {name}"
            )
        if name == "module2" and pilot_entry != final_entry:
            raise V6ContractError(
                "validator-only promotion changed Module2 repository"
            )
        if name == "integration" and pilot_entry["head"] != final_entry["head"]:
            try:
                subprocess.run(
                    [
                        "git", "-C", str(repository), "merge-base", "--is-ancestor",
                        str(pilot_entry["head"]), str(final_entry["head"]),
                    ],
                    check=True,
                    capture_output=True,
                )
            except (OSError, subprocess.CalledProcessError) as exc:
                raise V6ContractError(
                    "validator-only promotion Integration HEAD is not ancestral"
                ) from exc
            integration_rows, _integration_digest = _validator_only_diff_evidence(
                repository, str(pilot_entry["head"]), str(final_entry["head"])
            )
            if integration_rows != [{
                "status": "M", "path": "docs/CURRENT_STATE.md",
            }]:
                raise V6ContractError(
                    "validator-only promotion Integration diff is not docs-only"
                )
    if (
        pilot_runtime.get("driver_version") != freeze["driver_version"]
        or pilot_runtime.get("kernel_release") != freeze["kernel_release"]
    ):
        raise V6ContractError("validator-only promotion runtime platform mismatch")
    module3_root = Path(str(freeze["repositories"]["module3"]["path"])).resolve()
    from_head = str(pilot_repositories["module3"]["head"])
    to_head = str(freeze["repositories"]["module3"]["head"])
    if from_head == to_head:
        raise V6ContractError("validator-only promotion requires a new Module3 HEAD")
    try:
        subprocess.run(
            ["git", "-C", str(module3_root), "merge-base", "--is-ancestor", from_head, to_head],
            check=True,
            capture_output=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise V6ContractError("validator-only promotion from HEAD is not an ancestor") from exc
    diff_rows, canonical_diff_sha256 = _validator_only_diff_evidence(
        module3_root, from_head, to_head
    )
    legacy_allowed_paths = {
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py",
        "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
        "ros2_ws/src/robot_experiments/test/test_experiment_motion_quality.py",
        "ros2_ws/src/robot_experiments/test/test_v6_formal.py",
    }
    legacy_required_paths = {
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py",
        "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
    }
    evaluator_allowed_paths = {
        "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py",
        "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
        "ros2_ws/src/robot_experiments/test/test_v6_formal.py",
    }
    diff_paths = {row["path"] for row in diff_rows}
    legacy_promotion = bool(
        diff_rows
        and all(
            row["status"] == "M" and row["path"] in legacy_allowed_paths
            for row in diff_rows
        )
        and legacy_required_paths <= diff_paths
        and "ros2_ws/src/robot_experiments/test/test_experiment_motion_quality.py"
        in diff_paths
    )
    evaluator_shape = bool(
        diff_rows
        and all(
            row["status"] == "M" and row["path"] in evaluator_allowed_paths
            for row in diff_rows
        )
    )
    if not legacy_promotion and not evaluator_shape:
        raise V6ContractError(
            "validator-only promotion changed a disallowed path or status"
        )
    module3_diff = _mapping(
        promotion.get("module3_diff"), "validator_only_head_promotion.module3_diff"
    )
    _require_exact_keys(
        module3_diff,
        {
            "from_head", "to_head", "from_is_ancestor", "name_status",
            "canonical_diff_sha256",
        },
        "validator_only_head_promotion.module3_diff",
    )
    if (
        module3_diff.get("from_head") != from_head
        or module3_diff.get("to_head") != to_head
        or module3_diff.get("from_is_ancestor") is not True
        or module3_diff.get("name_status") != diff_rows
        or module3_diff.get("canonical_diff_sha256") != canonical_diff_sha256
    ):
        raise V6ContractError("validator-only promotion diff record mismatch")
    if legacy_promotion:
        _validator_only_ast_guard(module3_root, from_head, to_head)
    else:
        _evaluator_only_ast_guard(module3_root, from_head, to_head)
    loaded_identity = _validator_only_loaded_identity(module3_root, to_head)
    if promotion.get("loaded_validator") != loaded_identity:
        raise V6ContractError("validator-only promotion loaded validator identity mismatch")
    return promotion


def _build_validator_only_head_promotion(
    *, freeze: Mapping[str, Any], pilot_runtime: Mapping[str, Any]
) -> dict[str, Any]:
    module3_root = Path(str(freeze["repositories"]["module3"]["path"])).resolve()
    from_head = str(pilot_runtime["repositories"]["module3"]["head"])
    to_head = str(freeze["repositories"]["module3"]["head"])
    diff_rows, canonical_diff_sha256 = _validator_only_diff_evidence(
        module3_root, from_head, to_head
    )
    candidate = {
        "schema": "bio_nav.v6_validator_only_head_promotion.v1",
        "pilot_runtime": json.loads(json.dumps(pilot_runtime)),
        "final_repositories": json.loads(json.dumps(freeze["repositories"])),
        "module3_diff": {
            "from_head": from_head,
            "to_head": to_head,
            "from_is_ancestor": True,
            "name_status": diff_rows,
            "canonical_diff_sha256": canonical_diff_sha256,
        },
        "loaded_validator": _validator_only_loaded_identity(module3_root, to_head),
    }
    return _validate_validator_only_head_promotion(candidate, freeze=freeze)


def _derive_indoor_pilot_runtime(
    pilot_root: Path, *, repositories: Mapping[str, Any]
) -> dict[str, Any]:
    expected_paths = []
    for condition_id in INDOOR_CONDITION_IDS:
        for rep in range(1, 4):
            rep_root = pilot_root / condition_id / f"rep{rep}"
            matches = list(rep_root.glob("*/run-*/stack_contract.json"))
            if len(matches) != 1 or matches[0].is_symlink():
                raise V6ContractError("indoor Pilot stack snapshot topology mismatch")
            expected_paths.append(matches[0])
    payloads = []
    for path in expected_paths:
        try:
            payload = dict(_mapping(
                json.loads(path.read_text(encoding="utf-8")),
                "indoor_pilot_stack_contract",
            ))
        except (OSError, json.JSONDecodeError) as exc:
            raise V6ContractError(f"indoor Pilot stack snapshot is unreadable: {exc}") from exc
        _require_exact_keys(
            payload, _stack_contract_keys(payload), "indoor_pilot_stack_contract"
        )
        if (
            payload.get("schema") not in {
                STACK_CONTRACT_SCHEMA_V1,
                STACK_CONTRACT_SCHEMA_V2,
                STACK_CONTRACT_SCHEMA,
            }
            or payload.get("stack_session_id") != _stack_session_id(payload)
            or payload.get("integration_dirty") is not False
            or payload.get("module2_dirty") is not False
            or payload.get("module3_dirty") is not False
        ):
            raise V6ContractError("indoor Pilot stack snapshot identity/cleanliness mismatch")
        payloads.append(payload)
    head_tuples = {
        tuple(payload.get(f"{name}_head") for name in (
            "integration", "module2", "module3"
        ))
        for payload in payloads
    }
    if len(head_tuples) != 1:
        raise V6ContractError("indoor Pilot stack snapshots contain a mixed runtime tuple")
    tuples = []
    for payload in payloads:
        heads = {
            name: str(payload.get(f"{name}_head", ""))
            for name in ("integration", "module2", "module3")
        }
        trees = {}
        for name, head in heads.items():
            if payload.get("schema") in {STACK_CONTRACT_SCHEMA_V2, STACK_CONTRACT_SCHEMA}:
                trees[name] = str(payload.get(f"{name}_tree", ""))
            else:
                trees[name] = str(_validator_only_git_output(
                    Path(repositories[name]["path"]),
                    ["rev-parse", f"{head}^{{tree}}"],
                    binary=False,
                )).strip()
        tuples.append((
            heads["integration"], trees["integration"],
            heads["module2"], trees["module2"],
            heads["module3"], trees["module3"],
            payload.get("driver_version"), payload.get("kernel_release"),
            payload.get("t2_selector_path"), payload.get("t2_selector_sha256"),
        ))
    if len(set(tuples)) != 1:
        raise V6ContractError("indoor Pilot stack snapshots contain a mixed runtime tuple")
    (
        integration_head, integration_tree, module2_head, module2_tree,
        module3_head, module3_tree, driver, kernel, _selector, _selector_sha,
    ) = tuples[0]
    if not all(
        isinstance(head, str) and re.fullmatch(r"[0-9a-f]{40,64}", head)
        for head in (integration_head, module2_head, module3_head)
    ) or not all(
        isinstance(tree, str) and re.fullmatch(r"[0-9a-f]{40,64}", tree)
        for tree in (integration_tree, module2_tree, module3_tree)
    ) or not all(isinstance(value, str) and value for value in (driver, kernel)):
        raise V6ContractError("indoor Pilot stack snapshot repository HEAD is invalid")
    return {
        "repositories": {
            "integration": {
                **repositories["integration"], "head": integration_head,
                "tree": integration_tree,
            },
            "module2": {
                **repositories["module2"], "head": module2_head,
                "tree": module2_tree,
            },
            "module3": {
                **repositories["module3"], "head": module3_head,
                "tree": module3_tree,
            },
        },
        "driver_version": driver,
        "kernel_release": kernel,
    }


def _pilot_freeze_from_validator_promotion(
    freeze: Mapping[str, Any], promotion: Mapping[str, Any] | None
) -> dict[str, Any]:
    pilot_freeze = json.loads(json.dumps(freeze))
    if promotion is None:
        return pilot_freeze
    pilot_runtime = promotion["pilot_runtime"]
    pilot_freeze["repositories"] = json.loads(json.dumps(pilot_runtime["repositories"]))
    pilot_freeze["driver_version"] = pilot_runtime["driver_version"]
    pilot_freeze["kernel_release"] = pilot_runtime["kernel_release"]
    return pilot_freeze


def _validator_promotion_if_needed(
    *, freeze: Mapping[str, Any], pilot_runtime: Mapping[str, Any]
) -> dict[str, Any] | None:
    if (
        pilot_runtime["repositories"] == freeze["repositories"]
        and pilot_runtime["driver_version"] == freeze["driver_version"]
        and pilot_runtime["kernel_release"] == freeze["kernel_release"]
    ):
        return None
    return _build_validator_only_head_promotion(
        freeze=freeze, pilot_runtime=pilot_runtime
    )


def _qualification_tooling_ast_guard(
    repository: Path, from_head: str, to_head: str
) -> None:
    import ast
    import copy

    relative_path = "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py"
    sources = []
    for head in (from_head, to_head):
        value = _validator_only_git_output(
            repository, ["show", f"{head}:{relative_path}"], binary=False
        )
        assert isinstance(value, str)
        sources.append(value)
    trees = [
        ast.parse(source, filename=relative_path, type_comments=True)
        for source in sources
    ]
    body_only_names = {
        "_validate_formal_freeze",
        "load_indoor_campaign_manifest",
        "_evaluate_campaign",
        "evaluate_indoor_campaign",
        "indoor_dispatch_plan",
        "execute_indoor_campaign",
        "build_parser",
        "cli",
    }
    new_helper_shapes = {
        "_qualification_tooling_ast_guard": (
            ("repository", "from_head", "to_head"), ()
        ),
        "_qualification_source_identity": (
            ("module3_root", "current_head", "relative_path"), ()
        ),
        "_validate_historical_validator_promotion": (
            ("value",), ("parent_freeze",)
        ),
        "_validate_qualification_tooling_promotion": (
            ("value",), ("freeze", "parent_freeze")
        ),
        "_build_qualification_tooling_promotion": (
            (), ("freeze", "parent_freeze")
        ),
        "_indoor_continuation_parent_expectations": ((), ()),
        "_continuation_lexical_path": (
            ("value",), ("field", "must_exist", "leaf_kind")
        ),
        "_continuation_cli_argv_error": (("argv",), ()),
        "_validate_indoor_continuation_contract": (("manifest",), ()),
        "create_indoor_continuation_campaign": (
            (),
            (
                "parent_manifest_path",
                "output_manifest_path",
                "successor_output_root",
            ),
        ),
    }

    def keyed(tree: ast.Module) -> dict[str, ast.AST]:
        rows: dict[str, ast.AST] = {}
        anonymous: dict[str, int] = {}
        for node in tree.body:
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                key = f"{type(node).__name__}:{node.name}"
            elif isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(
                node.targets[0], ast.Name
            ):
                key = f"Assign:{node.targets[0].id}"
            else:
                kind = type(node).__name__
                index = anonymous.get(kind, 0)
                anonymous[kind] = index + 1
                key = f"{kind}:#{index}"
            if key in rows:
                raise V6ContractError(
                    f"qualification tooling promotion duplicated symbol: {key}"
                )
            rows[key] = node
        return rows

    before_nodes, after_nodes = (keyed(tree) for tree in trees)
    protected_before = []
    protected_after = []
    for source, nodes, target in (
        (sources[0], before_nodes, protected_before),
        (sources[1], after_nodes, protected_after),
    ):
        for key, node in nodes.items():
            name = key.split(":", 1)[1]
            if (
                key == "Assign:INDOOR_CONTINUATION_SCHEMA_VERSION"
                or name in body_only_names
                or name in new_helper_shapes
            ):
                continue
            target.append((
                key,
                ast.get_source_segment(source, node),
                ast.dump(node, include_attributes=False),
            ))
    if protected_before != protected_after:
        raise V6ContractError(
            "qualification tooling promotion changed protected v6_formal symbols"
        )
    for name in body_only_names:
        key = f"FunctionDef:{name}"
        before = before_nodes.get(key)
        after = after_nodes.get(key)
        if not isinstance(before, ast.FunctionDef) or not isinstance(after, ast.FunctionDef):
            raise V6ContractError(
                f"qualification tooling callable missing or changed kind: {name}"
            )
        headers = []
        for node in (before, after):
            header = copy.deepcopy(node)
            header.body = [ast.Pass()]
            headers.append(ast.dump(header, include_attributes=False))
        if headers[0] != headers[1]:
            raise V6ContractError(
                f"qualification tooling callable signature changed: {name}"
            )
    continuation_constant = after_nodes.get(
        "Assign:INDOOR_CONTINUATION_SCHEMA_VERSION"
    )
    if "Assign:INDOOR_CONTINUATION_SCHEMA_VERSION" in before_nodes or not (
        isinstance(continuation_constant, ast.Assign)
        and len(continuation_constant.targets) == 1
        and isinstance(continuation_constant.targets[0], ast.Name)
        and continuation_constant.targets[0].id
        == "INDOOR_CONTINUATION_SCHEMA_VERSION"
        and isinstance(continuation_constant.value, ast.Constant)
        and continuation_constant.value.value == INDOOR_CONTINUATION_SCHEMA_VERSION
    ):
        raise V6ContractError("qualification tooling continuation schema constant invalid")
    for name, (positional, keyword_only) in new_helper_shapes.items():
        if f"FunctionDef:{name}" in before_nodes:
            raise V6ContractError(
                f"qualification tooling helper existed before promotion: {name}"
            )
        node = after_nodes.get(f"FunctionDef:{name}")
        if not isinstance(node, ast.FunctionDef) or node.decorator_list:
            raise V6ContractError(
                f"qualification tooling helper kind/decorator mismatch: {name}"
            )
        arguments = node.args
        actual_positional = tuple(
            item.arg for item in (*arguments.posonlyargs, *arguments.args)
        )
        actual_keyword_only = tuple(item.arg for item in arguments.kwonlyargs)
        if (
            actual_positional != positional
            or actual_keyword_only != keyword_only
            or arguments.vararg is not None
            or arguments.kwarg is not None
            or arguments.defaults
            or any(value is not None for value in arguments.kw_defaults)
        ):
            raise V6ContractError(
                f"qualification tooling helper signature/default mismatch: {name}"
            )


def _qualification_source_identity(
    module3_root: Path, current_head: str, relative_path: str
) -> dict[str, str]:
    source_path = (module3_root / relative_path).resolve()
    tracked = _validator_only_git_output(
        module3_root, ["show", f"{current_head}:{relative_path}"], binary=True
    )
    assert isinstance(tracked, bytes)
    if not source_path.is_file() or source_path.read_bytes() != tracked:
        raise V6ContractError(
            f"qualification tooling source differs from current HEAD: {relative_path}"
        )
    blob = str(_validator_only_git_output(
        module3_root,
        ["rev-parse", f"{current_head}:{relative_path}"],
        binary=False,
    )).strip()
    return {
        "path": str(source_path),
        "sha256": hashlib.sha256(tracked).hexdigest(),
        "git_blob_oid": blob,
        "current_head": current_head,
    }


def _validate_historical_validator_promotion(
    value: Any, *, parent_freeze: Mapping[str, Any]
) -> dict[str, Any]:
    promotion = dict(_mapping(value, "historical_validator_promotion"))
    if promotion.get("schema") != "bio_nav.v6_validator_only_head_promotion.v1":
        raise V6ContractError("historical validator promotion schema mismatch")
    if promotion.get("final_repositories") != parent_freeze["repositories"]:
        raise V6ContractError("historical validator promotion final tuple mismatch")
    runtime = _mapping(promotion.get("pilot_runtime"), "historical pilot_runtime")
    repositories = _mapping(runtime.get("repositories"), "historical repositories")
    if (
        repositories.get("module2") != parent_freeze["repositories"]["module2"]
        or runtime.get("driver_version") != parent_freeze["driver_version"]
        or runtime.get("kernel_release") != parent_freeze["kernel_release"]
    ):
        raise V6ContractError("historical validator promotion Pilot tuple mismatch")
    historical_module2 = _mapping(
        repositories.get("module2"), "historical Module2 repository"
    )
    if set(historical_module2) not in (
        {"path", "head"}, {"path", "head", "tree"},
    ):
        raise V6ContractError("historical Module2 repository identity mismatch")
    module2_root = Path(str(historical_module2["path"])).resolve()
    module2_tree = str(_validator_only_git_output(
        module2_root,
        ["rev-parse", f"{historical_module2['head']}^{{tree}}"],
        binary=False,
    )).strip()
    if "tree" in historical_module2 and module2_tree != historical_module2["tree"]:
        raise V6ContractError("historical Module2 tree mismatch")
    pilot_integration = _mapping(
        repositories.get("integration"), "historical Integration repository"
    )
    final_integration = _mapping(
        parent_freeze["repositories"]["integration"],
        "historical final Integration repository",
    )
    if (
        set(pilot_integration) not in (
            {"path", "head"}, {"path", "head", "tree"},
        )
        or set(final_integration) not in (
            {"path", "head"}, {"path", "head", "tree"},
        )
        or Path(str(pilot_integration["path"])).resolve()
        != Path(str(final_integration["path"])).resolve()
    ):
        raise V6ContractError("historical Integration repository identity mismatch")
    integration_root = Path(str(final_integration["path"])).resolve()
    pilot_integration_tree = str(_validator_only_git_output(
        integration_root,
        ["rev-parse", f"{pilot_integration['head']}^{{tree}}"],
        binary=False,
    )).strip()
    final_integration_tree = str(_validator_only_git_output(
        integration_root,
        ["rev-parse", f"{final_integration['head']}^{{tree}}"],
        binary=False,
    )).strip()
    if (
        "tree" in pilot_integration
        and pilot_integration_tree != pilot_integration["tree"]
    ) or (
        "tree" in final_integration
        and final_integration_tree != final_integration["tree"]
    ):
        raise V6ContractError("historical Integration tree mismatch")
    if pilot_integration["head"] != final_integration["head"]:
        try:
            subprocess.run(
                [
                    "git", "-C", str(integration_root), "merge-base",
                    "--is-ancestor", str(pilot_integration["head"]),
                    str(final_integration["head"]),
                ],
                check=True,
                capture_output=True,
            )
        except (OSError, subprocess.CalledProcessError) as exc:
            raise V6ContractError(
                "historical Integration HEAD is not ancestral"
            ) from exc
        integration_rows, _integration_digest = _validator_only_diff_evidence(
            integration_root,
            str(pilot_integration["head"]),
            str(final_integration["head"]),
        )
        if integration_rows != [{
            "status": "M", "path": "docs/CURRENT_STATE.md",
        }]:
            raise V6ContractError("historical Integration diff is not docs-only")
    module3_root = Path(parent_freeze["repositories"]["module3"]["path"]).resolve()
    pilot_module3 = _mapping(
        repositories.get("module3"), "historical Module3 repository"
    )
    final_module3 = _mapping(
        parent_freeze["repositories"]["module3"],
        "historical final Module3 repository",
    )
    if set(pilot_module3) not in (
        {"path", "head"}, {"path", "head", "tree"},
    ) or set(final_module3) not in (
        {"path", "head"}, {"path", "head", "tree"},
    ) or Path(str(pilot_module3["path"])).resolve() != module3_root:
        raise V6ContractError("historical Module3 repository identity mismatch")
    from_head = str(pilot_module3["head"])
    to_head = str(final_module3["head"])
    from_tree = str(_validator_only_git_output(
        module3_root, ["rev-parse", f"{from_head}^{{tree}}"], binary=False
    )).strip()
    to_tree = str(_validator_only_git_output(
        module3_root, ["rev-parse", f"{to_head}^{{tree}}"], binary=False
    )).strip()
    if (
        "tree" in pilot_module3 and from_tree != pilot_module3["tree"]
    ) or (
        "tree" in final_module3 and to_tree != final_module3["tree"]
    ):
        raise V6ContractError("historical Module3 tree mismatch")
    try:
        subprocess.run(
            ["git", "-C", str(module3_root), "merge-base", "--is-ancestor", from_head, to_head],
            check=True,
            capture_output=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise V6ContractError("historical validator promotion is not ancestral") from exc
    rows, digest = _validator_only_diff_evidence(module3_root, from_head, to_head)
    module3_diff = _mapping(promotion.get("module3_diff"), "historical module3_diff")
    if not (
        module3_diff.get("from_head") == from_head
        and module3_diff.get("to_head") == to_head
        and module3_diff.get("from_is_ancestor") is True
        and module3_diff.get("name_status") == rows
        and module3_diff.get("canonical_diff_sha256") == digest
    ):
        raise V6ContractError("historical validator promotion diff mismatch")
    historical_paths = {row["path"] for row in rows}
    if (
        "ros2_ws/src/robot_experiments/test/test_experiment_motion_quality.py"
        in historical_paths
    ):
        _validator_only_ast_guard(module3_root, from_head, to_head)
    else:
        _evaluator_only_ast_guard(module3_root, from_head, to_head)
    loaded = _mapping(promotion.get("loaded_validator"), "historical loaded_validator")
    relative = "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
    source = _validator_only_git_output(
        module3_root, ["show", f"{to_head}:{relative}"], binary=True
    )
    assert isinstance(source, bytes)
    blob = str(_validator_only_git_output(
        module3_root, ["rev-parse", f"{to_head}:{relative}"], binary=False
    )).strip()
    if not (
        loaded.get("module") == "robot_experiments.experiment_runner"
        and loaded.get("symbol") == "validate_recorded_run_evidence"
        and Path(str(loaded.get("source_path", ""))).resolve()
        == (module3_root / relative).resolve()
        and loaded.get("source_sha256") == hashlib.sha256(source).hexdigest()
        and loaded.get("git_blob_oid") == blob
        and loaded.get("current_head") == to_head
    ):
        raise V6ContractError("historical validator identity mismatch")
    return promotion


def _validate_evaluator_hardening_chain(
    value: Any,
    *,
    freeze: Mapping[str, Any],
    pilot_manifest_sha256: str | None = None,
    pilot_aggregate_sha256: str | None = None,
) -> dict[str, Any]:
    chain = dict(_mapping(value, "evaluator_hardening_promotion"))
    _require_exact_keys(chain, {
        "first_stage_sha256", "aggregate_final_repositories",
        "hardened_repositories", "pilot_manifest_sha256",
        "pilot_aggregate_sha256", "integration_diff", "module3_diff",
        "loaded_validator",
    }, "evaluator_hardening_promotion")
    first_stage = freeze["validator_only_head_promotion"]
    first_stage_sha = hashlib.sha256(
        json.dumps(first_stage, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    if (
        chain.get("first_stage_sha256") != first_stage_sha
        or chain.get("aggregate_final_repositories")
        != first_stage["final_repositories"]
        or chain.get("hardened_repositories") != freeze["repositories"]
        or (
            pilot_manifest_sha256 is not None
            and chain.get("pilot_manifest_sha256") != pilot_manifest_sha256
        )
        or (
            pilot_aggregate_sha256 is not None
            and chain.get("pilot_aggregate_sha256") != pilot_aggregate_sha256
        )
    ):
        raise V6ContractError("evaluator hardening promotion binding mismatch")
    aggregate_repositories = first_stage["final_repositories"]
    historical_freeze = {
        **freeze,
        "repositories": aggregate_repositories,
    }
    _validate_historical_validator_promotion(
        first_stage, parent_freeze=historical_freeze
    )

    def repository_identity(name: str, entry_value: Any) -> tuple[Path, str, str]:
        entry = _mapping(entry_value, f"evaluator_hardening.{name}_repository")
        if set(entry) not in ({"path", "head"}, {"path", "head", "tree"}):
            raise V6ContractError(
                f"evaluator hardening {name} repository identity is invalid"
            )
        repository = Path(str(entry["path"])).expanduser()
        if not repository.is_absolute():
            raise V6ContractError(
                f"evaluator hardening {name} repository path is not absolute"
            )
        repository = repository.resolve()
        head = str(entry["head"])
        tree = str(_validator_only_git_output(
            repository, ["rev-parse", f"{head}^{{tree}}"], binary=False
        )).strip()
        if "tree" in entry and entry["tree"] != tree:
            raise V6ContractError(
                f"evaluator hardening {name} repository tree mismatch"
            )
        return repository, head, tree

    aggregate_identities = {
        name: repository_identity(name, aggregate_repositories[name])
        for name in ("integration", "module2", "module3")
    }
    hardened_identities = {
        name: repository_identity(name, freeze["repositories"][name])
        for name in ("integration", "module2", "module3")
    }
    if aggregate_identities["module2"] != hardened_identities["module2"]:
        raise V6ContractError("evaluator hardening changed Module2")
    for name, path, expected in (
        ("integration", "docs/CURRENT_STATE.md", {"M"}),
        ("module3", None, {"M"}),
    ):
        repository, from_head, _from_tree = aggregate_identities[name]
        hardened_repository, to_head, _to_tree = hardened_identities[name]
        if repository != hardened_repository:
            raise V6ContractError(
                f"evaluator hardening {name} repository path mismatch"
            )
        try:
            subprocess.run(
                [
                    "git", "-C", str(repository), "merge-base", "--is-ancestor",
                    from_head, to_head,
                ],
                check=True,
                capture_output=True,
            )
        except (OSError, subprocess.CalledProcessError) as exc:
            raise V6ContractError(
                f"evaluator hardening {name} HEAD is not ancestral"
            ) from exc
        rows, digest = _validator_only_diff_evidence(repository, from_head, to_head)
        record = _mapping(chain[f"{name}_diff"], f"evaluator_hardening.{name}_diff")
        if record != {"from_head": from_head, "to_head": to_head,
                      "name_status": rows, "canonical_diff_sha256": digest}:
            raise V6ContractError(f"evaluator hardening {name} diff record mismatch")
        if name == "integration":
            if rows != [{"status": "M", "path": path}]:
                raise V6ContractError("evaluator hardening Integration diff mismatch")
        else:
            if {row["path"] for row in rows} != {
                "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
                "ros2_ws/src/robot_experiments/test/test_v6_formal.py",
            } or {row["status"] for row in rows} != expected:
                raise V6ContractError("evaluator hardening Module3 diff mismatch")
            _evaluator_only_ast_guard(repository, from_head, to_head)
    loaded = _validator_only_loaded_identity(
        Path(freeze["repositories"]["module3"]["path"]),
        freeze["repositories"]["module3"]["head"],
    )
    if chain.get("loaded_validator") != loaded:
        raise V6ContractError("evaluator hardening loaded validator mismatch")
    return chain


def _validate_qualification_tooling_promotion(
    value: Any,
    *,
    freeze: Mapping[str, Any],
    parent_freeze: Mapping[str, Any],
) -> dict[str, Any]:
    promotion = dict(_mapping(value, "qualification_tooling_promotion"))
    _require_exact_keys(
        promotion,
        {
            "schema", "from_head", "to_head", "from_is_ancestor",
            "name_status", "canonical_diff_sha256", "v6_formal", "wrapper",
            "unchanged_files", "parent_validator_promotion_sha256",
        },
        "qualification_tooling_promotion",
    )
    if promotion.get("schema") != "bio_nav.v6_indoor_qualification_tooling_promotion.v1":
        raise V6ContractError("qualification tooling promotion schema mismatch")
    if (
        freeze["repositories"]["integration"]
        != parent_freeze["repositories"]["integration"]
        or freeze["repositories"]["module2"]
        != parent_freeze["repositories"]["module2"]
    ):
        raise V6ContractError("qualification tooling promotion changed Integration/Module2")
    module3_root = Path(freeze["repositories"]["module3"]["path"]).resolve()
    from_head = str(parent_freeze["repositories"]["module3"]["head"])
    to_head = str(freeze["repositories"]["module3"]["head"])
    current_head = str(_validator_only_git_output(
        module3_root, ["rev-parse", "HEAD"], binary=False
    )).strip()
    if current_head != to_head or _repository_tracked_dirty(module3_root):
        raise V6ContractError("qualification tooling current checkout drift")
    try:
        subprocess.run(
            ["git", "-C", str(module3_root), "merge-base", "--is-ancestor", from_head, to_head],
            check=True,
            capture_output=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise V6ContractError("qualification tooling promotion is not ancestral") from exc
    rows, digest = _validator_only_diff_evidence(module3_root, from_head, to_head)
    allowed = {
        "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
        "ros2_ws/src/robot_experiments/test/test_v6_formal.py",
        "scripts/run_v6_formal_episode.sh",
    }
    if (
        {row["path"] for row in rows} != allowed
        or any(row["status"] != "M" for row in rows)
        or promotion.get("from_head") != from_head
        or promotion.get("to_head") != to_head
        or promotion.get("from_is_ancestor") is not True
        or promotion.get("name_status") != rows
        or promotion.get("canonical_diff_sha256") != digest
    ):
        raise V6ContractError("qualification tooling promotion diff mismatch")
    _qualification_tooling_ast_guard(module3_root, from_head, to_head)
    identities = {
        "v6_formal": _qualification_source_identity(
            module3_root,
            to_head,
            "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
        ),
        "wrapper": _qualification_source_identity(
            module3_root, to_head, "scripts/run_v6_formal_episode.sh"
        ),
    }
    if promotion.get("v6_formal") != identities["v6_formal"] or promotion.get(
        "wrapper"
    ) != identities["wrapper"]:
        raise V6ContractError("qualification tooling source identity mismatch")
    if freeze.get("v6_formal") != {
        "path": identities["v6_formal"]["path"],
        "sha256": identities["v6_formal"]["sha256"],
    }:
        raise V6ContractError("qualification tooling freeze/v6_formal binding mismatch")
    unchanged_paths = {
        "experiment_runner": "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py",
        "t2_selector": "scripts/run_v6_kujiale_low_obstacles.sh",
        "experiment_launch": "ros2_ws/src/robot_experiments/launch/experiment.launch.py",
    }
    unchanged = {}
    for name, relative in unchanged_paths.items():
        before = _validator_only_git_output(
            module3_root, ["show", f"{from_head}:{relative}"], binary=True
        )
        after = _validator_only_git_output(
            module3_root, ["show", f"{to_head}:{relative}"], binary=True
        )
        assert isinstance(before, bytes) and isinstance(after, bytes)
        if before != after:
            raise V6ContractError(f"qualification tooling changed active file: {name}")
        unchanged[name] = {
            "path": str((module3_root / relative).resolve()),
            "sha256": hashlib.sha256(after).hexdigest(),
        }
    if promotion.get("unchanged_files") != unchanged:
        raise V6ContractError("qualification tooling unchanged-file record mismatch")
    parent_promotion = parent_freeze["validator_only_head_promotion"]
    parent_promotion_sha = hashlib.sha256(
        json.dumps(parent_promotion, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    if promotion.get("parent_validator_promotion_sha256") != parent_promotion_sha:
        raise V6ContractError("qualification tooling parent promotion binding mismatch")
    return promotion


def _build_qualification_tooling_promotion(
    *, freeze: Mapping[str, Any], parent_freeze: Mapping[str, Any]
) -> dict[str, Any]:
    module3_root = Path(freeze["repositories"]["module3"]["path"]).resolve()
    from_head = str(parent_freeze["repositories"]["module3"]["head"])
    to_head = str(freeze["repositories"]["module3"]["head"])
    rows, digest = _validator_only_diff_evidence(module3_root, from_head, to_head)
    unchanged_paths = {
        "experiment_runner": "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py",
        "t2_selector": "scripts/run_v6_kujiale_low_obstacles.sh",
        "experiment_launch": "ros2_ws/src/robot_experiments/launch/experiment.launch.py",
    }
    unchanged = {
        name: {
            "path": str((module3_root / relative).resolve()),
            "sha256": hashlib.sha256(
                module3_root.joinpath(relative).read_bytes()
            ).hexdigest(),
        }
        for name, relative in unchanged_paths.items()
    }
    parent_promotion = parent_freeze["validator_only_head_promotion"]
    candidate = {
        "schema": "bio_nav.v6_indoor_qualification_tooling_promotion.v1",
        "from_head": from_head,
        "to_head": to_head,
        "from_is_ancestor": True,
        "name_status": rows,
        "canonical_diff_sha256": digest,
        "v6_formal": _qualification_source_identity(
            module3_root,
            to_head,
            "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py",
        ),
        "wrapper": _qualification_source_identity(
            module3_root, to_head, "scripts/run_v6_formal_episode.sh"
        ),
        "unchanged_files": unchanged,
        "parent_validator_promotion_sha256": hashlib.sha256(
            json.dumps(parent_promotion, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest(),
    }
    return _validate_qualification_tooling_promotion(
        candidate, freeze=freeze, parent_freeze=parent_freeze
    )


def _indoor_continuation_parent_expectations() -> dict[str, Any]:
    parent_manifest = Path(
        "/mnt/nas_home/Bio_Nav_Data/experiments/pilots/"
        "v6_indoor_refreeze_c0c4d973_20260830T230825Z/indoor_campaign.json"
    )
    parent_output = Path(
        "/mnt/nas_home/Bio_Nav_Data/experiments/formal/"
        "v6_indoor_3x20_c0c4d973_20260830T230825Z"
    )
    run_root = (
        parent_output
        / "indoor_static"
        / "v6_final_kujiale_static"
        / "run-0001-seed-8601"
    )
    return {
        "parent_manifest": str(parent_manifest),
        "parent_manifest_sha256": (
            "144704d951a76777337ab564d20d9ed1b21992f56f9c341fc96c62bf6d75674b"
        ),
        "parent_freeze_digest": (
            "67dc31ebbb5ff2f1fdb9d924834ff0aab02ba86c3988fa936e88a558de28a03e"
        ),
        "parent_output_root": str(parent_output),
        "preserved_run_root": str(run_root),
        "stack_session_id": (
            "36607c86784dca24f36a080412796b89094200680b0f1a39f6a4a0a1e90c78ce"
        ),
        "files": {
            "manifest": "1e8d82ec376e79058d6b8b74fd1ed20389ba587b6fd4123268c21d7cfc4680cf",
            "summary": "21dc2bcd81b1ca0c05e3006b539e8f039c36fd23ae80f5f140fe454d13b62841",
            "trial": "ffdc5d542ba68c21e1507a0288cada9bdd99b285442c80afe8053d7646a60d3b",
            "checksums": "7d85d3705bd8ba65af3f704bf8183f020f378ca8956c3dfac92ab18a17774549",
            "stack_contract": "95a0ab6e0dbcc86ea27675e5a2c17b961d42647877d953ab820f53590b52c734",
        },
    }


def _continuation_lexical_path(
    value: str | Path,
    *,
    field: str,
    must_exist: bool,
    leaf_kind: str,
) -> Path:
    import stat

    path = Path(value).expanduser()
    base = Path(FORMAL_NAS_ROOT).expanduser()
    if (
        not path.is_absolute()
        or not base.is_absolute()
        or ".." in path.parts
        or leaf_kind not in {"file", "directory", "absent"}
        or (must_exist and leaf_kind == "absent")
        or (not must_exist and leaf_kind != "absent")
    ):
        raise V6ContractError(f"{field} lexical path contract is invalid")
    try:
        relative = path.relative_to(base)
    except ValueError as exc:
        raise V6ContractError(f"{field} escapes the trusted NAS base") from exc
    if not relative.parts:
        raise V6ContractError(f"{field} cannot be the trusted NAS base")
    current = base
    components = (base,)
    for part in relative.parts:
        current = current / part
        components = (*components, current)
    for index, component in enumerate(components):
        is_leaf = index == len(components) - 1
        exists = os.path.lexists(component)
        if not exists:
            if is_leaf and not must_exist:
                return path
            raise V6ContractError(f"{field} component does not exist: {component}")
        mode = os.lstat(component).st_mode
        if stat.S_ISLNK(mode):
            raise V6ContractError(f"{field} contains a symlink: {component}")
        if not is_leaf and not stat.S_ISDIR(mode):
            raise V6ContractError(f"{field} parent component is not a directory")
        if is_leaf and must_exist:
            if leaf_kind == "file" and not stat.S_ISREG(mode):
                raise V6ContractError(f"{field} leaf is not a regular file")
            if leaf_kind == "directory" and not stat.S_ISDIR(mode):
                raise V6ContractError(f"{field} leaf is not a directory")
    return path


def _continuation_cli_argv_error(argv: list[str]) -> str | None:
    required = {
        "--continue-indoor-parent",
        "--continuation-output-manifest",
        "--continuation-output-root",
    }
    if not any(
        token == name or token.startswith(f"{name}=")
        for token in argv
        for name in required
    ):
        return None
    if len(argv) != 6 or any(argv.count(name) != 1 for name in required):
        return "continuation mode requires exactly three unique path options"
    for index in range(0, len(argv), 2):
        if argv[index] not in required or argv[index + 1].startswith("--"):
            return "continuation mode accepts only its three path options"
    if {argv[index] for index in range(0, len(argv), 2)} != required:
        return "continuation mode path options are incomplete"
    return None


def _validate_indoor_continuation_contract(
    manifest: IndoorCampaignManifest,
) -> dict[str, Any]:
    record = dict(_mapping(
        manifest.freeze.get("indoor_continuation"), "indoor_continuation"
    ))
    _require_exact_keys(
        record,
        {
            "schema", "reason", "condition_id", "preserved_run_index",
            "continuation_count", "parent_manifest", "parent_output_root",
            "successor_output_root", "preserved_run", "epochs",
        },
        "indoor_continuation",
    )
    if not (
        record.get("schema") == INDOOR_CONTINUATION_SCHEMA_VERSION
        and record.get("reason") == "tooling_invalid_after_valid_episode"
        and record.get("condition_id") == "indoor_static"
        and record.get("preserved_run_index") == 1
        and record.get("continuation_count") == 1
    ):
        raise V6ContractError("indoor continuation identity/reason mismatch")
    _continuation_lexical_path(
        manifest.path,
        field="indoor continuation successor manifest",
        must_exist=True,
        leaf_kind="file",
    )
    expected_parent = _indoor_continuation_parent_expectations()
    parent_manifest_record = _mapping(
        record.get("parent_manifest"), "indoor_continuation.parent_manifest"
    )
    _require_exact_keys(
        parent_manifest_record,
        {"path", "sha256", "freeze_digest"},
        "indoor_continuation.parent_manifest",
    )
    _continuation_lexical_path(
        parent_manifest_record["path"],
        field="indoor continuation parent manifest",
        must_exist=True,
        leaf_kind="file",
    )
    parent_manifest_entry = _validate_frozen_file(
        {
            "path": parent_manifest_record["path"],
            "sha256": parent_manifest_record["sha256"],
        },
        "indoor_continuation.parent_manifest",
    )
    parent_manifest_path = Path(parent_manifest_entry["path"])
    expected_preserved = (
        record.get("preserved_run")
        if isinstance(record.get("preserved_run"), Mapping)
        else {}
    )
    expected_files = (
        expected_preserved.get("files")
        if isinstance(expected_preserved.get("files"), Mapping)
        else {}
    )
    if not (
        str(parent_manifest_path) == expected_parent["parent_manifest"]
        and parent_manifest_entry["sha256"]
        == expected_parent["parent_manifest_sha256"]
        and parent_manifest_record.get("freeze_digest")
        == expected_parent["parent_freeze_digest"]
        and record.get("parent_output_root") == expected_parent["parent_output_root"]
        and expected_preserved.get("root") == expected_parent["preserved_run_root"]
        and expected_preserved.get("stack_session_id")
        == expected_parent["stack_session_id"]
        and {
            name: row.get("sha256")
            for name, row in expected_files.items()
            if isinstance(row, Mapping)
        }
        == expected_parent["files"]
    ):
        raise V6ContractError("indoor continuation is not bound to the approved parent")
    try:
        parent_raw = dict(_mapping(
            json.loads(parent_manifest_path.read_text(encoding="utf-8")),
            "indoor_continuation.parent_manifest.document",
        ))
    except (OSError, json.JSONDecodeError) as exc:
        raise V6ContractError(f"indoor continuation parent manifest unreadable: {exc}") from exc
    if parent_raw.get("schema_version") != INDOOR_CAMPAIGN_SCHEMA_VERSION:
        raise V6ContractError("indoor continuation parent must be an indoor v2 campaign")
    parent_freeze = _mapping(parent_raw.get("freeze"), "indoor continuation parent freeze")
    parent_digest = hashlib.sha256(
        json.dumps(parent_freeze, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    if parent_manifest_record.get("freeze_digest") != parent_digest:
        raise V6ContractError("indoor continuation parent freeze digest mismatch")
    _validate_historical_validator_promotion(
        parent_freeze.get("validator_only_head_promotion"),
        parent_freeze=parent_freeze,
    )
    _validate_qualification_tooling_promotion(
        manifest.freeze.get("qualification_tooling_promotion"),
        freeze=manifest.freeze,
        parent_freeze=parent_freeze,
    )
    invariant_keys = {
        "driver_version", "kernel_release", "scenarios", "scenario_configs",
        "physical_contracts", "frozen_assets", "runner_entrypoint",
        "experiment_runner", "validator_only_head_promotion",
    }
    def invariant_value(freeze_value: Mapping[str, Any], key: str) -> Any:
        value = freeze_value.get(key)
        if key != "scenario_configs" or not isinstance(value, Mapping):
            return value
        return {
            condition_id: sorted(
                rows,
                key=lambda row: str(row.get("path", "")),
            )
            for condition_id, rows in value.items()
        }
    if any(
        invariant_value(manifest.freeze, key) != invariant_value(parent_freeze, key)
        for key in invariant_keys
    ):
        raise V6ContractError("indoor continuation changed frozen product/scenario/assets")
    if (
        manifest.freeze["repositories"]["integration"]
        != parent_freeze["repositories"]["integration"]
        or manifest.freeze["repositories"]["module2"]
        != parent_freeze["repositories"]["module2"]
    ):
        raise V6ContractError("indoor continuation changed Integration/Module2 tuple")
    parent_conditions = parent_raw.get("conditions")
    if not isinstance(parent_conditions, list) or len(parent_conditions) != 3:
        raise V6ContractError("indoor continuation parent condition set invalid")
    parent_by_id = {
        str(_mapping(row, "parent conditions[]").get("id")): row
        for row in parent_conditions
    }
    parent_output = Path(str(record.get("parent_output_root", ""))).expanduser()
    successor_output = Path(str(record.get("successor_output_root", ""))).expanduser()
    _continuation_lexical_path(
        parent_output,
        field="indoor continuation parent output root",
        must_exist=True,
        leaf_kind="directory",
    )
    _continuation_lexical_path(
        successor_output,
        field="indoor continuation successor output root",
        must_exist=True,
        leaf_kind="directory",
    )
    if not parent_output.is_absolute() or not successor_output.is_absolute():
        raise V6ContractError("indoor continuation output roots must be absolute")
    parent_output = parent_output.resolve()
    successor_output = successor_output.resolve()
    if (
        parent_output == successor_output
        or parent_output in successor_output.parents
        or successor_output in parent_output.parents
        or parent_output.is_symlink()
        or successor_output.is_symlink()
    ):
        raise V6ContractError("indoor continuation output roots overlap or use symlinks")
    if Path(str(parent_by_id["indoor_static"].get("output_directory", ""))).resolve() != (
        parent_output / "indoor_static"
    ):
        raise V6ContractError("indoor continuation parent output binding mismatch")
    successor_by_id = {condition.condition_id: condition for condition in manifest.conditions}
    if any(
        successor_by_id[condition_id].output_directory
        != successor_output / condition_id
        for condition_id in INDOOR_CONDITION_IDS
    ):
        raise V6ContractError("indoor continuation successor output binding mismatch")
    preserved = _mapping(record.get("preserved_run"), "indoor_continuation.preserved_run")
    _require_exact_keys(
        preserved,
        {
            "root", "scenario_id", "seed", "files", "stack_session_id",
            "sequence", "reset_generation", "classification",
            "collision_detected", "physical_collision_free",
            "path_deviation_percent",
        },
        "indoor_continuation.preserved_run",
    )
    static = successor_by_id["indoor_static"]
    expected_root = (
        parent_output / "indoor_static" / static.scenario_id / "run-0001-seed-8601"
    ).resolve()
    preserved_lexical = Path(str(preserved.get("root", ""))).expanduser()
    _continuation_lexical_path(
        preserved_lexical,
        field="indoor continuation preserved run root",
        must_exist=True,
        leaf_kind="directory",
    )
    preserved_root = preserved_lexical.resolve()
    if preserved_root != expected_root or preserved_root.is_symlink():
        raise V6ContractError("indoor continuation preserved run root mismatch")
    expected_file_names = {
        "manifest": "run_manifest.json",
        "summary": "run_summary.json",
        "trial": "TRIAL_DISPATCHED.json",
        "checksums": "checksums.sha256",
        "stack_contract": "stack_contract.json",
    }
    files = _mapping(preserved.get("files"), "indoor_continuation.preserved_run.files")
    _require_exact_keys(files, set(expected_file_names), "indoor_continuation.preserved_run.files")
    for name in expected_file_names:
        entry = _mapping(
            files[name], f"indoor_continuation.preserved_run.files.{name}"
        )
        _continuation_lexical_path(
            entry.get("path", ""),
            field=f"indoor continuation preserved {name}",
            must_exist=True,
            leaf_kind="file",
        )
    validated_files = {
        name: _validate_frozen_file(
            files.get(name), f"indoor_continuation.preserved_run.files.{name}",
            expected_path=preserved_root / filename,
        )
        for name, filename in expected_file_names.items()
    }
    try:
        from robot_experiments.experiment_runner import (
            ExperimentRunner,
            RECORDED_TOPIC_TYPES,
            _mcap_inventory_evidence,
            _mcap_required_topic_coverage,
            _route_prior_application_evidence,
        )
    except ImportError as exc:
        raise V6ContractError("indoor continuation validator import failed") from exc
    if not ExperimentRunner._checksums_are_verified(preserved_root):
        raise V6ContractError("indoor continuation preserved full checksums failed")
    try:
        episode = json.loads(Path(validated_files["manifest"]["path"]).read_text())
        summary = json.loads(Path(validated_files["summary"]["path"]).read_text())
        trial = json.loads(Path(validated_files["trial"]["path"]).read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise V6ContractError(f"indoor continuation preserved evidence unreadable: {exc}") from exc
    inventory = _mcap_inventory_evidence(preserved_root)
    coverage = _mcap_required_topic_coverage(
        preserved_root / "telemetry" / "metadata.yaml",
        scene="indoor",
        route_guided=True,
    )
    stored_coverage = summary.get("required_topic_coverage")
    coverage_keys = {
        "required_topics", "message_counts", "forbidden_topics",
        "forbidden_message_counts", "observed_forbidden_topics", "passed",
    }
    coverage_matches = bool(
        isinstance(stored_coverage, Mapping)
        and all(stored_coverage.get(key) == coverage.get(key) for key in coverage_keys)
    )
    route_prior = _route_prior_application_evidence(
        list(episode.get("route_edge_costs", [])), required=True
    )
    inventory_matches = bool(
        inventory.get("passed") is True
        and coverage.get("passed") is True
        and coverage_matches
        and all(
            inventory["topic_counts"].get(topic, 0)
            == coverage["message_counts"].get(topic, 0)
            for topic in coverage["required_topics"]
        )
        and all(
            inventory["topic_types"].get(topic) == RECORDED_TOPIC_TYPES[topic]
            for topic in coverage["required_topics"]
        )
        and inventory["semantic"]["collision_true_count"] > 0
        and summary.get("route_prior_application") == route_prior
        and route_prior.get("confirmed") is True
    )
    stack_contract, _stack_digest = _load_stack_contract_snapshot(
        Path(validated_files["stack_contract"]["path"]),
        expected_condition_id="indoor_static",
        freeze=parent_freeze,
    )
    sequence_receipt = _mapping(
        episode.get("stack_episode_receipt"), "preserved stack_episode_receipt"
    )
    reset_receipt = _mapping(episode.get("reset_receipt"), "preserved reset_receipt")
    summary_metric = summary.get("path_deviation_percent")
    episode_metric = episode.get("metrics", {}).get("path_deviation_percent")
    expected_reference = _expected_scenario_runtime_hashes(static)[2].get(
        "optimal_reference"
    )
    if not (
        preserved.get("scenario_id") == static.scenario_id
        and preserved.get("seed") == 8601
        and preserved.get("stack_session_id") == stack_contract["stack_session_id"]
        and preserved.get("sequence") == 1
        and preserved.get("reset_generation") == 2
        and preserved.get("classification") == "valid_product_failure"
        and preserved.get("collision_detected") is True
        and preserved.get("physical_collision_free") is False
        and preserved.get("path_deviation_percent") is None
        and episode.get("run_index") == 1
        and episode.get("random_seed") == 8601
        and episode.get("condition_stack_id") == "indoor_static"
        and episode.get("stack_session_id") == stack_contract["stack_session_id"]
        and episode.get("formal_freeze_digest") == parent_digest
        and summary.get("formal_freeze_digest") == parent_digest
        and episode.get("provenance", {}).get("git_head")
        == parent_freeze["repositories"]["module3"]["head"]
        and episode.get("optimal_reference_hash") == expected_reference
        and summary_metric is None
        and episode_metric is None
        and sequence_receipt.get("sequence") == 1
        and sequence_receipt.get("baseline") == 1
        and sequence_receipt.get("stack_session_id") == stack_contract["stack_session_id"]
        and reset_receipt.get("generation") == 2
        and trial.get("run_index") == 1
        and trial.get("stack_session_id") == stack_contract["stack_session_id"]
        and trial.get("stack_episode_receipt") == sequence_receipt
        and inventory_matches
        and episode.get("result") == "failure"
        and summary.get("navigation_contract_success") is False
        and summary.get("terminal_zero_confirmed") is True
        and summary.get("data_complete") is True
        and summary.get("checksums_verified") is True
        and summary.get("final_trial_metric_gate", {}).get("passed") is True
        and summary.get("episode_validity", {}).get("valid") is True
        and summary.get("strict_success") is False
        and summary.get("isaac_contact_sensor_collision_detected") is True
        and summary.get("physical_collision_free") is False
    ):
        raise V6ContractError("indoor continuation preserved run classification mismatch")
    epochs = record.get("epochs")
    expected_epochs = [
        {
            "epoch": 1,
            "condition_id": "indoor_static",
            "runs": [1],
            "boundary": "parent_cold",
            "stack_session_id": stack_contract["stack_session_id"],
        },
        {
            "epoch": 2,
            "condition_id": "indoor_static",
            "runs": list(range(2, 21)),
            "boundary": "continuation_cold_restart_then_hot_reset",
            "stack_session_id": None,
        },
    ]
    if epochs != expected_epochs:
        raise V6ContractError("indoor continuation epoch contract mismatch")
    retry_root = (
        successor_output / "indoor_static" / static.scenario_id / "run-0001-seed-8601"
    )
    if retry_root.exists() or retry_root.is_symlink():
        raise V6ContractError("indoor continuation cannot copy or retry preserved run1")
    return record


def create_indoor_continuation_campaign(
    *,
    parent_manifest_path: str | Path,
    output_manifest_path: str | Path,
    successor_output_root: str | Path,
) -> IndoorCampaignManifest:
    parent_path = Path(parent_manifest_path).expanduser()
    output_path = Path(output_manifest_path).expanduser()
    successor_root = Path(successor_output_root).expanduser()
    _continuation_lexical_path(
        parent_path,
        field="indoor continuation parent manifest argument",
        must_exist=True,
        leaf_kind="file",
    )
    _continuation_lexical_path(
        output_path,
        field="indoor continuation successor manifest argument",
        must_exist=False,
        leaf_kind="absent",
    )
    _continuation_lexical_path(
        successor_root,
        field="indoor continuation successor output argument",
        must_exist=False,
        leaf_kind="absent",
    )
    parent_path = parent_path.resolve()
    output_path = output_path.resolve()
    successor_root = successor_root.resolve()
    if (
        not parent_path.is_file()
        or output_path.exists()
        or successor_root.exists()
        or not output_path.parent.is_dir()
        or not successor_root.parent.is_dir()
    ):
        raise V6ContractError("indoor continuation requires new non-symlink outputs")
    for path in (parent_path, output_path.parent, successor_root.parent):
        try:
            path.relative_to(FORMAL_NAS_ROOT.resolve())
        except ValueError as exc:
            raise V6ContractError("indoor continuation paths must be under NAS root") from exc
        _validate_nas_mount(path)
    try:
        parent_raw = dict(_mapping(
            json.loads(parent_path.read_text(encoding="utf-8")),
            "indoor continuation parent manifest",
        ))
    except (OSError, json.JSONDecodeError) as exc:
        raise V6ContractError(f"indoor continuation parent unreadable: {exc}") from exc
    if parent_raw.get("schema_version") != INDOOR_CAMPAIGN_SCHEMA_VERSION:
        raise V6ContractError("indoor continuation parent schema mismatch")
    parent_sha = _file_sha256(parent_path)
    parent_freeze = dict(_mapping(parent_raw.get("freeze"), "parent freeze"))
    parent_digest = hashlib.sha256(
        json.dumps(parent_freeze, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    expected_parent = _indoor_continuation_parent_expectations()
    if not (
        str(parent_path) == expected_parent["parent_manifest"]
        and parent_sha == expected_parent["parent_manifest_sha256"]
        and parent_digest == expected_parent["parent_freeze_digest"]
    ):
        raise V6ContractError("indoor continuation parent is not the approved campaign")
    rows = parent_raw.get("conditions")
    if not isinstance(rows, list) or len(rows) != 3:
        raise V6ContractError("indoor continuation parent conditions invalid")
    by_id = {str(_mapping(row, "parent conditions[]").get("id")): row for row in rows}
    parent_static_lexical = Path(str(by_id["indoor_static"]["output_directory"]))
    _continuation_lexical_path(
        parent_static_lexical,
        field="indoor continuation parent static output",
        must_exist=True,
        leaf_kind="directory",
    )
    parent_static_output = parent_static_lexical.resolve()
    parent_output = parent_static_output.parent
    if str(parent_output) != expected_parent["parent_output_root"]:
        raise V6ContractError("indoor continuation parent output root mismatch")
    if (
        successor_root == parent_output
        or parent_output in successor_root.parents
        or successor_root in parent_output.parents
        or output_path == parent_path
        or parent_output in output_path.parents
        or successor_root in output_path.parents
    ):
        raise V6ContractError("indoor continuation parent/successor paths overlap")
    static_scenario = str(by_id["indoor_static"]["scenario_file"])
    scenario = load_scenario(static_scenario)
    preserved_lexical = (
        parent_static_lexical / scenario.scenario_id / "run-0001-seed-8601"
    )
    _continuation_lexical_path(
        preserved_lexical,
        field="indoor continuation preserved run1",
        must_exist=True,
        leaf_kind="directory",
    )
    preserved_root = preserved_lexical.resolve()
    file_names = {
        "manifest": "run_manifest.json",
        "summary": "run_summary.json",
        "trial": "TRIAL_DISPATCHED.json",
        "checksums": "checksums.sha256",
        "stack_contract": "stack_contract.json",
    }
    for name, filename in file_names.items():
        _continuation_lexical_path(
            preserved_lexical / filename,
            field=f"indoor continuation preserved run1 {name}",
            must_exist=True,
            leaf_kind="file",
        )
    if not (
        str(preserved_root) == expected_parent["preserved_run_root"]
        and {
            name: _file_sha256(preserved_root / filename)
            for name, filename in file_names.items()
        }
        == expected_parent["files"]
    ):
        raise V6ContractError("indoor continuation preserved run1 hash mismatch")
    episode = json.loads((preserved_root / "run_manifest.json").read_text())
    summary = json.loads((preserved_root / "run_summary.json").read_text())
    stack = json.loads((preserved_root / "stack_contract.json").read_text())
    if stack.get("stack_session_id") != expected_parent["stack_session_id"]:
        raise V6ContractError("indoor continuation preserved stack session mismatch")
    sequence = _mapping(episode.get("stack_episode_receipt"), "preserved sequence")
    reset = _mapping(episode.get("reset_receipt"), "preserved reset")
    module3_root = Path(parent_freeze["repositories"]["module3"]["path"]).resolve()
    current_repositories = {
        "integration": _repository_freeze_entry(
            Path(parent_freeze["repositories"]["integration"]["path"])
        ),
        "module2": _repository_freeze_entry(
            Path(parent_freeze["repositories"]["module2"]["path"])
        ),
        "module3": _repository_freeze_entry(module3_root),
    }
    if (
        current_repositories["integration"] != parent_freeze["repositories"]["integration"]
        or current_repositories["module2"] != parent_freeze["repositories"]["module2"]
    ):
        raise V6ContractError("indoor continuation Integration/Module2 drift")
    freeze = json.loads(json.dumps(parent_freeze))
    freeze["repositories"] = current_repositories
    freeze["v6_formal"] = _frozen_file_entry(Path(__file__).resolve())
    freeze["indoor_continuation"] = {
        "schema": INDOOR_CONTINUATION_SCHEMA_VERSION,
        "reason": "tooling_invalid_after_valid_episode",
        "condition_id": "indoor_static",
        "preserved_run_index": 1,
        "continuation_count": 1,
        "parent_manifest": {
            "path": str(parent_path),
            "sha256": parent_sha,
            "freeze_digest": parent_digest,
        },
        "parent_output_root": str(parent_output),
        "successor_output_root": str(successor_root),
        "preserved_run": {
            "root": str(preserved_root),
            "scenario_id": scenario.scenario_id,
            "seed": 8601,
            "files": {
                name: _frozen_file_entry(preserved_root / filename)
                for name, filename in file_names.items()
            },
            "stack_session_id": str(stack.get("stack_session_id", "")),
            "sequence": sequence.get("sequence"),
            "reset_generation": reset.get("generation"),
            "classification": "valid_product_failure",
            "collision_detected": summary.get(
                "isaac_contact_sensor_collision_detected"
            ),
            "physical_collision_free": summary.get("physical_collision_free"),
            "path_deviation_percent": summary.get("path_deviation_percent"),
        },
        "epochs": [
            {
                "epoch": 1,
                "condition_id": "indoor_static",
                "runs": [1],
                "boundary": "parent_cold",
                "stack_session_id": str(stack.get("stack_session_id", "")),
            },
            {
                "epoch": 2,
                "condition_id": "indoor_static",
                "runs": list(range(2, 21)),
                "boundary": "continuation_cold_restart_then_hot_reset",
                "stack_session_id": None,
            },
        ],
    }
    freeze["qualification_tooling_promotion"] = (
        _build_qualification_tooling_promotion(
            freeze=freeze, parent_freeze=parent_freeze
        )
    )
    candidate = json.loads(json.dumps(parent_raw))
    candidate["schema_version"] = INDOOR_CONTINUATION_SCHEMA_VERSION
    candidate["freeze"] = freeze
    candidate["conditions"] = [
        {**row, "output_directory": str(successor_root / str(row["id"]))}
        for row in rows
    ]
    successor_root.mkdir(exist_ok=False)
    temporary_fd, temporary_name = tempfile.mkstemp(
        prefix=f".{output_path.name}.", suffix=".tmp", dir=output_path.parent
    )
    os.close(temporary_fd)
    temporary = Path(temporary_name)
    published = False
    try:
        temporary.write_text(
            json.dumps(candidate, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        campaign = load_indoor_campaign_manifest(temporary)
        if _file_sha256(parent_path) != parent_sha:
            raise V6ContractError("indoor continuation parent manifest changed")
        try:
            os.link(temporary, output_path)
        except FileExistsError as exc:
            raise V6ContractError("indoor continuation manifest already exists") from exc
        published = True
        return replace(campaign, path=output_path)
    finally:
        temporary.unlink(missing_ok=True)
        if not published:
            try:
                successor_root.rmdir()
            except OSError:
                pass


def load_indoor_campaign_manifest(
    path: str | Path, *, _require_pilot_provenance: bool = True
) -> IndoorCampaignManifest:
    import stat

    lexical_path = Path(path).expanduser()
    if ".." in lexical_path.parts:
        raise V6ContractError("indoor manifest lexical path cannot contain '..'")
    if not lexical_path.is_absolute():
        lexical_path = Path.cwd() / lexical_path
    current = Path(lexical_path.anchor)
    for part in lexical_path.parts[1:]:
        current = current / part
        if not os.path.lexists(current):
            raise V6ContractError("indoor manifest lexical path does not exist")
        mode = os.lstat(current).st_mode
        if stat.S_ISLNK(mode):
            raise V6ContractError("indoor manifest lexical path contains a symlink")
        if current != lexical_path and not stat.S_ISDIR(mode):
            raise V6ContractError("indoor manifest lexical parent is not a directory")
    if not stat.S_ISREG(os.lstat(lexical_path).st_mode):
        raise V6ContractError("indoor manifest lexical leaf is not a regular file")
    manifest_path = lexical_path.resolve()
    raw = _mapping(yaml.safe_load(manifest_path.read_text(encoding="utf-8")), "indoor_manifest")
    required_keys = {
            "schema_version",
            "intended_use",
            "runs_per_condition",
            "runner_entrypoint",
            "freeze",
            "conditions",
    }
    if _require_pilot_provenance:
        required_keys.add("indoor_pilot_freeze_provenance")
    _require_exact_keys(raw, required_keys, "indoor_manifest")
    continuation = raw.get("schema_version") == INDOOR_CONTINUATION_SCHEMA_VERSION
    if raw.get("schema_version") not in {
        INDOOR_CAMPAIGN_SCHEMA_VERSION,
        INDOOR_CONTINUATION_SCHEMA_VERSION,
    }:
        raise V6ContractError(
            "indoor schema_version is unsupported"
        )
    if raw.get("intended_use") != "indoor_qualification":
        raise V6ContractError("indoor intended_use must be indoor_qualification")
    if raw.get("runs_per_condition") != FORMAL_RUNS_PER_CONDITION:
        raise V6ContractError("indoor runs_per_condition must be exactly 20")
    runner_entrypoint = Path(str(raw.get("runner_entrypoint", ""))).expanduser()
    if not runner_entrypoint.is_absolute():
        runner_entrypoint = manifest_path.parent / runner_entrypoint
    runner_entrypoint = runner_entrypoint.resolve()
    if not runner_entrypoint.is_file() or not os.access(
        runner_entrypoint, os.R_OK | os.X_OK
    ):
        raise V6ContractError("runner_entrypoint must be a readable executable file")
    condition_rows = raw.get("conditions")
    if not isinstance(condition_rows, list) or len(condition_rows) != 3:
        raise V6ContractError("indoor conditions must contain exactly three rows")
    conditions: list[FormalCondition] = []
    scenario_configs: dict[str, set[Path]] = {}
    output_directories: set[Path] = set()
    for index, value in enumerate(condition_rows):
        row = _mapping(value, f"conditions[{index}]")
        _require_exact_keys(
            row,
            {
                "id",
                "scene",
                "category",
                "scenario_file",
                "output_directory",
                "runner_arguments",
            },
            f"conditions[{index}]",
        )
        condition_id = str(row.get("id", ""))
        scene = str(row.get("scene", ""))
        category = str(row.get("category", ""))
        if condition_id != f"{scene}_{category}" or scene != "indoor":
            raise V6ContractError("indoor condition identity is invalid")
        scenario_file = Path(str(row.get("scenario_file", ""))).expanduser()
        if not scenario_file.is_absolute():
            scenario_file = manifest_path.parent / scenario_file
        scenario_file = scenario_file.resolve()
        if not scenario_file.is_file():
            raise V6ContractError(f"indoor scenario is missing: {scenario_file}")
        scenario = load_scenario(scenario_file)
        expected_scenario_type = "static" if category == "appearance" else category
        if scenario.scenario_type != expected_scenario_type:
            raise V6ContractError("indoor category differs from scenario type")
        if category == "appearance" and scenario.appearance_config_file is None:
            raise V6ContractError("indoor appearance scenario has no appearance config")
        if condition_id == "indoor_static" and scenario.optimal_reference_file is None:
            raise V6ContractError(
                "indoor static scenario must bind the current optimal reference"
            )
        if "rivermark" in f"{scenario.scenario_id} {scenario.map_version}".lower():
            raise V6ContractError("indoor campaign cannot reference outdoor scenario")
        selections = scenario.run_matrix or tuple(
            {
                "seed": seed,
                "condition_id": None,
                "dynamic_case_id": None,
                "dynamic_variant_id": None,
                "appearance_profile_id": None,
            }
            for seed in scenario.seeds
        )
        episode_identities = [
            dict(selection)
            if isinstance(selection, Mapping)
            else {
                "seed": selection.seed,
                "condition_id": selection.condition_id,
                "dynamic_case_id": selection.case_id,
                "dynamic_variant_id": selection.variant_id,
                "appearance_profile_id": selection.appearance_profile_id,
            }
            for selection in selections
        ]
        identity_keys = {
            (
                identity.get("seed"),
                identity.get("condition_id"),
                identity.get("dynamic_case_id"),
                identity.get("dynamic_variant_id"),
                identity.get("appearance_profile_id"),
            )
            for identity in episode_identities
        }
        if len(episode_identities) != 20 or len(identity_keys) != 20:
            raise V6ContractError("indoor scenario must contain 20 unique run identities")
        output_directory = Path(str(row.get("output_directory", ""))).expanduser()
        if not output_directory.is_absolute():
            raise V6ContractError("indoor output_directory must be absolute")
        output_directory = output_directory.resolve()
        if output_directory in output_directories:
            raise V6ContractError("indoor output directories must be unique")
        output_directories.add(output_directory)
        arguments = row.get("runner_arguments")
        if not isinstance(arguments, list) or not all(
            isinstance(argument, str) and ":=" in argument for argument in arguments
        ):
            raise V6ContractError("indoor runner_arguments must be ROS name:=value strings")
        reserved = {
            "scenario_file",
            "output_directory",
            "run_indices",
            "resume",
            "record_evidence",
            "record_bag",
            "fail_stop",
            "condition_stack_id",
            "stack_session_id",
            "condition_stack_contract_path",
            "formal_freeze_digest",
        }
        pairs = [argument.split(":=", 1) for argument in arguments]
        if any(name in reserved for name, _value in pairs):
            raise V6ContractError("indoor runner_arguments overrides dispatcher ownership")
        if len({name for name, _value in pairs}) != len(pairs):
            raise V6ContractError("indoor runner_arguments contains duplicate names")
        argument_map = dict(pairs)
        expected_stable_arguments = {
            "nav2_profile": "v6_low_obstacle_isolation",
            "nav2_config_file": str(_canonical_nav2_config()),
            "spawn_poses_file": str(_canonical_indoor_spawn_manifest()),
            "navigation_execution_backend": "route_guided",
            "require_module2_planning_ready": "true",
            "module2_planning_ready_timeout_sec": "120.0",
            "clear_slam_localization_buffer": "false",
            "reset_map_base_translation_tolerance_m": "0.1",
        }
        if argument_map != expected_stable_arguments:
            raise V6ContractError(
                "indoor runner_arguments must match the exact audited stable T3 contract"
            )
        _effective_indoor_spawn_manifest(arguments)
        scenario_configs[condition_id] = _scenario_runtime_config_paths(
            scenario, arguments
        ) | {
            (
                Path(__file__).resolve().parents[1]
                / "launch"
                / "experiment.launch.py"
            ).resolve()
        }
        conditions.append(
            FormalCondition(
                condition_id=condition_id,
                scene=scene,
                category=category,
                scenario_file=scenario_file,
                output_directory=output_directory,
                runner_arguments=tuple(arguments),
                scenario_id=scenario.scenario_id,
                episode_identities=tuple(episode_identities),
            )
        )
    if tuple(condition.condition_id for condition in conditions) != INDOOR_CONDITION_IDS:
        raise V6ContractError(
            "indoor conditions must use static, dynamic, appearance order"
        )
    condition_tuple = tuple(conditions)
    freeze, freeze_digest = _validate_formal_freeze(
        raw.get("freeze"),
        conditions=condition_tuple,
        scenario_configs=scenario_configs,
        runner_entrypoint=runner_entrypoint,
        condition_ids=INDOOR_CONDITION_IDS,
        frozen_asset_keys=INDOOR_FROZEN_ASSET_KEYS,
        expected_physical_contracts=_indoor_physical_contract_expectations(
            condition_tuple
        ),
    )
    _validate_indoor_static_reference_contract(condition_tuple[0], freeze)
    promotion_value = freeze.get("validator_only_head_promotion")
    hardening_value = freeze.get("evaluator_hardening_promotion")
    promotion = (
        promotion_value
        if continuation or hardening_value is not None
        else (
            _validate_validator_only_head_promotion(
                promotion_value, freeze=freeze
            )
            if promotion_value is not None
            else None
        )
    )
    if not _require_pilot_provenance:
        campaign = IndoorCampaignManifest(
            path=manifest_path,
            runner_entrypoint=runner_entrypoint,
            freeze=freeze,
            freeze_digest=freeze_digest,
            pilot_freeze_provenance={},
            conditions=condition_tuple,
        )
        if continuation:
            _validate_indoor_continuation_contract(campaign)
        return campaign
    provenance = _mapping(
        raw.get("indoor_pilot_freeze_provenance"), "indoor_pilot_freeze_provenance"
    )
    _require_exact_keys(
        provenance,
        {"schema", "pilot_manifest", "pilot_aggregate", "episodes"},
        "indoor_pilot_freeze_provenance",
    )
    if provenance.get("schema") != "bio_nav.v6_indoor_pilot_freeze_provenance.v1":
        raise V6ContractError("indoor Pilot freeze provenance schema mismatch")
    episodes = provenance.get("episodes")
    if not isinstance(episodes, list) or len(episodes) != 9:
        raise V6ContractError("indoor Pilot freeze provenance must index 9 episodes")
    if hardening_value is not None:
        _validate_evaluator_hardening_chain(
            hardening_value,
            freeze=freeze,
            pilot_manifest_sha256=_mapping(
                provenance.get("pilot_manifest"),
                "indoor_pilot_freeze_provenance.pilot_manifest",
            ).get("sha256"),
            pilot_aggregate_sha256=_mapping(
                provenance.get("pilot_aggregate"),
                "indoor_pilot_freeze_provenance.pilot_aggregate",
            ).get("sha256"),
        )
    _revalidate_indoor_pilot_freeze_provenance(
        provenance,
        conditions=condition_tuple,
        freeze=_pilot_freeze_from_validator_promotion(freeze, promotion),
        freeze_digest=freeze_digest,
    )
    campaign = IndoorCampaignManifest(
        path=manifest_path,
        runner_entrypoint=runner_entrypoint,
        freeze=freeze,
        freeze_digest=freeze_digest,
        pilot_freeze_provenance=provenance,
        conditions=condition_tuple,
    )
    if continuation:
        _validate_indoor_continuation_contract(campaign)
    return campaign


def _pilot_selection_identity(condition: FormalCondition, rep: int) -> Mapping[str, Any]:
    return condition.episode_identities[rep - 1]


def _viewport_winner_manifest(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V6ContractError(f"viewport winner attestation is unreadable: {exc}") from exc
    payload = _mapping(payload, "viewport_winner_manifest")
    winner = payload.get("winner", payload)
    winner = _mapping(winner, "viewport_winner_manifest.winner")
    arm = winner.get("viewport_arm", winner.get("selected_arm"))
    if arm != "B":
        raise V6ContractError("viewport startup A/B manifest must select arm B")
    return {
        "path": str(path.resolve()),
        "sha256": _file_sha256(path),
        "viewport_arm": "B",
    }


def load_outdoor_campaign_manifest(
    path: str | Path, *, _require_pilot_provenance: bool = True
) -> OutdoorCampaignManifest:
    import stat

    lexical = Path(path).expanduser()
    if ".." in lexical.parts:
        raise V6ContractError("outdoor manifest lexical path cannot contain '..'")
    if not lexical.is_absolute():
        lexical = Path.cwd() / lexical
    current = Path(lexical.anchor)
    for part in lexical.parts[1:]:
        current /= part
        if not os.path.lexists(current) or stat.S_ISLNK(os.lstat(current).st_mode):
            raise V6ContractError("outdoor manifest path is missing or contains symlink")
    if not stat.S_ISREG(os.lstat(lexical).st_mode):
        raise V6ContractError("outdoor manifest leaf is not a regular file")
    manifest_path = lexical.resolve()
    raw = _mapping(yaml.safe_load(manifest_path.read_text(encoding="utf-8")), "outdoor_manifest")
    required = {
        "schema_version", "intended_use", "runs_per_condition",
        "runner_entrypoint", "freeze", "conditions",
    }
    if _require_pilot_provenance:
        required.add("outdoor_pilot_freeze_provenance")
    _require_exact_keys(raw, required, "outdoor_manifest")
    if raw.get("schema_version") != OUTDOOR_CAMPAIGN_SCHEMA_VERSION:
        raise V6ContractError("outdoor schema_version is unsupported")
    if raw.get("intended_use") != "outdoor_qualification":
        raise V6ContractError("outdoor intended_use must be outdoor_qualification")
    if raw.get("runs_per_condition") != FORMAL_RUNS_PER_CONDITION:
        raise V6ContractError("outdoor runs_per_condition must be exactly 20")
    runner = Path(str(raw.get("runner_entrypoint", ""))).expanduser()
    if not runner.is_absolute():
        runner = manifest_path.parent / runner
    runner = runner.resolve()
    if not runner.is_file() or not os.access(runner, os.R_OK | os.X_OK):
        raise V6ContractError("outdoor runner_entrypoint is not executable")
    rows = raw.get("conditions")
    if not isinstance(rows, list) or len(rows) != 3:
        raise V6ContractError("outdoor conditions must contain three rows")
    conditions: list[FormalCondition] = []
    scenario_configs: dict[str, set[Path]] = {}
    outputs: set[Path] = set()
    expected_arguments = {
        "nav2_profile": "v6_low_obstacle_isolation",
        "nav2_config_file": str(_canonical_nav2_config()),
        "navigation_execution_backend": "route_guided",
        "require_module2_planning_ready": "true",
    }
    for index, value in enumerate(rows):
        row = _mapping(value, f"conditions[{index}]")
        _require_exact_keys(
            row,
            {"id", "scene", "category", "scenario_file", "output_directory", "runner_arguments"},
            f"conditions[{index}]",
        )
        condition_id = str(row.get("id", ""))
        scene = str(row.get("scene", ""))
        category = str(row.get("category", ""))
        if condition_id != f"{scene}_{category}" or scene != "outdoor":
            raise V6ContractError("outdoor condition identity is invalid")
        scenario_path = Path(str(row.get("scenario_file", ""))).expanduser()
        if not scenario_path.is_absolute():
            scenario_path = manifest_path.parent / scenario_path
        scenario_path = scenario_path.resolve()
        scenario = load_scenario(scenario_path)
        expected_type = "static" if category == "appearance" else category
        if scenario.scenario_type != expected_type or "rivermark" not in (
            f"{scenario.scenario_id} {scenario.map_version}".lower()
        ):
            raise V6ContractError("outdoor scenario identity/type mismatch")
        if category == "appearance" and scenario.appearance_config_file is None:
            raise V6ContractError("outdoor appearance scenario has no appearance config")
        if category == "static" and scenario.optimal_reference_file is None:
            raise V6ContractError("outdoor static scenario must bind optimal reference")
        selections = scenario.run_matrix or tuple(
            {
                "seed": seed, "condition_id": None, "dynamic_case_id": None,
                "dynamic_variant_id": None, "appearance_profile_id": None,
            }
            for seed in scenario.seeds
        )
        identities = [
            dict(item) if isinstance(item, Mapping) else {
                "seed": item.seed,
                "condition_id": item.condition_id,
                "dynamic_case_id": item.case_id,
                "dynamic_variant_id": item.variant_id,
                "appearance_profile_id": item.appearance_profile_id,
            }
            for item in selections
        ]
        identity_keys = {
            tuple(identity.get(name) for name in (
                "seed", "condition_id", "dynamic_case_id",
                "dynamic_variant_id", "appearance_profile_id",
            ))
            for identity in identities
        }
        if len(identities) != 20 or len(identity_keys) != 20:
            raise V6ContractError("outdoor scenario must contain 20 unique identities")
        output = Path(str(row.get("output_directory", ""))).expanduser()
        if not output.is_absolute() or output.resolve() in outputs:
            raise V6ContractError("outdoor output directories must be absolute and unique")
        output = output.resolve()
        outputs.add(output)
        arguments = row.get("runner_arguments")
        if not isinstance(arguments, list) or not all(
            isinstance(item, str) and ":=" in item for item in arguments
        ):
            raise V6ContractError("outdoor runner_arguments are invalid")
        pairs = [item.split(":=", 1) for item in arguments]
        if len({name for name, _ in pairs}) != len(pairs) or dict(pairs) != expected_arguments:
            raise V6ContractError("outdoor runner_arguments differ from audited contract")
        scenario_configs[condition_id] = _scenario_runtime_config_paths(
            scenario, arguments
        )
        conditions.append(FormalCondition(
            condition_id=condition_id,
            scene=scene,
            category=category,
            scenario_file=scenario_path,
            output_directory=output,
            runner_arguments=tuple(arguments),
            scenario_id=scenario.scenario_id,
            episode_identities=tuple(identities),
        ))
    if tuple(row.condition_id for row in conditions) != OUTDOOR_CONDITION_IDS:
        raise V6ContractError("outdoor conditions must use static, dynamic, appearance order")
    condition_tuple = tuple(conditions)
    freeze, freeze_digest = _validate_formal_freeze(
        raw.get("freeze"),
        conditions=condition_tuple,
        scenario_configs=scenario_configs,
        runner_entrypoint=runner,
        condition_ids=OUTDOOR_CONDITION_IDS,
        frozen_asset_keys=OUTDOOR_FROZEN_ASSET_KEYS,
    )
    if any("tree" not in freeze["repositories"][name] for name in freeze["repositories"]):
        raise V6ContractError("outdoor qualification requires repository HEAD+tree identity")
    winner = freeze["frozen_assets"]["rivermark_viewport_startup_winner_manifest"]
    parsed_winner = _viewport_winner_manifest(Path(winner["path"]))
    if parsed_winner["sha256"] != winner["sha256"]:
        raise V6ContractError("outdoor viewport winner digest mismatch")
    if not _require_pilot_provenance:
        return OutdoorCampaignManifest(
            manifest_path, runner, freeze, freeze_digest, {}, condition_tuple
        )
    provenance = _mapping(
        raw.get("outdoor_pilot_freeze_provenance"),
        "outdoor_pilot_freeze_provenance",
    )
    _require_exact_keys(
        provenance, {"schema", "pilot_manifest", "pilot_aggregate", "episodes"},
        "outdoor_pilot_freeze_provenance",
    )
    if provenance.get("schema") != "bio_nav.v6_outdoor_pilot_freeze_provenance.v1":
        raise V6ContractError("outdoor Pilot freeze provenance schema mismatch")
    if not isinstance(provenance.get("episodes"), list) or len(provenance["episodes"]) != 9:
        raise V6ContractError("outdoor Pilot freeze provenance must index 9 episodes")
    _revalidate_indoor_pilot_freeze_provenance(
        provenance,
        conditions=condition_tuple,
        freeze=freeze,
        freeze_digest=freeze_digest,
    )
    return OutdoorCampaignManifest(
        manifest_path, runner, freeze, freeze_digest, provenance, condition_tuple
    )


def _validate_sufficient_pilot_episode(
    *,
    condition: FormalCondition,
    rep: int,
    summary_path: Path,
    manifest_path: Path,
    stack_contract_path: Path,
    expected_stack_tuple_digest: str,
    freeze: Mapping[str, Any],
    freeze_digest: str,
) -> tuple[str, int, dict[str, Any]]:
    if summary_path.name != "run_summary.json" or manifest_path.name != "run_manifest.json":
        raise V6ContractError("Pilot episode must reference canonical summary/manifest names")
    if summary_path.parent != manifest_path.parent:
        raise V6ContractError("Pilot episode summary/manifest roots differ")
    stack_contract, stack_tuple_digest = _load_stack_contract_snapshot(
        stack_contract_path,
        expected_condition_id=condition.condition_id,
        freeze=freeze,
    )
    if stack_tuple_digest != expected_stack_tuple_digest:
        raise V6ContractError("Pilot stack normalized tuple digest mismatch")
    try:
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
        episode = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V6ContractError(f"Pilot episode evidence is unreadable: {exc}") from exc
    try:
        from robot_experiments.experiment_runner import validate_recorded_run_evidence

        validate_recorded_run_evidence(
            summary_path.parent,
            summary,
            episode,
            scene=condition.scene,
            route_guided=True,
            route_prior_required=True,
            expected_leg_count=5,
            cognitive_admission_required=True,
        )
    except (ConfigurationError, ImportError) as exc:
        raise V6ContractError(f"Pilot episode primary evidence failed: {exc}") from exc
    identity = _pilot_selection_identity(condition, rep)
    if not (
        episode.get("scenario_id") == condition.scenario_id
        and episode.get("run_index") == rep
        and episode.get("random_seed") == identity["seed"]
        and episode.get("condition_id") == identity.get("condition_id")
        and episode.get("dynamic_selection", {}).get("case_id")
        == identity.get("dynamic_case_id")
        and episode.get("dynamic_selection", {}).get("variant_id")
        == identity.get("dynamic_variant_id")
        and episode.get("appearance", {}).get("profile_id")
        == identity.get("appearance_profile_id")
    ):
        raise V6ContractError("Pilot episode run identity mismatch")
    if not (
        episode.get("condition_stack_id") == condition.condition_id
        and summary.get("condition_stack_id") == condition.condition_id
        and isinstance(episode.get("stack_session_id"), str)
        and episode.get("stack_session_id")
        and summary.get("stack_session_id") == episode.get("stack_session_id")
        and stack_contract["stack_session_id"] == episode.get("stack_session_id")
        and episode.get("formal_freeze_digest")
        == summary.get("formal_freeze_digest")
        and episode.get("formal_freeze_digest") in {None, "", freeze_digest}
    ):
        raise V6ContractError("Pilot episode frozen tuple/session mismatch")
    sequence_receipt = episode.get("stack_episode_receipt", {})
    summary_sequence_receipt = summary.get("condition_stack_attestation", {}).get(
        "stack_episode_receipt"
    )
    strict_group_receipt_ok = bool(
        stack_contract.get("schema") != STACK_CONTRACT_SCHEMA
        or (
            sequence_receipt.get("startup_kind")
            == ("cold" if rep == 1 else "hot_reset")
            and sequence_receipt.get("condition_stack_contract_path")
            == stack_contract.get("contract_path")
            and sequence_receipt.get("condition_stack_contract_sha256")
            == _file_sha256(stack_contract_path)
            and summary.get("condition_stack_attestation", {}).get(
                "condition_stack_contract_path"
            ) == stack_contract.get("contract_path")
            and summary.get("condition_stack_attestation", {}).get(
                "condition_stack_contract_sha256"
            ) == _file_sha256(stack_contract_path)
        )
    )
    if not (
        isinstance(sequence_receipt, Mapping)
        and summary_sequence_receipt == sequence_receipt
        and sequence_receipt.get("schema")
        == "bio_nav.v6_stack_episode_receipt.v1"
        and sequence_receipt.get("sequence") == rep
        and sequence_receipt.get("baseline") == 1
        and sequence_receipt.get("stack_session_id")
        == stack_contract["stack_session_id"]
        and sequence_receipt.get("t2_selector_path")
        == stack_contract["t2_selector_path"]
        and sequence_receipt.get("t2_selector_sha256")
        == stack_contract["t2_selector_sha256"]
        and strict_group_receipt_ok
        and all(
            sequence_receipt.get(name) == stack_contract.get(name)
            for name in (
                "viewport_arm",
                "disable_viewport_updates_requested",
                "disable_viewport_updates_observed",
                "viewport_startup_attestation_sha256",
                "viewport_runtime_attestation_sha256",
                "viewport_instance_uuid",
                "viewport_pid",
                "viewport_pgid",
                "viewport_start_ticks",
                "viewport_winner_manifest_sha256",
            )
        )
    ):
        raise V6ContractError("Pilot stack episode sequence/T2 receipt mismatch")
    reset_receipt = episode.get("reset_receipt", {})
    generation = reset_receipt.get("generation") if isinstance(reset_receipt, Mapping) else None
    if isinstance(generation, bool) or not isinstance(generation, int) or generation <= 0:
        raise V6ContractError("Pilot episode reset generation is invalid")
    if summary.get("reset_receipt") != reset_receipt:
        raise V6ContractError("Pilot episode/summary reset receipt mismatch")
    expected_robot_hash, expected_nav2_hash, expected_runtime_hashes = (
        _expected_scenario_runtime_hashes(condition)
    )
    static_reference_ok = True
    if condition.category == "static":
        summary_metric = summary.get("path_deviation_percent")
        episode_metric = episode.get("metrics", {}).get("path_deviation_percent")
        static_reference_ok = bool(
            expected_runtime_hashes.get("optimal_reference") is not None
            and episode.get("optimal_reference_hash")
            == expected_runtime_hashes["optimal_reference"]
            and isinstance(summary_metric, (int, float))
            and not isinstance(summary_metric, bool)
            and math.isfinite(float(summary_metric))
            and float(summary_metric) < 20.0
            and isinstance(episode_metric, (int, float))
            and not isinstance(episode_metric, bool)
            and float(episode_metric) == float(summary_metric)
        )
    provenance = episode.get("provenance", {})
    if not (
        episode.get("robot_config_hash") == expected_robot_hash
        and episode.get("nav2_config_hash") == expected_nav2_hash
        and episode.get("scenario_runtime_hashes") == expected_runtime_hashes
        and static_reference_ok
        and isinstance(provenance, Mapping)
        and provenance.get("git_head") == freeze["repositories"]["module3"]["head"]
        and provenance.get("git_tracked_dirty") is False
    ):
        raise V6ContractError("Pilot episode source/config provenance mismatch")
    map_hashes = provenance.get("map_and_posegraph_hashes", {})
    map_keys = (
        ("outdoor_map_yaml", "outdoor_map_pgm")
        if condition.scene == "outdoor"
        else ("indoor_map_yaml", "indoor_map_pgm")
    )
    if not isinstance(map_hashes, Mapping) or not {
        freeze["frozen_assets"][name]["sha256"] for name in map_keys
    } <= set(map_hashes.values()):
        raise V6ContractError("Pilot episode map provenance mismatch")
    checksum_path = summary_path.parent / "checksums.sha256"
    evidence_index = {
        "condition_id": condition.condition_id,
        "rep": rep,
        "summary": {"path": str(summary_path), "sha256": _file_sha256(summary_path)},
        "manifest": {"path": str(manifest_path), "sha256": _file_sha256(manifest_path)},
        "checksums": {"path": str(checksum_path), "sha256": _file_sha256(checksum_path)},
        "stack_contract": {
            "path": str(stack_contract_path),
            "sha256": _file_sha256(stack_contract_path),
        },
        "stack_tuple_digest": stack_tuple_digest,
    }
    return str(episode["stack_session_id"]), generation, evidence_index


def freeze_formal_manifest_from_pilot(
    *,
    pilot_manifest_path: str | Path,
    pilot_aggregate_path: str | Path,
    output_manifest_path: str | Path,
    formal_output_root: str | Path,
) -> FormalCampaignManifest:
    pilot_input = Path(pilot_manifest_path).expanduser()
    aggregate_input = Path(pilot_aggregate_path).expanduser()
    output_input = Path(output_manifest_path).expanduser()
    if not pilot_input.is_absolute() or not aggregate_input.is_absolute() or not output_input.is_absolute():
        raise V6ContractError("Pilot freezer paths must be absolute")
    pilot_path = pilot_input.resolve()
    aggregate_path = aggregate_input.resolve()
    output_path = output_input.resolve()
    output_root = Path(formal_output_root).expanduser()
    if not output_root.is_absolute():
        raise V6ContractError("formal output root must be absolute")
    output_root = output_root.resolve()
    try:
        output_root.relative_to(FORMAL_NAS_ROOT.resolve())
    except ValueError as exc:
        raise V6ContractError("formal output root must be under the NAS root") from exc
    _validate_nas_mount(output_root)
    if output_root.exists():
        raise V6ContractError("formal output root must be new")
    if output_path.exists() or not output_path.parent.is_dir():
        raise V6ContractError("output manifest must be a new file in an existing directory")
    try:
        pilot = yaml.safe_load(pilot_path.read_text(encoding="utf-8"))
        aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, yaml.YAMLError) as exc:
        raise V6ContractError(f"Pilot freezer input is unreadable: {exc}") from exc
    pilot = _mapping(pilot, "pilot_manifest")
    _require_exact_keys(
        pilot,
        {"schema_version", "intended_use", "runner_entrypoint", "freeze", "conditions"},
        "pilot_manifest",
    )
    if (
        pilot.get("schema_version") != SUFFICIENT_PILOT_MANIFEST_SCHEMA
        or pilot.get("intended_use") != "sufficient_pilot"
    ):
        raise V6ContractError("unsupported sufficient Pilot manifest")
    condition_rows = pilot.get("conditions")
    if not isinstance(condition_rows, list) or len(condition_rows) != 6:
        raise V6ContractError("Pilot manifest must contain six conditions")
    formal_conditions = []
    for row in condition_rows:
        row = dict(_mapping(row, "pilot_manifest.conditions[]"))
        _require_exact_keys(
            row,
            {"id", "scene", "category", "scenario_file", "runner_arguments"},
            "pilot_manifest.conditions[]",
        )
        if not Path(str(row.get("scenario_file", ""))).expanduser().is_absolute():
            raise V6ContractError("Pilot scenario paths must be absolute")
        formal_conditions.append(
            {**row, "output_directory": str(output_root / str(row["id"]))}
        )
    candidate = {
        "schema_version": FORMAL_CAMPAIGN_SCHEMA_VERSION,
        "intended_use": "formal_qualification",
        "execution_authorization": FORMAL_EXECUTION_NOT_AUTHORIZED,
        "runs_per_condition": FORMAL_RUNS_PER_CONDITION,
        "runner_entrypoint": pilot["runner_entrypoint"],
        "freeze": pilot["freeze"],
        "conditions": formal_conditions,
    }
    if not output_root.parent.is_dir():
        raise V6ContractError("formal output root parent must exist")
    try:
        output_root.mkdir(exist_ok=False)
    except FileExistsError as exc:
        raise V6ContractError("formal output root must be new") from exc
    try:
        temporary_fd, temporary_name = tempfile.mkstemp(
            prefix=f".{output_path.name}.", suffix=".tmp", dir=output_path.parent
        )
    except OSError:
        output_root.rmdir()
        raise
    os.close(temporary_fd)
    temporary = Path(temporary_name)
    published = False
    try:
        temporary.write_text(json.dumps(candidate, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        frozen = load_formal_campaign_manifest(temporary)
        aggregate = _mapping(aggregate, "pilot_aggregate")
        _require_exact_keys(
            aggregate,
            {"schema_version", "pilot_manifest", "conditions"},
            "pilot_aggregate",
        )
        if aggregate.get("schema_version") != SUFFICIENT_PILOT_AGGREGATE_SCHEMA:
            raise V6ContractError("unsupported sufficient Pilot aggregate")
        aggregate_manifest = Path(str(aggregate.get("pilot_manifest", ""))).expanduser()
        if not aggregate_manifest.is_absolute() or aggregate_manifest.resolve() != pilot_path:
            raise V6ContractError("Pilot aggregate manifest binding mismatch")
        rows = aggregate.get("conditions")
        if not isinstance(rows, list) or len(rows) != 6:
            raise V6ContractError("Pilot aggregate must contain six conditions")
        sessions: dict[str, str] = {}
        evidence_index: list[dict[str, Any]] = []
        for expected_condition, row_value in zip(frozen.conditions, rows):
            row = _mapping(row_value, "pilot_aggregate.conditions[]")
            _require_exact_keys(row, {"id", "scene", "category", "episodes"}, "pilot_aggregate.conditions[]")
            if (row.get("id"), row.get("scene"), row.get("category")) != (
                expected_condition.condition_id,
                expected_condition.scene,
                expected_condition.category,
            ):
                raise V6ContractError("Pilot aggregate condition order/identity mismatch")
            episodes = row.get("episodes")
            if not isinstance(episodes, list) or len(episodes) != 3:
                raise V6ContractError("Pilot condition must contain exactly three episodes")
            generations = []
            for rep, episode_row_value in enumerate(episodes, start=1):
                episode_row = _mapping(episode_row_value, "pilot_aggregate.episodes[]")
                _require_exact_keys(
                    episode_row,
                    {
                        "rep",
                        "boundary",
                        "summary_path",
                        "manifest_path",
                        "stack_contract_path",
                        "stack_tuple_digest",
                    },
                    "pilot_aggregate.episodes[]",
                )
                expected_boundary = "cold" if rep == 1 else "hot_reset"
                if episode_row.get("rep") != rep or episode_row.get("boundary") != expected_boundary:
                    raise V6ContractError("Pilot cold/hot episode order mismatch")
                summary_input = Path(str(episode_row["summary_path"])).expanduser()
                manifest_input = Path(str(episode_row["manifest_path"])).expanduser()
                stack_contract_input = Path(
                    str(episode_row["stack_contract_path"])
                ).expanduser()
                if (
                    not summary_input.is_absolute()
                    or not manifest_input.is_absolute()
                    or not stack_contract_input.is_absolute()
                ):
                    raise V6ContractError("Pilot episode evidence paths must be absolute")
                session, generation, indexed = _validate_sufficient_pilot_episode(
                    condition=expected_condition,
                    rep=rep,
                    summary_path=summary_input.resolve(),
                    manifest_path=manifest_input.resolve(),
                    stack_contract_path=stack_contract_input.resolve(),
                    expected_stack_tuple_digest=str(
                        episode_row["stack_tuple_digest"]
                    ),
                    freeze=frozen.freeze,
                    freeze_digest=frozen.freeze_digest,
                )
                sessions.setdefault(expected_condition.condition_id, session)
                if sessions[expected_condition.condition_id] != session:
                    raise V6ContractError("Pilot condition stack session changed")
                generations.append(generation)
                evidence_index.append(indexed)
            if generations != [2, 3, 4]:
                raise V6ContractError(
                    "Pilot reset generations do not match fresh T2 baseline"
                )
        candidate["pilot_freeze_provenance"] = {
            "schema": "bio_nav.v6_pilot_freeze_provenance.v1",
            "pilot_manifest": {
                "path": str(pilot_path),
                "sha256": _file_sha256(pilot_path),
            },
            "pilot_aggregate": {
                "path": str(aggregate_path),
                "sha256": _file_sha256(aggregate_path),
            },
            "episodes": evidence_index,
        }
        temporary.write_text(
            json.dumps(candidate, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        frozen = load_formal_campaign_manifest(temporary)
        try:
            os.link(temporary, output_path)
        except FileExistsError as exc:
            raise V6ContractError("output manifest already exists") from exc
        published = True
        return replace(frozen, path=output_path)
    finally:
        if temporary.exists():
            temporary.unlink()
        if not published:
            try:
                output_root.rmdir()
            except OSError:
                pass


def freeze_indoor_campaign_from_pilot(
    *,
    pilot_manifest_path: str | Path,
    pilot_aggregate_path: str | Path,
    output_manifest_path: str | Path,
    indoor_output_root: str | Path,
) -> IndoorCampaignManifest:
    inputs = [
        Path(value).expanduser()
        for value in (
            pilot_manifest_path,
            pilot_aggregate_path,
            output_manifest_path,
        )
    ]
    if not all(path.is_absolute() for path in inputs):
        raise V6ContractError("indoor freezer paths must be absolute")
    pilot_path, aggregate_path, output_path = (path.resolve() for path in inputs)
    output_root = Path(indoor_output_root).expanduser()
    if not output_root.is_absolute():
        raise V6ContractError("indoor output root must be absolute")
    output_root = output_root.resolve()
    try:
        output_root.relative_to(FORMAL_NAS_ROOT.resolve())
    except ValueError as exc:
        raise V6ContractError("indoor output root must be under the NAS root") from exc
    _validate_nas_mount(output_root)
    if output_root.exists():
        raise V6ContractError("indoor output root must be new")
    if output_path.exists() or not output_path.parent.is_dir():
        raise V6ContractError("indoor manifest must be a new file in an existing directory")
    try:
        pilot = _mapping(
            yaml.safe_load(pilot_path.read_text(encoding="utf-8")),
            "indoor_pilot_manifest",
        )
        aggregate = _mapping(
            json.loads(aggregate_path.read_text(encoding="utf-8")),
            "indoor_pilot_aggregate",
        )
    except (OSError, json.JSONDecodeError, yaml.YAMLError) as exc:
        raise V6ContractError(f"indoor freezer input is unreadable: {exc}") from exc
    _require_exact_keys(
        pilot,
        {
            "schema_version", "intended_use", "runner_entrypoint", "freeze",
            "conditions",
        },
        "indoor_pilot_manifest",
    )
    if (
        pilot.get("schema_version") != INDOOR_PILOT_MANIFEST_SCHEMA
        or pilot.get("intended_use") != "indoor_pilot"
    ):
        raise V6ContractError("unsupported indoor Pilot manifest")
    rows = pilot.get("conditions")
    if not isinstance(rows, list) or len(rows) != 3:
        raise V6ContractError("indoor Pilot manifest must contain three conditions")
    hardened_freeze = json.loads(json.dumps(pilot["freeze"]))
    first_stage = hardened_freeze.get("validator_only_head_promotion")
    if first_stage is not None:
        current_repositories = {
            name: _repository_freeze_entry_with_tree(
                Path(hardened_freeze["repositories"][name]["path"])
            )
            for name in ("integration", "module2", "module3")
        }
        aggregate_repositories = first_stage["final_repositories"]
        identity_changed = any(
            Path(str(aggregate_repositories[name]["path"])).resolve()
            != Path(str(current_repositories[name]["path"])).resolve()
            or aggregate_repositories[name]["head"]
            != current_repositories[name]["head"]
            for name in ("integration", "module2", "module3")
        )
        if identity_changed:
            records = {}
            for name in ("integration", "module3"):
                repository = Path(current_repositories[name]["path"])
                from_head = aggregate_repositories[name]["head"]
                to_head = current_repositories[name]["head"]
                rows_value, digest = _validator_only_diff_evidence(
                    repository, from_head, to_head
                )
                records[f"{name}_diff"] = {
                    "from_head": from_head,
                    "to_head": to_head,
                    "name_status": rows_value,
                    "canonical_diff_sha256": digest,
                }
            hardened_freeze["repositories"] = current_repositories
            hardened_freeze["v6_formal"] = _frozen_file_entry(Path(__file__).resolve())
            hardened_freeze["experiment_runner"] = _frozen_file_entry(
                Path(__file__).with_name("experiment_runner.py")
            )
            hardened_freeze["evaluator_hardening_promotion"] = {
                "first_stage_sha256": hashlib.sha256(
                    json.dumps(
                        first_stage, sort_keys=True, separators=(",", ":")
                    ).encode()
                ).hexdigest(),
                "aggregate_final_repositories": first_stage["final_repositories"],
                "hardened_repositories": current_repositories,
                "pilot_manifest_sha256": _file_sha256(pilot_path),
                "pilot_aggregate_sha256": _file_sha256(aggregate_path),
                **records,
                "loaded_validator": _validator_only_loaded_identity(
                    Path(current_repositories["module3"]["path"]),
                    current_repositories["module3"]["head"],
                ),
            }
            _validate_evaluator_hardening_chain(
                hardened_freeze["evaluator_hardening_promotion"],
                freeze=hardened_freeze,
                pilot_manifest_sha256=_file_sha256(pilot_path),
                pilot_aggregate_sha256=_file_sha256(aggregate_path),
            )
    candidate = {
        "schema_version": INDOOR_CAMPAIGN_SCHEMA_VERSION,
        "intended_use": "indoor_qualification",
        "runs_per_condition": FORMAL_RUNS_PER_CONDITION,
        "runner_entrypoint": pilot["runner_entrypoint"],
        "freeze": hardened_freeze,
        "conditions": [],
    }
    for row_value in rows:
        row = dict(_mapping(row_value, "indoor_pilot_manifest.conditions[]"))
        _require_exact_keys(
            row,
            {"id", "scene", "category", "scenario_file", "runner_arguments"},
            "indoor_pilot_manifest.conditions[]",
        )
        if not Path(str(row.get("scenario_file", ""))).expanduser().is_absolute():
            raise V6ContractError("indoor Pilot scenario paths must be absolute")
        candidate["conditions"].append(
            {**row, "output_directory": str(output_root / str(row["id"]))}
        )
    expected_aggregate_keys = {"schema_version", "pilot_manifest", "conditions"}
    pilot_promotion = _mapping(
        pilot.get("freeze"), "indoor_pilot_manifest.freeze"
    ).get("validator_only_head_promotion")
    if pilot_promotion is not None:
        expected_aggregate_keys.add("validator_only_head_promotion")
    _require_exact_keys(aggregate, expected_aggregate_keys, "indoor_pilot_aggregate")
    if aggregate.get("schema_version") != INDOOR_PILOT_AGGREGATE_SCHEMA:
        raise V6ContractError("unsupported indoor Pilot aggregate")
    aggregate_manifest = Path(str(aggregate.get("pilot_manifest", ""))).expanduser()
    if not aggregate_manifest.is_absolute() or aggregate_manifest.resolve() != pilot_path:
        raise V6ContractError("indoor Pilot aggregate manifest binding mismatch")
    aggregate_rows = aggregate.get("conditions")
    if not isinstance(aggregate_rows, list) or len(aggregate_rows) != 3:
        raise V6ContractError("indoor Pilot aggregate must contain three conditions")
    if aggregate.get("validator_only_head_promotion") != pilot_promotion:
        raise V6ContractError("indoor Pilot promotion binding mismatch")
    if not output_root.parent.is_dir():
        raise V6ContractError("indoor output root parent must exist")
    output_root.mkdir(exist_ok=False)
    try:
        temporary_fd, temporary_name = tempfile.mkstemp(
            prefix=f".{output_path.name}.", suffix=".tmp", dir=output_path.parent
        )
    except OSError:
        output_root.rmdir()
        raise
    os.close(temporary_fd)
    temporary = Path(temporary_name)
    published = False
    try:
        temporary.write_text(
            json.dumps(candidate, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        campaign = load_indoor_campaign_manifest(
            temporary, _require_pilot_provenance=False
        )
        pilot_freeze = _pilot_freeze_from_validator_promotion(
            campaign.freeze, campaign.freeze.get("validator_only_head_promotion")
        )
        sessions: dict[str, str] = {}
        evidence_index: list[dict[str, Any]] = []
        for expected_condition, row_value in zip(campaign.conditions, aggregate_rows):
            row = _mapping(row_value, "indoor_pilot_aggregate.conditions[]")
            _require_exact_keys(
                row, {"id", "scene", "category", "episodes"},
                "indoor_pilot_aggregate.conditions[]",
            )
            if (row.get("id"), row.get("scene"), row.get("category")) != (
                expected_condition.condition_id,
                expected_condition.scene,
                expected_condition.category,
            ):
                raise V6ContractError("indoor Pilot condition order/identity mismatch")
            episodes = row.get("episodes")
            if not isinstance(episodes, list) or len(episodes) != 3:
                raise V6ContractError("indoor Pilot condition must contain three episodes")
            generations = []
            for rep, episode_value in enumerate(episodes, start=1):
                episode_row = _mapping(episode_value, "indoor_pilot_aggregate.episodes[]")
                _require_exact_keys(
                    episode_row,
                    {
                        "rep", "boundary", "summary_path", "manifest_path",
                        "stack_contract_path", "stack_tuple_digest",
                    },
                    "indoor_pilot_aggregate.episodes[]",
                )
                boundary = "cold" if rep == 1 else "hot_reset"
                if episode_row.get("rep") != rep or episode_row.get("boundary") != boundary:
                    raise V6ContractError("indoor Pilot cold/hot episode order mismatch")
                evidence_paths = [
                    Path(str(episode_row[name])).expanduser()
                    for name in ("summary_path", "manifest_path", "stack_contract_path")
                ]
                if not all(path.is_absolute() for path in evidence_paths):
                    raise V6ContractError("indoor Pilot evidence paths must be absolute")
                session, generation, indexed = _validate_sufficient_pilot_episode(
                    condition=expected_condition,
                    rep=rep,
                    summary_path=evidence_paths[0].resolve(),
                    manifest_path=evidence_paths[1].resolve(),
                    stack_contract_path=evidence_paths[2].resolve(),
                    expected_stack_tuple_digest=str(episode_row["stack_tuple_digest"]),
                    freeze=pilot_freeze,
                    freeze_digest=campaign.freeze_digest,
                )
                sessions.setdefault(expected_condition.condition_id, session)
                if sessions[expected_condition.condition_id] != session:
                    raise V6ContractError("indoor Pilot condition stack session changed")
                generations.append(generation)
                evidence_index.append(indexed)
            if generations != [2, 3, 4]:
                raise V6ContractError(
                    "indoor Pilot reset generations do not match fresh T2 baseline"
                )
        candidate["indoor_pilot_freeze_provenance"] = {
            "schema": "bio_nav.v6_indoor_pilot_freeze_provenance.v1",
            "pilot_manifest": {
                "path": str(pilot_path),
                "sha256": _file_sha256(pilot_path),
            },
            "pilot_aggregate": {
                "path": str(aggregate_path),
                "sha256": _file_sha256(aggregate_path),
            },
            "episodes": evidence_index,
        }
        temporary.write_text(
            json.dumps(candidate, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        campaign = load_indoor_campaign_manifest(temporary)
        try:
            os.link(temporary, output_path)
        except FileExistsError as exc:
            raise V6ContractError("indoor manifest already exists") from exc
        published = True
        return replace(campaign, path=output_path)
    finally:
        temporary.unlink(missing_ok=True)
        if not published:
            try:
                output_root.rmdir()
            except OSError:
                pass


def freeze_outdoor_campaign_from_pilot(
    *,
    pilot_manifest_path: str | Path,
    pilot_aggregate_path: str | Path,
    output_manifest_path: str | Path,
    outdoor_output_root: str | Path,
) -> OutdoorCampaignManifest:
    inputs = [Path(value).expanduser() for value in (
        pilot_manifest_path, pilot_aggregate_path, output_manifest_path,
    )]
    if not all(path.is_absolute() for path in inputs):
        raise V6ContractError("outdoor freezer paths must be absolute")
    pilot_path, aggregate_path, output_path = (path.resolve() for path in inputs)
    output_root = Path(outdoor_output_root).expanduser()
    if not output_root.is_absolute():
        raise V6ContractError("outdoor output root must be absolute")
    output_root = output_root.resolve()
    try:
        output_root.relative_to(FORMAL_NAS_ROOT.resolve())
    except ValueError as exc:
        raise V6ContractError("outdoor output root must be under NAS") from exc
    _validate_nas_mount(output_root)
    if output_root.exists() or output_path.exists() or not output_path.parent.is_dir():
        raise V6ContractError("outdoor outputs must be new")
    try:
        pilot = _mapping(yaml.safe_load(pilot_path.read_text()), "outdoor_pilot_manifest")
        aggregate = _mapping(json.loads(aggregate_path.read_text()), "outdoor_pilot_aggregate")
    except (OSError, json.JSONDecodeError, yaml.YAMLError) as exc:
        raise V6ContractError(f"outdoor freezer input is unreadable: {exc}") from exc
    _require_exact_keys(
        pilot,
        {"schema_version", "intended_use", "runner_entrypoint", "freeze", "conditions"},
        "outdoor_pilot_manifest",
    )
    if (
        pilot.get("schema_version") != OUTDOOR_PILOT_MANIFEST_SCHEMA
        or pilot.get("intended_use") != "outdoor_pilot"
    ):
        raise V6ContractError("unsupported outdoor Pilot manifest")
    _require_exact_keys(
        aggregate, {"schema_version", "pilot_manifest", "conditions"},
        "outdoor_pilot_aggregate",
    )
    if aggregate.get("schema_version") != OUTDOOR_PILOT_AGGREGATE_SCHEMA:
        raise V6ContractError("unsupported outdoor Pilot aggregate")
    bound_manifest = Path(str(aggregate.get("pilot_manifest", ""))).expanduser()
    if not bound_manifest.is_absolute() or bound_manifest.resolve() != pilot_path:
        raise V6ContractError("outdoor Pilot aggregate manifest binding mismatch")
    pilot_rows = pilot.get("conditions")
    aggregate_rows = aggregate.get("conditions")
    if not isinstance(pilot_rows, list) or len(pilot_rows) != 3:
        raise V6ContractError("outdoor Pilot manifest must contain three conditions")
    if not isinstance(aggregate_rows, list) or len(aggregate_rows) != 3:
        raise V6ContractError("outdoor Pilot aggregate must contain three conditions")
    candidate = {
        "schema_version": OUTDOOR_CAMPAIGN_SCHEMA_VERSION,
        "intended_use": "outdoor_qualification",
        "runs_per_condition": FORMAL_RUNS_PER_CONDITION,
        "runner_entrypoint": pilot["runner_entrypoint"],
        "freeze": pilot["freeze"],
        "conditions": [],
    }
    for value in pilot_rows:
        row = dict(_mapping(value, "outdoor_pilot_manifest.conditions[]"))
        _require_exact_keys(
            row, {"id", "scene", "category", "scenario_file", "runner_arguments"},
            "outdoor_pilot_manifest.conditions[]",
        )
        candidate["conditions"].append({
            **row, "output_directory": str(output_root / str(row["id"]))
        })
    if not output_root.parent.is_dir():
        raise V6ContractError("outdoor output root parent must exist")
    output_root.mkdir(exist_ok=False)
    try:
        fd, name = tempfile.mkstemp(
            prefix=f".{output_path.name}.", suffix=".tmp", dir=output_path.parent
        )
    except OSError:
        output_root.rmdir()
        raise
    os.close(fd)
    temporary = Path(name)
    published = False
    try:
        temporary.write_text(json.dumps(candidate, indent=2, sort_keys=True) + "\n")
        campaign = load_outdoor_campaign_manifest(
            temporary, _require_pilot_provenance=False
        )
        evidence_index = []
        sessions: dict[str, str] = {}
        for condition, value in zip(campaign.conditions, aggregate_rows):
            row = _mapping(value, "outdoor_pilot_aggregate.conditions[]")
            _require_exact_keys(
                row, {"id", "scene", "category", "episodes"},
                "outdoor_pilot_aggregate.conditions[]",
            )
            if (row.get("id"), row.get("scene"), row.get("category")) != (
                condition.condition_id, condition.scene, condition.category,
            ):
                raise V6ContractError("outdoor Pilot condition identity/order mismatch")
            episodes = row.get("episodes")
            if not isinstance(episodes, list) or len(episodes) != 3:
                raise V6ContractError("outdoor Pilot condition must contain three episodes")
            generations = []
            for rep, episode_value in enumerate(episodes, start=1):
                episode = _mapping(episode_value, "outdoor_pilot_aggregate.episodes[]")
                _require_exact_keys(
                    episode,
                    {"rep", "boundary", "summary_path", "manifest_path", "stack_contract_path", "stack_tuple_digest"},
                    "outdoor_pilot_aggregate.episodes[]",
                )
                boundary = "cold" if rep == 1 else "hot_reset"
                if episode.get("rep") != rep or episode.get("boundary") != boundary:
                    raise V6ContractError("outdoor Pilot cold/hot order mismatch")
                paths = [Path(str(episode[name])).expanduser() for name in (
                    "summary_path", "manifest_path", "stack_contract_path",
                )]
                if not all(path.is_absolute() for path in paths):
                    raise V6ContractError("outdoor Pilot evidence paths must be absolute")
                session, generation, indexed = _validate_sufficient_pilot_episode(
                    condition=condition,
                    rep=rep,
                    summary_path=paths[0].resolve(),
                    manifest_path=paths[1].resolve(),
                    stack_contract_path=paths[2].resolve(),
                    expected_stack_tuple_digest=str(episode["stack_tuple_digest"]),
                    freeze=campaign.freeze,
                    freeze_digest=campaign.freeze_digest,
                )
                sessions.setdefault(condition.condition_id, session)
                if sessions[condition.condition_id] != session:
                    raise V6ContractError("outdoor Pilot condition stack session changed")
                generations.append(generation)
                evidence_index.append(indexed)
            if generations != [2, 3, 4]:
                raise V6ContractError("outdoor Pilot reset generations must be 2,3,4")
        candidate["outdoor_pilot_freeze_provenance"] = {
            "schema": "bio_nav.v6_outdoor_pilot_freeze_provenance.v1",
            "pilot_manifest": {"path": str(pilot_path), "sha256": _file_sha256(pilot_path)},
            "pilot_aggregate": {"path": str(aggregate_path), "sha256": _file_sha256(aggregate_path)},
            "episodes": evidence_index,
        }
        temporary.write_text(json.dumps(candidate, indent=2, sort_keys=True) + "\n")
        campaign = load_outdoor_campaign_manifest(temporary)
        try:
            os.link(temporary, output_path)
        except FileExistsError as exc:
            raise V6ContractError("outdoor manifest already exists") from exc
        published = True
        return replace(campaign, path=output_path)
    finally:
        temporary.unlink(missing_ok=True)
        if not published:
            try:
                output_root.rmdir()
            except OSError:
                pass


def _revalidate_pilot_freeze_provenance(
    provenance: Mapping[str, Any],
    *,
    conditions: tuple[FormalCondition, ...],
    freeze: Mapping[str, Any],
    freeze_digest: str,
) -> None:
    for name in ("pilot_manifest", "pilot_aggregate"):
        _validate_frozen_file(
            provenance.get(name), f"pilot_freeze_provenance.{name}"
        )
    rows = provenance.get("episodes")
    assert isinstance(rows, list)
    for index, row_value in enumerate(rows):
        row = _mapping(row_value, f"pilot_freeze_provenance.episodes[{index}]")
        _require_exact_keys(
            row,
            {
                "condition_id",
                "rep",
                "summary",
                "manifest",
                "checksums",
                "stack_contract",
                "stack_tuple_digest",
            },
            f"pilot_freeze_provenance.episodes[{index}]",
        )
        condition = conditions[index // 3]
        rep = index % 3 + 1
        if row.get("condition_id") != condition.condition_id or row.get("rep") != rep:
            raise V6ContractError("pilot freeze evidence index order mismatch")
        summary_entry = _validate_frozen_file(
            row.get("summary"), f"pilot_freeze_provenance.episodes[{index}].summary"
        )
        manifest_entry = _validate_frozen_file(
            row.get("manifest"), f"pilot_freeze_provenance.episodes[{index}].manifest"
        )
        checksum_entry = _validate_frozen_file(
            row.get("checksums"), f"pilot_freeze_provenance.episodes[{index}].checksums"
        )
        stack_entry = _validate_frozen_file(
            row.get("stack_contract"),
            f"pilot_freeze_provenance.episodes[{index}].stack_contract",
        )
        summary_path = Path(summary_entry["path"])
        if Path(checksum_entry["path"]) != summary_path.parent / "checksums.sha256":
            raise V6ContractError("pilot freeze checksum index path mismatch")
        _validate_sufficient_pilot_episode(
            condition=condition,
            rep=rep,
            summary_path=summary_path,
            manifest_path=Path(manifest_entry["path"]),
            stack_contract_path=Path(stack_entry["path"]),
            expected_stack_tuple_digest=str(row.get("stack_tuple_digest", "")),
            freeze=freeze,
            freeze_digest=freeze_digest,
        )


def _revalidate_indoor_pilot_freeze_provenance(
    provenance: Mapping[str, Any],
    *,
    conditions: tuple[FormalCondition, ...],
    freeze: Mapping[str, Any],
    freeze_digest: str,
) -> None:
    for name in ("pilot_manifest", "pilot_aggregate"):
        _validate_frozen_file(
            provenance.get(name), f"indoor_pilot_freeze_provenance.{name}"
        )
    rows = provenance.get("episodes")
    assert isinstance(rows, list)
    for index, row_value in enumerate(rows):
        path = f"indoor_pilot_freeze_provenance.episodes[{index}]"
        row = _mapping(row_value, path)
        _require_exact_keys(
            row,
            {
                "condition_id", "rep", "summary", "manifest", "checksums",
                "stack_contract", "stack_tuple_digest",
            },
            path,
        )
        condition = conditions[index // 3]
        rep = index % 3 + 1
        if row.get("condition_id") != condition.condition_id or row.get("rep") != rep:
            raise V6ContractError("indoor Pilot freeze evidence index order mismatch")
        summary_entry = _validate_frozen_file(row.get("summary"), f"{path}.summary")
        manifest_entry = _validate_frozen_file(row.get("manifest"), f"{path}.manifest")
        checksum_entry = _validate_frozen_file(row.get("checksums"), f"{path}.checksums")
        stack_entry = _validate_frozen_file(
            row.get("stack_contract"), f"{path}.stack_contract"
        )
        summary_path = Path(summary_entry["path"])
        if Path(checksum_entry["path"]) != summary_path.parent / "checksums.sha256":
            raise V6ContractError("indoor Pilot freeze checksum index path mismatch")
        _validate_sufficient_pilot_episode(
            condition=condition,
            rep=rep,
            summary_path=summary_path,
            manifest_path=Path(manifest_entry["path"]),
            stack_contract_path=Path(stack_entry["path"]),
            expected_stack_tuple_digest=str(row.get("stack_tuple_digest", "")),
            freeze=freeze,
            freeze_digest=freeze_digest,
        )


def _frozen_file_entry(path: Path) -> dict[str, str]:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise V6ContractError(f"required Pilot freeze file is missing: {path}")
    return {"path": str(path), "sha256": _file_sha256(path)}


def _repository_freeze_entry(path: Path) -> dict[str, str]:
    path = path.expanduser().resolve()
    try:
        head = subprocess.run(
            ["git", "-C", str(path), "rev-parse", "HEAD"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise V6ContractError(f"Pilot repository is invalid: {path}") from exc
    return {"path": str(path), "head": head}


def _repository_freeze_entry_with_tree(path: Path) -> dict[str, str]:
    entry = _repository_freeze_entry(path)
    try:
        tree = subprocess.run(
            ["git", "-C", entry["path"], "rev-parse", "HEAD^{tree}"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise V6ContractError(f"Pilot repository tree is invalid: {path}") from exc
    return {**entry, "tree": tree}


def _build_sufficient_pilot_manifest(
    pilot_root: Path,
) -> tuple[dict[str, Any], tuple[FormalCondition, ...], str, Mapping[str, Any]]:
    module3_root = Path(
        os.environ.get("BIO_NAV_MODULE3_ROOT", str(Path(__file__).resolve().parents[4]))
    ).expanduser().resolve()
    integration_root = Path(os.environ.get("BIO_NAV_INTEGRATION_ROOT", "")).expanduser()
    module2_root = Path(
        os.environ.get("BIO_NAV_MODULE2_ROOT")
        or os.environ.get("BIO_NAV_MODULE2_V310_ROOT", "")
    ).expanduser()
    asset_root = Path(os.environ.get("BIO_NAV_MODULE2_ASSET_ROOT", "")).expanduser()
    snapshot_root = Path(os.environ.get("BIO_NAV_ROUTE_PRIOR_SNAPSHOT", "")).expanduser()
    catalog_root = Path(os.environ.get("BIO_NAV_ROUTE_PRIOR_CATALOG", "")).expanduser()
    rivermark_usd = Path(os.environ.get("RIVERMARK_USD", "")).expanduser()
    for name, path in (
        ("Integration", integration_root),
        ("Module2", module2_root),
        ("Module3", module3_root),
        ("Module2 asset", asset_root),
        ("RoutePrior snapshot", snapshot_root),
        ("Rivermark catalog", catalog_root),
    ):
        if not path.is_absolute() or not path.exists():
            raise V6ContractError(f"{name} root is missing or not absolute")
    config_root = module3_root / "ros2_ws" / "src" / "robot_experiments" / "config"
    runner_entrypoint = module3_root / "scripts" / "run_experiment.sh"
    condition_rows = []
    scenario_entries = {}
    scenario_config_entries = {}
    for condition_id in FORMAL_CONDITION_IDS:
        scene, category = condition_id.split("_", 1)
        scenario_path = config_root / PILOT_SCENARIO_FILENAMES[condition_id]
        scenario = load_scenario(scenario_path)
        condition_rows.append({
            "id": condition_id,
            "scene": scene,
            "category": category,
            "scenario_file": str(scenario_path),
            "runner_arguments": [
                "nav2_profile:=v6_low_obstacle_isolation",
                f"nav2_config_file:={_canonical_nav2_config()}",
                "navigation_execution_backend:=route_guided",
                "require_module2_planning_ready:=true",
            ],
        })
        scenario_entries[condition_id] = _frozen_file_entry(scenario_path)
        configs = _scenario_runtime_config_paths(
            scenario, condition_rows[-1]["runner_arguments"]
        )
        scenario_config_entries[condition_id] = [
            _frozen_file_entry(path) for path in sorted(configs)
        ]
    frozen_assets = {
        "module1_checkpoint": _frozen_file_entry(
            asset_root / "weights/module1_mamba_metric_sensor_warm_v8.pt"
        ),
        "module2_srdr_checkpoint": _frozen_file_entry(
            asset_root / "weights/module2_srdr_v310_seed20260822.pt"
        ),
        "module2_visual_heads_shadow_checkpoint": _frozen_file_entry(
            asset_root / "weights/module2_srdr_v310_kujiale_0026_visual_heads_shadow_v1.pt"
        ),
        "selected_run4_visual_heads_checkpoint": _frozen_file_entry(
            asset_root / "weights/kujiale_0026_visual_heads_run4_v310.pt"
        ),
        "dino_checkpoint": _frozen_file_entry(
            asset_root / "third_party/dinov2/weights/dinov2_vits14_pretrain.pth"
        ),
        "indoor_route_prior_manifest": _frozen_file_entry(snapshot_root / "manifest.json"),
        "indoor_route_prior_m_sr": _frozen_file_entry(snapshot_root / "m_sr.npy"),
        "indoor_route_prior_m_dr": _frozen_file_entry(snapshot_root / "m_dr.npy"),
        "indoor_route_prior_transition": _frozen_file_entry(snapshot_root / "transition.npy"),
        "indoor_route_prior_valid_state_mask": _frozen_file_entry(snapshot_root / "valid_state_mask.npy"),
        "rivermark_usd": _frozen_file_entry(rivermark_usd),
        "rivermark_catalog": _frozen_file_entry(catalog_root / "catalog.json"),
        "rivermark_catalog_constraints_tree": {
            "path": str((catalog_root / "constraints").resolve()),
            "sha256": _constraints_tree_sha256(catalog_root / "constraints"),
        },
        "indoor_map_yaml": _frozen_file_entry(
            module3_root / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml"
        ),
        "indoor_map_pgm": _frozen_file_entry(
            module3_root / "data/maps/occupancy/v6_kujiale_isaacgen_v1.pgm"
        ),
        "outdoor_map_yaml": _frozen_file_entry(
            module3_root / "data/rivermark_demo/rivermark_selected.yaml"
        ),
        "outdoor_map_pgm": _frozen_file_entry(
            module3_root / "data/rivermark_demo/rivermark_selected.pgm"
        ),
    }
    pilot_manifest = {
        "schema_version": SUFFICIENT_PILOT_MANIFEST_SCHEMA,
        "intended_use": "sufficient_pilot",
        "runner_entrypoint": str(runner_entrypoint),
        "freeze": {
            "repositories": {
                "integration": _repository_freeze_entry(integration_root),
                "module2": _repository_freeze_entry(module2_root),
                "module3": _repository_freeze_entry(module3_root),
            },
            "driver_version": _current_driver_version(),
            "kernel_release": os.uname().release,
            "scenarios": scenario_entries,
            "scenario_configs": scenario_config_entries,
            "frozen_assets": frozen_assets,
            "runner_entrypoint": _frozen_file_entry(runner_entrypoint),
            "experiment_runner": _frozen_file_entry(
                module3_root / "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
            ),
            "v6_formal": _frozen_file_entry(
                module3_root / "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py"
            ),
        },
        "conditions": condition_rows,
    }
    validation_formal = {
        "schema_version": FORMAL_CAMPAIGN_SCHEMA_VERSION,
        "intended_use": "formal_qualification",
        "execution_authorization": FORMAL_EXECUTION_NOT_AUTHORIZED,
        "runs_per_condition": 20,
        "runner_entrypoint": str(runner_entrypoint),
        "freeze": pilot_manifest["freeze"],
        "conditions": [
            {**row, "output_directory": str(pilot_root / ".formal-validation" / row["id"])}
            for row in condition_rows
        ],
    }
    temporary_fd, temporary_name = tempfile.mkstemp(
        prefix=".pilot-freeze-validation.", suffix=".json", dir=pilot_root
    )
    os.close(temporary_fd)
    temporary = Path(temporary_name)
    try:
        temporary.write_text(json.dumps(validation_formal), encoding="utf-8")
        formal = load_formal_campaign_manifest(temporary)
    finally:
        temporary.unlink(missing_ok=True)
    return pilot_manifest, formal.conditions, formal.freeze_digest, formal.freeze


def _build_indoor_pilot_manifest(
    pilot_root: Path,
) -> tuple[dict[str, Any], tuple[FormalCondition, ...], str, Mapping[str, Any]]:
    module3_root = Path(
        os.environ.get("BIO_NAV_MODULE3_ROOT", str(Path(__file__).resolve().parents[4]))
    ).expanduser().resolve()
    integration_root = Path(os.environ.get("BIO_NAV_INTEGRATION_ROOT", "")).expanduser()
    module2_root = Path(
        os.environ.get("BIO_NAV_MODULE2_ROOT")
        or os.environ.get("BIO_NAV_MODULE2_V310_ROOT", "")
    ).expanduser()
    asset_root = Path(os.environ.get("BIO_NAV_MODULE2_ASSET_ROOT", "")).expanduser()
    snapshot_root = Path(os.environ.get("BIO_NAV_ROUTE_PRIOR_SNAPSHOT", "")).expanduser()
    for name, path in (
        ("Integration", integration_root),
        ("Module2", module2_root),
        ("Module3", module3_root),
        ("Module2 asset", asset_root),
        ("RoutePrior snapshot", snapshot_root),
    ):
        if not path.is_absolute() or not path.exists():
            raise V6ContractError(f"{name} root is missing or not absolute")
    config_root = module3_root / "ros2_ws/src/robot_experiments/config"
    runner_entrypoint = module3_root / "scripts/run_experiment.sh"
    nav2_config = (
        module3_root
        / "ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml"
    ).resolve()
    condition_rows = []
    scenario_entries = {}
    scenario_config_entries = {}
    physical_contracts = {}
    for condition_id in INDOOR_CONDITION_IDS:
        scene, category = condition_id.split("_", 1)
        scenario_path = config_root / PILOT_SCENARIO_FILENAMES[condition_id]
        scenario = load_scenario(scenario_path)
        runner_arguments = [
            "nav2_profile:=v6_low_obstacle_isolation",
            f"nav2_config_file:={nav2_config}",
            f"spawn_poses_file:={_canonical_indoor_spawn_manifest()}",
            "navigation_execution_backend:=route_guided",
            "require_module2_planning_ready:=true",
            "module2_planning_ready_timeout_sec:=120.0",
            "clear_slam_localization_buffer:=false",
            "reset_map_base_translation_tolerance_m:=0.1",
        ]
        condition_rows.append({
            "id": condition_id,
            "scene": scene,
            "category": category,
            "scenario_file": str(scenario_path),
            "runner_arguments": runner_arguments,
        })
        scenario_entries[condition_id] = _frozen_file_entry(scenario_path)
        scenario_config_entries[condition_id] = [
            _frozen_file_entry(path)
            for path in sorted(
                _scenario_runtime_config_paths(scenario, runner_arguments)
                | {
                    (
                        module3_root
                        / "ros2_ws/src/robot_experiments/launch/experiment.launch.py"
                    ).resolve()
                }
            )
        ]
        static_ids, dynamic_ids = INDOOR_EXPECTED_OBSTACLES[condition_id]
        if scenario.dynamic_config_file is None:
            raise V6ContractError(f"indoor physical config is missing: {condition_id}")
        physical_contracts[condition_id] = {
            "static_obstacle_ids": list(static_ids),
            "static_obstacle_count": len(static_ids),
            "dynamic_actor_ids": list(dynamic_ids),
            "dynamic_actor_count": len(dynamic_ids),
            "physical_config": _frozen_file_entry(
                scenario.resolve_path(scenario.dynamic_config_file)
            ),
            "scenario": _frozen_file_entry(scenario_path),
            "spawn_manifest": _frozen_file_entry(
                _canonical_indoor_spawn_manifest()
            ),
        }
    frozen_assets = {
        "module1_checkpoint": _frozen_file_entry(
            asset_root / "weights/module1_mamba_metric_sensor_warm_v8.pt"
        ),
        "module2_srdr_checkpoint": _frozen_file_entry(
            asset_root / "weights/module2_srdr_v310_seed20260822.pt"
        ),
        "module2_visual_heads_shadow_checkpoint": _frozen_file_entry(
            asset_root
            / "weights/module2_srdr_v310_kujiale_0026_visual_heads_shadow_v1.pt"
        ),
        "selected_run4_visual_heads_checkpoint": _frozen_file_entry(
            asset_root / "weights/kujiale_0026_visual_heads_run4_v310.pt"
        ),
        "dino_checkpoint": _frozen_file_entry(
            asset_root / "third_party/dinov2/weights/dinov2_vits14_pretrain.pth"
        ),
        "indoor_route_prior_manifest": _frozen_file_entry(
            snapshot_root / "manifest.json"
        ),
        "indoor_route_prior_m_sr": _frozen_file_entry(snapshot_root / "m_sr.npy"),
        "indoor_route_prior_m_dr": _frozen_file_entry(snapshot_root / "m_dr.npy"),
        "indoor_route_prior_transition": _frozen_file_entry(
            snapshot_root / "transition.npy"
        ),
        "indoor_route_prior_valid_state_mask": _frozen_file_entry(
            snapshot_root / "valid_state_mask.npy"
        ),
        "indoor_map_yaml": _frozen_file_entry(
            module3_root / "data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml"
        ),
        "indoor_map_pgm": _frozen_file_entry(
            module3_root / "data/maps/occupancy/v6_kujiale_isaacgen_v1.pgm"
        ),
    }
    freeze = {
        "repositories": {
            "integration": _repository_freeze_entry_with_tree(integration_root),
            "module2": _repository_freeze_entry_with_tree(module2_root),
            "module3": _repository_freeze_entry_with_tree(module3_root),
        },
        "driver_version": _current_driver_version(),
        "kernel_release": os.uname().release,
        "scenarios": scenario_entries,
        "scenario_configs": scenario_config_entries,
        "physical_contracts": physical_contracts,
        "frozen_assets": frozen_assets,
        "runner_entrypoint": _frozen_file_entry(runner_entrypoint),
        "experiment_runner": _frozen_file_entry(
            module3_root
            / "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
        ),
        "v6_formal": _frozen_file_entry(
            module3_root / "ros2_ws/src/robot_experiments/robot_experiments/v6_formal.py"
        ),
    }
    pilot_runtime = _derive_indoor_pilot_runtime(
        pilot_root, repositories=freeze["repositories"]
    )
    promotion = _validator_promotion_if_needed(
        freeze=freeze, pilot_runtime=pilot_runtime
    )
    if promotion is not None:
        freeze["validator_only_head_promotion"] = promotion
    pilot_manifest = {
        "schema_version": INDOOR_PILOT_MANIFEST_SCHEMA,
        "intended_use": "indoor_pilot",
        "runner_entrypoint": str(runner_entrypoint),
        "freeze": freeze,
        "conditions": condition_rows,
    }
    validation = {
        "schema_version": INDOOR_CAMPAIGN_SCHEMA_VERSION,
        "intended_use": "indoor_qualification",
        "runs_per_condition": FORMAL_RUNS_PER_CONDITION,
        "runner_entrypoint": str(runner_entrypoint),
        "freeze": freeze,
        "conditions": [
            {
                **row,
                "output_directory": str(
                    pilot_root / ".indoor-validation" / str(row["id"])
                ),
            }
            for row in condition_rows
        ],
    }
    temporary_fd, temporary_name = tempfile.mkstemp(
        prefix=".indoor-pilot-freeze-validation.", suffix=".json", dir=pilot_root
    )
    os.close(temporary_fd)
    temporary = Path(temporary_name)
    try:
        temporary.write_text(json.dumps(validation), encoding="utf-8")
        campaign = load_indoor_campaign_manifest(
            temporary, _require_pilot_provenance=False
        )
    finally:
        temporary.unlink(missing_ok=True)
    return pilot_manifest, campaign.conditions, campaign.freeze_digest, campaign.freeze


def _derive_outdoor_pilot_runtime(
    pilot_root: Path, *, repositories: Mapping[str, Any]
) -> dict[str, Any]:
    paths: list[tuple[str, Path]] = []
    for condition_id in OUTDOOR_CONDITION_IDS:
        for rep in range(1, 4):
            matches = list(
                (pilot_root / condition_id / f"rep{rep}").glob(
                    "*/run-*/stack_contract.json"
                )
            )
            if len(matches) != 1 or matches[0].is_symlink():
                raise V6ContractError("outdoor Pilot stack snapshot topology mismatch")
            paths.append((condition_id, matches[0]))
    tuples: dict[str, list[tuple[Any, ...]]] = {
        condition_id: [] for condition_id in OUTDOOR_CONDITION_IDS
    }
    for condition_id, path in paths:
        try:
            payload = dict(_mapping(json.loads(path.read_text()), "outdoor_pilot_stack"))
        except (OSError, json.JSONDecodeError) as exc:
            raise V6ContractError(f"outdoor Pilot stack is unreadable: {exc}") from exc
        _require_exact_keys(payload, _stack_contract_keys(payload), "outdoor_pilot_stack")
        if (
            payload.get("schema") != STACK_CONTRACT_SCHEMA
            or payload.get("stack_session_id") != _stack_session_id(payload)
            or payload.get("viewport_arm") != "B"
            or payload.get("disable_viewport_updates_requested") is not True
            or payload.get("disable_viewport_updates_observed") is not True
            or any(payload.get(f"{name}_dirty") is not False for name in (
                "integration", "module2", "module3"
            ))
        ):
            raise V6ContractError("outdoor Pilot stack identity/viewport mismatch")
        tuples[condition_id].append(tuple(payload.get(name) for name in (
            "integration_head", "integration_tree", "module2_head", "module2_tree",
            "module3_head", "module3_tree", "driver_version", "kernel_release",
            "viewport_winner_manifest_sha256", "viewport_runtime_attestation_sha256",
            "viewport_instance_uuid", "stack_session_id",
        )))
    if any(len(set(rows)) != 1 for rows in tuples.values()):
        raise V6ContractError("outdoor Pilot condition snapshots contain a mixed stack")
    condition_values = [rows[0] for rows in tuples.values()]
    if len({row[:9] for row in condition_values}) != 1:
        raise V6ContractError("outdoor Pilot stack snapshots contain a mixed runtime tuple")
    values = condition_values[0]
    return {
        "repositories": {
            name: {
                **repositories[name],
                "head": values[offset],
                "tree": values[offset + 1],
            }
            for name, offset in (("integration", 0), ("module2", 2), ("module3", 4))
        },
        "driver_version": values[6],
        "kernel_release": values[7],
        "viewport_winner_manifest_sha256": values[8],
    }


def _build_outdoor_pilot_manifest(
    pilot_root: Path,
) -> tuple[dict[str, Any], tuple[FormalCondition, ...], str, Mapping[str, Any]]:
    module3_root = Path(
        os.environ.get("BIO_NAV_MODULE3_ROOT", str(Path(__file__).resolve().parents[4]))
    ).expanduser().resolve()
    integration_root = Path(os.environ.get("BIO_NAV_INTEGRATION_ROOT", "")).expanduser()
    module2_root = Path(
        os.environ.get("BIO_NAV_MODULE2_ROOT")
        or os.environ.get("BIO_NAV_MODULE2_V310_ROOT", "")
    ).expanduser()
    asset_root = Path(os.environ.get("BIO_NAV_MODULE2_ASSET_ROOT", "")).expanduser()
    catalog_root = Path(os.environ.get("BIO_NAV_ROUTE_PRIOR_CATALOG", "")).expanduser()
    rivermark_usd = Path(os.environ.get("RIVERMARK_USD", "")).expanduser()
    winner_path = Path(
        os.environ.get("BIO_NAV_RIVERMARK_STARTUP_WINNER_MANIFEST", "")
    ).expanduser()
    for name, value in (
        ("Integration", integration_root), ("Module2", module2_root),
        ("Module3", module3_root), ("Module2 asset", asset_root),
        ("Rivermark catalog", catalog_root),
    ):
        if not value.is_absolute() or not value.exists():
            raise V6ContractError(f"{name} root is missing or not absolute")
    if not rivermark_usd.is_absolute() or not rivermark_usd.is_file():
        raise V6ContractError("Rivermark USD is missing or not absolute")
    if not winner_path.is_absolute() or not winner_path.is_file():
        raise V6ContractError("Rivermark viewport winner attestation is missing")
    winner = _viewport_winner_manifest(winner_path.resolve())
    config_root = module3_root / "ros2_ws/src/robot_experiments/config"
    runner = module3_root / "scripts/run_experiment.sh"
    nav2_config = _canonical_nav2_config()
    condition_rows = []
    scenarios = {}
    scenario_configs = {}
    for condition_id in OUTDOOR_CONDITION_IDS:
        scene, category = condition_id.split("_", 1)
        scenario_path = config_root / PILOT_SCENARIO_FILENAMES[condition_id]
        scenario = load_scenario(scenario_path)
        arguments = [
            "nav2_profile:=v6_low_obstacle_isolation",
            f"nav2_config_file:={nav2_config}",
            "navigation_execution_backend:=route_guided",
            "require_module2_planning_ready:=true",
        ]
        condition_rows.append({
            "id": condition_id, "scene": scene, "category": category,
            "scenario_file": str(scenario_path), "runner_arguments": arguments,
        })
        scenarios[condition_id] = _frozen_file_entry(scenario_path)
        scenario_configs[condition_id] = [
            _frozen_file_entry(path)
            for path in sorted(_scenario_runtime_config_paths(scenario, arguments))
        ]
    frozen_assets = {
        "module1_checkpoint": _frozen_file_entry(
            asset_root / "weights/module1_mamba_metric_sensor_warm_v8.pt"
        ),
        "module2_srdr_checkpoint": _frozen_file_entry(
            asset_root / "weights/module2_srdr_v310_seed20260822.pt"
        ),
        "module2_visual_heads_shadow_checkpoint": _frozen_file_entry(
            asset_root / "weights/module2_srdr_v310_kujiale_0026_visual_heads_shadow_v1.pt"
        ),
        "selected_run4_visual_heads_checkpoint": _frozen_file_entry(
            asset_root / "weights/kujiale_0026_visual_heads_run4_v310.pt"
        ),
        "dino_checkpoint": _frozen_file_entry(
            asset_root / "third_party/dinov2/weights/dinov2_vits14_pretrain.pth"
        ),
        "rivermark_usd": _frozen_file_entry(rivermark_usd),
        "rivermark_catalog": _frozen_file_entry(catalog_root / "catalog.json"),
        "rivermark_catalog_constraints_tree": {
            "path": str((catalog_root / "constraints").resolve()),
            "sha256": _constraints_tree_sha256(catalog_root / "constraints"),
        },
        "rivermark_viewport_startup_winner_manifest": {
            "path": winner["path"], "sha256": winner["sha256"],
        },
        "outdoor_map_yaml": _frozen_file_entry(
            module3_root / "data/rivermark_demo/rivermark_selected.yaml"
        ),
        "outdoor_map_pgm": _frozen_file_entry(
            module3_root / "data/rivermark_demo/rivermark_selected.pgm"
        ),
    }
    freeze = {
        "repositories": {
            "integration": _repository_freeze_entry_with_tree(integration_root),
            "module2": _repository_freeze_entry_with_tree(module2_root),
            "module3": _repository_freeze_entry_with_tree(module3_root),
        },
        "driver_version": _current_driver_version(),
        "kernel_release": os.uname().release,
        "scenarios": scenarios,
        "scenario_configs": scenario_configs,
        "frozen_assets": frozen_assets,
        "runner_entrypoint": _frozen_file_entry(runner),
        "experiment_runner": _frozen_file_entry(
            module3_root / "ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py"
        ),
        "v6_formal": _frozen_file_entry(Path(__file__).resolve()),
    }
    runtime = _derive_outdoor_pilot_runtime(pilot_root, repositories=freeze["repositories"])
    if not (
        runtime["repositories"] == freeze["repositories"]
        and runtime["driver_version"] == freeze["driver_version"]
        and runtime["kernel_release"] == freeze["kernel_release"]
        and runtime["viewport_winner_manifest_sha256"] == winner["sha256"]
    ):
        raise V6ContractError("outdoor Pilot must use the current exact tuple and winner B")
    pilot = {
        "schema_version": OUTDOOR_PILOT_MANIFEST_SCHEMA,
        "intended_use": "outdoor_pilot",
        "runner_entrypoint": str(runner),
        "freeze": freeze,
        "conditions": condition_rows,
    }
    validation = {
        "schema_version": OUTDOOR_CAMPAIGN_SCHEMA_VERSION,
        "intended_use": "outdoor_qualification",
        "runs_per_condition": FORMAL_RUNS_PER_CONDITION,
        "runner_entrypoint": str(runner),
        "freeze": freeze,
        "conditions": [
            {**row, "output_directory": str(pilot_root / ".outdoor-validation" / row["id"])}
            for row in condition_rows
        ],
    }
    fd, temporary_name = tempfile.mkstemp(
        prefix=".outdoor-pilot-freeze-validation.", suffix=".json", dir=pilot_root
    )
    os.close(fd)
    temporary = Path(temporary_name)
    try:
        temporary.write_text(json.dumps(validation), encoding="utf-8")
        campaign = load_outdoor_campaign_manifest(
            temporary, _require_pilot_provenance=False
        )
    finally:
        temporary.unlink(missing_ok=True)
    return pilot, campaign.conditions, campaign.freeze_digest, campaign.freeze


def _publish_no_clobber_json_pair(
    first_path: Path,
    first_payload: Mapping[str, Any],
    second_path: Path,
    second_payload: Mapping[str, Any],
) -> None:
    temporary_paths = []
    published = []
    try:
        for output, payload in ((first_path, first_payload), (second_path, second_payload)):
            fd, name = tempfile.mkstemp(
                prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
            )
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(payload, stream, indent=2, sort_keys=True)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            temporary_paths.append(Path(name))
        # The second (aggregate) link is the commit marker.  Consumers must
        # ignore a manifest unless the matching aggregate exists.
        for temporary, output in zip(temporary_paths, (first_path, second_path)):
            os.link(temporary, output)
            published.append(output)
    except OSError as exc:
        for output in published:
            output.unlink(missing_ok=True)
        raise V6ContractError("Pilot aggregate pair publish failed") from exc
    finally:
        for temporary in temporary_paths:
            temporary.unlink(missing_ok=True)


def _validate_indoor_pilot_sidecars(
    *,
    rep_root: Path,
    scenario_directory: Path,
    condition: FormalCondition,
    rep: int,
    expected_seed: int,
    final_manifest_path: Path,
) -> None:
    entries = list(rep_root.iterdir())
    if scenario_directory.is_symlink() or not scenario_directory.is_dir():
        raise V6ContractError("indoor Pilot scenario directory is not a real directory")
    sidecars = [path for path in entries if path != scenario_directory]
    if len(sidecars) != 2 or any(
        path.is_symlink() or not path.is_file() for path in sidecars
    ):
        raise V6ContractError(
            "indoor Pilot rep must contain exactly one regular JSON/CSV sidecar pair"
        )
    by_suffix = {path.suffix: path for path in sidecars}
    if set(by_suffix) != {".json", ".csv"}:
        raise V6ContractError("indoor Pilot sidecars must be one JSON and one CSV")
    json_path = by_suffix[".json"]
    csv_path = by_suffix[".csv"]
    if json_path.stem != csv_path.stem:
        raise V6ContractError("indoor Pilot sidecar stems differ")
    expected_prefix = (
        f"{condition.scenario_id}-run-{rep:04d}-seed-{expected_seed}-"
    )
    timestamp_pattern = r"[0-9]{8}T[0-9]{6}\.[0-9]{6}Z"
    if re.fullmatch(
        re.escape(expected_prefix) + timestamp_pattern, json_path.stem
    ) is None:
        raise V6ContractError("indoor Pilot sidecar stem identity is invalid")
    try:
        sidecar_manifest = json.loads(json_path.read_text(encoding="utf-8"))
        final_manifest = json.loads(final_manifest_path.read_text(encoding="utf-8"))
        if not isinstance(sidecar_manifest, Mapping) or not isinstance(
            final_manifest, Mapping
        ):
            raise V6ContractError("indoor Pilot sidecar/final manifest must be mappings")
        validate_manifest(sidecar_manifest)
    except (OSError, json.JSONDecodeError, ReportValidationError) as exc:
        raise V6ContractError(f"indoor Pilot JSON sidecar is invalid: {exc}") from exc
    canonical_json = (
        json.dumps(
            sidecar_manifest,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    if json_path.read_bytes() != canonical_json:
        raise V6ContractError("indoor Pilot JSON sidecar encoding is not canonical")
    if any(
        key not in final_manifest or final_manifest[key] != value
        for key, value in sidecar_manifest.items()
    ):
        raise V6ContractError(
            "indoor Pilot sidecar differs from checksummed final manifest projection"
        )
    identity_projection = {
        "scenario_id": condition.scenario_id,
        "run_index": rep,
        "random_seed": expected_seed,
    }
    if any(sidecar_manifest.get(key) != value for key, value in identity_projection.items()):
        raise V6ContractError("indoor Pilot sidecar run identity mismatch")
    for key in (
        "condition_id",
        "dynamic_selection",
        "appearance",
        "condition_stack_id",
        "stack_session_id",
        "provenance",
        "robot_config_hash",
        "nav2_config_hash",
        "scenario_runtime_hashes",
    ):
        if sidecar_manifest.get(key) != final_manifest.get(key):
            raise V6ContractError(
                f"indoor Pilot sidecar final-manifest mismatch: {key}"
            )
    fieldnames = list(REPRODUCIBILITY_FIELDS)
    fieldnames.extend(sorted(set(sidecar_manifest) - set(fieldnames)))
    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="raise")
    writer.writeheader()
    writer.writerow(
        {key: _csv_value(sidecar_manifest[key]) for key in fieldnames}
    )
    if csv_path.read_bytes() != stream.getvalue().encode("utf-8"):
        raise V6ContractError(
            "indoor Pilot CSV sidecar is not the deterministic report.py serialization"
        )


def aggregate_sufficient_pilot(
    *, pilot_root: str | Path, output_manifest: str | Path, output_aggregate: str | Path
) -> dict[str, Any]:
    root = Path(pilot_root).expanduser()
    manifest_output = Path(output_manifest).expanduser()
    aggregate_output = Path(output_aggregate).expanduser()
    if not all(path.is_absolute() for path in (root, manifest_output, aggregate_output)):
        raise V6ContractError("Pilot aggregate paths must be absolute")
    root = root.resolve()
    manifest_output = manifest_output.resolve()
    aggregate_output = aggregate_output.resolve()
    if not root.is_dir() or manifest_output.exists() or aggregate_output.exists():
        raise V6ContractError("Pilot root must exist and outputs must be new")
    for path in (root, manifest_output.parent, aggregate_output.parent):
        try:
            path.relative_to(FORMAL_NAS_ROOT.resolve())
        except ValueError as exc:
            raise V6ContractError("Pilot aggregate paths must be under the NAS root") from exc
        _validate_nas_mount(path)
    pilot_manifest, conditions, freeze_digest, freeze = (
        _build_sufficient_pilot_manifest(root)
    )
    aggregate_rows = []
    strict_successes = 0
    for condition in conditions:
        episodes = []
        for rep in range(1, 4):
            identity = condition.episode_identities[rep - 1]
            run_root = (
                root
                / condition.condition_id
                / f"rep{rep}"
                / condition.scenario_id
                / f"run-{rep:04d}-seed-{identity['seed']}"
            )
            summary_path = run_root / "run_summary.json"
            manifest_path = run_root / "run_manifest.json"
            stack_contract_path = run_root / "stack_contract.json"
            contract, tuple_digest = _load_stack_contract_snapshot(
                stack_contract_path,
                expected_condition_id=condition.condition_id,
                freeze=freeze,
            )
            _validate_sufficient_pilot_episode(
                condition=condition,
                rep=rep,
                summary_path=summary_path,
                manifest_path=manifest_path,
                stack_contract_path=stack_contract_path,
                expected_stack_tuple_digest=tuple_digest,
                freeze=freeze,
                freeze_digest=freeze_digest,
            )
            strict_successes += 1
            episodes.append({
                "rep": rep,
                "boundary": "cold" if rep == 1 else "hot_reset",
                "summary_path": str(summary_path),
                "manifest_path": str(manifest_path),
                "stack_contract_path": str(stack_contract_path),
                "stack_tuple_digest": tuple_digest,
            })
        aggregate_rows.append({
            "id": condition.condition_id,
            "scene": condition.scene,
            "category": condition.category,
            "episodes": episodes,
        })
    if strict_successes != 18:
        raise V6ContractError("sufficient Pilot did not produce 18 strict successes")
    aggregate = {
        "schema_version": SUFFICIENT_PILOT_AGGREGATE_SCHEMA,
        "pilot_manifest": str(manifest_output),
        "conditions": aggregate_rows,
    }
    _publish_no_clobber_json_pair(
        manifest_output, pilot_manifest, aggregate_output, aggregate
    )
    return {
        "qualification": "SUFFICIENT_PILOT_READY",
        "strict_successes": 18,
        "pilot_manifest": {
            "path": str(manifest_output), "sha256": _file_sha256(manifest_output)
        },
        "pilot_aggregate": {
            "path": str(aggregate_output), "sha256": _file_sha256(aggregate_output)
        },
        "dispatch": False,
    }


def aggregate_indoor_pilot(
    *, pilot_root: str | Path, output_manifest: str | Path, output_aggregate: str | Path
) -> dict[str, Any]:
    root = Path(pilot_root).expanduser()
    manifest_output = Path(output_manifest).expanduser()
    aggregate_output = Path(output_aggregate).expanduser()
    if not all(path.is_absolute() for path in (root, manifest_output, aggregate_output)):
        raise V6ContractError("indoor Pilot aggregate paths must be absolute")
    root = root.resolve()
    manifest_output = manifest_output.resolve()
    aggregate_output = aggregate_output.resolve()
    if not root.is_dir() or manifest_output.exists() or aggregate_output.exists():
        raise V6ContractError("indoor Pilot root must exist and outputs must be new")
    for path in (root, manifest_output.parent, aggregate_output.parent):
        try:
            path.relative_to(FORMAL_NAS_ROOT.resolve())
        except ValueError as exc:
            raise V6ContractError(
                "indoor Pilot aggregate paths must be under the NAS root"
            ) from exc
        _validate_nas_mount(path)
    top_level = {path.name for path in root.iterdir()}
    if top_level != set(INDOOR_CONDITION_IDS) or not all(
        (root / condition_id).is_dir() for condition_id in INDOOR_CONDITION_IDS
    ):
        raise V6ContractError(
            "indoor Pilot root must contain exactly the three indoor condition roots"
        )
    pilot_manifest, conditions, freeze_digest, freeze = _build_indoor_pilot_manifest(
        root
    )
    promotion = (
        _validate_validator_only_head_promotion(
            freeze.get("validator_only_head_promotion"), freeze=freeze
        )
        if freeze.get("validator_only_head_promotion") is not None
        else None
    )
    pilot_freeze = _pilot_freeze_from_validator_promotion(freeze, promotion)
    aggregate_rows = []
    strict_successes = 0
    for condition in conditions:
        condition_root = root / condition.condition_id
        expected_reps = {f"rep{rep}" for rep in range(1, 4)}
        if {path.name for path in condition_root.iterdir()} != expected_reps or not all(
            (condition_root / rep).is_dir() for rep in expected_reps
        ):
            raise V6ContractError(
                f"indoor Pilot condition must contain exactly rep1-rep3: {condition.condition_id}"
            )
        episodes = []
        for rep in range(1, 4):
            identity = condition.episode_identities[rep - 1]
            run_root = (
                root
                / condition.condition_id
                / f"rep{rep}"
                / condition.scenario_id
                / f"run-{rep:04d}-seed-{identity['seed']}"
            )
            rep_root = condition_root / f"rep{rep}"
            scenario_root = rep_root / condition.scenario_id
            if (
                len(list(rep_root.iterdir())) != 3
                or scenario_root.is_symlink()
                or not scenario_root.is_dir()
                or {path.name for path in scenario_root.iterdir()}
                != {run_root.name}
                or run_root.is_symlink()
                or not run_root.is_dir()
            ):
                raise V6ContractError(
                    f"indoor Pilot rep topology mismatch: {condition.condition_id}/rep{rep}"
                )
            summary_path = run_root / "run_summary.json"
            manifest_path = run_root / "run_manifest.json"
            stack_contract_path = run_root / "stack_contract.json"
            _validate_indoor_pilot_sidecars(
                rep_root=rep_root,
                scenario_directory=scenario_root,
                condition=condition,
                rep=rep,
                expected_seed=int(identity["seed"]),
                final_manifest_path=manifest_path,
            )
            _contract, tuple_digest = _load_stack_contract_snapshot(
                stack_contract_path,
                expected_condition_id=condition.condition_id,
                freeze=pilot_freeze,
            )
            _validate_sufficient_pilot_episode(
                condition=condition,
                rep=rep,
                summary_path=summary_path,
                manifest_path=manifest_path,
                stack_contract_path=stack_contract_path,
                expected_stack_tuple_digest=tuple_digest,
                freeze=pilot_freeze,
                freeze_digest=freeze_digest,
            )
            strict_successes += 1
            episodes.append({
                "rep": rep,
                "boundary": "cold" if rep == 1 else "hot_reset",
                "summary_path": str(summary_path),
                "manifest_path": str(manifest_path),
                "stack_contract_path": str(stack_contract_path),
                "stack_tuple_digest": tuple_digest,
            })
        aggregate_rows.append({
            "id": condition.condition_id,
            "scene": condition.scene,
            "category": condition.category,
            "episodes": episodes,
        })
    if strict_successes != 9:
        raise V6ContractError("indoor Pilot did not produce 9 strict successes")
    aggregate = {
        "schema_version": INDOOR_PILOT_AGGREGATE_SCHEMA,
        "pilot_manifest": str(manifest_output),
        "conditions": aggregate_rows,
    }
    if promotion is not None:
        aggregate["validator_only_head_promotion"] = promotion
    _publish_no_clobber_json_pair(
        manifest_output, pilot_manifest, aggregate_output, aggregate
    )
    return {
        "qualification": "INDOOR_PILOT_READY",
        "formal_qualification": NOT_QUALIFIED,
        "strict_successes": 9,
        "validator_only_head_promotion": promotion,
        "pilot_manifest": {
            "path": str(manifest_output),
            "sha256": _file_sha256(manifest_output),
        },
        "pilot_aggregate": {
            "path": str(aggregate_output),
            "sha256": _file_sha256(aggregate_output),
        },
        "dispatch": False,
    }


def aggregate_outdoor_pilot(
    *, pilot_root: str | Path, output_manifest: str | Path, output_aggregate: str | Path
) -> dict[str, Any]:
    root = Path(pilot_root).expanduser()
    manifest_output = Path(output_manifest).expanduser()
    aggregate_output = Path(output_aggregate).expanduser()
    if not all(path.is_absolute() for path in (root, manifest_output, aggregate_output)):
        raise V6ContractError("outdoor Pilot aggregate paths must be absolute")
    root, manifest_output, aggregate_output = (
        root.resolve(), manifest_output.resolve(), aggregate_output.resolve()
    )
    if not root.is_dir() or manifest_output.exists() or aggregate_output.exists():
        raise V6ContractError("outdoor Pilot root must exist and outputs must be new")
    for path in (root, manifest_output.parent, aggregate_output.parent):
        try:
            path.relative_to(FORMAL_NAS_ROOT.resolve())
        except ValueError as exc:
            raise V6ContractError("outdoor Pilot paths must be under NAS") from exc
        _validate_nas_mount(path)
    if {path.name for path in root.iterdir()} != set(OUTDOOR_CONDITION_IDS):
        raise V6ContractError("outdoor Pilot root must contain exactly three conditions")
    pilot, conditions, freeze_digest, freeze = _build_outdoor_pilot_manifest(root)
    aggregate_rows = []
    for condition in conditions:
        condition_root = root / condition.condition_id
        expected_reps = {f"rep{rep}" for rep in range(1, 4)}
        if {path.name for path in condition_root.iterdir()} != expected_reps:
            raise V6ContractError("outdoor Pilot condition must contain rep1-rep3")
        episodes = []
        for rep in range(1, 4):
            identity = condition.episode_identities[rep - 1]
            rep_root = condition_root / f"rep{rep}"
            scenario_root = rep_root / condition.scenario_id
            run_root = scenario_root / f"run-{rep:04d}-seed-{identity['seed']}"
            if (
                len(list(rep_root.iterdir())) != 3
                or scenario_root.is_symlink()
                or not scenario_root.is_dir()
                or {path.name for path in scenario_root.iterdir()} != {run_root.name}
                or run_root.is_symlink()
                or not run_root.is_dir()
            ):
                raise V6ContractError("outdoor Pilot rep topology mismatch")
            summary = run_root / "run_summary.json"
            episode = run_root / "run_manifest.json"
            stack = run_root / "stack_contract.json"
            _validate_indoor_pilot_sidecars(
                rep_root=rep_root,
                scenario_directory=scenario_root,
                condition=condition,
                rep=rep,
                expected_seed=int(identity["seed"]),
                final_manifest_path=episode,
            )
            _contract, tuple_digest = _load_stack_contract_snapshot(
                stack, expected_condition_id=condition.condition_id, freeze=freeze
            )
            _validate_sufficient_pilot_episode(
                condition=condition,
                rep=rep,
                summary_path=summary,
                manifest_path=episode,
                stack_contract_path=stack,
                expected_stack_tuple_digest=tuple_digest,
                freeze=freeze,
                freeze_digest=freeze_digest,
            )
            episodes.append({
                "rep": rep,
                "boundary": "cold" if rep == 1 else "hot_reset",
                "summary_path": str(summary),
                "manifest_path": str(episode),
                "stack_contract_path": str(stack),
                "stack_tuple_digest": tuple_digest,
            })
        aggregate_rows.append({
            "id": condition.condition_id,
            "scene": condition.scene,
            "category": condition.category,
            "episodes": episodes,
        })
    aggregate = {
        "schema_version": OUTDOOR_PILOT_AGGREGATE_SCHEMA,
        "pilot_manifest": str(manifest_output),
        "conditions": aggregate_rows,
    }
    _publish_no_clobber_json_pair(
        manifest_output, pilot, aggregate_output, aggregate
    )
    return {
        "qualification": "OUTDOOR_PILOT_READY",
        "formal_qualification": NOT_QUALIFIED,
        "strict_successes": 9,
        "pilot_manifest": {
            "path": str(manifest_output), "sha256": _file_sha256(manifest_output)
        },
        "pilot_aggregate": {
            "path": str(aggregate_output), "sha256": _file_sha256(aggregate_output)
        },
        "dispatch": False,
    }


def _checksums_verified(run_root: Path) -> bool:
    checksum_path = run_root / "checksums.sha256"
    if not checksum_path.is_file():
        return False
    try:
        entries = [
            line.split("  ", 1)
            for line in checksum_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
    except OSError:
        return False
    if not entries or any(len(entry) != 2 for entry in entries):
        return False
    covered = {relative for _digest, relative in entries}
    actual_regular_files = {
        str(path.relative_to(run_root))
        for path in run_root.rglob("*")
        if path.is_file()
        and not path.is_symlink()
        and path.name != "checksums.sha256"
    }
    if covered != actual_regular_files:
        return False
    if not {
        "run_summary.json",
        "run_manifest.json",
        "telemetry/metadata.yaml",
    } <= covered or not any(
        relative.startswith("telemetry/") and relative.endswith(".mcap")
        for relative in covered
    ):
        return False
    for digest, relative in entries:
        candidate = run_root / relative
        try:
            candidate.resolve().relative_to(run_root.resolve())
        except ValueError:
            return False
        if (
            len(digest) != 64
            or not candidate.is_file()
            or hashlib.sha256(candidate.read_bytes()).hexdigest() != digest
        ):
            return False
    return True


STACK_CONTRACT_SCHEMA_V1 = "bio_nav.v6_stack_contract.v1"
STACK_CONTRACT_SCHEMA_V2 = "bio_nav.v6_stack_contract.v2"
STACK_CONTRACT_SCHEMA = "bio_nav.v6_stack_contract.v3"
STACK_CONTRACT_KEYS_V1 = {
    "schema",
    "condition_id",
    "scene",
    "condition",
    "arm",
    "domain",
    "startup_profile",
    "pid",
    "pgid",
    "start_ticks",
    "boot_id",
    "integration_head",
    "module2_head",
    "module3_head",
    "integration_dirty",
    "module2_dirty",
    "module3_dirty",
    "driver_version",
    "kernel_release",
    "t2_selector_path",
    "t2_selector_sha256",
    "episode_sequence_path",
    "stack_session_id",
}
STACK_CONTRACT_V2_ONLY_KEYS = {
    "integration_tree",
    "module2_tree",
    "module3_tree",
    "viewport_arm",
    "disable_viewport_updates_requested",
    "disable_viewport_updates_observed",
    "viewport_startup_attestation_sha256",
}
STACK_CONTRACT_KEYS_V2 = STACK_CONTRACT_KEYS_V1 | STACK_CONTRACT_V2_ONLY_KEYS
STACK_CONTRACT_V3_ONLY_KEYS = {
    "contract_path",
    "viewport_runtime_attestation_path",
    "viewport_runtime_attestation_sha256",
    "viewport_instance_uuid",
    "viewport_pid",
    "viewport_pgid",
    "viewport_start_ticks",
    "viewport_winner_manifest_sha256",
}
STACK_CONTRACT_KEYS = STACK_CONTRACT_KEYS_V2 | STACK_CONTRACT_V3_ONLY_KEYS


def _stack_contract_keys(payload: Mapping[str, Any]) -> set[str]:
    if payload.get("schema") == STACK_CONTRACT_SCHEMA_V1:
        return STACK_CONTRACT_KEYS_V1
    if payload.get("schema") == STACK_CONTRACT_SCHEMA_V2:
        return STACK_CONTRACT_KEYS_V2
    if payload.get("schema") == STACK_CONTRACT_SCHEMA:
        return STACK_CONTRACT_KEYS
    raise V6ContractError("stack contract schema mismatch")


def _stack_session_id(payload: Mapping[str, Any]) -> str:
    keys = _stack_contract_keys(payload)
    basis = {key: payload[key] for key in keys - {"stack_session_id"}}
    canonical = json.dumps(basis, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode()).hexdigest()


STACK_TUPLE_KEYS = (
    "condition_id",
    "scene",
    "condition",
    "arm",
    "domain",
    "startup_profile",
    "integration_head",
    "module2_head",
    "module3_head",
    "integration_dirty",
    "module2_dirty",
    "module3_dirty",
    "driver_version",
    "kernel_release",
    "t2_selector_path",
    "t2_selector_sha256",
)


def _stack_tuple_digest(payload: Mapping[str, Any]) -> str:
    normalized = {key: payload[key] for key in STACK_TUPLE_KEYS}
    for key in sorted(STACK_CONTRACT_V2_ONLY_KEYS | STACK_CONTRACT_V3_ONLY_KEYS):
        if key in payload:
            normalized[key] = payload[key]
    canonical = json.dumps(normalized, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode()).hexdigest()


def _validate_viewport_runtime_snapshot(
    stack_path: Path,
    stack: Mapping[str, Any],
    *,
    expected_condition_id: str,
    freeze: Mapping[str, Any],
) -> dict[str, Any]:
    import stat
    import uuid

    evidence_path = stack_path.parent / "viewport_runtime_attestation.json"
    try:
        metadata = os.lstat(evidence_path)
        runtime = json.loads(evidence_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V6ContractError(f"viewport runtime evidence is unreadable: {exc}") from exc
    if (
        stat.S_ISLNK(metadata.st_mode)
        or not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != os.getuid()
        or stat.S_IMODE(metadata.st_mode) != 0o600
    ):
        raise V6ContractError("viewport runtime evidence owner/mode/type mismatch")
    runtime = _mapping(runtime, "viewport_runtime_attestation")
    _require_exact_keys(runtime, {
        "schema", "instance_uuid", "pid", "pgid", "start_ticks", "boot_id",
        "producer_script_realpath", "producer_script_sha256", "producer_executable",
        "argv_sha256", "start_wall_time_ns", "module3", "viewport_arm", "readbacks", "scene",
        "run_root", "launcher_path", "winner_manifest", "command_contract",
    }, "viewport_runtime_attestation")
    try:
        instance = str(uuid.UUID(str(runtime.get("instance_uuid", ""))))
    except (ValueError, AttributeError) as exc:
        raise V6ContractError("viewport runtime instance UUID invalid") from exc
    repositories = freeze["repositories"]
    winner = freeze["frozen_assets"].get(
        "rivermark_viewport_startup_winner_manifest"
    )
    readbacks = runtime.get("readbacks")
    expected_command = {
        "navigation_source": str(
            Path(repositories["module3"]["path"])
            / "isaac_sim/apps/navigation_sim.py"
        ),
        "producer_executable": runtime.get("producer_executable"),
        "argv_sha256": runtime.get("argv_sha256"),
        "disable_viewport_updates": True,
        "viewport_arm_identity": "B",
        "viewport_runtime_attestation": str(
            Path(str(stack.get("viewport_runtime_attestation_path", ""))).resolve()
        ),
        "viewport_winner_manifest": str(Path(str(winner.get("path", ""))).resolve())
        if isinstance(winner, Mapping)
        else "",
        "viewport_winner_manifest_sha256": winner.get("sha256")
        if isinstance(winner, Mapping)
        else None,
        "viewport_run_root": str(
            Path(str(stack.get("contract_path", ""))).resolve().parent
        ),
        "viewport_scene": f"rivermark:{expected_condition_id.split('_', 1)[1]}",
        "viewport_launcher": str(
            Path(repositories["module3"]["path"]).resolve()
            / "scripts/run_v6_rivermark.sh"
        ),
    }
    if not (
        runtime.get("schema") == "bio_nav.v6_viewport_runtime_attestation.v1"
        and instance == runtime.get("instance_uuid") == stack.get("viewport_instance_uuid")
        and runtime.get("pid") == stack.get("viewport_pid")
        and runtime.get("pgid") == stack.get("viewport_pgid")
        and runtime.get("start_ticks") == stack.get("viewport_start_ticks")
        and stack.get("viewport_runtime_attestation_sha256")
        == _file_sha256(evidence_path)
        == stack.get("viewport_startup_attestation_sha256")
        and runtime.get("module3") == repositories["module3"]
        and runtime.get("producer_script_realpath") == str(
            Path(repositories["module3"]["path"])
            / "isaac_sim/apps/navigation_sim.py"
        )
        and runtime.get("producer_script_sha256") == _file_sha256(
            Path(repositories["module3"]["path"])
            / "isaac_sim/apps/navigation_sim.py"
        )
        and runtime.get("viewport_arm") == stack.get("viewport_arm") == "B"
        and runtime.get("command_contract") == expected_command
        and isinstance(readbacks, list)
        and [row.get("phase") for row in readbacks]
        == ["post_construction", "pre_ready"]
        and all(
            row.get("requested_disabled") is True
            and row.get("observed_enabled") is False
            and row.get("match") is True
            for row in readbacks
        )
        and runtime.get("scene") == f"rivermark:{expected_condition_id.split('_', 1)[1]}"
        and Path(str(runtime.get("run_root", ""))).resolve()
        == Path(str(stack.get("contract_path", ""))).resolve().parent
        and Path(str(runtime.get("launcher_path", ""))).resolve()
        == Path(repositories["module3"]["path"]).resolve()
        / "scripts/run_v6_rivermark.sh"
        and isinstance(winner, Mapping)
        and runtime.get("winner_manifest") == winner
        and stack.get("viewport_winner_manifest_sha256") == winner.get("sha256")
    ):
        raise V6ContractError("outdoor viewport runtime attestation mismatch")
    return dict(runtime)


def _load_stack_contract_snapshot(
    path: Path,
    *,
    expected_condition_id: str,
    freeze: Mapping[str, Any],
) -> tuple[dict[str, Any], str]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V6ContractError(f"Pilot stack contract is unreadable: {exc}") from exc
    payload = dict(_mapping(payload, "pilot_stack_contract"))
    keys = _stack_contract_keys(payload)
    _require_exact_keys(payload, keys, "pilot_stack_contract")
    if (
        payload.get("condition_id") != expected_condition_id
        or payload.get("condition_id") != f"{payload.get('scene')}_{payload.get('condition')}"
        or payload.get("stack_session_id") != _stack_session_id(payload)
    ):
        raise V6ContractError("Pilot stack contract identity/digest mismatch")
    if (
        payload["schema"] != STACK_CONTRACT_SCHEMA
        and expected_condition_id.startswith("outdoor_")
        and "rivermark_viewport_startup_winner_manifest"
        in freeze.get("frozen_assets", {})
    ):
        raise V6ContractError("outdoor qualification requires v3 runtime attestation")
    expected_profile = (
        "module2_causal_obstacle_outdoor"
        if expected_condition_id.startswith("outdoor_")
        else "module2_causal_obstacle_active"
    )
    if (
        payload.get("arm") != "M3"
        or payload.get("startup_profile") != expected_profile
        or isinstance(payload.get("domain"), bool)
        or not isinstance(payload.get("domain"), int)
        or not 0 <= payload["domain"] <= 232
        or not all(
            isinstance(payload.get(name), int)
            and not isinstance(payload.get(name), bool)
            and payload[name] > 0
            for name in ("pid", "pgid", "start_ticks")
        )
        or not isinstance(payload.get("boot_id"), str)
        or not payload["boot_id"]
    ):
        raise V6ContractError("Pilot stack contract runtime fields are invalid")
    repositories = freeze["repositories"]
    if not (
        payload.get("integration_head") == repositories["integration"]["head"]
        and payload.get("module2_head") == repositories["module2"]["head"]
        and payload.get("module3_head") == repositories["module3"]["head"]
        and payload.get("integration_dirty") is False
        and payload.get("module2_dirty") is False
        and payload.get("module3_dirty") is False
        and payload.get("driver_version") == freeze["driver_version"]
        and payload.get("kernel_release") == freeze["kernel_release"]
    ):
        raise V6ContractError("Pilot stack contract frozen tuple mismatch")
    if payload["schema"] in {STACK_CONTRACT_SCHEMA_V2, STACK_CONTRACT_SCHEMA}:
        if any(
            payload.get(f"{name}_tree") != repositories[name].get("tree")
            for name in ("integration", "module2", "module3")
        ):
            raise V6ContractError("Pilot stack contract repository tree mismatch")
        if payload["schema"] == STACK_CONTRACT_SCHEMA and (
            not Path(str(payload.get("contract_path", ""))).is_absolute()
            or Path(str(payload.get("contract_path", ""))).name
            != "stack.contract.json"
        ):
            raise V6ContractError("stack contract source path is invalid")
        if expected_condition_id.startswith("outdoor_"):
            if not (
                payload["schema"] == STACK_CONTRACT_SCHEMA
                and payload.get("viewport_arm") == "B"
                and payload.get("disable_viewport_updates_requested") is True
                and payload.get("disable_viewport_updates_observed") is True
                and payload.get("viewport_startup_attestation_sha256")
                == payload.get("viewport_runtime_attestation_sha256")
            ):
                raise V6ContractError("outdoor viewport arm attestation mismatch")
            _validate_viewport_runtime_snapshot(
                path,
                payload,
                expected_condition_id=expected_condition_id,
                freeze=freeze,
            )
        elif not (
            payload.get("viewport_arm") == "not_applicable"
            and payload.get("disable_viewport_updates_requested") == "not_applicable"
            and payload.get("disable_viewport_updates_observed") == "not_applicable"
            and payload.get("viewport_startup_attestation_sha256") == "not_applicable"
            and payload.get("viewport_runtime_attestation_path") == "not_applicable"
            and payload.get("viewport_runtime_attestation_sha256") == "not_applicable"
            and payload.get("viewport_instance_uuid") == "not_applicable"
            and payload.get("viewport_winner_manifest_sha256") == "not_applicable"
        ):
            raise V6ContractError("indoor viewport fields must be not_applicable")
    expected_selector = (
        Path(repositories["module3"]["path"])
        / "scripts"
        / (
            "run_v6_rivermark.sh"
            if expected_condition_id.startswith("outdoor_")
            else "run_v6_kujiale_low_obstacles.sh"
        )
    ).resolve()
    if (
        Path(str(payload.get("t2_selector_path", ""))).resolve() != expected_selector
        or payload.get("t2_selector_sha256") != _file_sha256(expected_selector)
        or not Path(str(payload.get("episode_sequence_path", ""))).is_absolute()
    ):
        raise V6ContractError("Pilot T2 selector/sequence attestation mismatch")
    return payload, _stack_tuple_digest(payload)


def validate_condition_stack_contract(
    path: str | Path, *, expected_condition_id: str
) -> dict[str, Any]:
    contract_path = Path(path).expanduser().resolve()
    try:
        payload = json.loads(contract_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise V6ContractError(f"condition stack contract is unreadable: {exc}") from exc
    payload = dict(_mapping(payload, "condition_stack_contract"))
    keys = _stack_contract_keys(payload)
    _require_exact_keys(payload, keys, "condition_stack_contract")
    if payload["condition_id"] != expected_condition_id:
        raise V6ContractError("condition stack contract condition mismatch")
    if payload["condition_id"] != f"{payload['scene']}_{payload['condition']}":
        raise V6ContractError("condition stack contract identity is inconsistent")
    if payload["scene"] not in {"indoor", "outdoor"} or payload[
        "condition"
    ] not in {"static", "dynamic", "appearance"}:
        raise V6ContractError("condition stack contract scene/condition is invalid")
    if payload["arm"] not in {"M0", "M1", "M2", "M3"}:
        raise V6ContractError("condition stack contract arm is invalid")
    for name in ("pid", "pgid", "start_ticks"):
        value = payload[name]
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise V6ContractError(f"condition stack contract {name} is invalid")
    if (
        isinstance(payload["domain"], bool)
        or not isinstance(payload["domain"], int)
        or not 0 <= payload["domain"] <= 232
        or not isinstance(payload["startup_profile"], str)
        or not payload["startup_profile"]
    ):
        raise V6ContractError("condition stack contract runtime identity is invalid")
    try:
        boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        stat = Path(f"/proc/{payload['pid']}/stat").read_text().strip()
    except OSError as exc:
        raise V6ContractError("condition stack process is not live") from exc
    try:
        fields = stat.rsplit(")", 1)[1].split()
        state = fields[0]
        actual_pgid = int(fields[2])
        actual_start_ticks = int(fields[19])
    except (IndexError, ValueError) as exc:
        raise V6ContractError("condition stack process identity is malformed") from exc
    if state == "Z" or actual_pgid != payload["pgid"]:
        raise V6ContractError("condition stack process group is stale")
    if actual_start_ticks != payload["start_ticks"]:
        raise V6ContractError("condition stack process start_ticks is stale")
    if boot_id != payload["boot_id"]:
        raise V6ContractError("condition stack boot_id is stale")
    if payload["stack_session_id"] != _stack_session_id(payload):
        raise V6ContractError("condition stack session digest mismatch")
    return payload


def _evaluate_campaign(
    manifest: FormalCampaignManifest | IndoorCampaignManifest,
    *,
    indoor_only: bool,
) -> dict[str, Any]:
    condition_results: list[dict[str, Any]] = []
    total_strict = 0
    total_valid = 0
    total_present = 0
    blockers: list[str] = []
    continuation = (
        manifest.freeze.get("indoor_continuation")
        if indoor_only
        and manifest.freeze.get("indoor_continuation", {}).get("schema")
        == INDOOR_CONTINUATION_SCHEMA_VERSION
        else None
    )
    for condition in manifest.conditions:
        expected_robot_hash, expected_nav2_hash, expected_runtime_hashes = (
            _expected_scenario_runtime_hashes(condition)
        )
        runs: list[dict[str, Any]] = []
        next_run_index: int | None = None
        condition_blockers: list[str] = []
        static_path_deviations: list[float] = []
        stack_session_ids: set[str] = set()
        present_run_indices: set[int] = set()
        reset_generations: dict[int, int] = {}
        stack_sequences: dict[int, int] = {}
        startup_kinds: dict[int, str] = {}
        strict_half_group = (
            len(manifest.conditions) == 3
            and continuation is None
            and all(
                "tree" in entry
                for entry in manifest.freeze["repositories"].values()
            )
        )
        for run_index, identity in enumerate(
            condition.episode_identities, start=1
        ):
            seed = int(identity["seed"])
            root = (
                Path(continuation["preserved_run"]["root"])
                if continuation
                and condition.condition_id == "indoor_static"
                and run_index == 1
                else (
                    condition.output_directory
                    / condition.scenario_id
                    / f"run-{run_index:04d}-seed-{seed}"
                )
            )
            if not root.exists():
                if next_run_index is None:
                    next_run_index = run_index
                runs.append(
                    {"run_index": run_index, "seed": seed, "status": "pending"}
                )
                continue
            total_present += 1
            present_run_indices.add(run_index)
            summary_path = root / "run_summary.json"
            manifest_path = root / "run_manifest.json"
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                episode = json.loads(manifest_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                status = "invalid_evidence"
                detail = f"{type(exc).__name__}:{exc}"
                condition_blockers.append(f"run-{run_index}:invalid_evidence")
            else:
                if (
                    continuation
                    and condition.condition_id == "indoor_static"
                    and run_index == 1
                ):
                    primary_evidence = {"strict_success": False}
                    primary_evidence_ok = True
                    primary_evidence_detail = ""
                else:
                    try:
                        from robot_experiments.experiment_runner import (
                            validate_recorded_run_evidence,
                        )

                        primary_evidence = validate_recorded_run_evidence(
                            root,
                            summary,
                            episode,
                            scene=condition.scene,
                            route_guided=True,
                            route_prior_required=True,
                            expected_leg_count=5,
                            require_strict_success=False,
                            cognitive_admission_required=True,
                        )
                        primary_evidence_ok = True
                        primary_evidence_detail = ""
                    except (ConfigurationError, ImportError) as exc:
                        primary_evidence = {"strict_success": False}
                        primary_evidence_ok = False
                        primary_evidence_detail = f"{type(exc).__name__}:{exc}"
                episode_stack_session = episode.get("stack_session_id")
                summary_stack_session = summary.get("stack_session_id")
                reset_receipt = episode.get("reset_receipt", {})
                reset_generation = (
                    reset_receipt.get("generation")
                    if isinstance(reset_receipt, Mapping)
                    else None
                )
                reset_identity_ok = bool(
                    isinstance(reset_generation, int)
                    and not isinstance(reset_generation, bool)
                    and reset_generation > 0
                )
                if reset_identity_ok:
                    reset_generations[run_index] = reset_generation
                sequence_receipt = episode.get("stack_episode_receipt", {})
                sequence = (
                    sequence_receipt.get("sequence")
                    if isinstance(sequence_receipt, Mapping)
                    else None
                )
                if isinstance(sequence, int) and not isinstance(sequence, bool):
                    stack_sequences[run_index] = sequence
                startup_kind = (
                    sequence_receipt.get("startup_kind")
                    if isinstance(sequence_receipt, Mapping)
                    else None
                )
                if isinstance(startup_kind, str):
                    startup_kinds[run_index] = startup_kind
                expected_freeze_digest = (
                    continuation["parent_manifest"]["freeze_digest"]
                    if continuation
                    and condition.condition_id == "indoor_static"
                    and run_index == 1
                    else manifest.freeze_digest
                )
                continuation_sequence_ok = True
                if continuation and condition.condition_id == "indoor_static":
                    expected_sequence = 1 if run_index in {1, 2} else run_index - 1
                    expected_generation = 2 if run_index in {1, 2} else run_index
                    parent_session = continuation["preserved_run"]["stack_session_id"]
                    continuation_sequence_ok = bool(
                        sequence == expected_sequence
                        and reset_generation == expected_generation
                        and (
                            episode_stack_session == parent_session
                            if run_index == 1
                            else episode_stack_session != parent_session
                        )
                    )
                half_stack_binding_ok = True
                if strict_half_group:
                    try:
                        stack_snapshot_path = root / "stack_contract.json"
                        stack_snapshot, _stack_digest = _load_stack_contract_snapshot(
                            stack_snapshot_path,
                            expected_condition_id=condition.condition_id,
                            freeze=manifest.freeze,
                        )
                        stack_snapshot_sha256 = _file_sha256(stack_snapshot_path)
                    except (OSError, V6ContractError):
                        half_stack_binding_ok = False
                    else:
                        summary_attestation = summary.get(
                            "condition_stack_attestation", {}
                        )
                        half_stack_binding_ok = bool(
                            isinstance(summary_attestation, Mapping)
                            and sequence_receipt.get("sequence") == run_index
                            and sequence_receipt.get("startup_kind")
                            == ("cold" if run_index == 1 else "hot_reset")
                            and reset_generation == run_index + 1
                            and summary.get("reset_receipt") == reset_receipt
                            and sequence_receipt.get("stack_session_id")
                            == stack_snapshot["stack_session_id"]
                            == episode_stack_session
                            and sequence_receipt.get(
                                "condition_stack_contract_path"
                            ) == stack_snapshot.get("contract_path")
                            and sequence_receipt.get(
                                "condition_stack_contract_sha256"
                            ) == stack_snapshot_sha256
                            and summary_attestation.get(
                                "condition_stack_contract_path"
                            ) == stack_snapshot.get("contract_path")
                            and summary_attestation.get(
                                "condition_stack_contract_sha256"
                            ) == stack_snapshot_sha256
                            and summary_attestation.get("stack_episode_receipt")
                            == sequence_receipt
                        )
                stack_identity_ok = bool(
                    episode.get("condition_stack_id") == condition.condition_id
                    and summary.get("condition_stack_id") == condition.condition_id
                    and isinstance(episode_stack_session, str)
                    and bool(episode_stack_session)
                    and summary_stack_session == episode_stack_session
                    and reset_identity_ok
                    and episode.get("formal_freeze_digest")
                    == expected_freeze_digest
                    and summary.get("formal_freeze_digest")
                    == expected_freeze_digest
                    and continuation_sequence_ok
                    and half_stack_binding_ok
                )
                if stack_identity_ok:
                    stack_session_ids.add(episode_stack_session)
                identity_ok = bool(
                    episode.get("run_index") == run_index
                    and episode.get("random_seed") == seed
                    and episode.get("scenario_id") == condition.scenario_id
                    and episode.get("condition_id")
                    == identity.get("condition_id")
                    and episode.get("dynamic_selection", {}).get("case_id")
                    == identity.get("dynamic_case_id")
                    and episode.get("dynamic_selection", {}).get("variant_id")
                    == identity.get("dynamic_variant_id")
                    and episode.get("appearance", {}).get("profile_id")
                    == identity.get("appearance_profile_id")
                    and stack_identity_ok
                )
                provenance = episode.get("provenance", {})
                source_config_ok = bool(
                    episode.get("robot_config_hash") == expected_robot_hash
                    and episode.get("nav2_config_hash") == expected_nav2_hash
                    and episode.get("scenario_runtime_hashes")
                    == expected_runtime_hashes
                    and isinstance(provenance, Mapping)
                    and provenance.get("git_head")
                    == (
                        manifest.freeze["qualification_tooling_promotion"]["from_head"]
                        if continuation
                        and condition.condition_id == "indoor_static"
                        and run_index == 1
                        else manifest.freeze["repositories"]["module3"]["head"]
                    )
                    and provenance.get("git_tracked_dirty") is False
                )
                static_metric = None
                static_metric_ok = True
                if condition.category == "static":
                    summary_metric = summary.get("path_deviation_percent")
                    episode_metric = episode.get("metrics", {}).get(
                        "path_deviation_percent"
                    )
                    metric_pair_is_null = (
                        summary_metric is None and episode_metric is None
                    )
                    metric_pair_is_finite_equal = bool(
                        isinstance(summary_metric, (int, float))
                        and not isinstance(summary_metric, bool)
                        and math.isfinite(float(summary_metric))
                        and isinstance(episode_metric, (int, float))
                        and not isinstance(episode_metric, bool)
                        and math.isfinite(float(episode_metric))
                        and float(episode_metric) == float(summary_metric)
                    )
                    static_metric_ok = bool(
                        episode.get("optimal_reference_hash")
                        == expected_runtime_hashes.get("optimal_reference")
                        and expected_runtime_hashes.get("optimal_reference") is not None
                        and (metric_pair_is_null or metric_pair_is_finite_equal)
                    )
                    if static_metric_ok and metric_pair_is_finite_equal:
                        static_metric = float(summary_metric)
                valid = bool(
                    identity_ok
                    and source_config_ok
                    and static_metric_ok
                    and primary_evidence_ok
                    and summary.get("episode_validity", {}).get("valid") is True
                    and summary.get("checksums_verified") is True
                    and summary.get("final_trial_metric_gate", {}).get("passed")
                    is True
                    and _checksums_verified(root)
                )
                strict = bool(
                    valid
                    and primary_evidence.get("strict_success") is True
                    and (
                        condition.category != "static"
                        or (
                            static_metric is not None
                            and static_metric < 20.0
                        )
                    )
                )
                if strict:
                    status = "strict_success"
                    total_strict += 1
                    if static_metric is not None:
                        static_path_deviations.append(static_metric)
                elif valid:
                    status = "product_failure"
                else:
                    status = "invalid_evidence"
                    condition_blockers.append(f"run-{run_index}:invalid_evidence")
                if valid:
                    total_valid += 1
                detail = primary_evidence_detail
            runs.append(
                {
                    "run_index": run_index,
                    "seed": seed,
                    "status": status,
                    "detail": detail,
                    "root": str(root),
                    **(
                        {"path_deviation_percent": static_metric}
                        if static_metric is not None
                        else {}
                    ),
                }
            )
        if continuation and condition.condition_id == "indoor_static":
            parent_session = continuation["preserved_run"]["stack_session_id"]
            successor_sessions = stack_session_ids - {parent_session}
            if parent_session not in stack_session_ids or len(successor_sessions) > 1:
                condition_blockers.append("continuation_stack_session_mismatch")
            if present_run_indices != set(reset_generations) or present_run_indices != set(
                stack_sequences
            ):
                condition_blockers.append("continuation_sequence_or_generation_missing")
            elif present_run_indices:
                ordered_indices = sorted(present_run_indices)
                if ordered_indices != list(range(1, ordered_indices[-1] + 1)) or any(
                    stack_sequences[index] != (1 if index in {1, 2} else index - 1)
                    or reset_generations[index] != (2 if index in {1, 2} else index)
                    for index in ordered_indices
                ):
                    condition_blockers.append("continuation_sequence_or_generation_invalid")
        else:
            if len(stack_session_ids) > 1:
                condition_blockers.append("stack_session_mismatch")
            if strict_half_group and (
                present_run_indices != set(reset_generations)
                or present_run_indices != set(stack_sequences)
                or present_run_indices != set(startup_kinds)
            ):
                condition_blockers.append("half_stack_sequence_or_startup_missing")
            elif strict_half_group and present_run_indices:
                ordered_indices = sorted(present_run_indices)
                if ordered_indices != list(range(1, ordered_indices[-1] + 1)) or any(
                    stack_sequences[index] != index
                    or reset_generations[index] != index + 1
                    or startup_kinds[index]
                    != ("cold" if index == 1 else "hot_reset")
                    for index in ordered_indices
                ):
                    condition_blockers.append("half_stack_sequence_or_startup_invalid")
            elif present_run_indices != set(reset_generations):
                condition_blockers.append("reset_generation_missing")
            elif present_run_indices:
                ordered_indices = sorted(present_run_indices)
                base_generation = reset_generations[ordered_indices[0]]
                if ordered_indices != list(range(1, ordered_indices[-1] + 1)) or any(
                    reset_generations[index] != base_generation + index - 1
                    for index in ordered_indices
                ):
                    condition_blockers.append("reset_generation_discontinuous")
        strict_successes = sum(
            run["status"] == "strict_success" for run in runs
        )
        valid_episodes = sum(
            run["status"] in {"strict_success", "product_failure"}
            for run in runs
        )
        required_successes = (
            INDOOR_SUCCESS_THRESHOLDS[condition.condition_id]
            if indoor_only
            else FORMAL_SUCCESS_THRESHOLDS[condition.condition_id]
        )
        failure_budget = FORMAL_RUNS_PER_CONDITION - required_successes
        if not any(
            "invalid_evidence" in blocker for blocker in condition_blockers
        ):
            failures = valid_episodes - strict_successes
            if (
                failures > failure_budget
                or strict_successes
                + (FORMAL_RUNS_PER_CONDITION - valid_episodes)
                < required_successes
            ):
                condition_blockers.append("early_fail_unreachable")
        path_statistics = None
        if condition.category == "static":
            ordered = sorted(static_path_deviations)
            path_statistics = {
                "count": len(ordered),
                "mean": statistics.mean(ordered) if ordered else None,
                "p50": statistics.median(ordered) if ordered else None,
                "p95": (
                    ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
                    if ordered
                    else None
                ),
                "max": max(ordered) if ordered else None,
                "threshold_exclusive": 20.0,
            }
        condition_pass = bool(
            valid_episodes == FORMAL_RUNS_PER_CONDITION
            and strict_successes >= required_successes
            and not condition_blockers
            and (
                condition.category != "static"
                or (
                    path_statistics is not None
                    and path_statistics["count"] == strict_successes
                    and path_statistics["max"] is not None
                    and path_statistics["max"] < 20.0
                )
            )
        )
        blockers.extend(
            f"{condition.condition_id}:{item}" for item in condition_blockers
        )
        condition_result = {
            "id": condition.condition_id,
            "strict_successes": strict_successes,
            "valid_episodes": valid_episodes,
            "next_run_index": None if condition_blockers else next_run_index,
            "blockers": condition_blockers,
            "stack_session_id": (
                (
                    next(iter(stack_session_ids - {
                        continuation["preserved_run"]["stack_session_id"]
                    }))
                    if len(stack_session_ids - {
                        continuation["preserved_run"]["stack_session_id"]
                    }) == 1
                    else None
                )
                if continuation and condition.condition_id == "indoor_static"
                else next(iter(stack_session_ids))
                if len(stack_session_ids) == 1
                else None
            ),
            "reset_generation_base": (
                reset_generations.get(2)
                if continuation and condition.condition_id == "indoor_static"
                else reset_generations.get(1)
            ),
            "runs": runs,
        }
        condition_result.update({
            "product_failures": valid_episodes - strict_successes,
            "required_strict_successes": required_successes,
            "failure_budget": failure_budget,
            "qualification": "PASS" if condition_pass else "INCOMPLETE",
        })
        if path_statistics is not None:
            condition_result["path_deviation_percent"] = path_statistics
        if indoor_only:
            if continuation and condition.condition_id == "indoor_static":
                successor_sessions = stack_session_ids - {
                    continuation["preserved_run"]["stack_session_id"]
                }
                condition_result["continuation"] = {
                    "parent_failure_count": 1,
                    "collision_count": 1,
                    "cold_starts": 2,
                    "hot_resets": 18,
                    "epochs": continuation["epochs"],
                    "parent_stack_session_id": continuation["preserved_run"][
                        "stack_session_id"
                    ],
                    "successor_stack_session_id": (
                        next(iter(successor_sessions))
                        if len(successor_sessions) == 1
                        else None
                    ),
                }
        condition_results.append(condition_result)
    expected_episodes = len(manifest.conditions) * FORMAL_RUNS_PER_CONDITION
    result = {
        "schema_version": (
            INDOOR_CAMPAIGN_SCHEMA_VERSION
            if indoor_only
            else FORMAL_CAMPAIGN_SCHEMA_VERSION
        ),
        "formal_qualification": (
            NOT_QUALIFIED
            if indoor_only
            else (
                "PASS"
                if (
                    manifest.authorization == FORMAL_EXECUTION_AUTHORIZED
                    and all(condition["qualification"] == "PASS" for condition in condition_results)
                    and total_valid == expected_episodes
                    and not blockers
                )
                else "INCOMPLETE"
            )
        ),
        "expected_episodes": expected_episodes,
        "present_episodes": total_present,
        "valid_episodes": total_valid,
        "strict_successes": total_strict,
        "blockers": blockers,
        "conditions": condition_results,
    }
    if indoor_only:
        conditions_pass = all(
            row["qualification"] == "PASS" for row in condition_results
        )
        if conditions_pass and total_valid == expected_episodes and not blockers:
            if continuation:
                result["qualification"] = (
                    "INDOOR_QUALIFICATION_PASS_WITH_STATIC_CONTINUATION"
                )
                result["campaign_status"] = (
                    "INDOOR_QUALIFICATION_PASS_WITH_STATIC_CONTINUATION"
                )
                result["continuation"] = {
                    "static_result": "19/20 strict + 1 collision product failure",
                    "static_cold_starts": 2,
                    "static_hot_resets": 18,
                    "epochs": continuation["epochs"],
                    "parent_freeze_digest": continuation["parent_manifest"][
                        "freeze_digest"
                    ],
                    "successor_freeze_digest": manifest.freeze_digest,
                }
            else:
                result["qualification"] = "INDOOR_QUALIFICATION_PASS"
                result["campaign_status"] = "INDOOR_QUALIFICATION_PASS"
        elif any("early_fail_unreachable" in blocker for blocker in blockers):
            result["qualification"] = "INCOMPLETE"
            result["campaign_status"] = "EARLY_FAIL_UNREACHABLE"
        elif blockers:
            result["qualification"] = "INCOMPLETE"
            result["campaign_status"] = "STOP_INVALID"
        else:
            result["qualification"] = "INCOMPLETE"
            result["campaign_status"] = "IN_PROGRESS"
    else:
        result["execution_authorization"] = manifest.authorization
    return result


def evaluate_formal_campaign(manifest: FormalCampaignManifest) -> dict[str, Any]:
    return _evaluate_campaign(manifest, indoor_only=False)


def evaluate_indoor_campaign(manifest: IndoorCampaignManifest) -> dict[str, Any]:
    if manifest.freeze.get("indoor_continuation", {}).get("schema") == (
        INDOOR_CONTINUATION_SCHEMA_VERSION
    ):
        _validate_indoor_continuation_contract(manifest)
    return _evaluate_campaign(manifest, indoor_only=True)


def _outdoor_as_authorized_formal(
    manifest: OutdoorCampaignManifest,
) -> FormalCampaignManifest:
    return FormalCampaignManifest(
        path=manifest.path,
        authorization=FORMAL_EXECUTION_AUTHORIZED,
        runner_entrypoint=manifest.runner_entrypoint,
        freeze=manifest.freeze,
        freeze_digest=manifest.freeze_digest,
        pilot_freeze_provenance=manifest.pilot_freeze_provenance,
        conditions=manifest.conditions,
    )


def evaluate_outdoor_campaign(manifest: OutdoorCampaignManifest) -> dict[str, Any]:
    result = _evaluate_campaign(_outdoor_as_authorized_formal(manifest), indoor_only=False)
    result["schema_version"] = OUTDOOR_CAMPAIGN_SCHEMA_VERSION
    result["formal_qualification"] = NOT_QUALIFIED
    result.pop("execution_authorization", None)
    if (
        result["valid_episodes"] == 60
        and not result["blockers"]
        and all(row["qualification"] == "PASS" for row in result["conditions"])
    ):
        result["qualification"] = "OUTDOOR_QUALIFICATION_PASS"
        result["campaign_status"] = "OUTDOOR_QUALIFICATION_PASS"
    elif any("early_fail_unreachable" in blocker for blocker in result["blockers"]):
        result["qualification"] = "INCOMPLETE"
        result["campaign_status"] = "EARLY_FAIL_UNREACHABLE"
    elif result["blockers"]:
        result["qualification"] = "INCOMPLETE"
        result["campaign_status"] = "STOP_INVALID"
    else:
        result["qualification"] = "INCOMPLETE"
        result["campaign_status"] = "IN_PROGRESS"
    return result


def outdoor_dispatch_plan(
    manifest: OutdoorCampaignManifest, aggregate: Mapping[str, Any]
) -> list[dict[str, Any]]:
    plans = formal_dispatch_plan(manifest, aggregate)
    by_condition = {row["condition_id"]: row for row in plans}
    aggregates = {row["id"]: row for row in aggregate["conditions"]}
    for condition_id in OUTDOOR_CONDITION_IDS:
        if aggregates[condition_id]["qualification"] == "PASS":
            continue
        plan = by_condition.get(condition_id)
        return [plan] if plan is not None else []
    return []


def formal_dispatch_plan(
    manifest: FormalCampaignManifest | IndoorCampaignManifest,
    aggregate: Mapping[str, Any],
) -> list[dict[str, Any]]:
    script = manifest.runner_entrypoint
    plans: list[dict[str, Any]] = []
    by_id = {row["id"]: row for row in aggregate["conditions"]}
    for condition in manifest.conditions:
        run_index = by_id[condition.condition_id]["next_run_index"]
        if run_index is None:
            continue
        command = [
            str(script),
            str(condition.scenario_file),
            str(condition.output_directory),
            *condition.runner_arguments,
            "record_evidence:=true",
            "record_bag:=true",
            "fail_stop:=true",
            f"run_indices:={run_index}",
            "resume:=false",
        ]
        plans.append(
            {
                "condition_id": condition.condition_id,
                "run_index": run_index,
                "seed": condition.episode_identities[run_index - 1]["seed"],
                "episode_identity": dict(
                    condition.episode_identities[run_index - 1]
                ),
                "stack_session": condition.condition_id,
                "stack_boundary": "cold" if run_index == 1 else "hot_reset",
                "requires_existing_condition_stack": True,
                "condition_stack_contract_required": True,
                "command": command,
                "command_text": shlex.join(command),
            }
        )
    return plans


def indoor_dispatch_plan(
    manifest: IndoorCampaignManifest, aggregate: Mapping[str, Any]
) -> list[dict[str, Any]]:
    continuation = manifest.freeze.get("indoor_continuation")
    if isinstance(continuation, Mapping) and continuation.get("schema") == (
        INDOOR_CONTINUATION_SCHEMA_VERSION
    ):
        static_row = next(
            row for row in aggregate["conditions"] if row["id"] == "indoor_static"
        )
        next_static = static_row.get("next_run_index")
        if next_static is not None and (next_static == 1 or next_static > 20):
            raise V6ContractError(
                "indoor continuation cannot retry run1 or dispatch beyond run20"
            )
    plans = formal_dispatch_plan(manifest, aggregate)
    if isinstance(continuation, Mapping) and continuation.get("schema") == (
        INDOOR_CONTINUATION_SCHEMA_VERSION
    ):
        for row in plans:
            if row["condition_id"] != "indoor_static":
                continue
            if row["run_index"] == 1 or row["run_index"] > 20:
                raise V6ContractError(
                    "indoor continuation cannot retry run1 or dispatch beyond run20"
                )
            row["stack_boundary"] = (
                "continuation_cold_restart"
                if row["run_index"] == 2
                else "hot_reset"
            )
            row["continuation_epoch"] = 2
            row["preserved_parent_run"] = 1
    by_condition = {row["condition_id"]: row for row in plans}
    aggregate_by_condition = {row["id"]: row for row in aggregate["conditions"]}
    for condition_id in INDOOR_CONDITION_IDS:
        condition = aggregate_by_condition[condition_id]
        if condition["qualification"] == "PASS":
            continue
        plan = by_condition.get(condition_id)
        return [plan] if plan is not None else []
    return []


def execute_formal_campaign(
    manifest: FormalCampaignManifest,
    *,
    condition_stack_id: str,
    condition_stack_contract: str | Path,
) -> dict[str, Any]:
    if manifest.authorization != FORMAL_EXECUTION_AUTHORIZED:
        raise V6ContractError(
            "formal execution refused: manifest is NOT_AUTHORIZED"
        )
    if manifest.pilot_freeze_provenance is None:
        raise V6ContractError(
            "formal execution requires complete Pilot freeze provenance"
        )
    condition_ids = {condition.condition_id for condition in manifest.conditions}
    if condition_stack_id not in condition_ids:
        raise V6ContractError(
            f"unknown formal condition stack: {condition_stack_id}"
        )
    aggregate = evaluate_formal_campaign(manifest)
    if aggregate["blockers"]:
        raise V6ContractError(
            "formal campaign blocked: " + ",".join(aggregate["blockers"])
        )
    selected = [
        row
        for row in formal_dispatch_plan(manifest, aggregate)
        if row["condition_id"] == condition_stack_id
    ]
    if len(selected) != 1:
        raise V6ContractError(
            f"formal condition has no unique pending episode: {condition_stack_id}"
        )
    row = selected[0]
    if row["stack_session"] != condition_stack_id:
        raise V6ContractError("formal dispatch stack-session mismatch")
    contract = validate_condition_stack_contract(
        condition_stack_contract,
        expected_condition_id=condition_stack_id,
    )
    contract, _contract_tuple_digest = _load_stack_contract_snapshot(
        Path(condition_stack_contract).expanduser().resolve(),
        expected_condition_id=condition_stack_id,
        freeze=manifest.freeze,
    )
    freeze_repositories = manifest.freeze["repositories"]
    for contract_key, repository in (
        ("integration_head", "integration"),
        ("module2_head", "module2"),
        ("module3_head", "module3"),
    ):
        if contract[contract_key] != freeze_repositories[repository]["head"]:
            raise V6ContractError("formal condition stack repository head mismatch")
    if (
        contract["driver_version"] != manifest.freeze["driver_version"]
        or contract["kernel_release"] != manifest.freeze["kernel_release"]
    ):
        raise V6ContractError("formal condition stack system freeze mismatch")
    expected_profile = (
        "module2_causal_obstacle_outdoor"
        if condition_stack_id.startswith("outdoor_")
        else "module2_causal_obstacle_active"
    )
    if contract["arm"] != "M3" or contract["startup_profile"] != expected_profile:
        raise V6ContractError("formal condition stack is not the M3 active profile")
    if os.environ.get("ROS_DOMAIN_ID") != str(contract["domain"]):
        raise V6ContractError("formal condition stack ROS domain mismatch")
    selected_aggregate = next(
        condition
        for condition in aggregate["conditions"]
        if condition["id"] == condition_stack_id
    )
    recorded_stack_session = selected_aggregate.get("stack_session_id")
    if (
        recorded_stack_session
        and recorded_stack_session != contract["stack_session_id"]
    ):
        raise V6ContractError(
            "live condition stack session differs from recorded episodes"
        )
    before_completed = {
        (condition["id"], run["run_index"], run["status"])
        for condition in aggregate["conditions"]
        for run in condition["runs"]
        if run["status"] != "pending"
    }
    command = [
        *row["command"],
        f"condition_stack_id:={condition_stack_id}",
        f"stack_session_id:={contract['stack_session_id']}",
        f"formal_freeze_digest:={manifest.freeze_digest}",
        f"condition_stack_contract_path:={Path(condition_stack_contract).resolve()}",
    ]
    # The caller owns the already-running matching T1/T2 stack.  Dispatch one
    # episode only, then return so a stack switch can never occur implicitly.
    subprocess.run(command, check=True)
    after = evaluate_formal_campaign(manifest)
    if after["blockers"]:
        raise V6ContractError(
            "formal campaign blocked after dispatch: "
            + ",".join(after["blockers"])
        )
    after_completed = {
        (condition["id"], run["run_index"], run["status"])
        for condition in after["conditions"]
        for run in condition["runs"]
        if run["status"] != "pending"
    }
    added = after_completed - before_completed
    if (
        before_completed - after_completed
        or len(added) != 1
        or next(iter(added))[:2] != (condition_stack_id, row["run_index"])
        or next(iter(added))[2] not in {"strict_success", "product_failure"}
    ):
        raise V6ContractError(
            "formal episode did not add exactly one valid target run"
        )
    return after


def execute_indoor_campaign(
    manifest: IndoorCampaignManifest,
    *,
    condition_stack_id: str,
    condition_stack_contract: str | Path,
) -> dict[str, Any]:
    continuation = manifest.freeze.get("indoor_continuation")
    if isinstance(continuation, Mapping) and continuation.get("schema") == (
        INDOOR_CONTINUATION_SCHEMA_VERSION
    ):
        _validate_indoor_continuation_contract(manifest)
    else:
        hardening = manifest.freeze.get("evaluator_hardening_promotion")
        if hardening is not None:
            provenance = manifest.pilot_freeze_provenance
            _validate_evaluator_hardening_chain(
                hardening,
                freeze=manifest.freeze,
                pilot_manifest_sha256=provenance["pilot_manifest"]["sha256"],
                pilot_aggregate_sha256=provenance["pilot_aggregate"]["sha256"],
            )
        else:
            promotion = manifest.freeze.get("validator_only_head_promotion")
            if promotion is not None:
                _validate_validator_only_head_promotion(
                    promotion, freeze=manifest.freeze
                )
    if condition_stack_id not in INDOOR_CONDITION_IDS:
        raise V6ContractError(f"unknown indoor condition stack: {condition_stack_id}")
    aggregate = evaluate_indoor_campaign(manifest)
    if aggregate["blockers"]:
        raise V6ContractError(
            "indoor campaign blocked: " + ",".join(aggregate["blockers"])
        )
    selected = [
        row
        for row in indoor_dispatch_plan(manifest, aggregate)
        if row["condition_id"] == condition_stack_id
    ]
    if len(selected) != 1:
        raise V6ContractError(
            f"indoor condition has no unique pending episode: {condition_stack_id}"
        )
    row = selected[0]
    validate_condition_stack_contract(
        condition_stack_contract, expected_condition_id=condition_stack_id
    )
    contract, _tuple_digest = _load_stack_contract_snapshot(
        Path(condition_stack_contract).expanduser().resolve(),
        expected_condition_id=condition_stack_id,
        freeze=manifest.freeze,
    )
    repositories = manifest.freeze["repositories"]
    for contract_key, repository in (
        ("integration_head", "integration"),
        ("module2_head", "module2"),
        ("module3_head", "module3"),
    ):
        if contract[contract_key] != repositories[repository]["head"]:
            raise V6ContractError("indoor condition stack repository head mismatch")
    if (
        contract["driver_version"] != manifest.freeze["driver_version"]
        or contract["kernel_release"] != manifest.freeze["kernel_release"]
        or contract["arm"] != "M3"
        or contract["startup_profile"] != "module2_causal_obstacle_active"
    ):
        raise V6ContractError("indoor condition stack system/profile mismatch")
    if os.environ.get("ROS_DOMAIN_ID") != str(contract["domain"]):
        raise V6ContractError("indoor condition stack ROS domain mismatch")
    frozen_spawn = Path(
        manifest.freeze["physical_contracts"][condition_stack_id][
            "spawn_manifest"
        ]["path"]
    )
    configured_spawn = os.environ.get("ISAAC_NAV_SPAWN_POSES", "").strip()
    effective_spawn = (
        Path(configured_spawn).expanduser()
        if configured_spawn
        else _canonical_indoor_spawn_manifest()
    )
    if not effective_spawn.is_absolute() or effective_spawn.resolve() != frozen_spawn:
        raise V6ContractError("indoor spawn manifest differs from frozen contract")
    selected_aggregate = next(
        condition
        for condition in aggregate["conditions"]
        if condition["id"] == condition_stack_id
    )
    recorded_session = selected_aggregate.get("stack_session_id")
    if (
        isinstance(continuation, Mapping)
        and condition_stack_id == "indoor_static"
        and contract["stack_session_id"]
        == continuation["preserved_run"]["stack_session_id"]
    ):
        raise V6ContractError(
            "indoor continuation epoch2 requires a new stack session"
        )
    if recorded_session and recorded_session != contract["stack_session_id"]:
        raise V6ContractError(
            "live indoor stack session differs from recorded episodes"
        )
    before_completed = {
        (condition["id"], run["run_index"], run["status"])
        for condition in aggregate["conditions"]
        for run in condition["runs"]
        if run["status"] != "pending"
    }
    command = [
        *row["command"],
        f"condition_stack_id:={condition_stack_id}",
        f"stack_session_id:={contract['stack_session_id']}",
        f"formal_freeze_digest:={manifest.freeze_digest}",
        f"condition_stack_contract_path:={Path(condition_stack_contract).resolve()}",
    ]
    subprocess.run(command, check=True)
    after = evaluate_indoor_campaign(manifest)
    if after["blockers"]:
        raise V6ContractError(
            "indoor campaign blocked after dispatch: " + ",".join(after["blockers"])
        )
    after_completed = {
        (condition["id"], run["run_index"], run["status"])
        for condition in after["conditions"]
        for run in condition["runs"]
        if run["status"] != "pending"
    }
    completed_run = next(
        run
        for condition in after["conditions"]
        if condition["id"] == condition_stack_id
        for run in condition["runs"]
        if run["run_index"] == row["run_index"]
    )
    if completed_run["status"] not in {"strict_success", "product_failure"}:
        raise V6ContractError("indoor episode did not produce a valid target run")
    expected = {
        (condition_stack_id, row["run_index"], completed_run["status"])
    }
    if before_completed - after_completed or after_completed - before_completed != expected:
        raise V6ContractError(
            "indoor episode did not add exactly one valid target run"
        )
    return after


def execute_outdoor_campaign(
    manifest: OutdoorCampaignManifest,
    *,
    condition_stack_id: str,
    condition_stack_contract: str | Path,
) -> dict[str, Any]:
    if condition_stack_id not in OUTDOOR_CONDITION_IDS:
        raise V6ContractError(f"unknown outdoor condition stack: {condition_stack_id}")
    aggregate = evaluate_outdoor_campaign(manifest)
    if aggregate["blockers"]:
        raise V6ContractError(
            "outdoor campaign blocked: " + ",".join(aggregate["blockers"])
        )
    plans = outdoor_dispatch_plan(manifest, aggregate)
    if len(plans) != 1 or plans[0]["condition_id"] != condition_stack_id:
        raise V6ContractError(
            f"outdoor condition is not the next unique episode: {condition_stack_id}"
        )
    contract, _digest = _load_stack_contract_snapshot(
        Path(condition_stack_contract).expanduser().resolve(),
        expected_condition_id=condition_stack_id,
        freeze=manifest.freeze,
    )
    if not (
        contract.get("viewport_arm") == "B"
        and contract.get("disable_viewport_updates_requested") is True
        and contract.get("disable_viewport_updates_observed") is True
        and contract.get("viewport_startup_attestation_sha256")
        == contract.get("viewport_runtime_attestation_sha256")
        and contract.get("viewport_winner_manifest_sha256")
        == manifest.freeze["frozen_assets"][
            "rivermark_viewport_startup_winner_manifest"
        ]["sha256"]
    ):
        raise V6ContractError("outdoor execution viewport winner attestation mismatch")
    execute_formal_campaign(
        _outdoor_as_authorized_formal(manifest),
        condition_stack_id=condition_stack_id,
        condition_stack_contract=condition_stack_contract,
    )
    return evaluate_outdoor_campaign(manifest)


def _half_evidence_digest(
    manifest: IndoorCampaignManifest | OutdoorCampaignManifest,
) -> str:
    rows = []
    for condition in manifest.conditions:
        for run_index, identity in enumerate(condition.episode_identities, start=1):
            root = (
                condition.output_directory
                / condition.scenario_id
                / f"run-{run_index:04d}-seed-{int(identity['seed'])}"
            )
            if root.is_symlink() or not root.is_dir():
                raise V6ContractError("combined half evidence is partial or symlinked")
            entries = {}
            for filename in (
                "run_summary.json", "run_manifest.json", "checksums.sha256",
                "stack_contract.json",
            ):
                evidence = root / filename
                if evidence.is_symlink() or not evidence.is_file():
                    raise V6ContractError("combined half evidence core file is missing")
                entries[filename] = _file_sha256(evidence)
            contract, tuple_digest = _load_stack_contract_snapshot(
                root / "stack_contract.json",
                expected_condition_id=condition.condition_id,
                freeze=manifest.freeze,
            )
            if contract.get("schema") != STACK_CONTRACT_SCHEMA:
                raise V6ContractError("combined halves reject old stack evidence")
            try:
                episode = json.loads((root / "run_manifest.json").read_text())
                summary = json.loads((root / "run_summary.json").read_text())
            except (OSError, json.JSONDecodeError) as exc:
                raise V6ContractError("combined half core evidence is unreadable") from exc
            receipt = _mapping(
                episode.get("stack_episode_receipt"),
                "combined.stack_episode_receipt",
            )
            attestation = _mapping(
                summary.get("condition_stack_attestation"),
                "combined.condition_stack_attestation",
            )
            if not (
                receipt.get("sequence") == run_index
                and receipt.get("startup_kind")
                == ("cold" if run_index == 1 else "hot_reset")
                and episode.get("reset_receipt", {}).get("generation")
                == run_index + 1
                and summary.get("reset_receipt") == episode.get("reset_receipt")
                and receipt.get("stack_session_id")
                == contract["stack_session_id"]
                == episode.get("stack_session_id")
                == summary.get("stack_session_id")
                and receipt.get("condition_stack_contract_path")
                == contract.get("contract_path")
                and receipt.get("condition_stack_contract_sha256")
                == entries["stack_contract.json"]
                and attestation.get("condition_stack_contract_path")
                == contract.get("contract_path")
                and attestation.get("condition_stack_contract_sha256")
                == entries["stack_contract.json"]
                and attestation.get("stack_episode_receipt") == receipt
            ):
                raise V6ContractError("combined half stack group binding mismatch")
            rows.append({
                "condition_id": condition.condition_id,
                "run_index": run_index,
                "seed": int(identity["seed"]),
                "root": str(root.resolve()),
                "files": entries,
                "stack_tuple_digest": tuple_digest,
            })
    if len(rows) != 60:
        raise V6ContractError("combined half must independently index 60 episodes")
    canonical = json.dumps(rows, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode()).hexdigest()


def combine_qualified_halves(
    indoor_manifest_path: str | Path,
    outdoor_manifest_path: str | Path,
) -> dict[str, Any]:
    indoor = load_indoor_campaign_manifest(indoor_manifest_path)
    outdoor = load_outdoor_campaign_manifest(outdoor_manifest_path)
    if (
        indoor.freeze.get("validator_only_head_promotion") is not None
        or indoor.freeze.get("indoor_continuation") is not None
    ):
        raise V6ContractError("combined halves require fresh same-tuple indoor evidence")
    indoor_result = evaluate_indoor_campaign(indoor)
    outdoor_result = evaluate_outdoor_campaign(outdoor)
    if indoor_result.get("qualification") != "INDOOR_QUALIFICATION_PASS":
        raise V6ContractError("combined halves require indoor PASS")
    if outdoor_result.get("qualification") != "OUTDOOR_QUALIFICATION_PASS":
        raise V6ContractError("combined halves require outdoor PASS")
    if not (
        indoor_result["present_episodes"] == indoor_result["valid_episodes"] == 60
        and outdoor_result["present_episodes"] == outdoor_result["valid_episodes"] == 60
    ):
        raise V6ContractError("combined halves reject partial or invalid evidence")
    indoor_repositories = indoor.freeze["repositories"]
    outdoor_repositories = outdoor.freeze["repositories"]
    if indoor_repositories != outdoor_repositories or any(
        "tree" not in row for row in indoor_repositories.values()
    ):
        raise V6ContractError("combined halves repository HEAD+tree tuple mismatch")
    for name, row in indoor_repositories.items():
        path = Path(row["path"])
        current = _repository_freeze_entry_with_tree(path)
        if current != row or _repository_tracked_dirty(path):
            raise V6ContractError(f"combined halves tracked provenance drift: {name}")
    if (
        indoor.freeze["driver_version"] != outdoor.freeze["driver_version"]
        or indoor.freeze["kernel_release"] != outdoor.freeze["kernel_release"]
    ):
        raise V6ContractError("combined halves driver/kernel policy mismatch")
    for name in COMMON_HALF_ASSET_KEYS:
        if indoor.freeze["frozen_assets"].get(name) != outdoor.freeze[
            "frozen_assets"
        ].get(name):
            raise V6ContractError(f"combined halves shared asset mismatch: {name}")
    for name in ("runner_entrypoint", "experiment_runner", "v6_formal"):
        if indoor.freeze[name] != outdoor.freeze[name]:
            raise V6ContractError(f"combined halves tooling/profile mismatch: {name}")
    indoor_evidence_digest = _half_evidence_digest(indoor)
    outdoor_evidence_digest = _half_evidence_digest(outdoor)
    condition_rows = []
    for result in (indoor_result, outdoor_result):
        for row in result["conditions"]:
            condition_rows.append({
                "id": row["id"],
                "qualification": row["qualification"],
                "strict_successes": row["strict_successes"],
                "valid_episodes": row["valid_episodes"],
                "required_strict_successes": row["required_strict_successes"],
                **(
                    {"path_deviation_percent": row["path_deviation_percent"]}
                    if "path_deviation_percent" in row
                    else {}
                ),
            })
    if tuple(row["id"] for row in condition_rows) != (
        *INDOOR_CONDITION_IDS, *OUTDOOR_CONDITION_IDS,
    ):
        raise V6ContractError("combined condition order mismatch")
    return {
        "schema_version": COMBINED_HALVES_SCHEMA_VERSION,
        "qualification": "COMBINED_QUALIFICATION_PASS",
        "formal_qualification": NOT_QUALIFIED,
        "dispatch": False,
        "repositories": json.loads(json.dumps(indoor_repositories)),
        "platform_policy": {
            "driver_version": indoor.freeze["driver_version"],
            "kernel_release": indoor.freeze["kernel_release"],
        },
        "halves": {
            "indoor": {
                "manifest": {
                    "path": str(indoor.path), "sha256": _file_sha256(indoor.path)
                },
                "freeze_digest": indoor.freeze_digest,
                "evidence_digest": indoor_evidence_digest,
                "validated_episodes": 60,
            },
            "outdoor": {
                "manifest": {
                    "path": str(outdoor.path), "sha256": _file_sha256(outdoor.path)
                },
                "freeze_digest": outdoor.freeze_digest,
                "evidence_digest": outdoor_evidence_digest,
                "validated_episodes": 60,
            },
        },
        "conditions": condition_rows,
    }


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise V6ContractError(f"{path} must be a mapping")
    return value


def _require_exact_keys(raw: Mapping[str, Any], expected: set[str], path: str) -> None:
    actual = set(raw)
    if actual != expected:
        raise V6ContractError(
            f"{path} keys must be {sorted(expected)}; got {sorted(actual)}"
        )


def _finite_float(value: Any, path: str) -> float:
    if isinstance(value, bool):
        raise V6ContractError(f"{path} must be finite numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise V6ContractError(f"{path} must be finite numeric") from exc
    if not math.isfinite(result):
        raise V6ContractError(f"{path} must be finite numeric")
    return result


def _pose(raw: Mapping[str, Any], path: str) -> tuple[str, float, float, float]:
    frame_id = str(raw.get("frame_id", ""))
    if frame_id != "map":
        raise V6ContractError(f"{path}.frame_id must be map")
    return (
        frame_id,
        _finite_float(raw.get("x"), f"{path}.x"),
        _finite_float(raw.get("y"), f"{path}.y"),
        _finite_float(raw.get("yaw_deg"), f"{path}.yaw_deg"),
    )


def _xy_goal(raw: Mapping[str, Any], path: str) -> tuple[str, float, float]:
    if set(raw) != {"id", "frame_id", "x", "y"}:
        raise V6ContractError(f"{path} must contain only id/frame_id/x/y")
    frame_id = str(raw.get("frame_id", ""))
    if frame_id != "map":
        raise V6ContractError(f"{path}.frame_id must be map")
    return (
        frame_id,
        _finite_float(raw.get("x"), f"{path}.x"),
        _finite_float(raw.get("y"), f"{path}.y"),
    )


def _validate_phase_b_shadow_assets(assets: Mapping[str, Any]) -> None:
    config_path = Path(str(assets["module1_shadow_config"])).expanduser().resolve()
    checkpoint_path = Path(
        str(assets["module1_shadow_checkpoint"])
    ).expanduser().resolve()
    try:
        document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise V6ContractError(
            f"assets.module1_shadow_config is unreadable: {config_path}"
        ) from exc
    document = _mapping(document, "module1_shadow_config")
    scene = _mapping(document.get("scene"), "module1_shadow_config.scene")
    if scene.get("scene_id") != R5_PHASE_B_SCENE_ID:
        raise V6ContractError("Module1 shadow scene_id differs from Phase B scene")
    if scene.get("map_id") != "v6_kujiale_isaacgen_v1":
        raise V6ContractError("Module1 shadow map_id must be v6_kujiale_isaacgen_v1")
    transform = scene.get("T_map_canvas")
    if transform != [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]:
        raise V6ContractError("Module1 shadow T_map_canvas must be identity")
    state_mask = scene.get("valid_state_mask")
    state_ids = scene.get("valid_state_ids")
    if (
        not isinstance(state_mask, list)
        or len(state_mask) != 256
        or any(not isinstance(value, bool) for value in state_mask)
        or sum(state_mask) != 51
    ):
        raise V6ContractError("Module1 shadow valid_state_mask must contain 51/256 states")
    expected_ids = [index for index, enabled in enumerate(state_mask) if enabled]
    if state_ids != expected_ids:
        raise V6ContractError("Module1 shadow valid_state_ids disagree with mask")
    configured_checkpoint = str(document.get("checkpoint", "")).strip()
    if not configured_checkpoint:
        raise V6ContractError("Module1 shadow checkpoint must be configured")
    configured_path = Path(configured_checkpoint).expanduser()
    if not configured_path.is_absolute():
        configured_path = config_path.parent.parent / configured_path
    if configured_path.resolve() != checkpoint_path:
        raise V6ContractError(
            "assets.module1_shadow_checkpoint differs from the shadow config"
        )


def load_manifest(path: str | Path) -> Manifest:
    manifest_path = Path(path).expanduser().resolve()
    raw = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    raw = _mapping(raw, "manifest")
    if raw.get("schema_version") != SCHEMA_VERSION:
        raise V6ContractError(f"schema_version must be {SCHEMA_VERSION}")
    _require_exact_keys(
        raw,
        {
            "schema_version",
            "intended_use",
            "scene",
            "runtime",
            "assets",
            "mission",
            "dynamic_schedule",
            "episodes",
        },
        "manifest",
    )
    if raw.get("intended_use") != "engineering_pilot":
        raise V6ContractError("intended_use must be engineering_pilot")

    scene = _mapping(raw.get("scene"), "scene")
    _require_exact_keys(scene, {"id", "world", "category"}, "scene")
    scene_id = str(scene.get("id", ""))
    category = str(scene.get("category", ""))
    if scene_id not in {"v6_kujiale_clearance_r2", R5_PHASE_B_SCENE_ID}:
        raise V6ContractError("scene.id is not an R3 Phase2 Kujiale scene")
    if scene.get("world") != "kujiale" or category != "static":
        raise V6ContractError("R3 Phase2 scene must be static Kujiale")

    expected_runtime = (
        R5_PHASE_B_RUNTIME
        if scene_id == R5_PHASE_B_SCENE_ID
        else R3_PHASE2_RUNTIME
    )
    runtime = _mapping(raw.get("runtime"), "runtime")
    _require_exact_keys(runtime, set(expected_runtime), "runtime")
    for name, expected in expected_runtime.items():
        value = runtime.get(name)
        if isinstance(expected, Mapping):
            value = _mapping(value, f"runtime.{name}")
            _require_exact_keys(value, set(expected), f"runtime.{name}")
        if value != expected:
            raise V6ContractError(f"runtime.{name} must be {expected!r}")

    assets = _mapping(raw.get("assets"), "assets")
    asset_keys = (
        R5_PHASE_B_ASSET_KEYS
        if scene_id == R5_PHASE_B_SCENE_ID
        else R3_PHASE2_ASSET_KEYS
    )
    _require_exact_keys(assets, asset_keys, "assets")
    for name, value in assets.items():
        if not isinstance(value, str) or not value:
            raise V6ContractError(f"assets.{name} must be a non-empty path")
    if scene_id == R5_PHASE_B_SCENE_ID:
        expected_suffixes = {
            "scene_asset": "/kujiale_0026/kujiale_0026_A_to_B_door_open.usd",
            "occupancy_map": "/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml",
            "spawn_manifest": (
                "/isaac_sim/configs/environments/"
                "kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
            ),
            "route_graph": (
                "/ros2_ws/src/robot_route_planner/config/"
                "v6_kujiale_isaacgen_v1_gvg_v1.geojson"
            ),
        }
        for name, suffix in expected_suffixes.items():
            if not str(assets[name]).endswith(suffix):
                raise V6ContractError(f"assets.{name} is not the accepted Phase B asset")
        _validate_phase_b_shadow_assets(assets)

    mission = _mapping(raw.get("mission"), "mission")
    _require_exact_keys(mission, {"reset_pose", "legs"}, "mission")
    reset_pose = _mapping(mission.get("reset_pose"), "mission.reset_pose")
    _require_exact_keys(
        reset_pose, {"id", "frame_id", "x", "y", "yaw_deg"}, "mission.reset_pose"
    )
    _, reset_x, reset_y, _ = _pose(reset_pose, "mission.reset_pose")
    mission_rows = mission.get("legs")
    if not isinstance(mission_rows, list) or len(mission_rows) != 5:
        raise V6ContractError("mission.legs must contain exactly five rows")
    mission_legs: list[MissionLeg] = []
    previous_xy = (reset_x, reset_y)
    seen_ids: set[str] = set()
    for index, leg_value in enumerate(mission_rows):
        leg = _mapping(leg_value, f"mission.legs[{index}]")
        goal_id = str(leg.get("id", ""))
        if not goal_id or goal_id in seen_ids:
            raise V6ContractError(f"mission.legs[{index}].id must be unique and non-empty")
        frame_id, x, y = _xy_goal(leg, f"mission.legs[{index}]")
        if math.hypot(x - previous_xy[0], y - previous_xy[1]) <= 1.0e-6:
            raise V6ContractError(f"mission.legs[{index}] is a zero-distance goal")
        mission_legs.append(MissionLeg(goal_id, frame_id, x, y))
        seen_ids.add(goal_id)
        previous_xy = (x, y)
    if [leg.goal_id for leg in mission_legs] != ["G2", "G3", "G4", "G5", "G1"]:
        raise V6ContractError("mission.legs must follow G1->G2->G3->G4->G5->G1")

    schedule_rows = raw.get("dynamic_schedule")
    if not isinstance(schedule_rows, list):
        raise V6ContractError("dynamic_schedule must be a list")
    dynamic_schedule: list[DynamicScheduleEntry] = []
    scheduled_legs: set[str] = set()
    scheduled_groups: set[str] = set()
    for index, row_value in enumerate(schedule_rows):
        row = _mapping(row_value, f"dynamic_schedule[{index}]")
        _require_exact_keys(row, {"leg_id", "group"}, f"dynamic_schedule[{index}]")
        leg_id = str(row.get("leg_id", ""))
        group = str(row.get("group", ""))
        if leg_id not in seen_ids:
            raise V6ContractError(f"dynamic_schedule[{index}].leg_id is not a mission leg")
        if not group:
            raise V6ContractError(f"dynamic_schedule[{index}].group must be non-empty")
        if leg_id in scheduled_legs or group in scheduled_groups:
            raise V6ContractError("dynamic_schedule leg_id/group must be unique")
        scheduled_legs.add(leg_id)
        scheduled_groups.add(group)
        dynamic_schedule.append(DynamicScheduleEntry(leg_id, group))

    rows = raw.get("episodes")
    if not isinstance(rows, list) or not rows:
        raise V6ContractError("episodes must contain at least one row")
    episodes: list[Episode] = []
    episode_seeds: set[int] = set()
    for index, row_value in enumerate(rows):
        row = _mapping(row_value, f"episodes[{index}]")
        _require_exact_keys(
            row,
            {"seed", "variant_id", "reset_pose_name", "dynamic_case_id"},
            f"episodes[{index}]",
        )
        seed = row.get("seed")
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise V6ContractError(f"episodes[{index}].seed must be non-negative int")
        if seed in episode_seeds:
            raise V6ContractError(f"episodes[{index}].seed must be unique")
        episode_seeds.add(seed)
        episodes.append(
            Episode(
                seed=seed,
                variant_id=str(row.get("variant_id", "")),
                appearance_profile_id=(
                    str(row["appearance_profile_id"])
                    if row.get("appearance_profile_id") is not None
                    else None
                ),
                reset_pose_name=str(row.get("reset_pose_name", "")),
                dynamic_case_id=str(row.get("dynamic_case_id", "")),
            )
        )
    return Manifest(
        path=manifest_path,
        raw=raw,
        scene_id=scene_id,
        category=category,
        runtime=runtime,
        assets={name: str(value) for name, value in assets.items()},
        reset_pose=reset_pose,
        mission_legs=tuple(mission_legs),
        dynamic_schedule=tuple(dynamic_schedule),
        episodes=tuple(episodes),
    )


def authorize_manifest(
    manifest: Manifest,
    *,
    mode: str,
) -> str:
    """Authorize only the bounded R3 Phase2 engineering pilot."""

    if mode not in {"formal", "pilot"}:
        raise V6ContractError("mode must be formal or pilot")
    if mode == "formal":
        raise V6ContractError("formal dispatch refused: R3 Phase2 is engineering pilot only")
    return NOT_QUALIFIED


@dataclass
class ReadinessFacts:
    reset_service_ready: bool = False
    reset_event_publisher_ready: bool = False
    reset_subscriber_roster_ready: bool = False
    route_goal_subscriber_ready: bool = False
    clock_seen: bool = False
    scan_seen: bool = False
    map_seen: bool = False
    navigation_graph_seen: bool = False
    estimated_odom_seen: bool = False

    def missing(self) -> tuple[str, ...]:
        return tuple(name for name, value in vars(self).items() if not value)


@dataclass
class EpisodeGuard:
    """Exactly-once reset and R3 Phase2 baseline navigation readiness."""

    state: str = "WAITING_READINESS"
    stop_reason: str = ""
    reset_calls: int = 0
    reset_events: int = 0
    initialpose_messages: int = 0
    amcl_messages: int = 0
    initialpose_stamp_ns: int | None = None
    post_initialpose_amcl_seen: bool = False
    nav2_active: bool = False
    tf_active: bool = False
    reset_gate_generation: int | None = None
    reset_gate_released_generation: int | None = None
    goal_publications: int = 0
    route_progress_messages: int = 0
    route_completion_messages: int = 0
    route_succeeded: bool = False
    mission_leg_ids: tuple[str, ...] = ()
    completed_leg_ids: list[str] = field(default_factory=list)
    current_leg_progress_messages: int = 0

    def stop(self, reason: str) -> None:
        if self.state != "STOP":
            self.state = "STOP"
            self.stop_reason = reason

    def arm_reset(
        self,
        facts: ReadinessFacts,
    ) -> None:
        missing = facts.missing()
        if missing:
            raise V6ContractError(f"reset readiness missing: {', '.join(missing)}")
        if self.goal_publications:
            self.stop("reset_with_active_goal_forbidden")
            raise V6ContractError(self.stop_reason)
        if self.reset_calls:
            self.stop("reset_retry_forbidden")
            raise V6ContractError(self.stop_reason)
        self.state = "RESET_ARMED"

    def record_reset_call(self) -> None:
        if self.state != "RESET_ARMED" or self.reset_calls:
            self.stop("reset_retry_forbidden")
            raise V6ContractError("reset_retry_forbidden")
        self.reset_calls = 1
        self.state = "RESET_IN_FLIGHT"

    def record_reset_response(self, success: bool | None) -> None:
        if self.reset_calls != 1 or self.state == "STOP":
            self.stop("unexpected_reset_response")
            return
        if success is not True:
            self.stop("reset_response_unknown" if success is None else "reset_rejected")
            return
        self.state = "WAITING_RESET_EVENT"

    def record_reset_event(self) -> None:
        self.reset_events += 1
        if self.reset_events > 1:
            self.stop("second_reset_event")
            return
        if self.reset_calls != 1:
            self.stop("reset_event_without_call")
            return
        self.state = "WAITING_INITIALPOSE"

    def record_initialpose(self, stamp_ns: int) -> None:
        self.initialpose_messages += 1
        if self.reset_events != 1:
            self.stop("initialpose_outside_reset_epoch")
            return
        if stamp_ns <= 0:
            self.stop("initialpose_stamp_invalid")
            return
        # The enrollment/reseed machinery republishes the same calibrated
        # pose as a short burst (and the activation gate may request another
        # reseed burst during recovery); the first seed of the generation is
        # the freshness anchor and repeats within the epoch are accepted.
        if self.initialpose_stamp_ns is None:
            self.initialpose_stamp_ns = int(stamp_ns)
            self.state = "WAITING_AMCL"

    def record_amcl(self, stamp_ns: int) -> None:
        self.amcl_messages += 1
        if self.initialpose_stamp_ns is None:
            return
        if stamp_ns <= self.initialpose_stamp_ns:
            # AMCL stamps poses with the source scan time, which lags the
            # seed's publication stamp; straddling samples are stale-stamped
            # and skipped, and B5 seed confirmation gates the seed itself.
            return
        self.post_initialpose_amcl_seen = True
        self._maybe_goal_ready()

    def record_navigation_ready(self, *, nav2_active: bool, tf_active: bool) -> None:
        self.nav2_active = bool(nav2_active)
        self.tf_active = bool(tf_active)
        self._maybe_goal_ready()

    def record_reset_receipt_generation(self, generation: int) -> None:
        """Bind goal readiness to this reset receipt's stop-gate generation."""
        self.reset_gate_generation = int(generation)
        self._maybe_goal_ready()

    def record_reset_gate_status(self, generation: int, held: bool) -> None:
        # Only the release of this reset's own gate generation arms the
        # fact; a release latched from an earlier generation is stale.
        if not held:
            self.reset_gate_released_generation = int(generation)
        self._maybe_goal_ready()

    def _maybe_goal_ready(self) -> None:
        if self.state == "STOP":
            return
        # The readiness facts stay latched once the episode advances past
        # GOAL_READY, so the steady-state prior/AMCL/B5 stream would
        # otherwise regress NAVIGATING/LEG_SUCCEEDED back to GOAL_READY.
        if self.state in {"NAVIGATING", "LEG_SUCCEEDED", "SUCCEEDED", "FAILED"}:
            return
        if all(
            (
                self.reset_calls == 1,
                self.reset_events == 1,
                self.initialpose_messages >= 1,
                self.post_initialpose_amcl_seen,
                self.nav2_active,
                self.tf_active,
                self.reset_gate_released,
            )
        ):
            self.state = "GOAL_READY"

    @property
    def reset_gate_released(self) -> bool:
        """The ResetStopGate released this reset receipt's own generation."""
        return (
            self.reset_gate_generation is not None
            and self.reset_gate_released_generation == self.reset_gate_generation
        )

    @property
    def goal_ready(self) -> bool:
        return self.state == "GOAL_READY" and not self.stop_reason

    @property
    def localization_ready(self) -> bool:
        return bool(
            not self.stop_reason
            and self.reset_events == 1
            and self.initialpose_messages >= 1
            and self.post_initialpose_amcl_seen
        )

    def record_goal_publication(self, goal_id: str | None = None) -> None:
        first_leg = self.goal_publications == 0
        if (first_leg and not self.goal_ready) or (not first_leg and self.state != "LEG_SUCCEEDED"):
            self.stop("route_goal_publication_not_authorized")
            raise V6ContractError(self.stop_reason)
        if self.goal_publications >= max(1, len(self.mission_leg_ids)):
            self.stop("extra_route_goal_publication")
            raise V6ContractError(self.stop_reason)
        if self.mission_leg_ids:
            expected = self.mission_leg_ids[self.goal_publications]
            if goal_id != expected:
                self.stop(f"mission_leg_order:{goal_id}!={expected}")
                raise V6ContractError(self.stop_reason)
        self.goal_publications += 1
        self.current_leg_progress_messages = 0
        self.state = "NAVIGATING"

    def record_route_progress(self) -> None:
        if self.state == "NAVIGATING":
            self.route_progress_messages += 1
            self.current_leg_progress_messages += 1

    def record_route_completion(self, succeeded: bool) -> None:
        if self.state != "NAVIGATING":
            return
        self.route_completion_messages += 1
        self.route_succeeded = bool(succeeded)
        if not self.current_leg_progress_messages:
            self.stop("route_completed_without_progress")
        elif not succeeded:
            self.state = "FAILED"
        else:
            if self.mission_leg_ids:
                self.completed_leg_ids.append(
                    self.mission_leg_ids[self.goal_publications - 1]
                )
            final_leg = self.goal_publications >= max(1, len(self.mission_leg_ids))
            self.state = "SUCCEEDED" if final_leg else "LEG_SUCCEEDED"


@dataclass
class DynamicActionLedger:
    """Claim each dynamic service action once; a failed claim is never retried."""

    claimed: set[tuple[str, str]] = field(default_factory=set)
    events: list[dict[str, Any]] = field(default_factory=list)

    def claim(self, group: str, action: str) -> None:
        if action not in {"trigger", "complete"}:
            raise V6ContractError(f"unknown dynamic action {action}")
        key = (group, action)
        if key in self.claimed:
            raise V6ContractError(f"dynamic action retry forbidden: {group}/{action}")
        if action == "complete" and (group, "trigger") not in self.claimed:
            raise V6ContractError(f"dynamic completion before trigger: {group}")
        self.claimed.add(key)
        self.events.append({"group": group, "action": action, "result": "claimed"})

    def record(self, group: str, action: str, result: str, detail: str = "") -> None:
        self.events.append(
            {"group": group, "action": action, "result": result, "detail": detail}
        )


def _message_summary(message: Any) -> dict[str, Any]:
    summary: dict[str, Any] = {"type": type(message).__name__}
    for name in (
        "reset_epoch",
        "sequence",
        "request_id",
        "graph_id",
        "revision",
        "event_sequence",
        "accepted",
        "success",
        "failure",
        "reason",
        "applied",
        "rejection_mask",
        "raised_cell_count",
        "data",
    ):
        if hasattr(message, name):
            value = getattr(message, name)
            if isinstance(value, (str, bool, int, float)) or value is None:
                summary[name] = value
    for vector_name in ("linear", "angular"):
        vector = getattr(message, vector_name, None)
        if vector is not None:
            summary[vector_name] = {
                axis: float(getattr(vector, axis))
                for axis in ("x", "y", "z")
                if hasattr(vector, axis)
            }
    return summary


class V6FormalNode:
    """Runtime adapter, imported lazily so manifest checks need no ROS graph."""

    TERMINAL_ZERO_TIMEOUT_SEC = 2.0
    TERMINAL_ZERO_PERIOD_SEC = 0.05
    TERMINAL_ZERO_QUIET_SEC = 0.30
    TERMINAL_ZERO_CADENCE_TOLERANCE_SEC = 0.10

    def __init__(
        self,
        manifest: Manifest,
        episode: Episode,
        output_jsonl: Path,
        *,
        qualification: str = ENGINEERING_PILOT,
    ):
        import rclpy
        from action_msgs.srv import CancelGoal
        from bio_nav_interfaces.msg import (
            CanonicalRoute,
            NavigationGraph,
            RouteProgress,
        )
        from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rosgraph_msgs.msg import Clock
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.parameter_client import AsyncParameterClient
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool, Empty, String
        from std_srvs.srv import Trigger
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage

        class _Node(Node):
            pass

        self._rclpy = rclpy
        self.node = _Node("bio_nav_v6_formal_single_episode")
        self.manifest = manifest
        self.episode = episode
        self.output_jsonl = output_jsonl
        self.qualification = qualification
        self.guard = EpisodeGuard(
            mission_leg_ids=tuple(item.goal_id for item in manifest.mission_legs)
        )
        self.facts = ReadinessFacts()
        self.pre_reset_quiet_since: float | None = None
        self._cmd_window: deque[tuple[float, bool]] = deque()
        self._odom_window: deque[tuple[float, float, float]] = deque()
        self.post_reset_odom_xy: list[tuple[float, float]] = []
        self.map_odom_tf_seen = False
        self.odom_base_tf_seen = False
        self.canonical_route_count = 0
        self.collision = False
        self.route_goal_results: list[dict[str, Any]] = []
        self.obstacle_state_messages: list[dict[str, Any]] = []
        self.dynamic_actions = DynamicActionLedger()
        self.dynamic_clients: dict[tuple[str, str], Any] = {}
        self.reset_receipt: dict[str, Any] | None = None
        self._terminal_cancel_requested = False
        self._terminal_cancel_future = None
        self._terminal_started_monotonic: float | None = None
        self._navigation_terminal_observed = False
        self._terminal_zero_settled = False
        self._terminal_zero_confirmed = False
        self._terminal_zero_reason = "not_required"
        self._terminal_topic_summary: dict[str, dict[str, Any]] = {}
        self._cmd_vel_sim_last_receive_monotonic: float | None = None
        self._cmd_vel_sim_last_nonzero_monotonic: float | None = None
        self._cmd_vel_sim_zero_stamps: deque[float] = deque()
        self._types = {
            "CancelGoal": CancelGoal,
            "PoseStamped": PoseStamped,
            "Twist": Twist,
            "Trigger": Trigger,
            "Parameter": Parameter,
        }
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        command_observation_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        terminal_zero_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.route_goal_publisher = self.node.create_publisher(
            PoseStamped, "/bio_nav/route_goal", reliable
        )
        self.terminal_zero_publisher = self.node.create_publisher(
            Twist, "/cmd_vel_nav", terminal_zero_qos
        )
        self.reset_client = self.node.create_client(Trigger, "/simulation/reset")
        self.nav2_active_client = self.node.create_client(
            Trigger, "/lifecycle_manager_navigation/is_active"
        )
        self.navigate_cancel_client = self.node.create_client(
            CancelGoal, "/navigate_to_pose/_action/cancel_goal"
        )
        self.isaac_parameters = AsyncParameterClient(self.node, "/isaac_navigation_sim")

        def sub(message_type, topic, callback, qos=reliable):
            return self.node.create_subscription(message_type, topic, callback, qos)

        self.subscriptions = [
            sub(Clock, "/clock", lambda m: self._fact("clock_seen", "/clock", m), sensor),
            sub(LaserScan, "/scan", lambda m: self._fact("scan_seen", "/scan", m), sensor),
            sub(Odometry, "/odom", self._odom, sensor),
            sub(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose, reliable),
            sub(PoseWithCovarianceStamped, "/initialpose", self._initialpose, reliable),
            sub(TFMessage, "/tf", self._tf),
            sub(TFMessage, "/tf_static", self._tf, latched),
            sub(OccupancyGrid, "/map", lambda m: self._fact("map_seen", "/map", m), latched),
            sub(Empty, "/simulation/reset_event", self._reset_event),
            sub(String, "/simulation/reset_stop_gate/status", self._reset_gate_status, latched),
            sub(NavigationGraph, "/bio_nav/navigation_graph", lambda m: self._fact("navigation_graph_seen", "/bio_nav/navigation_graph", m), latched),
            sub(CanonicalRoute, "/bio_nav/canonical_route", self._canonical_route, latched),
            sub(RouteProgress, "/bio_nav/route_progress", self._route_progress),
            sub(Bool, "/bio_nav/route_goal_complete", self._route_complete),
            sub(String, "/bio_nav/route_goal_result", self._route_result),
            sub(Twist, "/cmd_vel", lambda m: self._track_command("/cmd_vel", m), command_observation_qos),
            sub(Twist, "/cmd_vel_nav", lambda m: self._track_command("/cmd_vel_nav", m), command_observation_qos),
            sub(Twist, "/cmd_vel_sim", lambda m: self._track_command("/cmd_vel_sim", m), command_observation_qos),
            sub(Bool, "/simulation/collision", self._collision),
            sub(String, "/simulation/collision_diagnostics", self._capture_callback("/simulation/collision_diagnostics")),
            sub(String, "/experiment/obstacles/state", self._obstacle_state),
        ]

    def _write(self, event: str, **payload: Any) -> None:
        append_evidence_jsonl(self.output_jsonl, event, **payload)

    def _capture(self, topic: str, message: Any, *, important: bool = False) -> None:
        summary = _message_summary(message)
        settling = (
            getattr(self, "_terminal_started_monotonic", None) is not None
            and not getattr(self, "_terminal_zero_settled", False)
        )
        if settling and not important:
            topics = getattr(self, "_terminal_topic_summary", None)
            if topics is None:
                topics = {}
                self._terminal_topic_summary = topics
            topic_summary = topics.setdefault(
                topic, {"count": 0, "last_message": summary}
            )
            topic_summary["count"] += 1
            topic_summary["last_message"] = summary
            return
        self._write("topic", topic=topic, message=summary)

    def _flush_terminal_topic_summary(self) -> None:
        topics = getattr(self, "_terminal_topic_summary", {})
        self._terminal_topic_summary = {}
        self._write("terminal_topic_summary", topics=topics)

    def _capture_callback(self, topic: str):
        return lambda message: self._capture(topic, message)

    def _fact(self, name: str, topic: str, message: Any) -> None:
        setattr(self.facts, name, True)
        self._capture(topic, message)

    def _odom(self, message: Any) -> None:
        self.facts.estimated_odom_seen = True
        now = time.monotonic()
        x = float(message.pose.pose.position.x)
        y = float(message.pose.pose.position.y)
        self._odom_window.append((now, x, y))
        horizon = 4.0 * PRE_RESET_NEGATIVE_WINDOW_S
        while self._odom_window and now - self._odom_window[0][0] > horizon:
            self._odom_window.popleft()
        if self.guard.reset_events == 1 and not self.guard.goal_publications:
            self.post_reset_odom_xy.append((x, y))
        self._capture("/odom", message)

    def _track_command(self, topic: str, message: Any) -> None:
        nonzero = any(
            abs(float(value)) > COMMAND_ZERO_TOLERANCE
            for value in (
                message.linear.x,
                message.linear.y,
                message.angular.z,
            )
        )
        now = time.monotonic()
        if topic == "/cmd_vel_sim":
            self._cmd_vel_sim_last_receive_monotonic = now
            if nonzero:
                self._cmd_vel_sim_last_nonzero_monotonic = now
                self._cmd_vel_sim_zero_stamps.clear()
            else:
                self._cmd_vel_sim_zero_stamps.append(now)
        self._cmd_window.append((now, nonzero))
        horizon = 4.0 * PRE_RESET_NEGATIVE_WINDOW_S
        while self._cmd_window and now - self._cmd_window[0][0] > horizon:
            self._cmd_window.popleft()
        if nonzero and self.guard.reset_calls and not self.guard.goal_publications:
            self.guard.stop(f"post_reset_command_nonzero:{topic}")
        self._capture(topic, message)

    def _reset_event(self, message: Any) -> None:
        previous_reset_events = self.guard.reset_events
        self.guard.record_reset_event()
        if (
            previous_reset_events == 0
            and self.guard.reset_events == 1
            and not self.guard.stop_reason
        ):
            # TF observed before this accepted reset epoch cannot satisfy
            # post-reset navigation readiness.  A duplicate or out-of-order
            # reset event retains EpisodeGuard's existing STOP semantics and
            # must not rewrite the current epoch's observations.
            self.map_odom_tf_seen = False
            self.odom_base_tf_seen = False
            self.guard.tf_active = False
        self._capture("/simulation/reset_event", message)

    def _reset_gate_status(self, message: Any) -> None:
        if self.guard.reset_calls:
            try:
                document = json.loads(str(message.data))
                generation = document["generation"]
                held = document["held"]
                valid = (
                    not isinstance(generation, bool)
                    and isinstance(generation, int)
                    and generation >= 0
                    and isinstance(held, bool)
                )
            except (AttributeError, KeyError, TypeError, json.JSONDecodeError):
                valid = False
            if not valid:
                self.guard.stop("reset_gate_status_invalid")
                return
            self.guard.record_reset_gate_status(generation, held)
        self._capture("/simulation/reset_stop_gate/status", message)

    @staticmethod
    def _header_stamp_ns(message: Any) -> int:
        stamp = message.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _amcl_pose(self, message: Any) -> None:
        if self.guard.reset_calls:
            self.guard.record_amcl(self._header_stamp_ns(message))
        self._capture("/amcl_pose", message)

    def _initialpose(self, message: Any) -> None:
        if self.guard.reset_calls:
            self.guard.record_initialpose(self._header_stamp_ns(message))
        self._capture("/initialpose", message)

    def _tf(self, message: Any) -> None:
        for transform in message.transforms:
            parent = str(transform.header.frame_id).lstrip("/")
            child = str(transform.child_frame_id).lstrip("/")
            self.map_odom_tf_seen |= parent == "map" and child == "odom"
            self.odom_base_tf_seen |= parent == "odom" and child in {
                "base_link", "base_footprint"
            }

    def _canonical_route(self, message: Any) -> None:
        self.canonical_route_count += 1
        self._capture("/bio_nav/canonical_route", message)

    def _route_progress(self, message: Any) -> None:
        self._track_route_signal("route_progress")
        self.guard.record_route_progress()
        self._capture("/bio_nav/route_progress", message)

    def _route_complete(self, message: Any) -> None:
        self._track_route_signal("route_goal_complete")
        if self.guard.goal_publications > len(self.guard.completed_leg_ids):
            self._navigation_terminal_observed = True
        self.guard.record_route_completion(bool(message.data))
        if self.guard.state in {"FAILED", "STOP"}:
            self._cancel_active_navigation_once(
                self.guard.stop_reason or "route_failed"
            )
        self._capture("/bio_nav/route_goal_complete", message, important=True)

    def _route_result(self, message: Any) -> None:
        self._track_route_signal("route_goal_result")
        row = self._json_message(message)
        self.route_goal_results.append(row)
        self._capture("/bio_nav/route_goal_result", message, important=True)

    def _track_route_signal(self, kind: str) -> None:
        """Route traffic is only legal after this runner's first goal."""
        if self.guard.reset_calls and not self.guard.goal_publications:
            self.guard.stop(f"stale_{kind}_after_reset")

    @staticmethod
    def _json_message(message: Any) -> dict[str, Any]:
        try:
            value = json.loads(str(message.data))
        except (AttributeError, json.JSONDecodeError):
            return {"raw": str(getattr(message, "data", ""))}
        return value if isinstance(value, dict) else {"value": value}

    def _obstacle_state(self, message: Any) -> None:
        row = self._json_message(message)
        self.obstacle_state_messages.append(row)
        self._write("obstacle_state", state=row)

    def _collision(self, message: Any) -> None:
        first_collision = bool(message.data) and not self.collision
        self.collision = self.collision or bool(message.data)
        if message.data:
            self.guard.stop("collision")
            self._cancel_active_navigation_once("collision")
        self._capture(
            "/simulation/collision", message, important=first_collision
        )

    def _cancel_active_navigation_once(self, reason: str) -> None:
        self._start_terminal_settle(cancel_navigation=True, reason=reason)

    def _start_terminal_settle(
        self, *, cancel_navigation: bool, reason: str
    ) -> None:
        if self._terminal_started_monotonic is None:
            self._terminal_started_monotonic = time.monotonic()
            self._terminal_zero_reason = "pending"
            self._cmd_vel_sim_zero_stamps.clear()
        if not cancel_navigation:
            return
        active_goal = self.guard.goal_publications > len(
            self.guard.completed_leg_ids
        )
        if self._terminal_cancel_requested or not active_goal:
            return
        self._terminal_cancel_requested = True
        CancelGoal = self._types["CancelGoal"]
        self._terminal_cancel_future = self.navigate_cancel_client.call_async(
            CancelGoal.Request()
        )
        self._write("terminal_navigation_cancel_requested", reason=reason)

    def _settle_terminal_zero(self) -> bool:
        if self._terminal_zero_settled:
            return self._terminal_zero_confirmed

        terminal_start = self._terminal_started_monotonic
        if terminal_start is None:
            self._start_terminal_settle(
                cancel_navigation=False, reason=self.guard.stop_reason or self.guard.state
            )
            terminal_start = self._terminal_started_monotonic
        assert terminal_start is not None
        deadline = terminal_start + self.TERMINAL_ZERO_TIMEOUT_SEC
        next_publish = terminal_start
        Twist = self._types["Twist"]
        zero = Twist()

        while self._rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.terminal_zero_publisher.publish(zero)
                next_publish = now + self.TERMINAL_ZERO_PERIOD_SEC

            self._rclpy.spin_once(
                self.node,
                timeout_sec=min(
                    self.TERMINAL_ZERO_PERIOD_SEC,
                    max(0.0, deadline - time.monotonic()),
                ),
            )
            now = time.monotonic()
            while (
                self._cmd_vel_sim_zero_stamps
                and self._cmd_vel_sim_zero_stamps[0] <= terminal_start
            ):
                self._cmd_vel_sim_zero_stamps.popleft()

            cancel_complete = bool(
                not self._terminal_cancel_requested
                or (
                    self._terminal_cancel_future is not None
                    and self._terminal_cancel_future.done()
                )
                or self._navigation_terminal_observed
            )
            repeated_zero = len(self._cmd_vel_sim_zero_stamps) >= 2
            first_zero = (
                self._cmd_vel_sim_zero_stamps[0] if repeated_zero else None
            )
            last_zero = self._cmd_vel_sim_zero_stamps[-1] if repeated_zero else None
            if (
                cancel_complete
                and first_zero is not None
                and last_zero is not None
                and last_zero - first_zero >= self.TERMINAL_ZERO_QUIET_SEC
                and now - last_zero <= self.TERMINAL_ZERO_CADENCE_TOLERANCE_SEC
            ):
                self._terminal_zero_settled = True
                self._terminal_zero_confirmed = True
                self._terminal_zero_reason = "terminal_zero_confirmed"
                self._flush_terminal_topic_summary()
                self._write(
                    "terminal_zero_confirmed",
                    root_reason=self.guard.stop_reason,
                )
                return True

        self._terminal_zero_settled = True
        self._terminal_zero_reason = "terminal_zero_timeout"
        if self.guard.state == "SUCCEEDED":
            self.guard.stop("terminal_zero_timeout_after_success")
        self._flush_terminal_topic_summary()
        self._write("terminal_zero_timeout", root_reason=self.guard.stop_reason)
        return False

    def _spin_until(self, predicate, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while self._rclpy.ok() and time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return bool(predicate())

    def _refresh_endpoint_facts(self) -> None:
        by_topic = {sub.topic_name: sub for sub in self.subscriptions}
        self.facts.reset_service_ready = self.reset_client.service_is_ready()
        self.facts.reset_event_publisher_ready = (
            by_topic["/simulation/reset_event"].get_publisher_count() > 0
        )
        # Runner and ResetStopGate must both observe the single reset event.
        self.facts.reset_subscriber_roster_ready = (
            self.node.count_subscribers("/simulation/reset_event") >= 2
        )
        self.facts.route_goal_subscriber_ready = (
            self.route_goal_publisher.get_subscription_count() > 0
        )

    def _publisher_ownership_violations(self) -> tuple[str, ...]:
        return tuple(
            f"{topic}={self.node.count_publishers(topic)}"
            for topic in SOLE_PUBLISHER_TOPICS
            if self.node.count_publishers(topic) != 1
        )

    def _pre_reset_still(self) -> bool:
        """Cold boundary: zero commands and a bounded odom span in-window."""
        now = time.monotonic()
        horizon = now - PRE_RESET_NEGATIVE_WINDOW_S
        if any(nonzero for stamp, nonzero in self._cmd_window if stamp >= horizon):
            return False
        window = [(x, y) for stamp, x, y in self._odom_window if stamp >= horizon]
        if not window:
            return False
        xs = [point[0] for point in window]
        ys = [point[1] for point in window]
        return bool(
            max(xs) - min(xs) <= PRE_RESET_STILL_SPAN_M
            and max(ys) - min(ys) <= PRE_RESET_STILL_SPAN_M
        )

    def _readiness_blockers(self) -> str:
        blockers: list[str] = []
        missing = self.facts.missing()
        if missing:
            blockers.append("facts:" + ",".join(missing))
        ownership = self._publisher_ownership_violations()
        if ownership:
            blockers.append("publisher_ownership:" + ",".join(ownership))
        if not self._pre_reset_still():
            blockers.append("pre_reset_not_still")
        return ";".join(blockers)

    def _assert_ground_truth_firewall(self) -> None:
        offending = [
            subscription.topic_name
            for subscription in self.subscriptions
            if subscription.topic_name.startswith(GT_PREFIX)
        ]
        if offending:
            raise V6ContractError(
                "dispatcher Ground Truth firewall violated: " + ",".join(offending)
            )

    def _check_post_reset_odom(self) -> None:
        """Odometry must land at the re-zeroed origin and stay bounded."""
        samples = self.post_reset_odom_xy[1:]  # skip one straddling sample
        if len(samples) < 2:
            self.guard.stop("post_reset_odom_missing")
            return
        landing = math.hypot(samples[0][0], samples[0][1])
        if landing > POST_RESET_ODOM_LANDING_M:
            self.guard.stop(f"post_reset_odom_landing:{landing:.3f}")
            return
        xs = [point[0] for point in samples]
        ys = [point[1] for point in samples]
        span = max(max(xs) - min(xs), max(ys) - min(ys))
        if span > POST_RESET_ODOM_SPAN_M:
            self.guard.stop(f"post_reset_odom_span:{span:.3f}")

    def _pre_reset_ready(self) -> bool:
        self._refresh_endpoint_facts()
        if self._readiness_blockers():
            self.pre_reset_quiet_since = None
            return False
        now = time.monotonic()
        if self.pre_reset_quiet_since is None:
            self.pre_reset_quiet_since = now
            return False
        return now - self.pre_reset_quiet_since >= PRE_RESET_NEGATIVE_WINDOW_S

    def _nav2_is_active(self, timeout_sec: float) -> bool:
        if not self.nav2_active_client.wait_for_service(timeout_sec=timeout_sec):
            return False
        Trigger = self._types["Trigger"]
        future = self.nav2_active_client.call_async(Trigger.Request())
        if not self._spin_until(future.done, timeout_sec):
            return False
        response = future.result()
        return bool(response is not None and response.success is True)

    def _wait_nav2_and_tf_ready(self, timeout_sec: float) -> None:
        """Poll the post-reset Nav2/TF probe until the reset budget expires.

        The activation gate's post-reset time-jump recovery briefly pauses
        Nav2; a one-shot probe landing inside the pause window failed closed
        spuriously (live R5: episode aborted with nav2_or_tf_not_ready 6 s
        after start, while the gate resumed ~3 s later).  Poll until both
        facts hold; an exhausted budget still fails closed with the same
        reason.
        """
        deadline = time.monotonic() + timeout_sec
        while True:
            self.guard.record_navigation_ready(
                nav2_active=self._nav2_is_active(NAV2_PROBE_ATTEMPT_TIMEOUT_SEC),
                tf_active=self.map_odom_tf_seen and self.odom_base_tf_seen,
            )
            if self.guard.nav2_active and self.guard.tf_active:
                return
            if self.guard.state == "STOP" or time.monotonic() >= deadline:
                self.guard.stop("nav2_or_tf_not_ready")
                return
            self._rclpy.spin_once(self.node, timeout_sec=0.5)

    def _set_episode_parameters(self, timeout_sec: float) -> None:
        Parameter = self._types["Parameter"]
        if not self.isaac_parameters.wait_for_services(timeout_sec=timeout_sec):
            raise V6ContractError("Isaac reset parameter services unavailable")
        params = [
            Parameter("reset_seed", value=self.episode.seed),
            Parameter("reset_pose_name", value=self.episode.reset_pose_name),
            Parameter("dynamic_case_id", value=self.episode.dynamic_case_id),
            Parameter("dynamic_variant_id", value=self.episode.variant_id),
        ]
        if self.episode.appearance_profile_id:
            params.append(Parameter("appearance_profile_id", value=self.episode.appearance_profile_id))
        future = self.isaac_parameters.set_parameters(params)
        if not self._spin_until(future.done, timeout_sec):
            raise V6ContractError("setting episode parameters timed out")
        response = future.result()
        if response is None or any(not result.successful for result in response.results):
            raise V6ContractError("Isaac rejected episode parameters")
        self._write("episode_parameters_set", seed=self.episode.seed)

    def _goal_message(self, leg: MissionLeg):
        PoseStamped = self._types["PoseStamped"]
        message = PoseStamped()
        message.header.frame_id = leg.frame_id
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.position.x = leg.x
        message.pose.position.y = leg.y
        message.pose.orientation.w = 1.0
        return message

    def _call_dynamic_action(self, group: str, action: str, timeout_sec: float) -> bool:
        if not group:
            return True
        try:
            self.dynamic_actions.claim(group, action)
        except V6ContractError as exc:
            self.guard.stop(str(exc))
            return False
        Trigger = self._types["Trigger"]
        key = (group, action)
        client = self.dynamic_clients.get(key)
        if client is None:
            client = self.node.create_client(
                Trigger, f"/experiment/obstacles/{group}/{action}"
            )
            self.dynamic_clients[key] = client
        self._write("dynamic_action", group=group, action=action, phase="call")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            reason = f"dynamic_{action}_service_unavailable:{group}"
            self.dynamic_actions.record(group, action, "service_unavailable")
            self.guard.stop(reason)
            return False
        future = client.call_async(Trigger.Request())
        if not self._spin_until(future.done, timeout_sec):
            reason = f"dynamic_{action}_timeout:{group}"
            self.dynamic_actions.record(group, action, "timeout")
            self.guard.stop(reason)
            return False
        response = future.result()
        if response is None or response.success is not True:
            detail = "no response" if response is None else str(response.message)
            reason = f"dynamic_{action}_rejected:{group}:{detail}"
            self.dynamic_actions.record(group, action, "rejected", detail)
            self.guard.stop(reason)
            return False
        detail = str(response.message)
        self.dynamic_actions.record(group, action, "accepted", detail)
        self._write(
            "dynamic_action",
            group=group,
            action=action,
            phase="response",
            success=True,
            detail=detail,
        )
        return True

    def _run_mission_leg(
        self,
        *,
        index: int,
        leg: MissionLeg,
        dynamic_group: str,
        reset_timeout_sec: float,
        navigation_timeout_sec: float,
    ) -> None:
        route_baseline = self.canonical_route_count
        result_baseline = len(self.route_goal_results)
        self._navigation_terminal_observed = False
        triggered = bool(dynamic_group) and self._call_dynamic_action(
            dynamic_group, "trigger", reset_timeout_sec
        )
        if dynamic_group and not triggered:
            return
        self.guard.record_goal_publication(leg.goal_id)
        self.route_goal_publisher.publish(self._goal_message(leg))
        self._write(
            "route_goal_published",
            topic="/bio_nav/route_goal",
            leg_id=leg.goal_id,
            leg_index=index,
            result_messages_before=result_baseline,
        )
        completed = self._spin_until(
            lambda: self.guard.state
            in {"LEG_SUCCEEDED", "SUCCEEDED", "FAILED", "STOP"},
            navigation_timeout_sec,
        )
        if not completed:
            self.guard.stop(f"route_completion_timeout:{leg.goal_id}")
        if triggered:
            self._call_dynamic_action(dynamic_group, "complete", reset_timeout_sec)
        if self.canonical_route_count <= route_baseline:
            self.guard.stop(f"canonical_route_missing:{leg.goal_id}")
        self._write(
            "mission_leg_result",
            leg_id=leg.goal_id,
            state=self.guard.state,
            route_progress_messages=self.guard.current_leg_progress_messages,
            route_result_messages=len(self.route_goal_results) - result_baseline,
        )

    def run(self, *, readiness_timeout_sec: float, reset_timeout_sec: float, navigation_timeout_sec: float) -> dict[str, Any]:
        self._assert_ground_truth_firewall()
        self._write(
            "episode_start",
            qualification=self.qualification,
            formal_qualification=NOT_QUALIFIED,
            manifest=str(self.manifest.path),
            seed=self.episode.seed,
            runtime=dict(self.manifest.runtime),
            assets=dict(self.manifest.assets),
        )
        ready = self._spin_until(
            self._pre_reset_ready,
            readiness_timeout_sec,
        )
        if not ready:
            self.guard.stop("readiness_timeout:" + (self._readiness_blockers() or "unknown"))
            return self.result()
        self.guard.arm_reset(self.facts)
        self._set_episode_parameters(reset_timeout_sec)

        Trigger = self._types["Trigger"]
        self.guard.record_reset_call()
        future = self.reset_client.call_async(Trigger.Request())
        if not self._spin_until(future.done, reset_timeout_sec):
            self.guard.record_reset_response(None)
            return self.result()
        response = future.result()
        self.guard.record_reset_response(response.success if response is not None else None)
        if self.guard.state == "STOP":
            return self.result()
        try:
            self.reset_receipt = parse_reset_receipt(
                response.message,
                requested_seed=self.episode.seed,
                requested_case_id=self.episode.dynamic_case_id,
                requested_variant_id=self.episode.variant_id,
                requested_pose=self.episode.reset_pose_name,
            )
        except ResetReceiptError as exc:
            self.guard.stop(f"reset_receipt_mismatch:{exc}")
            self._write("reset_receipt_rejected", detail=str(exc))
            return self.result()
        self._write("reset_receipt", **self.reset_receipt)
        self.guard.record_reset_receipt_generation(int(self.reset_receipt["generation"]))
        if not self._spin_until(
            lambda: self.guard.localization_ready or self.guard.state == "STOP",
            reset_timeout_sec,
        ):
            self.guard.stop("post_reset_readiness_timeout")
            return self.result()
        if self.guard.state == "STOP":
            return self.result()
        self._check_post_reset_odom()
        if self.guard.state == "STOP":
            return self.result()
        self._wait_nav2_and_tf_ready(reset_timeout_sec)
        if not (self.guard.nav2_active and self.guard.tf_active):
            return self.result()
        # The route coordinator drops goals published while the ResetStopGate
        # still holds this reset's generation, without retry; wait out the
        # release instead of publishing the single goal into the HOLD.
        if not self._spin_until(
            lambda: self.guard.goal_ready or self.guard.state == "STOP",
            reset_timeout_sec,
        ):
            self.guard.stop("reset_gate_release_timeout")
            return self.result()
        if self.guard.state == "STOP":
            return self.result()

        schedule_by_leg = {
            item.leg_id: item.group for item in self.manifest.dynamic_schedule
        }
        for index, leg in enumerate(self.manifest.mission_legs):
            self._run_mission_leg(
                index=index,
                leg=leg,
                dynamic_group=schedule_by_leg.get(leg.goal_id, ""),
                reset_timeout_sec=reset_timeout_sec,
                navigation_timeout_sec=navigation_timeout_sec,
            )
            if self.guard.state not in {"LEG_SUCCEEDED", "SUCCEEDED"}:
                break
        return self.result()

    def result(self) -> dict[str, Any]:
        if self.guard.state in {"SUCCEEDED", "FAILED", "STOP"}:
            cancel_navigation = self.guard.state in {"FAILED", "STOP"}
            self._start_terminal_settle(
                cancel_navigation=cancel_navigation,
                reason=self.guard.stop_reason or self.guard.state,
            )
            self._settle_terminal_zero()
        row = {
            "qualification": self.qualification,
            "formal_qualification": NOT_QUALIFIED,
            "state": self.guard.state,
            "stop_reason": self.guard.stop_reason,
            "reset_calls": self.guard.reset_calls,
            "reset_events": self.guard.reset_events,
            "reset_receipt": dict(getattr(self, "reset_receipt", None) or {}),
            "goal_publications": self.guard.goal_publications,
            "route_progress_messages": self.guard.route_progress_messages,
            "route_completion_messages": self.guard.route_completion_messages,
            "completed_leg_ids": list(self.guard.completed_leg_ids),
            "route_goal_results": list(self.route_goal_results),
            "dynamic_actions": list(self.dynamic_actions.events),
            "actor_states": self._actor_state_summary(),
            "collision": self.collision,
            "terminal_zero_confirmed": self._terminal_zero_confirmed,
            "terminal_zero_reason": self._terminal_zero_reason,
        }
        self._write("episode_result", **row)
        return row

    def _actor_state_summary(self) -> dict[str, list[str]]:
        summary: dict[str, set[str]] = {}
        for snapshot in self.obstacle_state_messages:
            obstacles = snapshot.get("obstacles", [])
            if not isinstance(obstacles, list):
                continue
            for obstacle in obstacles:
                if not isinstance(obstacle, Mapping):
                    continue
                obstacle_id = str(obstacle.get("id", ""))
                state = str(obstacle.get("state", ""))
                if obstacle_id and state:
                    summary.setdefault(obstacle_id, set()).add(state)
        return {name: sorted(states) for name, states in sorted(summary.items())}

    def destroy(self) -> None:
        self.node.destroy_node()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    manifest_group = parser.add_mutually_exclusive_group(required=True)
    manifest_group.add_argument("--manifest")
    manifest_group.add_argument("--formal-manifest")
    manifest_group.add_argument("--indoor-manifest")
    manifest_group.add_argument("--outdoor-manifest")
    manifest_group.add_argument("--pilot-manifest")
    manifest_group.add_argument("--indoor-pilot-manifest")
    manifest_group.add_argument("--outdoor-pilot-manifest")
    manifest_group.add_argument("--continue-indoor-parent")
    manifest_group.add_argument(
        "--combine-qualified-halves", nargs=2, metavar=("INDOOR", "OUTDOOR")
    )
    parser.add_argument("--pilot-aggregate")
    parser.add_argument("--output-manifest")
    parser.add_argument("--formal-output-root")
    parser.add_argument("--indoor-pilot-aggregate")
    parser.add_argument("--indoor-output-root")
    parser.add_argument("--outdoor-pilot-aggregate")
    parser.add_argument("--outdoor-output-root")
    manifest_group.add_argument("--aggregate-pilot-root")
    manifest_group.add_argument("--aggregate-indoor-pilot-root")
    manifest_group.add_argument("--aggregate-outdoor-pilot-root")
    parser.add_argument("--output-pilot-manifest")
    parser.add_argument("--output-pilot-aggregate")
    parser.add_argument("--continuation-output-manifest")
    parser.add_argument("--continuation-output-root")
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--pilot", action="store_true")
    parser.add_argument("--dispatch-pilot", action="store_true")
    parser.add_argument("--execute-formal", action="store_true")
    parser.add_argument("--execute-indoor", action="store_true")
    parser.add_argument("--execute-outdoor", action="store_true")
    parser.add_argument("--condition-stack-id")
    parser.add_argument("--condition-stack-contract")
    parser.add_argument("--output-jsonl")
    parser.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    parser.add_argument("--reset-timeout-sec", type=float, default=120.0)
    parser.add_argument("--navigation-timeout-sec", type=float, default=900.0)
    return parser


def cli(argv: list[str] | None = None) -> int:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    parser = build_parser()
    continuation_error = _continuation_cli_argv_error(raw_argv)
    if continuation_error is not None:
        parser.error(continuation_error)
    args = parser.parse_args(raw_argv)
    try:
        if args.combine_qualified_halves:
            forbidden = (
                args.pilot or args.dispatch_pilot or args.execute_formal
                or args.execute_indoor or args.execute_outdoor
                or args.condition_stack_id or args.condition_stack_contract
            )
            if forbidden:
                raise V6ContractError("combine-qualified-halves is read-only")
            print(json.dumps(combine_qualified_halves(
                args.combine_qualified_halves[0], args.combine_qualified_halves[1]
            ), sort_keys=True))
            return 0
        if args.continue_indoor_parent:
            if not args.continuation_output_manifest or not args.continuation_output_root:
                raise V6ContractError(
                    "indoor continuation requires successor manifest and output root"
                )
            if (
                args.pilot
                or args.dispatch_pilot
                or args.execute_formal
                or args.execute_indoor
                or args.condition_stack_id
                or args.condition_stack_contract
            ):
                raise V6ContractError("indoor continuation creation cannot dispatch")
            campaign = create_indoor_continuation_campaign(
                parent_manifest_path=args.continue_indoor_parent,
                output_manifest_path=args.continuation_output_manifest,
                successor_output_root=args.continuation_output_root,
            )
            print(json.dumps({
                "qualification": "INDOOR_CONTINUATION_READY",
                "formal_qualification": NOT_QUALIFIED,
                "indoor_progress": "1/60",
                "dispatch": False,
                "manifest": str(campaign.path),
                "freeze_digest": campaign.freeze_digest,
                "reason": "tooling_invalid_after_valid_episode",
            }, sort_keys=True))
            return 0
        if args.aggregate_indoor_pilot_root:
            if not args.output_pilot_manifest or not args.output_pilot_aggregate:
                raise V6ContractError(
                    "indoor Pilot aggregate mode requires both output paths"
                )
            if args.pilot or args.dispatch_pilot or args.execute_formal or args.execute_indoor or args.execute_outdoor:
                raise V6ContractError("indoor Pilot aggregate mode cannot dispatch")
            result = aggregate_indoor_pilot(
                pilot_root=args.aggregate_indoor_pilot_root,
                output_manifest=args.output_pilot_manifest,
                output_aggregate=args.output_pilot_aggregate,
            )
            print(json.dumps(result, sort_keys=True))
            return 0
        if args.aggregate_outdoor_pilot_root:
            if not args.output_pilot_manifest or not args.output_pilot_aggregate:
                raise V6ContractError(
                    "outdoor Pilot aggregate mode requires both output paths"
                )
            if (
                args.pilot or args.dispatch_pilot or args.execute_formal
                or args.execute_indoor or args.execute_outdoor
            ):
                raise V6ContractError("outdoor Pilot aggregate mode cannot dispatch")
            result = aggregate_outdoor_pilot(
                pilot_root=args.aggregate_outdoor_pilot_root,
                output_manifest=args.output_pilot_manifest,
                output_aggregate=args.output_pilot_aggregate,
            )
            print(json.dumps(result, sort_keys=True))
            return 0
        if args.aggregate_pilot_root:
            if not args.output_pilot_manifest or not args.output_pilot_aggregate:
                raise V6ContractError(
                    "Pilot aggregate mode requires both output paths"
                )
            if args.pilot or args.dispatch_pilot or args.execute_formal or args.execute_indoor or args.execute_outdoor:
                raise V6ContractError("Pilot aggregate mode cannot dispatch")
            result = aggregate_sufficient_pilot(
                pilot_root=args.aggregate_pilot_root,
                output_manifest=args.output_pilot_manifest,
                output_aggregate=args.output_pilot_aggregate,
            )
            print(json.dumps(result, sort_keys=True))
            return 0
        if args.indoor_pilot_manifest:
            if (
                not args.indoor_pilot_aggregate
                or not args.output_manifest
                or not args.indoor_output_root
            ):
                raise V6ContractError(
                    "indoor Pilot freezer requires aggregate, output manifest, and indoor output root"
                )
            if args.pilot or args.dispatch_pilot or args.execute_formal or args.execute_indoor or args.execute_outdoor:
                raise V6ContractError("indoor Pilot freezer cannot dispatch episodes")
            frozen = freeze_indoor_campaign_from_pilot(
                pilot_manifest_path=args.indoor_pilot_manifest,
                pilot_aggregate_path=args.indoor_pilot_aggregate,
                output_manifest_path=args.output_manifest,
                indoor_output_root=args.indoor_output_root,
            )
            print(json.dumps({
                "qualification": "INDOOR_CAMPAIGN_READY",
                "formal_qualification": NOT_QUALIFIED,
                "indoor_progress": "0/60",
                "dispatch": False,
                "manifest": str(frozen.path),
                "freeze_digest": frozen.freeze_digest,
            }, sort_keys=True))
            return 0
        if args.outdoor_pilot_manifest:
            if (
                not args.outdoor_pilot_aggregate
                or not args.output_manifest
                or not args.outdoor_output_root
            ):
                raise V6ContractError(
                    "outdoor Pilot freezer requires aggregate, output manifest, and output root"
                )
            if (
                args.pilot or args.dispatch_pilot or args.execute_formal
                or args.execute_indoor or args.execute_outdoor
            ):
                raise V6ContractError("outdoor Pilot freezer cannot dispatch")
            frozen = freeze_outdoor_campaign_from_pilot(
                pilot_manifest_path=args.outdoor_pilot_manifest,
                pilot_aggregate_path=args.outdoor_pilot_aggregate,
                output_manifest_path=args.output_manifest,
                outdoor_output_root=args.outdoor_output_root,
            )
            print(json.dumps({
                "qualification": "OUTDOOR_CAMPAIGN_READY",
                "formal_qualification": NOT_QUALIFIED,
                "outdoor_progress": "0/60",
                "dispatch": False,
                "manifest": str(frozen.path),
                "freeze_digest": frozen.freeze_digest,
            }, sort_keys=True))
            return 0
        if args.pilot_manifest:
            if (
                not args.pilot_aggregate
                or not args.output_manifest
                or not args.formal_output_root
            ):
                raise V6ContractError(
                    "Pilot freezer requires aggregate, output manifest, and formal output root"
                )
            if args.pilot or args.dispatch_pilot or args.execute_formal or args.execute_indoor:
                raise V6ContractError("Pilot freezer cannot dispatch episodes")
            frozen = freeze_formal_manifest_from_pilot(
                pilot_manifest_path=args.pilot_manifest,
                pilot_aggregate_path=args.pilot_aggregate,
                output_manifest_path=args.output_manifest,
                formal_output_root=args.formal_output_root,
            )
            print(json.dumps({
                "qualification": "FORMAL_READY_MANIFEST",
                "execution_authorization": frozen.authorization,
                "formal_progress": (
                    f"0/{len(frozen.conditions) * FORMAL_RUNS_PER_CONDITION}"
                ),
                "dispatch": False,
                "manifest": str(frozen.path),
                "freeze_digest": frozen.freeze_digest,
            }, sort_keys=True))
            return 0
        if args.indoor_manifest:
            if args.pilot or args.dispatch_pilot or args.output_jsonl or args.execute_formal or args.execute_outdoor:
                raise V6ContractError(
                    "indoor campaign manifest cannot use Pilot/formal options"
                )
            if args.execute_indoor and (
                not args.condition_stack_id or not args.condition_stack_contract
            ):
                raise V6ContractError(
                    "--execute-indoor requires --condition-stack-id and "
                    "--condition-stack-contract"
                )
            if not args.execute_indoor and (
                args.condition_stack_id or args.condition_stack_contract
            ):
                raise V6ContractError(
                    "condition stack options require --execute-indoor"
                )
            campaign = load_indoor_campaign_manifest(args.indoor_manifest)
            aggregate = (
                execute_indoor_campaign(
                    campaign,
                    condition_stack_id=args.condition_stack_id,
                    condition_stack_contract=args.condition_stack_contract,
                )
                if args.execute_indoor
                else evaluate_indoor_campaign(campaign)
            )
            plans = indoor_dispatch_plan(campaign, aggregate)
            print(json.dumps({
                "qualification": aggregate["qualification"],
                "formal_qualification": NOT_QUALIFIED,
                "freeze_digest": campaign.freeze_digest,
                "dispatch": args.execute_indoor,
                "aggregate": aggregate,
                "resume_points": {
                    row["id"]: row["next_run_index"]
                    for row in aggregate["conditions"]
                },
                "dispatch_plan": plans,
            }, sort_keys=True))
            return 0
        if args.outdoor_manifest:
            if (
                args.pilot or args.dispatch_pilot or args.output_jsonl
                or args.execute_formal or args.execute_indoor
            ):
                raise V6ContractError(
                    "outdoor campaign manifest cannot use Pilot/formal/indoor options"
                )
            if args.execute_outdoor and (
                not args.condition_stack_id or not args.condition_stack_contract
            ):
                raise V6ContractError(
                    "--execute-outdoor requires --condition-stack-id and contract"
                )
            if not args.execute_outdoor and (
                args.condition_stack_id or args.condition_stack_contract
            ):
                raise V6ContractError(
                    "condition stack options require --execute-outdoor"
                )
            campaign = load_outdoor_campaign_manifest(args.outdoor_manifest)
            aggregate = (
                execute_outdoor_campaign(
                    campaign,
                    condition_stack_id=args.condition_stack_id,
                    condition_stack_contract=args.condition_stack_contract,
                )
                if args.execute_outdoor
                else evaluate_outdoor_campaign(campaign)
            )
            print(json.dumps({
                "qualification": aggregate["qualification"],
                "formal_qualification": NOT_QUALIFIED,
                "freeze_digest": campaign.freeze_digest,
                "dispatch": args.execute_outdoor,
                "aggregate": aggregate,
                "resume_points": {
                    row["id"]: row["next_run_index"]
                    for row in aggregate["conditions"]
                },
                "dispatch_plan": outdoor_dispatch_plan(campaign, aggregate),
            }, sort_keys=True))
            return 0
        if args.formal_manifest:
            if args.pilot or args.dispatch_pilot or args.output_jsonl or args.execute_indoor or args.execute_outdoor:
                raise V6ContractError(
                    "formal campaign manifest cannot use engineering-pilot options"
                )
            if args.execute_formal and (
                not args.condition_stack_id or not args.condition_stack_contract
            ):
                raise V6ContractError(
                    "--execute-formal requires --condition-stack-id and "
                    "--condition-stack-contract"
                )
            if not args.execute_formal and (
                args.condition_stack_id or args.condition_stack_contract
            ):
                raise V6ContractError(
                    "condition stack options require --execute-formal"
                )
            campaign = load_formal_campaign_manifest(args.formal_manifest)
            aggregate = (
                execute_formal_campaign(
                    campaign,
                    condition_stack_id=args.condition_stack_id,
                    condition_stack_contract=args.condition_stack_contract,
                )
                if args.execute_formal
                else evaluate_formal_campaign(campaign)
            )
            plans = formal_dispatch_plan(campaign, aggregate)
            print(
                json.dumps(
                    {
                        "qualification": "FORMAL_CAMPAIGN",
                        "execution_authorization": campaign.authorization,
                        "freeze_digest": campaign.freeze_digest,
                        "dispatch": args.execute_formal,
                        "aggregate": aggregate,
                        "resume_points": {
                            row["id"]: row["next_run_index"]
                            for row in aggregate["conditions"]
                        },
                        "dispatch_plan": plans,
                    },
                    sort_keys=True,
                )
            )
            return 0
        if (
            args.pilot_aggregate
            or args.output_manifest
            or args.formal_output_root
            or args.output_pilot_manifest
            or args.output_pilot_aggregate
            or args.indoor_pilot_aggregate
            or args.indoor_output_root
            or args.outdoor_pilot_aggregate
            or args.outdoor_output_root
            or args.continuation_output_manifest
            or args.continuation_output_root
        ):
            raise V6ContractError("Pilot freezer options require --pilot-manifest")
        if (
            args.execute_formal
            or args.execute_indoor
            or args.execute_outdoor
            or args.condition_stack_id
            or args.condition_stack_contract
        ):
            raise V6ContractError(
                "formal execution options require --formal-manifest"
            )
        if args.dispatch_pilot and not args.pilot:
            raise V6ContractError("--dispatch-pilot requires --pilot")
        assert args.manifest is not None
        manifest = load_manifest(args.manifest)
        mode = "pilot" if args.pilot else "formal"
        qualification = authorize_manifest(manifest, mode=mode)
        if args.pilot and not args.dispatch_pilot:
            print(json.dumps({
                "qualification": ENGINEERING_PILOT,
                "formal_qualification": qualification,
                "dispatch": False,
                "runtime": dict(manifest.runtime),
                "assets": dict(manifest.assets),
                "dynamic_schedule": [vars(item) for item in manifest.dynamic_schedule],
            }, sort_keys=True))
            return 0
        if args.output_jsonl is None:
            raise V6ContractError("pilot dispatch requires --output-jsonl")
        if not 0 <= args.episode_index < len(manifest.episodes):
            raise V6ContractError("episode-index out of range")
        import rclpy
        rclpy.init(args=None)
        adapter = V6FormalNode(
            manifest,
            manifest.episodes[args.episode_index],
            Path(args.output_jsonl).expanduser().resolve(),
            qualification=ENGINEERING_PILOT,
        )
        try:
            result = adapter.run(
                readiness_timeout_sec=args.readiness_timeout_sec,
                reset_timeout_sec=args.reset_timeout_sec,
                navigation_timeout_sec=args.navigation_timeout_sec,
            )
            print(json.dumps(result, sort_keys=True))
            return 0 if result["state"] == "SUCCEEDED" else 2
        finally:
            adapter.destroy()
            rclpy.shutdown()
    except (
        OSError,
        subprocess.CalledProcessError,
        ConfigurationError,
        V6ContractError,
        yaml.YAMLError,
    ) as exc:
        print(f"STOP: {exc}", file=sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(cli())


if __name__ == "__main__":
    main()
