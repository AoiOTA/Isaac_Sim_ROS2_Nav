"""V6 estimated-autonomy episode dispatcher and engineering pilot adapter.

This module deliberately does not share the legacy experiment runner.  The
dispatcher owns reset and RouteCoordinator goal sequencing only.  Ground Truth
is reserved for the independent ``estimated_state_evaluator``/recorder.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass, field, replace
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
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
from robot_experiments.scenario import load_scenario


SCHEMA_VERSION = "bio_nav_v6_r3_phase2_pilot_manifest_v1"
FORMAL_CAMPAIGN_SCHEMA_VERSION = "bio_nav_v6_formal_campaign_v1"
SUFFICIENT_PILOT_MANIFEST_SCHEMA = "bio_nav_v6_sufficient_pilot_manifest_v1"
SUFFICIENT_PILOT_AGGREGATE_SCHEMA = "bio_nav_v6_sufficient_pilot_aggregate_v1"
FORMAL_NAS_ROOT = Path("/mnt/nas_home")
FORMAL_CONDITION_IDS = (
    "indoor_static",
    "outdoor_static",
    "outdoor_dynamic",
    "outdoor_appearance",
    "indoor_dynamic",
    "indoor_appearance",
)
FORMAL_RUNS_PER_CONDITION = 20
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


def _validate_formal_freeze(
    value: Any,
    *,
    conditions: tuple[FormalCondition, ...],
    scenario_configs: Mapping[str, set[Path]],
    runner_entrypoint: Path,
) -> tuple[dict[str, Any], str]:
    freeze = _mapping(value, "freeze")
    _require_exact_keys(
        freeze,
        {
            "repositories",
            "driver_version",
            "kernel_release",
            "scenarios",
            "scenario_configs",
            "frozen_assets",
            "runner_entrypoint",
            "experiment_runner",
            "v6_formal",
        },
        "freeze",
    )
    repositories = _mapping(freeze.get("repositories"), "freeze.repositories")
    _require_exact_keys(
        repositories, {"integration", "module2", "module3"}, "freeze.repositories"
    )
    normalized_repositories: dict[str, dict[str, str]] = {}
    for name, value_entry in repositories.items():
        entry = _mapping(value_entry, f"freeze.repositories.{name}")
        _require_exact_keys(entry, {"path", "head"}, f"freeze.repositories.{name}")
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
        tracked_clean = subprocess.run(
            ["git", "-C", str(repo_path), "diff", "--quiet", "HEAD", "--"]
        )
        index_clean = subprocess.run(
            ["git", "-C", str(repo_path), "diff", "--cached", "--quiet"]
        )
        if tracked_clean.returncode != 0 or index_clean.returncode != 0:
            raise V6ContractError(f"freeze repository tracked state is dirty: {name}")
        normalized_repositories[name] = {"path": str(repo_path), "head": actual_head}
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
    for condition_id in FORMAL_CONDITION_IDS:
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
    _require_exact_keys(asset_rows, FORMAL_FROZEN_ASSET_KEYS, "freeze.frozen_assets")
    normalized_assets = {}
    for name in sorted(FORMAL_FROZEN_ASSET_KEYS):
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
    canonical = json.dumps(normalized, sort_keys=True, separators=(",", ":"))
    return normalized, hashlib.sha256(canonical.encode()).hexdigest()


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
        scenario_configs[condition_id] = {
            scenario.resolve_path(path)
            for path in (
                scenario.robot_config_file,
                scenario.nav2_config_file,
                scenario.dynamic_config_file,
                scenario.appearance_config_file,
                scenario.optimal_reference_file,
            )
            if path is not None
        }
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


def _pilot_selection_identity(condition: FormalCondition, rep: int) -> Mapping[str, Any]:
    return condition.episode_identities[rep - 1]


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
        and episode.get("formal_freeze_digest") == freeze_digest
        and summary.get("formal_freeze_digest") == freeze_digest
    ):
        raise V6ContractError("Pilot episode frozen tuple/session mismatch")
    sequence_receipt = episode.get("stack_episode_receipt", {})
    summary_sequence_receipt = summary.get("condition_stack_attestation", {}).get(
        "stack_episode_receipt"
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
    ):
        raise V6ContractError("Pilot stack episode sequence/T2 receipt mismatch")
    reset_receipt = episode.get("reset_receipt", {})
    generation = reset_receipt.get("generation") if isinstance(reset_receipt, Mapping) else None
    if isinstance(generation, bool) or not isinstance(generation, int) or generation <= 0:
        raise V6ContractError("Pilot episode reset generation is invalid")
    scenario = load_scenario(condition.scenario_file)
    expected_robot_hash = _file_sha256(scenario.resolve_path(scenario.robot_config_file))
    expected_nav2_hash = _file_sha256(scenario.resolve_path(scenario.nav2_config_file))
    expected_runtime_hashes = {
        name: _file_sha256(scenario.resolve_path(path))
        for name, path in (
            ("robot_config", scenario.robot_config_file),
            ("nav2_config", scenario.nav2_config_file),
            ("dynamic_config", scenario.dynamic_config_file),
            ("appearance_config", scenario.appearance_config_file),
            ("optimal_reference", scenario.optimal_reference_file),
        )
        if path is not None
    }
    provenance = episode.get("provenance", {})
    if not (
        episode.get("robot_config_hash") == expected_robot_hash
        and episode.get("nav2_config_hash") == expected_nav2_hash
        and episode.get("scenario_runtime_hashes") == expected_runtime_hashes
        and isinstance(provenance, Mapping)
        and provenance.get("git_head") == freeze["repositories"]["module3"]["head"]
        and provenance.get("git_dirty") is False
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


STACK_CONTRACT_SCHEMA = "bio_nav.v6_stack_contract.v1"
STACK_CONTRACT_KEYS = {
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


def _stack_session_id(payload: Mapping[str, Any]) -> str:
    basis = {key: payload[key] for key in STACK_CONTRACT_KEYS - {"stack_session_id"}}
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
    canonical = json.dumps(normalized, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode()).hexdigest()


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
    _require_exact_keys(payload, STACK_CONTRACT_KEYS, "pilot_stack_contract")
    if (
        payload.get("schema") != STACK_CONTRACT_SCHEMA
        or payload.get("condition_id") != expected_condition_id
        or payload.get("condition_id") != f"{payload.get('scene')}_{payload.get('condition')}"
        or payload.get("stack_session_id") != _stack_session_id(payload)
    ):
        raise V6ContractError("Pilot stack contract identity/digest mismatch")
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
    _require_exact_keys(payload, STACK_CONTRACT_KEYS, "condition_stack_contract")
    if payload["schema"] != STACK_CONTRACT_SCHEMA:
        raise V6ContractError("condition stack contract schema mismatch")
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


def evaluate_formal_campaign(
    manifest: FormalCampaignManifest,
) -> dict[str, Any]:
    condition_results: list[dict[str, Any]] = []
    total_strict = 0
    total_valid = 0
    total_present = 0
    blockers: list[str] = []
    for condition in manifest.conditions:
        runs: list[dict[str, Any]] = []
        next_run_index: int | None = None
        condition_blockers: list[str] = []
        stack_session_ids: set[str] = set()
        present_run_indices: set[int] = set()
        reset_generations: dict[int, int] = {}
        for run_index, identity in enumerate(
            condition.episode_identities, start=1
        ):
            seed = int(identity["seed"])
            root = (
                condition.output_directory
                / condition.scenario_id
                / f"run-{run_index:04d}-seed-{seed}"
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
                stack_identity_ok = bool(
                    episode.get("condition_stack_id") == condition.condition_id
                    and summary.get("condition_stack_id") == condition.condition_id
                    and isinstance(episode_stack_session, str)
                    and bool(episode_stack_session)
                    and summary_stack_session == episode_stack_session
                    and reset_identity_ok
                    and episode.get("formal_freeze_digest")
                    == manifest.freeze_digest
                    and summary.get("formal_freeze_digest")
                    == manifest.freeze_digest
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
                valid = bool(
                    identity_ok
                    and summary.get("episode_validity", {}).get("valid") is True
                    and summary.get("checksums_verified") is True
                    and summary.get("final_trial_metric_gate", {}).get("passed")
                    is True
                    and _checksums_verified(root)
                )
                strict = bool(valid and summary.get("strict_success") is True)
                if strict:
                    status = "strict_success"
                    total_strict += 1
                elif valid:
                    status = "product_failure"
                    condition_blockers.append(f"run-{run_index}:product_failure")
                else:
                    status = "invalid_evidence"
                    condition_blockers.append(f"run-{run_index}:invalid_evidence")
                if valid:
                    total_valid += 1
                detail = ""
            runs.append(
                {
                    "run_index": run_index,
                    "seed": seed,
                    "status": status,
                    "detail": detail,
                    "root": str(root),
                }
            )
        if len(stack_session_ids) > 1:
            condition_blockers.append("stack_session_mismatch")
        if present_run_indices != set(reset_generations):
            condition_blockers.append("reset_generation_missing")
        elif present_run_indices:
            ordered_indices = sorted(present_run_indices)
            base_generation = reset_generations[ordered_indices[0]]
            if ordered_indices != list(range(1, ordered_indices[-1] + 1)) or any(
                reset_generations[index] != base_generation + index - 1
                for index in ordered_indices
            ):
                condition_blockers.append("reset_generation_discontinuous")
        blockers.extend(
            f"{condition.condition_id}:{item}" for item in condition_blockers
        )
        condition_results.append(
            {
                "id": condition.condition_id,
                "strict_successes": sum(
                    run["status"] == "strict_success" for run in runs
                ),
                "valid_episodes": sum(
                    run["status"] in {"strict_success", "product_failure"}
                    for run in runs
                ),
                "next_run_index": None if condition_blockers else next_run_index,
                "blockers": condition_blockers,
                "stack_session_id": (
                    next(iter(stack_session_ids))
                    if len(stack_session_ids) == 1
                    else None
                ),
                "reset_generation_base": reset_generations.get(1),
                "runs": runs,
            }
        )
    return {
        "schema_version": FORMAL_CAMPAIGN_SCHEMA_VERSION,
        "formal_qualification": (
            "PASS"
            if (
                manifest.authorization == FORMAL_EXECUTION_AUTHORIZED
                and total_strict == 120
                and not blockers
            )
            else "INCOMPLETE"
        ),
        "execution_authorization": manifest.authorization,
        "expected_episodes": 120,
        "present_episodes": total_present,
        "valid_episodes": total_valid,
        "strict_successes": total_strict,
        "blockers": blockers,
        "conditions": condition_results,
    }


def formal_dispatch_plan(
    manifest: FormalCampaignManifest, aggregate: Mapping[str, Any]
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
    expected = {
        (condition_stack_id, row["run_index"], "strict_success")
    }
    if before_completed - after_completed or after_completed - before_completed != expected:
        raise V6ContractError(
            "formal episode did not add exactly one strict-success target run"
        )
    return after


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
    parser = argparse.ArgumentParser(description=__doc__)
    manifest_group = parser.add_mutually_exclusive_group(required=True)
    manifest_group.add_argument("--manifest")
    manifest_group.add_argument("--formal-manifest")
    manifest_group.add_argument("--pilot-manifest")
    parser.add_argument("--pilot-aggregate")
    parser.add_argument("--output-manifest")
    parser.add_argument("--formal-output-root")
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--pilot", action="store_true")
    parser.add_argument("--dispatch-pilot", action="store_true")
    parser.add_argument("--execute-formal", action="store_true")
    parser.add_argument("--condition-stack-id")
    parser.add_argument("--condition-stack-contract")
    parser.add_argument("--output-jsonl")
    parser.add_argument("--readiness-timeout-sec", type=float, default=120.0)
    parser.add_argument("--reset-timeout-sec", type=float, default=120.0)
    parser.add_argument("--navigation-timeout-sec", type=float, default=900.0)
    return parser


def cli(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.pilot_manifest:
            if (
                not args.pilot_aggregate
                or not args.output_manifest
                or not args.formal_output_root
            ):
                raise V6ContractError(
                    "Pilot freezer requires aggregate, output manifest, and formal output root"
                )
            if args.pilot or args.dispatch_pilot or args.execute_formal:
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
                "formal_progress": "0/120",
                "dispatch": False,
                "manifest": str(frozen.path),
                "freeze_digest": frozen.freeze_digest,
            }, sort_keys=True))
            return 0
        if args.formal_manifest:
            if args.pilot or args.dispatch_pilot or args.output_jsonl:
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
        if args.pilot_aggregate or args.output_manifest or args.formal_output_root:
            raise V6ContractError("Pilot freezer options require --pilot-manifest")
        if (
            args.execute_formal
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
