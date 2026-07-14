"""Pure report validation, hashing, and atomic JSON/CSV writers."""

from __future__ import annotations

import csv
import hashlib
import hmac
import json
import math
import os
import re
from pathlib import Path
import tempfile
from typing import Any, Callable, Mapping, TextIO


REPRODUCIBILITY_FIELDS = (
    "scenario_id",
    "random_seed",
    "map_version",
    "posegraph_version",
    "robot_config_hash",
    "nav2_config_hash",
    "dynamic_runtime_contract",
    "spawn_pose_name",
    "usd_start_pose",
    "map_start_pose",
    "goal_pose",
    "obstacle_trajectories",
    "physics_dt",
    "rtf",
    "result",
    "failure_reason",
)
_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_PRIM_PATH_PATTERN = re.compile(
    r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
_LOWERCASE_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_PROFILE_FLAGS = {
    "legacy_baseline": (False, False),
    "threshold_only": (False, True),
    "explicit_material": (True, True),
}
_COMBINE_MODES = {"average", "min", "multiply", "max"}
_CONTACT_KEYS = {
    "profile_path",
    "profile_sha256",
    "profile_id",
    "profile_mode",
    "overlay_identifier",
    "overlay_sha256",
    "explicit_materials",
    "thresholds_authored",
    "scene",
    "collider_contract",
    "wheel_colliders",
    "ground_colliders",
    "wheel_bindings",
    "ground_bindings",
    "wheel_material",
    "ground_material",
    "stage_usd_readback_verified",
}
_GROUND_TOPOLOGY_KEYS = {
    "profile_path",
    "profile_sha256",
    "profile_id",
    "environment_id",
    "operation",
    "source_asset_path",
    "source_asset_sha256",
    "overlay_identifier",
    "overlay_sha256",
    "source_colliders",
    "source_collider_count",
    "source_collider_paths_sha256",
    "target_colliders",
    "target_collider_count",
    "target_collider_paths_sha256",
    "disabled_colliders",
    "disabled_collider_count",
    "disabled_collider_paths_sha256",
    "stage_usd_readback_verified",
}
_GROUND_TOPOLOGY_OPERATIONS = {
    "preserve_source_colliders",
    "disable_non_target_colliders",
}


class ReportValidationError(ValueError):
    """Raised when a report is incomplete or not strict-JSON compatible."""


def configuration_sha256(path: str | os.PathLike[str]) -> str:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f"configuration file does not exist: {source}")
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_json_value(value: Any, location: str) -> None:
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ReportValidationError(f"{location} contains NaN or infinity")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            if not isinstance(key, str):
                raise ReportValidationError(f"{location} contains a non-string key")
            _validate_json_value(child, f"{location}.{key}")
        return
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            _validate_json_value(child, f"{location}[{index}]")
        return
    raise ReportValidationError(f"{location} has unsupported type {type(value).__name__}")


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    missing = [field for field in REPRODUCIBILITY_FIELDS if field not in manifest]
    if missing:
        raise ReportValidationError(f"missing reproducibility fields: {', '.join(missing)}")
    _validate_json_value(manifest, "manifest")
    for hash_field in ("robot_config_hash", "nav2_config_hash"):
        value = manifest[hash_field]
        if not isinstance(value, str) or len(value) != 64:
            raise ReportValidationError(f"{hash_field} must be a SHA256 hex digest")
        try:
            int(value, 16)
        except ValueError as exc:
            raise ReportValidationError(f"{hash_field} must be a SHA256 hex digest") from exc
    dynamic_contract = manifest["dynamic_runtime_contract"]
    if not isinstance(dynamic_contract, Mapping):
        raise ReportValidationError(
            "dynamic_runtime_contract must be a mapping"
        )
    if dynamic_contract.get("verified") is not True:
        raise ReportValidationError(
            "dynamic_runtime_contract must be runtime-verified"
        )
    dynamic_hash = dynamic_contract.get("config_sha256")
    if not isinstance(dynamic_hash, str) or len(dynamic_hash) != 64:
        raise ReportValidationError(
            "dynamic_runtime_contract.config_sha256 must be a SHA256 hex digest"
        )
    try:
        int(dynamic_hash, 16)
    except ValueError as exc:
        raise ReportValidationError(
            "dynamic_runtime_contract.config_sha256 must be a SHA256 hex digest"
        ) from exc


def _required_mapping(
    value: Mapping[str, Any], key: str, location: str
) -> Mapping[str, Any]:
    child = value.get(key)
    if not isinstance(child, Mapping):
        raise ReportValidationError(f"{location}.{key} must be a mapping")
    return child


def _required_string(value: Any, location: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ReportValidationError(f"{location} must be a non-empty string")
    return value


def _validate_sha256(value: Any, location: str) -> None:
    if not isinstance(value, str) or len(value) != 64:
        raise ReportValidationError(f"{location} must be a SHA256 hex digest")
    try:
        int(value, 16)
    except ValueError as exc:
        raise ReportValidationError(
            f"{location} must be a SHA256 hex digest"
        ) from exc


def _validate_lowercase_sha256(value: Any, location: str) -> str:
    if not isinstance(value, str) or not _LOWERCASE_SHA256_PATTERN.fullmatch(
        value
    ):
        raise ReportValidationError(
            f"{location} must be a lowercase SHA256 hex digest"
        )
    return value


def _canonical_path_sha256(paths: list[str]) -> str:
    canonical = json.dumps(
        sorted(paths),
        separators=(",", ":"),
        allow_nan=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _ground_topology_path_sequence(
    value: Any,
    location: str,
    *,
    allow_empty: bool,
) -> list[str]:
    if not isinstance(value, list):
        raise ReportValidationError(f"{location} must be a JSON list")
    if not allow_empty and not value:
        raise ReportValidationError(f"{location} must not be empty")
    if not all(
        isinstance(path, str) and _PRIM_PATH_PATTERN.fullmatch(path)
        for path in value
    ):
        raise ReportValidationError(
            f"{location} must contain valid absolute USD prim paths"
        )
    if value != sorted(value):
        raise ReportValidationError(f"{location} must be sorted")
    if len(set(value)) != len(value):
        raise ReportValidationError(f"{location} must contain unique paths")
    return value


def _topology_count(value: Any, location: str, *, allow_zero: bool) -> int:
    minimum = 0 if allow_zero else 1
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        qualifier = "non-negative" if allow_zero else "positive"
        raise ReportValidationError(
            f"{location} must be a {qualifier} integer"
        )
    return value


def decode_hashed_contact_snapshot(
    payload: object,
    expected_sha256: object,
) -> dict[str, Any]:
    """Verify and decode the canonical contact JSON exposed by Isaac."""

    if not isinstance(payload, str):
        raise ReportValidationError(
            "runtime_provenance.contact.json must be a string"
        )
    _validate_sha256(
        expected_sha256,
        "runtime_provenance.contact.sha256",
    )
    actual_sha256 = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    if not hmac.compare_digest(actual_sha256, expected_sha256):
        raise ReportValidationError(
            "runtime_provenance.contact JSON SHA256 mismatch"
        )

    def reject_constant(token: str) -> None:
        raise ValueError(f"non-finite JSON constant {token}")

    def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"duplicate JSON key {key!r}")
            result[key] = value
        return result

    try:
        decoded = json.loads(
            payload,
            parse_constant=reject_constant,
            object_pairs_hook=reject_duplicate_keys,
        )
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ReportValidationError(
            f"runtime_provenance.contact.json must be valid JSON: {exc}"
        ) from exc
    if not isinstance(decoded, dict):
        raise ReportValidationError(
            "runtime_provenance.contact.json root must be a mapping"
        )
    canonical = json.dumps(
        decoded,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    if canonical != payload:
        raise ReportValidationError(
            "runtime_provenance.contact.json must be canonical strict JSON"
        )
    return decoded


def _absolute_file_path(value: Any, location: str) -> str:
    path = _required_string(value, location)
    if not Path(path).is_absolute():
        raise ReportValidationError(f"{location} must be an absolute path")
    return path


def _absolute_prim_path(value: Any, location: str) -> str:
    path = _required_string(value, location)
    if not path.startswith("/") or path == "/" or "//" in path:
        raise ReportValidationError(
            f"{location} must be an absolute prim path"
        )
    return path


def _finite_nonnegative(value: Any, location: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) < 0.0
    ):
        raise ReportValidationError(
            f"{location} must be a finite non-negative number"
        )
    return float(value)


def _string_sequence(
    value: Any,
    location: str,
    *,
    allow_empty: bool,
) -> list[str]:
    if not isinstance(value, (list, tuple)):
        raise ReportValidationError(f"{location} must be a list")
    values = [
        _required_string(item, f"{location}[{index}]")
        for index, item in enumerate(value)
    ]
    if not allow_empty and not values:
        raise ReportValidationError(f"{location} must not be empty")
    if len(set(values)) != len(values):
        raise ReportValidationError(f"{location} must contain unique values")
    return values


def _prim_path_sequence(
    value: Any,
    location: str,
    *,
    expected_count: int | None = None,
) -> list[str]:
    values = _string_sequence(value, location, allow_empty=False)
    paths = [
        _absolute_prim_path(path, f"{location}[{index}]")
        for index, path in enumerate(values)
    ]
    if expected_count is not None and len(paths) != expected_count:
        raise ReportValidationError(
            f"{location} must contain exactly {expected_count} paths"
        )
    return paths


def _validate_contact_material(
    value: Any,
    location: str,
) -> Mapping[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ReportValidationError(f"{location} must be a mapping or null")
    required = {
        "material_path",
        "static_friction",
        "dynamic_friction",
        "restitution",
        "friction_combine_mode",
        "restitution_combine_mode",
        "friction_combine_mode_authored",
        "restitution_combine_mode_authored",
    }
    if set(value) != required:
        raise ReportValidationError(
            f"{location} keys must be exactly {sorted(required)}"
        )
    _absolute_prim_path(value["material_path"], f"{location}.material_path")
    static_friction = _finite_nonnegative(
        value["static_friction"],
        f"{location}.static_friction",
    )
    dynamic_friction = _finite_nonnegative(
        value["dynamic_friction"],
        f"{location}.dynamic_friction",
    )
    if dynamic_friction > static_friction:
        raise ReportValidationError(
            f"{location}.dynamic_friction must not exceed static_friction"
        )
    restitution = _finite_nonnegative(
        value["restitution"],
        f"{location}.restitution",
    )
    if restitution > 1.0:
        raise ReportValidationError(f"{location}.restitution must be in [0, 1]")
    for name in ("friction_combine_mode", "restitution_combine_mode"):
        combine_mode = value[name]
        if combine_mode is not None and (
            not isinstance(combine_mode, str)
            or combine_mode not in _COMBINE_MODES
        ):
            raise ReportValidationError(
                f"{location}.{name} must be null or a supported combine mode"
            )
    for name in (
        "friction_combine_mode_authored",
        "restitution_combine_mode_authored",
    ):
        if not isinstance(value[name], bool):
            raise ReportValidationError(f"{location}.{name} must be boolean")
    return value


def _validate_contact_bindings(
    value: Any,
    location: str,
    collider_paths: list[str],
    material: Mapping[str, Any] | None,
    *,
    explicit: bool,
) -> None:
    if not isinstance(value, (list, tuple)) or len(value) != len(collider_paths):
        raise ReportValidationError(
            f"{location} must contain one binding per collider"
        )
    required = {
        "collider_path",
        "direct_physics_material_path",
        "effective_physics_material_path",
    }
    seen: list[str] = []
    direct_paths: list[str | None] = []
    effective_paths: list[str | None] = []
    for index, binding in enumerate(value):
        child_location = f"{location}[{index}]"
        if not isinstance(binding, Mapping) or set(binding) != required:
            raise ReportValidationError(
                f"{child_location} must contain collider, direct, and "
                "effective material paths"
            )
        seen.append(
            _absolute_prim_path(
                binding["collider_path"],
                f"{child_location}.collider_path",
            )
        )
        for name, destination in (
            ("direct_physics_material_path", direct_paths),
            ("effective_physics_material_path", effective_paths),
        ):
            path = binding[name]
            destination.append(
                None
                if path is None
                else _absolute_prim_path(path, f"{child_location}.{name}")
            )
    if set(seen) != set(collider_paths) or len(set(seen)) != len(seen):
        raise ReportValidationError(
            f"{location} must have a one-to-one collider mapping"
        )
    material_path = None if material is None else material["material_path"]
    if material_path is None:
        if any(path is not None for path in effective_paths):
            raise ReportValidationError(
                f"{location} effective binding requires material evidence"
            )
    elif set(effective_paths) != {material_path}:
        raise ReportValidationError(
            f"{location} effective binding does not match material evidence"
        )
    if explicit and set(direct_paths) != {material_path}:
        raise ReportValidationError(
            f"{location} direct binding does not match explicit material"
        )


def _validate_contact_provenance(contact: Mapping[str, Any]) -> None:
    location = "runtime_provenance.contact"
    if set(contact) != _CONTACT_KEYS:
        raise ReportValidationError(
            f"{location} keys must be exactly {sorted(_CONTACT_KEYS)}"
        )
    _absolute_file_path(contact["profile_path"], f"{location}.profile_path")
    _validate_sha256(contact["profile_sha256"], f"{location}.profile_sha256")
    profile_id = _required_string(contact["profile_id"], f"{location}.profile_id")
    if not _IDENTIFIER_PATTERN.fullmatch(profile_id):
        raise ReportValidationError(f"{location}.profile_id must be path-safe")
    profile_mode = contact["profile_mode"]
    if not isinstance(profile_mode, str) or profile_mode not in _PROFILE_FLAGS:
        raise ReportValidationError(
            f"{location}.profile_mode must be one of {sorted(_PROFILE_FLAGS)}"
        )
    overlay_identifier = _required_string(
        contact["overlay_identifier"],
        f"{location}.overlay_identifier",
    )
    if not overlay_identifier.startswith("anon:"):
        raise ReportValidationError(
            f"{location}.overlay_identifier must identify an anonymous layer"
        )
    _validate_sha256(contact["overlay_sha256"], f"{location}.overlay_sha256")
    if contact["stage_usd_readback_verified"] is not True:
        raise ReportValidationError(
            f"{location}.stage_usd_readback_verified must be true"
        )
    flags = (contact["explicit_materials"], contact["thresholds_authored"])
    if not all(isinstance(flag, bool) for flag in flags):
        raise ReportValidationError(f"{location} profile flags must be boolean")
    if flags != _PROFILE_FLAGS[profile_mode]:
        raise ReportValidationError(
            f"{location} flags disagree with {profile_mode} mode"
        )

    scene = _required_mapping(contact, "scene", location)
    scene_keys = {
        "physics_scene_path",
        "friction_correlation_distance",
        "friction_offset_threshold",
        "friction_type",
    }
    if set(scene) != scene_keys:
        raise ReportValidationError(
            f"{location}.scene keys must be exactly {sorted(scene_keys)}"
        )
    _absolute_prim_path(
        scene["physics_scene_path"],
        f"{location}.scene.physics_scene_path",
    )
    _finite_nonnegative(
        scene["friction_correlation_distance"],
        f"{location}.scene.friction_correlation_distance",
    )
    _finite_nonnegative(
        scene["friction_offset_threshold"],
        f"{location}.scene.friction_offset_threshold",
    )
    if scene["friction_type"] is not None:
        _required_string(
            scene["friction_type"],
            f"{location}.scene.friction_type",
        )

    contract = _required_mapping(contact, "collider_contract", location)
    contract_keys = {
        "wheel_joint_names",
        "wheel_expected_count",
        "ground_required_prim_paths",
        "ground_semantic_classes",
        "ground_expected_enabled_count",
    }
    if set(contract) != contract_keys:
        raise ReportValidationError(
            f"{location}.collider_contract keys must be exactly "
            f"{sorted(contract_keys)}"
        )
    wheel_expected_count = contract["wheel_expected_count"]
    if (
        isinstance(wheel_expected_count, bool)
        or not isinstance(wheel_expected_count, int)
        or wheel_expected_count != 4
    ):
        raise ReportValidationError(
            f"{location}.collider_contract.wheel_expected_count must be 4"
        )
    wheel_joint_names = _string_sequence(
        contract["wheel_joint_names"],
        f"{location}.collider_contract.wheel_joint_names",
        allow_empty=False,
    )
    if len(wheel_joint_names) != wheel_expected_count:
        raise ReportValidationError(
            f"{location}.collider_contract.wheel_joint_names must contain 4 names"
        )
    ground_expected_count = contract["ground_expected_enabled_count"]
    if (
        isinstance(ground_expected_count, bool)
        or not isinstance(ground_expected_count, int)
        or ground_expected_count < 1
    ):
        raise ReportValidationError(
            f"{location}.collider_contract.ground_expected_enabled_count "
            "must be a positive integer"
        )
    required_ground = _prim_path_sequence(
        contract["ground_required_prim_paths"],
        f"{location}.collider_contract.ground_required_prim_paths",
    )
    if len(required_ground) > ground_expected_count:
        raise ReportValidationError(
            f"{location}.collider_contract has more required ground paths "
            "than its expected count"
        )
    semantic_classes = _string_sequence(
        contract["ground_semantic_classes"],
        f"{location}.collider_contract.ground_semantic_classes",
        allow_empty=True,
    )
    if any(not _IDENTIFIER_PATTERN.fullmatch(value) for value in semantic_classes):
        raise ReportValidationError(
            f"{location}.collider_contract.ground_semantic_classes "
            "must be path-safe"
        )

    wheel_colliders = _prim_path_sequence(
        contact["wheel_colliders"],
        f"{location}.wheel_colliders",
        expected_count=wheel_expected_count,
    )
    ground_colliders = _prim_path_sequence(
        contact["ground_colliders"],
        f"{location}.ground_colliders",
        expected_count=ground_expected_count,
    )
    missing_ground = sorted(set(required_ground) - set(ground_colliders))
    if missing_ground:
        raise ReportValidationError(
            f"{location}.ground_colliders is missing required paths: "
            f"{missing_ground}"
        )

    wheel_material = _validate_contact_material(
        contact["wheel_material"],
        f"{location}.wheel_material",
    )
    ground_material = _validate_contact_material(
        contact["ground_material"],
        f"{location}.ground_material",
    )
    explicit = profile_mode == "explicit_material"
    if explicit and (wheel_material is None or ground_material is None):
        raise ReportValidationError(
            f"{location} explicit_material mode requires two material snapshots"
        )
    _validate_contact_bindings(
        contact["wheel_bindings"],
        f"{location}.wheel_bindings",
        wheel_colliders,
        wheel_material,
        explicit=explicit,
    )
    _validate_contact_bindings(
        contact["ground_bindings"],
        f"{location}.ground_bindings",
        ground_colliders,
        ground_material,
        explicit=explicit,
    )
    if explicit:
        assert wheel_material is not None
        assert ground_material is not None
        for material, group in (
            (wheel_material, "wheel"),
            (ground_material, "ground"),
        ):
            for combine in ("friction", "restitution"):
                if (
                    material[f"{combine}_combine_mode"] not in _COMBINE_MODES
                    or material[f"{combine}_combine_mode_authored"] is not True
                ):
                    raise ReportValidationError(
                        f"{location}.{group}_material explicit {combine} "
                        "combine mode must be authored"
                    )
        if (
            wheel_material["friction_combine_mode"]
            != ground_material["friction_combine_mode"]
            or wheel_material["restitution_combine_mode"]
            != ground_material["restitution_combine_mode"]
        ):
            raise ReportValidationError(
                f"{location} explicit material combine modes must agree"
            )


def _validate_ground_topology_provenance(
    topology: Mapping[str, Any],
    environment: Mapping[str, Any],
    contact: Mapping[str, Any],
) -> None:
    location = "runtime_provenance.ground_topology"
    if set(topology) != _GROUND_TOPOLOGY_KEYS:
        raise ReportValidationError(
            f"{location} keys must be exactly "
            f"{sorted(_GROUND_TOPOLOGY_KEYS)}"
        )

    _absolute_file_path(topology["profile_path"], f"{location}.profile_path")
    _validate_lowercase_sha256(
        topology["profile_sha256"],
        f"{location}.profile_sha256",
    )
    profile_id = _required_string(
        topology["profile_id"],
        f"{location}.profile_id",
    )
    if not _IDENTIFIER_PATTERN.fullmatch(profile_id):
        raise ReportValidationError(f"{location}.profile_id must be path-safe")

    environment_id = _required_string(
        topology["environment_id"],
        f"{location}.environment_id",
    )
    if not _IDENTIFIER_PATTERN.fullmatch(environment_id):
        raise ReportValidationError(
            f"{location}.environment_id must be path-safe"
        )
    if environment_id != environment["id"]:
        raise ReportValidationError(
            f"{location}.environment_id must match runtime environment"
        )

    operation = _required_string(
        topology["operation"],
        f"{location}.operation",
    )
    if operation not in _GROUND_TOPOLOGY_OPERATIONS:
        raise ReportValidationError(
            f"{location}.operation must be one of "
            f"{sorted(_GROUND_TOPOLOGY_OPERATIONS)}"
        )

    source_asset_path = _absolute_file_path(
        topology["source_asset_path"],
        f"{location}.source_asset_path",
    )
    source_asset_sha256 = _validate_lowercase_sha256(
        topology["source_asset_sha256"],
        f"{location}.source_asset_sha256",
    )
    environment_source = environment["source_asset"]
    if source_asset_path != environment_source["path"]:
        raise ReportValidationError(
            f"{location}.source_asset_path must match runtime environment"
        )
    if not hmac.compare_digest(
        source_asset_sha256,
        environment_source["sha256"],
    ):
        raise ReportValidationError(
            f"{location}.source_asset_sha256 must match runtime environment"
        )

    overlay_identifier = _required_string(
        topology["overlay_identifier"],
        f"{location}.overlay_identifier",
    )
    if not overlay_identifier.startswith("anon:"):
        raise ReportValidationError(
            f"{location}.overlay_identifier must identify an anonymous layer"
        )
    _validate_lowercase_sha256(
        topology["overlay_sha256"],
        f"{location}.overlay_sha256",
    )
    if topology["stage_usd_readback_verified"] is not True:
        raise ReportValidationError(
            f"{location}.stage_usd_readback_verified must be true"
        )

    path_sets: dict[str, list[str]] = {}
    counts: dict[str, int] = {}
    for name, allow_empty in (
        ("source", False),
        ("target", False),
        ("disabled", True),
    ):
        paths = _ground_topology_path_sequence(
            topology[f"{name}_colliders"],
            f"{location}.{name}_colliders",
            allow_empty=allow_empty,
        )
        count = _topology_count(
            topology[f"{name}_collider_count"],
            f"{location}.{name}_collider_count",
            allow_zero=allow_empty,
        )
        if count != len(paths):
            raise ReportValidationError(
                f"{location}.{name}_collider_count must match "
                f"{name}_colliders"
            )
        recorded_sha256 = _validate_lowercase_sha256(
            topology[f"{name}_collider_paths_sha256"],
            f"{location}.{name}_collider_paths_sha256",
        )
        actual_sha256 = _canonical_path_sha256(paths)
        if not hmac.compare_digest(recorded_sha256, actual_sha256):
            raise ReportValidationError(
                f"{location}.{name}_collider_paths_sha256 does not match "
                "the canonical sorted path list"
            )
        path_sets[name] = paths
        counts[name] = count

    source = set(path_sets["source"])
    target = set(path_sets["target"])
    disabled = set(path_sets["disabled"])
    if target & disabled:
        raise ReportValidationError(
            f"{location} target and disabled collider sets must be disjoint"
        )
    if target | disabled != source:
        raise ReportValidationError(
            f"{location} target and disabled collider sets must partition source"
        )

    if operation == "preserve_source_colliders":
        if target != source or disabled:
            raise ReportValidationError(
                f"{location} preserve_source_colliders requires target equal "
                "source and an empty disabled set"
            )
    elif not target < source or disabled != source - target:
        raise ReportValidationError(
            f"{location} disable_non_target_colliders requires target to be a "
            "true source subset and disabled to equal source minus target"
        )

    ground_colliders = contact["ground_colliders"]
    if path_sets["target"] != ground_colliders:
        raise ReportValidationError(
            f"{location}.target_colliders must equal "
            "runtime_provenance.contact.ground_colliders"
        )
    contract = contact["collider_contract"]
    if counts["target"] != contract["ground_expected_enabled_count"]:
        raise ReportValidationError(
            f"{location}.target_collider_count must equal the contact "
            "ground_expected_enabled_count"
        )
    required_ground = set(contract["ground_required_prim_paths"])
    if not required_ground.issubset(target):
        raise ReportValidationError(
            f"{location}.target_colliders must contain all contact required "
            "ground prim paths"
        )
    semantic_classes = contract["ground_semantic_classes"]
    if not semantic_classes and required_ground != target:
        raise ReportValidationError(
            f"{location} contact ground selectors without semantic classes "
            "must enumerate the complete target set"
        )


def validate_runtime_provenance(provenance: Mapping[str, Any]) -> None:
    """Validate the Isaac-startup snapshot embedded in a diagnostic report."""

    if not isinstance(provenance, Mapping):
        raise ReportValidationError("runtime_provenance must be a mapping")
    _validate_json_value(provenance, "runtime_provenance")
    if provenance.get("verified") is not True:
        raise ReportValidationError("runtime_provenance must be runtime-verified")
    schema_version = provenance.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version not in {3, 4, 5}
    ):
        raise ReportValidationError(
            "runtime_provenance.schema_version must be integer 3, 4, or 5"
        )
    expected_root_keys = {
        "verified",
        "schema_version",
        "robot",
        "environment",
        "simulation",
        "contact",
        "git",
    }
    if schema_version == 5:
        expected_root_keys.add("ground_topology")
    if set(provenance) != expected_root_keys:
        raise ReportValidationError(
            "runtime_provenance keys must be exactly "
            f"{sorted(expected_root_keys)}"
        )

    robot = _required_mapping(provenance, "robot", "runtime_provenance")
    robot_keys = {"config", "asset", "solver"}
    if schema_version in {4, 5}:
        robot_keys.add("kinematics")
    if set(robot) != robot_keys:
        raise ReportValidationError(
            "runtime_provenance.robot keys must be exactly "
            f"{sorted(robot_keys)}"
        )
    for name in ("config", "asset"):
        input_file = _required_mapping(
            robot, name, "runtime_provenance.robot"
        )
        if set(input_file) != {"path", "sha256"}:
            raise ReportValidationError(
                f"runtime_provenance.robot.{name} keys must be exactly "
                "['path', 'sha256']"
            )
        _required_string(
            input_file.get("path"),
            f"runtime_provenance.robot.{name}.path",
        )
        _validate_sha256(
            input_file.get("sha256"),
            f"runtime_provenance.robot.{name}.sha256",
        )
    solver = _required_mapping(
        robot, "solver", "runtime_provenance.robot"
    )
    solver_keys = {
        "position_iterations",
        "velocity_iterations",
        "stage_articulation_usd_readback_verified",
    }
    if set(solver) != solver_keys:
        raise ReportValidationError(
            "runtime_provenance.robot.solver keys must be exactly "
            f"{sorted(solver_keys)}"
        )
    for name in ("position_iterations", "velocity_iterations"):
        count = solver.get(name)
        if (
            isinstance(count, bool)
            or not isinstance(count, int)
            or not 1 <= count <= 255
        ):
            raise ReportValidationError(
                f"runtime_provenance.robot.solver.{name} must be an integer "
                "in [1, 255]"
            )
    if solver.get("stage_articulation_usd_readback_verified") is not True:
        raise ReportValidationError(
            "runtime_provenance.robot.solver."
            "stage_articulation_usd_readback_verified must be true"
        )

    if schema_version in {4, 5}:
        kinematics = _required_mapping(
            robot, "kinematics", "runtime_provenance.robot"
        )
        kinematics_keys = {
            "profile_id",
            "lifecycle",
            "wheel_radius_m",
            "wheel_width_m",
            "geometric_track_width_m",
            "effective_track_width_m",
            "controller_contract_verified",
        }
        if set(kinematics) != kinematics_keys:
            raise ReportValidationError(
                "runtime_provenance.robot.kinematics keys must be exactly "
                f"{sorted(kinematics_keys)}"
            )
        for name in ("profile_id", "lifecycle"):
            value = _required_string(
                kinematics.get(name),
                f"runtime_provenance.robot.kinematics.{name}",
            )
            if not _IDENTIFIER_PATTERN.fullmatch(value):
                raise ReportValidationError(
                    f"runtime_provenance.robot.kinematics.{name} must be a "
                    "path-safe identifier"
                )
        if kinematics.get("lifecycle") not in {
            "stable_baseline",
            "experimental_candidate",
        }:
            raise ReportValidationError(
                "runtime_provenance.robot.kinematics.lifecycle is unsupported"
            )
        for name in (
            "wheel_radius_m",
            "wheel_width_m",
            "geometric_track_width_m",
            "effective_track_width_m",
        ):
            value = kinematics.get(name)
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or float(value) <= 0.0
            ):
                raise ReportValidationError(
                    f"runtime_provenance.robot.kinematics.{name} must be a "
                    "finite positive number"
                )
        if kinematics.get("controller_contract_verified") is not True:
            raise ReportValidationError(
                "runtime_provenance.robot.kinematics."
                "controller_contract_verified must be true"
            )

    environment = _required_mapping(
        provenance, "environment", "runtime_provenance"
    )
    environment_keys = {
        "id",
        "project_stage",
        "source_asset",
        "asset_root",
        "asset_version",
        "composed_root_layer_sha256",
    }
    if set(environment) != environment_keys:
        raise ReportValidationError(
            "runtime_provenance.environment keys must be exactly "
            f"{sorted(environment_keys)}"
        )
    environment_id = _required_string(
        environment.get("id"),
        "runtime_provenance.environment.id",
    )
    if not _IDENTIFIER_PATTERN.fullmatch(environment_id):
        raise ReportValidationError(
            "runtime_provenance.environment.id must be a path-safe identifier"
        )
    for name in ("project_stage", "source_asset"):
        input_file = _required_mapping(
            environment, name, "runtime_provenance.environment"
        )
        if set(input_file) != {"path", "sha256"}:
            raise ReportValidationError(
                f"runtime_provenance.environment.{name} keys must be "
                "exactly ['path', 'sha256']"
            )
        _required_string(
            input_file.get("path"),
            f"runtime_provenance.environment.{name}.path",
        )
        _validate_sha256(
            input_file.get("sha256"),
            f"runtime_provenance.environment.{name}.sha256",
        )
    _required_string(
        environment.get("asset_root"),
        "runtime_provenance.environment.asset_root",
    )
    _required_string(
        environment.get("asset_version"),
        "runtime_provenance.environment.asset_version",
    )
    _validate_sha256(
        environment.get("composed_root_layer_sha256"),
        "runtime_provenance.environment.composed_root_layer_sha256",
    )

    simulation = _required_mapping(
        provenance, "simulation", "runtime_provenance"
    )
    simulation_keys = {"navigation_mode", "odometry_mode", "physics_hz"}
    if set(simulation) != simulation_keys:
        raise ReportValidationError(
            "runtime_provenance.simulation keys must be exactly "
            f"{sorted(simulation_keys)}"
        )
    for name in ("navigation_mode", "odometry_mode"):
        _required_string(
            simulation.get(name),
            f"runtime_provenance.simulation.{name}",
        )
    physics_hz = simulation.get("physics_hz")
    if (
        isinstance(physics_hz, bool)
        or not isinstance(physics_hz, (int, float))
        or not math.isfinite(float(physics_hz))
        or physics_hz <= 0
    ):
        raise ReportValidationError(
            "runtime_provenance.simulation.physics_hz must be positive"
        )

    contact = _required_mapping(
        provenance,
        "contact",
        "runtime_provenance",
    )
    _validate_contact_provenance(contact)
    if schema_version == 5:
        ground_topology = _required_mapping(
            provenance,
            "ground_topology",
            "runtime_provenance",
        )
        _validate_ground_topology_provenance(
            ground_topology,
            environment,
            contact,
        )

    git = _required_mapping(provenance, "git", "runtime_provenance")
    if set(git) != {"commit", "branch", "dirty"}:
        raise ReportValidationError(
            "runtime_provenance.git keys must be exactly "
            "['branch', 'commit', 'dirty']"
        )
    commit = git.get("commit")
    if not isinstance(commit, str) or len(commit) not in {40, 64}:
        raise ReportValidationError(
            "runtime_provenance.git.commit must be a Git object id"
        )
    try:
        int(commit, 16)
    except ValueError as exc:
        raise ReportValidationError(
            "runtime_provenance.git.commit must be a Git object id"
        ) from exc
    _required_string(git.get("branch"), "runtime_provenance.git.branch")
    if not isinstance(git.get("dirty"), bool):
        raise ReportValidationError(
            "runtime_provenance.git.dirty must be boolean"
        )


def _atomic_text_write(path: Path, writer: Callable[[TextIO], None]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
        suffix=".tmp",
        text=True,
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            writer(stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


def _csv_value(value: Any) -> str | int | float | bool:
    if value is None:
        return ""
    if isinstance(value, (str, int, float, bool)):
        return value
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _validate_stem(stem: str) -> None:
    if not stem or stem in {".", ".."} or Path(stem).name != stem:
        raise ValueError("report stem must be one path-safe file name")


def write_run_report(
    manifest: Mapping[str, Any],
    output_directory: str | os.PathLike[str],
    stem: str,
) -> tuple[Path, Path]:
    """Atomically replace one strict JSON report and its single-row CSV peer."""
    validate_manifest(manifest)
    _validate_stem(stem)
    output = Path(output_directory)
    json_path = output / f"{stem}.json"
    csv_path = output / f"{stem}.csv"

    def write_json(stream: TextIO) -> None:
        json.dump(manifest, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    fieldnames = list(REPRODUCIBILITY_FIELDS)
    fieldnames.extend(sorted(set(manifest) - set(fieldnames)))

    def write_csv(stream: TextIO) -> None:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="raise")
        writer.writeheader()
        writer.writerow({key: _csv_value(manifest[key]) for key in fieldnames})

    _atomic_text_write(json_path, write_json)
    _atomic_text_write(csv_path, write_csv)
    return json_path, csv_path


def write_strict_json_report(
    document: Mapping[str, Any],
    output_path: str | os.PathLike[str],
) -> Path:
    """Atomically write any mapping after enforcing strict JSON values."""
    if not isinstance(document, Mapping):
        raise ReportValidationError("JSON report root must be a mapping")
    _validate_json_value(document, "report")
    destination = Path(output_path).expanduser()
    if not destination.name or destination.name in {".", ".."}:
        raise ValueError("output_path must name a JSON report file")

    def write_json(stream: TextIO) -> None:
        json.dump(document, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")

    _atomic_text_write(destination, write_json)
    return destination
