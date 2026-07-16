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
_RESET_STRATEGY_KEYS = {
    "schema_version",
    "id",
    "lift_distance_m",
    "separation_step_count",
    "recontact_step_count",
    "contact_probe",
}
_RESET_CONTACT_PROBE_KEYS = {
    "schema_version",
    "enabled",
    "wheel_bindings",
    "wheel_count",
    "ground_filter_paths",
    "ground_filter_count",
    "max_contact_count",
    "report_threshold_n",
    "stage_usd_readback_verified",
}
_RESET_STRATEGY_SEMANTICS = {
    "pose_restore_v1": (0.0, 0, 1),
    "separate_recontact_0p20m_1step_v1": (0.2, 1, 1),
}
_WHEEL_VELOCITY_DRIVE_KEYS = {
    "schema_version",
    "profile_path",
    "profile_sha256",
    "profile_id",
    "configured_si",
    "authored_usd",
    "joint_paths",
    "overlay_identifier",
    "overlay_sha256",
    "stage_usd_readback_verified",
    "physics_tensor",
}
_MASS_COLLISION_KEYS = {
    "schema_version",
    "profile_path",
    "profile_sha256",
    "profile_id",
    "profile_mode",
    "robot_asset_sha256",
    "sensor_shells",
    "base_inertial",
    "expected_link_masses",
    "expected_total_mass_kg",
    "overlay_id",
    "overlay_identifier",
    "overlay_sha256",
    "stage_usd_readback_verified",
    "physics_tensor",
}
_MASS_COLLISION_MODES = {
    "legacy_default_sensor_density",
    "sensor_shells_disabled",
    "fixed_base_inertial_sensor_shell_collision",
}
_CONTROL_GRAPH_KEYS = {
    "schema_version",
    "wheel_command_application",
    "topology",
    "topology_sha256",
    "materialized_readback_verified",
}
_WHEEL_COMMAND_APPLICATIONS = {
    "split_axle_v1": ("FrontController", "RearController"),
    "single_four_wheel_write_v1": ("WheelController",),
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


def _decode_hashed_snapshot(
    payload: object,
    expected_sha256: object,
    *,
    location: str,
) -> dict[str, Any]:
    if not isinstance(payload, str):
        raise ReportValidationError(f"{location}.json must be a string")
    _validate_sha256(
        expected_sha256,
        f"{location}.sha256",
    )
    actual_sha256 = hashlib.sha256(payload.encode("utf-8")).hexdigest()
    if not hmac.compare_digest(actual_sha256, expected_sha256):
        raise ReportValidationError(f"{location} JSON SHA256 mismatch")

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
            f"{location}.json must be valid JSON: {exc}"
        ) from exc
    if not isinstance(decoded, dict):
        raise ReportValidationError(f"{location}.json root must be a mapping")
    try:
        canonical = json.dumps(
            decoded,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise ReportValidationError(
            f"{location}.json must be strict JSON: {exc}"
        ) from exc
    if canonical != payload:
        raise ReportValidationError(
            f"{location}.json must be canonical strict JSON"
        )
    return decoded


def decode_hashed_contact_snapshot(
    payload: object,
    expected_sha256: object,
) -> dict[str, Any]:
    """Verify and decode the canonical contact JSON exposed by Isaac."""

    return _decode_hashed_snapshot(
        payload,
        expected_sha256,
        location="runtime_provenance.contact",
    )


def decode_hashed_reset_strategy_snapshot(
    payload: object,
    expected_sha256: object,
) -> dict[str, Any]:
    """Verify and decode Isaac's canonical reset-strategy JSON pair."""

    return _decode_hashed_snapshot(
        payload,
        expected_sha256,
        location="runtime_provenance.simulation.reset_strategy",
    )


def decode_hashed_runtime_snapshot(
    payload: object,
    expected_sha256: object,
    *,
    component: str,
) -> dict[str, Any]:
    """Decode one schema-v7 JSON/SHA parameter pair without ambiguity."""

    supported = {
        "robot.wheel_velocity_drive",
        "robot.mass_collision",
        "control_graph",
    }
    if component not in supported:
        raise ReportValidationError(
            f"unsupported component for hashed runtime snapshot: {component!r}"
        )
    return _decode_hashed_snapshot(
        payload,
        expected_sha256,
        location=f"runtime_provenance.{component}",
    )


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


def _validate_reset_strategy_provenance(
    reset_strategy: Mapping[str, Any],
    contact: Mapping[str, Any],
    ground_topology: Mapping[str, Any],
) -> None:
    location = "runtime_provenance.simulation.reset_strategy"
    if set(reset_strategy) != _RESET_STRATEGY_KEYS:
        raise ReportValidationError(
            f"{location} keys must be exactly "
            f"{sorted(_RESET_STRATEGY_KEYS)}"
        )
    schema_version = reset_strategy.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 1
    ):
        raise ReportValidationError(
            f"{location}.schema_version must be integer 1"
        )
    identifier = _required_string(reset_strategy.get("id"), f"{location}.id")
    if not _IDENTIFIER_PATTERN.fullmatch(identifier):
        raise ReportValidationError(f"{location}.id must be path-safe")
    expected_semantics = _RESET_STRATEGY_SEMANTICS.get(identifier)
    if expected_semantics is None:
        raise ReportValidationError(f"{location}.id is unsupported")

    lift_distance_m = reset_strategy.get("lift_distance_m")
    separation_step_count = reset_strategy.get("separation_step_count")
    recontact_step_count = reset_strategy.get("recontact_step_count")
    if (
        not isinstance(lift_distance_m, float)
        or not math.isfinite(lift_distance_m)
        or isinstance(separation_step_count, bool)
        or not isinstance(separation_step_count, int)
        or isinstance(recontact_step_count, bool)
        or not isinstance(recontact_step_count, int)
    ):
        raise ReportValidationError(
            f"{location} strategy semantics must use one finite float and "
            "integer step counts"
        )
    if (
        lift_distance_m,
        separation_step_count,
        recontact_step_count,
    ) != expected_semantics:
        raise ReportValidationError(
            f"{location} strategy semantics do not match id {identifier!r}"
        )

    probe = _required_mapping(reset_strategy, "contact_probe", location)
    probe_location = f"{location}.contact_probe"
    if set(probe) != _RESET_CONTACT_PROBE_KEYS:
        raise ReportValidationError(
            f"{probe_location} keys must be exactly "
            f"{sorted(_RESET_CONTACT_PROBE_KEYS)}"
        )
    probe_schema_version = probe.get("schema_version")
    if (
        isinstance(probe_schema_version, bool)
        or not isinstance(probe_schema_version, int)
        or probe_schema_version != 1
    ):
        raise ReportValidationError(
            f"{probe_location}.schema_version must be integer 1"
        )
    if probe.get("enabled") is not True:
        raise ReportValidationError(f"{probe_location}.enabled must be true")
    if probe.get("stage_usd_readback_verified") is not True:
        raise ReportValidationError(
            f"{probe_location}.stage_usd_readback_verified must be true"
        )

    wheel_bindings = probe.get("wheel_bindings")
    if not isinstance(wheel_bindings, list) or len(wheel_bindings) != 4:
        raise ReportValidationError(
            f"{probe_location}.wheel_bindings must contain exactly 4 entries"
        )
    joint_names: list[str] = []
    wheel_link_paths: list[str] = []
    for index, binding in enumerate(wheel_bindings):
        binding_location = f"{probe_location}.wheel_bindings[{index}]"
        if not isinstance(binding, Mapping) or set(binding) != {
            "joint_name",
            "wheel_link_path",
        }:
            raise ReportValidationError(
                f"{binding_location} keys must be exactly "
                "['joint_name', 'wheel_link_path']"
            )
        joint_name = _required_string(
            binding.get("joint_name"),
            f"{binding_location}.joint_name",
        )
        if not _IDENTIFIER_PATTERN.fullmatch(joint_name):
            raise ReportValidationError(
                f"{binding_location}.joint_name must be path-safe"
            )
        joint_names.append(joint_name)
        wheel_link_paths.append(
            _absolute_prim_path(
                binding.get("wheel_link_path"),
                f"{binding_location}.wheel_link_path",
            )
        )
    if len(set(wheel_link_paths)) != len(wheel_link_paths):
        raise ReportValidationError(
            f"{probe_location}.wheel_bindings wheel link paths must be unique"
        )
    expected_joint_names = contact["collider_contract"]["wheel_joint_names"]
    if joint_names != expected_joint_names:
        raise ReportValidationError(
            f"{probe_location}.wheel_bindings must preserve wheel joint order"
        )
    wheel_colliders = contact["wheel_colliders"]
    for index, (wheel_link_path, collider_path) in enumerate(
        zip(wheel_link_paths, wheel_colliders, strict=True)
    ):
        if not (
            collider_path == wheel_link_path
            or collider_path.startswith(f"{wheel_link_path.rstrip('/')}/")
        ):
            raise ReportValidationError(
                f"{probe_location}.wheel_bindings[{index}].wheel_link_path "
                "must be the bound contact wheel collider or its ancestor"
            )
    wheel_count = probe.get("wheel_count")
    if (
        isinstance(wheel_count, bool)
        or not isinstance(wheel_count, int)
        or wheel_count != 4
        or wheel_count != len(wheel_bindings)
    ):
        raise ReportValidationError(
            f"{probe_location}.wheel_count must be 4"
        )

    ground_filter_paths = _ground_topology_path_sequence(
        probe.get("ground_filter_paths"),
        f"{probe_location}.ground_filter_paths",
        allow_empty=False,
    )
    if ground_filter_paths != ground_topology["target_colliders"]:
        raise ReportValidationError(
            f"{probe_location}.ground_filter_paths must equal the ground "
            "topology target"
        )
    ground_filter_count = probe.get("ground_filter_count")
    if (
        isinstance(ground_filter_count, bool)
        or not isinstance(ground_filter_count, int)
        or ground_filter_count != len(ground_filter_paths)
    ):
        raise ReportValidationError(
            f"{probe_location}.ground_filter_count must equal the path count"
        )
    max_contact_count = probe.get("max_contact_count")
    if (
        isinstance(max_contact_count, bool)
        or not isinstance(max_contact_count, int)
        or max_contact_count != 128
    ):
        raise ReportValidationError(
            f"{probe_location}.max_contact_count must be 128"
        )
    report_threshold_n = probe.get("report_threshold_n")
    if not isinstance(report_threshold_n, float) or report_threshold_n != 0.0:
        raise ReportValidationError(
            f"{probe_location}.report_threshold_n must be 0.0"
        )


def _exact_keys(
    value: Mapping[str, Any],
    expected: set[str],
    location: str,
) -> None:
    if set(value) != expected:
        raise ReportValidationError(
            f"{location} keys must be exactly {sorted(expected)}"
        )


def _schema_version_one(value: Any, location: str) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value != 1:
        raise ReportValidationError(f"{location} must be integer 1")


def _finite_number(
    value: Any,
    location: str,
    *,
    positive: bool = False,
    nonnegative: bool = False,
) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
    ):
        raise ReportValidationError(f"{location} must be a finite number")
    number = float(value)
    if positive and number <= 0.0:
        raise ReportValidationError(
            f"{location} must be a finite positive number"
        )
    if nonnegative and number < 0.0:
        raise ReportValidationError(
            f"{location} must be a finite non-negative number"
        )
    return number


def _finite_vector(
    value: Any,
    location: str,
    *,
    length: int,
    positive: bool = False,
    nonnegative: bool = False,
) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ReportValidationError(
            f"{location} must contain exactly {length} finite numbers"
        )
    return [
        _finite_number(
            item,
            f"{location}[{index}]",
            positive=positive,
            nonnegative=nonnegative,
        )
        for index, item in enumerate(value)
    ]


def _inertia_matrix(value: Any, location: str) -> list[list[float]]:
    if not isinstance(value, list) or len(value) != 3:
        raise ReportValidationError(
            f"{location} must be a finite 3x3 matrix"
        )
    matrix = [
        _finite_vector(row, f"{location}[{index}]", length=3)
        for index, row in enumerate(value)
    ]
    for row in range(3):
        if matrix[row][row] <= 0.0:
            raise ReportValidationError(
                f"{location} diagonal must be positive"
            )
        for column in range(row + 1, 3):
            if not math.isclose(
                matrix[row][column],
                matrix[column][row],
                rel_tol=1.0e-6,
                abs_tol=1.0e-9,
            ):
                raise ReportValidationError(
                    f"{location} must be symmetric"
                )
    return matrix


def _numbers_close(actual: float, expected: float) -> bool:
    return math.isclose(
        actual,
        expected,
        rel_tol=1.0e-6,
        abs_tol=1.0e-8,
    )


def _validate_wheel_velocity_drive_provenance(
    drive: Mapping[str, Any],
    robot_config: Mapping[str, Any],
) -> None:
    location = "runtime_provenance.robot.wheel_velocity_drive"
    _exact_keys(drive, _WHEEL_VELOCITY_DRIVE_KEYS, location)
    _schema_version_one(drive["schema_version"], f"{location}.schema_version")
    profile_path = _absolute_file_path(
        drive["profile_path"], f"{location}.profile_path"
    )
    if profile_path != robot_config["path"]:
        raise ReportValidationError(
            f"{location}.profile_path must match robot.config.path"
        )
    profile_sha256 = _validate_lowercase_sha256(
        drive["profile_sha256"], f"{location}.profile_sha256"
    )
    if not hmac.compare_digest(profile_sha256, robot_config["sha256"]):
        raise ReportValidationError(
            f"{location}.profile_sha256 must match robot.config.sha256"
        )
    profile_id = _required_string(
        drive["profile_id"], f"{location}.profile_id"
    )
    if not _IDENTIFIER_PATTERN.fullmatch(profile_id):
        raise ReportValidationError(f"{location}.profile_id must be path-safe")

    configured = _required_mapping(drive, "configured_si", location)
    configured_keys = {
        "drive_type",
        "stiffness_n_m_per_rad",
        "damping_n_m_s_per_rad",
        "max_effort_n_m",
        "max_joint_velocity_rad_s",
    }
    _exact_keys(configured, configured_keys, f"{location}.configured_si")
    if configured["drive_type"] != "force":
        raise ReportValidationError(
            f"{location}.configured_si.drive_type must equal force"
        )
    stiffness = _finite_number(
        configured["stiffness_n_m_per_rad"],
        f"{location}.configured_si.stiffness_n_m_per_rad",
        nonnegative=True,
    )
    if stiffness != 0.0:
        raise ReportValidationError(
            f"{location}.configured_si.stiffness_n_m_per_rad must equal 0"
        )
    damping = _finite_number(
        configured["damping_n_m_s_per_rad"],
        f"{location}.configured_si.damping_n_m_s_per_rad",
        positive=True,
    )
    max_effort = _finite_number(
        configured["max_effort_n_m"],
        f"{location}.configured_si.max_effort_n_m",
        positive=True,
    )
    max_velocity = _finite_number(
        configured["max_joint_velocity_rad_s"],
        f"{location}.configured_si.max_joint_velocity_rad_s",
        positive=True,
    )

    authored = _required_mapping(drive, "authored_usd", location)
    authored_keys = {
        "drive_type",
        "stiffness_n_m_per_degree",
        "damping_n_m_s_per_degree",
        "max_force_n_m",
        "max_joint_velocity_deg_s",
    }
    _exact_keys(authored, authored_keys, f"{location}.authored_usd")
    if authored["drive_type"] != "force":
        raise ReportValidationError(
            f"{location}.authored_usd.drive_type must equal force"
        )
    authored_values = {
        "stiffness_n_m_per_degree": _finite_number(
            authored["stiffness_n_m_per_degree"],
            f"{location}.authored_usd.stiffness_n_m_per_degree",
            nonnegative=True,
        ),
        "damping_n_m_s_per_degree": _finite_number(
            authored["damping_n_m_s_per_degree"],
            f"{location}.authored_usd.damping_n_m_s_per_degree",
            positive=True,
        ),
        "max_force_n_m": _finite_number(
            authored["max_force_n_m"],
            f"{location}.authored_usd.max_force_n_m",
            positive=True,
        ),
        "max_joint_velocity_deg_s": _finite_number(
            authored["max_joint_velocity_deg_s"],
            f"{location}.authored_usd.max_joint_velocity_deg_s",
            positive=True,
        ),
    }
    conversions = {
        "stiffness_n_m_per_degree": (stiffness * math.pi / 180.0),
        "damping_n_m_s_per_degree": (damping * math.pi / 180.0),
        "max_force_n_m": max_effort,
        "max_joint_velocity_deg_s": (max_velocity * 180.0 / math.pi),
    }
    conversion_labels = {
        "stiffness_n_m_per_degree": "stiffness conversion",
        "damping_n_m_s_per_degree": "damping conversion",
        "max_force_n_m": "max effort conversion",
        "max_joint_velocity_deg_s": "max velocity conversion",
    }
    for name, expected in conversions.items():
        if not _numbers_close(authored_values[name], expected):
            raise ReportValidationError(
                f"{location}.authored_usd {conversion_labels[name]} mismatch"
            )

    joint_paths = drive["joint_paths"]
    if not isinstance(joint_paths, list) or len(joint_paths) != 4:
        raise ReportValidationError(
            f"{location}.joint_paths must contain exactly 4 paths"
        )
    normalized_joint_paths = [
        _absolute_prim_path(path, f"{location}.joint_paths[{index}]")
        for index, path in enumerate(joint_paths)
    ]
    if len(set(normalized_joint_paths)) != 4:
        raise ReportValidationError(
            f"{location}.joint_paths must contain four unique paths"
        )
    overlay_identifier = _required_string(
        drive["overlay_identifier"], f"{location}.overlay_identifier"
    )
    if not overlay_identifier.startswith("anon:"):
        raise ReportValidationError(
            f"{location}.overlay_identifier must identify an anonymous layer"
        )
    overlay_sha256 = _validate_lowercase_sha256(
        drive["overlay_sha256"], f"{location}.overlay_sha256"
    )
    if drive["stage_usd_readback_verified"] is not True:
        raise ReportValidationError(
            f"{location}.stage_usd_readback_verified must be true"
        )

    tensor = _required_mapping(drive, "physics_tensor", location)
    tensor_location = f"{location}.physics_tensor"
    tensor_keys = {
        "schema_version",
        "profile_path",
        "profile_sha256",
        "profile_id",
        "stage_overlay_sha256",
        "dof_names",
        "dof_indices",
        "drive_types",
        "stiffnesses_n_m_per_rad",
        "dampings_n_m_s_per_rad",
        "max_efforts_n_m",
        "max_joint_velocities_rad_s",
        "physics_tensor_readback_verified",
    }
    _exact_keys(tensor, tensor_keys, tensor_location)
    _schema_version_one(
        tensor["schema_version"], f"{tensor_location}.schema_version"
    )
    for name, expected in (
        ("profile_path", profile_path),
        ("profile_sha256", profile_sha256),
        ("profile_id", profile_id),
        ("stage_overlay_sha256", overlay_sha256),
    ):
        actual = tensor[name]
        if name.endswith("sha256"):
            _validate_lowercase_sha256(actual, f"{tensor_location}.{name}")
        else:
            _required_string(actual, f"{tensor_location}.{name}")
        if actual != expected:
            label = (
                "Stage evidence"
                if name in {"profile_path", "profile_sha256", "profile_id"}
                else "Stage overlay"
            )
            raise ReportValidationError(
                f"{tensor_location}.{name} must match {label}"
            )
    dof_names = tensor["dof_names"]
    if not isinstance(dof_names, list) or len(dof_names) != 4:
        raise ReportValidationError(
            f"{tensor_location}.dof_names must contain exactly 4 names"
        )
    normalized_dof_names = [
        _required_string(name, f"{tensor_location}.dof_names[{index}]")
        for index, name in enumerate(dof_names)
    ]
    if len(set(normalized_dof_names)) != 4:
        raise ReportValidationError(
            f"{tensor_location}.dof_names must contain unique names"
        )
    if normalized_dof_names != [Path(path).name for path in joint_paths]:
        raise ReportValidationError(
            f"{tensor_location}.dof_names must match Stage joint_paths"
        )
    indices = tensor["dof_indices"]
    if (
        not isinstance(indices, list)
        or len(indices) != 4
        or any(
            isinstance(index, bool)
            or not isinstance(index, int)
            or index < 0
            for index in indices
        )
        or len(set(indices)) != 4
    ):
        raise ReportValidationError(
            f"{tensor_location}.dof_indices must contain exactly 4 unique "
            "non-negative integers"
        )
    drive_types = tensor["drive_types"]
    if drive_types != ["force"] * 4:
        raise ReportValidationError(
            f"{tensor_location}.drive_types must contain four force drives"
        )
    tensor_vectors = {
        "stiffnesses_n_m_per_rad": _finite_vector(
            tensor["stiffnesses_n_m_per_rad"],
            f"{tensor_location}.stiffnesses_n_m_per_rad",
            length=4,
            nonnegative=True,
        ),
        "dampings_n_m_s_per_rad": _finite_vector(
            tensor["dampings_n_m_s_per_rad"],
            f"{tensor_location}.dampings_n_m_s_per_rad",
            length=4,
            positive=True,
        ),
        "max_efforts_n_m": _finite_vector(
            tensor["max_efforts_n_m"],
            f"{tensor_location}.max_efforts_n_m",
            length=4,
            positive=True,
        ),
        "max_joint_velocities_rad_s": _finite_vector(
            tensor["max_joint_velocities_rad_s"],
            f"{tensor_location}.max_joint_velocities_rad_s",
            length=4,
            positive=True,
        ),
    }
    expected_vectors = {
        "stiffnesses_n_m_per_rad": stiffness,
        "dampings_n_m_s_per_rad": damping,
        "max_efforts_n_m": max_effort,
        "max_joint_velocities_rad_s": max_velocity,
    }
    for name, values in tensor_vectors.items():
        if any(
            not _numbers_close(actual, expected_vectors[name])
            for actual in values
        ):
            raise ReportValidationError(
                f"{tensor_location}.{name} must match configured_si"
            )
    if tensor["physics_tensor_readback_verified"] is not True:
        raise ReportValidationError(
            f"{tensor_location}.physics_tensor_readback_verified must be true"
        )


def _validate_mass_collision_provenance(
    mass: Mapping[str, Any],
    robot_asset: Mapping[str, Any],
) -> None:
    location = "runtime_provenance.robot.mass_collision"
    _exact_keys(mass, _MASS_COLLISION_KEYS, location)
    _schema_version_one(mass["schema_version"], f"{location}.schema_version")
    _required_string(mass["profile_path"], f"{location}.profile_path")
    _validate_lowercase_sha256(
        mass["profile_sha256"], f"{location}.profile_sha256"
    )
    profile_id = _required_string(
        mass["profile_id"], f"{location}.profile_id"
    )
    if not _IDENTIFIER_PATTERN.fullmatch(profile_id):
        raise ReportValidationError(f"{location}.profile_id must be path-safe")
    profile_mode = mass["profile_mode"]
    if profile_mode not in _MASS_COLLISION_MODES:
        raise ReportValidationError(
            f"{location}.profile_mode must be one of "
            f"{sorted(_MASS_COLLISION_MODES)}"
        )
    robot_asset_sha256 = _validate_lowercase_sha256(
        mass["robot_asset_sha256"], f"{location}.robot_asset_sha256"
    )
    if not hmac.compare_digest(robot_asset_sha256, robot_asset["sha256"]):
        raise ReportValidationError(
            f"{location}.robot_asset_sha256 must match robot.asset.sha256"
        )

    sensor_shells = mass["sensor_shells"]
    if not isinstance(sensor_shells, list) or len(sensor_shells) != 2:
        raise ReportValidationError(
            f"{location}.sensor_shells must contain exactly 2 entries"
        )
    shell_paths: list[str] = []
    expected_shell_flag = profile_mode != "sensor_shells_disabled"
    for index, shell in enumerate(sensor_shells):
        shell_location = f"{location}.sensor_shells[{index}]"
        if not isinstance(shell, Mapping):
            raise ReportValidationError(f"{shell_location} must be a mapping")
        _exact_keys(
            shell,
            {"prim_path", "active", "collision_enabled"},
            shell_location,
        )
        shell_paths.append(
            _absolute_prim_path(shell["prim_path"], f"{shell_location}.prim_path")
        )
        if not isinstance(shell["active"], bool) or not isinstance(
            shell["collision_enabled"], bool
        ):
            raise ReportValidationError(
                f"{shell_location} shell flags must be boolean"
            )
        if (
            shell["active"] is not expected_shell_flag
            or shell["collision_enabled"] is not expected_shell_flag
        ):
            raise ReportValidationError(
                f"{shell_location} shell flags disagree with profile_mode"
            )
    if shell_paths != sorted(shell_paths) or len(set(shell_paths)) != 2:
        raise ReportValidationError(
            f"{location}.sensor_shells must have sorted unique prim paths"
        )

    base_inertial = mass["base_inertial"]
    if profile_mode == "fixed_base_inertial_sensor_shell_collision":
        if not isinstance(base_inertial, Mapping):
            raise ReportValidationError(
                f"{location}.base_inertial must be a mapping for fixed mode"
            )
    elif base_inertial is not None:
        raise ReportValidationError(
            f"{location}.base_inertial must be null for {profile_mode}"
        )
    normalized_base: dict[str, Any] | None = None
    if isinstance(base_inertial, Mapping):
        base_location = f"{location}.base_inertial"
        _exact_keys(
            base_inertial,
            {"prim_path", "mass_kg", "center_of_mass_m", "inertia_kg_m2"},
            base_location,
        )
        normalized_base = {
            "prim_path": _absolute_prim_path(
                base_inertial["prim_path"], f"{base_location}.prim_path"
            ),
            "mass_kg": _finite_number(
                base_inertial["mass_kg"],
                f"{base_location}.mass_kg",
                positive=True,
            ),
            "center_of_mass_m": _finite_vector(
                base_inertial["center_of_mass_m"],
                f"{base_location}.center_of_mass_m",
                length=3,
            ),
            "inertia_kg_m2": _inertia_matrix(
                base_inertial["inertia_kg_m2"],
                f"{base_location}.inertia_kg_m2",
            ),
        }

    expected_links = mass["expected_link_masses"]
    if not isinstance(expected_links, list) or len(expected_links) != 5:
        raise ReportValidationError(
            f"{location}.expected_link_masses must contain exactly 5 entries"
        )
    expected_by_path: dict[str, float] = {}
    expected_paths: list[str] = []
    for index, link in enumerate(expected_links):
        link_location = f"{location}.expected_link_masses[{index}]"
        if not isinstance(link, Mapping):
            raise ReportValidationError(f"{link_location} must be a mapping")
        _exact_keys(link, {"prim_path", "mass_kg"}, link_location)
        path = _absolute_prim_path(
            link["prim_path"], f"{link_location}.prim_path"
        )
        expected_paths.append(path)
        expected_by_path[path] = _finite_number(
            link["mass_kg"], f"{link_location}.mass_kg", positive=True
        )
    if expected_paths != sorted(expected_paths) or len(expected_by_path) != 5:
        raise ReportValidationError(
            f"{location}.expected_link_masses must have sorted unique paths"
        )
    expected_total = _finite_number(
        mass["expected_total_mass_kg"],
        f"{location}.expected_total_mass_kg",
        positive=True,
    )
    if not _numbers_close(sum(expected_by_path.values()), expected_total):
        raise ReportValidationError(
            f"{location}.expected_total_mass_kg must equal link mass sum"
        )
    if normalized_base is not None:
        path = normalized_base["prim_path"]
        if path not in expected_by_path or not _numbers_close(
            expected_by_path[path], normalized_base["mass_kg"]
        ):
            raise ReportValidationError(
                f"{location}.base_inertial mass must match expected_link_masses"
            )

    overlay_id = _required_string(
        mass["overlay_id"], f"{location}.overlay_id"
    )
    if overlay_id != f"mass_collision_profile/{profile_id}":
        raise ReportValidationError(
            f"{location}.overlay_id must link to profile_id"
        )
    overlay_identifier = _required_string(
        mass["overlay_identifier"], f"{location}.overlay_identifier"
    )
    if not overlay_identifier.startswith("anon:"):
        raise ReportValidationError(
            f"{location}.overlay_identifier must identify an anonymous layer"
        )
    _validate_lowercase_sha256(
        mass["overlay_sha256"], f"{location}.overlay_sha256"
    )
    if mass["stage_usd_readback_verified"] is not True:
        raise ReportValidationError(
            f"{location}.stage_usd_readback_verified must be true"
        )

    tensor = _required_mapping(mass, "physics_tensor", location)
    tensor_location = f"{location}.physics_tensor"
    tensor_keys = {
        "schema_version",
        "profile_id",
        "links",
        "total_mass_kg",
        "physics_tensor_readback_verified",
    }
    _exact_keys(tensor, tensor_keys, tensor_location)
    _schema_version_one(
        tensor["schema_version"], f"{tensor_location}.schema_version"
    )
    tensor_profile_id = _required_string(
        tensor["profile_id"], f"{tensor_location}.profile_id"
    )
    if tensor_profile_id != profile_id:
        raise ReportValidationError(
            f"{tensor_location}.profile_id must match Stage evidence"
        )
    tensor_links = tensor["links"]
    if not isinstance(tensor_links, list) or len(tensor_links) != 5:
        raise ReportValidationError(
            f"{tensor_location}.links must contain exactly 5 entries"
        )
    tensor_by_path: dict[str, dict[str, Any]] = {}
    tensor_paths: list[str] = []
    for index, link in enumerate(tensor_links):
        link_location = f"{tensor_location}.links[{index}]"
        if not isinstance(link, Mapping):
            raise ReportValidationError(f"{link_location} must be a mapping")
        _exact_keys(
            link,
            {
                "name",
                "prim_path",
                "mass_kg",
                "center_of_mass_m",
                "inertia_kg_m2",
            },
            link_location,
        )
        name = _required_string(link["name"], f"{link_location}.name")
        path = _absolute_prim_path(
            link["prim_path"], f"{link_location}.prim_path"
        )
        if Path(path).name != name:
            raise ReportValidationError(
                f"{link_location}.name must match prim_path basename"
            )
        tensor_paths.append(path)
        tensor_by_path[path] = {
            "mass_kg": _finite_number(
                link["mass_kg"], f"{link_location}.mass_kg", positive=True
            ),
            "center_of_mass_m": _finite_vector(
                link["center_of_mass_m"],
                f"{link_location}.center_of_mass_m",
                length=3,
            ),
            "inertia_kg_m2": _inertia_matrix(
                link["inertia_kg_m2"], f"{link_location}.inertia_kg_m2"
            ),
        }
    if tensor_paths != sorted(tensor_paths) or len(tensor_by_path) != 5:
        raise ReportValidationError(
            f"{tensor_location}.links must have sorted unique prim paths"
        )
    if set(tensor_by_path) != set(expected_by_path):
        raise ReportValidationError(
            f"{tensor_location}.links must match expected_link_masses paths"
        )
    for path, expected_mass in expected_by_path.items():
        if not _numbers_close(tensor_by_path[path]["mass_kg"], expected_mass):
            raise ReportValidationError(
                f"{tensor_location}.links masses must match Stage evidence"
            )
    tensor_total = _finite_number(
        tensor["total_mass_kg"],
        f"{tensor_location}.total_mass_kg",
        positive=True,
    )
    if not _numbers_close(tensor_total, expected_total):
        raise ReportValidationError(
            f"{tensor_location}.total_mass_kg must match "
            "expected_total_mass_kg"
        )
    if not _numbers_close(
        sum(link["mass_kg"] for link in tensor_by_path.values()), tensor_total
    ):
        raise ReportValidationError(
            f"{tensor_location}.total_mass_kg must equal tensor link mass sum"
        )
    if normalized_base is not None:
        actual_base = tensor_by_path[normalized_base["prim_path"]]
        if any(
            not _numbers_close(actual, expected)
            for actual, expected in zip(
                actual_base["center_of_mass_m"],
                normalized_base["center_of_mass_m"],
                strict=True,
            )
        ):
            raise ReportValidationError(
                f"{tensor_location} base COM must match Stage evidence"
            )
        if any(
            not _numbers_close(actual, expected)
            for actual_row, expected_row in zip(
                actual_base["inertia_kg_m2"],
                normalized_base["inertia_kg_m2"],
                strict=True,
            )
            for actual, expected in zip(
                actual_row, expected_row, strict=True
            )
        ):
            raise ReportValidationError(
                f"{tensor_location} base inertia must match Stage evidence"
            )
    if tensor["physics_tensor_readback_verified"] is not True:
        raise ReportValidationError(
            f"{tensor_location}.physics_tensor_readback_verified must be true"
        )


def _validate_control_graph_provenance(control: Mapping[str, Any]) -> None:
    location = "runtime_provenance.control_graph"
    _exact_keys(control, _CONTROL_GRAPH_KEYS, location)
    _schema_version_one(control["schema_version"], f"{location}.schema_version")
    mode = control["wheel_command_application"]
    if mode not in _WHEEL_COMMAND_APPLICATIONS:
        raise ReportValidationError(
            f"{location}.wheel_command_application must be one of "
            f"{sorted(_WHEEL_COMMAND_APPLICATIONS)}"
        )
    topology = _required_mapping(control, "topology", location)
    topology_location = f"{location}.topology"
    _exact_keys(
        topology,
        {
            "graph_path",
            "pipeline_stage",
            "nodes",
            "connections",
            "command_writers",
        },
        topology_location,
    )
    graph_path = _absolute_prim_path(
        topology["graph_path"], f"{topology_location}.graph_path"
    )
    if not graph_path.startswith("/World/Graphs/"):
        raise ReportValidationError(
            f"{topology_location}.graph_path must be under /World/Graphs"
        )
    if topology["pipeline_stage"] != "execution":
        raise ReportValidationError(
            f"{topology_location}.pipeline_stage must equal execution"
        )

    nodes = topology["nodes"]
    if not isinstance(nodes, list) or not nodes:
        raise ReportValidationError(
            f"{topology_location}.nodes must be a non-empty list"
        )
    normalized_nodes: list[tuple[str, str]] = []
    for index, node in enumerate(nodes):
        node_location = f"{topology_location}.nodes[{index}]"
        if not isinstance(node, Mapping):
            raise ReportValidationError(f"{node_location} must be a mapping")
        _exact_keys(node, {"name", "type_name"}, node_location)
        normalized_nodes.append(
            (
                _required_string(node["name"], f"{node_location}.name"),
                _required_string(
                    node["type_name"], f"{node_location}.type_name"
                ),
            )
        )
    if normalized_nodes != sorted(normalized_nodes):
        raise ReportValidationError(f"{topology_location}.nodes must be sorted")
    node_names = [name for name, _ in normalized_nodes]
    if len(set(node_names)) != len(node_names):
        raise ReportValidationError(
            f"{topology_location}.nodes must contain unique names"
        )

    connections = topology["connections"]
    if not isinstance(connections, list) or not connections:
        raise ReportValidationError(
            f"{topology_location}.connections must be a non-empty list"
        )
    normalized_connections: list[tuple[str, str]] = []
    for index, connection in enumerate(connections):
        child_location = f"{topology_location}.connections[{index}]"
        if not isinstance(connection, Mapping):
            raise ReportValidationError(f"{child_location} must be a mapping")
        _exact_keys(connection, {"source", "target"}, child_location)
        source = _required_string(
            connection["source"], f"{child_location}.source"
        )
        target = _required_string(
            connection["target"], f"{child_location}.target"
        )
        if (
            "." not in source
            or "." not in target
            or source.split(".", 1)[0] not in node_names
            or target.split(".", 1)[0] not in node_names
        ):
            raise ReportValidationError(
                f"{child_location} must reference known nodes"
            )
        normalized_connections.append((source, target))
    if normalized_connections != sorted(normalized_connections):
        raise ReportValidationError(
            f"{topology_location}.connections must be sorted"
        )
    if len(set(normalized_connections)) != len(normalized_connections):
        raise ReportValidationError(
            f"{topology_location}.connections must be unique"
        )

    writers = topology["command_writers"]
    expected_writer_names = _WHEEL_COMMAND_APPLICATIONS[mode]
    if not isinstance(writers, list) or len(writers) != len(
        expected_writer_names
    ):
        raise ReportValidationError(
            f"{topology_location}.command_writers disagree with "
            "wheel_command_application"
        )
    writer_names: list[str] = []
    wheel_joint_names: list[str] = []
    for index, writer in enumerate(writers):
        writer_location = f"{topology_location}.command_writers[{index}]"
        if not isinstance(writer, Mapping):
            raise ReportValidationError(f"{writer_location} must be a mapping")
        _exact_keys(
            writer, {"node", "target_prim", "joint_names"}, writer_location
        )
        writer_name = _required_string(
            writer["node"], f"{writer_location}.node"
        )
        writer_names.append(writer_name)
        if writer_name not in node_names:
            raise ReportValidationError(
                f"{writer_location}.node must reference a topology node"
            )
        _absolute_prim_path(
            writer["target_prim"], f"{writer_location}.target_prim"
        )
        joint_names = writer["joint_names"]
        if not isinstance(joint_names, list) or not joint_names:
            raise ReportValidationError(
                f"{writer_location}.joint_names must be a non-empty list"
            )
        wheel_joint_names.extend(
            _required_string(
                name, f"{writer_location}.joint_names[{joint_index}]"
            )
            for joint_index, name in enumerate(joint_names)
        )
    if tuple(writer_names) != expected_writer_names:
        raise ReportValidationError(
            f"{topology_location}.command_writers disagree with "
            "wheel_command_application"
        )
    if len(wheel_joint_names) != 4 or len(set(wheel_joint_names)) != 4:
        raise ReportValidationError(
            f"{topology_location} command writers must cover four unique "
            "wheel joints"
        )
    topology_sha256 = _validate_lowercase_sha256(
        control["topology_sha256"], f"{location}.topology_sha256"
    )
    canonical_topology = json.dumps(
        topology,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )
    actual_sha256 = hashlib.sha256(
        canonical_topology.encode("utf-8")
    ).hexdigest()
    if not hmac.compare_digest(topology_sha256, actual_sha256):
        raise ReportValidationError(
            f"{location}.topology_sha256 does not match canonical topology"
        )
    if control["materialized_readback_verified"] is not True:
        raise ReportValidationError(
            f"{location}.materialized_readback_verified must be true"
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
        or schema_version not in {3, 4, 5, 6, 7}
    ):
        raise ReportValidationError(
            "runtime_provenance.schema_version must be integer "
            "3, 4, 5, 6, or 7"
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
    if schema_version in {5, 6, 7}:
        expected_root_keys.add("ground_topology")
    if schema_version == 7:
        expected_root_keys.add("control_graph")
    if set(provenance) != expected_root_keys:
        raise ReportValidationError(
            "runtime_provenance keys must be exactly "
            f"{sorted(expected_root_keys)}"
        )

    robot = _required_mapping(provenance, "robot", "runtime_provenance")
    robot_keys = {"config", "asset", "solver"}
    if schema_version in {4, 5, 6, 7}:
        robot_keys.add("kinematics")
    if schema_version == 7:
        robot_keys.update({"wheel_velocity_drive", "mass_collision"})
    if set(robot) != robot_keys:
        raise ReportValidationError(
            "runtime_provenance.robot keys must be exactly "
            f"{sorted(robot_keys)}"
        )
    for name in ("config", "asset"):
        input_file = _required_mapping(
            robot, name, "runtime_provenance.robot"
        )
        input_file_keys = {"path", "sha256"}
        if schema_version == 7 and name == "config":
            input_file_keys.add("schema_version")
        if set(input_file) != input_file_keys:
            raise ReportValidationError(
                f"runtime_provenance.robot.{name} keys must be exactly "
                f"{sorted(input_file_keys)}"
            )
        if schema_version == 7 and name == "config":
            config_schema_version = input_file.get("schema_version")
            if (
                isinstance(config_schema_version, bool)
                or not isinstance(config_schema_version, int)
                or config_schema_version != 3
            ):
                raise ReportValidationError(
                    "runtime_provenance.robot.config.schema_version must be "
                    "integer 3"
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

    if schema_version in {4, 5, 6, 7}:
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

    if schema_version == 7:
        _validate_wheel_velocity_drive_provenance(
            _required_mapping(
                robot,
                "wheel_velocity_drive",
                "runtime_provenance.robot",
            ),
            _required_mapping(robot, "config", "runtime_provenance.robot"),
        )
        _validate_mass_collision_provenance(
            _required_mapping(
                robot,
                "mass_collision",
                "runtime_provenance.robot",
            ),
            _required_mapping(robot, "asset", "runtime_provenance.robot"),
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
    if schema_version in {6, 7}:
        simulation_keys.add("reset_strategy")
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
    if schema_version in {5, 6, 7}:
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
        if schema_version in {6, 7}:
            reset_strategy = _required_mapping(
                simulation,
                "reset_strategy",
                "runtime_provenance.simulation",
            )
            _validate_reset_strategy_provenance(
                reset_strategy,
                contact,
                ground_topology,
            )
    if schema_version == 7:
        _validate_control_graph_provenance(
            _required_mapping(
                provenance,
                "control_graph",
                "runtime_provenance",
            )
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
