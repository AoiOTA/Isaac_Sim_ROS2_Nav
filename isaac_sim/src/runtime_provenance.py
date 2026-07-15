"""Capture immutable evidence for the inputs loaded by one Isaac process."""

from __future__ import annotations

import hashlib
import json
import math
import re
from pathlib import Path
import subprocess
from typing import Any, Mapping

from isaac_sim.src.robot.kinematics_config import load_robot_config_contract


class RuntimeProvenanceError(RuntimeError):
    """Raised when a runtime input cannot be bound to reproducible evidence."""


_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_CONTACT_SNAPSHOT_KEYS = {
    "profile_path",
    "profile_sha256",
    "profile_id",
    "profile_mode",
    "overlay_identifier",
    "overlay_sha256",
    "explicit_materials",
    "thresholds_authored",
    "scene",
    "wheel_colliders",
    "ground_colliders",
    "wheel_bindings",
    "ground_bindings",
    "wheel_material",
    "ground_material",
    "stage_usd_readback_verified",
}
_GROUND_TOPOLOGY_SNAPSHOT_KEYS = {
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
_PROFILE_FLAGS = {
    "legacy_baseline": (False, False),
    "threshold_only": (False, True),
    "explicit_material": (True, True),
}
_COMBINE_MODES = {"average", "min", "multiply", "max"}
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


def file_sha256(path: str | Path) -> str:
    """Return the SHA256 digest of one required regular file."""

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise RuntimeProvenanceError(
            f"runtime provenance input is not a file: {source}"
        )
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_metadata(root: str | Path) -> dict[str, object]:
    """Snapshot the source revision and dirty state at Isaac startup."""

    repository = Path(root).expanduser().resolve()

    def git(*arguments: str) -> str:
        try:
            result = subprocess.run(
                ["git", "-C", str(repository), *arguments],
                text=True,
                capture_output=True,
                check=False,
                timeout=3.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeProvenanceError(
                f"failed to inspect Git repository {repository}: {exc}"
            ) from exc
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeProvenanceError(
                f"Git metadata command failed in {repository}: {detail}"
            )
        return result.stdout.strip()

    commit = git("rev-parse", "HEAD")
    branch = git("branch", "--show-current") or "detached"
    status = git("status", "--porcelain", "--untracked-files=normal")
    return {
        "commit": commit,
        "branch": branch,
        "dirty": bool(status),
    }


def _solver_iteration_pair(
    values: object,
    *,
    location: str,
) -> tuple[int, int]:
    if not isinstance(values, (list, tuple)) or len(values) != 2:
        raise RuntimeProvenanceError(
            f"{location} must contain position and velocity solver counts"
        )
    counts: list[int] = []
    for name, value in zip(("position", "velocity"), values):
        if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or not 1 <= value <= 255
        ):
            raise RuntimeProvenanceError(
                f"{location}.{name} must be an integer in [1, 255]"
            )
        counts.append(value)
    return counts[0], counts[1]


def stage_articulation_solver_iterations(
    stage: Any,
    articulation_root: str,
) -> tuple[int, int]:
    """Read solver counts from the effective composed Stage attributes."""

    prim = stage.GetPrimAtPath(articulation_root)
    if not prim or not prim.IsValid():
        raise RuntimeProvenanceError(
            f"runtime provenance articulation root is invalid: {articulation_root}"
        )
    values = []
    for name in (
        "physxArticulation:solverPositionIterationCount",
        "physxArticulation:solverVelocityIterationCount",
    ):
        attribute = prim.GetAttribute(name)
        if not attribute or not attribute.IsValid():
            raise RuntimeProvenanceError(
                f"runtime provenance solver attribute is missing: {name}"
            )
        values.append(attribute.Get())
    return _solver_iteration_pair(values, location="stage solver")


def _canonical_json(value: object, *, location: str) -> str:
    try:
        return json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
    except (TypeError, ValueError) as exc:
        raise RuntimeProvenanceError(
            f"{location} is not canonical strict-JSON compatible: {exc}"
        ) from exc


def _sha256_digest(value: object, *, location: str) -> str:
    if not isinstance(value, str) or not _SHA256_PATTERN.fullmatch(value):
        raise RuntimeProvenanceError(f"{location} must be a SHA256 hex digest")
    return value


def _nonempty_string(value: object, *, location: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise RuntimeProvenanceError(f"{location} must be a non-empty string")
    return value


def _absolute_prim_path(value: object, *, location: str) -> str:
    path = _nonempty_string(value, location=location)
    if not path.startswith("/") or path == "/" or "//" in path:
        raise RuntimeProvenanceError(
            f"{location} must be an absolute USD prim path"
        )
    return path


def _finite_nonnegative(value: object, *, location: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) < 0.0
    ):
        raise RuntimeProvenanceError(
            f"{location} must be a finite non-negative number"
        )
    return float(value)


def _nonnegative_integer(
    value: object,
    *,
    location: str,
    allow_zero: bool,
) -> int:
    minimum = 0 if allow_zero else 1
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        qualifier = "non-negative" if allow_zero else "positive"
        raise RuntimeProvenanceError(
            f"{location} must be a {qualifier} integer"
        )
    return value


def _path_list(
    value: object,
    *,
    location: str,
    expected_count: int,
) -> list[str]:
    if not isinstance(value, list) or len(value) != expected_count:
        raise RuntimeProvenanceError(
            f"{location} must contain exactly {expected_count} paths"
        )
    paths = [
        _absolute_prim_path(path, location=f"{location}[{index}]")
        for index, path in enumerate(value)
    ]
    if len(set(paths)) != len(paths):
        raise RuntimeProvenanceError(f"{location} must contain unique paths")
    return paths


def _material_snapshot(
    value: object,
    *,
    location: str,
) -> Mapping[str, Any] | None:
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise RuntimeProvenanceError(f"{location} must be a mapping or null")
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
        raise RuntimeProvenanceError(
            f"{location} keys must be exactly {sorted(required)}"
        )
    _absolute_prim_path(value["material_path"], location=f"{location}.material_path")
    static_friction = _finite_nonnegative(
        value["static_friction"],
        location=f"{location}.static_friction",
    )
    dynamic_friction = _finite_nonnegative(
        value["dynamic_friction"],
        location=f"{location}.dynamic_friction",
    )
    if dynamic_friction > static_friction:
        raise RuntimeProvenanceError(
            f"{location}.dynamic_friction must not exceed static_friction"
        )
    restitution = _finite_nonnegative(
        value["restitution"],
        location=f"{location}.restitution",
    )
    if restitution > 1.0:
        raise RuntimeProvenanceError(f"{location}.restitution must be in [0, 1]")
    for name in ("friction_combine_mode", "restitution_combine_mode"):
        mode = value[name]
        if mode is not None and (
            not isinstance(mode, str) or mode not in _COMBINE_MODES
        ):
            raise RuntimeProvenanceError(
                f"{location}.{name} must be null or a supported combine mode"
            )
    for name in (
        "friction_combine_mode_authored",
        "restitution_combine_mode_authored",
    ):
        if not isinstance(value[name], bool):
            raise RuntimeProvenanceError(f"{location}.{name} must be boolean")
    return value


def _binding_snapshots(
    value: object,
    *,
    location: str,
    collider_paths: list[str],
    material: Mapping[str, Any] | None,
    require_direct_material: bool,
) -> None:
    if not isinstance(value, list) or len(value) != len(collider_paths):
        raise RuntimeProvenanceError(
            f"{location} must contain one binding per collider"
        )
    seen: list[str] = []
    effective_paths: list[str | None] = []
    direct_paths: list[str | None] = []
    required_keys = {
        "collider_path",
        "direct_physics_material_path",
        "effective_physics_material_path",
    }
    for index, binding in enumerate(value):
        binding_location = f"{location}[{index}]"
        if not isinstance(binding, Mapping) or set(binding) != required_keys:
            raise RuntimeProvenanceError(
                f"{binding_location} must contain collider, direct, and "
                "effective material paths"
            )
        seen.append(
            _absolute_prim_path(
                binding["collider_path"],
                location=f"{binding_location}.collider_path",
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
                else _absolute_prim_path(
                    path,
                    location=f"{binding_location}.{name}",
                )
            )
    if set(seen) != set(collider_paths) or len(set(seen)) != len(seen):
        raise RuntimeProvenanceError(
            f"{location} must have a one-to-one collider mapping"
        )
    material_path = None if material is None else material["material_path"]
    if material_path is None:
        if any(path is not None for path in effective_paths):
            raise RuntimeProvenanceError(
                f"{location} effective bindings require a material snapshot"
            )
    elif set(effective_paths) != {material_path}:
        raise RuntimeProvenanceError(
            f"{location} effective bindings must match the material snapshot"
        )
    if require_direct_material and set(direct_paths) != {material_path}:
        raise RuntimeProvenanceError(
            f"{location} direct bindings must match the explicit material"
        )


def _capture_ground_topology_provenance(
    config: Any,
    stage: Any,
    ground_topology_snapshot: object | None,
) -> tuple[dict[str, object], Any]:
    try:
        from isaac_sim.src.stage.ground_topology import (
            capture_ground_topology_snapshot,
            collider_paths_sha256,
            load_ground_topology_profile,
        )

        profile = load_ground_topology_profile(
            config.files.ground_topology_profile
        )
        captured = capture_ground_topology_snapshot(stage, config)
        if ground_topology_snapshot is not None:
            supplied = ground_topology_snapshot
            if hasattr(supplied, "to_dict") and callable(supplied.to_dict):
                supplied = supplied.to_dict()
            fresh = captured
            if hasattr(fresh, "to_dict") and callable(fresh.to_dict):
                fresh = fresh.to_dict()
            if _canonical_json(
                supplied,
                location="supplied ground topology snapshot",
            ) != _canonical_json(
                fresh,
                location="fresh ground topology Stage readback",
            ):
                raise RuntimeProvenanceError(
                    "supplied ground topology snapshot is stale or differs "
                    "from the current Stage readback"
                )
    except Exception as exc:
        raise RuntimeProvenanceError(
            f"failed to capture effective Stage ground topology: {exc}"
        ) from exc
    if hasattr(captured, "to_dict") and callable(captured.to_dict):
        captured = captured.to_dict()
    if not isinstance(captured, Mapping):
        raise RuntimeProvenanceError(
            "runtime ground topology snapshot must be a mapping"
        )
    snapshot = json.loads(
        _canonical_json(captured, location="runtime ground topology snapshot")
    )
    if set(snapshot) != _GROUND_TOPOLOGY_SNAPSHOT_KEYS:
        raise RuntimeProvenanceError(
            "runtime ground topology snapshot keys must be exactly "
            f"{sorted(_GROUND_TOPOLOGY_SNAPSHOT_KEYS)}"
        )

    expected_profile_path = str(
        Path(config.files.ground_topology_profile).expanduser().resolve()
    )
    if snapshot["profile_path"] != expected_profile_path:
        raise RuntimeProvenanceError(
            "runtime ground topology profile path does not match config: "
            f"expected={expected_profile_path}, "
            f"actual={snapshot['profile_path']}"
        )
    expected_profile_sha256 = file_sha256(
        config.files.ground_topology_profile
    )
    if (
        snapshot["profile_sha256"] != expected_profile_sha256
        or snapshot["profile_sha256"] != profile.sha256
    ):
        raise RuntimeProvenanceError(
            "runtime ground topology profile SHA256 does not match config"
        )
    expected_identity = (
        profile.identifier,
        profile.environment_id,
        profile.operation,
    )
    actual_identity = (
        snapshot["profile_id"],
        snapshot["environment_id"],
        snapshot["operation"],
    )
    if actual_identity != expected_identity:
        raise RuntimeProvenanceError(
            "runtime ground topology profile identity does not match config: "
            f"expected={expected_identity}, actual={actual_identity}"
        )
    if snapshot["environment_id"] != config.environment.identifier:
        raise RuntimeProvenanceError(
            "runtime ground topology environment does not match project config"
        )

    expected_source_path = str(
        Path(config.environment.source_asset).expanduser().resolve()
    )
    if snapshot["source_asset_path"] != expected_source_path:
        raise RuntimeProvenanceError(
            "runtime ground topology source asset path does not match config: "
            f"expected={expected_source_path}, "
            f"actual={snapshot['source_asset_path']}"
        )
    expected_source_sha256 = file_sha256(config.environment.source_asset)
    if (
        snapshot["source_asset_sha256"] != expected_source_sha256
        or snapshot["source_asset_sha256"] != profile.source_asset_sha256
    ):
        raise RuntimeProvenanceError(
            "runtime ground topology source asset SHA256 does not match config"
        )

    overlay_identifier = _nonempty_string(
        snapshot["overlay_identifier"],
        location="runtime ground topology overlay identifier",
    )
    if not overlay_identifier.startswith("anon:"):
        raise RuntimeProvenanceError(
            "runtime ground topology overlay identifier must name an "
            "anonymous layer"
        )
    _sha256_digest(
        snapshot["overlay_sha256"],
        location="runtime ground topology overlay SHA256",
    )
    if snapshot["stage_usd_readback_verified"] is not True:
        raise RuntimeProvenanceError(
            "runtime ground topology Stage USD readback must be verified"
        )

    source_resolver = config.environment.ground_colliders
    expected_source_contract = (
        tuple(profile.source.required_prim_paths),
        tuple(profile.source.semantic_classes),
        profile.source.collider_count,
    )
    actual_source_contract = (
        tuple(source_resolver.required_prim_paths),
        tuple(source_resolver.semantic_classes),
        source_resolver.expected_enabled_count,
    )
    if actual_source_contract != expected_source_contract:
        raise RuntimeProvenanceError(
            "runtime ground topology source collider contract does not match "
            f"project config: expected={expected_source_contract}, "
            f"actual={actual_source_contract}"
        )

    collider_sets: dict[str, list[str]] = {}
    for name, spec, allow_zero in (
        ("source", profile.source, False),
        ("target", profile.target, False),
        ("disabled", profile.disabled, True),
    ):
        count_key = f"{name}_collider_count"
        paths_key = f"{name}_colliders"
        sha256_key = f"{name}_collider_paths_sha256"
        count = _nonnegative_integer(
            snapshot[count_key],
            location=f"runtime ground topology {name} collider count",
            allow_zero=allow_zero,
        )
        paths = _path_list(
            snapshot[paths_key],
            location=f"runtime ground topology {name} colliders",
            expected_count=count,
        )
        if paths != sorted(paths):
            raise RuntimeProvenanceError(
                f"runtime ground topology {name} colliders must be sorted"
            )
        declared_sha256 = _sha256_digest(
            snapshot[sha256_key],
            location=f"runtime ground topology {name} collider paths SHA256",
        )
        actual_sha256 = collider_paths_sha256(paths)
        if (
            count != spec.collider_count
            or declared_sha256 != spec.collider_paths_sha256
            or declared_sha256 != actual_sha256
        ):
            raise RuntimeProvenanceError(
                f"runtime ground topology {name} collider set does not match "
                "the profile and canonical path hash"
            )
        required_paths = getattr(spec, "required_prim_paths", ())
        missing_required = sorted(set(required_paths) - set(paths))
        if missing_required:
            raise RuntimeProvenanceError(
                f"runtime ground topology {name} collider set is missing "
                f"required paths: {missing_required}"
            )
        collider_sets[name] = paths

    source_set = set(collider_sets["source"])
    target_set = set(collider_sets["target"])
    disabled_set = set(collider_sets["disabled"])
    if (
        target_set & disabled_set
        or target_set | disabled_set != source_set
    ):
        raise RuntimeProvenanceError(
            "runtime ground topology target and disabled colliders must form "
            "an exact disjoint partition of source colliders"
        )
    return snapshot, profile


def _capture_contact_provenance(
    config: Any,
    stage: Any,
    ground_topology: Mapping[str, Any],
    ground_topology_profile: Any,
    contact_snapshot: object | None,
) -> dict[str, object]:
    try:
        from isaac_sim.src.stage.contact_setup import (
            capture_contact_profile_snapshot,
            load_contact_profile,
        )

        profile = load_contact_profile(config.files.contact_profile)
        captured = capture_contact_profile_snapshot(stage, config)
        if contact_snapshot is not None:
            supplied = contact_snapshot
            if hasattr(supplied, "to_dict") and callable(supplied.to_dict):
                supplied = supplied.to_dict()
            fresh = captured
            if hasattr(fresh, "to_dict") and callable(fresh.to_dict):
                fresh = fresh.to_dict()
            if _canonical_json(
                supplied,
                location="supplied contact snapshot",
            ) != _canonical_json(
                fresh,
                location="fresh contact Stage readback",
            ):
                raise RuntimeProvenanceError(
                    "supplied contact snapshot is stale or differs from the "
                    "current Stage readback"
                )
    except Exception as exc:
        raise RuntimeProvenanceError(
            f"failed to capture effective Stage contact profile: {exc}"
        ) from exc
    if hasattr(captured, "to_dict") and callable(captured.to_dict):
        captured = captured.to_dict()
    if not isinstance(captured, Mapping):
        raise RuntimeProvenanceError(
            "runtime contact snapshot must be a mapping"
        )
    # A strict-JSON round trip normalizes dataclass tuples to lists and rejects
    # arbitrary Python objects before anything is exposed as ROS parameters.
    snapshot = json.loads(
        _canonical_json(captured, location="runtime contact snapshot")
    )
    if set(snapshot) != _CONTACT_SNAPSHOT_KEYS:
        raise RuntimeProvenanceError(
            "runtime contact snapshot keys must be exactly "
            f"{sorted(_CONTACT_SNAPSHOT_KEYS)}"
        )

    expected_profile_path = str(
        Path(config.files.contact_profile).expanduser().resolve()
    )
    if snapshot["profile_path"] != expected_profile_path:
        raise RuntimeProvenanceError(
            "runtime contact profile path does not match config: "
            f"expected={expected_profile_path}, actual={snapshot['profile_path']}"
        )
    expected_profile_sha256 = file_sha256(config.files.contact_profile)
    if snapshot["profile_sha256"] != expected_profile_sha256:
        raise RuntimeProvenanceError(
            "runtime contact profile SHA256 does not match config"
        )
    if snapshot["profile_id"] != profile.identifier:
        raise RuntimeProvenanceError(
            "runtime contact profile id does not match config"
        )
    if snapshot["profile_mode"] != profile.mode:
        raise RuntimeProvenanceError(
            "runtime contact profile mode does not match config"
        )
    _sha256_digest(
        snapshot["overlay_sha256"],
        location="runtime contact overlay SHA256",
    )
    overlay_identifier = _nonempty_string(
        snapshot["overlay_identifier"],
        location="runtime contact overlay identifier",
    )
    if not overlay_identifier.startswith("anon:"):
        raise RuntimeProvenanceError(
            "runtime contact overlay identifier must name an anonymous layer"
        )
    if snapshot["stage_usd_readback_verified"] is not True:
        raise RuntimeProvenanceError(
            "runtime contact Stage USD readback must be verified"
        )
    expected_flags = _PROFILE_FLAGS[profile.mode]
    actual_flags = (
        snapshot["explicit_materials"],
        snapshot["thresholds_authored"],
    )
    if not all(isinstance(flag, bool) for flag in actual_flags):
        raise RuntimeProvenanceError(
            "runtime contact profile flags must be boolean"
        )
    if actual_flags != expected_flags:
        raise RuntimeProvenanceError(
            f"runtime contact flags disagree with profile mode {profile.mode}"
        )

    scene = snapshot["scene"]
    if not isinstance(scene, Mapping):
        raise RuntimeProvenanceError("runtime contact scene must be a mapping")
    scene_keys = {
        "physics_scene_path",
        "friction_correlation_distance",
        "friction_offset_threshold",
        "friction_type",
    }
    if set(scene) != scene_keys:
        raise RuntimeProvenanceError(
            f"runtime contact scene keys must be exactly {sorted(scene_keys)}"
        )
    if scene["physics_scene_path"] != config.simulation.expected_physics_scene:
        raise RuntimeProvenanceError(
            "runtime contact PhysicsScene does not match config"
        )
    _finite_nonnegative(
        scene["friction_correlation_distance"],
        location="runtime contact friction correlation distance",
    )
    _finite_nonnegative(
        scene["friction_offset_threshold"],
        location="runtime contact friction offset threshold",
    )
    friction_type = scene["friction_type"]
    if friction_type is not None:
        _nonempty_string(
            friction_type,
            location="runtime contact friction type",
        )

    wheel_joints = list(config.robot.wheel_joints)
    if (
        len(wheel_joints) != 4
        or len(set(wheel_joints)) != 4
        or not all(isinstance(name, str) and name for name in wheel_joints)
    ):
        raise RuntimeProvenanceError(
            "runtime contact config must contain four unique wheel joints"
        )
    topology_ground_colliders = list(ground_topology["target_colliders"])
    ground_expected_count = ground_topology["target_collider_count"]
    wheel_colliders = _path_list(
        snapshot["wheel_colliders"],
        location="runtime contact wheel colliders",
        expected_count=4,
    )
    ground_colliders = _path_list(
        snapshot["ground_colliders"],
        location="runtime contact ground colliders",
        expected_count=ground_expected_count,
    )
    if ground_colliders != topology_ground_colliders:
        raise RuntimeProvenanceError(
            "runtime contact ground colliders do not match the readback-verified "
            "ground topology target"
        )

    wheel_material = _material_snapshot(
        snapshot["wheel_material"],
        location="runtime contact wheel material",
    )
    ground_material = _material_snapshot(
        snapshot["ground_material"],
        location="runtime contact ground material",
    )
    explicit = profile.mode == "explicit_material"
    if explicit and (wheel_material is None or ground_material is None):
        raise RuntimeProvenanceError(
            "runtime explicit contact profile requires wheel and ground materials"
        )
    _binding_snapshots(
        snapshot["wheel_bindings"],
        location="runtime contact wheel bindings",
        collider_paths=wheel_colliders,
        material=wheel_material,
        require_direct_material=explicit,
    )
    _binding_snapshots(
        snapshot["ground_bindings"],
        location="runtime contact ground bindings",
        collider_paths=ground_colliders,
        material=ground_material,
        require_direct_material=explicit,
    )
    if explicit:
        assert wheel_material is not None
        assert ground_material is not None
        for material, name in (
            (wheel_material, "wheel"),
            (ground_material, "ground"),
        ):
            for combine in ("friction", "restitution"):
                if (
                    material[f"{combine}_combine_mode"] not in _COMBINE_MODES
                    or material[f"{combine}_combine_mode_authored"] is not True
                ):
                    raise RuntimeProvenanceError(
                        f"runtime explicit {name} {combine} combine mode "
                        "must be authored"
                    )
        if (
            wheel_material["friction_combine_mode"]
            != ground_material["friction_combine_mode"]
            or wheel_material["restitution_combine_mode"]
            != ground_material["restitution_combine_mode"]
        ):
            raise RuntimeProvenanceError(
                "runtime explicit wheel and ground combine modes disagree"
            )

    snapshot["collider_contract"] = {
        "wheel_joint_names": wheel_joints,
        "wheel_expected_count": 4,
        "ground_required_prim_paths": list(
            ground_topology_profile.target.required_prim_paths
        ),
        "ground_semantic_classes": list(
            ground_topology_profile.target.semantic_classes
        ),
        "ground_expected_enabled_count": ground_expected_count,
    }
    return snapshot


def _capture_reset_strategy_provenance(
    config: Any,
    ground_topology: Mapping[str, Any],
    contact: Mapping[str, Any],
    snapshot: object,
) -> dict[str, object]:
    location = "runtime reset strategy snapshot"
    if not isinstance(snapshot, Mapping):
        raise RuntimeProvenanceError(f"{location} must be a mapping")
    if set(snapshot) != _RESET_STRATEGY_KEYS:
        raise RuntimeProvenanceError(
            f"{location} keys must be exactly "
            f"{sorted(_RESET_STRATEGY_KEYS)}"
        )
    schema_version = snapshot.get("schema_version")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 1
    ):
        raise RuntimeProvenanceError(
            f"{location}.schema_version must be integer 1"
        )
    identifier = _nonempty_string(snapshot.get("id"), location=f"{location}.id")
    if not _IDENTIFIER_PATTERN.fullmatch(identifier):
        raise RuntimeProvenanceError(
            f"{location}.id must be a path-safe identifier"
        )
    expected_semantics = _RESET_STRATEGY_SEMANTICS.get(identifier)
    if expected_semantics is None:
        raise RuntimeProvenanceError(f"{location}.id is unsupported")
    configured = config.simulation.reset_strategy
    if (
        configured.schema_version != schema_version
        or configured.identifier != identifier
    ):
        raise RuntimeProvenanceError(
            f"{location} does not match the configured reset strategy "
            "config identity"
        )

    lift_distance_m = snapshot.get("lift_distance_m")
    separation_step_count = snapshot.get("separation_step_count")
    recontact_step_count = snapshot.get("recontact_step_count")
    if (
        not isinstance(lift_distance_m, float)
        or not math.isfinite(lift_distance_m)
        or isinstance(separation_step_count, bool)
        or not isinstance(separation_step_count, int)
        or isinstance(recontact_step_count, bool)
        or not isinstance(recontact_step_count, int)
    ):
        raise RuntimeProvenanceError(
            f"{location} strategy semantics must use one finite float and "
            "integer step counts"
        )
    if (
        lift_distance_m,
        separation_step_count,
        recontact_step_count,
    ) != expected_semantics:
        raise RuntimeProvenanceError(
            f"{location} strategy semantics do not match id {identifier!r}"
        )

    probe = snapshot.get("contact_probe")
    probe_location = f"{location}.contact_probe"
    if not isinstance(probe, Mapping):
        raise RuntimeProvenanceError(f"{probe_location} must be a mapping")
    if set(probe) != _RESET_CONTACT_PROBE_KEYS:
        raise RuntimeProvenanceError(
            f"{probe_location} keys must be exactly "
            f"{sorted(_RESET_CONTACT_PROBE_KEYS)}"
        )
    probe_schema_version = probe.get("schema_version")
    if (
        isinstance(probe_schema_version, bool)
        or not isinstance(probe_schema_version, int)
        or probe_schema_version != 1
    ):
        raise RuntimeProvenanceError(
            f"{probe_location}.schema_version must be integer 1"
        )
    if probe.get("enabled") is not True:
        raise RuntimeProvenanceError(f"{probe_location}.enabled must be true")
    if probe.get("stage_usd_readback_verified") is not True:
        raise RuntimeProvenanceError(
            f"{probe_location} Stage USD readback must be verified"
        )

    wheel_bindings = probe.get("wheel_bindings")
    if not isinstance(wheel_bindings, list) or len(wheel_bindings) != 4:
        raise RuntimeProvenanceError(
            f"{probe_location}.wheel_bindings must contain exactly 4 entries"
        )
    normalized_wheel_bindings: list[dict[str, str]] = []
    joint_names: list[str] = []
    wheel_link_paths: list[str] = []
    for index, binding in enumerate(wheel_bindings):
        binding_location = f"{probe_location}.wheel_bindings[{index}]"
        if not isinstance(binding, Mapping) or set(binding) != {
            "joint_name",
            "wheel_link_path",
        }:
            raise RuntimeProvenanceError(
                f"{binding_location} keys must be exactly "
                "['joint_name', 'wheel_link_path']"
            )
        joint_name = _nonempty_string(
            binding.get("joint_name"),
            location=f"{binding_location}.joint_name",
        )
        if not _IDENTIFIER_PATTERN.fullmatch(joint_name):
            raise RuntimeProvenanceError(
                f"{binding_location}.joint_name must be path-safe"
            )
        wheel_link_path = _absolute_prim_path(
            binding.get("wheel_link_path"),
            location=f"{binding_location}.wheel_link_path",
        )
        joint_names.append(joint_name)
        wheel_link_paths.append(wheel_link_path)
        normalized_wheel_bindings.append(
            {
                "joint_name": joint_name,
                "wheel_link_path": wheel_link_path,
            }
        )
    if len(set(wheel_link_paths)) != len(wheel_link_paths):
        raise RuntimeProvenanceError(
            f"{probe_location}.wheel link paths must be unique"
        )
    expected_joint_names = list(config.robot.wheel_joints)
    if joint_names != expected_joint_names:
        raise RuntimeProvenanceError(
            f"{probe_location}.wheel_bindings must preserve config wheel "
            "joint order"
        )
    wheel_colliders = _path_list(
        contact.get("wheel_colliders"),
        location="runtime contact wheel colliders",
        expected_count=4,
    )
    for index, (wheel_link_path, collider_path) in enumerate(
        zip(wheel_link_paths, wheel_colliders, strict=True)
    ):
        if not (
            collider_path == wheel_link_path
            or collider_path.startswith(f"{wheel_link_path.rstrip('/')}/")
        ):
            raise RuntimeProvenanceError(
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
        raise RuntimeProvenanceError(
            f"{probe_location}.wheel_count must be 4"
        )

    ground_filter_paths = probe.get("ground_filter_paths")
    if not isinstance(ground_filter_paths, list) or not ground_filter_paths:
        raise RuntimeProvenanceError(
            f"{probe_location}.ground_filter_paths must be a non-empty list"
        )
    normalized_ground_paths = [
        _absolute_prim_path(
            path,
            location=f"{probe_location}.ground_filter_paths[{index}]",
        )
        for index, path in enumerate(ground_filter_paths)
    ]
    if normalized_ground_paths != sorted(normalized_ground_paths):
        raise RuntimeProvenanceError(
            f"{probe_location}.ground_filter_paths must be sorted"
        )
    if len(set(normalized_ground_paths)) != len(normalized_ground_paths):
        raise RuntimeProvenanceError(
            f"{probe_location}.ground_filter_paths must be unique"
        )
    if normalized_ground_paths != ground_topology["target_colliders"]:
        raise RuntimeProvenanceError(
            f"{probe_location}.ground_filter_paths disagree with the ground "
            "topology target"
        )
    ground_filter_count = probe.get("ground_filter_count")
    if (
        isinstance(ground_filter_count, bool)
        or not isinstance(ground_filter_count, int)
        or ground_filter_count != len(normalized_ground_paths)
    ):
        raise RuntimeProvenanceError(
            f"{probe_location}.ground_filter_count must equal the path count"
        )
    max_contact_count = probe.get("max_contact_count")
    if (
        isinstance(max_contact_count, bool)
        or not isinstance(max_contact_count, int)
        or max_contact_count != 128
    ):
        raise RuntimeProvenanceError(
            f"{probe_location}.max_contact_count must be 128"
        )
    report_threshold_n = probe.get("report_threshold_n")
    if not isinstance(report_threshold_n, float) or report_threshold_n != 0.0:
        raise RuntimeProvenanceError(
            f"{probe_location}.report_threshold_n must be 0.0"
        )

    return {
        "schema_version": schema_version,
        "id": identifier,
        "lift_distance_m": lift_distance_m,
        "separation_step_count": separation_step_count,
        "recontact_step_count": recontact_step_count,
        "contact_probe": {
            "schema_version": probe_schema_version,
            "enabled": True,
            "wheel_bindings": normalized_wheel_bindings,
            "wheel_count": wheel_count,
            "ground_filter_paths": normalized_ground_paths,
            "ground_filter_count": ground_filter_count,
            "max_contact_count": max_contact_count,
            "report_threshold_n": report_threshold_n,
            "stage_usd_readback_verified": True,
        },
    }


def capture_runtime_provenance(
    config: Any,
    stage: Any,
    *,
    articulation_usd_solver_iterations: tuple[int, int],
    repository_root: str | Path,
    reset_strategy_snapshot: object,
    ground_topology_snapshot: object | None = None,
    contact_snapshot: object | None = None,
) -> dict[str, object]:
    """Capture the effective files and in-memory Stage loaded by Isaac."""

    stage_solver_iterations = stage_articulation_solver_iterations(
        stage,
        config.robot.articulation_root,
    )
    articulation_usd_solver_iterations = _solver_iteration_pair(
        articulation_usd_solver_iterations,
        location="initialized articulation USD readback solver",
    )
    if stage_solver_iterations != articulation_usd_solver_iterations:
        raise RuntimeProvenanceError(
            "Stage and initialized articulation USD solver readback disagree: "
            f"stage={stage_solver_iterations}, "
            f"articulation_usd={articulation_usd_solver_iterations}"
        )
    root_layer_source = stage.GetRootLayer().ExportToString()
    if not isinstance(root_layer_source, str) or not root_layer_source:
        raise RuntimeProvenanceError(
            "composed Stage root layer could not be exported"
        )
    ground_topology, ground_topology_profile = (
        _capture_ground_topology_provenance(
            config,
            stage,
            ground_topology_snapshot,
        )
    )
    contact = _capture_contact_provenance(
        config,
        stage,
        ground_topology,
        ground_topology_profile,
        contact_snapshot,
    )
    reset_strategy = _capture_reset_strategy_provenance(
        config,
        ground_topology,
        contact,
        reset_strategy_snapshot,
    )
    try:
        robot_contract = load_robot_config_contract(config.files.robot)
    except (OSError, ValueError) as exc:
        raise RuntimeProvenanceError(
            "runtime robot kinematics/controller contract is invalid: "
            f"{exc}"
        ) from exc
    kinematics = robot_contract.kinematics
    return {
        "schema_version": 6,
        "robot": {
            "config": {
                "path": str(config.files.robot),
                "sha256": file_sha256(config.files.robot),
            },
            "asset": {
                "path": str(config.robot.asset_path),
                "sha256": file_sha256(config.robot.asset_path),
            },
            "solver": {
                "position_iterations": articulation_usd_solver_iterations[0],
                "velocity_iterations": articulation_usd_solver_iterations[1],
                "stage_articulation_usd_readback_verified": True,
            },
            "kinematics": {
                "profile_id": kinematics.kinematics_profile_id,
                "lifecycle": kinematics.lifecycle,
                "wheel_radius_m": kinematics.wheel_radius,
                "wheel_width_m": kinematics.wheel_width,
                "geometric_track_width_m": (
                    kinematics.geometric_track_width
                ),
                "effective_track_width_m": (
                    kinematics.effective_track_width
                ),
                "controller_contract_verified": True,
            },
        },
        "environment": {
            "id": config.environment.identifier,
            "project_stage": {
                "path": str(config.environment.project_stage),
                "sha256": file_sha256(config.environment.project_stage),
            },
            "source_asset": {
                "path": str(config.environment.source_asset),
                "sha256": file_sha256(config.environment.source_asset),
            },
            "asset_root": str(config.asset_root),
            "asset_version": config.asset_root.name,
            "composed_root_layer_sha256": hashlib.sha256(
                root_layer_source.encode("utf-8")
            ).hexdigest(),
        },
        "simulation": {
            "navigation_mode": config.simulation.navigation_mode,
            "odometry_mode": config.simulation.odometry_mode,
            "physics_hz": config.simulation.physics_hz,
            "reset_strategy": reset_strategy,
        },
        "ground_topology": ground_topology,
        "contact": contact,
        "git": git_metadata(repository_root),
    }


def runtime_provenance_parameters(
    provenance: Mapping[str, Any],
) -> dict[str, str | bool | int | float]:
    """Flatten a captured snapshot into read-only ROS parameter values."""

    robot = provenance["robot"]
    environment = provenance["environment"]
    simulation = provenance["simulation"]
    ground_topology_json = _canonical_json(
        provenance["ground_topology"],
        location="runtime ground topology provenance",
    )
    contact_json = _canonical_json(
        provenance["contact"],
        location="runtime contact provenance",
    )
    reset_strategy_json = _canonical_json(
        simulation["reset_strategy"],
        location="runtime reset strategy provenance",
    )
    git = provenance["git"]
    return {
        "runtime_provenance.schema_version": provenance["schema_version"],
        "runtime_provenance.robot.config.path": robot["config"]["path"],
        "runtime_provenance.robot.config.sha256": robot["config"]["sha256"],
        "runtime_provenance.robot.asset.path": robot["asset"]["path"],
        "runtime_provenance.robot.asset.sha256": robot["asset"]["sha256"],
        "runtime_provenance.robot.solver.position_iterations": robot["solver"][
            "position_iterations"
        ],
        "runtime_provenance.robot.solver.velocity_iterations": robot["solver"][
            "velocity_iterations"
        ],
        "runtime_provenance.robot.solver."
        "stage_articulation_usd_readback_verified": robot["solver"][
            "stage_articulation_usd_readback_verified"
        ],
        "runtime_provenance.robot.kinematics.profile_id": robot[
            "kinematics"
        ]["profile_id"],
        "runtime_provenance.robot.kinematics.lifecycle": robot[
            "kinematics"
        ]["lifecycle"],
        "runtime_provenance.robot.kinematics.wheel_radius_m": robot[
            "kinematics"
        ]["wheel_radius_m"],
        "runtime_provenance.robot.kinematics.wheel_width_m": robot[
            "kinematics"
        ]["wheel_width_m"],
        "runtime_provenance.robot.kinematics.geometric_track_width_m": robot[
            "kinematics"
        ]["geometric_track_width_m"],
        "runtime_provenance.robot.kinematics.effective_track_width_m": robot[
            "kinematics"
        ]["effective_track_width_m"],
        "runtime_provenance.robot.kinematics.controller_contract_verified": (
            robot["kinematics"]["controller_contract_verified"]
        ),
        "runtime_provenance.environment.id": environment["id"],
        "runtime_provenance.environment.project_stage.path": environment[
            "project_stage"
        ]["path"],
        "runtime_provenance.environment.project_stage.sha256": environment[
            "project_stage"
        ]["sha256"],
        "runtime_provenance.environment.source_asset.path": environment[
            "source_asset"
        ]["path"],
        "runtime_provenance.environment.source_asset.sha256": environment[
            "source_asset"
        ]["sha256"],
        "runtime_provenance.environment.asset_root": environment["asset_root"],
        "runtime_provenance.environment.asset_version": environment[
            "asset_version"
        ],
        "runtime_provenance.environment.composed_root_layer_sha256": environment[
            "composed_root_layer_sha256"
        ],
        "runtime_provenance.simulation.navigation_mode": simulation[
            "navigation_mode"
        ],
        "runtime_provenance.simulation.odometry_mode": simulation["odometry_mode"],
        "runtime_provenance.simulation.physics_hz": simulation["physics_hz"],
        "runtime_provenance.simulation.reset_strategy.json": (
            reset_strategy_json
        ),
        "runtime_provenance.simulation.reset_strategy.sha256": hashlib.sha256(
            reset_strategy_json.encode("utf-8")
        ).hexdigest(),
        "runtime_provenance.ground_topology.json": ground_topology_json,
        "runtime_provenance.ground_topology.sha256": hashlib.sha256(
            ground_topology_json.encode("utf-8")
        ).hexdigest(),
        "runtime_provenance.contact.json": contact_json,
        "runtime_provenance.contact.sha256": hashlib.sha256(
            contact_json.encode("utf-8")
        ).hexdigest(),
        "runtime_provenance.git.commit": git["commit"],
        "runtime_provenance.git.branch": git["branch"],
        "runtime_provenance.git.dirty": git["dirty"],
    }
