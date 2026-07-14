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
_PROFILE_FLAGS = {
    "legacy_baseline": (False, False),
    "threshold_only": (False, True),
    "explicit_material": (True, True),
}
_COMBINE_MODES = {"average", "min", "multiply", "max"}


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


def _capture_contact_provenance(
    config: Any,
    stage: Any,
    contact_snapshot: object | None,
) -> dict[str, object]:
    try:
        from isaac_sim.src.stage.contact_setup import (
            capture_contact_profile_snapshot,
            load_contact_profile,
        )

        profile = load_contact_profile(config.files.contact_profile)
        captured = (
            capture_contact_profile_snapshot(stage, config)
            if contact_snapshot is None
            else contact_snapshot
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
    ground_config = config.environment.ground_colliders
    ground_expected_count = ground_config.expected_enabled_count
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
    missing_required = sorted(
        set(ground_config.required_prim_paths) - set(ground_colliders)
    )
    if missing_required:
        raise RuntimeProvenanceError(
            "runtime contact is missing required ground collider paths: "
            f"{missing_required}"
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
            ground_config.required_prim_paths
        ),
        "ground_semantic_classes": list(ground_config.semantic_classes),
        "ground_expected_enabled_count": ground_expected_count,
    }
    return snapshot


def capture_runtime_provenance(
    config: Any,
    stage: Any,
    *,
    articulation_usd_solver_iterations: tuple[int, int],
    repository_root: str | Path,
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
    contact = _capture_contact_provenance(
        config,
        stage,
        contact_snapshot,
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
        "schema_version": 4,
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
        },
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
    contact_json = _canonical_json(
        provenance["contact"],
        location="runtime contact provenance",
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
        "runtime_provenance.contact.json": contact_json,
        "runtime_provenance.contact.sha256": hashlib.sha256(
            contact_json.encode("utf-8")
        ).hexdigest(),
        "runtime_provenance.git.commit": git["commit"],
        "runtime_provenance.git.branch": git["branch"],
        "runtime_provenance.git.dirty": git["dirty"],
    }
