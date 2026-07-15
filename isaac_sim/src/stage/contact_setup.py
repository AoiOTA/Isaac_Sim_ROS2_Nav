"""Strict, reversible contact profiles authored in an anonymous USD layer.

The committed robot and environment assets remain read-only.  A selected
profile is applied to the Stage session layer before PhysX is initialized,
then read back from the effective composed Stage for provenance consumers.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from pathlib import Path
import re
from typing import Any, Mapping

import yaml

from isaac_sim.src.config import ProjectConfig


class ContactSetupError(RuntimeError):
    """Raised when a contact profile or effective Stage contract is invalid."""


_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_PROFILE_LAYER_MARKER = "isaac_nav_contact_profile_layer"
_PROFILE_MODES = {"legacy_baseline", "threshold_only", "explicit_material"}
_COMBINE_MODES = {"average", "min", "multiply", "max"}
_WHEEL_MATERIAL_PATH = "/World/PhysicsMaterials/ContactProfile/Wheel"
_GROUND_MATERIAL_PATH = "/World/PhysicsMaterials/ContactProfile/Ground"


@dataclass(frozen=True)
class SceneContactSpec:
    friction_correlation_distance: float
    friction_offset_threshold: float


@dataclass(frozen=True)
class ContactMaterialSpec:
    static_friction: float
    dynamic_friction: float
    restitution: float
    friction_combine_mode: str
    restitution_combine_mode: str


@dataclass(frozen=True)
class ContactProfile:
    path: Path
    sha256: str
    identifier: str
    mode: str
    scene: SceneContactSpec | None
    wheel_material: ContactMaterialSpec | None
    ground_material: ContactMaterialSpec | None


@dataclass(frozen=True)
class SceneContactSnapshot:
    physics_scene_path: str
    friction_correlation_distance: float
    friction_offset_threshold: float
    friction_type: str | None


@dataclass(frozen=True)
class ContactBindingSnapshot:
    collider_path: str
    direct_physics_material_path: str | None
    effective_physics_material_path: str | None


@dataclass(frozen=True)
class ContactMaterialSnapshot:
    material_path: str
    static_friction: float
    dynamic_friction: float
    restitution: float
    friction_combine_mode: str | None
    restitution_combine_mode: str | None
    friction_combine_mode_authored: bool
    restitution_combine_mode_authored: bool


@dataclass(frozen=True)
class ContactProfileSnapshot:
    """JSON-serializable effective Stage evidence for later provenance."""

    profile_path: str
    profile_sha256: str
    profile_id: str
    profile_mode: str
    overlay_identifier: str
    overlay_sha256: str
    explicit_materials: bool
    thresholds_authored: bool
    scene: SceneContactSnapshot
    wheel_colliders: tuple[str, ...]
    ground_colliders: tuple[str, ...]
    wheel_bindings: tuple[ContactBindingSnapshot, ...]
    ground_bindings: tuple[ContactBindingSnapshot, ...]
    wheel_material: ContactMaterialSnapshot | None
    ground_material: ContactMaterialSnapshot | None
    stage_usd_readback_verified: bool

    def to_dict(self) -> dict[str, object]:
        """Return a nested snapshot suitable for JSON and ROS provenance."""

        return asdict(self)


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ContactSetupError(f"{name} must be a mapping")
    return dict(value)


def _keys(value: Mapping[str, Any], allowed: set[str], name: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ContactSetupError(f"unknown {name} keys: {unknown}")


def _required(value: Mapping[str, Any], key: str, name: str) -> Any:
    if key not in value:
        raise ContactSetupError(f"missing required key {name}.{key}")
    return value[key]


def _nonnegative_number(value: Any, name: str) -> float:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0.0
    ):
        raise ContactSetupError(f"{name} must be a finite non-negative number")
    return float(value)


def _parse_scene(value: Any) -> SceneContactSpec:
    data = _mapping(value, "contact_profile.scene")
    _keys(
        data,
        {"friction_correlation_distance", "friction_offset_threshold"},
        "contact_profile.scene",
    )
    return SceneContactSpec(
        friction_correlation_distance=_nonnegative_number(
            _required(
                data,
                "friction_correlation_distance",
                "contact_profile.scene",
            ),
            "contact_profile.scene.friction_correlation_distance",
        ),
        friction_offset_threshold=_nonnegative_number(
            _required(
                data,
                "friction_offset_threshold",
                "contact_profile.scene",
            ),
            "contact_profile.scene.friction_offset_threshold",
        ),
    )


def _parse_material(value: Any, name: str) -> ContactMaterialSpec:
    data = _mapping(value, name)
    _keys(
        data,
        {
            "static_friction",
            "dynamic_friction",
            "restitution",
            "friction_combine_mode",
            "restitution_combine_mode",
        },
        name,
    )
    static_friction = _nonnegative_number(
        _required(data, "static_friction", name),
        f"{name}.static_friction",
    )
    dynamic_friction = _nonnegative_number(
        _required(data, "dynamic_friction", name),
        f"{name}.dynamic_friction",
    )
    if dynamic_friction > static_friction:
        raise ContactSetupError(
            f"{name}.dynamic_friction must not exceed static_friction"
        )
    restitution = _nonnegative_number(
        _required(data, "restitution", name),
        f"{name}.restitution",
    )
    if restitution > 1.0:
        raise ContactSetupError(f"{name}.restitution must be in [0, 1]")
    friction_combine_mode = _required(data, "friction_combine_mode", name)
    restitution_combine_mode = _required(
        data,
        "restitution_combine_mode",
        name,
    )
    if (
        not isinstance(friction_combine_mode, str)
        or friction_combine_mode not in _COMBINE_MODES
    ):
        raise ContactSetupError(
            f"{name}.friction_combine_mode must be one of "
            f"{sorted(_COMBINE_MODES)}"
        )
    if (
        not isinstance(restitution_combine_mode, str)
        or restitution_combine_mode not in _COMBINE_MODES
    ):
        raise ContactSetupError(
            f"{name}.restitution_combine_mode must be one of "
            f"{sorted(_COMBINE_MODES)}"
        )
    return ContactMaterialSpec(
        static_friction=static_friction,
        dynamic_friction=dynamic_friction,
        restitution=restitution,
        friction_combine_mode=friction_combine_mode,
        restitution_combine_mode=restitution_combine_mode,
    )


def load_contact_profile(path: str | Path) -> ContactProfile:
    """Load one profile with mode-dependent, fail-closed schema validation."""

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise ContactSetupError(f"contact profile is not a file: {source}")
    with source.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream)
    data = _mapping(loaded, "contact_profile")
    _keys(
        data,
        {
            "schema_version",
            "id",
            "mode",
            "scene",
            "wheel_material",
            "ground_material",
        },
        "contact_profile",
    )
    schema_version = _required(data, "schema_version", "contact_profile")
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 1
    ):
        raise ContactSetupError("contact_profile.schema_version must be 1")
    identifier = _required(data, "id", "contact_profile")
    if not isinstance(identifier, str) or not _IDENTIFIER_PATTERN.fullmatch(
        identifier
    ):
        raise ContactSetupError("contact_profile.id must be path-safe")
    mode = _required(data, "mode", "contact_profile")
    if not isinstance(mode, str) or mode not in _PROFILE_MODES:
        raise ContactSetupError(
            f"contact_profile.mode must be one of {sorted(_PROFILE_MODES)}"
        )

    scene = _parse_scene(data["scene"]) if "scene" in data else None
    wheel_material = (
        _parse_material(data["wheel_material"], "contact_profile.wheel_material")
        if "wheel_material" in data
        else None
    )
    ground_material = (
        _parse_material(
            data["ground_material"],
            "contact_profile.ground_material",
        )
        if "ground_material" in data
        else None
    )
    if mode == "legacy_baseline":
        if scene is not None or wheel_material is not None or ground_material is not None:
            raise ContactSetupError(
                "legacy_baseline must not author scene or material values"
            )
    elif mode == "threshold_only":
        if scene is None:
            raise ContactSetupError("threshold_only requires contact_profile.scene")
        if wheel_material is not None or ground_material is not None:
            raise ContactSetupError("threshold_only must not author materials")
    else:
        if scene is None or wheel_material is None or ground_material is None:
            raise ContactSetupError(
                "explicit_material requires scene, wheel_material, and "
                "ground_material"
            )
        if (
            wheel_material.friction_combine_mode
            != ground_material.friction_combine_mode
            or wheel_material.restitution_combine_mode
            != ground_material.restitution_combine_mode
        ):
            raise ContactSetupError(
                "explicit wheel and ground materials must use identical "
                "combine modes"
            )
    return ContactProfile(
        path=source,
        sha256=_file_sha256(source),
        identifier=identifier,
        mode=mode,
        scene=scene,
        wheel_material=wheel_material,
        ground_material=ground_material,
    )


def _collision_is_enabled(prim: object) -> bool:
    from pxr import UsdPhysics

    if not prim or not prim.IsValid() or not prim.IsActive():
        return False
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        return False
    return UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True


def resolve_wheel_colliders(stage: object, config: ProjectConfig) -> tuple[str, ...]:
    """Resolve exactly one enabled collider for each configured wheel joint."""

    from pxr import Usd, UsdPhysics

    paths: list[str] = []
    for joint_name in config.robot.wheel_joints:
        joints = [
            prim
            for prim in stage.TraverseAll()
            if prim.IsActive()
            and prim.GetName() == joint_name
            and prim.IsA(UsdPhysics.RevoluteJoint)
        ]
        if len(joints) != 1:
            raise ContactSetupError(
                f"wheel joint {joint_name!r} resolved to {len(joints)} prims"
            )
        joint = UsdPhysics.RevoluteJoint(joints[0])
        bodies = tuple(joint.GetBody0Rel().GetTargets()) + tuple(
            joint.GetBody1Rel().GetTargets()
        )
        wheel_paths = [
            path for path in bodies if str(path) != config.robot.base_link_prim
        ]
        if len(wheel_paths) != 1:
            raise ContactSetupError(
                f"wheel joint {joint_name!r} must connect one wheel body"
            )
        wheel = stage.GetPrimAtPath(wheel_paths[0])
        colliders = [
            prim
            for prim in Usd.PrimRange(wheel)
            if _collision_is_enabled(prim)
        ]
        if len(colliders) != 1:
            raise ContactSetupError(
                f"wheel joint {joint_name!r} resolved to {len(colliders)} "
                "enabled colliders"
            )
        paths.append(str(colliders[0].GetPath()))
    if len(paths) != 4 or len(set(paths)) != 4:
        raise ContactSetupError("contact setup requires four unique wheel colliders")
    return tuple(paths)


def resolve_ground_colliders(stage: object, config: ProjectConfig) -> tuple[str, ...]:
    """Consume the already-applied, readback-verified topology target set."""

    from isaac_sim.src.stage.ground_topology import (
        capture_ground_topology_snapshot,
    )

    return capture_ground_topology_snapshot(stage, config).target_colliders


def _remove_existing_contact_layers(stage: object) -> None:
    from pxr import Sdf

    session = stage.GetSessionLayer()
    for identifier in list(session.subLayerPaths):
        layer = Sdf.Layer.Find(identifier)
        if layer and layer.customLayerData.get(_PROFILE_LAYER_MARKER) is True:
            session.subLayerPaths.remove(identifier)


def _create_contact_layer(stage: object, profile: ContactProfile):
    from pxr import Sdf

    _remove_existing_contact_layers(stage)
    layer = Sdf.Layer.CreateAnonymous(f"contact_{profile.identifier}.usda")
    layer.customLayerData = {
        _PROFILE_LAYER_MARKER: True,
        "contact_profile_id": profile.identifier,
        "contact_profile_mode": profile.mode,
        "contact_profile_sha256": profile.sha256,
    }
    stage.GetSessionLayer().subLayerPaths.insert(0, layer.identifier)
    return layer


def _apply_schema_token(prim: object, schema_name: str) -> None:
    if schema_name not in prim.GetAppliedSchemas():
        if not prim.AddAppliedSchema(schema_name):
            raise ContactSetupError(
                f"failed to apply {schema_name} to {prim.GetPath()}"
            )


def _author_scene(stage: object, config: ProjectConfig, spec: SceneContactSpec) -> None:
    from pxr import Sdf

    prim = stage.GetPrimAtPath(config.simulation.expected_physics_scene)
    if not prim or not prim.IsValid():
        raise ContactSetupError(
            "contact profile requires the configured PhysicsScene to exist "
            "before PhysicsSetup"
        )
    _apply_schema_token(prim, "PhysxSceneAPI")
    prim.CreateAttribute(
        "physxScene:frictionCorrelationDistance",
        Sdf.ValueTypeNames.Float,
    ).Set(spec.friction_correlation_distance)
    prim.CreateAttribute(
        "physxScene:frictionOffsetThreshold",
        Sdf.ValueTypeNames.Float,
    ).Set(spec.friction_offset_threshold)
    prim.CreateAttribute(
        "physxScene:frictionType",
        Sdf.ValueTypeNames.Token,
        False,
        Sdf.VariabilityUniform,
    ).Set("patch")


def _author_material(stage: object, path: str, spec: ContactMaterialSpec):
    from pxr import Sdf, UsdPhysics, UsdShade

    material = UsdShade.Material.Define(stage, path)
    prim = material.GetPrim()
    material_api = UsdPhysics.MaterialAPI.Apply(prim)
    material_api.CreateStaticFrictionAttr().Set(spec.static_friction)
    material_api.CreateDynamicFrictionAttr().Set(spec.dynamic_friction)
    material_api.CreateRestitutionAttr().Set(spec.restitution)
    _apply_schema_token(prim, "PhysxMaterialAPI")
    prim.CreateAttribute(
        "physxMaterial:frictionCombineMode",
        Sdf.ValueTypeNames.Token,
        False,
        Sdf.VariabilityUniform,
    ).Set(spec.friction_combine_mode)
    prim.CreateAttribute(
        "physxMaterial:restitutionCombineMode",
        Sdf.ValueTypeNames.Token,
        False,
        Sdf.VariabilityUniform,
    ).Set(spec.restitution_combine_mode)
    return material


def _bind_material(stage: object, collider_paths: tuple[str, ...], material: object) -> None:
    from pxr import UsdShade

    for path in collider_paths:
        prim = stage.GetPrimAtPath(path)
        if not _collision_is_enabled(prim):
            raise ContactSetupError(f"cannot bind invalid collider: {path}")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            material,
            UsdShade.Tokens.weakerThanDescendants,
            "physics",
        )


def _find_contact_layer(stage: object, profile: ContactProfile):
    from pxr import Sdf

    matches = []
    for identifier in stage.GetSessionLayer().subLayerPaths:
        layer = Sdf.Layer.Find(identifier)
        if layer and layer.customLayerData.get(_PROFILE_LAYER_MARKER) is True:
            matches.append(layer)
    if len(matches) != 1:
        raise ContactSetupError(
            f"expected one active contact session layer, found {len(matches)}"
        )
    layer = matches[0]
    metadata = layer.customLayerData
    expected = {
        "contact_profile_id": profile.identifier,
        "contact_profile_mode": profile.mode,
        "contact_profile_sha256": profile.sha256,
    }
    actual = {key: metadata.get(key) for key in expected}
    if actual != expected:
        raise ContactSetupError(
            f"contact session layer metadata mismatch: expected={expected}, "
            f"actual={actual}"
        )
    return layer


def _read_float_attribute(prim: object, name: str) -> float:
    attribute = prim.GetAttribute(name)
    value = attribute.Get() if attribute.IsValid() else None
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0.0
    ):
        raise ContactSetupError(
            f"effective Stage attribute {prim.GetPath()}.{name} is invalid: "
            f"{value!r}"
        )
    return float(value)


def _read_scene(stage: object, config: ProjectConfig) -> SceneContactSnapshot:
    path = config.simulation.expected_physics_scene
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise ContactSetupError(f"effective PhysicsScene is missing: {path}")
    friction_type_attr = prim.GetAttribute("physxScene:frictionType")
    friction_type = (
        str(friction_type_attr.Get())
        if friction_type_attr.IsValid() and friction_type_attr.Get() is not None
        else None
    )
    return SceneContactSnapshot(
        physics_scene_path=path,
        friction_correlation_distance=_read_float_attribute(
            prim,
            "physxScene:frictionCorrelationDistance",
        ),
        friction_offset_threshold=_read_float_attribute(
            prim,
            "physxScene:frictionOffsetThreshold",
        ),
        friction_type=friction_type,
    )


def _path_or_none(path: object) -> str | None:
    if path is None:
        return None
    value = str(path)
    return value or None


def _read_bindings(
    stage: object,
    collider_paths: tuple[str, ...],
) -> tuple[ContactBindingSnapshot, ...]:
    from pxr import UsdPhysics, UsdShade

    snapshots = []
    for path in collider_paths:
        api = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath(path))
        direct_path = _path_or_none(
            api.GetDirectBinding("physics").GetMaterialPath()
        )
        material, _relationship = api.ComputeBoundMaterial("physics")
        effective_path = (
            str(material.GetPath())
            if material
            and material.GetPrim().IsValid()
            and material.GetPrim().HasAPI(UsdPhysics.MaterialAPI)
            else None
        )
        snapshots.append(
            ContactBindingSnapshot(
                collider_path=path,
                direct_physics_material_path=direct_path,
                effective_physics_material_path=effective_path,
            )
        )
    return tuple(snapshots)


def _read_group_material(
    stage: object,
    bindings: tuple[ContactBindingSnapshot, ...],
    group: str,
) -> ContactMaterialSnapshot | None:
    from pxr import UsdPhysics

    paths = {binding.effective_physics_material_path for binding in bindings}
    if paths == {None}:
        return None
    if None in paths or len(paths) != 1:
        raise ContactSetupError(
            f"{group} colliders do not resolve one homogeneous physics material: "
            f"{sorted(str(path) for path in paths)}"
        )
    material_path = next(iter(paths))
    assert material_path is not None
    prim = stage.GetPrimAtPath(material_path)
    if not prim or not prim.IsValid() or not prim.HasAPI(UsdPhysics.MaterialAPI):
        raise ContactSetupError(
            f"{group} effective material lacks PhysicsMaterialAPI: {material_path}"
        )
    friction_attr = prim.GetAttribute("physxMaterial:frictionCombineMode")
    restitution_attr = prim.GetAttribute("physxMaterial:restitutionCombineMode")
    friction_value = (
        str(friction_attr.Get())
        if friction_attr.IsValid() and friction_attr.Get() is not None
        else None
    )
    restitution_value = (
        str(restitution_attr.Get())
        if restitution_attr.IsValid() and restitution_attr.Get() is not None
        else None
    )
    return ContactMaterialSnapshot(
        material_path=material_path,
        static_friction=_read_float_attribute(
            prim,
            "physics:staticFriction",
        ),
        dynamic_friction=_read_float_attribute(
            prim,
            "physics:dynamicFriction",
        ),
        restitution=_read_float_attribute(
            prim,
            "physics:restitution",
        ),
        friction_combine_mode=friction_value,
        restitution_combine_mode=restitution_value,
        friction_combine_mode_authored=(
            friction_attr.IsValid() and friction_attr.HasAuthoredValueOpinion()
        ),
        restitution_combine_mode_authored=(
            restitution_attr.IsValid()
            and restitution_attr.HasAuthoredValueOpinion()
        ),
    )


def _close(actual: float, expected: float) -> bool:
    return math.isclose(actual, expected, rel_tol=1e-7, abs_tol=1e-9)


def _verify_material_snapshot(
    actual: ContactMaterialSnapshot | None,
    expected_path: str,
    expected: ContactMaterialSpec,
    group: str,
) -> None:
    if actual is None:
        raise ContactSetupError(f"explicit {group} material did not resolve")
    if actual.material_path != expected_path:
        raise ContactSetupError(
            f"explicit {group} material path mismatch: {actual.material_path}"
        )
    pairs = (
        ("static_friction", actual.static_friction, expected.static_friction),
        ("dynamic_friction", actual.dynamic_friction, expected.dynamic_friction),
        ("restitution", actual.restitution, expected.restitution),
    )
    for name, value, target in pairs:
        if not _close(value, target):
            raise ContactSetupError(
                f"explicit {group} {name} readback mismatch: "
                f"expected={target}, actual={value}"
            )
    if (
        actual.friction_combine_mode != expected.friction_combine_mode
        or actual.restitution_combine_mode != expected.restitution_combine_mode
        or not actual.friction_combine_mode_authored
        or not actual.restitution_combine_mode_authored
    ):
        raise ContactSetupError(f"explicit {group} combine-mode readback mismatch")


def capture_contact_profile_snapshot(
    stage: object,
    config: ProjectConfig,
) -> ContactProfileSnapshot:
    """Re-read and verify the active contact profile from the composed Stage."""

    profile = load_contact_profile(config.files.contact_profile)
    layer = _find_contact_layer(stage, profile)
    wheel_colliders = resolve_wheel_colliders(stage, config)
    ground_colliders = resolve_ground_colliders(stage, config)
    scene = _read_scene(stage, config)
    wheel_bindings = _read_bindings(stage, wheel_colliders)
    ground_bindings = _read_bindings(stage, ground_colliders)
    wheel_material = _read_group_material(
        stage,
        wheel_bindings,
        "wheel",
    )
    ground_material = _read_group_material(
        stage,
        ground_bindings,
        "ground",
    )

    if profile.scene is not None:
        if not _close(
            scene.friction_correlation_distance,
            profile.scene.friction_correlation_distance,
        ) or not _close(
            scene.friction_offset_threshold,
            profile.scene.friction_offset_threshold,
        ):
            raise ContactSetupError("scene contact threshold readback mismatch")
        if scene.friction_type != "patch":
            raise ContactSetupError("contact profiles require patch friction")
    if profile.mode == "explicit_material":
        assert profile.wheel_material is not None
        assert profile.ground_material is not None
        if {
            binding.direct_physics_material_path for binding in wheel_bindings
        } != {_WHEEL_MATERIAL_PATH}:
            raise ContactSetupError("explicit wheel physics bindings mismatch")
        if {
            binding.direct_physics_material_path for binding in ground_bindings
        } != {_GROUND_MATERIAL_PATH}:
            raise ContactSetupError("explicit ground physics bindings mismatch")
        _verify_material_snapshot(
            wheel_material,
            _WHEEL_MATERIAL_PATH,
            profile.wheel_material,
            "wheel",
        )
        _verify_material_snapshot(
            ground_material,
            _GROUND_MATERIAL_PATH,
            profile.ground_material,
            "ground",
        )

    overlay_source = layer.ExportToString()
    return ContactProfileSnapshot(
        profile_path=str(profile.path),
        profile_sha256=profile.sha256,
        profile_id=profile.identifier,
        profile_mode=profile.mode,
        overlay_identifier=layer.identifier,
        overlay_sha256=hashlib.sha256(
            overlay_source.encode("utf-8")
        ).hexdigest(),
        explicit_materials=profile.mode == "explicit_material",
        thresholds_authored=profile.scene is not None,
        scene=scene,
        wheel_colliders=wheel_colliders,
        ground_colliders=ground_colliders,
        wheel_bindings=wheel_bindings,
        ground_bindings=ground_bindings,
        wheel_material=wheel_material,
        ground_material=ground_material,
        stage_usd_readback_verified=True,
    )


def apply_contact_profile(
    stage: object,
    config: ProjectConfig,
) -> ContactProfileSnapshot:
    """Apply the selected profile without changing any persistent USD layer."""

    profile = load_contact_profile(config.files.contact_profile)
    wheel_colliders = resolve_wheel_colliders(stage, config)
    ground_colliders = resolve_ground_colliders(stage, config)
    layer = _create_contact_layer(stage, profile)
    original_target = stage.GetEditTarget()
    try:
        stage.SetEditTarget(layer)
        if profile.scene is not None:
            _author_scene(stage, config, profile.scene)
        if profile.mode == "explicit_material":
            assert profile.wheel_material is not None
            assert profile.ground_material is not None
            wheel_material = _author_material(
                stage,
                _WHEEL_MATERIAL_PATH,
                profile.wheel_material,
            )
            ground_material = _author_material(
                stage,
                _GROUND_MATERIAL_PATH,
                profile.ground_material,
            )
            _bind_material(stage, wheel_colliders, wheel_material)
            _bind_material(stage, ground_colliders, ground_material)
    except Exception:
        stage.SetEditTarget(original_target)
        _remove_existing_contact_layers(stage)
        raise
    finally:
        if stage.GetEditTarget() != original_target:
            stage.SetEditTarget(original_target)
    try:
        return capture_contact_profile_snapshot(stage, config)
    except Exception:
        _remove_existing_contact_layers(stage)
        raise


__all__ = [
    "ContactProfile",
    "ContactProfileSnapshot",
    "ContactSetupError",
    "apply_contact_profile",
    "capture_contact_profile_snapshot",
    "load_contact_profile",
    "resolve_ground_colliders",
    "resolve_wheel_colliders",
]
