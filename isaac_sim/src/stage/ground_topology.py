"""Strict, reversible ground-collider topology profiles.

The selected environment asset remains read-only.  A versioned profile first
locks the source asset bytes and the exact source/target collider path sets,
then authors only ``physics:collisionEnabled = false`` opinions for colliders
excluded from the target set in an anonymous Stage session layer.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import re
from typing import Any, Iterable, Mapping

from isaac_sim.src.config import ProjectConfig
from isaac_sim.src.yaml_utils import YamlConfigError, load_mapping


class GroundTopologyError(RuntimeError):
    """Raised when a topology profile or its effective USD state is invalid."""


_PROFILE_LAYER_MARKER = "isaac_nav_ground_topology_layer"
_OPERATIONS = {
    "preserve_source_colliders",
    "disable_non_target_colliders",
}
_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
_PRIM_PATH_PATTERN = re.compile(
    r"^/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*$"
)
_SHA256_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_SEMANTIC_CLASS_ATTRIBUTE = "semantic:Semantics:params:semanticData"
_SEMANTIC_TYPE_ATTRIBUTE = "semantic:Semantics:params:semanticType"
_COLLISION_ENABLED_ATTRIBUTE = "physics:collisionEnabled"


@dataclass(frozen=True)
class ColliderSetSpec:
    required_prim_paths: tuple[str, ...]
    semantic_classes: tuple[str, ...]
    collider_count: int
    collider_paths_sha256: str


@dataclass(frozen=True)
class DisabledColliderSetSpec:
    collider_count: int
    collider_paths_sha256: str


@dataclass(frozen=True)
class GroundTopologyProfile:
    path: Path
    sha256: str
    identifier: str
    environment_id: str
    operation: str
    source_asset_sha256: str
    source: ColliderSetSpec
    target: ColliderSetSpec
    disabled: DisabledColliderSetSpec


@dataclass(frozen=True)
class GroundTopologySnapshot:
    """JSON-serializable identity and effective-Stage topology evidence."""

    profile_path: str
    profile_sha256: str
    profile_id: str
    environment_id: str
    operation: str
    source_asset_path: str
    source_asset_sha256: str
    overlay_identifier: str
    overlay_sha256: str
    source_colliders: tuple[str, ...]
    source_collider_count: int
    source_collider_paths_sha256: str
    target_colliders: tuple[str, ...]
    target_collider_count: int
    target_collider_paths_sha256: str
    disabled_colliders: tuple[str, ...]
    disabled_collider_count: int
    disabled_collider_paths_sha256: str
    stage_usd_readback_verified: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def collider_paths_sha256(paths: Iterable[str]) -> str:
    """Hash a collider path set using the frozen canonical JSON encoding."""

    values = tuple(paths)
    if not all(isinstance(path, str) for path in values):
        raise GroundTopologyError("collider paths must all be strings")
    if len(set(values)) != len(values):
        raise GroundTopologyError("collider path sets must not contain duplicates")
    canonical = json.dumps(
        sorted(values),
        separators=(",", ":"),
        allow_nan=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _mapping(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise GroundTopologyError(f"{name} must be a mapping")
    return dict(value)


def _keys(value: Mapping[str, Any], allowed: set[str], name: str) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise GroundTopologyError(f"unknown {name} keys: {unknown}")
    missing = sorted(allowed - set(value))
    if missing:
        raise GroundTopologyError(f"missing {name} keys: {missing}")


def _identifier(value: Any, name: str) -> str:
    if not isinstance(value, str) or not _IDENTIFIER_PATTERN.fullmatch(value):
        raise GroundTopologyError(f"{name} must be a path-safe identifier")
    return value


def _sha256(value: Any, name: str) -> str:
    if not isinstance(value, str) or not _SHA256_PATTERN.fullmatch(value):
        raise GroundTopologyError(f"{name} must be a lowercase SHA256")
    return value


def _count(value: Any, name: str, *, allow_zero: bool) -> int:
    minimum = 0 if allow_zero else 1
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        qualifier = "non-negative" if allow_zero else "positive"
        raise GroundTopologyError(f"{name} must be a {qualifier} integer")
    return value


def _path_tuple(value: Any, name: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise GroundTopologyError(f"{name} must be a list")
    paths = tuple(value)
    if not all(
        isinstance(path, str) and _PRIM_PATH_PATTERN.fullmatch(path)
        for path in paths
    ):
        raise GroundTopologyError(
            f"{name} must contain valid absolute USD prim paths"
        )
    if len(set(paths)) != len(paths):
        raise GroundTopologyError(f"{name} must not contain duplicates")
    return paths


def _semantic_tuple(value: Any, name: str) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise GroundTopologyError(f"{name} must be a list")
    classes = tuple(value)
    if not all(
        isinstance(item, str) and _IDENTIFIER_PATTERN.fullmatch(item)
        for item in classes
    ):
        raise GroundTopologyError(
            f"{name} must contain path-safe semantic identifiers"
        )
    if len(set(classes)) != len(classes):
        raise GroundTopologyError(f"{name} must not contain duplicates")
    return classes


def _parse_collider_set(value: Any, name: str) -> ColliderSetSpec:
    data = _mapping(value, name)
    allowed = {
        "required_prim_paths",
        "semantic_classes",
        "collider_count",
        "collider_paths_sha256",
    }
    _keys(data, allowed, name)
    required_paths = _path_tuple(
        data["required_prim_paths"],
        f"{name}.required_prim_paths",
    )
    semantic_classes = _semantic_tuple(
        data["semantic_classes"],
        f"{name}.semantic_classes",
    )
    if not required_paths and not semantic_classes:
        raise GroundTopologyError(
            f"{name} must select at least one required path or semantic class"
        )
    count = _count(data["collider_count"], f"{name}.collider_count", allow_zero=False)
    if count < len(required_paths):
        raise GroundTopologyError(
            f"{name}.collider_count must cover every required prim path"
        )
    return ColliderSetSpec(
        required_prim_paths=required_paths,
        semantic_classes=semantic_classes,
        collider_count=count,
        collider_paths_sha256=_sha256(
            data["collider_paths_sha256"],
            f"{name}.collider_paths_sha256",
        ),
    )


def load_ground_topology_profile(path: str | Path) -> GroundTopologyProfile:
    """Load a versioned profile with exact keys and mode-dependent invariants."""

    source_path = Path(path).expanduser().resolve()
    if not source_path.is_file():
        raise GroundTopologyError(
            f"ground topology profile is not a file: {source_path}"
        )
    try:
        data = load_mapping(source_path)
    except (OSError, YamlConfigError) as exc:
        raise GroundTopologyError(
            f"invalid ground topology profile {source_path}: {exc}"
        ) from exc
    allowed = {
        "schema_version",
        "id",
        "environment_id",
        "operation",
        "source",
        "target",
        "disabled",
    }
    _keys(data, allowed, "ground_topology_profile")
    version = data["schema_version"]
    if isinstance(version, bool) or not isinstance(version, int) or version != 1:
        raise GroundTopologyError(
            "ground_topology_profile.schema_version must be 1"
        )
    identifier = _identifier(data["id"], "ground_topology_profile.id")
    environment_id = _identifier(
        data["environment_id"],
        "ground_topology_profile.environment_id",
    )
    operation = data["operation"]
    if not isinstance(operation, str) or operation not in _OPERATIONS:
        raise GroundTopologyError(
            "ground_topology_profile.operation must be one of "
            f"{sorted(_OPERATIONS)}"
        )

    source_data = _mapping(data["source"], "ground_topology_profile.source")
    _keys(
        source_data,
        {
            "asset_sha256",
            "required_prim_paths",
            "semantic_classes",
            "collider_count",
            "collider_paths_sha256",
        },
        "ground_topology_profile.source",
    )
    source_asset_sha256 = _sha256(
        source_data.pop("asset_sha256"),
        "ground_topology_profile.source.asset_sha256",
    )
    source_spec = _parse_collider_set(
        source_data,
        "ground_topology_profile.source",
    )
    target_spec = _parse_collider_set(
        data["target"],
        "ground_topology_profile.target",
    )
    disabled_data = _mapping(
        data["disabled"],
        "ground_topology_profile.disabled",
    )
    _keys(
        disabled_data,
        {"collider_count", "collider_paths_sha256"},
        "ground_topology_profile.disabled",
    )
    disabled_spec = DisabledColliderSetSpec(
        collider_count=_count(
            disabled_data["collider_count"],
            "ground_topology_profile.disabled.collider_count",
            allow_zero=True,
        ),
        collider_paths_sha256=_sha256(
            disabled_data["collider_paths_sha256"],
            "ground_topology_profile.disabled.collider_paths_sha256",
        ),
    )

    if not set(target_spec.required_prim_paths).issubset(
        source_spec.required_prim_paths
    ) or not set(target_spec.semantic_classes).issubset(
        source_spec.semantic_classes
    ):
        raise GroundTopologyError(
            "ground_topology_profile.target selectors must be subsets of source"
        )
    if (
        target_spec.collider_count + disabled_spec.collider_count
        != source_spec.collider_count
    ):
        raise GroundTopologyError(
            "ground topology source count must equal target plus disabled counts"
        )
    empty_sha256 = collider_paths_sha256(())
    if operation == "preserve_source_colliders":
        if source_spec != target_spec:
            raise GroundTopologyError(
                "preserve_source_colliders requires identical source and target"
            )
        if (
            disabled_spec.collider_count != 0
            or disabled_spec.collider_paths_sha256 != empty_sha256
        ):
            raise GroundTopologyError(
                "preserve_source_colliders requires the canonical empty disabled set"
            )
    elif disabled_spec.collider_count == 0:
        raise GroundTopologyError(
            "disable_non_target_colliders requires at least one disabled collider"
        )

    return GroundTopologyProfile(
        path=source_path,
        sha256=_file_sha256(source_path),
        identifier=identifier,
        environment_id=environment_id,
        operation=operation,
        source_asset_sha256=source_asset_sha256,
        source=source_spec,
        target=target_spec,
        disabled=disabled_spec,
    )


def _collision_is_enabled(prim: object) -> bool:
    from pxr import UsdPhysics

    if not prim or not prim.IsValid() or not prim.IsActive():
        return False
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        return False
    return UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True


def _resolve_collider_set(
    stage: object,
    spec: ColliderSetSpec,
    name: str,
) -> tuple[str, ...]:
    paths: set[str] = set()
    for path in spec.required_prim_paths:
        prim = stage.GetPrimAtPath(path)
        if not _collision_is_enabled(prim):
            raise GroundTopologyError(
                f"{name} required collider is missing or disabled: {path}"
            )
        paths.add(path)

    semantic_classes = set(spec.semantic_classes)
    matched_classes: set[str] = set()
    for prim in stage.TraverseAll():
        if not semantic_classes or not _collision_is_enabled(prim):
            continue
        class_attribute = prim.GetAttribute(_SEMANTIC_CLASS_ATTRIBUTE)
        type_attribute = prim.GetAttribute(_SEMANTIC_TYPE_ATTRIBUTE)
        semantic_class = (
            class_attribute.Get() if class_attribute.IsValid() else None
        )
        semantic_type = type_attribute.Get() if type_attribute.IsValid() else None
        if semantic_type == "class" and semantic_class in semantic_classes:
            paths.add(str(prim.GetPath()))
            matched_classes.add(str(semantic_class))
    if matched_classes != semantic_classes:
        raise GroundTopologyError(
            f"{name} semantic class mismatch: configured="
            f"{sorted(semantic_classes)}, matched={sorted(matched_classes)}"
        )

    result = tuple(sorted(paths))
    actual_hash = collider_paths_sha256(result)
    if len(result) != spec.collider_count or actual_hash != spec.collider_paths_sha256:
        raise GroundTopologyError(
            f"{name} collider set mismatch: expected_count={spec.collider_count}, "
            f"actual_count={len(result)}, expected_sha256="
            f"{spec.collider_paths_sha256}, actual_sha256={actual_hash}, "
            f"paths={list(result)}"
        )
    return result


def _validate_project_source_contract(
    config: ProjectConfig,
    profile: GroundTopologyProfile,
) -> None:
    if config.environment.identifier != profile.environment_id:
        raise GroundTopologyError(
            "ground topology environment mismatch: "
            f"project={config.environment.identifier}, "
            f"profile={profile.environment_id}"
        )
    resolver = config.environment.ground_colliders
    expected = (
        profile.source.required_prim_paths,
        profile.source.semantic_classes,
        profile.source.collider_count,
    )
    actual = (
        resolver.required_prim_paths,
        resolver.semantic_classes,
        resolver.expected_enabled_count,
    )
    if actual != expected:
        raise GroundTopologyError(
            "project ground source resolver mismatch: "
            f"expected={expected}, actual={actual}"
        )


def _load_source_sets(
    config: ProjectConfig,
    profile: GroundTopologyProfile,
) -> tuple[tuple[str, ...], tuple[str, ...], tuple[str, ...]]:
    from pxr import Usd

    _validate_project_source_contract(config, profile)
    source_asset = config.environment.source_asset
    if not source_asset.is_file():
        raise GroundTopologyError(
            f"ground topology source asset is not a file: {source_asset}"
        )
    actual_asset_sha256 = _file_sha256(source_asset)
    if actual_asset_sha256 != profile.source_asset_sha256:
        raise GroundTopologyError(
            "ground topology source asset SHA256 mismatch: "
            f"expected={profile.source_asset_sha256}, "
            f"actual={actual_asset_sha256}, path={source_asset}"
        )
    source_stage = Usd.Stage.Open(str(source_asset))
    if source_stage is None:
        raise GroundTopologyError(
            f"failed to open ground topology source asset: {source_asset}"
        )
    source_colliders = _resolve_collider_set(
        source_stage,
        profile.source,
        "ground_topology_profile.source",
    )
    target_colliders = _resolve_collider_set(
        source_stage,
        profile.target,
        "ground_topology_profile.target",
    )
    if not set(target_colliders).issubset(source_colliders):
        raise GroundTopologyError(
            "ground topology target colliders are not a subset of source"
        )
    disabled_colliders = tuple(
        sorted(set(source_colliders) - set(target_colliders))
    )
    disabled_hash = collider_paths_sha256(disabled_colliders)
    if (
        len(disabled_colliders) != profile.disabled.collider_count
        or disabled_hash != profile.disabled.collider_paths_sha256
    ):
        raise GroundTopologyError(
            "ground topology disabled collider set mismatch: "
            f"expected_count={profile.disabled.collider_count}, "
            f"actual_count={len(disabled_colliders)}, expected_sha256="
            f"{profile.disabled.collider_paths_sha256}, "
            f"actual_sha256={disabled_hash}, paths={list(disabled_colliders)}"
        )
    return source_colliders, target_colliders, disabled_colliders


def _remove_existing_topology_layers(stage: object) -> None:
    from pxr import Sdf

    session = stage.GetSessionLayer()
    for identifier in list(session.subLayerPaths):
        layer = Sdf.Layer.Find(identifier)
        if layer and layer.customLayerData.get(_PROFILE_LAYER_MARKER) is True:
            session.subLayerPaths.remove(identifier)


def _layer_metadata(
    profile: GroundTopologyProfile,
) -> dict[str, object]:
    return {
        _PROFILE_LAYER_MARKER: True,
        "ground_topology_profile_id": profile.identifier,
        "ground_topology_profile_sha256": profile.sha256,
        "ground_topology_environment_id": profile.environment_id,
        "ground_topology_operation": profile.operation,
        "ground_topology_source_asset_sha256": profile.source_asset_sha256,
        "ground_topology_source_collider_count": profile.source.collider_count,
        "ground_topology_source_collider_paths_sha256": (
            profile.source.collider_paths_sha256
        ),
        "ground_topology_target_collider_count": profile.target.collider_count,
        "ground_topology_target_collider_paths_sha256": (
            profile.target.collider_paths_sha256
        ),
        "ground_topology_disabled_collider_count": (
            profile.disabled.collider_count
        ),
        "ground_topology_disabled_collider_paths_sha256": (
            profile.disabled.collider_paths_sha256
        ),
    }


def _create_topology_layer(stage: object, profile: GroundTopologyProfile):
    from pxr import Sdf

    layer = Sdf.Layer.CreateAnonymous(f"ground_topology_{profile.identifier}.usda")
    layer.customLayerData = _layer_metadata(profile)
    stage.GetSessionLayer().subLayerPaths.insert(0, layer.identifier)
    return layer


def _find_topology_layer(stage: object, profile: GroundTopologyProfile):
    from pxr import Sdf

    matches = []
    for identifier in stage.GetSessionLayer().subLayerPaths:
        layer = Sdf.Layer.Find(identifier)
        if layer and layer.customLayerData.get(_PROFILE_LAYER_MARKER) is True:
            matches.append(layer)
    if len(matches) != 1:
        raise GroundTopologyError(
            f"expected one active ground topology session layer, found {len(matches)}"
        )
    layer = matches[0]
    expected = _layer_metadata(profile)
    actual = dict(layer.customLayerData)
    if actual != expected:
        raise GroundTopologyError(
            "ground topology session layer metadata mismatch: "
            f"expected={expected}, actual={actual}"
        )
    return layer


def _author_disabled_colliders(
    stage: object,
    disabled_colliders: tuple[str, ...],
) -> None:
    from pxr import UsdPhysics

    for path in disabled_colliders:
        prim = stage.GetPrimAtPath(path)
        if not _collision_is_enabled(prim):
            raise GroundTopologyError(
                f"cannot disable invalid source ground collider: {path}"
            )
        attribute = UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr()
        if not attribute.Set(False):
            raise GroundTopologyError(
                f"failed to disable source ground collider: {path}"
            )


def _validate_layer_authorship(
    layer: object,
    disabled_colliders: tuple[str, ...],
) -> None:
    from pxr import Sdf

    expected_attributes = {
        str(Sdf.Path(path).AppendProperty(_COLLISION_ENABLED_ATTRIBUTE))
        for path in disabled_colliders
    }
    expected_prim_paths = {
        "/" + "/".join(path.strip("/").split("/")[:depth])
        for path in disabled_colliders
        for depth in range(1, len(path.strip("/").split("/")) + 1)
    }
    actual_attributes: set[str] = set()
    actual_prim_paths: set[str] = set()
    invalid_specs: list[str] = []

    def visit(path: object) -> None:
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.PseudoRootSpec):
            return
        if isinstance(spec, Sdf.PrimSpec):
            prim_path = str(path)
            actual_prim_paths.add(prim_path)
            unexpected_metadata = set(spec.ListInfoKeys()) - {"specifier"}
            if unexpected_metadata or spec.specifier != Sdf.SpecifierOver:
                invalid_specs.append(str(path))
            return
        if isinstance(spec, Sdf.AttributeSpec):
            property_path = str(path)
            actual_attributes.add(property_path)
            if (
                property_path not in expected_attributes
                or spec.name != _COLLISION_ENABLED_ATTRIBUTE
                or spec.typeName != Sdf.ValueTypeNames.Bool
                or spec.default is not False
            ):
                invalid_specs.append(property_path)
            return
        invalid_specs.append(str(path))

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    if (
        invalid_specs
        or actual_attributes != expected_attributes
        or actual_prim_paths != expected_prim_paths
    ):
        raise GroundTopologyError(
            "ground topology layer authored opinions outside the exact "
            "physics:collisionEnabled=false contract: "
            f"invalid={sorted(invalid_specs)}, expected_attributes="
            f"{sorted(expected_attributes)}, actual_attributes="
            f"{sorted(actual_attributes)}, expected_prims="
            f"{sorted(expected_prim_paths)}, actual_prims="
            f"{sorted(actual_prim_paths)}"
        )


def capture_ground_topology_snapshot(
    stage: object,
    config: ProjectConfig,
) -> GroundTopologySnapshot:
    """Re-read and verify the active topology layer and effective colliders."""

    from pxr import UsdPhysics

    profile = load_ground_topology_profile(config.files.ground_topology_profile)
    source_colliders, target_colliders, disabled_colliders = _load_source_sets(
        config,
        profile,
    )
    layer = _find_topology_layer(stage, profile)
    _validate_layer_authorship(layer, disabled_colliders)

    target_set = set(target_colliders)
    for path in source_colliders:
        prim = stage.GetPrimAtPath(path)
        if (
            not prim
            or not prim.IsValid()
            or not prim.IsActive()
            or not prim.HasAPI(UsdPhysics.CollisionAPI)
        ):
            raise GroundTopologyError(
                f"effective ground collider is missing, inactive, or invalid: {path}"
            )
        collision_enabled = (
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        )
        expected_enabled = path in target_set
        if collision_enabled is not expected_enabled:
            raise GroundTopologyError(
                "effective ground collisionEnabled readback mismatch: "
                f"path={path}, expected={expected_enabled}, "
                f"actual={collision_enabled!r}"
            )

    overlay_source = layer.ExportToString()
    return GroundTopologySnapshot(
        profile_path=str(profile.path),
        profile_sha256=profile.sha256,
        profile_id=profile.identifier,
        environment_id=profile.environment_id,
        operation=profile.operation,
        source_asset_path=str(config.environment.source_asset),
        source_asset_sha256=profile.source_asset_sha256,
        overlay_identifier=layer.identifier,
        overlay_sha256=hashlib.sha256(
            overlay_source.encode("utf-8")
        ).hexdigest(),
        source_colliders=source_colliders,
        source_collider_count=len(source_colliders),
        source_collider_paths_sha256=collider_paths_sha256(source_colliders),
        target_colliders=target_colliders,
        target_collider_count=len(target_colliders),
        target_collider_paths_sha256=collider_paths_sha256(target_colliders),
        disabled_colliders=disabled_colliders,
        disabled_collider_count=len(disabled_colliders),
        disabled_collider_paths_sha256=collider_paths_sha256(
            disabled_colliders
        ),
        stage_usd_readback_verified=True,
    )


def apply_ground_topology(
    stage: object,
    config: ProjectConfig,
) -> GroundTopologySnapshot:
    """Apply one profile without changing any persistent environment layer."""

    original_target = stage.GetEditTarget()
    _remove_existing_topology_layers(stage)
    try:
        profile = load_ground_topology_profile(
            config.files.ground_topology_profile
        )
        _source, _target, disabled_colliders = _load_source_sets(config, profile)
        layer = _create_topology_layer(stage, profile)
        stage.SetEditTarget(layer)
        _author_disabled_colliders(stage, disabled_colliders)
        stage.SetEditTarget(original_target)
        return capture_ground_topology_snapshot(stage, config)
    except Exception:
        if stage.GetEditTarget() != original_target:
            stage.SetEditTarget(original_target)
        _remove_existing_topology_layers(stage)
        raise
    finally:
        if stage.GetEditTarget() != original_target:
            stage.SetEditTarget(original_target)


__all__ = [
    "ColliderSetSpec",
    "DisabledColliderSetSpec",
    "GroundTopologyError",
    "GroundTopologyProfile",
    "GroundTopologySnapshot",
    "apply_ground_topology",
    "capture_ground_topology_snapshot",
    "collider_paths_sha256",
    "load_ground_topology_profile",
]
