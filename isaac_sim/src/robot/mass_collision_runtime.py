"""Pre-physics mass/collision authoring and fresh USD/PhysX readback.

The persistent Jackal asset remains immutable.  A selected schema-v1 profile
is materialized as exactly one anonymous session sublayer, verified from the
composed Stage, and later cross-checked against Isaac's physics tensor API.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re
from typing import Any, Iterable, Sequence

from isaac_sim.src.robot.kinematics_config import load_robot_config_contract
from isaac_sim.src.robot.mass_collision_config import (
    BaseInertial,
    MassCollisionProfile,
    load_mass_collision_profile,
    resolve_prim_suffix,
)


_LAYER_MARKER = "isaac_nav_mass_collision_profile_layer"
_COLLISION_ATTRIBUTE = "physics:collisionEnabled"
_MASS_ATTRIBUTE = "physics:mass"
_CENTER_OF_MASS_ATTRIBUTE = "physics:centerOfMass"
_DIAGONAL_INERTIA_ATTRIBUTE = "physics:diagonalInertia"
_PRINCIPAL_AXES_ATTRIBUTE = "physics:principalAxes"
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_STAGE_ABS_TOL = 1e-7
_TENSOR_ABS_TOL = 1e-5


class MassCollisionRuntimeError(RuntimeError):
    """Raised when USD authoring or runtime tensor evidence is inconsistent."""


@dataclass(frozen=True)
class ProfileEvidence:
    path: str
    sha256: str
    id: str
    mode: str

    def to_dict(self) -> dict[str, object]:
        return {
            "path": self.path,
            "sha256": self.sha256,
            "id": self.id,
            "mode": self.mode,
        }


@dataclass(frozen=True)
class OverlayEvidence:
    id: str
    identifier: str
    sha256: str

    def to_dict(self) -> dict[str, object]:
        return {
            "id": self.id,
            "identifier": self.identifier,
            "sha256": self.sha256,
        }


@dataclass(frozen=True)
class ShellStageSnapshot:
    prim_path: str
    active: bool
    collision_enabled: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "prim_path": self.prim_path,
            "active": self.active,
            "collision_enabled": self.collision_enabled,
        }


@dataclass(frozen=True)
class BaseInertialStageSnapshot:
    prim_path: str
    mass_kg: float
    center_of_mass_m: tuple[float, float, float]
    inertia_kg_m2: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]

    def to_dict(self) -> dict[str, object]:
        return {
            "prim_path": self.prim_path,
            "mass_kg": self.mass_kg,
            "center_of_mass_m": list(self.center_of_mass_m),
            "inertia_kg_m2": [list(row) for row in self.inertia_kg_m2],
        }


@dataclass(frozen=True)
class LinkMassExpectationSnapshot:
    prim_path: str
    mass_kg: float

    def to_dict(self) -> dict[str, object]:
        return {"prim_path": self.prim_path, "mass_kg": self.mass_kg}


@dataclass(frozen=True)
class MassCollisionStageSnapshot:
    schema_version: int
    profile: ProfileEvidence
    robot_asset_sha256: str
    sensor_shells: tuple[ShellStageSnapshot, ShellStageSnapshot]
    base_inertial: BaseInertialStageSnapshot | None
    expected_link_masses: tuple[
        LinkMassExpectationSnapshot,
        LinkMassExpectationSnapshot,
        LinkMassExpectationSnapshot,
        LinkMassExpectationSnapshot,
        LinkMassExpectationSnapshot,
    ]
    expected_total_mass_kg: float
    overlay: OverlayEvidence
    stage_usd_readback_verified: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "profile": self.profile.to_dict(),
            "robot_asset_sha256": self.robot_asset_sha256,
            "sensor_shells": [shell.to_dict() for shell in self.sensor_shells],
            "base_inertial": (
                self.base_inertial.to_dict()
                if self.base_inertial is not None
                else None
            ),
            "expected_link_masses": [
                expectation.to_dict()
                for expectation in self.expected_link_masses
            ],
            "expected_total_mass_kg": self.expected_total_mass_kg,
            "overlay": self.overlay.to_dict(),
            "stage_usd_readback_verified": self.stage_usd_readback_verified,
        }


@dataclass(frozen=True)
class TensorLinkSnapshot:
    name: str
    prim_path: str
    mass_kg: float
    center_of_mass_m: tuple[float, float, float]
    inertia_kg_m2: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]

    def to_dict(self) -> dict[str, object]:
        return {
            "name": self.name,
            "prim_path": self.prim_path,
            "mass_kg": self.mass_kg,
            "center_of_mass_m": list(self.center_of_mass_m),
            "inertia_kg_m2": [list(row) for row in self.inertia_kg_m2],
        }


@dataclass(frozen=True)
class MassTensorSnapshot:
    schema_version: int
    profile_id: str
    links: tuple[
        TensorLinkSnapshot,
        TensorLinkSnapshot,
        TensorLinkSnapshot,
        TensorLinkSnapshot,
        TensorLinkSnapshot,
    ]
    total_mass_kg: float
    physics_tensor_readback_verified: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "profile_id": self.profile_id,
            "links": [link.to_dict() for link in self.links],
            "total_mass_kg": self.total_mass_kg,
            "physics_tensor_readback_verified": (
                self.physics_tensor_readback_verified
            ),
        }


@dataclass(frozen=True)
class _ProfileContext:
    profile_path: Path
    profile: MassCollisionProfile
    profile_evidence: ProfileEvidence
    asset_sha256: str


def apply_mass_collision_profile(
    stage: object,
    config: object,
) -> MassCollisionStageSnapshot:
    """Author the selected profile in one anonymous pre-physics session layer."""

    context = _load_profile_context(config)
    original_target = stage.GetEditTarget()
    _remove_existing_layers(stage)
    try:
        layer = _create_layer(stage, context)
        stage.SetEditTarget(layer)
        _author_profile(stage, config, context.profile)
        stage.SetEditTarget(original_target)
        return capture_mass_collision_snapshot(stage, config)
    except Exception:
        if stage.GetEditTarget() != original_target:
            stage.SetEditTarget(original_target)
        _remove_existing_layers(stage)
        raise
    finally:
        if stage.GetEditTarget() != original_target:
            stage.SetEditTarget(original_target)


def capture_mass_collision_snapshot(
    stage: object,
    config: object,
) -> MassCollisionStageSnapshot:
    """Freshly verify the exact layer and composed Stage state."""

    context = _load_profile_context(config)
    profile = context.profile
    layer = _find_layer(stage, context)
    _validate_layer_authorship(layer, config, profile)

    shells = tuple(
        sorted(
            (
                _read_shell(
                    stage,
                    layer,
                    resolve_prim_suffix(
                        config.robot.articulation_root,
                        shell.prim_suffix,
                    ),
                    shell.active,
                    shell.collision_enabled,
                )
                for shell in profile.sensor_shells
            ),
            key=lambda item: item.prim_path,
        )
    )
    base_inertial = (
        _read_base_inertial(stage, config, profile.base_inertial)
        if profile.base_inertial is not None
        else None
    )
    expected_link_masses = tuple(
        sorted(
            (
                LinkMassExpectationSnapshot(
                    prim_path=resolve_prim_suffix(
                        config.robot.articulation_root,
                        expectation.prim_suffix,
                    ),
                    mass_kg=expectation.mass_kg,
                )
                for expectation in profile.expected_link_masses
            ),
            key=lambda item: item.prim_path,
        )
    )
    overlay_source = layer.ExportToString()
    return MassCollisionStageSnapshot(
        schema_version=1,
        profile=context.profile_evidence,
        robot_asset_sha256=context.asset_sha256,
        sensor_shells=shells,  # type: ignore[arg-type]
        base_inertial=base_inertial,
        expected_link_masses=expected_link_masses,  # type: ignore[arg-type]
        expected_total_mass_kg=profile.expected_total_mass_kg,
        overlay=OverlayEvidence(
            id=f"mass_collision_profile/{profile.profile_id}",
            identifier=layer.identifier,
            sha256=hashlib.sha256(
                overlay_source.encode("utf-8")
            ).hexdigest(),
        ),
        stage_usd_readback_verified=True,
    )


def capture_mass_tensor_snapshot(
    articulation_or_wrapper: object,
    config: object,
    stage_snapshot: MassCollisionStageSnapshot,
) -> MassTensorSnapshot:
    """Verify five-link mass/COM/inertia evidence from Isaac's tensor API."""

    context = _load_profile_context(config)
    _validate_stage_snapshot_binding(stage_snapshot, config, context)
    articulation = _unwrap_articulation(articulation_or_wrapper)

    validity = getattr(articulation, "is_physics_tensor_entity_valid", None)
    if not callable(validity) or validity() is not True:
        raise MassCollisionRuntimeError(
            "articulation physics tensor entity is not valid"
        )
    names = getattr(articulation, "link_names", None)
    if (
        not isinstance(names, (list, tuple))
        or not all(isinstance(name, str) and name for name in names)
        or len(set(names)) != len(names)
    ):
        raise MassCollisionRuntimeError(
            "articulation link_names must be five unique nonempty strings"
        )
    names = list(names)
    paths = _one_articulation_link_paths(
        getattr(articulation, "link_paths", None)
    )
    expected_by_path = {
        item.prim_path: item.mass_kg
        for item in stage_snapshot.expected_link_masses
    }
    if len(paths) != 5 or set(paths) != set(expected_by_path):
        raise MassCollisionRuntimeError(
            "articulation link_paths must contain exactly the five profile "
            "target links"
        )
    if len(names) != len(paths):
        raise MassCollisionRuntimeError(
            "articulation link name/path order mismatch: row lengths differ"
        )
    for index, (name, path) in enumerate(zip(names, paths)):
        if Path(path).name != name:
            raise MassCollisionRuntimeError(
                "articulation link name/path order mismatch at index "
                f"{index}: name={name!r}, path={path!r}"
            )

    ordered_target_paths = sorted(expected_by_path)
    ordered_target_names = [Path(path).name for path in ordered_target_paths]
    expected_indices = [paths.index(path) for path in ordered_target_paths]
    index_reader = getattr(articulation, "get_link_indices", None)
    if not callable(index_reader):
        raise MassCollisionRuntimeError(
            "articulation must provide get_link_indices"
        )
    actual_indices = _tensor_to_python(
        index_reader(ordered_target_names),
        context="get_link_indices",
    )
    if actual_indices != expected_indices:
        raise MassCollisionRuntimeError(
            "articulation get_link_indices order mismatch: "
            f"expected={expected_indices}, actual={actual_indices}"
        )

    masses = _read_tensor_matrix(
        articulation,
        "get_link_masses",
        expected_shape=(1, 5),
    )[0]
    com_reader = getattr(articulation, "get_link_coms", None)
    if not callable(com_reader):
        raise MassCollisionRuntimeError("articulation must provide get_link_coms")
    com_result = com_reader()
    if not isinstance(com_result, tuple) or len(com_result) != 2:
        raise MassCollisionRuntimeError(
            "get_link_coms must return position and orientation tensors"
        )
    positions = _validated_nested_numeric(
        _tensor_to_python(com_result[0], context="get_link_coms positions"),
        expected_shape=(1, 5, 3),
        context="get_link_coms position shape",
    )[0]
    orientations = _validated_nested_numeric(
        _tensor_to_python(
            com_result[1], context="get_link_coms orientations"
        ),
        expected_shape=(1, 5, 4),
        context="get_link_coms orientation shape",
    )[0]
    inertias = _read_tensor_matrix(
        articulation,
        "get_link_inertias",
        expected_shape=(1, 5, 9),
    )[0]

    links_by_path: dict[str, TensorLinkSnapshot] = {}
    for index, path in enumerate(paths):
        mass_kg = _finite_number(
            masses[index], context=f"link mass {path}", positive=True
        )
        expected_mass_kg = expected_by_path[path]
        if not math.isclose(
            mass_kg,
            expected_mass_kg,
            rel_tol=1e-6,
            abs_tol=_TENSOR_ABS_TOL,
        ):
            raise MassCollisionRuntimeError(
                "physics tensor mass readback mismatch: "
                f"path={path}, expected={expected_mass_kg}, actual={mass_kg}"
            )
        center_of_mass_m = tuple(
            _finite_number(value, context=f"link COM {path}")
            for value in positions[index]
        )
        orientation_wxyz = tuple(
            _finite_number(value, context=f"link COM orientation {path}")
            for value in orientations[index]
        )
        quaternion_norm = math.sqrt(
            sum(value * value for value in orientation_wxyz)
        )
        if not math.isclose(
            quaternion_norm, 1.0, rel_tol=1e-5, abs_tol=1e-5
        ):
            raise MassCollisionRuntimeError(
                f"link COM orientation must be a unit quaternion: {path}"
            )
        principal_inertia = _matrix_from_flat(
            inertias[index], context=f"link inertia {path}"
        )
        _require_symmetric_positive_definite(
            principal_inertia, context=f"link inertia {path}"
        )
        full_inertia = _rotate_inertia(
            principal_inertia,
            _quaternion_to_matrix(orientation_wxyz),
        )
        links_by_path[path] = TensorLinkSnapshot(
            name=names[index],
            prim_path=path,
            mass_kg=mass_kg,
            center_of_mass_m=center_of_mass_m,  # type: ignore[arg-type]
            inertia_kg_m2=full_inertia,
        )

    total_mass_kg = sum(link.mass_kg for link in links_by_path.values())
    if not math.isclose(
        total_mass_kg,
        stage_snapshot.expected_total_mass_kg,
        rel_tol=1e-6,
        abs_tol=5 * _TENSOR_ABS_TOL,
    ):
        raise MassCollisionRuntimeError(
            "physics tensor total mass readback mismatch: "
            f"expected={stage_snapshot.expected_total_mass_kg}, "
            f"actual={total_mass_kg}"
        )

    if stage_snapshot.base_inertial is not None:
        expected_base = stage_snapshot.base_inertial
        actual_base = links_by_path[expected_base.prim_path]
        if not _vector_close(
            actual_base.center_of_mass_m,
            expected_base.center_of_mass_m,
            abs_tol=_TENSOR_ABS_TOL,
        ):
            raise MassCollisionRuntimeError(
                "physics tensor fixed base COM readback mismatch"
            )
        if not _matrix_close(
            actual_base.inertia_kg_m2,
            expected_base.inertia_kg_m2,
            abs_tol=2 * _TENSOR_ABS_TOL,
        ):
            raise MassCollisionRuntimeError(
                "physics tensor fixed base inertia readback mismatch"
            )

    links = tuple(
        links_by_path[path] for path in sorted(links_by_path)
    )
    return MassTensorSnapshot(
        schema_version=1,
        profile_id=context.profile.profile_id,
        links=links,  # type: ignore[arg-type]
        total_mass_kg=total_mass_kg,
        physics_tensor_readback_verified=True,
    )


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _repository_relative_path(path: Path) -> str:
    source = path.resolve()
    for ancestor in source.parents:
        if (
            (ancestor / "pyproject.toml").is_file()
            and (ancestor / "isaac_sim").is_dir()
        ):
            try:
                return source.relative_to(ancestor).as_posix()
            except ValueError:
                continue
    raise MassCollisionRuntimeError(
        f"mass/collision profile is outside the project repository: {source}"
    )


def _load_profile_context(config: object) -> _ProfileContext:
    try:
        contract = load_robot_config_contract(config.files.robot)
        profile_path = contract.mass_collision_profile.resolve()
        profile = load_mass_collision_profile(profile_path)
        asset_path = Path(config.robot.asset_path).resolve()
    except (AttributeError, OSError, ValueError) as exc:
        raise MassCollisionRuntimeError(
            f"mass/collision runtime configuration is invalid: {exc}"
        ) from exc
    if not asset_path.is_file():
        raise MassCollisionRuntimeError(
            f"robot asset is not a regular file: {asset_path}"
        )
    asset_sha256 = _file_sha256(asset_path)
    if asset_sha256 != profile.robot_asset_sha256:
        raise MassCollisionRuntimeError(
            "robot asset SHA256 does not match mass/collision profile: "
            f"expected={profile.robot_asset_sha256}, actual={asset_sha256}"
        )
    expected_base_path = resolve_prim_suffix(
        config.robot.articulation_root, profile.base_prim_suffix
    )
    if expected_base_path != config.robot.base_link_prim:
        raise MassCollisionRuntimeError(
            "mass/collision base prim does not match project robot binding: "
            f"expected={expected_base_path}, "
            f"configured={config.robot.base_link_prim}"
        )
    profile_sha256 = _file_sha256(profile_path)
    return _ProfileContext(
        profile_path=profile_path,
        profile=profile,
        profile_evidence=ProfileEvidence(
            path=_repository_relative_path(profile_path),
            sha256=profile_sha256,
            id=profile.profile_id,
            mode=profile.mode,
        ),
        asset_sha256=asset_sha256,
    )


def _layer_metadata(context: _ProfileContext) -> dict[str, object]:
    return {
        _LAYER_MARKER: True,
        "mass_collision_schema_version": 1,
        "mass_collision_profile_path": context.profile_evidence.path,
        "mass_collision_profile_sha256": context.profile_evidence.sha256,
        "mass_collision_profile_id": context.profile.profile_id,
        "mass_collision_profile_mode": context.profile.mode,
        "robot_asset_sha256": context.asset_sha256,
    }


def _remove_existing_layers(stage: object) -> None:
    from pxr import Sdf

    session = stage.GetSessionLayer()
    for identifier in list(session.subLayerPaths):
        layer = Sdf.Layer.Find(identifier)
        if layer and layer.customLayerData.get(_LAYER_MARKER) is True:
            session.subLayerPaths.remove(identifier)


def _create_layer(stage: object, context: _ProfileContext):
    from pxr import Sdf

    layer = Sdf.Layer.CreateAnonymous(
        f"mass_collision_{context.profile.profile_id}.usda"
    )
    layer.customLayerData = _layer_metadata(context)
    stage.GetSessionLayer().subLayerPaths.insert(0, layer.identifier)
    return layer


def _find_layer(stage: object, context: _ProfileContext):
    from pxr import Sdf

    matches = []
    for identifier in stage.GetSessionLayer().subLayerPaths:
        layer = Sdf.Layer.Find(identifier)
        if layer and layer.customLayerData.get(_LAYER_MARKER) is True:
            matches.append(layer)
    if len(matches) != 1:
        raise MassCollisionRuntimeError(
            "expected one active mass/collision session layer, "
            f"found {len(matches)}"
        )
    layer = matches[0]
    expected = _layer_metadata(context)
    actual = dict(layer.customLayerData)
    if actual != expected:
        raise MassCollisionRuntimeError(
            "mass/collision session layer metadata mismatch: "
            f"expected={expected}, actual={actual}"
        )
    return layer


def _author_profile(
    stage: object,
    config: object,
    profile: MassCollisionProfile,
) -> None:
    from pxr import Gf, UsdPhysics

    if profile.mode == "legacy_default_sensor_density":
        return
    for shell in profile.sensor_shells:
        path = resolve_prim_suffix(
            config.robot.articulation_root, shell.prim_suffix
        )
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid() or not prim.HasAPI(
            UsdPhysics.CollisionAPI
        ):
            raise MassCollisionRuntimeError(
                f"mass/collision sensor shell is invalid: {path}"
            )
        collision_attribute = UsdPhysics.CollisionAPI(
            prim
        ).CreateCollisionEnabledAttr()
        if not collision_attribute.Set(shell.collision_enabled):
            raise MassCollisionRuntimeError(
                f"failed to author shell collisionEnabled: {path}"
            )
        if not prim.SetActive(shell.active):
            raise MassCollisionRuntimeError(
                f"failed to author shell active state: {path}"
            )

    if profile.base_inertial is None:
        return
    prim = stage.GetPrimAtPath(config.robot.base_link_prim)
    if not prim or not prim.IsValid() or not prim.HasAPI(UsdPhysics.MassAPI):
        raise MassCollisionRuntimeError(
            f"fixed base inertial prim is invalid: {config.robot.base_link_prim}"
        )
    diagonal, principal_axes_wxyz = _decompose_inertia(
        profile.base_inertial.inertia_kg_m2
    )
    mass_api = UsdPhysics.MassAPI(prim)
    writes = (
        (mass_api.CreateMassAttr(), profile.base_inertial.mass_kg),
        (
            mass_api.CreateCenterOfMassAttr(),
            Gf.Vec3f(*profile.base_inertial.center_of_mass_m),
        ),
        (mass_api.CreateDiagonalInertiaAttr(), Gf.Vec3f(*diagonal)),
        (
            mass_api.CreatePrincipalAxesAttr(),
            Gf.Quatf(
                principal_axes_wxyz[0],
                Gf.Vec3f(*principal_axes_wxyz[1:]),
            ),
        ),
    )
    for attribute, value in writes:
        if not attribute.Set(value):
            raise MassCollisionRuntimeError(
                f"failed to author fixed base inertial: {attribute.GetPath()}"
            )


def _expected_authorship(
    config: object,
    profile: MassCollisionProfile,
) -> tuple[dict[str, tuple[object, object]], dict[str, bool]]:
    from pxr import Sdf

    attributes: dict[str, tuple[object, object]] = {}
    active: dict[str, bool] = {}
    if profile.mode != "legacy_default_sensor_density":
        for shell in profile.sensor_shells:
            path = resolve_prim_suffix(
                config.robot.articulation_root, shell.prim_suffix
            )
            attributes[f"{path}.{_COLLISION_ATTRIBUTE}"] = (
                Sdf.ValueTypeNames.Bool,
                shell.collision_enabled,
            )
            active[path] = shell.active
    if profile.base_inertial is not None:
        base = config.robot.base_link_prim
        attributes.update(
            {
                f"{base}.{_MASS_ATTRIBUTE}": (
                    Sdf.ValueTypeNames.Float,
                    None,
                ),
                f"{base}.{_CENTER_OF_MASS_ATTRIBUTE}": (
                    Sdf.ValueTypeNames.Point3f,
                    None,
                ),
                f"{base}.{_DIAGONAL_INERTIA_ATTRIBUTE}": (
                    Sdf.ValueTypeNames.Float3,
                    None,
                ),
                f"{base}.{_PRINCIPAL_AXES_ATTRIBUTE}": (
                    Sdf.ValueTypeNames.Quatf,
                    None,
                ),
            }
        )
    return attributes, active


def _ancestor_paths(paths: Iterable[str]) -> set[str]:
    return {
        "/" + "/".join(path.strip("/").split("/")[:depth])
        for path in paths
        for depth in range(1, len(path.strip("/").split("/")) + 1)
    }


def _validate_layer_authorship(
    layer: object,
    config: object,
    profile: MassCollisionProfile,
) -> None:
    from pxr import Sdf

    expected_attributes, expected_active = _expected_authorship(config, profile)
    authored_prim_targets = {
        str(Sdf.Path(path).GetPrimPath()) for path in expected_attributes
    } | set(expected_active)
    expected_prims = _ancestor_paths(authored_prim_targets)
    actual_attributes: dict[str, object] = {}
    actual_active: dict[str, object] = {}
    actual_prims: set[str] = set()
    invalid: list[str] = []

    def visit(path: object) -> None:
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.PseudoRootSpec):
            return
        if isinstance(spec, Sdf.PrimSpec):
            prim_path = str(path)
            actual_prims.add(prim_path)
            info_keys = set(spec.ListInfoKeys())
            allowed = {"specifier"}
            if prim_path in expected_active:
                allowed.add("active")
                if "active" in info_keys:
                    actual_active[prim_path] = spec.GetInfo("active")
            if info_keys - allowed or spec.specifier != Sdf.SpecifierOver:
                invalid.append(prim_path)
            return
        if isinstance(spec, Sdf.AttributeSpec):
            property_path = str(path)
            actual_attributes[property_path] = spec.default
            expected = expected_attributes.get(property_path)
            if expected is None or spec.typeName != expected[0]:
                invalid.append(property_path)
            elif expected[1] is not None and spec.default is not expected[1]:
                invalid.append(property_path)
            return
        invalid.append(str(path))

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    if (
        invalid
        or set(actual_attributes) != set(expected_attributes)
        or actual_active != expected_active
        or actual_prims != expected_prims
    ):
        raise MassCollisionRuntimeError(
            "mass/collision layer authored opinions outside the exact "
            "profile contract: "
            f"invalid={sorted(invalid)}, expected_attributes="
            f"{sorted(expected_attributes)}, actual_attributes="
            f"{sorted(actual_attributes)}, expected_active={expected_active}, "
            f"actual_active={actual_active}, expected_prims="
            f"{sorted(expected_prims)}, actual_prims={sorted(actual_prims)}"
        )


def _read_shell(
    stage: object,
    layer: object,
    path: str,
    expected_active: bool,
    expected_collision_enabled: bool,
) -> ShellStageSnapshot:
    from pxr import Sdf, UsdPhysics

    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise MassCollisionRuntimeError(
            f"effective sensor shell prim is missing: {path}"
        )
    active = prim.IsActive()
    if active:
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            raise MassCollisionRuntimeError(
                f"effective sensor shell lacks CollisionAPI: {path}"
            )
        collision_enabled = UsdPhysics.CollisionAPI(
            prim
        ).GetCollisionEnabledAttr().Get()
    else:
        spec = layer.GetAttributeAtPath(
            Sdf.Path(path).AppendProperty(_COLLISION_ATTRIBUTE)
        )
        collision_enabled = spec.default if spec is not None else None
    if (
        active is not expected_active
        or collision_enabled is not expected_collision_enabled
    ):
        raise MassCollisionRuntimeError(
            "sensor shell readback mismatch: "
            f"path={path}, expected_active={expected_active}, "
            f"actual_active={active}, expected_collisionEnabled="
            f"{expected_collision_enabled}, actual_collisionEnabled="
            f"{collision_enabled!r}"
        )
    return ShellStageSnapshot(
        prim_path=path,
        active=active,
        collision_enabled=collision_enabled,
    )


def _read_base_inertial(
    stage: object,
    config: object,
    expected: BaseInertial,
) -> BaseInertialStageSnapshot:
    from pxr import UsdPhysics

    path = config.robot.base_link_prim
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid() or not prim.HasAPI(UsdPhysics.MassAPI):
        raise MassCollisionRuntimeError(
            f"effective fixed base inertial prim is invalid: {path}"
        )
    mass_api = UsdPhysics.MassAPI(prim)
    mass_kg = _finite_number(
        mass_api.GetMassAttr().Get(), context="Stage base mass", positive=True
    )
    center_of_mass_m = _three_vector(
        mass_api.GetCenterOfMassAttr().Get(), context="Stage base COM"
    )
    diagonal = _three_vector(
        mass_api.GetDiagonalInertiaAttr().Get(),
        context="Stage base diagonal inertia",
    )
    if any(value <= 0.0 for value in diagonal):
        raise MassCollisionRuntimeError(
            "Stage base diagonal inertia must be positive"
        )
    quaternion = mass_api.GetPrincipalAxesAttr().Get()
    try:
        imaginary = quaternion.GetImaginary()
        principal_axes_wxyz = (
            float(quaternion.GetReal()),
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
        )
    except (AttributeError, IndexError, TypeError, ValueError) as exc:
        raise MassCollisionRuntimeError(
            "Stage base principalAxes quaternion is invalid"
        ) from exc
    if not all(math.isfinite(value) for value in principal_axes_wxyz):
        raise MassCollisionRuntimeError(
            "Stage base principalAxes quaternion must be finite"
        )
    inertia = _rotate_inertia(
        (
            (diagonal[0], 0.0, 0.0),
            (0.0, diagonal[1], 0.0),
            (0.0, 0.0, diagonal[2]),
        ),
        _quaternion_to_matrix(principal_axes_wxyz),
    )
    if not math.isclose(
        mass_kg,
        expected.mass_kg,
        rel_tol=1e-7,
        abs_tol=_STAGE_ABS_TOL,
    ):
        raise MassCollisionRuntimeError("fixed base mass Stage readback mismatch")
    if not _vector_close(
        center_of_mass_m,
        expected.center_of_mass_m,
        abs_tol=_STAGE_ABS_TOL,
    ):
        raise MassCollisionRuntimeError("fixed base COM Stage readback mismatch")
    if not _matrix_close(
        inertia,
        expected.inertia_kg_m2,
        abs_tol=_STAGE_ABS_TOL,
    ):
        raise MassCollisionRuntimeError(
            "fixed base inertia Stage readback mismatch"
        )
    return BaseInertialStageSnapshot(
        prim_path=path,
        mass_kg=mass_kg,
        center_of_mass_m=center_of_mass_m,
        inertia_kg_m2=inertia,
    )


def _validate_stage_snapshot_binding(
    snapshot: MassCollisionStageSnapshot,
    config: object,
    context: _ProfileContext,
) -> None:
    if not isinstance(snapshot, MassCollisionStageSnapshot):
        raise MassCollisionRuntimeError(
            "stage snapshot must be a MassCollisionStageSnapshot"
        )
    if snapshot.schema_version != 1 or not snapshot.stage_usd_readback_verified:
        raise MassCollisionRuntimeError("mass/collision stage snapshot is not verified")
    profile = context.profile
    expected_shells = tuple(
        sorted(
            (
                ShellStageSnapshot(
                    prim_path=resolve_prim_suffix(
                        config.robot.articulation_root, shell.prim_suffix
                    ),
                    active=shell.active,
                    collision_enabled=shell.collision_enabled,
                )
                for shell in profile.sensor_shells
            ),
            key=lambda item: item.prim_path,
        )
    )
    expected_masses = tuple(
        sorted(
            (
                LinkMassExpectationSnapshot(
                    prim_path=resolve_prim_suffix(
                        config.robot.articulation_root,
                        expectation.prim_suffix,
                    ),
                    mass_kg=expectation.mass_kg,
                )
                for expectation in profile.expected_link_masses
            ),
            key=lambda item: item.prim_path,
        )
    )
    expected_base = (
        BaseInertialStageSnapshot(
            prim_path=config.robot.base_link_prim,
            mass_kg=profile.base_inertial.mass_kg,
            center_of_mass_m=profile.base_inertial.center_of_mass_m,
            inertia_kg_m2=profile.base_inertial.inertia_kg_m2,
        )
        if profile.base_inertial is not None
        else None
    )
    if snapshot.profile != context.profile_evidence:
        raise MassCollisionRuntimeError(
            "mass/collision stage snapshot profile mismatch"
        )
    if (
        snapshot.robot_asset_sha256 != context.asset_sha256
        or snapshot.sensor_shells != expected_shells
        or snapshot.base_inertial != expected_base
        or snapshot.expected_link_masses != expected_masses
        or not math.isclose(
            snapshot.expected_total_mass_kg,
            profile.expected_total_mass_kg,
            rel_tol=1e-9,
            abs_tol=1e-9,
        )
        or snapshot.overlay.id
        != f"mass_collision_profile/{profile.profile_id}"
        or not snapshot.overlay.identifier.startswith("anon:")
        or _SHA256_PATTERN.fullmatch(snapshot.overlay.sha256) is None
    ):
        raise MassCollisionRuntimeError(
            "mass/collision stage snapshot contract mismatch"
        )


def _unwrap_articulation(value: object) -> object:
    required = (
        "get_link_masses",
        "get_link_coms",
        "get_link_inertias",
    )
    if all(callable(getattr(value, name, None)) for name in required):
        return value
    try:
        articulation = value.articulation
    except (AttributeError, RuntimeError) as exc:
        raise MassCollisionRuntimeError(
            "expected an initialized Articulation or runtime wrapper"
        ) from exc
    if not all(callable(getattr(articulation, name, None)) for name in required):
        raise MassCollisionRuntimeError(
            "runtime wrapper does not expose an Isaac Articulation"
        )
    return articulation


def _one_articulation_link_paths(value: object) -> list[str]:
    if (
        not isinstance(value, (list, tuple))
        or len(value) != 1
        or not isinstance(value[0], (list, tuple))
        or not all(isinstance(path, str) and path for path in value[0])
    ):
        raise MassCollisionRuntimeError(
            "articulation link_paths must contain one ordered articulation row"
        )
    return list(value[0])


def _tensor_to_python(value: object, *, context: str) -> object:
    try:
        converted = value.numpy()
    except (AttributeError, RuntimeError, TypeError, ValueError) as exc:
        raise MassCollisionRuntimeError(
            f"{context} must be an Isaac tensor with numpy readback"
        ) from exc
    if hasattr(converted, "tolist"):
        converted = converted.tolist()
    return converted


def _shape(value: object) -> tuple[int, ...] | None:
    if not isinstance(value, (list, tuple)):
        return ()
    if not value:
        return (0,)
    child_shapes = [_shape(item) for item in value]
    if any(shape is None for shape in child_shapes) or len(set(child_shapes)) != 1:
        return None
    return (len(value),) + child_shapes[0]  # type: ignore[operator]


def _validated_nested_numeric(
    value: object,
    *,
    expected_shape: tuple[int, ...],
    context: str,
) -> object:
    actual_shape = _shape(value)
    if actual_shape != expected_shape:
        raise MassCollisionRuntimeError(
            f"{context} mismatch: expected={expected_shape}, actual={actual_shape}"
        )

    def validate(item: object) -> None:
        if isinstance(item, (list, tuple)):
            for child in item:
                validate(child)
        else:
            _finite_number(item, context=context)

    validate(value)
    return value


def _read_tensor_matrix(
    articulation: object,
    method_name: str,
    *,
    expected_shape: tuple[int, ...],
) -> object:
    method = getattr(articulation, method_name, None)
    if not callable(method):
        raise MassCollisionRuntimeError(
            f"articulation must provide {method_name}"
        )
    value = _tensor_to_python(method(), context=method_name)
    return _validated_nested_numeric(
        value,
        expected_shape=expected_shape,
        context=f"{method_name} shape",
    )


def _finite_number(
    value: object,
    *,
    context: str,
    positive: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MassCollisionRuntimeError(f"{context} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise MassCollisionRuntimeError(f"{context} must be finite")
    if positive and result <= 0.0:
        raise MassCollisionRuntimeError(f"{context} must be positive")
    return result


def _three_vector(value: object, *, context: str) -> tuple[float, float, float]:
    try:
        items = tuple(value)
    except TypeError as exc:
        raise MassCollisionRuntimeError(
            f"{context} must contain three finite numbers"
        ) from exc
    if len(items) != 3:
        raise MassCollisionRuntimeError(
            f"{context} must contain three finite numbers"
        )
    return tuple(  # type: ignore[return-value]
        _finite_number(item, context=context) for item in items
    )


def _matrix_from_flat(
    values: Sequence[object],
    *,
    context: str,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    if len(values) != 9:
        raise MassCollisionRuntimeError(f"{context} must contain nine values")
    flat = tuple(_finite_number(value, context=context) for value in values)
    return (
        (flat[0], flat[1], flat[2]),
        (flat[3], flat[4], flat[5]),
        (flat[6], flat[7], flat[8]),
    )


def _require_symmetric_positive_definite(
    matrix: Sequence[Sequence[float]],
    *,
    context: str,
) -> None:
    if not _matrix_close(matrix, _transpose(matrix), abs_tol=1e-5):
        raise MassCollisionRuntimeError(f"{context} must be symmetric")
    a, b, c = matrix[0]
    _, d, e = matrix[1]
    _, _, f = matrix[2]
    if (
        a <= 0.0
        or a * d - b * b <= 0.0
        or a * (d * f - e * e)
        - b * (b * f - c * e)
        + c * (b * e - c * d)
        <= 0.0
    ):
        raise MassCollisionRuntimeError(
            f"{context} must be symmetric positive definite"
        )


def _decompose_inertia(
    matrix: Sequence[Sequence[float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Deterministic Jacobi eigendecomposition for a symmetric 3x3 tensor."""

    working = [[float(matrix[row][column]) for column in range(3)] for row in range(3)]
    vectors = [
        [1.0 if row == column else 0.0 for column in range(3)]
        for row in range(3)
    ]
    pairs = ((0, 1), (0, 2), (1, 2))
    for _ in range(64):
        p, q = max(pairs, key=lambda pair: abs(working[pair[0]][pair[1]]))
        off_diagonal = working[p][q]
        if abs(off_diagonal) <= 1e-15:
            break
        angle = 0.5 * math.atan2(
            2.0 * off_diagonal, working[q][q] - working[p][p]
        )
        cosine = math.cos(angle)
        sine = math.sin(angle)
        app = working[p][p]
        aqq = working[q][q]
        apq = working[p][q]
        for index in range(3):
            if index in (p, q):
                continue
            aip = working[index][p]
            aiq = working[index][q]
            working[index][p] = working[p][index] = cosine * aip - sine * aiq
            working[index][q] = working[q][index] = sine * aip + cosine * aiq
        working[p][p] = (
            cosine * cosine * app
            - 2.0 * sine * cosine * apq
            + sine * sine * aqq
        )
        working[q][q] = (
            sine * sine * app
            + 2.0 * sine * cosine * apq
            + cosine * cosine * aqq
        )
        working[p][q] = working[q][p] = 0.0
        for row in range(3):
            vip = vectors[row][p]
            viq = vectors[row][q]
            vectors[row][p] = cosine * vip - sine * viq
            vectors[row][q] = sine * vip + cosine * viq

    order = sorted(range(3), key=lambda index: (working[index][index], index))
    diagonal = tuple(working[index][index] for index in order)
    rotation = [[vectors[row][column] for column in order] for row in range(3)]
    for column in range(3):
        pivot = max(range(3), key=lambda row: (abs(rotation[row][column]), -row))
        if rotation[pivot][column] < 0.0:
            for row in range(3):
                rotation[row][column] *= -1.0
    if _determinant(rotation) < 0.0:
        for row in range(3):
            rotation[row][2] *= -1.0
    quaternion = _matrix_to_quaternion(rotation)
    reconstructed = _rotate_inertia(
        (
            (diagonal[0], 0.0, 0.0),
            (0.0, diagonal[1], 0.0),
            (0.0, 0.0, diagonal[2]),
        ),
        rotation,
    )
    if not _matrix_close(reconstructed, matrix, abs_tol=1e-10):
        raise MassCollisionRuntimeError(
            "deterministic inertia eigendecomposition failed reconstruction"
        )
    return diagonal, quaternion  # type: ignore[return-value]


def _matrix_to_quaternion(
    matrix: Sequence[Sequence[float]],
) -> tuple[float, float, float, float]:
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            0.25 * scale,
            (matrix[2][1] - matrix[1][2]) / scale,
            (matrix[0][2] - matrix[2][0]) / scale,
            (matrix[1][0] - matrix[0][1]) / scale,
        )
    else:
        index = max(range(3), key=lambda item: matrix[item][item])
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
            quaternion = (
                (matrix[2][1] - matrix[1][2]) / scale,
                0.25 * scale,
                (matrix[0][1] + matrix[1][0]) / scale,
                (matrix[0][2] + matrix[2][0]) / scale,
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
            quaternion = (
                (matrix[0][2] - matrix[2][0]) / scale,
                (matrix[0][1] + matrix[1][0]) / scale,
                0.25 * scale,
                (matrix[1][2] + matrix[2][1]) / scale,
            )
        else:
            scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
            quaternion = (
                (matrix[1][0] - matrix[0][1]) / scale,
                (matrix[0][2] + matrix[2][0]) / scale,
                (matrix[1][2] + matrix[2][1]) / scale,
                0.25 * scale,
            )
    norm = math.sqrt(sum(value * value for value in quaternion))
    normalized = tuple(value / norm for value in quaternion)
    first_nonzero = next(
        (value for value in normalized if abs(value) > 1e-15), 1.0
    )
    if first_nonzero < 0.0:
        normalized = tuple(-value for value in normalized)
    return normalized  # type: ignore[return-value]


def _quaternion_to_matrix(
    quaternion_wxyz: Sequence[float],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    if len(quaternion_wxyz) != 4:
        raise MassCollisionRuntimeError("quaternion must contain four values")
    w, x, y, z = (
        _finite_number(value, context="quaternion")
        for value in quaternion_wxyz
    )
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isclose(norm, 1.0, rel_tol=1e-5, abs_tol=1e-5):
        raise MassCollisionRuntimeError("quaternion must have unit norm")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def _rotate_inertia(
    inertia: Sequence[Sequence[float]],
    rotation: Sequence[Sequence[float]],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    product = _matrix_multiply(rotation, inertia)
    return _matrix_multiply(product, _transpose(rotation))


def _matrix_multiply(
    left: Sequence[Sequence[float]],
    right: Sequence[Sequence[float]],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    return tuple(  # type: ignore[return-value]
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _transpose(
    matrix: Sequence[Sequence[float]],
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    return tuple(  # type: ignore[return-value]
        tuple(matrix[column][row] for column in range(3)) for row in range(3)
    )


def _determinant(matrix: Sequence[Sequence[float]]) -> float:
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (
        d * h - e * g
    )


def _vector_close(
    actual: Sequence[float],
    expected: Sequence[float],
    *,
    abs_tol: float,
) -> bool:
    return len(actual) == len(expected) and all(
        math.isclose(a, e, rel_tol=1e-6, abs_tol=abs_tol)
        for a, e in zip(actual, expected)
    )


def _matrix_close(
    actual: Sequence[Sequence[float]],
    expected: Sequence[Sequence[float]],
    *,
    abs_tol: float,
) -> bool:
    return len(actual) == len(expected) and all(
        _vector_close(actual_row, expected_row, abs_tol=abs_tol)
        for actual_row, expected_row in zip(actual, expected)
    )


__all__ = [
    "BaseInertialStageSnapshot",
    "LinkMassExpectationSnapshot",
    "MassCollisionRuntimeError",
    "MassCollisionStageSnapshot",
    "MassTensorSnapshot",
    "OverlayEvidence",
    "ProfileEvidence",
    "ShellStageSnapshot",
    "TensorLinkSnapshot",
    "apply_mass_collision_profile",
    "capture_mass_collision_snapshot",
    "capture_mass_tensor_snapshot",
]
