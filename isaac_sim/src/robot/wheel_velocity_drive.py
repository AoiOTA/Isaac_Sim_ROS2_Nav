"""Versioned, reversible wheel velocity-drive USD overlay.

The selected robot asset stays read-only.  Drive opinions are authored in one
anonymous Stage session layer before PhysX initializes, then re-read from both
that exact layer and the effective composed Stage for provenance.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import math
from numbers import Real
from pathlib import Path

from isaac_sim.src.robot.kinematics_config import (
    RobotWheelVelocityDrive,
    load_robot_config_contract,
)


class WheelVelocityDriveError(RuntimeError):
    """Raised when wheel-drive authorship or readback is not exact."""


_LAYER_MARKER = "isaac_nav_wheel_velocity_drive_layer"


@dataclass(frozen=True)
class WheelVelocityDriveConfiguredSi:
    drive_type: str
    stiffness_n_m_per_rad: float
    damping_n_m_s_per_rad: float
    max_effort_n_m: float
    max_joint_velocity_rad_s: float


@dataclass(frozen=True)
class WheelVelocityDriveAuthoredUsd:
    drive_type: str
    stiffness_n_m_per_degree: float
    damping_n_m_s_per_degree: float
    max_force_n_m: float
    max_joint_velocity_deg_s: float


@dataclass(frozen=True)
class WheelVelocityDriveSnapshot:
    """Canonical schema-v1 Stage evidence for the selected drive profile."""

    schema_version: int
    profile_path: str
    profile_sha256: str
    profile_id: str
    configured_si: WheelVelocityDriveConfiguredSi
    authored_usd: WheelVelocityDriveAuthoredUsd
    joint_paths: tuple[str, str, str, str]
    overlay_identifier: str
    overlay_sha256: str
    stage_usd_readback_verified: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class WheelVelocityDriveTensorSnapshot:
    """Schema-v1 evidence read from an initialized physics tensor view."""

    schema_version: int
    profile_path: str
    profile_sha256: str
    profile_id: str
    stage_overlay_sha256: str
    dof_names: tuple[str, str, str, str]
    dof_indices: tuple[int, int, int, int]
    drive_types: tuple[str, str, str, str]
    stiffnesses_n_m_per_rad: tuple[float, float, float, float]
    dampings_n_m_s_per_rad: tuple[float, float, float, float]
    max_efforts_n_m: tuple[float, float, float, float]
    max_joint_velocities_rad_s: tuple[float, float, float, float]
    physics_tensor_readback_verified: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True)
class _Profile:
    path: Path
    sha256: str
    identifier: str
    configured_si: WheelVelocityDriveConfiguredSi
    authored_usd: WheelVelocityDriveAuthoredUsd
    joint_names: tuple[str, str, str, str]


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _profile(config: object) -> _Profile:
    path = Path(config.files.robot).resolve()
    contract = load_robot_config_contract(path)
    drive = contract.wheel_velocity_drive
    configured = _configured_si(drive)
    return _Profile(
        path=path,
        sha256=_file_sha256(path),
        identifier=drive.profile_id,
        configured_si=configured,
        authored_usd=_authored_usd(configured),
        joint_names=contract.wheel_joints.ordered,
    )


def _configured_si(
    drive: RobotWheelVelocityDrive,
) -> WheelVelocityDriveConfiguredSi:
    return WheelVelocityDriveConfiguredSi(
        drive_type=drive.drive_type,
        stiffness_n_m_per_rad=drive.stiffness_n_m_per_rad,
        damping_n_m_s_per_rad=drive.damping_n_m_s_per_rad,
        max_effort_n_m=drive.max_effort_n_m,
        max_joint_velocity_rad_s=drive.max_joint_velocity_rad_s,
    )


def _authored_usd(
    configured: WheelVelocityDriveConfiguredSi,
) -> WheelVelocityDriveAuthoredUsd:
    return WheelVelocityDriveAuthoredUsd(
        drive_type=configured.drive_type,
        stiffness_n_m_per_degree=(
            configured.stiffness_n_m_per_rad * math.pi / 180.0
        ),
        damping_n_m_s_per_degree=(
            configured.damping_n_m_s_per_rad * math.pi / 180.0
        ),
        max_force_n_m=configured.max_effort_n_m,
        max_joint_velocity_deg_s=(
            configured.max_joint_velocity_rad_s * 180.0 / math.pi
        ),
    )


def _resolve_joint_paths(
    stage: object,
    config: object,
    profile: _Profile,
) -> tuple[str, str, str, str]:
    from pxr import Sdf, UsdPhysics

    root = Sdf.Path(config.robot.runtime_prim_path)
    if not root.IsAbsolutePath() or not root.IsPrimPath():
        raise WheelVelocityDriveError(
            f"runtime robot path must be an absolute prim path: {root}"
        )
    paths: list[str] = []
    for name in profile.joint_names:
        matches = [
            prim
            for prim in stage.TraverseAll()
            if prim.GetName() == name and prim.GetPath().HasPrefix(root)
        ]
        if len(matches) != 1:
            raise WheelVelocityDriveError(
                f"runtime wheel joint {name!r} resolved to {len(matches)} prims"
            )
        prim = matches[0]
        if (
            not prim.IsValid()
            or not prim.IsActive()
            or not prim.IsA(UsdPhysics.RevoluteJoint)
        ):
            raise WheelVelocityDriveError(
                f"runtime wheel joint {name!r} must be one active RevoluteJoint"
            )
        if not prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
            raise WheelVelocityDriveError(
                f"runtime wheel joint {name!r} lacks angular DriveAPI"
            )
        paths.append(str(prim.GetPath()))
    if len(paths) != 4 or len(set(paths)) != 4:
        raise WheelVelocityDriveError(
            "wheel velocity-drive overlay requires four unique runtime joints"
        )
    return tuple(paths)  # type: ignore[return-value]


def _layer_metadata(profile: _Profile) -> dict[str, object]:
    return {
        _LAYER_MARKER: True,
        "wheel_velocity_drive_schema_version": 1,
        "wheel_velocity_drive_profile_id": profile.identifier,
        "wheel_velocity_drive_profile_sha256": profile.sha256,
    }


def _remove_existing_layers(stage: object) -> None:
    from pxr import Sdf

    session = stage.GetSessionLayer()
    for identifier in list(session.subLayerPaths):
        layer = Sdf.Layer.Find(identifier)
        if layer and layer.customLayerData.get(_LAYER_MARKER) is True:
            session.subLayerPaths.remove(identifier)


def _create_layer(stage: object, profile: _Profile):
    from pxr import Sdf

    layer = Sdf.Layer.CreateAnonymous(
        f"wheel_velocity_drive_{profile.identifier}.usda"
    )
    layer.customLayerData = _layer_metadata(profile)
    stage.GetSessionLayer().subLayerPaths.insert(0, layer.identifier)
    return layer


def _find_layer(stage: object, profile: _Profile):
    from pxr import Sdf

    matches = []
    for identifier in stage.GetSessionLayer().subLayerPaths:
        layer = Sdf.Layer.Find(identifier)
        if layer and layer.customLayerData.get(_LAYER_MARKER) is True:
            matches.append(layer)
    if len(matches) != 1:
        raise WheelVelocityDriveError(
            "expected one active wheel velocity-drive session layer, "
            f"found {len(matches)}"
        )
    layer = matches[0]
    expected = _layer_metadata(profile)
    actual = dict(layer.customLayerData)
    if actual != expected:
        raise WheelVelocityDriveError(
            "wheel velocity-drive layer metadata mismatch: "
            f"expected={expected}, actual={actual}"
        )
    return layer


def _set(attribute: object, value: object, *, context: str) -> None:
    if not attribute.IsValid() or not attribute.Set(value):
        raise WheelVelocityDriveError(f"failed to author {context}")


def _author_joints(
    stage: object,
    joint_paths: tuple[str, str, str, str],
    authored: WheelVelocityDriveAuthoredUsd,
) -> None:
    from pxr import Sdf, UsdPhysics

    for path in joint_paths:
        prim = stage.GetPrimAtPath(path)
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            raise WheelVelocityDriveError(
                f"angular DriveAPI disappeared before authorship: {path}"
            )
        _set(
            drive.CreateTypeAttr(),
            authored.drive_type,
            context=f"{path}.drive type",
        )
        _set(
            drive.CreateStiffnessAttr(),
            authored.stiffness_n_m_per_degree,
            context=f"{path}.stiffness",
        )
        _set(
            drive.CreateDampingAttr(),
            authored.damping_n_m_s_per_degree,
            context=f"{path}.damping",
        )
        _set(
            drive.CreateMaxForceAttr(),
            authored.max_force_n_m,
            context=f"{path}.maxForce",
        )
        max_velocity = prim.CreateAttribute(
            "physxJoint:maxJointVelocity",
            Sdf.ValueTypeNames.Float,
            custom=False,
        )
        _set(
            max_velocity,
            authored.max_joint_velocity_deg_s,
            context=f"{path}.maxJointVelocity",
        )


def _close(actual: object, expected: float) -> bool:
    return (
        not isinstance(actual, bool)
        and isinstance(actual, Real)
        and math.isfinite(float(actual))
        and math.isclose(
            float(actual),
            expected,
            rel_tol=1e-6,
            abs_tol=1e-8,
        )
    )


def _expected_layer_defaults(
    joint_paths: tuple[str, str, str, str],
    authored: WheelVelocityDriveAuthoredUsd,
) -> dict[str, tuple[object, object, object]]:
    from pxr import Sdf

    values = (
        (
            "drive:angular:physics:type",
            Sdf.ValueTypeNames.Token,
            authored.drive_type,
            Sdf.VariabilityUniform,
        ),
        (
            "drive:angular:physics:stiffness",
            Sdf.ValueTypeNames.Float,
            authored.stiffness_n_m_per_degree,
            Sdf.VariabilityVarying,
        ),
        (
            "drive:angular:physics:damping",
            Sdf.ValueTypeNames.Float,
            authored.damping_n_m_s_per_degree,
            Sdf.VariabilityVarying,
        ),
        (
            "drive:angular:physics:maxForce",
            Sdf.ValueTypeNames.Float,
            authored.max_force_n_m,
            Sdf.VariabilityVarying,
        ),
        (
            "physxJoint:maxJointVelocity",
            Sdf.ValueTypeNames.Float,
            authored.max_joint_velocity_deg_s,
            Sdf.VariabilityVarying,
        ),
    )
    return {
        str(Sdf.Path(path).AppendProperty(name)): (
            type_name,
            value,
            variability,
        )
        for path in joint_paths
        for name, type_name, value, variability in values
    }


def _validate_layer_authorship(
    layer: object,
    joint_paths: tuple[str, str, str, str],
    authored: WheelVelocityDriveAuthoredUsd,
) -> None:
    from pxr import Sdf

    expected_attributes = _expected_layer_defaults(joint_paths, authored)
    expected_prim_paths = {
        "/" + "/".join(path.strip("/").split("/")[:depth])
        for path in joint_paths
        for depth in range(1, len(path.strip("/").split("/")) + 1)
    }
    actual_attributes: set[str] = set()
    actual_prim_paths: set[str] = set()
    invalid: list[str] = []

    def visit(path: object) -> None:
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.PseudoRootSpec):
            return
        if isinstance(spec, Sdf.PrimSpec):
            actual_prim_paths.add(str(path))
            if (
                spec.specifier != Sdf.SpecifierOver
                or set(spec.ListInfoKeys()) - {"specifier"}
            ):
                invalid.append(str(path))
            return
        if isinstance(spec, Sdf.AttributeSpec):
            property_path = str(path)
            actual_attributes.add(property_path)
            expected = expected_attributes.get(property_path)
            if (
                expected is None
                or spec.typeName != expected[0]
                or spec.variability != expected[2]
                or spec.custom is not False
                or set(spec.ListInfoKeys())
                != {"default", "typeName", "variability", "custom"}
            ):
                invalid.append(property_path)
                return
            target = expected[1]
            if isinstance(target, str):
                if str(spec.default) != target:
                    invalid.append(property_path)
            elif not _close(spec.default, target):
                invalid.append(property_path)
            return
        invalid.append(str(path))

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    if (
        invalid
        or actual_attributes != set(expected_attributes)
        or actual_prim_paths != expected_prim_paths
    ):
        raise WheelVelocityDriveError(
            "wheel velocity-drive layer authored opinions outside the exact "
            "five-attribute-per-joint contract: "
            f"invalid={sorted(invalid)}, expected_attributes="
            f"{sorted(expected_attributes)}, actual_attributes="
            f"{sorted(actual_attributes)}, expected_prims="
            f"{sorted(expected_prim_paths)}, actual_prims="
            f"{sorted(actual_prim_paths)}"
        )


def _read_effective_joints(
    stage: object,
    joint_paths: tuple[str, str, str, str],
    expected: WheelVelocityDriveAuthoredUsd,
) -> None:
    from pxr import UsdPhysics

    for path in joint_paths:
        prim = stage.GetPrimAtPath(path)
        if (
            not prim.IsValid()
            or not prim.IsActive()
            or not prim.IsA(UsdPhysics.RevoluteJoint)
            or not prim.HasAPI(UsdPhysics.DriveAPI, "angular")
        ):
            raise WheelVelocityDriveError(
                f"effective runtime wheel joint is invalid: {path}"
            )
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        actual_type = drive.GetTypeAttr().Get()
        values = (
            ("stiffness", drive.GetStiffnessAttr().Get(), expected.stiffness_n_m_per_degree),
            ("damping", drive.GetDampingAttr().Get(), expected.damping_n_m_s_per_degree),
            ("maxForce", drive.GetMaxForceAttr().Get(), expected.max_force_n_m),
            (
                "maxJointVelocity",
                prim.GetAttribute("physxJoint:maxJointVelocity").Get(),
                expected.max_joint_velocity_deg_s,
            ),
        )
        if str(actual_type) != expected.drive_type:
            raise WheelVelocityDriveError(
                f"effective drive type readback mismatch at {path}: "
                f"expected={expected.drive_type}, actual={actual_type!r}"
            )
        for name, actual, target in values:
            if not _close(actual, target):
                raise WheelVelocityDriveError(
                    f"effective {name} readback mismatch at {path}: "
                    f"expected={target}, actual={actual!r}"
                )


def _validate_stage_snapshot(
    config: object,
    profile: _Profile,
    snapshot: WheelVelocityDriveSnapshot,
) -> None:
    expected_paths = tuple(
        f"{config.robot.runtime_prim_path}/{name}"
        for name in profile.joint_names
    )
    overlay_identifier_valid = (
        isinstance(snapshot.overlay_identifier, str)
        and snapshot.overlay_identifier.startswith("anon:")
    )
    overlay_sha256_valid = (
        isinstance(snapshot.overlay_sha256, str)
        and len(snapshot.overlay_sha256) == 64
        and all(
            character in "0123456789abcdef"
            for character in snapshot.overlay_sha256
        )
    )
    expected = (
        snapshot.schema_version == 1,
        snapshot.profile_path == str(profile.path),
        snapshot.profile_sha256 == profile.sha256,
        snapshot.profile_id == profile.identifier,
        snapshot.configured_si == profile.configured_si,
        snapshot.authored_usd == profile.authored_usd,
        snapshot.joint_paths == expected_paths,
        overlay_identifier_valid,
        overlay_sha256_valid,
        snapshot.stage_usd_readback_verified is True,
    )
    if not all(expected):
        raise WheelVelocityDriveError(
            "Stage wheel velocity-drive snapshot does not match the selected "
            "schema-v1 profile"
        )


def _unwrap_articulation(
    articulation_or_wrapper: object,
) -> tuple[object, tuple[str, ...]]:
    getter = getattr(articulation_or_wrapper, "get_dof_names", None)
    if callable(getter):
        try:
            articulation = articulation_or_wrapper.articulation
            names_value = getter()
        except Exception as exc:
            raise WheelVelocityDriveError(
                "initialized articulation wrapper is unavailable"
            ) from exc
    else:
        articulation = articulation_or_wrapper
        try:
            names_value = articulation.dof_names
        except Exception as exc:
            raise WheelVelocityDriveError(
                "experimental Articulation.dof_names is unavailable"
            ) from exc
    try:
        names = tuple(names_value)
    except TypeError as exc:
        raise WheelVelocityDriveError(
            "articulation dof_names must be an ordered sequence"
        ) from exc
    if (
        not names
        or not all(isinstance(name, str) and name for name in names)
        or len(set(names)) != len(names)
    ):
        raise WheelVelocityDriveError(
            "articulation dof_names must be nonempty unique strings"
        )
    validity = getattr(
        articulation,
        "is_physics_tensor_entity_valid",
        None,
    )
    if not callable(validity):
        raise WheelVelocityDriveError(
            "Articulation lacks physics tensor validity readback"
        )
    try:
        valid = validity()
    except Exception as exc:
        raise WheelVelocityDriveError(
            "physics tensor validity readback failed"
        ) from exc
    if valid is not True:
        raise WheelVelocityDriveError(
            "physics articulation tensor entity is not valid"
        )
    return articulation, names


def _call_tensor_getter(
    articulation: object,
    name: str,
) -> object:
    getter = getattr(articulation, name, None)
    if not callable(getter):
        raise WheelVelocityDriveError(
            f"Articulation tensor API {name} is unavailable"
        )
    try:
        return getter()
    except Exception as exc:
        raise WheelVelocityDriveError(
            f"Articulation tensor API {name} failed"
        ) from exc


def _single_tensor_row(
    values: object,
    *,
    name: str,
    width: int,
    numeric: bool,
) -> tuple[object, ...]:
    if numeric:
        converter = getattr(values, "numpy", None)
        if not callable(converter):
            raise WheelVelocityDriveError(
                f"{name} tensor lacks numpy readback"
            )
        try:
            values = converter()
        except Exception as exc:
            raise WheelVelocityDriveError(
                f"{name} tensor numpy readback failed"
            ) from exc
    try:
        rows = list(values)
    except TypeError as exc:
        raise WheelVelocityDriveError(
            f"{name} tensor readback must be two-dimensional"
        ) from exc
    if len(rows) != 1:
        raise WheelVelocityDriveError(
            f"{name} tensor expected one articulation row, got {len(rows)}"
        )
    try:
        row = tuple(rows[0])
    except TypeError as exc:
        raise WheelVelocityDriveError(
            f"{name} tensor row must be a sequence"
        ) from exc
    if len(row) != width:
        raise WheelVelocityDriveError(
            f"{name} tensor expected {width} DOF values, got {len(row)}"
        )
    if numeric:
        if not all(
            not isinstance(value, bool)
            and isinstance(value, Real)
            and math.isfinite(float(value))
            for value in row
        ):
            raise WheelVelocityDriveError(
                f"{name} tensor contains a non-finite or non-numeric value"
            )
        return tuple(float(value) for value in row)
    if not all(isinstance(value, str) for value in row):
        raise WheelVelocityDriveError(
            f"{name} tensor contains a non-string drive type"
        )
    return row


def _verify_selected_values(
    *,
    actual: tuple[float, float, float, float],
    expected: float,
    name: str,
) -> None:
    mismatches = [
        (index, value)
        for index, value in enumerate(actual)
        if not math.isclose(
            value,
            expected,
            rel_tol=1e-5,
            abs_tol=1e-7,
        )
    ]
    if mismatches:
        raise WheelVelocityDriveError(
            f"physics tensor {name} readback mismatch: "
            f"expected={expected}, actual={actual}, mismatches={mismatches}"
        )


def capture_wheel_drive_tensor_snapshot(
    articulation_or_wrapper: object,
    config: object,
    stage_snapshot: WheelVelocityDriveSnapshot,
) -> WheelVelocityDriveTensorSnapshot:
    """Verify the initialized PhysX tensor view in SI units.

    Stage evidence is an explicit input so callers cannot accidentally present
    a composed-USD readback as proof that PhysX consumed those values.
    """

    if not isinstance(stage_snapshot, WheelVelocityDriveSnapshot):
        raise WheelVelocityDriveError(
            "tensor verification requires a WheelVelocityDriveSnapshot"
        )
    profile = _profile(config)
    _validate_stage_snapshot(config, profile, stage_snapshot)
    articulation, dof_names = _unwrap_articulation(articulation_or_wrapper)
    indices: list[int] = []
    for name in profile.joint_names:
        matches = [index for index, value in enumerate(dof_names) if value == name]
        if len(matches) != 1:
            raise WheelVelocityDriveError(
                f"wheel DOF {name!r} resolved to {len(matches)} tensor indices"
            )
        indices.append(matches[0])

    drive_types_row = _single_tensor_row(
        _call_tensor_getter(articulation, "get_dof_drive_types"),
        name="drive types",
        width=len(dof_names),
        numeric=False,
    )
    gains = _call_tensor_getter(articulation, "get_dof_gains")
    if not isinstance(gains, (tuple, list)) or len(gains) != 2:
        raise WheelVelocityDriveError(
            "get_dof_gains must return stiffness and damping tensors"
        )
    stiffness_row = _single_tensor_row(
        gains[0],
        name="stiffness",
        width=len(dof_names),
        numeric=True,
    )
    damping_row = _single_tensor_row(
        gains[1],
        name="damping",
        width=len(dof_names),
        numeric=True,
    )
    max_effort_row = _single_tensor_row(
        _call_tensor_getter(articulation, "get_dof_max_efforts"),
        name="max effort",
        width=len(dof_names),
        numeric=True,
    )
    max_velocity_row = _single_tensor_row(
        _call_tensor_getter(articulation, "get_dof_max_velocities"),
        name="max joint velocity",
        width=len(dof_names),
        numeric=True,
    )

    selected_drive_types = tuple(drive_types_row[index] for index in indices)
    selected_stiffnesses = tuple(
        float(stiffness_row[index]) for index in indices
    )
    selected_dampings = tuple(float(damping_row[index]) for index in indices)
    selected_max_efforts = tuple(
        float(max_effort_row[index]) for index in indices
    )
    selected_max_velocities = tuple(
        float(max_velocity_row[index]) for index in indices
    )
    configured = profile.configured_si
    if selected_drive_types != (configured.drive_type,) * 4:
        raise WheelVelocityDriveError(
            "physics tensor drive type readback mismatch: "
            f"expected={configured.drive_type}, actual={selected_drive_types}"
        )
    _verify_selected_values(
        actual=selected_stiffnesses,
        expected=configured.stiffness_n_m_per_rad,
        name="stiffness",
    )
    _verify_selected_values(
        actual=selected_dampings,
        expected=configured.damping_n_m_s_per_rad,
        name="damping",
    )
    _verify_selected_values(
        actual=selected_max_efforts,
        expected=configured.max_effort_n_m,
        name="max effort",
    )
    _verify_selected_values(
        actual=selected_max_velocities,
        expected=configured.max_joint_velocity_rad_s,
        name="max joint velocity",
    )
    return WheelVelocityDriveTensorSnapshot(
        schema_version=1,
        profile_path=str(profile.path),
        profile_sha256=profile.sha256,
        profile_id=profile.identifier,
        stage_overlay_sha256=stage_snapshot.overlay_sha256,
        dof_names=profile.joint_names,
        dof_indices=tuple(indices),  # type: ignore[arg-type]
        drive_types=selected_drive_types,  # type: ignore[arg-type]
        stiffnesses_n_m_per_rad=selected_stiffnesses,  # type: ignore[arg-type]
        dampings_n_m_s_per_rad=selected_dampings,  # type: ignore[arg-type]
        max_efforts_n_m=selected_max_efforts,  # type: ignore[arg-type]
        max_joint_velocities_rad_s=selected_max_velocities,  # type: ignore[arg-type]
        physics_tensor_readback_verified=True,
    )


def capture_wheel_velocity_drive_snapshot(
    stage: object,
    config: object,
) -> WheelVelocityDriveSnapshot:
    """Re-read the exact overlay and effective Stage drive values."""

    profile = _profile(config)
    joint_paths = _resolve_joint_paths(stage, config, profile)
    layer = _find_layer(stage, profile)
    _validate_layer_authorship(layer, joint_paths, profile.authored_usd)
    _read_effective_joints(stage, joint_paths, profile.authored_usd)
    source = layer.ExportToString()
    return WheelVelocityDriveSnapshot(
        schema_version=1,
        profile_path=str(profile.path),
        profile_sha256=profile.sha256,
        profile_id=profile.identifier,
        configured_si=profile.configured_si,
        authored_usd=profile.authored_usd,
        joint_paths=joint_paths,
        overlay_identifier=layer.identifier,
        overlay_sha256=hashlib.sha256(source.encode("utf-8")).hexdigest(),
        stage_usd_readback_verified=True,
    )


def apply_wheel_velocity_drive(
    stage: object,
    config: object,
) -> WheelVelocityDriveSnapshot:
    """Author the schema-v1 drive contract in one anonymous session layer."""

    original_target = stage.GetEditTarget()
    _remove_existing_layers(stage)
    try:
        profile = _profile(config)
        joint_paths = _resolve_joint_paths(stage, config, profile)
        layer = _create_layer(stage, profile)
        stage.SetEditTarget(layer)
        _author_joints(stage, joint_paths, profile.authored_usd)
        stage.SetEditTarget(original_target)
        return capture_wheel_velocity_drive_snapshot(stage, config)
    except Exception:
        if stage.GetEditTarget() != original_target:
            stage.SetEditTarget(original_target)
        _remove_existing_layers(stage)
        raise
    finally:
        if stage.GetEditTarget() != original_target:
            stage.SetEditTarget(original_target)


__all__ = [
    "WheelVelocityDriveAuthoredUsd",
    "WheelVelocityDriveConfiguredSi",
    "WheelVelocityDriveError",
    "WheelVelocityDriveSnapshot",
    "WheelVelocityDriveTensorSnapshot",
    "apply_wheel_velocity_drive",
    "capture_wheel_drive_tensor_snapshot",
    "capture_wheel_velocity_drive_snapshot",
]
