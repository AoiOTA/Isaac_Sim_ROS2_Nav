from __future__ import annotations

from dataclasses import replace
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import pytest
import yaml

from isaac_sim.src.config import load_project_config
from isaac_sim.src.robot import wheel_velocity_drive
from isaac_sim.src.robot.kinematics_config import load_robot_config_contract


ROOT = Path(__file__).resolve().parents[2]
PROJECT_CONFIG = ROOT / "isaac_sim/configs/project.yaml"
ROBOT_CONFIG = ROOT / "isaac_sim/configs/robots/jackal.yaml"
ROBOT_ASSET = ROOT / "isaac_sim/assets/robots/jackal/jackal_nav.usda"
ISAAC_ASSET_ROOT = Path("/home/lyb/isaacsim_assets/Assets/Isaac/6.0")
MARKER = "isaac_nav_wheel_velocity_drive_layer"


try:
    import pxr  # noqa: F401
except ImportError:
    HAS_PXR = False
else:
    HAS_PXR = True


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _config(tmp_path: Path):
    data = yaml.safe_load(ROBOT_CONFIG.read_text())
    mass_profile = load_robot_config_contract(
        ROBOT_CONFIG
    ).mass_collision_profile
    data["mass_collision_profile"] = str(mass_profile)
    data["wheel_velocity_drive"] = {
        "schema_version": 1,
        "profile_id": "jackal_drive_test_v1",
        "drive_type": "force",
        "stiffness_n_m_per_rad": 0.0,
        "damping_n_m_s_per_rad": 2.5,
        "max_effort_n_m": 6.25,
        "max_joint_velocity_rad_s": 20.0,
    }
    robot_path = tmp_path / "jackal_drive_test.yaml"
    robot_path.write_text(yaml.safe_dump(data, sort_keys=False))
    config = load_project_config(
        PROJECT_CONFIG,
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": str(ISAAC_ASSET_ROOT),
        },
    )
    # The runtime root remains a project composition property, but wheel names
    # must come from the selected schema-v3 robot contract rather than this
    # legacy duplicate in project.yaml.
    return replace(
        config,
        robot=replace(
            config.robot,
            wheel_joints=("bogus_0", "bogus_1", "bogus_2", "bogus_3"),
        ),
        files=replace(config.files, robot=robot_path),
    )


def _synthetic_stage(config):
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    contract = load_robot_config_contract(config.files.robot)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Robots")
    UsdGeom.Xform.Define(stage, config.robot.runtime_prim_path)
    for name in contract.wheel_joints.ordered:
        joint = UsdPhysics.RevoluteJoint.Define(
            stage,
            f"{config.robot.runtime_prim_path}/{name}",
        )
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr().Set("acceleration")
        drive.CreateStiffnessAttr().Set(9.0)
        drive.CreateDampingAttr().Set(8.0)
        drive.CreateMaxForceAttr().Set(7.0)
        joint.GetPrim().CreateAttribute(
            "physxJoint:maxJointVelocity",
            Sdf.ValueTypeNames.Float,
            custom=False,
        ).Set(6.0)
    stage.SetEditTarget(stage.GetRootLayer())
    return stage


def _drive_layers(stage):
    from pxr import Sdf

    return [
        layer
        for identifier in stage.GetSessionLayer().subLayerPaths
        if (layer := Sdf.Layer.Find(identifier))
        and layer.customLayerData.get(MARKER) is True
    ]


def _stage_snapshot(config):
    contract = load_robot_config_contract(config.files.robot)
    drive = contract.wheel_velocity_drive
    configured = wheel_velocity_drive.WheelVelocityDriveConfiguredSi(
        drive_type=drive.drive_type,
        stiffness_n_m_per_rad=drive.stiffness_n_m_per_rad,
        damping_n_m_s_per_rad=drive.damping_n_m_s_per_rad,
        max_effort_n_m=drive.max_effort_n_m,
        max_joint_velocity_rad_s=drive.max_joint_velocity_rad_s,
    )
    authored = wheel_velocity_drive.WheelVelocityDriveAuthoredUsd(
        drive_type=drive.drive_type,
        stiffness_n_m_per_degree=(
            drive.stiffness_n_m_per_rad * math.pi / 180.0
        ),
        damping_n_m_s_per_degree=(
            drive.damping_n_m_s_per_rad * math.pi / 180.0
        ),
        max_force_n_m=drive.max_effort_n_m,
        max_joint_velocity_deg_s=(
            drive.max_joint_velocity_rad_s * 180.0 / math.pi
        ),
    )
    return wheel_velocity_drive.WheelVelocityDriveSnapshot(
        schema_version=1,
        profile_path=str(config.files.robot.resolve()),
        profile_sha256=_sha256(config.files.robot),
        profile_id=drive.profile_id,
        configured_si=configured,
        authored_usd=authored,
        joint_paths=tuple(
            f"{config.robot.runtime_prim_path}/{name}"
            for name in contract.wheel_joints.ordered
        ),
        overlay_identifier="anon:unit-test:wheel_velocity_drive.usda",
        overlay_sha256="a" * 64,
        stage_usd_readback_verified=True,
    )


class _TensorValues:
    def __init__(self, values):
        self._values = values

    def numpy(self):
        import numpy as np

        return np.asarray(self._values, dtype=np.float32)


class _FakeTensorArticulation:
    def __init__(self, config):
        joints = load_robot_config_contract(config.files.robot).wheel_joints
        self.dof_names = [
            "auxiliary_joint",
            joints.rear_right,
            joints.front_right,
            joints.front_left,
            joints.rear_left,
        ]
        self.valid = True
        self.drive_types = [["none", "force", "force", "force", "force"]]
        self.stiffnesses = [[1.0, 0.0, 0.0, 0.0, 0.0]]
        self.dampings = [[1.0, 2.5, 2.5, 2.5, 2.5]]
        self.max_efforts = [[1.0, 6.25, 6.25, 6.25, 6.25]]
        self.max_velocities = [[1.0, 20.0, 20.0, 20.0, 20.0]]
        self.calls = []

    def is_physics_tensor_entity_valid(self):
        return self.valid

    def get_dof_drive_types(self):
        self.calls.append("get_dof_drive_types")
        return self.drive_types

    def get_dof_gains(self):
        self.calls.append("get_dof_gains")
        return _TensorValues(self.stiffnesses), _TensorValues(self.dampings)

    def get_dof_max_efforts(self):
        self.calls.append("get_dof_max_efforts")
        return _TensorValues(self.max_efforts)

    def get_dof_max_velocities(self):
        self.calls.append("get_dof_max_velocities")
        return _TensorValues(self.max_velocities)


class _ArticulationRuntimeWrapper:
    def __init__(self, articulation):
        self.articulation = articulation

    def get_dof_names(self):
        return tuple(self.articulation.dof_names)


def test_wheel_velocity_drive_module_exists():
    assert importlib.util.find_spec(
        "isaac_sim.src.robot.wheel_velocity_drive"
    ) is not None


@pytest.mark.parametrize("wrapped", (False, True))
def test_tensor_capture_selects_four_wheels_by_name_and_verifies_si_values(
    tmp_path,
    wrapped,
):
    config = _config(tmp_path)
    stage_snapshot = _stage_snapshot(config)
    articulation = _FakeTensorArticulation(config)
    source = (
        _ArticulationRuntimeWrapper(articulation)
        if wrapped
        else articulation
    )

    capture = getattr(
        wheel_velocity_drive,
        "capture_wheel_drive_tensor_snapshot",
    )
    snapshot = capture(source, config, stage_snapshot)

    joints = load_robot_config_contract(config.files.robot).wheel_joints
    assert snapshot.schema_version == 1
    assert snapshot.profile_path == stage_snapshot.profile_path
    assert snapshot.profile_sha256 == stage_snapshot.profile_sha256
    assert snapshot.profile_id == stage_snapshot.profile_id
    assert snapshot.stage_overlay_sha256 == stage_snapshot.overlay_sha256
    assert snapshot.dof_names == joints.ordered
    assert snapshot.dof_indices == (3, 2, 4, 1)
    assert snapshot.drive_types == ("force",) * 4
    assert snapshot.stiffnesses_n_m_per_rad == (0.0,) * 4
    assert snapshot.dampings_n_m_s_per_rad == (2.5,) * 4
    assert snapshot.max_efforts_n_m == (6.25,) * 4
    assert snapshot.max_joint_velocities_rad_s == (20.0,) * 4
    assert snapshot.physics_tensor_readback_verified is True
    assert articulation.calls == [
        "get_dof_drive_types",
        "get_dof_gains",
        "get_dof_max_efforts",
        "get_dof_max_velocities",
    ]
    assert json.dumps(
        snapshot.to_dict(),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


@pytest.mark.parametrize(
    "field, replacement, message",
    (
        ("profile_id", "wrong_profile", "does not match"),
        ("profile_sha256", "b" * 64, "does not match"),
        ("overlay_identifier", None, "does not match"),
        ("overlay_sha256", "not-a-sha", "does not match"),
        ("stage_usd_readback_verified", False, "does not match"),
    ),
)
def test_tensor_capture_rejects_unbound_or_unverified_stage_snapshot(
    tmp_path,
    field,
    replacement,
    message,
):
    config = _config(tmp_path)
    stage_snapshot = replace(
        _stage_snapshot(config),
        **{field: replacement},
    )

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match=message,
    ):
        wheel_velocity_drive.capture_wheel_drive_tensor_snapshot(
            _FakeTensorArticulation(config),
            config,
            stage_snapshot,
        )


@pytest.mark.parametrize(
    "mutation, message",
    (
        ("invalid_entity", "tensor entity is not valid"),
        ("missing_name", "resolved to 0 tensor indices"),
        ("duplicate_name", "dof_names must be nonempty unique strings"),
        ("missing_getter", "get_dof_max_efforts is unavailable"),
    ),
)
def test_tensor_capture_fails_closed_when_required_api_or_dof_is_missing(
    tmp_path,
    mutation,
    message,
):
    config = _config(tmp_path)
    articulation = _FakeTensorArticulation(config)
    joints = load_robot_config_contract(config.files.robot).wheel_joints
    if mutation == "invalid_entity":
        articulation.valid = False
    elif mutation == "missing_name":
        articulation.dof_names[4] = "missing_rear_left"
    elif mutation == "duplicate_name":
        articulation.dof_names[0] = joints.front_left
    else:
        articulation.get_dof_max_efforts = None

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match=message,
    ):
        wheel_velocity_drive.capture_wheel_drive_tensor_snapshot(
            articulation,
            config,
            _stage_snapshot(config),
        )


@pytest.mark.parametrize(
    "mutation, message",
    (
        ("drive_rows", "drive types tensor expected one articulation row"),
        ("stiffness_width", "stiffness tensor expected 5 DOF values"),
        ("damping_rows", "damping tensor expected one articulation row"),
        ("effort_flat", "max effort tensor expected one articulation row"),
        ("velocity_nan", "max joint velocity tensor contains"),
    ),
)
def test_tensor_capture_rejects_wrong_shapes_and_nonfinite_values(
    tmp_path,
    mutation,
    message,
):
    config = _config(tmp_path)
    articulation = _FakeTensorArticulation(config)
    if mutation == "drive_rows":
        articulation.drive_types = articulation.drive_types[0]
    elif mutation == "stiffness_width":
        articulation.stiffnesses = [[0.0, 0.0]]
    elif mutation == "damping_rows":
        articulation.dampings = [
            articulation.dampings[0],
            articulation.dampings[0],
        ]
    elif mutation == "effort_flat":
        articulation.max_efforts = articulation.max_efforts[0]
    else:
        articulation.max_velocities[0][3] = float("nan")

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match=message,
    ):
        wheel_velocity_drive.capture_wheel_drive_tensor_snapshot(
            articulation,
            config,
            _stage_snapshot(config),
        )


@pytest.mark.parametrize(
    "field, value, message",
    (
        ("drive_types", "acceleration", "drive type readback mismatch"),
        ("stiffnesses", 1.0, "stiffness readback mismatch"),
        ("dampings", 3.0, "damping readback mismatch"),
        ("max_efforts", 7.0, "max effort readback mismatch"),
        ("max_velocities", 21.0, "max joint velocity readback mismatch"),
    ),
)
def test_tensor_capture_rejects_each_tampered_wheel_value(
    tmp_path,
    field,
    value,
    message,
):
    config = _config(tmp_path)
    articulation = _FakeTensorArticulation(config)
    getattr(articulation, field)[0][3] = value

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match=message,
    ):
        wheel_velocity_drive.capture_wheel_drive_tensor_snapshot(
            articulation,
            config,
            _stage_snapshot(config),
        )


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_apply_authors_exact_converted_drive_contract_in_anonymous_layer(
    tmp_path,
):
    from pxr import Sdf, UsdPhysics

    config = _config(tmp_path)
    contract = load_robot_config_contract(config.files.robot)
    stage = _synthetic_stage(config)
    root_before = stage.GetRootLayer().ExportToString()

    apply = getattr(wheel_velocity_drive, "apply_wheel_velocity_drive")
    snapshot = apply(stage, config)

    expected_paths = tuple(
        f"{config.robot.runtime_prim_path}/{name}"
        for name in contract.wheel_joints.ordered
    )
    assert snapshot.schema_version == 1
    assert snapshot.profile_path == str(config.files.robot.resolve())
    assert snapshot.profile_sha256 == _sha256(config.files.robot)
    assert snapshot.profile_id == "jackal_drive_test_v1"
    assert snapshot.configured_si.drive_type == "force"
    assert snapshot.configured_si.stiffness_n_m_per_rad == 0.0
    assert snapshot.configured_si.damping_n_m_s_per_rad == 2.5
    assert snapshot.configured_si.max_effort_n_m == 6.25
    assert snapshot.configured_si.max_joint_velocity_rad_s == 20.0
    assert snapshot.authored_usd.drive_type == "force"
    assert snapshot.authored_usd.stiffness_n_m_per_degree == 0.0
    assert snapshot.authored_usd.damping_n_m_s_per_degree == pytest.approx(
        2.5 * math.pi / 180.0
    )
    assert snapshot.authored_usd.max_force_n_m == 6.25
    assert snapshot.authored_usd.max_joint_velocity_deg_s == pytest.approx(
        20.0 * 180.0 / math.pi
    )
    assert snapshot.joint_paths == expected_paths
    assert snapshot.overlay_identifier.startswith("anon:")
    assert len(snapshot.overlay_sha256) == 64
    assert snapshot.stage_usd_readback_verified is True
    assert json.dumps(
        snapshot.to_dict(),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )

    layers = _drive_layers(stage)
    assert len(layers) == 1
    expected_attributes = {
        str(Sdf.Path(path).AppendProperty(attribute))
        for path in expected_paths
        for attribute in (
            "drive:angular:physics:type",
            "drive:angular:physics:stiffness",
            "drive:angular:physics:damping",
            "drive:angular:physics:maxForce",
            "physxJoint:maxJointVelocity",
        )
    }
    actual_attributes = set()

    def collect(path):
        if isinstance(layers[0].GetObjectAtPath(path), Sdf.AttributeSpec):
            actual_attributes.add(str(path))

    layers[0].Traverse(Sdf.Path.absoluteRootPath, collect)
    assert actual_attributes == expected_attributes
    assert stage.GetRootLayer().ExportToString() == root_before

    for path in expected_paths:
        prim = stage.GetPrimAtPath(path)
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        assert drive.GetTypeAttr().Get() == "force"
        assert drive.GetStiffnessAttr().Get() == pytest.approx(0.0)
        assert drive.GetDampingAttr().Get() == pytest.approx(
            2.5 * math.pi / 180.0
        )
        assert drive.GetMaxForceAttr().Get() == pytest.approx(6.25)
        assert prim.GetAttribute("physxJoint:maxJointVelocity").Get() == (
            pytest.approx(20.0 * 180.0 / math.pi)
        )


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_real_jackal_composition_is_reversible_and_persistent_assets_stay_clean(
    tmp_path,
):
    from pxr import Usd, UsdGeom

    config = _config(tmp_path)
    watched = (
        ROBOT_ASSET,
        ROBOT_ASSET.parent / "source/jackal_original.usd",
    )
    before = {path: _sha256(path) for path in watched}
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Robots")
    robot = UsdGeom.Xform.Define(
        stage,
        config.robot.runtime_prim_path,
    ).GetPrim()
    assert robot.GetReferences().AddReference(str(ROBOT_ASSET))
    root_before = stage.GetRootLayer().ExportToString()

    first = wheel_velocity_drive.apply_wheel_velocity_drive(stage, config)
    # Keep the detached Sdf layer alive so the registry cannot recycle its
    # anonymous identifier while proving a fresh layer was created.
    first_layer = _drive_layers(stage)[0]
    second = wheel_velocity_drive.apply_wheel_velocity_drive(stage, config)

    assert first.overlay_identifier != second.overlay_identifier
    assert first_layer.identifier == first.overlay_identifier
    assert first.overlay_sha256 == second.overlay_sha256
    assert len(_drive_layers(stage)) == 1
    assert (
        wheel_velocity_drive.capture_wheel_velocity_drive_snapshot(
            stage,
            config,
        )
        == second
    )
    assert stage.GetRootLayer().ExportToString() == root_before
    assert {path: _sha256(path) for path in watched} == before


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
@pytest.mark.parametrize(
    "mutation, message",
    (
        ("missing", "resolved to 0 prims"),
        ("wrong_type", "must be one active RevoluteJoint"),
        ("duplicate", "resolved to 2 prims"),
        ("missing_drive", "lacks angular DriveAPI"),
    ),
)
def test_apply_rejects_invalid_runtime_joint_contract(
    tmp_path,
    mutation,
    message,
):
    from pxr import UsdGeom, UsdPhysics

    config = _config(tmp_path)
    stage = _synthetic_stage(config)
    name = load_robot_config_contract(
        config.files.robot
    ).wheel_joints.front_left
    path = f"{config.robot.runtime_prim_path}/{name}"
    if mutation == "missing":
        assert stage.RemovePrim(path)
    elif mutation == "wrong_type":
        assert stage.RemovePrim(path)
        UsdPhysics.PrismaticJoint.Define(stage, path)
    elif mutation == "duplicate":
        UsdGeom.Xform.Define(stage, f"{config.robot.runtime_prim_path}/Duplicate")
        joint = UsdPhysics.RevoluteJoint.Define(
            stage,
            f"{config.robot.runtime_prim_path}/Duplicate/{name}",
        )
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    else:
        prim = stage.GetPrimAtPath(path)
        assert prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match=message,
    ):
        wheel_velocity_drive.apply_wheel_velocity_drive(stage, config)
    assert _drive_layers(stage) == []


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_capture_rejects_duplicate_overlay_marker(tmp_path):
    from pxr import Sdf

    config = _config(tmp_path)
    stage = _synthetic_stage(config)
    wheel_velocity_drive.apply_wheel_velocity_drive(stage, config)
    duplicate = Sdf.Layer.CreateAnonymous("duplicate_wheel_drive.usda")
    duplicate.customLayerData = {MARKER: True}
    stage.GetSessionLayer().subLayerPaths.insert(0, duplicate.identifier)

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match="expected one active.*found 2",
    ):
        wheel_velocity_drive.capture_wheel_velocity_drive_snapshot(
            stage,
            config,
        )


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_capture_rejects_tampered_layer_metadata(tmp_path):
    config = _config(tmp_path)
    stage = _synthetic_stage(config)
    wheel_velocity_drive.apply_wheel_velocity_drive(stage, config)
    layer = _drive_layers(stage)[0]
    metadata = dict(layer.customLayerData)
    metadata["wheel_velocity_drive_profile_id"] = "tampered"
    layer.customLayerData = metadata

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match="layer metadata mismatch",
    ):
        wheel_velocity_drive.capture_wheel_velocity_drive_snapshot(
            stage,
            config,
        )


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_capture_rejects_authorship_outside_exact_contract(tmp_path):
    from pxr import Sdf

    config = _config(tmp_path)
    stage = _synthetic_stage(config)
    wheel_velocity_drive.apply_wheel_velocity_drive(stage, config)
    Sdf.CreatePrimInLayer(_drive_layers(stage)[0], "/Unexpected")

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match="authored opinions outside the exact",
    ):
        wheel_velocity_drive.capture_wheel_velocity_drive_snapshot(
            stage,
            config,
        )


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_capture_rejects_metadata_tamper_on_expected_attribute(tmp_path):
    from pxr import Sdf

    config = _config(tmp_path)
    stage = _synthetic_stage(config)
    snapshot = wheel_velocity_drive.apply_wheel_velocity_drive(stage, config)
    property_path = Sdf.Path(snapshot.joint_paths[0]).AppendProperty(
        "drive:angular:physics:damping"
    )
    specification = _drive_layers(stage)[0].GetAttributeAtPath(property_path)
    specification.SetInfo("documentation", "tampered")

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match="authored opinions outside the exact",
    ):
        wheel_velocity_drive.capture_wheel_velocity_drive_snapshot(
            stage,
            config,
        )


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_capture_rejects_stronger_effective_stage_readback_tamper(tmp_path):
    config = _config(tmp_path)
    stage = _synthetic_stage(config)
    wheel_velocity_drive.apply_wheel_velocity_drive(stage, config)
    joint = load_robot_config_contract(
        config.files.robot
    ).wheel_joints.front_left
    path = f"{config.robot.runtime_prim_path}/{joint}"
    original_target = stage.GetEditTarget()
    stage.SetEditTarget(stage.GetSessionLayer())
    assert stage.GetPrimAtPath(path).GetAttribute(
        "drive:angular:physics:damping"
    ).Set(99.0)
    stage.SetEditTarget(original_target)

    with pytest.raises(
        wheel_velocity_drive.WheelVelocityDriveError,
        match="effective damping readback mismatch",
    ):
        wheel_velocity_drive.capture_wheel_velocity_drive_snapshot(
            stage,
            config,
        )
