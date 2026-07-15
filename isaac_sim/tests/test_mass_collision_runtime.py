from __future__ import annotations

from dataclasses import replace
import hashlib
import math
from pathlib import Path

import pytest
import yaml

from isaac_sim.src.config import load_project_config
from isaac_sim.src.robot.kinematics_config import load_robot_config_contract
from isaac_sim.src.robot.mass_collision_config import (
    load_mass_collision_profile,
    resolve_prim_suffix,
)


ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = "/home/lyb/isaacsim_assets/Assets/Isaac/6.0"
ROBOT_CONFIG = ROOT / "isaac_sim/configs/robots/jackal.yaml"
PROFILE_DIR = ROOT / "isaac_sim/configs/robot_mass_profiles"
MARKER = "isaac_nav_mass_collision_profile_layer"
PROFILE_IDS = (
    "legacy_default_sensor_density_v1",
    "sensor_shells_disabled_v1",
    "fixed_base_inertial_sensor_shell_collision_v1",
)


try:
    import pxr  # noqa: F401
except ImportError:
    HAS_PXR = False
else:
    HAS_PXR = True


def _api():
    from isaac_sim.src.robot.mass_collision_runtime import (
        BaseInertialStageSnapshot,
        LinkMassExpectationSnapshot,
        MassCollisionRuntimeError,
        MassCollisionStageSnapshot,
        MassTensorSnapshot,
        OverlayEvidence,
        ProfileEvidence,
        ShellStageSnapshot,
        TensorLinkSnapshot,
        apply_mass_collision_profile,
        capture_mass_collision_snapshot,
        capture_mass_tensor_snapshot,
    )

    return {
        "BaseInertialStageSnapshot": BaseInertialStageSnapshot,
        "LinkMassExpectationSnapshot": LinkMassExpectationSnapshot,
        "MassCollisionRuntimeError": MassCollisionRuntimeError,
        "MassCollisionStageSnapshot": MassCollisionStageSnapshot,
        "MassTensorSnapshot": MassTensorSnapshot,
        "OverlayEvidence": OverlayEvidence,
        "ProfileEvidence": ProfileEvidence,
        "ShellStageSnapshot": ShellStageSnapshot,
        "TensorLinkSnapshot": TensorLinkSnapshot,
        "apply": apply_mass_collision_profile,
        "capture": capture_mass_collision_snapshot,
        "capture_tensor": capture_mass_tensor_snapshot,
    }


def _project_config(tmp_path: Path, profile_id: str):
    config = load_project_config(
        ROOT / "isaac_sim/configs/project.yaml",
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": ASSET_ROOT,
        },
    )
    robot = yaml.safe_load(ROBOT_CONFIG.read_text(encoding="utf-8"))
    robot["mass_collision_profile"] = str(
        PROFILE_DIR / f"{profile_id}.yaml"
    )
    path = tmp_path / f"robot_{profile_id}.yaml"
    path.write_text(yaml.safe_dump(robot, sort_keys=False), encoding="utf-8")
    return replace(config, files=replace(config.files, robot=path))


def _file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _new_jackal_stage(config):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Robots")
    robot = UsdGeom.Xform.Define(
        stage, config.robot.articulation_root
    ).GetPrim()
    assert robot.GetReferences().AddReference(str(config.robot.asset_path))
    assert stage.GetPrimAtPath(config.robot.base_link_prim).IsValid()
    return stage


def _profile_layers(stage):
    from pxr import Sdf

    return [
        layer
        for identifier in stage.GetSessionLayer().subLayerPaths
        if (layer := Sdf.Layer.Find(identifier))
        and layer.customLayerData.get(MARKER) is True
    ]


def _authored_contract(layer):
    from pxr import Sdf

    attributes: dict[str, object] = {}
    active: dict[str, object] = {}
    invalid: list[str] = []

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.PseudoRootSpec):
            return
        if isinstance(spec, Sdf.PrimSpec):
            extra = set(spec.ListInfoKeys()) - {"specifier", "active"}
            if extra or spec.specifier != Sdf.SpecifierOver:
                invalid.append(str(path))
            if "active" in spec.ListInfoKeys():
                active[str(path)] = spec.GetInfo("active")
            return
        if isinstance(spec, Sdf.AttributeSpec):
            attributes[str(path)] = spec.default
            return
        invalid.append(str(path))

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    assert invalid == []
    return attributes, active


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
@pytest.mark.parametrize(
    ("profile_id", "active", "collision_enabled", "has_inertial"),
    [
        ("legacy_default_sensor_density_v1", True, True, False),
        ("sensor_shells_disabled_v1", False, False, False),
        (
            "fixed_base_inertial_sensor_shell_collision_v1",
            True,
            True,
            True,
        ),
    ],
)
def test_stage_profiles_use_one_anonymous_exact_session_overlay(
    tmp_path,
    profile_id,
    active,
    collision_enabled,
    has_inertial,
):
    api = _api()
    config = _project_config(tmp_path, profile_id)
    stage = _new_jackal_stage(config)
    root_before = stage.GetRootLayer().ExportToString()
    asset_before = _file_sha256(config.robot.asset_path)

    snapshot = api["apply"](stage, config)

    assert stage.GetRootLayer().ExportToString() == root_before
    assert _file_sha256(config.robot.asset_path) == asset_before
    layers = _profile_layers(stage)
    assert len(layers) == 1
    assert layers[0].identifier.startswith("anon:")
    assert dict(layers[0].customLayerData) == {
        MARKER: True,
        "mass_collision_schema_version": 1,
        "mass_collision_profile_path": (
            f"isaac_sim/configs/robot_mass_profiles/{profile_id}.yaml"
        ),
        "mass_collision_profile_sha256": _file_sha256(
            PROFILE_DIR / f"{profile_id}.yaml"
        ),
        "mass_collision_profile_id": profile_id,
        "mass_collision_profile_mode": load_mass_collision_profile(
            PROFILE_DIR / f"{profile_id}.yaml"
        ).mode,
        "robot_asset_sha256": asset_before,
    }
    assert snapshot.schema_version == 1
    assert snapshot.profile.id == profile_id
    assert snapshot.profile.path == (
        f"isaac_sim/configs/robot_mass_profiles/{profile_id}.yaml"
    )
    assert snapshot.profile.sha256 == _file_sha256(
        PROFILE_DIR / f"{profile_id}.yaml"
    )
    assert snapshot.robot_asset_sha256 == asset_before
    assert [
        (shell.active, shell.collision_enabled)
        for shell in snapshot.sensor_shells
    ] == [
        (active, collision_enabled),
        (active, collision_enabled),
    ]
    assert (snapshot.base_inertial is not None) is has_inertial
    assert snapshot.overlay.id == f"mass_collision_profile/{profile_id}"
    assert snapshot.overlay.identifier.startswith("anon:")
    assert len(snapshot.overlay.sha256) == 64
    assert snapshot.stage_usd_readback_verified is True
    assert snapshot == api["capture"](stage, config)

    document = snapshot.to_dict()
    assert list(document) == [
        "schema_version",
        "profile",
        "robot_asset_sha256",
        "sensor_shells",
        "base_inertial",
        "expected_link_masses",
        "expected_total_mass_kg",
        "overlay",
        "stage_usd_readback_verified",
    ]
    assert document["profile"] == {
        "path": snapshot.profile.path,
        "sha256": snapshot.profile.sha256,
        "id": snapshot.profile.id,
        "mode": snapshot.profile.mode,
    }
    assert document["overlay"] == {
        "id": snapshot.overlay.id,
        "identifier": snapshot.overlay.identifier,
        "sha256": snapshot.overlay.sha256,
    }
    assert [item["prim_path"] for item in document["sensor_shells"]] == sorted(
        item["prim_path"] for item in document["sensor_shells"]
    )
    assert [
        item["prim_path"] for item in document["expected_link_masses"]
    ] == sorted(item["prim_path"] for item in document["expected_link_masses"])


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
@pytest.mark.parametrize("profile_id", PROFILE_IDS)
def test_overlay_authorship_is_exact_for_each_mode(tmp_path, profile_id):
    api = _api()
    config = _project_config(tmp_path, profile_id)
    stage = _new_jackal_stage(config)

    api["apply"](stage, config)
    attributes, active = _authored_contract(_profile_layers(stage)[0])

    shell_paths = {
        resolve_prim_suffix(config.robot.articulation_root, suffix)
        for suffix in (
            "/base_link/collisions/bumblebee_camera",
            "/base_link/collisions/sick_lms1xx_lidar",
        )
    }
    collision_attributes = {
        f"{path}.physics:collisionEnabled" for path in shell_paths
    }
    if profile_id == "legacy_default_sensor_density_v1":
        assert attributes == {}
        assert active == {}
    elif profile_id == "sensor_shells_disabled_v1":
        assert set(attributes) == collision_attributes
        assert set(attributes.values()) == {False}
        assert active == {path: False for path in shell_paths}
    else:
        base = config.robot.base_link_prim
        assert set(attributes) == collision_attributes | {
            f"{base}.physics:mass",
            f"{base}.physics:centerOfMass",
            f"{base}.physics:diagonalInertia",
            f"{base}.physics:principalAxes",
        }
        assert all(
            attributes[path] is True for path in collision_attributes
        )
        assert active == {path: True for path in shell_paths}


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_fixed_stage_readback_reconstructs_the_audited_full_inertia(tmp_path):
    api = _api()
    profile_id = "fixed_base_inertial_sensor_shell_collision_v1"
    config = _project_config(tmp_path, profile_id)
    stage = _new_jackal_stage(config)
    expected = load_mass_collision_profile(PROFILE_DIR / f"{profile_id}.yaml")

    snapshot = api["apply"](stage, config)

    assert snapshot.base_inertial is not None
    assert expected.base_inertial is not None
    assert snapshot.base_inertial.prim_path == config.robot.base_link_prim
    assert snapshot.base_inertial.mass_kg == pytest.approx(
        expected.base_inertial.mass_kg
    )
    assert snapshot.base_inertial.center_of_mass_m == pytest.approx(
        expected.base_inertial.center_of_mass_m
    )
    for actual_row, expected_row in zip(
        snapshot.base_inertial.inertia_kg_m2,
        expected.base_inertial.inertia_kg_m2,
    ):
        assert actual_row == pytest.approx(expected_row, abs=1e-7)


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_reapply_replaces_old_layer_and_keeps_stable_overlay_hash(tmp_path):
    api = _api()
    config = _project_config(tmp_path, "sensor_shells_disabled_v1")
    stage = _new_jackal_stage(config)

    first = api["apply"](stage, config)
    second = api["apply"](stage, config)

    assert len(_profile_layers(stage)) == 1
    assert _profile_layers(stage)[0].identifier.startswith("anon:")
    assert second.overlay.id == first.overlay.id
    assert second.overlay.sha256 == first.overlay.sha256


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_capture_rejects_duplicate_marker_and_tampered_metadata(tmp_path):
    from pxr import Sdf

    api = _api()
    error = api["MassCollisionRuntimeError"]
    config = _project_config(tmp_path, "legacy_default_sensor_density_v1")
    stage = _new_jackal_stage(config)
    api["apply"](stage, config)
    duplicate = Sdf.Layer.CreateAnonymous("duplicate_mass_collision.usda")
    duplicate.customLayerData = {MARKER: True}
    stage.GetSessionLayer().subLayerPaths.insert(0, duplicate.identifier)

    with pytest.raises(error, match="expected one active.*found 2"):
        api["capture"](stage, config)

    stage.GetSessionLayer().subLayerPaths.remove(duplicate.identifier)
    layer = _profile_layers(stage)[0]
    metadata = dict(layer.customLayerData)
    metadata["mass_collision_profile_mode"] = "tampered"
    layer.customLayerData = metadata
    with pytest.raises(error, match="metadata mismatch"):
        api["capture"](stage, config)


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_capture_rejects_authorship_and_effective_state_tampering(tmp_path):
    from pxr import Sdf

    api = _api()
    error = api["MassCollisionRuntimeError"]
    config = _project_config(tmp_path, "sensor_shells_disabled_v1")
    stage = _new_jackal_stage(config)
    api["apply"](stage, config)
    layer = _profile_layers(stage)[0]
    Sdf.CreatePrimInLayer(layer, "/Unexpected")

    with pytest.raises(error, match="authored opinions outside"):
        api["capture"](stage, config)


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_legacy_mode_rejects_inherited_shell_state_mismatch(tmp_path):
    from pxr import UsdPhysics

    api = _api()
    error = api["MassCollisionRuntimeError"]
    config = _project_config(tmp_path, "legacy_default_sensor_density_v1")
    stage = _new_jackal_stage(config)
    shell = stage.GetPrimAtPath(
        f"{config.robot.base_link_prim}/collisions/bumblebee_camera"
    )
    UsdPhysics.CollisionAPI(shell).CreateCollisionEnabledAttr().Set(False)

    with pytest.raises(error, match="shell readback mismatch"):
        api["apply"](stage, config)
    assert _profile_layers(stage) == []


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_apply_failure_restores_edit_target_and_removes_overlay(
    tmp_path, monkeypatch
):
    api = _api()
    module = __import__(
        "isaac_sim.src.robot.mass_collision_runtime", fromlist=["ignored"]
    )
    error = api["MassCollisionRuntimeError"]
    config = _project_config(tmp_path, "legacy_default_sensor_density_v1")
    stage = _new_jackal_stage(config)
    original_target = stage.GetEditTarget()

    def fail_capture(_stage, _config):
        raise error("injected readback failure")

    monkeypatch.setattr(module, "capture_mass_collision_snapshot", fail_capture)
    with pytest.raises(error, match="injected readback failure"):
        api["apply"](stage, config)

    assert stage.GetEditTarget() == original_target
    assert _profile_layers(stage) == []


class _FakeTensor:
    def __init__(self, values):
        self._values = values

    def numpy(self):
        return self._values


class _FakeArticulation:
    def __init__(
        self,
        *,
        link_names,
        link_paths,
        masses,
        com_positions,
        com_orientations,
        inertias,
        link_indices=None,
    ):
        self.link_names = link_names
        self.link_paths = [link_paths]
        self._masses = masses
        self._com_positions = com_positions
        self._com_orientations = com_orientations
        self._inertias = inertias
        self._link_indices = link_indices

    def is_physics_tensor_entity_valid(self):
        return True

    def get_link_indices(self, names):
        if self._link_indices is not None:
            return _FakeTensor(self._link_indices)
        return _FakeTensor([self.link_names.index(name) for name in names])

    def get_link_masses(self):
        return _FakeTensor(self._masses)

    def get_link_coms(self):
        return (
            _FakeTensor(self._com_positions),
            _FakeTensor(self._com_orientations),
        )

    def get_link_inertias(self):
        return _FakeTensor(self._inertias)


class _FakeWrapper:
    def __init__(self, articulation):
        self.articulation = articulation


def _stage_snapshot(tmp_path: Path, profile_id: str):
    api = _api()
    config = _project_config(tmp_path, profile_id)
    contract = load_robot_config_contract(config.files.robot)
    profile = load_mass_collision_profile(contract.mass_collision_profile)
    root = config.robot.articulation_root
    profile_path = (
        f"isaac_sim/configs/robot_mass_profiles/{profile_id}.yaml"
    )
    shells = tuple(
        api["ShellStageSnapshot"](
            prim_path=resolve_prim_suffix(root, shell.prim_suffix),
            active=shell.active,
            collision_enabled=shell.collision_enabled,
        )
        for shell in profile.sensor_shells
    )
    base_inertial = (
        api["BaseInertialStageSnapshot"](
            prim_path=resolve_prim_suffix(root, profile.base_prim_suffix),
            mass_kg=profile.base_inertial.mass_kg,
            center_of_mass_m=profile.base_inertial.center_of_mass_m,
            inertia_kg_m2=profile.base_inertial.inertia_kg_m2,
        )
        if profile.base_inertial is not None
        else None
    )
    link_masses = tuple(
        api["LinkMassExpectationSnapshot"](
            prim_path=resolve_prim_suffix(root, expectation.prim_suffix),
            mass_kg=expectation.mass_kg,
        )
        for expectation in profile.expected_link_masses
    )
    snapshot = api["MassCollisionStageSnapshot"](
        schema_version=1,
        profile=api["ProfileEvidence"](
            path=profile_path,
            sha256=_file_sha256(contract.mass_collision_profile),
            id=profile.profile_id,
            mode=profile.mode,
        ),
        robot_asset_sha256=profile.robot_asset_sha256,
        sensor_shells=tuple(sorted(shells, key=lambda item: item.prim_path)),
        base_inertial=base_inertial,
        expected_link_masses=tuple(
            sorted(link_masses, key=lambda item: item.prim_path)
        ),
        expected_total_mass_kg=profile.expected_total_mass_kg,
        overlay=api["OverlayEvidence"](
            id=f"mass_collision_profile/{profile.profile_id}",
            identifier="anon:unit-test:mass_collision.usda",
            sha256="a" * 64,
        ),
        stage_usd_readback_verified=True,
    )
    return config, snapshot


def _fake_articulation(snapshot, *, fixed=True):
    expectations = list(snapshot.expected_link_masses)
    names = [Path(item.prim_path).name for item in expectations]
    paths = [item.prim_path for item in expectations]
    masses = [[item.mass_kg for item in expectations]]
    positions = [[[0.0, 0.0, 0.0] for _ in expectations]]
    inertias = [
        [[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] for _ in expectations]
    ]
    if fixed and snapshot.base_inertial is not None:
        base_index = paths.index(snapshot.base_inertial.prim_path)
        positions[0][base_index] = list(snapshot.base_inertial.center_of_mass_m)
        inertias[0][base_index] = [
            value
            for row in snapshot.base_inertial.inertia_kg_m2
            for value in row
        ]
    orientations = [[[1.0, 0.0, 0.0, 0.0] for _ in expectations]]
    return _FakeArticulation(
        link_names=names,
        link_paths=paths,
        masses=masses,
        com_positions=positions,
        com_orientations=orientations,
        inertias=inertias,
    )


@pytest.mark.parametrize("wrapped", [False, True])
def test_tensor_capture_accepts_direct_or_runtime_wrapper_and_is_canonical(
    tmp_path, wrapped
):
    api = _api()
    config, stage_snapshot = _stage_snapshot(
        tmp_path, "fixed_base_inertial_sensor_shell_collision_v1"
    )
    articulation = _fake_articulation(stage_snapshot)
    source = _FakeWrapper(articulation) if wrapped else articulation

    snapshot = api["capture_tensor"](source, config, stage_snapshot)

    assert snapshot.schema_version == 1
    assert snapshot.profile_id == stage_snapshot.profile.id
    assert snapshot.physics_tensor_readback_verified is True
    assert [link.prim_path for link in snapshot.links] == sorted(
        link.prim_path for link in snapshot.links
    )
    assert snapshot.total_mass_kg == pytest.approx(
        stage_snapshot.expected_total_mass_kg
    )
    assert snapshot.to_dict() == {
        "schema_version": 1,
        "profile_id": stage_snapshot.profile.id,
        "links": [link.to_dict() for link in snapshot.links],
        "total_mass_kg": snapshot.total_mass_kg,
        "physics_tensor_readback_verified": True,
    }


@pytest.mark.parametrize(
    ("field", "replacement", "message"),
    [
        ("masses", [[17.1, 0.477, 0.477, 0.477, 0.477]], "mass readback mismatch"),
        ("masses", [[math.nan, 0.477, 0.477, 0.477, 0.477]], "finite"),
        ("masses", [[17.0, 0.477]], "get_link_masses shape"),
        ("masses", [[17.0] * 5, [17.0] * 5], "get_link_masses shape"),
        ("com_positions", [[[0.0, 0.0, 0.0]]], "get_link_coms position shape"),
        (
            "com_orientations",
            [[[1.0, 0.0, 0.0]] * 5],
            "get_link_coms orientation shape",
        ),
        ("inertias", [[[1.0] * 8] * 5], "get_link_inertias shape"),
    ],
)
def test_tensor_capture_rejects_tampered_values_and_shapes(
    tmp_path, field, replacement, message
):
    api = _api()
    error = api["MassCollisionRuntimeError"]
    config, stage_snapshot = _stage_snapshot(
        tmp_path, "fixed_base_inertial_sensor_shell_collision_v1"
    )
    articulation = _fake_articulation(stage_snapshot)
    setattr(articulation, f"_{field}", replacement)

    with pytest.raises(error, match=message):
        api["capture_tensor"](articulation, config, stage_snapshot)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("duplicate_name", "link_names must be five unique"),
        ("path_name_mismatch", "link name/path order mismatch"),
        ("extra_path", "link_paths must contain exactly"),
        ("missing_path", "link_paths must contain exactly"),
        ("bad_indices", "get_link_indices order mismatch"),
        ("shuffled_names_only", "link name/path order mismatch"),
    ],
)
def test_tensor_capture_rejects_name_path_and_order_tampering(
    tmp_path, mutation, message
):
    api = _api()
    error = api["MassCollisionRuntimeError"]
    config, stage_snapshot = _stage_snapshot(
        tmp_path, "sensor_shells_disabled_v1"
    )
    articulation = _fake_articulation(stage_snapshot)
    if mutation == "duplicate_name":
        articulation.link_names[1] = articulation.link_names[0]
    elif mutation == "path_name_mismatch":
        articulation.link_paths[0][0], articulation.link_paths[0][1] = (
            articulation.link_paths[0][1],
            articulation.link_paths[0][0],
        )
    elif mutation == "extra_path":
        articulation.link_names.append("caster_link")
        articulation.link_paths[0].append(
            f"{config.robot.articulation_root}/caster_link"
        )
    elif mutation == "missing_path":
        articulation.link_names.pop()
        articulation.link_paths[0].pop()
    elif mutation == "bad_indices":
        articulation._link_indices = [1, 0, 2, 3, 4]
    else:
        articulation.link_names[0], articulation.link_names[1] = (
            articulation.link_names[1],
            articulation.link_names[0],
        )

    with pytest.raises(error, match=message):
        api["capture_tensor"](articulation, config, stage_snapshot)


@pytest.mark.parametrize("field", ["center_of_mass", "inertia"])
def test_fixed_tensor_capture_verifies_base_com_and_full_inertia(
    tmp_path, field
):
    api = _api()
    error = api["MassCollisionRuntimeError"]
    config, stage_snapshot = _stage_snapshot(
        tmp_path, "fixed_base_inertial_sensor_shell_collision_v1"
    )
    articulation = _fake_articulation(stage_snapshot)
    base_index = articulation.link_names.index("base_link")
    if field == "center_of_mass":
        articulation._com_positions[0][base_index][0] += 0.01
        message = "base COM readback mismatch"
    else:
        articulation._inertias[0][base_index][0] += 0.01
        message = "base inertia readback mismatch"

    with pytest.raises(error, match=message):
        api["capture_tensor"](articulation, config, stage_snapshot)


def test_tensor_capture_requires_verified_matching_stage_snapshot(tmp_path):
    api = _api()
    error = api["MassCollisionRuntimeError"]
    config, stage_snapshot = _stage_snapshot(
        tmp_path, "sensor_shells_disabled_v1"
    )
    articulation = _fake_articulation(stage_snapshot)

    with pytest.raises(error, match="stage snapshot is not verified"):
        api["capture_tensor"](
            articulation,
            config,
            replace(stage_snapshot, stage_usd_readback_verified=False),
        )

    with pytest.raises(error, match="stage snapshot profile mismatch"):
        api["capture_tensor"](
            articulation,
            config,
            replace(
                stage_snapshot,
                profile=replace(stage_snapshot.profile, id="tampered_v1"),
            ),
        )
