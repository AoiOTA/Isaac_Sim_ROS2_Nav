from __future__ import annotations

from dataclasses import replace
import hashlib
from pathlib import Path

import pytest


try:
    import pxr  # noqa: F401
except ImportError:
    HAS_PXR = False
else:
    HAS_PXR = True
pytestmark = [
    pytest.mark.isaac,
    pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable"),
]

from isaac_sim.src.config import load_project_config  # noqa: E402
from isaac_sim.src.stage.contact_setup import apply_contact_profile  # noqa: E402
from isaac_sim.src.stage.ground_topology import (  # noqa: E402
    GroundTopologyError,
    apply_ground_topology,
    capture_ground_topology_snapshot,
)
from isaac_sim.src.stage.scene_composer import SceneComposer  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "isaac_sim/configs/ground_topologies"
PHYSICS = ROOT / "isaac_sim/configs/physics"
ASSET_ROOT = "/home/lyb/isaacsim_assets/Assets/Isaac/6.0"
MARKER = "isaac_nav_ground_topology_layer"


def _config(
    topology: str = "warehouse_combined32_v1.yaml",
    contact: str = "legacy_baseline.yaml",
):
    config = load_project_config(
        ROOT / "isaac_sim/configs/project.yaml",
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": ASSET_ROOT,
        },
    )
    return replace(
        config,
        files=replace(
            config.files,
            ground_topology_profile=PROFILES / topology,
            contact_profile=PHYSICS / contact,
        ),
    )


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _topology_layers(stage):
    from pxr import Sdf

    return [
        layer
        for identifier in stage.GetSessionLayer().subLayerPaths
        if (layer := Sdf.Layer.Find(identifier))
        and layer.customLayerData.get(MARKER) is True
    ]


def _authored_attributes(layer) -> set[str]:
    from pxr import Sdf

    paths: set[str] = set()

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.AttributeSpec):
            paths.add(str(path))
        if isinstance(spec, Sdf.PrimSpec):
            assert "active" not in spec.ListInfoKeys()

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)
    return paths


def test_combined32_profile_is_a_zero_behavior_anonymous_overlay():
    config = _config()
    watched = (
        config.environment.project_stage,
        config.environment.source_asset,
        config.robot.asset_path,
    )
    before = {path: _sha256(path) for path in watched}
    composer = SceneComposer(config)
    stage = composer.compose(save=False)
    snapshot = composer.ground_topology_snapshot

    assert snapshot is not None
    assert snapshot.profile_id == "warehouse_combined32_v1"
    assert snapshot.operation == "preserve_source_colliders"
    assert snapshot.source_collider_count == 32
    assert snapshot.target_colliders == snapshot.source_colliders
    assert snapshot.target_collider_count == 32
    assert snapshot.disabled_colliders == ()
    assert snapshot.disabled_collider_count == 0
    assert snapshot.source_collider_paths_sha256 == (
        "e616741f500b267739c9fe04eb9115e22796053ddec4c079a87baed39f7c13be"
    )
    assert snapshot.stage_usd_readback_verified is True
    assert snapshot.overlay_identifier.startswith("anon:")
    assert len(snapshot.overlay_sha256) == 64
    assert composer.contact_snapshot is not None
    assert composer.contact_snapshot.ground_colliders == snapshot.target_colliders

    layers = _topology_layers(stage)
    assert len(layers) == 1
    assert _authored_attributes(layers[0]) == set()
    assert {path: _sha256(path) for path in watched} == before


@pytest.mark.parametrize(
    "topology",
    (
        "warehouse_combined32_v1.yaml",
        "warehouse_plane_only1_v1.yaml",
    ),
)
def test_overlay_content_hash_is_stable_across_anonymous_layers(topology):
    config = _config(topology=topology)
    snapshots = []
    for _ in range(2):
        composer = SceneComposer(config)
        composer.compose(save=False)
        snapshots.append(composer.ground_topology_snapshot)

    assert all(snapshot is not None for snapshot in snapshots)
    first, second = snapshots
    assert first.overlay_identifier != second.overlay_identifier
    assert first.overlay_sha256 == second.overlay_sha256


def test_plane_only_profile_disables_exactly_31_decals_and_contact_uses_target():
    from pxr import Sdf, UsdPhysics, UsdShade

    config = _config(
        topology="warehouse_plane_only1_v1.yaml",
        contact="explicit_material.yaml",
    )
    source_before = _sha256(config.environment.source_asset)
    composer = SceneComposer(config)
    stage = composer.compose(save=False)
    snapshot = composer.ground_topology_snapshot
    contact = composer.contact_snapshot

    assert snapshot is not None
    assert contact is not None
    assert snapshot.operation == "disable_non_target_colliders"
    assert snapshot.source_collider_count == 32
    assert snapshot.target_colliders == ("/Root/GroundPlane/CollisionPlane",)
    assert snapshot.target_collider_count == 1
    assert snapshot.disabled_collider_count == 31
    assert set(snapshot.source_colliders) == (
        set(snapshot.target_colliders) | set(snapshot.disabled_colliders)
    )
    assert not set(snapshot.target_colliders) & set(snapshot.disabled_colliders)
    assert snapshot.disabled_collider_paths_sha256 == (
        "867b4f96daf93cfce58c7512d36626f91ca8a4cde3738756b71a980b842ad70f"
    )
    assert contact.ground_colliders == snapshot.target_colliders
    assert len(contact.ground_bindings) == 1

    layers = _topology_layers(stage)
    assert len(layers) == 1
    expected_attributes = {
        str(Sdf.Path(path).AppendProperty("physics:collisionEnabled"))
        for path in snapshot.disabled_colliders
    }
    assert _authored_attributes(layers[0]) == expected_attributes
    assert "active = false" not in layers[0].ExportToString()

    for path in snapshot.disabled_colliders:
        prim = stage.GetPrimAtPath(path)
        assert prim.IsActive()
        assert prim.HasAPI(UsdPhysics.CollisionAPI)
        assert UsdPhysics.CollisionAPI(
            prim
        ).GetCollisionEnabledAttr().Get() is False
        direct = UsdShade.MaterialBindingAPI(prim).GetDirectBinding(
            "physics"
        ).GetMaterialPath()
        assert direct.isEmpty
        assert stage.GetRootLayer().GetAttributeAtPath(
            Sdf.Path(path).AppendProperty("physics:collisionEnabled")
        ) is None
    assert _sha256(config.environment.source_asset) == source_before


def test_plane_only_overlay_is_reversible_and_keeps_one_unique_marker():
    from pxr import UsdPhysics

    plane_config = _config(topology="warehouse_plane_only1_v1.yaml")
    composer = SceneComposer(plane_config)
    stage = composer.compose(save=False)
    plane = composer.ground_topology_snapshot
    assert plane is not None
    assert all(
        UsdPhysics.CollisionAPI(
            stage.GetPrimAtPath(path)
        ).GetCollisionEnabledAttr().Get()
        is False
        for path in plane.disabled_colliders
    )

    combined_config = _config()
    combined = apply_ground_topology(stage, combined_config)
    assert combined.target_collider_count == 32
    assert combined.disabled_colliders == ()
    assert all(
        UsdPhysics.CollisionAPI(
            stage.GetPrimAtPath(path)
        ).GetCollisionEnabledAttr().Get()
        is True
        for path in plane.disabled_colliders
    )
    assert len(_topology_layers(stage)) == 1

    contact = apply_contact_profile(stage, combined_config)
    assert contact.ground_colliders == combined.target_colliders
    assert capture_ground_topology_snapshot(stage, combined_config) == combined


def test_duplicate_topology_marker_fails_readback():
    from pxr import Sdf

    config = _config()
    stage = SceneComposer(config).compose(save=False)
    duplicate = Sdf.Layer.CreateAnonymous("duplicate_ground_topology.usda")
    duplicate.customLayerData = {MARKER: True}
    stage.GetSessionLayer().subLayerPaths.insert(0, duplicate.identifier)

    with pytest.raises(GroundTopologyError, match="expected one active.*found 2"):
        capture_ground_topology_snapshot(stage, config)


def test_topology_readback_rejects_extra_prim_opinions():
    from pxr import Sdf

    config = _config(topology="warehouse_plane_only1_v1.yaml")
    stage = SceneComposer(config).compose(save=False)
    layer = _topology_layers(stage)[0]
    root_spec = layer.GetPrimAtPath("/Root")
    root_spec.SetInfo("documentation", "unexpected topology opinion")

    with pytest.raises(GroundTopologyError, match="authored opinions outside"):
        capture_ground_topology_snapshot(stage, config)

    root_spec.ClearInfo("documentation")
    Sdf.CreatePrimInLayer(layer, "/Unexpected")
    with pytest.raises(GroundTopologyError, match="authored opinions outside"):
        capture_ground_topology_snapshot(stage, config)


def test_source_asset_sha_mismatch_fails_closed_and_removes_old_overlay(
    tmp_path,
):
    config = _config()
    stage = SceneComposer(config).compose(save=False)
    source = (
        PROFILES / "warehouse_plane_only1_v1.yaml"
    ).read_text()
    invalid_profile = tmp_path / "invalid_source_sha.yaml"
    invalid_profile.write_text(
        source.replace(
            "eb27e4bad446bba981b81c27529a284dfb55a30adc01eaadb80bb0fe997a0dc0",
            "a" * 64,
            1,
        )
    )
    invalid = replace(
        config,
        files=replace(
            config.files,
            ground_topology_profile=invalid_profile,
        ),
    )

    with pytest.raises(GroundTopologyError, match="source asset SHA256 mismatch"):
        apply_ground_topology(stage, invalid)
    assert _topology_layers(stage) == []


def test_failure_after_authoring_restores_edit_target_and_removes_overlay(
    monkeypatch,
):
    from pxr import UsdPhysics

    import isaac_sim.src.stage.ground_topology as module

    combined_config = _config()
    stage = SceneComposer(combined_config).compose(save=False)
    original_target = stage.GetEditTarget()
    plane_config = _config(topology="warehouse_plane_only1_v1.yaml")

    def fail_readback(_stage, _config):
        raise GroundTopologyError("injected readback failure")

    monkeypatch.setattr(module, "capture_ground_topology_snapshot", fail_readback)
    with pytest.raises(GroundTopologyError, match="injected readback failure"):
        apply_ground_topology(stage, plane_config)

    assert stage.GetEditTarget() == original_target
    assert _topology_layers(stage) == []
    source = module.load_ground_topology_profile(
        plane_config.files.ground_topology_profile
    )
    _all, _target, disabled = module._load_source_sets(plane_config, source)
    assert all(
        UsdPhysics.CollisionAPI(
            stage.GetPrimAtPath(path)
        ).GetCollisionEnabledAttr().Get()
        is True
        for path in disabled
    )
