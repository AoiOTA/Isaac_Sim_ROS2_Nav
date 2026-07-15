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
from isaac_sim.src.stage.contact_setup import (  # noqa: E402
    ContactSetupError,
    apply_contact_profile,
    capture_contact_profile_snapshot,
    load_contact_profile,
)
from isaac_sim.src.stage.physics_setup import find_all_physics_scenes  # noqa: E402
from isaac_sim.src.stage.ground_topology import GroundTopologyError  # noqa: E402
from isaac_sim.src.stage.scene_composer import SceneComposer  # noqa: E402


ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = "/home/lyb/isaacsim_assets/Assets/Isaac/6.0"
PHYSICS = ROOT / "isaac_sim/configs/physics"


def _config(
    project: str = "project.yaml",
    profile: str | None = None,
):
    config = load_project_config(
        ROOT / "isaac_sim/configs" / project,
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": ASSET_ROOT,
        },
    )
    if profile is not None:
        config = replace(
            config,
            files=replace(config.files, contact_profile=PHYSICS / profile),
        )
    return config


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_profile_catalog_locks_threshold_matrix_and_explicit_material_values():
    expected_thresholds = {
        "threshold_corr_0p00025_offset_0p0004.yaml": (0.00025, 0.0004),
        "threshold_corr_0p025_offset_0p0004.yaml": (0.025, 0.0004),
        "threshold_corr_0p00025_offset_0p04.yaml": (0.00025, 0.04),
        "threshold_corr_0p025_offset_0p04.yaml": (0.025, 0.04),
    }
    assert {path.name for path in PHYSICS.glob("*.yaml")} == {
        "legacy_baseline.yaml",
        "explicit_material.yaml",
        *expected_thresholds,
    }
    legacy = load_contact_profile(PHYSICS / "legacy_baseline.yaml")
    assert legacy.mode == "legacy_baseline"
    assert legacy.scene is None
    assert legacy.wheel_material is None
    assert legacy.ground_material is None

    for name, (correlation, offset) in expected_thresholds.items():
        profile = load_contact_profile(PHYSICS / name)
        assert profile.mode == "threshold_only"
        assert profile.scene is not None
        assert profile.scene.friction_correlation_distance == pytest.approx(
            correlation
        )
        assert profile.scene.friction_offset_threshold == pytest.approx(offset)
        assert profile.wheel_material is None
        assert profile.ground_material is None

    explicit = load_contact_profile(PHYSICS / "explicit_material.yaml")
    assert explicit.mode == "explicit_material"
    assert explicit.wheel_material is not None
    assert explicit.ground_material is not None
    assert explicit.wheel_material.static_friction == pytest.approx(0.2)
    assert explicit.wheel_material.dynamic_friction == pytest.approx(0.2)
    assert explicit.ground_material.static_friction == pytest.approx(0.5)
    assert explicit.ground_material.dynamic_friction == pytest.approx(0.5)
    assert explicit.wheel_material.friction_combine_mode == "average"
    assert explicit.ground_material.friction_combine_mode == "average"


def test_profile_parser_rejects_unknown_keys(tmp_path):
    candidate = tmp_path / "invalid.yaml"
    candidate.write_text(
        "schema_version: 1\n"
        "id: invalid\n"
        "mode: legacy_baseline\n"
        "typo: true\n",
        encoding="utf-8",
    )
    with pytest.raises(ContactSetupError, match="unknown contact_profile keys"):
        load_contact_profile(candidate)


@pytest.mark.parametrize(
    ("schema_version", "mode", "message"),
    [
        ("true", "legacy_baseline", "schema_version"),
        ("1.0", "legacy_baseline", "schema_version"),
        ("1", "[legacy_baseline]", "contact_profile.mode"),
    ],
)
def test_profile_parser_rejects_wrong_scalar_types(
    tmp_path,
    schema_version,
    mode,
    message,
):
    candidate = tmp_path / "invalid-types.yaml"
    candidate.write_text(
        f"schema_version: {schema_version}\n"
        "id: invalid_types\n"
        f"mode: {mode}\n",
        encoding="utf-8",
    )
    with pytest.raises(ContactSetupError, match=message):
        load_contact_profile(candidate)


def test_default_legacy_profile_preserves_effective_physics_and_assets():
    config = _config()
    watched = (
        config.environment.project_stage,
        config.environment.source_asset,
        config.robot.asset_path,
    )
    before = {path: _sha256(path) for path in watched}
    composer = SceneComposer(config)
    stage = composer.compose(save=False)
    snapshot = composer.contact_snapshot
    assert snapshot is not None

    assert snapshot.profile_id == "legacy_baseline"
    assert snapshot.profile_mode == "legacy_baseline"
    assert snapshot.explicit_materials is False
    assert snapshot.thresholds_authored is False
    assert snapshot.stage_usd_readback_verified is True
    assert snapshot.overlay_identifier.startswith("anon:")
    assert len(snapshot.overlay_sha256) == 64
    assert snapshot.scene.friction_correlation_distance == pytest.approx(0.00025)
    assert snapshot.scene.friction_offset_threshold == pytest.approx(0.0004)
    # The Warehouse asset relies on the PhysX schema fallback for friction
    # type, so pure USD readback correctly reports no authored token.
    assert snapshot.scene.friction_type is None

    assert len(snapshot.wheel_colliders) == 4
    assert len(set(snapshot.wheel_colliders)) == 4
    assert all(path.endswith("/collisions_v2") for path in snapshot.wheel_colliders)
    assert len(snapshot.ground_colliders) == 32
    assert "/Root/GroundPlane/CollisionPlane" in snapshot.ground_colliders
    assert {item.direct_physics_material_path for item in snapshot.wheel_bindings} == {
        None
    }
    assert {
        item.effective_physics_material_path for item in snapshot.wheel_bindings
    } == {f"{config.robot.runtime_prim_path}/PhysicsMaterials/wheels"}
    assert {item.direct_physics_material_path for item in snapshot.ground_bindings} == {
        None
    }
    assert {
        item.effective_physics_material_path for item in snapshot.ground_bindings
    } == {None}
    assert snapshot.wheel_material is not None
    assert snapshot.wheel_material.static_friction == pytest.approx(0.2)
    assert snapshot.wheel_material.dynamic_friction == pytest.approx(0.2)
    assert snapshot.wheel_material.restitution == pytest.approx(0.0)
    assert snapshot.ground_material is None
    assert capture_contact_profile_snapshot(stage, config) == snapshot
    assert {path: _sha256(path) for path in watched} == before


@pytest.mark.parametrize(
    ("profile", "correlation", "offset"),
    [
        ("threshold_corr_0p00025_offset_0p0004.yaml", 0.00025, 0.0004),
        ("threshold_corr_0p025_offset_0p0004.yaml", 0.025, 0.0004),
        ("threshold_corr_0p00025_offset_0p04.yaml", 0.00025, 0.04),
        ("threshold_corr_0p025_offset_0p04.yaml", 0.025, 0.04),
    ],
)
def test_threshold_profiles_author_all_four_scene_combinations(
    profile,
    correlation,
    offset,
):
    config = _config(profile=profile)
    composer = SceneComposer(config)
    composer.compose(save=False)
    snapshot = composer.contact_snapshot
    assert snapshot is not None
    assert snapshot.profile_mode == "threshold_only"
    assert snapshot.thresholds_authored is True
    assert snapshot.explicit_materials is False
    assert snapshot.scene.friction_correlation_distance == pytest.approx(correlation)
    assert snapshot.scene.friction_offset_threshold == pytest.approx(offset)
    assert {item.direct_physics_material_path for item in snapshot.wheel_bindings} == {
        None
    }
    assert {item.direct_physics_material_path for item in snapshot.ground_bindings} == {
        None
    }
    assert snapshot.wheel_material is not None
    assert snapshot.wheel_material.static_friction == pytest.approx(0.2)
    assert snapshot.ground_material is None


def test_explicit_profile_binds_exact_colliders_in_anonymous_session_layer():
    config = _config(profile="explicit_material.yaml")
    watched = (
        config.environment.project_stage,
        config.environment.source_asset,
        config.robot.asset_path,
    )
    before = {path: _sha256(path) for path in watched}
    composer = SceneComposer(config)
    stage = composer.compose(save=False)
    snapshot = composer.contact_snapshot
    assert snapshot is not None

    wheel_path = "/World/PhysicsMaterials/ContactProfile/Wheel"
    ground_path = "/World/PhysicsMaterials/ContactProfile/Ground"
    assert snapshot.profile_mode == "explicit_material"
    assert snapshot.explicit_materials is True
    assert len(snapshot.wheel_bindings) == 4
    assert len(snapshot.ground_bindings) == 32
    assert {item.direct_physics_material_path for item in snapshot.wheel_bindings} == {
        wheel_path
    }
    assert {
        item.effective_physics_material_path for item in snapshot.wheel_bindings
    } == {wheel_path}
    assert {item.direct_physics_material_path for item in snapshot.ground_bindings} == {
        ground_path
    }
    assert {
        item.effective_physics_material_path for item in snapshot.ground_bindings
    } == {ground_path}

    assert snapshot.wheel_material is not None
    assert snapshot.ground_material is not None
    assert snapshot.wheel_material.static_friction == pytest.approx(0.2)
    assert snapshot.wheel_material.dynamic_friction == pytest.approx(0.2)
    assert snapshot.ground_material.static_friction == pytest.approx(0.5)
    assert snapshot.ground_material.dynamic_friction == pytest.approx(0.5)
    for material in (snapshot.wheel_material, snapshot.ground_material):
        assert material.friction_combine_mode == "average"
        assert material.restitution_combine_mode == "average"
        assert material.friction_combine_mode_authored is True
        assert material.restitution_combine_mode_authored is True

    assert stage.GetRootLayer().GetPrimAtPath(wheel_path) is None
    assert stage.GetRootLayer().GetPrimAtPath(ground_path) is None
    assert {path: _sha256(path) for path in watched} == before

    second = apply_contact_profile(stage, config)
    assert second.overlay_sha256 == snapshot.overlay_sha256
    from pxr import Sdf

    contact_layers = [
        layer
        for identifier in stage.GetSessionLayer().subLayerPaths
        if (layer := Sdf.Layer.Find(identifier))
        and layer.customLayerData.get("isaac_nav_contact_profile_layer") is True
    ]
    assert len(contact_layers) == 1
    assert capture_contact_profile_snapshot(stage, config) == second


def test_simple_plane_composition_is_isolated_from_warehouse():
    warehouse = _config()
    simple = _config(project="simple_plane.project.yaml")
    assert simple.environment.project_stage != warehouse.environment.project_stage
    before = _sha256(simple.environment.project_stage)
    composer = SceneComposer(simple)
    stage = composer.compose(save=False)
    snapshot = composer.contact_snapshot
    assert snapshot is not None

    sublayers = {
        str(Path(path).resolve()) for path in stage.GetRootLayer().subLayerPaths
    }
    assert sublayers == {str(simple.environment.source_asset)}
    assert str(warehouse.environment.source_asset) not in sublayers
    assert snapshot.ground_colliders == ("/Root/GroundPlane/CollisionPlane",)
    assert len(snapshot.wheel_colliders) == 4
    scenes = find_all_physics_scenes(stage)
    assert [str(scene.GetPath()) for scene in scenes] == [
        simple.simulation.expected_physics_scene
    ]
    assert _sha256(simple.environment.project_stage) == before


def test_scene_composer_rejects_stale_environment_sublayers(tmp_path):
    from pxr import Usd, UsdGeom

    warehouse = _config()
    simple = _config(project="simple_plane.project.yaml")
    project_stage = tmp_path / "contaminated_simple_plane.usda"
    stage = Usd.Stage.CreateNew(str(project_stage))
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    stage.GetRootLayer().subLayerPaths.append(
        str(warehouse.environment.source_asset)
    )
    assert stage.GetRootLayer().Save()
    contaminated = replace(
        simple,
        environment=replace(
            simple.environment,
            project_stage=project_stage,
        ),
    )
    with pytest.raises(RuntimeError, match="exactly the selected environment"):
        SceneComposer(contaminated).compose(save=False)


def test_ground_source_resolver_count_mismatch_fails_closed():
    config = _config()
    invalid = replace(
        config,
        environment=replace(
            config.environment,
            ground_colliders=replace(
                config.environment.ground_colliders,
                expected_enabled_count=31,
            ),
        ),
    )
    with pytest.raises(GroundTopologyError, match="source resolver mismatch"):
        SceneComposer(invalid).compose(save=False)


def test_ground_source_resolver_requires_exact_semantic_classes():
    config = _config()
    invalid = replace(
        config,
        environment=replace(
            config.environment,
            ground_colliders=replace(
                config.environment.ground_colliders,
                semantic_classes=("floor_decal", "typo"),
                expected_enabled_count=32,
            ),
        ),
    )
    with pytest.raises(GroundTopologyError, match="source resolver mismatch"):
        SceneComposer(invalid).compose(save=False)
