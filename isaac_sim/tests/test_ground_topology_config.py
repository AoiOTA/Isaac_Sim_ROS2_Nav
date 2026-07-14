from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from isaac_sim.src.config import load_project_config
from isaac_sim.src.stage.ground_topology import (
    GroundTopologyError,
    collider_paths_sha256,
    load_ground_topology_profile,
)


ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "isaac_sim/configs/ground_topologies"


def test_profile_catalog_locks_all_three_versioned_topologies():
    assert {path.name for path in PROFILES.glob("*.yaml")} == {
        "warehouse_combined32_v1.yaml",
        "warehouse_plane_only1_v1.yaml",
        "simple_plane_only1_v1.yaml",
    }

    combined = load_ground_topology_profile(
        PROFILES / "warehouse_combined32_v1.yaml"
    )
    plane_only = load_ground_topology_profile(
        PROFILES / "warehouse_plane_only1_v1.yaml"
    )
    simple = load_ground_topology_profile(
        PROFILES / "simple_plane_only1_v1.yaml"
    )

    assert combined.identifier == "warehouse_combined32_v1"
    assert combined.environment_id == "Warehouse"
    assert combined.operation == "preserve_source_colliders"
    assert combined.source.collider_count == 32
    assert combined.target.collider_count == 32
    assert combined.disabled.collider_count == 0

    assert plane_only.identifier == "warehouse_plane_only1_v1"
    assert plane_only.operation == "disable_non_target_colliders"
    assert plane_only.source == combined.source
    assert plane_only.target.collider_count == 1
    assert plane_only.disabled.collider_count == 31

    assert simple.identifier == "simple_plane_only1_v1"
    assert simple.environment_id == "SimplePlane"
    assert simple.operation == "preserve_source_colliders"
    assert simple.source.collider_count == 1
    assert simple.source == simple.target
    assert simple.disabled.collider_count == 0


def test_collider_path_hash_uses_frozen_canonical_json_encoding():
    plane = "/Root/GroundPlane/CollisionPlane"
    assert collider_paths_sha256(()) == (
        "4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945"
    )
    assert collider_paths_sha256((plane,)) == (
        "093b0b40e3e87c6102b5e60ab009a27b36b45428ac4e61f424ea89d054448e3f"
    )
    assert collider_paths_sha256(("/Z", "/A")) == collider_paths_sha256(
        ("/A", "/Z")
    )
    with pytest.raises(GroundTopologyError, match="duplicates"):
        collider_paths_sha256((plane, plane))


@pytest.mark.parametrize(
    ("old", "new", "message"),
    [
        ("schema_version: 1", "schema_version: true", "schema_version"),
        ("operation: preserve_source_colliders", "operation: typo", "operation"),
        (
            "collider_count: 32",
            "collider_count: 31",
            "source count must equal target plus disabled",
        ),
        (
            "asset_sha256: eb27e4bad446bba981b81c27529a284dfb55a30adc01eaadb80bb0fe997a0dc0",
            "asset_sha256: INVALID",
            "lowercase SHA256",
        ),
    ],
)
def test_profile_parser_rejects_wrong_types_and_invariants(
    tmp_path,
    old,
    new,
    message,
):
    source = (PROFILES / "warehouse_combined32_v1.yaml").read_text()
    candidate = tmp_path / "invalid.yaml"
    candidate.write_text(source.replace(old, new, 1))
    with pytest.raises(GroundTopologyError, match=message):
        load_ground_topology_profile(candidate)


def test_profile_parser_rejects_unknown_and_duplicate_keys(tmp_path):
    source = (PROFILES / "simple_plane_only1_v1.yaml").read_text()
    unknown = tmp_path / "unknown.yaml"
    unknown.write_text(source + "unknown: true\n")
    with pytest.raises(GroundTopologyError, match="unknown.*keys"):
        load_ground_topology_profile(unknown)

    duplicate = tmp_path / "duplicate.yaml"
    duplicate.write_text(source + "id: duplicate\n")
    with pytest.raises(GroundTopologyError, match="duplicate YAML mapping key"):
        load_ground_topology_profile(duplicate)


def test_project_configs_select_matching_stable_topology_profiles():
    environment = {
        "PROJECT_ROOT": str(ROOT),
        "ISAAC_ASSET_ROOT": "/home/lyb/isaacsim_assets/Assets/Isaac/6.0",
    }
    warehouse = load_project_config(
        ROOT / "isaac_sim/configs/project.yaml",
        environment,
    )
    simple = load_project_config(
        ROOT / "isaac_sim/configs/simple_plane.project.yaml",
        environment,
    )
    assert warehouse.files.ground_topology_profile == (
        PROFILES / "warehouse_combined32_v1.yaml"
    )
    assert simple.files.ground_topology_profile == (
        PROFILES / "simple_plane_only1_v1.yaml"
    )
    assert replace(
        warehouse.files,
        ground_topology_profile=PROFILES / "warehouse_plane_only1_v1.yaml",
    ).ground_topology_profile.name == "warehouse_plane_only1_v1.yaml"
