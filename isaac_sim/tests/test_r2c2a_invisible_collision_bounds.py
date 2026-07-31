from __future__ import annotations

from pathlib import Path

import pytest

from isaac_sim.src.diagnostics.r2c2_free_space_envelope import Bounds3D
from isaac_sim.src.diagnostics.r2c2a_invisible_collision_bounds import (
    CollisionBoundsConfig,
    FallbackMapping,
    load_collision_bounds_config,
    resolve_collision_bounds,
)


PATH = "/Root/door_handle"
GPRIM = f"{PATH}/mesh"
FINITE = Bounds3D(0.0, 0.0, 0.0, 1.0, 1.0, 1.0)
NONFINITE = Bounds3D(float("nan"), 0.0, 0.0, 1.0, 1.0, 1.0)


def _config() -> CollisionBoundsConfig:
    return CollisionBoundsConfig("kujiale.usd", (FallbackMapping(PATH, GPRIM),))


def _resolve(**overrides):
    values = {
        "path": PATH,
        "primary_bounds": NONFINITE,
        "fallback_bounds": FINITE,
        "collision_enabled": True,
        "is_leaf_collision": True,
        "active_gprim_paths": [GPRIM],
        "descendant_collision_paths": [],
        "descendants_finite": True,
        "effective_visibility": "invisible",
        "collision_schema_noncanonical": True,
        "config": _config(),
    }
    values.update(overrides)
    return resolve_collision_bounds(**values)


def test_visible_primary_never_uses_fallback() -> None:
    result = _resolve(primary_bounds=FINITE, path="/Root/ordinary")
    assert result.valid
    assert result.bounds_source == "VISIBLE_WORLD_BBOX"
    assert result.fallback_reason is None


def test_unique_invisible_descendant_uses_frozen_fallback() -> None:
    result = _resolve()
    assert result.valid
    assert result.bounds_source == "INVISIBLE_COLLISION_SUBTREE_FALLBACK"
    assert result.source_gprim_paths == (GPRIM,)
    assert result.fallback_reason == "PRIMARY_BOUNDS_NONFINITE"
    assert result.collision_schema_noncanonical


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"path": "/Root/unknown"}, "PRIMARY_BOUNDS_NONFINITE_UNKNOWN_FALLBACK"),
        ({"collision_enabled": False}, "FALLBACK_COLLISION_DISABLED"),
        ({"is_leaf_collision": False}, "FALLBACK_COLLISION_NOT_LEAF"),
        ({"descendant_collision_paths": ["/Root/door_handle/nested"]}, "FALLBACK_NESTED_COLLISION"),
        ({"active_gprim_paths": []}, "FALLBACK_SOURCE_GPRIM_MISMATCH"),
        ({"active_gprim_paths": [GPRIM, f"{PATH}/other"]}, "FALLBACK_SOURCE_GPRIM_MISMATCH"),
        ({"descendants_finite": False}, "FALLBACK_DESCENDANT_NONFINITE"),
        ({"fallback_bounds": NONFINITE}, "FALLBACK_BOUNDS_NONFINITE"),
    ],
)
def test_invalid_fallbacks_fail_closed(overrides, reason: str) -> None:
    result = _resolve(**overrides)
    assert not result.valid
    assert result.bounds_source == "UNRESOLVED"
    assert result.fallback_reason == reason


def test_config_requires_absolute_unique_descendant_mapping(tmp_path: Path) -> None:
    config = tmp_path / "bounds.yaml"
    config.write_text(
        """schema: bio_nav_stage2_2_r2c2a_collision_bounds_config_v1
source_asset_name: kujiale.usd
fallbacks:
  - collision_prim: /Root/a
    source_gprim: /Root/a/mesh
""",
        encoding="utf-8",
    )
    parsed = load_collision_bounds_config(config)
    assert parsed.mapping_for("/Root/a") == FallbackMapping("/Root/a", "/Root/a/mesh")

    config.write_text(
        """schema: bio_nav_stage2_2_r2c2a_collision_bounds_config_v1
source_asset_name: kujiale.usd
fallbacks:
  - collision_prim: /Root/a
    source_gprim: /Root/not-a-descendant
""",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="descend"):
        load_collision_bounds_config(config)
