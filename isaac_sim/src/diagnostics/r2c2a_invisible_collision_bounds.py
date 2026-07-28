"""Stage 2.2-R2C2A collision-aware world-bound resolution.

The frozen Kujiale export has four enabled collision containers whose sole
geometry child is invisible.  Rendering visibility must not make an otherwise
unique physical geometry disappear from the no-motion envelope diagnostic.
This module is deliberately independent of USD/Kit so the fail-closed
contract can be tested with ordinary Python.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

from isaac_sim.src.diagnostics.r2c2_free_space_envelope import Bounds3D
from isaac_sim.src.yaml_utils import load_mapping, reject_unknown, require_keys


SCHEMA = "bio_nav_stage2_2_r2c2a_free_space_envelope_v1"
CONFIG_SCHEMA = "bio_nav_stage2_2_r2c2a_collision_bounds_config_v1"


@dataclass(frozen=True)
class FallbackMapping:
    collision_prim: str
    source_gprim: str


@dataclass(frozen=True)
class CollisionBoundsConfig:
    source_asset_name: str
    fallbacks: tuple[FallbackMapping, ...]

    def mapping_for(self, path: str) -> FallbackMapping | None:
        return next((item for item in self.fallbacks if item.collision_prim == path), None)


@dataclass(frozen=True)
class BoundsResolution:
    bounds: Bounds3D
    bounds_source: str
    source_gprim_paths: tuple[str, ...]
    effective_visibility: str
    fallback_reason: str | None
    collision_schema_noncanonical: bool

    @property
    def valid(self) -> bool:
        return self.bounds.finite()

    def trace_fields(self) -> dict[str, object]:
        return {
            "bounds_source": self.bounds_source,
            "source_gprim_paths": list(self.source_gprim_paths),
            "effective_visibility": self.effective_visibility,
            "fallback_reason": self.fallback_reason,
            "collision_schema_noncanonical": self.collision_schema_noncanonical,
        }


def _invalid(*, source_gprim_paths: Sequence[str], effective_visibility: str, reason: str, noncanonical: bool) -> BoundsResolution:
    return BoundsResolution(
        Bounds3D(float("nan"), float("nan"), float("nan"), float("nan"), float("nan"), float("nan")),
        "UNRESOLVED",
        tuple(source_gprim_paths),
        effective_visibility,
        reason,
        noncanonical,
    )


def load_collision_bounds_config(path: str | Path) -> CollisionBoundsConfig:
    value = load_mapping(path)
    reject_unknown(value, {"schema", "source_asset_name", "fallbacks"}, context="R2C2A collision bounds config")
    require_keys(value, {"schema", "source_asset_name", "fallbacks"}, context="R2C2A collision bounds config")
    if value["schema"] != CONFIG_SCHEMA:
        raise ValueError(f"R2C2A collision bounds schema must be {CONFIG_SCHEMA}")
    if not isinstance(value["source_asset_name"], str) or not value["source_asset_name"]:
        raise ValueError("R2C2A source_asset_name must be non-empty")
    raw_fallbacks = value["fallbacks"]
    if not isinstance(raw_fallbacks, list) or not raw_fallbacks:
        raise ValueError("R2C2A fallbacks must be a non-empty list")
    mappings: list[FallbackMapping] = []
    for index, item in enumerate(raw_fallbacks):
        if not isinstance(item, dict):
            raise ValueError(f"R2C2A fallback[{index}] must be a mapping")
        reject_unknown(item, {"collision_prim", "source_gprim"}, context=f"R2C2A fallback[{index}]")
        require_keys(item, {"collision_prim", "source_gprim"}, context=f"R2C2A fallback[{index}]")
        collision, gprim = item["collision_prim"], item["source_gprim"]
        if not isinstance(collision, str) or not collision.startswith("/"):
            raise ValueError(f"R2C2A fallback[{index}].collision_prim must be an absolute path")
        if not isinstance(gprim, str) or not gprim.startswith(f"{collision}/"):
            raise ValueError(f"R2C2A fallback[{index}].source_gprim must descend from collision_prim")
        mappings.append(FallbackMapping(collision, gprim))
    if len({item.collision_prim for item in mappings}) != len(mappings):
        raise ValueError("R2C2A fallback collision_prim paths must be unique")
    if len({item.source_gprim for item in mappings}) != len(mappings):
        raise ValueError("R2C2A fallback source_gprim paths must be unique")
    return CollisionBoundsConfig(str(value["source_asset_name"]), tuple(mappings))


def resolve_collision_bounds(
    *,
    path: str,
    primary_bounds: Bounds3D,
    fallback_bounds: Bounds3D,
    collision_enabled: bool,
    is_leaf_collision: bool,
    active_gprim_paths: Iterable[str],
    descendant_collision_paths: Iterable[str],
    descendants_finite: bool,
    effective_visibility: str,
    collision_schema_noncanonical: bool,
    config: CollisionBoundsConfig,
) -> BoundsResolution:
    """Resolve one enabled collision container without silently discarding it."""

    gprims = tuple(sorted(active_gprim_paths))
    nested_collisions = tuple(sorted(descendant_collision_paths))
    if primary_bounds.finite():
        return BoundsResolution(
            primary_bounds,
            "VISIBLE_WORLD_BBOX",
            gprims,
            effective_visibility,
            None,
            collision_schema_noncanonical,
        )
    mapping = config.mapping_for(path)
    if mapping is None:
        return _invalid(source_gprim_paths=gprims, effective_visibility=effective_visibility, reason="PRIMARY_BOUNDS_NONFINITE_UNKNOWN_FALLBACK", noncanonical=collision_schema_noncanonical)
    if not collision_enabled:
        return _invalid(source_gprim_paths=gprims, effective_visibility=effective_visibility, reason="FALLBACK_COLLISION_DISABLED", noncanonical=collision_schema_noncanonical)
    if not is_leaf_collision:
        return _invalid(source_gprim_paths=gprims, effective_visibility=effective_visibility, reason="FALLBACK_COLLISION_NOT_LEAF", noncanonical=collision_schema_noncanonical)
    if nested_collisions:
        return _invalid(source_gprim_paths=gprims, effective_visibility=effective_visibility, reason="FALLBACK_NESTED_COLLISION", noncanonical=collision_schema_noncanonical)
    if gprims != (mapping.source_gprim,):
        return _invalid(source_gprim_paths=gprims, effective_visibility=effective_visibility, reason="FALLBACK_SOURCE_GPRIM_MISMATCH", noncanonical=collision_schema_noncanonical)
    if not descendants_finite:
        return _invalid(source_gprim_paths=gprims, effective_visibility=effective_visibility, reason="FALLBACK_DESCENDANT_NONFINITE", noncanonical=collision_schema_noncanonical)
    if not fallback_bounds.finite():
        return _invalid(source_gprim_paths=gprims, effective_visibility=effective_visibility, reason="FALLBACK_BOUNDS_NONFINITE", noncanonical=collision_schema_noncanonical)
    return BoundsResolution(
        fallback_bounds,
        "INVISIBLE_COLLISION_SUBTREE_FALLBACK",
        gprims,
        effective_visibility,
        "PRIMARY_BOUNDS_NONFINITE",
        collision_schema_noncanonical,
    )
