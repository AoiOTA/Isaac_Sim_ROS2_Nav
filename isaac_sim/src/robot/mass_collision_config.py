"""Strict mass/collision profiles for the composed Jackal articulation.

This module is intentionally free of Isaac and NumPy imports.  It validates
the immutable configuration contract before runtime code authors or reads any
USD/PhysX state.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
from typing import Any, Mapping

from isaac_sim.src.yaml_utils import (
    YamlConfigError,
    load_mapping,
    reject_unknown,
    require_keys,
    require_number,
    require_vector,
)


_PROFILE_FIELDS = frozenset(
    {
        "schema_version",
        "profile_id",
        "mode",
        "robot_asset_sha256",
        "base_prim_suffix",
        "sensor_shells",
        "base_inertial",
        "expected_link_masses",
        "expected_total_mass_kg",
    }
)
_SENSOR_SHELL_FIELDS = frozenset(
    {"prim_suffix", "active", "collision_enabled"}
)
_BASE_INERTIAL_FIELDS = frozenset(
    {"mass_kg", "center_of_mass_m", "inertia_kg_m2"}
)
_PROFILE_MODES = {
    "legacy_default_sensor_density_v1": "legacy_default_sensor_density",
    "sensor_shells_disabled_v1": "sensor_shells_disabled",
    "fixed_base_inertial_sensor_shell_collision_v1": (
        "fixed_base_inertial_sensor_shell_collision"
    ),
}
_BASE_PRIM_SUFFIX = "/base_link"
_SENSOR_SHELL_SUFFIXES = (
    "/base_link/collisions/bumblebee_camera",
    "/base_link/collisions/sick_lms1xx_lidar",
)
_LINK_SUFFIXES = (
    "/base_link",
    "/front_left_wheel_link",
    "/front_right_wheel_link",
    "/rear_left_wheel_link",
    "/rear_right_wheel_link",
)
_EXPECTED_SHELL_STATE = {
    "legacy_default_sensor_density": (True, True),
    "sensor_shells_disabled": (False, False),
    "fixed_base_inertial_sensor_shell_collision": (True, True),
}
_EXPECTED_BASE_MASS_KG = {
    "legacy_default_sensor_density": 18.3179048,
    "sensor_shells_disabled": 17.0,
    "fixed_base_inertial_sensor_shell_collision": 17.0,
}
_EXPECTED_TOTAL_MASS_KG = {
    "legacy_default_sensor_density": 20.2259048,
    "sensor_shells_disabled": 18.908,
    "fixed_base_inertial_sensor_shell_collision": 18.908,
}
_FIXED_CENTER_OF_MASS_M = (0.00504, 0.00062, 0.104328)
_FIXED_INERTIA_KG_M2 = (
    (0.18669458, -0.00005312, -0.00105626),
    (-0.00005312, 0.30087811, -0.00012994),
    (-0.00105626, -0.00012994, 0.38648003),
)
_SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")
_PRIM_PATH_PATTERN = re.compile(
    r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*"
)


@dataclass(frozen=True)
class SensorShellExpectation:
    prim_suffix: str
    active: bool
    collision_enabled: bool


@dataclass(frozen=True)
class BaseInertial:
    mass_kg: float
    center_of_mass_m: tuple[float, float, float]
    inertia_kg_m2: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]


@dataclass(frozen=True)
class LinkMassExpectation:
    prim_suffix: str
    mass_kg: float


@dataclass(frozen=True)
class MassCollisionProfile:
    schema_version: int
    profile_id: str
    mode: str
    robot_asset_sha256: str
    base_prim_suffix: str
    sensor_shells: tuple[SensorShellExpectation, SensorShellExpectation]
    base_inertial: BaseInertial | None
    expected_link_masses: tuple[
        LinkMassExpectation,
        LinkMassExpectation,
        LinkMassExpectation,
        LinkMassExpectation,
        LinkMassExpectation,
    ]
    expected_total_mass_kg: float


def load_mass_collision_profile(path: str | Path) -> MassCollisionProfile:
    """Load one schema-v1 mass/collision profile from strict YAML."""

    return parse_mass_collision_profile(load_mapping(path))


def parse_mass_collision_profile(
    data: Mapping[str, Any],
) -> MassCollisionProfile:
    """Validate a loaded schema-v1 mass/collision profile mapping."""

    if not isinstance(data, dict):
        raise YamlConfigError("mass collision profile must be a mapping")
    reject_unknown(data, _PROFILE_FIELDS, context="mass collision profile")
    require_keys(data, _PROFILE_FIELDS, context="mass collision profile")

    schema_version = data["schema_version"]
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 1
    ):
        raise YamlConfigError(
            "mass collision profile schema_version must be integer 1"
        )

    profile_id = data["profile_id"]
    mode = data["mode"]
    expected_mode = (
        _PROFILE_MODES.get(profile_id) if isinstance(profile_id, str) else None
    )
    if (
        not isinstance(mode, str)
        or expected_mode is None
        or mode != expected_mode
    ):
        raise YamlConfigError(
            "mass collision profile profile_id and mode must be one of the "
            "closed schema-v1 pairs"
        )

    robot_asset_sha256 = data["robot_asset_sha256"]
    if (
        not isinstance(robot_asset_sha256, str)
        or _SHA256_PATTERN.fullmatch(robot_asset_sha256) is None
    ):
        raise YamlConfigError(
            "robot_asset_sha256 must be 64 lowercase hexadecimal characters"
        )

    base_prim_suffix = data["base_prim_suffix"]
    if base_prim_suffix != _BASE_PRIM_SUFFIX:
        raise YamlConfigError(
            "base_prim_suffix must be the exact schema-v1 suffix /base_link"
        )

    sensor_shells = _parse_sensor_shells(data["sensor_shells"], mode)
    base_inertial = _parse_base_inertial(data["base_inertial"], mode)
    if base_inertial is not None and not _base_inertial_matches_audit(
        base_inertial
    ):
        raise YamlConfigError(
            "fixed base_inertial must match the audited v1 contract"
        )
    expected_link_masses = _parse_expected_link_masses(
        data["expected_link_masses"]
    )
    expected_total_mass_kg = require_number(
        data["expected_total_mass_kg"],
        context="expected_total_mass_kg",
        positive=True,
    )

    calculated_total_mass_kg = sum(
        expectation.mass_kg for expectation in expected_link_masses
    )
    if not math.isclose(
        expected_total_mass_kg,
        calculated_total_mass_kg,
        rel_tol=1e-9,
        abs_tol=1e-9,
    ):
        raise YamlConfigError(
            "expected_total_mass_kg must equal the sum of "
            "expected_link_masses"
        )

    masses_by_suffix = {
        expectation.prim_suffix: expectation.mass_kg
        for expectation in expected_link_masses
    }
    expected_base_mass_kg = _EXPECTED_BASE_MASS_KG[mode]
    if not math.isclose(
        masses_by_suffix[_BASE_PRIM_SUFFIX],
        expected_base_mass_kg,
        rel_tol=1e-9,
        abs_tol=1e-9,
    ) or any(
        not math.isclose(
            masses_by_suffix[suffix], 0.477, rel_tol=1e-9, abs_tol=1e-9
        )
        for suffix in _LINK_SUFFIXES[1:]
    ):
        raise YamlConfigError(
            f"expected_link_masses must match the audited {profile_id} contract"
        )
    if not math.isclose(
        expected_total_mass_kg,
        _EXPECTED_TOTAL_MASS_KG[mode],
        rel_tol=1e-9,
        abs_tol=1e-9,
    ):
        raise YamlConfigError(
            f"expected_total_mass_kg must match the audited {profile_id} contract"
        )

    return MassCollisionProfile(
        schema_version=schema_version,
        profile_id=profile_id,
        mode=mode,
        robot_asset_sha256=robot_asset_sha256,
        base_prim_suffix=base_prim_suffix,
        sensor_shells=sensor_shells,
        base_inertial=base_inertial,
        expected_link_masses=expected_link_masses,
        expected_total_mass_kg=expected_total_mass_kg,
    )


def resolve_prim_suffix(articulation_root: str, prim_suffix: str) -> str:
    """Safely join a validated absolute prim suffix to an articulation root."""

    _require_prim_path(articulation_root, context="articulation_root")
    _require_prim_path(prim_suffix, context="prim_suffix")
    return articulation_root + prim_suffix


def _require_prim_path(value: Any, *, context: str) -> str:
    if (
        not isinstance(value, str)
        or _PRIM_PATH_PATTERN.fullmatch(value) is None
    ):
        raise YamlConfigError(
            f"{context} must be an absolute canonical USD prim path"
        )
    return value


def _parse_sensor_shells(
    raw_shells: Any, mode: str
) -> tuple[SensorShellExpectation, SensorShellExpectation]:
    if not isinstance(raw_shells, list) or len(raw_shells) != 2:
        raise YamlConfigError(
            "sensor_shells must be a two-element list with the exact "
            "schema-v1 sensor shell suffixes"
        )

    parsed_by_suffix: dict[str, SensorShellExpectation] = {}
    for index, raw_shell in enumerate(raw_shells):
        context = f"sensor_shells[{index}]"
        if not isinstance(raw_shell, dict):
            raise YamlConfigError(f"{context} must be a mapping")
        reject_unknown(raw_shell, _SENSOR_SHELL_FIELDS, context=context)
        require_keys(raw_shell, _SENSOR_SHELL_FIELDS, context=context)
        suffix = raw_shell["prim_suffix"]
        if not isinstance(suffix, str):
            raise YamlConfigError(f"{context}.prim_suffix must be a string")
        active = raw_shell["active"]
        collision_enabled = raw_shell["collision_enabled"]
        if not isinstance(active, bool):
            raise YamlConfigError(f"{context}.active must be boolean")
        if not isinstance(collision_enabled, bool):
            raise YamlConfigError(
                f"{context}.collision_enabled must be boolean"
            )
        if suffix in parsed_by_suffix:
            raise YamlConfigError(
                f"duplicate sensor_shell prim_suffix: {suffix}"
            )
        parsed_by_suffix[suffix] = SensorShellExpectation(
            prim_suffix=suffix,
            active=active,
            collision_enabled=collision_enabled,
        )

    if set(parsed_by_suffix) != set(_SENSOR_SHELL_SUFFIXES):
        raise YamlConfigError(
            "sensor_shells must contain the exact schema-v1 sensor shell "
            "suffixes"
        )
    expected_active, expected_collision_enabled = _EXPECTED_SHELL_STATE[mode]
    if any(
        shell.active is not expected_active
        or shell.collision_enabled is not expected_collision_enabled
        for shell in parsed_by_suffix.values()
    ):
        raise YamlConfigError(
            "sensor shell state must match the selected mass/collision mode"
        )

    return tuple(  # type: ignore[return-value]
        parsed_by_suffix[suffix] for suffix in _SENSOR_SHELL_SUFFIXES
    )


def _parse_base_inertial(raw_inertial: Any, mode: str) -> BaseInertial | None:
    fixed_mode = mode == "fixed_base_inertial_sensor_shell_collision"
    if not fixed_mode:
        if raw_inertial is not None:
            raise YamlConfigError(
                f"base_inertial must be null for mode {mode}"
            )
        return None
    if not isinstance(raw_inertial, dict):
        raise YamlConfigError(
            "base_inertial must be a mapping for fixed-base-inertial mode"
        )
    reject_unknown(raw_inertial, _BASE_INERTIAL_FIELDS, context="base_inertial")
    require_keys(raw_inertial, _BASE_INERTIAL_FIELDS, context="base_inertial")

    mass_kg = require_number(
        raw_inertial["mass_kg"],
        context="base_inertial.mass_kg",
        positive=True,
    )
    center_of_mass_m = require_vector(
        raw_inertial["center_of_mass_m"],
        3,
        context="base_inertial.center_of_mass_m",
    )
    inertia_kg_m2 = _parse_inertia_matrix(raw_inertial["inertia_kg_m2"])
    return BaseInertial(
        mass_kg=mass_kg,
        center_of_mass_m=center_of_mass_m,  # type: ignore[arg-type]
        inertia_kg_m2=inertia_kg_m2,
    )


def _parse_inertia_matrix(
    value: Any,
) -> tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]:
    if (
        not isinstance(value, list)
        or len(value) != 3
        or any(not isinstance(row, list) or len(row) != 3 for row in value)
    ):
        raise YamlConfigError(
            "base_inertial.inertia_kg_m2 must be a 3x3 list"
        )
    rows = tuple(
        tuple(
            require_number(
                item,
                context=f"base_inertial.inertia_kg_m2[{row_index}]"
                f"[{column_index}]",
            )
            for column_index, item in enumerate(row)
        )
        for row_index, row in enumerate(value)
    )
    if any(
        not math.isclose(
            rows[row][column],
            rows[column][row],
            rel_tol=0.0,
            abs_tol=1e-12,
        )
        for row in range(3)
        for column in range(row + 1, 3)
    ):
        raise YamlConfigError(
            "base_inertial.inertia_kg_m2 must be symmetric"
        )

    a, b, c = rows[0]
    _, d, e = rows[1]
    _, _, f = rows[2]
    leading_minor_1 = a
    leading_minor_2 = a * d - b * b
    determinant = a * (d * f - e * e) - b * (b * f - c * e) + c * (
        b * e - c * d
    )
    if (
        leading_minor_1 <= 0.0
        or leading_minor_2 <= 0.0
        or determinant <= 0.0
    ):
        raise YamlConfigError(
            "base_inertial.inertia_kg_m2 must be symmetric positive definite"
        )
    return rows  # type: ignore[return-value]


def _parse_expected_link_masses(
    raw_masses: Any,
) -> tuple[
    LinkMassExpectation,
    LinkMassExpectation,
    LinkMassExpectation,
    LinkMassExpectation,
    LinkMassExpectation,
]:
    if not isinstance(raw_masses, dict):
        raise YamlConfigError("expected_link_masses must be a mapping")
    reject_unknown(
        raw_masses, _LINK_SUFFIXES, context="expected_link_masses"
    )
    require_keys(raw_masses, _LINK_SUFFIXES, context="expected_link_masses")
    return tuple(  # type: ignore[return-value]
        LinkMassExpectation(
            prim_suffix=suffix,
            mass_kg=require_number(
                raw_masses[suffix],
                context=f"expected_link_masses[{suffix}]",
                positive=True,
            ),
        )
        for suffix in _LINK_SUFFIXES
    )


def _base_inertial_matches_audit(inertial: BaseInertial) -> bool:
    if not math.isclose(
        inertial.mass_kg, 17.0, rel_tol=1e-12, abs_tol=1e-12
    ):
        return False
    if any(
        not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
        for actual, expected in zip(
            inertial.center_of_mass_m, _FIXED_CENTER_OF_MASS_M
        )
    ):
        return False
    return all(
        math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
        for actual_row, expected_row in zip(
            inertial.inertia_kg_m2, _FIXED_INERTIA_KG_M2
        )
        for actual, expected in zip(actual_row, expected_row)
    )
