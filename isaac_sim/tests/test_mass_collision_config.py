from __future__ import annotations

import copy
import math
from pathlib import Path

import pytest
import yaml

from isaac_sim.src.yaml_utils import YamlConfigError


ROOT = Path(__file__).resolve().parents[2]
PROFILE_DIR = ROOT / "isaac_sim/configs/robot_mass_profiles"
ASSET_SHA256 = "bf870a06c9b974eea2607dd7f33bb536eb930f2a7795ed07f25def792b150a8a"
SENSOR_SHELL_SUFFIXES = (
    "/base_link/collisions/bumblebee_camera",
    "/base_link/collisions/sick_lms1xx_lidar",
)
LINK_SUFFIXES = (
    "/base_link",
    "/front_left_wheel_link",
    "/front_right_wheel_link",
    "/rear_left_wheel_link",
    "/rear_right_wheel_link",
)
FIXED_CENTER_OF_MASS_M = (0.00504, 0.00062, 0.104328)
FIXED_INERTIA_KG_M2 = (
    (0.18669458, -0.00005312, -0.00105626),
    (-0.00005312, 0.30087811, -0.00012994),
    (-0.00105626, -0.00012994, 0.38648003),
)


def _api():
    from isaac_sim.src.robot.mass_collision_config import (
        load_mass_collision_profile,
        parse_mass_collision_profile,
        resolve_prim_suffix,
    )

    return (
        load_mass_collision_profile,
        parse_mass_collision_profile,
        resolve_prim_suffix,
    )


def _profile_path(profile_id: str) -> Path:
    return PROFILE_DIR / f"{profile_id}.yaml"


def _load_raw(profile_id: str = "fixed_base_inertial_sensor_shell_collision_v1"):
    return yaml.safe_load(_profile_path(profile_id).read_text(encoding="utf-8"))


def _write_candidate(tmp_path: Path, data: object) -> Path:
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


@pytest.mark.parametrize(
    (
        "profile_id",
        "mode",
        "shell_active",
        "shell_collision_enabled",
        "base_mass_kg",
        "total_mass_kg",
        "authors_base_inertial",
    ),
    [
        (
            "legacy_default_sensor_density_v1",
            "legacy_default_sensor_density",
            True,
            True,
            18.3179048,
            20.2259048,
            False,
        ),
        (
            "sensor_shells_disabled_v1",
            "sensor_shells_disabled",
            False,
            False,
            17.0,
            18.908,
            False,
        ),
        (
            "fixed_base_inertial_sensor_shell_collision_v1",
            "fixed_base_inertial_sensor_shell_collision",
            True,
            True,
            17.0,
            18.908,
            True,
        ),
    ],
)
def test_loads_versioned_profiles_with_exact_mode_semantics(
    profile_id,
    mode,
    shell_active,
    shell_collision_enabled,
    base_mass_kg,
    total_mass_kg,
    authors_base_inertial,
):
    load_profile, _, _ = _api()

    profile = load_profile(_profile_path(profile_id))

    assert profile.schema_version == 1
    assert profile.profile_id == profile_id
    assert profile.mode == mode
    assert profile.robot_asset_sha256 == ASSET_SHA256
    assert profile.base_prim_suffix == "/base_link"
    assert tuple(shell.prim_suffix for shell in profile.sensor_shells) == (
        SENSOR_SHELL_SUFFIXES
    )
    assert all(shell.active is shell_active for shell in profile.sensor_shells)
    assert all(
        shell.collision_enabled is shell_collision_enabled
        for shell in profile.sensor_shells
    )
    assert tuple(
        expectation.prim_suffix
        for expectation in profile.expected_link_masses
    ) == LINK_SUFFIXES
    expected_masses = dict(
        (expectation.prim_suffix, expectation.mass_kg)
        for expectation in profile.expected_link_masses
    )
    assert expected_masses["/base_link"] == base_mass_kg
    assert set(expected_masses.values()) == {base_mass_kg, 0.477}
    assert profile.expected_total_mass_kg == total_mass_kg
    assert (profile.base_inertial is not None) is authors_base_inertial


def test_fixed_profile_freezes_the_audited_no_shell_base_inertial():
    load_profile, _, _ = _api()

    profile = load_profile(
        _profile_path("fixed_base_inertial_sensor_shell_collision_v1")
    )

    assert profile.base_inertial is not None
    assert profile.base_inertial.mass_kg == 17.0
    assert profile.base_inertial.center_of_mass_m == FIXED_CENTER_OF_MASS_M
    assert profile.base_inertial.inertia_kg_m2 == FIXED_INERTIA_KG_M2


def test_profile_loader_rejects_duplicate_yaml_mapping_keys(tmp_path):
    load_profile, _, _ = _api()
    source = _profile_path("legacy_default_sensor_density_v1").read_text(
        encoding="utf-8"
    )
    path = tmp_path / "profile.yaml"
    path.write_text(source + "\nmode: sensor_shells_disabled\n", encoding="utf-8")

    with pytest.raises(YamlConfigError, match="duplicate YAML mapping key.*mode"):
        load_profile(path)


@pytest.mark.parametrize("mutation", ["unknown", "missing"])
def test_profile_contract_has_an_exact_top_level_shape(tmp_path, mutation):
    load_profile, _, _ = _api()
    data = _load_raw()
    if mutation == "unknown":
        data["base_mass_typo"] = 17.0
        message = "unknown mass collision profile keys.*base_mass_typo"
    else:
        data.pop("expected_total_mass_kg")
        message = "missing mass collision profile keys.*expected_total_mass_kg"

    with pytest.raises(YamlConfigError, match=message):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize("schema_version", [0, 2, True, 1.0, "1", None])
def test_profile_schema_is_exact_integer_one(tmp_path, schema_version):
    load_profile, _, _ = _api()
    data = _load_raw()
    data["schema_version"] = schema_version

    with pytest.raises(
        YamlConfigError,
        match="mass collision profile schema_version must be integer 1",
    ):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    ("profile_id", "mode"),
    [
        (
            "fixed_base_inertial_sensor_shell_collision_v1",
            "sensor_shells_disabled",
        ),
        ("unknown_v1", "fixed_base_inertial_sensor_shell_collision"),
        (
            "fixed_base_inertial_sensor_shell_collision_v2",
            "fixed_base_inertial_sensor_shell_collision",
        ),
        ("sensor_shells_disabled_v1", "unknown"),
    ],
)
def test_profile_id_and_mode_are_a_closed_one_to_one_contract(
    tmp_path, profile_id, mode
):
    load_profile, _, _ = _api()
    data = _load_raw()
    data["profile_id"] = profile_id
    data["mode"] = mode

    with pytest.raises(YamlConfigError, match="profile_id.*mode"):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    "digest",
    [
        "f" * 63,
        "f" * 65,
        "F" * 64,
        "g" * 64,
        "sha256:" + "f" * 64,
        True,
        None,
    ],
)
def test_robot_asset_sha256_is_canonical_lowercase_hex(tmp_path, digest):
    load_profile, _, _ = _api()
    data = _load_raw()
    data["robot_asset_sha256"] = digest

    with pytest.raises(
        YamlConfigError,
        match="robot_asset_sha256 must be 64 lowercase hexadecimal characters",
    ):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    "suffix",
    [
        "base_link",
        "/base_link/",
        "//base_link",
        "/base_link//collisions",
        "/base_link/../wheel",
        "/World/base_link",
        True,
        None,
    ],
)
def test_base_prim_suffix_is_the_exact_safe_contract(tmp_path, suffix):
    load_profile, _, _ = _api()
    data = _load_raw()
    data["base_prim_suffix"] = suffix

    with pytest.raises(YamlConfigError, match="base_prim_suffix"):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    "mutation",
    [
        "missing",
        "duplicate",
        "extra",
        "wrong_suffix",
        "unknown_field",
        "missing_field",
        "non_boolean_active",
        "non_boolean_collision",
    ],
)
def test_sensor_shell_contract_is_exact_unique_and_typed(tmp_path, mutation):
    load_profile, _, _ = _api()
    data = _load_raw()
    shells = data["sensor_shells"]
    if mutation == "missing":
        shells.pop()
    elif mutation == "duplicate":
        shells[1]["prim_suffix"] = shells[0]["prim_suffix"]
    elif mutation == "extra":
        shells.append(copy.deepcopy(shells[0]))
        shells[-1]["prim_suffix"] = "/base_link/collisions/other"
    elif mutation == "wrong_suffix":
        shells[0]["prim_suffix"] = "/base_link/collisions/camera"
    elif mutation == "unknown_field":
        shells[0]["enabled"] = True
    elif mutation == "missing_field":
        shells[0].pop("collision_enabled")
    elif mutation == "non_boolean_active":
        shells[0]["active"] = 1
    else:
        shells[0]["collision_enabled"] = "true"

    with pytest.raises(YamlConfigError, match="sensor_shell"):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    ("profile_id", "active", "collision_enabled"),
    [
        ("legacy_default_sensor_density_v1", False, True),
        ("legacy_default_sensor_density_v1", True, False),
        ("sensor_shells_disabled_v1", True, False),
        ("sensor_shells_disabled_v1", False, True),
        ("fixed_base_inertial_sensor_shell_collision_v1", False, True),
        ("fixed_base_inertial_sensor_shell_collision_v1", True, False),
    ],
)
def test_shell_state_must_match_the_selected_mode(
    tmp_path, profile_id, active, collision_enabled
):
    load_profile, _, _ = _api()
    data = _load_raw(profile_id)
    data["sensor_shells"][0]["active"] = active
    data["sensor_shells"][0]["collision_enabled"] = collision_enabled

    with pytest.raises(YamlConfigError, match="sensor shell state.*mode"):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    "profile_id",
    ["legacy_default_sensor_density_v1", "sensor_shells_disabled_v1"],
)
def test_non_authoring_modes_require_null_base_inertial(tmp_path, profile_id):
    load_profile, _, _ = _api()
    data = _load_raw(profile_id)
    data["base_inertial"] = copy.deepcopy(
        _load_raw()["base_inertial"]
    )

    with pytest.raises(YamlConfigError, match="base_inertial must be null.*mode"):
        load_profile(_write_candidate(tmp_path, data))


def test_fixed_mode_requires_a_complete_base_inertial(tmp_path):
    load_profile, _, _ = _api()
    data = _load_raw()
    data["base_inertial"] = None

    with pytest.raises(YamlConfigError, match="base_inertial must be a mapping"):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    "mutation",
    ["unknown", "missing_mass", "bad_com_shape", "bad_matrix_shape"],
)
def test_base_inertial_shape_is_strict(tmp_path, mutation):
    load_profile, _, _ = _api()
    data = _load_raw()
    inertial = data["base_inertial"]
    if mutation == "unknown":
        inertial["density"] = 1.0
        message = "unknown base_inertial keys.*density"
    elif mutation == "missing_mass":
        inertial.pop("mass_kg")
        message = "missing base_inertial keys.*mass_kg"
    elif mutation == "bad_com_shape":
        inertial["center_of_mass_m"] = [0.0, 0.0]
        message = "center_of_mass_m must be a 3-element list"
    else:
        inertial["inertia_kg_m2"] = [[1.0, 0.0], [0.0, 1.0]]
        message = "inertia_kg_m2 must be a 3x3 list"

    with pytest.raises(YamlConfigError, match=message):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("mass", True, "must be numeric"),
        ("mass", 0.0, "must be positive"),
        ("mass", math.inf, "must be finite"),
        ("com", math.nan, "must be finite"),
        ("inertia", math.inf, "must be finite"),
    ],
)
def test_base_inertial_numbers_are_finite_and_mass_is_positive(
    tmp_path, field, value, message
):
    load_profile, _, _ = _api()
    data = _load_raw()
    if field == "mass":
        data["base_inertial"]["mass_kg"] = value
    elif field == "com":
        data["base_inertial"]["center_of_mass_m"][1] = value
    else:
        data["base_inertial"]["inertia_kg_m2"][1][1] = value

    with pytest.raises(YamlConfigError, match=message):
        load_profile(_write_candidate(tmp_path, data))


def test_inertia_tensor_must_be_symmetric(tmp_path):
    load_profile, _, _ = _api()
    data = _load_raw()
    data["base_inertial"]["inertia_kg_m2"][0][1] = 0.01

    with pytest.raises(YamlConfigError, match="inertia_kg_m2 must be symmetric"):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    "matrix",
    [
        [[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [[1.0, 2.0, 0.0], [2.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
    ],
)
def test_inertia_tensor_must_be_positive_definite_without_numpy(
    tmp_path, matrix
):
    load_profile, _, _ = _api()
    data = _load_raw()
    data["base_inertial"]["inertia_kg_m2"] = matrix

    with pytest.raises(
        YamlConfigError,
        match="inertia_kg_m2 must be symmetric positive definite",
    ):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    "mutation",
    [
        "missing",
        "extra",
        "wrong_suffix",
        "non_numeric",
        "nonfinite",
        "nonpositive",
    ],
)
def test_expected_link_mass_contract_is_exact_and_positive(
    tmp_path, mutation
):
    load_profile, _, _ = _api()
    data = _load_raw()
    masses = data["expected_link_masses"]
    if mutation == "missing":
        masses.pop("/rear_right_wheel_link")
    elif mutation == "extra":
        masses["/caster_link"] = 0.1
    elif mutation == "wrong_suffix":
        masses["rear_right_wheel_link"] = masses.pop(
            "/rear_right_wheel_link"
        )
    elif mutation == "non_numeric":
        masses["/base_link"] = True
    elif mutation == "nonfinite":
        masses["/base_link"] = math.nan
    else:
        masses["/base_link"] = 0.0

    with pytest.raises(YamlConfigError, match="expected_link_masses"):
        load_profile(_write_candidate(tmp_path, data))


def test_expected_total_mass_must_equal_the_five_link_masses(tmp_path):
    load_profile, _, _ = _api()
    data = _load_raw()
    data["expected_total_mass_kg"] += 0.001

    with pytest.raises(
        YamlConfigError,
        match="expected_total_mass_kg must equal the sum of expected_link_masses",
    ):
        load_profile(_write_candidate(tmp_path, data))


def test_fixed_base_inertial_mass_must_match_expected_base_mass(tmp_path):
    load_profile, _, _ = _api()
    data = _load_raw()
    data["base_inertial"]["mass_kg"] = 16.9
    data["expected_link_masses"]["/base_link"] = 16.9
    data["expected_total_mass_kg"] = 18.808

    with pytest.raises(
        YamlConfigError,
        match="fixed base_inertial must match the audited v1 contract",
    ):
        load_profile(_write_candidate(tmp_path, data))


@pytest.mark.parametrize(
    "articulation_root",
    [
        "World/Robot",
        "/World/Robot/",
        "//World/Robot",
        "/World//Robot",
        "/World/../Robot",
        "/World/Robot-1",
        "/",
        True,
        None,
    ],
)
def test_prim_suffix_resolver_rejects_unsafe_articulation_roots(
    articulation_root
):
    _, _, resolve_prim_suffix = _api()

    with pytest.raises(YamlConfigError, match="articulation_root"):
        resolve_prim_suffix(articulation_root, "/base_link")


@pytest.mark.parametrize(
    "suffix",
    [
        "base_link",
        "/base_link/",
        "//base_link",
        "/base_link//collisions",
        "/base_link/../wheel",
        "/base-link",
        "/",
        True,
        None,
    ],
)
def test_prim_suffix_resolver_rejects_unsafe_suffixes(suffix):
    _, _, resolve_prim_suffix = _api()

    with pytest.raises(YamlConfigError, match="prim_suffix"):
        resolve_prim_suffix("/World/Robot_1", suffix)


def test_prim_suffix_resolver_joins_without_path_normalization():
    _, _, resolve_prim_suffix = _api()

    assert (
        resolve_prim_suffix(
            "/World/Robot_1",
            "/base_link/collisions/bumblebee_camera",
        )
        == "/World/Robot_1/base_link/collisions/bumblebee_camera"
    )


def test_parse_api_accepts_a_loaded_mapping():
    _, parse_profile, _ = _api()

    profile = parse_profile(_load_raw("sensor_shells_disabled_v1"))

    assert profile.profile_id == "sensor_shells_disabled_v1"
    assert profile.base_inertial is None
