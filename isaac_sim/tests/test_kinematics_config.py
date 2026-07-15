from __future__ import annotations

import math
from dataclasses import asdict, is_dataclass
from pathlib import Path

import pytest
import yaml

from isaac_sim.graphs.control_graph import load_controller_config
from isaac_sim.src.robot.kinematics_config import (
    load_robot_config_contract,
    load_robot_kinematics_config,
)
from isaac_sim.src.yaml_utils import YamlConfigError


ROOT = Path(__file__).resolve().parents[2]
JACKAL = ROOT / "isaac_sim/configs/robots/jackal.yaml"
EXPERIMENTAL_ROBOTS = (
    (
        ROOT
        / "isaac_sim/configs/robots/experimental/jackal_etw_0p989_v1.yaml",
        "jackal_etw_0p989_v1",
        0.989,
    ),
    (
        ROOT
        / "isaac_sim/configs/robots/experimental/jackal_etw_1p012_v1.yaml",
        "jackal_etw_1p012_v1",
        1.012,
    ),
)
ROBOT_CONFIG_FIELDS = {
    "schema_version",
    "name",
    "kinematics_profile_id",
    "lifecycle",
    "wheel_radius",
    "wheel_width",
    "geometric_track_width",
    "effective_track_width",
    "wheelbase",
    "base_mass",
    "wheel_mass",
    "nominal_total_mass",
    "mass_collision_profile",
    "wheel_velocity_drive",
    "physics",
    "wheel_joints",
    "controller",
    "frames",
    "footprint",
    "static_transforms",
}
WHEEL_VELOCITY_DRIVE_FIELDS = {
    "schema_version",
    "profile_id",
    "drive_type",
    "stiffness_n_m_per_rad",
    "damping_n_m_s_per_rad",
    "max_effort_n_m",
    "max_joint_velocity_rad_s",
}
CONTROLLER_FIELDS = {
    "max_linear_speed",
    "max_angular_speed",
    "max_wheel_speed",
    "max_acceleration",
    "max_deceleration",
    "max_angular_acceleration",
}


def _candidate(tmp_path: Path, mutate) -> Path:
    data = yaml.safe_load(JACKAL.read_text(encoding="utf-8"))
    mass_profile = tmp_path / "mass_profile.yaml"
    mass_profile.write_text("schema_version: 1\n", encoding="utf-8")
    data["mass_collision_profile"] = mass_profile.name
    mutate(data)
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def test_jackal_kinematics_contract_is_explicit_and_behavior_preserving():
    kinematics = load_robot_kinematics_config(JACKAL)

    assert kinematics.schema_version == 3
    assert kinematics.kinematics_profile_id == "jackal_legacy_geometric_v1"
    assert kinematics.lifecycle == "stable_baseline"
    assert kinematics.wheel_radius == 0.098
    assert kinematics.wheel_width == 0.040
    assert kinematics.geometric_track_width == 0.37559
    assert kinematics.effective_track_width == 0.37559
    assert kinematics.wheelbase == 0.262
    assert kinematics.base_mass == 17.0
    assert kinematics.wheel_mass == 0.477
    assert kinematics.nominal_total_mass == 18.908

    contract = load_robot_config_contract(JACKAL)
    assert contract.schema_version == 3
    assert contract.mass_collision_profile == (
        ROOT
        / "isaac_sim/configs/robot_mass_profiles/legacy_default_sensor_density_v1.yaml"
    ).resolve()
    assert is_dataclass(contract.wheel_velocity_drive)
    assert asdict(contract.wheel_velocity_drive) == {
        "schema_version": 1,
        "profile_id": "jackal_drive_legacy_finite_guard_v1",
        "drive_type": "force",
        "stiffness_n_m_per_rad": 0.0,
        "damping_n_m_s_per_rad": 572957795.1308232,
        "max_effort_n_m": 1_000_000_000.0,
        "max_joint_velocity_rad_s": 1_000_000_000.0,
    }

    joints = contract.wheel_joints
    assert joints.ordered == (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    )
    assert joints.front == (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
    )
    assert joints.rear == (
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    )


@pytest.mark.parametrize(
    ("candidate_path", "profile_id", "effective_track_width"),
    EXPERIMENTAL_ROBOTS,
)
def test_effective_track_candidates_only_change_declared_experimental_fields(
    candidate_path, profile_id, effective_track_width
):
    stable = yaml.safe_load(JACKAL.read_text(encoding="utf-8"))
    candidate = yaml.safe_load(candidate_path.read_text(encoding="utf-8"))
    allowed_differences = {
        "kinematics_profile_id",
        "lifecycle",
        "effective_track_width",
        "mass_collision_profile",
    }

    assert candidate.keys() == stable.keys()
    for key in stable.keys() - allowed_differences:
        assert candidate[key] == stable[key]
    assert candidate["kinematics_profile_id"] == profile_id
    assert candidate["lifecycle"] == "experimental_candidate"
    assert candidate["effective_track_width"] == effective_track_width

    contract = load_robot_config_contract(candidate_path)
    assert contract.kinematics.kinematics_profile_id == profile_id
    assert contract.kinematics.lifecycle == "experimental_candidate"
    assert contract.kinematics.effective_track_width == effective_track_width
    assert contract.mass_collision_profile == load_robot_config_contract(
        JACKAL
    ).mass_collision_profile
    assert load_controller_config(candidate_path)[
        "effective_track_width"
    ] == effective_track_width


def test_kinematics_contract_rejects_unknown_top_level_keys(tmp_path):
    path = _candidate(tmp_path, lambda data: data.update({"wheelbase_typo": 1.0}))

    with pytest.raises(
        YamlConfigError,
        match="unknown robot config keys.*wheelbase_typo",
    ):
        load_robot_kinematics_config(path)


def test_kinematics_contract_rejects_duplicate_yaml_keys(tmp_path):
    path = tmp_path / "robot.yaml"
    path.write_text(
        JACKAL.read_text(encoding="utf-8")
        + "\neffective_track_width: 1.0\n",
        encoding="utf-8",
    )

    with pytest.raises(
        YamlConfigError,
        match="duplicate YAML mapping key.*effective_track_width",
    ):
        load_robot_kinematics_config(path)


@pytest.mark.parametrize(
    "field",
    sorted(ROBOT_CONFIG_FIELDS),
)
def test_kinematics_contract_requires_every_explicit_field(tmp_path, field):
    path = _candidate(tmp_path, lambda data: data.pop(field))

    with pytest.raises(YamlConfigError, match=f"missing robot config keys.*{field}"):
        load_robot_kinematics_config(path)


@pytest.mark.parametrize("schema_version", [1, 2, True, 3.0, "3", None])
def test_kinematics_contract_rejects_old_or_non_integer_schema(
    tmp_path, schema_version
):
    path = _candidate(
        tmp_path,
        lambda data: data.update({"schema_version": schema_version}),
    )

    with pytest.raises(
        YamlConfigError,
        match="robot config schema_version must be integer 3",
    ):
        load_robot_kinematics_config(path)


@pytest.mark.parametrize(
    "profile_id",
    [None, True, "", " leading", "trailing ", "path/profile", "two words"],
)
def test_kinematics_profile_id_must_be_path_safe(tmp_path, profile_id):
    path = _candidate(
        tmp_path,
        lambda data: data.update({"kinematics_profile_id": profile_id}),
    )

    with pytest.raises(
        YamlConfigError,
        match="robot.kinematics_profile_id must match",
    ):
        load_robot_kinematics_config(path)


@pytest.mark.parametrize(
    "lifecycle",
    [None, True, "", "stable", "experimental", "STABLE_BASELINE"],
)
def test_kinematics_lifecycle_is_a_closed_enum(tmp_path, lifecycle):
    path = _candidate(
        tmp_path,
        lambda data: data.update({"lifecycle": lifecycle}),
    )

    with pytest.raises(
        YamlConfigError,
        match="robot.lifecycle must be one of.*stable_baseline.*experimental_candidate",
    ):
        load_robot_kinematics_config(path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        (field, value, message)
        for field in (
            "wheel_radius",
            "wheel_width",
            "geometric_track_width",
            "effective_track_width",
            "wheelbase",
            "base_mass",
            "wheel_mass",
            "nominal_total_mass",
        )
        for value, message in (
            (True, "must be numeric"),
            (False, "must be numeric"),
            (math.nan, "must be finite"),
            (math.inf, "must be finite"),
            (0.0, "must be positive"),
            (-0.1, "must be positive"),
        )
    ],
)
def test_kinematics_dimensions_are_strict_positive_finite_numbers(
    tmp_path, field, value, message
):
    path = _candidate(
        tmp_path,
        lambda data: data.update({field: value}),
    )

    with pytest.raises(
        YamlConfigError,
        match=rf"robot\.{field} {message}",
    ):
        load_robot_kinematics_config(path)


def test_nominal_total_mass_must_equal_base_plus_four_wheels(tmp_path):
    path = _candidate(
        tmp_path,
        lambda data: data.update({"nominal_total_mass": 18.0}),
    )

    with pytest.raises(
        YamlConfigError,
        match="robot.nominal_total_mass must equal base_mass \\+ 4 \\* wheel_mass",
    ):
        load_robot_config_contract(path)


@pytest.mark.parametrize("mutation", ["unknown", "missing", "duplicate", "unsafe"])
def test_wheel_joint_mapping_is_exact_unique_and_command_safe(tmp_path, mutation):
    def mutate(data):
        joints = data["wheel_joints"]
        if mutation == "unknown":
            joints["middle_left"] = "middle_left_wheel_joint"
        elif mutation == "missing":
            joints.pop("rear_right")
        elif mutation == "duplicate":
            joints["rear_right"] = joints["front_right"]
        else:
            joints["rear_right"] = "rear right wheel joint"

    path = _candidate(tmp_path, mutate)

    with pytest.raises(YamlConfigError, match="robot.wheel_joints"):
        load_robot_config_contract(path)


def test_controller_owns_limits_but_not_kinematic_dimensions():
    from isaac_sim.src.robot.kinematics_config import (
        RobotControllerConfig,
        load_robot_config_contract,
    )

    raw = yaml.safe_load(JACKAL.read_text(encoding="utf-8"))
    assert set(raw) == ROBOT_CONFIG_FIELDS
    assert set(raw["controller"]) == CONTROLLER_FIELDS

    contract = load_robot_config_contract(JACKAL)
    assert contract.controller == RobotControllerConfig(
        max_linear_speed=1.0,
        max_angular_speed=1.5,
        max_wheel_speed=15.0,
        max_acceleration=0.75,
        max_deceleration=1.0,
        max_angular_acceleration=2.0,
    )

    controller = load_controller_config(JACKAL)
    assert controller == {
        "wheel_radius": 0.098,
        "effective_track_width": 0.37559,
        "max_linear_speed": 1.0,
        "max_angular_speed": 1.5,
        "max_wheel_speed": 15.0,
        "max_acceleration": 0.75,
        "max_deceleration": 1.0,
        "max_angular_acceleration": 2.0,
    }


@pytest.mark.parametrize("legacy_key", ["wheel_radius", "wheel_distance"])
def test_controller_rejects_legacy_duplicated_kinematics(tmp_path, legacy_key):
    path = _candidate(
        tmp_path,
        lambda data: data["controller"].update({legacy_key: 0.37559}),
    )

    with pytest.raises(
        YamlConfigError,
        match=rf"unknown robot\.controller keys.*{legacy_key}",
    ):
        load_controller_config(path)


@pytest.mark.parametrize("field", sorted(CONTROLLER_FIELDS))
def test_controller_requires_every_limit(tmp_path, field):
    path = _candidate(
        tmp_path,
        lambda data: data["controller"].pop(field),
    )

    with pytest.raises(
        YamlConfigError,
        match=rf"missing robot\.controller keys.*{field}",
    ):
        load_controller_config(path)


@pytest.mark.parametrize(
    ("value", "message"),
    [
        (True, "must be numeric"),
        (math.nan, "must be finite"),
        (math.inf, "must be finite"),
        (0.0, "must be positive"),
        (-0.1, "must be positive"),
    ],
)
def test_controller_limits_are_strict_positive_finite_numbers(
    tmp_path, value, message
):
    path = _candidate(
        tmp_path,
        lambda data: data["controller"].update({"max_linear_speed": value}),
    )

    with pytest.raises(
        YamlConfigError,
        match=rf"robot\.controller\.max_linear_speed {message}",
    ):
        load_controller_config(path)


@pytest.mark.parametrize("value", [None, True, "", "   "])
def test_mass_collision_profile_must_be_a_nonempty_path_string(
    tmp_path, value
):
    path = _candidate(
        tmp_path,
        lambda data: data.update({"mass_collision_profile": value}),
    )

    with pytest.raises(
        YamlConfigError,
        match="robot.mass_collision_profile must be a nonempty path string",
    ):
        load_robot_config_contract(path)


def test_mass_collision_profile_is_resolved_relative_to_robot_yaml(tmp_path):
    profile_dir = tmp_path / "profiles"
    profile_dir.mkdir()
    profile = profile_dir / "mass.yaml"
    profile.write_text("schema_version: 1\n", encoding="utf-8")
    path = _candidate(
        tmp_path,
        lambda data: data.update(
            {"mass_collision_profile": "profiles/../profiles/mass.yaml"}
        ),
    )

    contract = load_robot_config_contract(path)

    assert contract.mass_collision_profile == profile.resolve()
    assert contract.mass_collision_profile.is_absolute()


@pytest.mark.parametrize("target_kind", ["missing", "directory"])
def test_mass_collision_profile_must_reference_an_existing_regular_file(
    tmp_path, target_kind
):
    target = tmp_path / target_kind
    if target_kind == "directory":
        target.mkdir()
    path = _candidate(
        tmp_path,
        lambda data: data.update({"mass_collision_profile": target.name}),
    )

    with pytest.raises(
        YamlConfigError,
        match=(
            "robot.mass_collision_profile must reference an existing regular "
            "file"
        ),
    ):
        load_robot_config_contract(path)


@pytest.mark.parametrize(
    "mutation",
    ["unknown", *sorted(WHEEL_VELOCITY_DRIVE_FIELDS)],
)
def test_wheel_velocity_drive_mapping_is_exact(tmp_path, mutation):
    def mutate(data):
        drive = data["wheel_velocity_drive"]
        if mutation == "unknown":
            drive["maximum_force"] = 1.0
        else:
            drive.pop(mutation)

    path = _candidate(tmp_path, mutate)

    with pytest.raises(YamlConfigError, match="robot.wheel_velocity_drive"):
        load_robot_config_contract(path)


@pytest.mark.parametrize("value", [None, True, 1, [], "force"])
def test_wheel_velocity_drive_must_be_a_mapping(tmp_path, value):
    path = _candidate(
        tmp_path,
        lambda data: data.update({"wheel_velocity_drive": value}),
    )

    with pytest.raises(
        YamlConfigError,
        match="robot.wheel_velocity_drive must be a mapping",
    ):
        load_robot_config_contract(path)


@pytest.mark.parametrize("schema_version", [0, 2, True, 1.0, "1", None])
def test_wheel_velocity_drive_schema_is_strict_integer_one(
    tmp_path, schema_version
):
    path = _candidate(
        tmp_path,
        lambda data: data["wheel_velocity_drive"].update(
            {"schema_version": schema_version}
        ),
    )

    with pytest.raises(
        YamlConfigError,
        match="robot.wheel_velocity_drive.schema_version must be integer 1",
    ):
        load_robot_config_contract(path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        (field, value, message)
        for field in (
            "damping_n_m_s_per_rad",
            "max_effort_n_m",
            "max_joint_velocity_rad_s",
        )
        for value, message in (
            (True, "must be numeric"),
            (math.nan, "must be finite"),
            (math.inf, "must be finite"),
            (0.0, "must be positive"),
            (-1.0, "must be positive"),
        )
    ],
)
def test_wheel_velocity_drive_positive_fields_are_strict_finite_numbers(
    tmp_path, field, value, message
):
    path = _candidate(
        tmp_path,
        lambda data: data["wheel_velocity_drive"].update({field: value}),
    )

    with pytest.raises(
        YamlConfigError,
        match=rf"robot\.wheel_velocity_drive\.{field} {message}",
    ):
        load_robot_config_contract(path)


@pytest.mark.parametrize(
    "value",
    [True, math.nan, math.inf, -1.0, 0.000001],
)
def test_wheel_velocity_drive_stiffness_must_be_finite_numeric_zero(
    tmp_path, value
):
    path = _candidate(
        tmp_path,
        lambda data: data["wheel_velocity_drive"].update(
            {"stiffness_n_m_per_rad": value}
        ),
    )

    with pytest.raises(
        YamlConfigError,
        match="robot.wheel_velocity_drive.stiffness_n_m_per_rad",
    ):
        load_robot_config_contract(path)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("profile_id", None, "must match"),
        ("profile_id", True, "must match"),
        ("profile_id", "", "must match"),
        ("profile_id", "drive profile", "must match"),
        ("drive_type", None, "must equal force"),
        ("drive_type", True, "must equal force"),
        ("drive_type", "acceleration", "must equal force"),
    ],
)
def test_wheel_velocity_drive_identity_is_strict(
    tmp_path, field, value, message
):
    path = _candidate(
        tmp_path,
        lambda data: data["wheel_velocity_drive"].update({field: value}),
    )

    with pytest.raises(
        YamlConfigError,
        match=rf"robot\.wheel_velocity_drive\.{field} {message}",
    ):
        load_robot_config_contract(path)


def test_controller_max_wheel_speed_must_not_exceed_drive_limit(tmp_path):
    def mutate(data):
        data["controller"]["max_wheel_speed"] = 16.0
        data["wheel_velocity_drive"]["max_joint_velocity_rad_s"] = 15.0

    path = _candidate(tmp_path, mutate)

    with pytest.raises(
        YamlConfigError,
        match=(
            "robot.controller.max_wheel_speed must be less than or equal to "
            "robot.wheel_velocity_drive.max_joint_velocity_rad_s"
        ),
    ):
        load_robot_config_contract(path)


def test_custom_robot_template_exposes_schema_v3_placeholders():
    custom = ROOT / "isaac_sim/configs/robots/custom_robot.yaml"
    raw = yaml.safe_load(custom.read_text(encoding="utf-8"))
    assert set(raw) == ROBOT_CONFIG_FIELDS
    assert raw["schema_version"] == 3
    assert raw["kinematics_profile_id"] is None
    assert raw["lifecycle"] is None
    assert raw["effective_track_width"] is None
    assert raw["mass_collision_profile"] is None
    assert set(raw["wheel_velocity_drive"]) == WHEEL_VELOCITY_DRIVE_FIELDS
    assert raw["wheel_velocity_drive"]["schema_version"] == 1
    for field in WHEEL_VELOCITY_DRIVE_FIELDS - {"schema_version"}:
        assert raw["wheel_velocity_drive"][field] is None
    assert "wheel_radius" not in raw["controller"]
    assert "wheel_distance" not in raw["controller"]

    with pytest.raises(
        YamlConfigError,
        match="robot.kinematics_profile_id must match",
    ):
        load_robot_kinematics_config(custom)
