from __future__ import annotations

import math
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
    "physics",
    "wheel_joints",
    "controller",
    "frames",
    "footprint",
    "static_transforms",
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
    mutate(data)
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    return path


def test_jackal_kinematics_contract_is_explicit_and_behavior_preserving():
    kinematics = load_robot_kinematics_config(JACKAL)

    assert kinematics.schema_version == 2
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

    joints = load_robot_config_contract(JACKAL).wheel_joints
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


@pytest.mark.parametrize("schema_version", [1, True, 2.0, "2", None])
def test_kinematics_contract_rejects_old_or_non_integer_schema(
    tmp_path, schema_version
):
    path = _candidate(
        tmp_path,
        lambda data: data.update({"schema_version": schema_version}),
    )

    with pytest.raises(
        YamlConfigError,
        match="robot config schema_version must be integer 2",
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


def test_custom_robot_template_exposes_schema_v2_placeholders():
    custom = ROOT / "isaac_sim/configs/robots/custom_robot.yaml"
    raw = yaml.safe_load(custom.read_text(encoding="utf-8"))
    assert set(raw) == ROBOT_CONFIG_FIELDS
    assert raw["schema_version"] == 2
    assert raw["kinematics_profile_id"] is None
    assert raw["lifecycle"] is None
    assert raw["effective_track_width"] is None
    assert "wheel_radius" not in raw["controller"]
    assert "wheel_distance" not in raw["controller"]

    with pytest.raises(
        YamlConfigError,
        match="robot.kinematics_profile_id must match",
    ):
        load_robot_kinematics_config(custom)
