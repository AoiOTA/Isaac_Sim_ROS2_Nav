"""Strict robot kinematics contract shared by Isaac graph builders."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from isaac_sim.src.yaml_utils import (
    YamlConfigError,
    load_mapping,
    reject_unknown,
    require_keys,
    require_number,
)


_ROBOT_CONFIG_FIELDS = frozenset(
    {
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
)
_PROFILE_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
_JOINT_NAME_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_.-]*")
_LIFECYCLES = frozenset({"stable_baseline", "experimental_candidate"})
_WHEEL_JOINT_FIELDS = frozenset(
    {"front_left", "front_right", "rear_left", "rear_right"}
)
_CONTROLLER_FIELDS = frozenset(
    {
        "max_linear_speed",
        "max_angular_speed",
        "max_wheel_speed",
        "max_acceleration",
        "max_deceleration",
        "max_angular_acceleration",
    }
)


@dataclass(frozen=True)
class RobotKinematicsConfig:
    schema_version: int
    kinematics_profile_id: str
    lifecycle: str
    wheel_radius: float
    wheel_width: float
    geometric_track_width: float
    effective_track_width: float
    wheelbase: float
    base_mass: float
    wheel_mass: float
    nominal_total_mass: float


@dataclass(frozen=True)
class RobotWheelJoints:
    front_left: str
    front_right: str
    rear_left: str
    rear_right: str

    @property
    def ordered(self) -> tuple[str, str, str, str]:
        return (
            self.front_left,
            self.front_right,
            self.rear_left,
            self.rear_right,
        )

    @property
    def front(self) -> tuple[str, str]:
        return (self.front_left, self.front_right)

    @property
    def rear(self) -> tuple[str, str]:
        return (self.rear_left, self.rear_right)


@dataclass(frozen=True)
class RobotControllerConfig:
    max_linear_speed: float
    max_angular_speed: float
    max_wheel_speed: float
    max_acceleration: float
    max_deceleration: float
    max_angular_acceleration: float


@dataclass(frozen=True)
class RobotConfigContract:
    kinematics: RobotKinematicsConfig
    wheel_joints: RobotWheelJoints
    controller: RobotControllerConfig


def load_robot_config_contract(path: str | Path) -> RobotConfigContract:
    """Load and validate the complete robot/controller schema-v2 contract."""

    return parse_robot_config_contract(load_mapping(path))


def load_robot_kinematics_config(
    path: str | Path,
) -> RobotKinematicsConfig:
    """Load validated kinematics from a complete robot schema-v2 YAML file."""

    return load_robot_config_contract(path).kinematics


def parse_robot_config_contract(
    data: Mapping[str, Any],
) -> RobotConfigContract:
    """Validate a loaded robot mapping, including its controller limits."""

    kinematics = parse_robot_kinematics_config(data)
    wheel_joints = data["wheel_joints"]
    if not isinstance(wheel_joints, dict):
        raise YamlConfigError("robot.wheel_joints must be a mapping")
    reject_unknown(
        wheel_joints,
        _WHEEL_JOINT_FIELDS,
        context="robot.wheel_joints",
    )
    require_keys(
        wheel_joints,
        _WHEEL_JOINT_FIELDS,
        context="robot.wheel_joints",
    )

    def joint_name(field: str) -> str:
        value = wheel_joints[field]
        if (
            not isinstance(value, str)
            or _JOINT_NAME_PATTERN.fullmatch(value) is None
        ):
            raise YamlConfigError(
                f"robot.wheel_joints.{field} must match "
                "[A-Za-z_][A-Za-z0-9_.-]*"
            )
        return value

    parsed_joints = RobotWheelJoints(
        front_left=joint_name("front_left"),
        front_right=joint_name("front_right"),
        rear_left=joint_name("rear_left"),
        rear_right=joint_name("rear_right"),
    )
    if len(set(parsed_joints.ordered)) != 4:
        raise YamlConfigError("robot.wheel_joints values must be unique")

    controller = data["controller"]
    if not isinstance(controller, dict):
        raise YamlConfigError("robot.controller must be a mapping")
    reject_unknown(controller, _CONTROLLER_FIELDS, context="robot.controller")
    require_keys(controller, _CONTROLLER_FIELDS, context="robot.controller")

    def controller_number(field: str) -> float:
        return require_number(
            controller[field],
            context=f"robot.controller.{field}",
            positive=True,
        )

    return RobotConfigContract(
        kinematics=kinematics,
        wheel_joints=parsed_joints,
        controller=RobotControllerConfig(
            max_linear_speed=controller_number("max_linear_speed"),
            max_angular_speed=controller_number("max_angular_speed"),
            max_wheel_speed=controller_number("max_wheel_speed"),
            max_acceleration=controller_number("max_acceleration"),
            max_deceleration=controller_number("max_deceleration"),
            max_angular_acceleration=controller_number(
                "max_angular_acceleration"
            ),
        ),
    )


def parse_robot_kinematics_config(
    data: Mapping[str, Any],
) -> RobotKinematicsConfig:
    """Validate and extract kinematics from a loaded robot mapping."""

    if not isinstance(data, dict):
        raise YamlConfigError("robot config must be a mapping")
    reject_unknown(data, _ROBOT_CONFIG_FIELDS, context="robot config")
    require_keys(data, _ROBOT_CONFIG_FIELDS, context="robot config")

    schema_version = data["schema_version"]
    if (
        isinstance(schema_version, bool)
        or not isinstance(schema_version, int)
        or schema_version != 2
    ):
        raise YamlConfigError("robot config schema_version must be integer 2")

    profile_id = data["kinematics_profile_id"]
    if (
        not isinstance(profile_id, str)
        or _PROFILE_ID_PATTERN.fullmatch(profile_id) is None
    ):
        raise YamlConfigError(
            "robot.kinematics_profile_id must match "
            "[A-Za-z0-9][A-Za-z0-9_.-]*"
        )

    lifecycle = data["lifecycle"]
    if not isinstance(lifecycle, str) or lifecycle not in _LIFECYCLES:
        raise YamlConfigError(
            "robot.lifecycle must be one of stable_baseline, "
            "experimental_candidate"
        )

    wheel_radius = require_number(
        data["wheel_radius"], context="robot.wheel_radius", positive=True
    )
    wheel_width = require_number(
        data["wheel_width"], context="robot.wheel_width", positive=True
    )
    geometric_track_width = require_number(
        data["geometric_track_width"],
        context="robot.geometric_track_width",
        positive=True,
    )
    effective_track_width = require_number(
        data["effective_track_width"],
        context="robot.effective_track_width",
        positive=True,
    )
    wheelbase = require_number(
        data["wheelbase"], context="robot.wheelbase", positive=True
    )
    base_mass = require_number(
        data["base_mass"], context="robot.base_mass", positive=True
    )
    wheel_mass = require_number(
        data["wheel_mass"], context="robot.wheel_mass", positive=True
    )
    nominal_total_mass = require_number(
        data["nominal_total_mass"],
        context="robot.nominal_total_mass",
        positive=True,
    )
    expected_total_mass = base_mass + 4.0 * wheel_mass
    if not math.isclose(
        nominal_total_mass,
        expected_total_mass,
        rel_tol=1e-9,
        abs_tol=1e-9,
    ):
        raise YamlConfigError(
            "robot.nominal_total_mass must equal base_mass + 4 * wheel_mass"
        )

    return RobotKinematicsConfig(
        schema_version=schema_version,
        kinematics_profile_id=profile_id,
        lifecycle=lifecycle,
        wheel_radius=wheel_radius,
        wheel_width=wheel_width,
        geometric_track_width=geometric_track_width,
        effective_track_width=effective_track_width,
        wheelbase=wheelbase,
        base_mass=base_mass,
        wheel_mass=wheel_mass,
        nominal_total_mass=nominal_total_mass,
    )
