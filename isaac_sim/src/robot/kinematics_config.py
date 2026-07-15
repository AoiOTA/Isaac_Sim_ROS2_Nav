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
        "mass_collision_profile",
        "wheel_velocity_drive",
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
_WHEEL_VELOCITY_DRIVE_FIELDS = frozenset(
    {
        "schema_version",
        "profile_id",
        "drive_type",
        "stiffness_n_m_per_rad",
        "damping_n_m_s_per_rad",
        "max_effort_n_m",
        "max_joint_velocity_rad_s",
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
class RobotWheelVelocityDrive:
    schema_version: int
    profile_id: str
    drive_type: str
    stiffness_n_m_per_rad: float
    damping_n_m_s_per_rad: float
    max_effort_n_m: float
    max_joint_velocity_rad_s: float


@dataclass(frozen=True)
class RobotConfigContract:
    schema_version: int
    kinematics: RobotKinematicsConfig
    wheel_joints: RobotWheelJoints
    controller: RobotControllerConfig
    mass_collision_profile: Path
    wheel_velocity_drive: RobotWheelVelocityDrive


def load_robot_config_contract(path: str | Path) -> RobotConfigContract:
    """Load and validate the complete robot/controller schema-v3 contract."""

    config_path = Path(path).resolve()
    contract = parse_robot_config_contract(
        load_mapping(config_path),
        config_path=config_path,
    )
    if not contract.mass_collision_profile.is_file():
        raise YamlConfigError(
            "robot.mass_collision_profile must reference an existing regular "
            f"file: {contract.mass_collision_profile}"
        )
    return contract


def load_robot_kinematics_config(
    path: str | Path,
) -> RobotKinematicsConfig:
    """Load validated kinematics from a complete robot schema-v3 YAML file."""

    return load_robot_config_contract(path).kinematics


def parse_robot_config_contract(
    data: Mapping[str, Any],
    *,
    config_path: str | Path | None = None,
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

    parsed_controller = RobotControllerConfig(
        max_linear_speed=controller_number("max_linear_speed"),
        max_angular_speed=controller_number("max_angular_speed"),
        max_wheel_speed=controller_number("max_wheel_speed"),
        max_acceleration=controller_number("max_acceleration"),
        max_deceleration=controller_number("max_deceleration"),
        max_angular_acceleration=controller_number(
            "max_angular_acceleration"
        ),
    )

    drive = data["wheel_velocity_drive"]
    if not isinstance(drive, dict):
        raise YamlConfigError("robot.wheel_velocity_drive must be a mapping")
    reject_unknown(
        drive,
        _WHEEL_VELOCITY_DRIVE_FIELDS,
        context="robot.wheel_velocity_drive",
    )
    require_keys(
        drive,
        _WHEEL_VELOCITY_DRIVE_FIELDS,
        context="robot.wheel_velocity_drive",
    )

    drive_schema_version = drive["schema_version"]
    if (
        isinstance(drive_schema_version, bool)
        or not isinstance(drive_schema_version, int)
        or drive_schema_version != 1
    ):
        raise YamlConfigError(
            "robot.wheel_velocity_drive.schema_version must be integer 1"
        )

    drive_profile_id = drive["profile_id"]
    if (
        not isinstance(drive_profile_id, str)
        or _PROFILE_ID_PATTERN.fullmatch(drive_profile_id) is None
    ):
        raise YamlConfigError(
            "robot.wheel_velocity_drive.profile_id must match "
            "[A-Za-z0-9][A-Za-z0-9_.-]*"
        )

    drive_type = drive["drive_type"]
    if not isinstance(drive_type, str) or drive_type != "force":
        raise YamlConfigError(
            "robot.wheel_velocity_drive.drive_type must equal force"
        )

    stiffness = require_number(
        drive["stiffness_n_m_per_rad"],
        context=(
            "robot.wheel_velocity_drive.stiffness_n_m_per_rad"
        ),
    )
    if stiffness != 0.0:
        raise YamlConfigError(
            "robot.wheel_velocity_drive.stiffness_n_m_per_rad must equal 0"
        )

    def positive_drive_number(field: str) -> float:
        return require_number(
            drive[field],
            context=f"robot.wheel_velocity_drive.{field}",
            positive=True,
        )

    parsed_drive = RobotWheelVelocityDrive(
        schema_version=drive_schema_version,
        profile_id=drive_profile_id,
        drive_type=drive_type,
        stiffness_n_m_per_rad=stiffness,
        damping_n_m_s_per_rad=positive_drive_number(
            "damping_n_m_s_per_rad"
        ),
        max_effort_n_m=positive_drive_number("max_effort_n_m"),
        max_joint_velocity_rad_s=positive_drive_number(
            "max_joint_velocity_rad_s"
        ),
    )
    if (
        parsed_controller.max_wheel_speed
        > parsed_drive.max_joint_velocity_rad_s
    ):
        raise YamlConfigError(
            "robot.controller.max_wheel_speed must be less than or equal to "
            "robot.wheel_velocity_drive.max_joint_velocity_rad_s"
        )

    mass_collision_profile = data["mass_collision_profile"]
    if (
        not isinstance(mass_collision_profile, str)
        or not mass_collision_profile.strip()
    ):
        raise YamlConfigError(
            "robot.mass_collision_profile must be a nonempty path string"
    )
    parsed_mass_collision_profile = Path(mass_collision_profile)
    if (
        config_path is not None
        and not parsed_mass_collision_profile.is_absolute()
    ):
        parsed_mass_collision_profile = (
            Path(config_path).resolve().parent / parsed_mass_collision_profile
        )
    parsed_mass_collision_profile = parsed_mass_collision_profile.resolve()

    return RobotConfigContract(
        schema_version=kinematics.schema_version,
        kinematics=kinematics,
        wheel_joints=parsed_joints,
        controller=parsed_controller,
        mass_collision_profile=parsed_mass_collision_profile,
        wheel_velocity_drive=parsed_drive,
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
        or schema_version != 3
    ):
        raise YamlConfigError("robot config schema_version must be integer 3")

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
