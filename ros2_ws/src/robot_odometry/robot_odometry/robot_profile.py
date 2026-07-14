"""Strict robot-kinematics identity shared with the Isaac runtime."""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re

import yaml


_TOP_LEVEL_KEYS = frozenset({
    'schema_version',
    'name',
    'kinematics_profile_id',
    'lifecycle',
    'wheel_radius',
    'wheel_width',
    'geometric_track_width',
    'effective_track_width',
    'wheelbase',
    'base_mass',
    'wheel_mass',
    'nominal_total_mass',
    'physics',
    'wheel_joints',
    'controller',
    'frames',
    'footprint',
    'static_transforms',
})
_WHEEL_JOINT_KEYS = frozenset({
    'front_left', 'front_right', 'rear_left', 'rear_right'})
_CONTROLLER_KEYS = frozenset({
    'max_linear_speed',
    'max_angular_speed',
    'max_wheel_speed',
    'max_acceleration',
    'max_deceleration',
    'max_angular_acceleration',
})
_LIFECYCLES = frozenset({'stable_baseline', 'experimental_candidate'})
_PROFILE_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]*$')
_JOINT_NAME = re.compile(r'^[A-Za-z_][A-Za-z0-9_.-]*$')
ISAAC_KINEMATICS_PARAMETER_NAMES = (
    'runtime_provenance.schema_version',
    'runtime_provenance.robot.config.path',
    'runtime_provenance.robot.config.sha256',
    'runtime_provenance.robot.kinematics.profile_id',
    'runtime_provenance.robot.kinematics.lifecycle',
    'runtime_provenance.robot.kinematics.wheel_radius_m',
    'runtime_provenance.robot.kinematics.wheel_width_m',
    'runtime_provenance.robot.kinematics.geometric_track_width_m',
    'runtime_provenance.robot.kinematics.effective_track_width_m',
    'runtime_provenance.robot.kinematics.controller_contract_verified',
)


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f'duplicate YAML key: {key}')
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


@dataclass(frozen=True)
class RobotKinematicsProfile:
    """The wheel-odometry inputs and immutable source identity."""

    source: Path
    sha256: str
    profile_id: str
    lifecycle: str
    wheel_radius_m: float
    wheel_width_m: float
    geometric_track_width_m: float
    effective_track_width_m: float
    wheelbase_m: float
    base_mass_kg: float
    wheel_mass_kg: float
    nominal_total_mass_kg: float
    left_joint_names: tuple[str, ...]
    right_joint_names: tuple[str, ...]


@dataclass(frozen=True)
class IsaacKinematicsSnapshot:
    """Typed readback from the Isaac runtime provenance parameters."""

    schema_version: int
    config_path: Path
    config_sha256: str
    profile_id: str
    lifecycle: str
    wheel_radius_m: float
    wheel_width_m: float
    geometric_track_width_m: float
    effective_track_width_m: float
    controller_contract_verified: bool


def _mapping(value, location):
    if not isinstance(value, dict):
        raise ValueError(f'{location} must be a YAML mapping')
    return value


def _exact_keys(value, expected, location):
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        unknown = sorted(actual - expected)
        raise ValueError(
            f'{location} keys mismatch: missing={missing}, unknown={unknown}')


def _positive_number(value, location):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{location} must be a finite positive number')
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f'{location} must be a finite positive number')
    return parsed


def _joint_name(value, location):
    if not isinstance(value, str) or _JOINT_NAME.fullmatch(value) is None:
        raise ValueError(
            f'{location} must match [A-Za-z_][A-Za-z0-9_.-]*')
    return value


def load_robot_profile(path):
    """Load wheel kinematics from one schema-v2 robot YAML file."""
    source = Path(path).expanduser().resolve()
    raw = source.read_bytes()
    try:
        document = yaml.load(raw, Loader=_UniqueKeyLoader)
    except yaml.YAMLError as exc:
        raise ValueError(f'invalid robot config YAML: {exc}') from exc
    document = _mapping(document, 'robot config')
    _exact_keys(document, _TOP_LEVEL_KEYS, 'robot config')
    schema_version = document['schema_version']
    if (isinstance(schema_version, bool)
            or not isinstance(schema_version, int)
            or schema_version != 2):
        raise ValueError('schema_version must be integer 2')

    profile_id = document['kinematics_profile_id']
    if not isinstance(profile_id, str) or not _PROFILE_ID.fullmatch(profile_id):
        raise ValueError(
            'kinematics_profile_id must be a path-safe profile_id')
    lifecycle = document['lifecycle']
    if lifecycle not in _LIFECYCLES:
        raise ValueError(
            'lifecycle must be stable_baseline or experimental_candidate')

    wheel_radius = _positive_number(
        document['wheel_radius'], 'wheel_radius')
    wheel_width = _positive_number(
        document['wheel_width'], 'wheel_width')
    geometric_track_width = _positive_number(
        document['geometric_track_width'], 'geometric_track_width')
    effective_track_width = _positive_number(
        document['effective_track_width'], 'effective_track_width')
    wheelbase = _positive_number(document['wheelbase'], 'wheelbase')
    base_mass = _positive_number(document['base_mass'], 'base_mass')
    wheel_mass = _positive_number(document['wheel_mass'], 'wheel_mass')
    nominal_total_mass = _positive_number(
        document['nominal_total_mass'], 'nominal_total_mass')
    if not math.isclose(
            nominal_total_mass, base_mass + 4.0 * wheel_mass,
            rel_tol=1e-9, abs_tol=1e-9):
        raise ValueError(
            'nominal_total_mass must equal base_mass + 4 * wheel_mass')

    joints = _mapping(document['wheel_joints'], 'wheel_joints')
    _exact_keys(joints, _WHEEL_JOINT_KEYS, 'wheel_joints')
    joint_names = {
        key: _joint_name(value, f'wheel_joints.{key}')
        for key, value in joints.items()
    }
    if len(set(joint_names.values())) != len(joint_names):
        raise ValueError('wheel_joints values must be unique')

    controller = _mapping(document['controller'], 'controller')
    _exact_keys(controller, _CONTROLLER_KEYS, 'controller')
    for key, value in controller.items():
        _positive_number(value, f'controller.{key}')

    return RobotKinematicsProfile(
        source=source,
        sha256=hashlib.sha256(raw).hexdigest(),
        profile_id=profile_id,
        lifecycle=lifecycle,
        wheel_radius_m=wheel_radius,
        wheel_width_m=wheel_width,
        geometric_track_width_m=geometric_track_width,
        effective_track_width_m=effective_track_width,
        wheelbase_m=wheelbase,
        base_mass_kg=base_mass,
        wheel_mass_kg=wheel_mass,
        nominal_total_mass_kg=nominal_total_mass,
        left_joint_names=(
            joint_names['front_left'], joint_names['rear_left']),
        right_joint_names=(
            joint_names['front_right'], joint_names['rear_right']),
    )


def validate_isaac_kinematics(profile, parameters):
    """Parse and validate one Isaac runtime kinematics readback."""
    _exact_keys(
        parameters,
        frozenset(ISAAC_KINEMATICS_PARAMETER_NAMES),
        'Isaac kinematics parameter',
    )

    try:
        current_sha256 = hashlib.sha256(profile.source.read_bytes()).hexdigest()
    except OSError as exc:
        raise ValueError(
            f'cannot re-read local robot config during handshake: {exc}'
        ) from exc
    if current_sha256 != profile.sha256:
        raise ValueError(
            'robot config changed after local loading and before Isaac '
            'kinematics verification')

    schema_version = parameters['runtime_provenance.schema_version']
    if (isinstance(schema_version, bool)
            or not isinstance(schema_version, int)
            or schema_version != 5):
        raise ValueError(
            'runtime_provenance.schema_version must be integer 5')

    sha256 = parameters['runtime_provenance.robot.config.sha256']
    if (not isinstance(sha256, str)
            or re.fullmatch(r'[0-9a-f]{64}', sha256) is None):
        raise ValueError(
            'runtime_provenance.robot.config.sha256 must be lowercase SHA256')
    if sha256 != profile.sha256:
        raise ValueError(
            'runtime_provenance.robot.config.sha256 does not match local '
            'robot config')

    raw_path = parameters['runtime_provenance.robot.config.path']
    if not isinstance(raw_path, str) or not raw_path:
        raise ValueError(
            'runtime_provenance.robot.config.path must be a non-empty string')
    config_path = Path(raw_path).expanduser().resolve()
    if config_path != profile.source:
        raise ValueError(
            'runtime_provenance.robot.config.path does not match local '
            'robot config')

    expected = {
        'runtime_provenance.robot.kinematics.profile_id': profile.profile_id,
        'runtime_provenance.robot.kinematics.lifecycle': profile.lifecycle,
        'runtime_provenance.robot.kinematics.wheel_radius_m': (
            profile.wheel_radius_m),
        'runtime_provenance.robot.kinematics.wheel_width_m': (
            profile.wheel_width_m),
        'runtime_provenance.robot.kinematics.geometric_track_width_m': (
            profile.geometric_track_width_m),
        'runtime_provenance.robot.kinematics.effective_track_width_m': (
            profile.effective_track_width_m),
    }
    for name, expected_value in expected.items():
        actual = parameters[name]
        if isinstance(expected_value, float):
            if not isinstance(actual, float) or not math.isfinite(actual):
                raise ValueError(f'{name} must be a finite ROS double')
        elif not isinstance(actual, str):
            raise ValueError(f'{name} must be a ROS string')
        if actual != expected_value:
            raise ValueError(
                f'{name} does not match local robot config: '
                f'local={expected_value!r}, Isaac={actual!r}')

    verified_name = (
        'runtime_provenance.robot.kinematics.controller_contract_verified')
    controller_verified = parameters[verified_name]
    if controller_verified is not True:
        raise ValueError(f'{verified_name} must be boolean true')

    return IsaacKinematicsSnapshot(
        schema_version=schema_version,
        config_path=config_path,
        config_sha256=sha256,
        profile_id=parameters[
            'runtime_provenance.robot.kinematics.profile_id'],
        lifecycle=parameters[
            'runtime_provenance.robot.kinematics.lifecycle'],
        wheel_radius_m=parameters[
            'runtime_provenance.robot.kinematics.wheel_radius_m'],
        wheel_width_m=parameters[
            'runtime_provenance.robot.kinematics.wheel_width_m'],
        geometric_track_width_m=parameters[
            'runtime_provenance.robot.kinematics.geometric_track_width_m'],
        effective_track_width_m=parameters[
            'runtime_provenance.robot.kinematics.effective_track_width_m'],
        controller_contract_verified=controller_verified,
    )
