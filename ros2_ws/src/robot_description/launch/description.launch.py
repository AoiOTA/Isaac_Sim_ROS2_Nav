import math
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml


_PHYSICAL_GEOMETRY_KEYS = (
    'wheel_radius',
    'wheel_width',
    'geometric_track_width',
    'wheelbase',
    'base_mass',
    'wheel_mass',
)
_ROBOT_CONFIG_FIELDS = frozenset({
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
_PROFILE_ID_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]*')
_LIFECYCLES = frozenset({'stable_baseline', 'experimental_candidate'})
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
_JOINT_NAME_PATTERN = re.compile(r'[A-Za-z_][A-Za-z0-9_.-]*')
# jackal_base.xacro intentionally preserves the measured legacy inertia
# tensors. They are not safely derivable from this schema's limited geometry.
_FIXED_JACKAL_INERTIA_INPUTS = {
    'wheel_radius': 0.098,
    'wheel_width': 0.040,
    'base_mass': 17.0,
    'wheel_mass': 0.477,
}


class _UniqueKeySafeLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects silently shadowed mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise yaml.constructor.ConstructorError(
                'while constructing a mapping',
                node.start_mark,
                f'duplicate YAML mapping key: {key!r}',
                key_node.start_mark,
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _require_positive_number(config, key):
    value = config[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f'robot_config_file {key} must be a number')
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise RuntimeError(
            f'robot_config_file {key} must be finite and greater than 0')
    return number


def _require_exact_mapping(config, key, expected_keys):
    value = config[key]
    if not isinstance(value, dict):
        raise RuntimeError(f'robot_config_file {key} must be a mapping')
    actual_keys = set(value)
    if actual_keys != expected_keys:
        raise RuntimeError(
            f'robot_config_file {key} keys mismatch: '
            f'missing={sorted(expected_keys - actual_keys)}, '
            f'unknown={sorted(actual_keys - expected_keys)}')
    return value


def _load_robot_geometry(robot_config_file):
    config_path = Path(robot_config_file).expanduser()
    if not config_path.exists():
        raise RuntimeError(
            f'robot_config_file does not exist: {config_path}')
    if not config_path.is_file():
        raise RuntimeError(
            f'robot_config_file is not a regular file: {config_path}')

    try:
        with config_path.open(encoding='utf-8') as stream:
            config = yaml.load(stream, Loader=_UniqueKeySafeLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise RuntimeError(
            f'failed to read robot_config_file {config_path}: {exc}') from exc

    if not isinstance(config, dict):
        raise RuntimeError('robot_config_file root must be a mapping')
    unknown_fields = sorted(set(config) - _ROBOT_CONFIG_FIELDS)
    if unknown_fields:
        raise RuntimeError(
            'robot_config_file has unknown top-level keys: '
            + ', '.join(unknown_fields))
    missing_fields = sorted(_ROBOT_CONFIG_FIELDS - set(config))
    if missing_fields:
        raise RuntimeError(
            'robot_config_file is missing top-level keys: '
            + ', '.join(missing_fields))
    if type(config.get('schema_version')) is not int \
            or config['schema_version'] != 2:
        raise RuntimeError('robot_config_file schema_version must be integer 2')

    profile_id = config['kinematics_profile_id']
    if not isinstance(profile_id, str) \
            or _PROFILE_ID_PATTERN.fullmatch(profile_id) is None:
        raise RuntimeError(
            'robot_config_file kinematics_profile_id must match '
            '[A-Za-z0-9][A-Za-z0-9_.-]*')
    lifecycle = config['lifecycle']
    if not isinstance(lifecycle, str) or lifecycle not in _LIFECYCLES:
        raise RuntimeError(
            'robot_config_file lifecycle must be one of stable_baseline, '
            'experimental_candidate')
    geometry = {
        key: _require_positive_number(config, key)
        for key in _PHYSICAL_GEOMETRY_KEYS
    }
    _require_positive_number(config, 'effective_track_width')
    nominal_total_mass = _require_positive_number(
        config, 'nominal_total_mass')
    expected_total_mass = geometry['base_mass'] + 4.0 * geometry['wheel_mass']
    if not math.isclose(
            nominal_total_mass, expected_total_mass,
            rel_tol=1e-9, abs_tol=1e-9):
        raise RuntimeError(
            'robot_config_file nominal_total_mass must equal '
            'base_mass + 4 * wheel_mass')

    wheel_joints = _require_exact_mapping(
        config, 'wheel_joints', _WHEEL_JOINT_KEYS)
    parsed_joints = {}
    for position, joint_name in wheel_joints.items():
        if not isinstance(joint_name, str) \
                or _JOINT_NAME_PATTERN.fullmatch(joint_name) is None:
            raise RuntimeError(
                f'robot_config_file wheel_joints.{position} must match '
                '[A-Za-z_][A-Za-z0-9_.-]*')
        parsed_joints[position] = joint_name
    if len(set(parsed_joints.values())) != 4:
        raise RuntimeError(
            'robot_config_file wheel_joints values must be unique')

    controller = _require_exact_mapping(
        config, 'controller', _CONTROLLER_KEYS)
    for key in _CONTROLLER_KEYS:
        _require_positive_number(controller, key)

    if config['name'] != 'jackal':
        raise RuntimeError(
            'robot_description uses fixed Jackal inertia and requires '
            'robot_config_file name=jackal')
    for key, expected in _FIXED_JACKAL_INERTIA_INPUTS.items():
        if not math.isclose(
                geometry[key], expected, rel_tol=0.0, abs_tol=1e-12):
            raise RuntimeError(
                'robot_description fixed Jackal inertia is incompatible '
                f'with {key}={geometry[key]!r}; expected {expected!r}')

    geometry.update({
        f'{position}_joint_name': joint_name
        for position, joint_name in parsed_joints.items()
    })
    return geometry


def _launch_setup(context):
    publish_tf = LaunchConfiguration('publish_tf').perform(context).lower()
    if publish_tf not in {'true', 'false'}:
        raise RuntimeError('publish_tf must be true or false')
    robot_config_file = LaunchConfiguration(
        'robot_config_file').perform(context)
    geometry = _load_robot_geometry(robot_config_file)
    prefix = LaunchConfiguration('prefix')
    xacro_file = LaunchConfiguration('xacro_file')
    xacro_command = ['xacro', ' ', xacro_file, ' prefix:=', prefix]
    for key in _PHYSICAL_GEOMETRY_KEYS:
        xacro_command.extend([
            f' {key}:=',
            format(geometry[key], '.17g'),
        ])
    for position in sorted(_WHEEL_JOINT_KEYS):
        key = f'{position}_joint_name'
        xacro_command.extend([f' {key}:=', geometry[key]])
    robot_description = ParameterValue(
        Command(xacro_command),
        value_type=str,
    )

    if publish_tf == 'true':
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        )]
    return [Node(
        package='robot_description',
        executable='robot_description_publisher',
        name='robot_description_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )]


def generate_launch_description():
    description_share = Path(get_package_share_directory('robot_description'))
    default_xacro_file = description_share / 'urdf' / 'jackal.urdf.xacro'

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument(
            'xacro_file', default_value=str(default_xacro_file)),
        DeclareLaunchArgument(
            'robot_config_file',
            description=(
                'Absolute path to the schema v2 robot YAML that owns the '
                'physical wheel geometry and masses')),
        DeclareLaunchArgument(
            'publish_tf',
            default_value='true',
            description=(
                'Use robot_state_publisher when true; publish only the '
                'description topic when false')),
        OpaqueFunction(function=_launch_setup),
    ])
