"""Pure mode validation shared by launch and unit tests."""

from dataclasses import dataclass
from pathlib import Path


OPERATIONS = frozenset({
    'mapping', 'incremental_mapping', 'localization', 'navigation'})
ODOMETRY_MODES = frozenset({'ideal', 'realistic'})
STRUCTURE_TF_SOURCES = frozenset({'isaac', 'rsp'})
NAV2_PROFILES = frozenset({'stable', 'performance'})


@dataclass(frozen=True)
class ModeSelection:
    """Normalized and validated bringup mode selection."""

    operation: str
    odometry_mode: str
    structure_tf_source: str
    posegraph_prefix: str
    occupancy_map_file: str


@dataclass(frozen=True)
class RobotRuntimeFiles:
    """Robot-specific files that may change without changing stack topology."""

    description_file: str
    wheel_odometry_params_file: str
    nav2_params_file: str


def validate_robot_runtime_files(
    description_file,
    wheel_odometry_params_file,
    nav2_params_file,
    check_files=True,
):
    """Validate the three ROS migration seams before launching any nodes."""
    values = {
        'description_file': description_file.strip(),
        'wheel_odometry_params_file': wheel_odometry_params_file.strip(),
        'nav2_params_file': nav2_params_file.strip(),
    }
    for name, value in values.items():
        if not value:
            raise ValueError(f'{name} must not be empty')
        if check_files and not Path(value).is_file():
            raise ValueError(f'{name} does not exist: {value}')
    if not values['description_file'].endswith(('.urdf', '.xacro')):
        raise ValueError('description_file must be a .urdf or .xacro file')
    for name in ('wheel_odometry_params_file', 'nav2_params_file'):
        if not values[name].endswith(('.yaml', '.yml')):
            raise ValueError(f'{name} must be a YAML file')
    return RobotRuntimeFiles(**values)


def posegraph_prefix(value):
    """Normalize a SLAM Toolbox serialized map path to its prefix."""
    normalized = value.strip()
    for suffix in ('.posegraph', '.data'):
        if normalized.endswith(suffix):
            return normalized[:-len(suffix)]
    return normalized


def validate_nav2_profile(value):
    """Normalize the bounded Nav2 overlay selected at launch."""
    profile = value.strip().lower()
    _require_choice('nav2_profile', profile, NAV2_PROFILES)
    return profile


def validate_mode(
    operation,
    odometry_mode,
    structure_tf_source,
    posegraph_file='',
    map_file='',
    check_posegraph_files=True,
):
    """Reject combinations that violate TF and SLAM ownership contracts."""
    operation = operation.strip().lower()
    odometry_mode = odometry_mode.strip().lower()
    structure_tf_source = structure_tf_source.strip().lower()
    prefix = posegraph_prefix(posegraph_file)
    occupancy_map = map_file.strip()

    _require_choice('operation', operation, OPERATIONS)
    _require_choice('odometry_mode', odometry_mode, ODOMETRY_MODES)
    _require_choice(
        'structure_tf_source', structure_tf_source, STRUCTURE_TF_SOURCES)

    if odometry_mode == 'ideal' and structure_tf_source == 'rsp':
        raise ValueError(
            'ideal odometry is an Isaac-owned mode; structure_tf_source=rsp '
            'is reserved for the realistic/standard ROS ownership mode')

    if operation == 'mapping' and prefix:
        raise ValueError(
            'posegraph_file is a localization input and must be empty in '
            'baseline mapping mode')
    if operation in {'incremental_mapping', 'localization', 'navigation'}:
        if not prefix:
            raise ValueError(
                f'posegraph_file is required for {operation} mode')
        if check_posegraph_files:
            missing = [
                prefix + suffix
                for suffix in ('.posegraph', '.data')
                if not Path(prefix + suffix).is_file()
            ]
            if missing:
                raise ValueError(
                    'serialized pose graph is incomplete; missing: '
                    + ', '.join(missing))
    if operation in {'localization', 'navigation'}:
        if not occupancy_map:
            raise ValueError(
                f'map_file is required for {operation} mode')
        if check_posegraph_files and not Path(occupancy_map).is_file():
            raise ValueError(
                f'occupancy map YAML does not exist: {occupancy_map}')
    elif occupancy_map:
        raise ValueError(
            f'map_file is not an input to {operation} mode')

    return ModeSelection(
        operation=operation,
        odometry_mode=odometry_mode,
        structure_tf_source=structure_tf_source,
        posegraph_prefix=prefix,
        occupancy_map_file=occupancy_map,
    )


def _require_choice(name, value, choices):
    if value not in choices:
        expected = ', '.join(sorted(choices))
        raise ValueError(f'{name} must be one of [{expected}], got {value!r}')
