"""Pure mode validation shared by launch and unit tests."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml

from .map_manifest import load_map_manifest
from .map_manifest import MapManifestError
from .map_manifest import validate_initial_pose_contract


OPERATIONS = frozenset({
    'mapping', 'incremental_mapping', 'localization', 'navigation'})
ODOMETRY_MODES = frozenset({'ideal', 'realistic'})
STRUCTURE_TF_SOURCES = frozenset({'isaac', 'rsp'})
LOCALIZATION_BACKENDS = frozenset({'ideal', 'amcl', 'slam_toolbox'})
NAV2_PROFILES = frozenset({
    'stable', 'performance', 'dynamic_avoidance', 'bio_nav_planning_only',
    'bio_nav_risk_only', 'bio_nav_tiebreak_risk',
    'attempt21_static_collection',
    'attempt22_reachability_shadow',
    'attempt23_global_prior',
    'attempt23_static_observer',
    'bio_nav_rgbd_risk_shadow', 'bio_nav_rgbd_risk_ab',
    'bio_nav_rgbd_risk_static_opt_in'})


@dataclass(frozen=True)
class ModeSelection:
    """Normalized and validated bringup mode selection."""

    operation: str
    odometry_mode: str
    structure_tf_source: str
    posegraph_prefix: str
    occupancy_map_file: str
    map_manifest_file: str = ''
    map_version: str = ''
    map_bundle_sha256: str = ''
    localization_backend: str = ''


@dataclass(frozen=True)
class RobotRuntimeFiles:
    """Robot-specific files that may change without changing stack topology."""

    description_file: str
    wheel_odometry_params_file: str
    nav2_params_file: str


@dataclass(frozen=True)
class Nav2ControllerProfile:
    """Validated MPPI timing and workload values from a Nav2 overlay."""

    source: Path
    controller_frequency: float
    model_dt: float
    time_steps: int
    batch_size: int

    @property
    def controller_period(self):
        return 1.0 / self.controller_frequency


def _profile_mapping(value, location):
    if not isinstance(value, dict):
        raise ValueError(f'{location} must be a YAML mapping')
    return value


def _positive_profile_number(value, location):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{location} must be a finite positive number')
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise ValueError(f'{location} must be a finite positive number')
    return parsed


def _positive_profile_integer(value, location):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f'{location} must be a positive integer')
    return value


def validate_nav2_profile_params_file(path):
    """Parse and validate an MPPI overlay before launch creates ROS nodes.

    Nav2 1.3.12 aborts the controller process when the configured control
    period is longer than MPPI's model time step.  Keeping this check in the
    pure launch contract turns that late process abort into a deterministic
    configuration error.
    """
    source = Path(path).expanduser()
    if source.suffix not in {'.yaml', '.yml'}:
        raise ValueError(
            'nav2_profile_params_file must be a YAML file: '
            f'{source}')
    if not source.is_file():
        raise ValueError(
            'nav2_profile_params_file does not exist: '
            f'{source}')
    try:
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
    except OSError as exc:
        raise ValueError(
            f'cannot read nav2_profile_params_file {source}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise ValueError(
            f'invalid YAML in nav2_profile_params_file {source}: {exc}'
        ) from exc

    root = _profile_mapping(document, 'nav2 profile root')
    controller = _profile_mapping(
        root.get('controller_server'), 'controller_server')
    parameters = _profile_mapping(
        controller.get('ros__parameters'),
        'controller_server.ros__parameters',
    )
    follow_path = _profile_mapping(
        parameters.get('FollowPath'),
        'controller_server.ros__parameters.FollowPath',
    )
    controller_frequency = _positive_profile_number(
        parameters.get('controller_frequency'),
        'controller_server.ros__parameters.controller_frequency',
    )
    model_dt = _positive_profile_number(
        follow_path.get('model_dt'),
        'controller_server.ros__parameters.FollowPath.model_dt',
    )
    time_steps = _positive_profile_integer(
        follow_path.get('time_steps'),
        'controller_server.ros__parameters.FollowPath.time_steps',
    )
    batch_size = _positive_profile_integer(
        follow_path.get('batch_size'),
        'controller_server.ros__parameters.FollowPath.batch_size',
    )

    controller_period = 1.0 / controller_frequency
    tolerance = max(1.0e-12, abs(model_dt) * 1.0e-12)
    if controller_period > model_dt + tolerance:
        minimum_frequency = 1.0 / model_dt
        raise ValueError(
            'Nav2 1.3.12 MPPI timing constraint violated: controller period '
            f'1/controller_frequency={controller_period:.6f}s exceeds '
            f'FollowPath.model_dt={model_dt:.6f}s '
            f'(controller_frequency={controller_frequency:g}Hz). '
            'The controller period must be <= model_dt; set '
            f'controller_frequency to at least {minimum_frequency:g}Hz or '
            'increase model_dt.')

    return Nav2ControllerProfile(
        source=source.resolve(),
        controller_frequency=controller_frequency,
        model_dt=model_dt,
        time_steps=time_steps,
        batch_size=batch_size,
    )


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


def resolve_localization_backend(odometry_mode, localization_backend=''):
    """Fill an empty localization backend from the odometry mode.

    Ideal odometry pairs with ideal localization; realistic odometry keeps
    the historical SLAM Toolbox pose-graph localization default, so legacy
    callers that never pass a backend keep their behavior.
    """
    backend = localization_backend.strip().lower()
    if backend:
        return backend
    return 'ideal' if odometry_mode == 'ideal' else 'slam_toolbox'


def validate_mode(
    operation,
    odometry_mode,
    structure_tf_source,
    posegraph_file='',
    map_file='',
    check_posegraph_files=True,
    map_manifest_file='',
    project_root='',
    initial_pose_source='auto',
    spawn_poses_file='',
    spawn_pose_name='mapping_start',
    localization_backend='',
):
    """Reject combinations that violate TF and SLAM ownership contracts.

    The localization backend selects the map->odom owner: ideal publishes a
    fresh transform through ideal_localization_tf, amcl localizes the
    estimated odometry against the occupancy map, and slam_toolbox keeps the
    serialized pose-graph localization.  An empty backend derives from the
    odometry mode (ideal->ideal, realistic->slam_toolbox).
    """
    operation = operation.strip().lower()
    odometry_mode = odometry_mode.strip().lower()
    structure_tf_source = structure_tf_source.strip().lower()
    backend = resolve_localization_backend(
        odometry_mode, localization_backend)
    prefix = posegraph_prefix(posegraph_file)
    occupancy_map = map_file.strip()
    manifest_path = map_manifest_file.strip()

    _require_choice('operation', operation, OPERATIONS)
    _require_choice('odometry_mode', odometry_mode, ODOMETRY_MODES)
    _require_choice(
        'structure_tf_source', structure_tf_source, STRUCTURE_TF_SOURCES)
    _require_choice(
        'localization_backend', backend, LOCALIZATION_BACKENDS)

    if odometry_mode == 'ideal' and structure_tf_source == 'rsp':
        raise ValueError(
            'ideal odometry is an Isaac-owned mode; structure_tf_source=rsp '
            'is reserved for the realistic/standard ROS ownership mode')

    if odometry_mode == 'ideal' and backend != 'ideal':
        raise ValueError(
            f'localization_backend={backend} requires realistic odometry; '
            'ideal odometry owns map->odom through ideal_localization_tf')

    if operation == 'mapping' and prefix:
        raise ValueError(
            'posegraph_file is a localization input and must be empty in '
            'baseline mapping mode')
    saved_map_operation = operation in {
        'incremental_mapping', 'localization', 'navigation'}
    # AMCL consumes the occupancy map and laser scan only; the serialized
    # pose graph remains a SLAM Toolbox input.  Mapping operations still
    # require it because they resume SLAM Toolbox mapping from it.
    amcl_localization = (
        backend == 'amcl' and operation in {'localization', 'navigation'})
    if saved_map_operation:
        if amcl_localization:
            if prefix:
                raise ValueError(
                    'posegraph_file is a SLAM Toolbox input and must be '
                    'empty with localization_backend=amcl')
        else:
            if not prefix:
                raise ValueError(
                    f'posegraph_file is required for {operation} mode with '
                    f'localization_backend={backend}')
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

    map_version = ''
    map_bundle_sha256 = ''
    if saved_map_operation and check_posegraph_files \
            and not amcl_localization:
        if not manifest_path:
            version = Path(prefix).name
            map_root = Path(prefix).parent.parent
            manifest_path = str(map_root / 'manifests' / f'{version}.yaml')
        try:
            manifest = load_map_manifest(
                manifest_path,
                project_root=project_root or None,
            )
        except MapManifestError as exc:
            raise ValueError(str(exc)) from exc
        requested_prefix = Path(prefix).expanduser().absolute()
        if requested_prefix != manifest.posegraph_prefix:
            raise ValueError(
                'posegraph_file does not match map manifest: '
                f'{prefix} != {manifest.posegraph_prefix}')
        requested_occupancy = (
            Path(occupancy_map).expanduser().absolute()
            if occupancy_map else None
        )
        if requested_occupancy is not None \
                and requested_occupancy != manifest.occupancy_yaml:
            raise ValueError(
                'map_file does not match map manifest: '
                f'{occupancy_map} != {manifest.occupancy_yaml}')
        try:
            validate_initial_pose_contract(
                manifest,
                initial_pose_source=initial_pose_source,
                spawn_poses_file=spawn_poses_file,
                spawn_pose_name=spawn_pose_name,
            )
        except MapManifestError as exc:
            raise ValueError(str(exc)) from exc
        manifest_path = str(manifest.source)
        prefix = str(manifest.posegraph_prefix)
        if occupancy_map:
            occupancy_map = str(manifest.occupancy_yaml)
        map_version = manifest.map_version
        map_bundle_sha256 = manifest.bundle_sha256

    return ModeSelection(
        operation=operation,
        odometry_mode=odometry_mode,
        structure_tf_source=structure_tf_source,
        posegraph_prefix=prefix,
        occupancy_map_file=occupancy_map,
        map_manifest_file=manifest_path,
        map_version=map_version,
        map_bundle_sha256=map_bundle_sha256,
        localization_backend=backend,
    )


def _require_choice(name, value, choices):
    if value not in choices:
        expected = ', '.join(sorted(choices))
        raise ValueError(f'{name} must be one of [{expected}], got {value!r}')
