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
ODOMETRY_MODES = frozenset({'ideal', 'realistic', 'estimated'})
STRUCTURE_TF_SOURCES = frozenset({'isaac', 'rsp'})
LOCALIZATION_MAP_CONTRACTS = frozenset({
    'posegraph_bundle', 'occupancy_only'})
LOCALIZATION_OWNERS = frozenset({'auto', 'ideal', 'grid'})
NAV2_PROFILES = frozenset({
    'stable', 'performance', 'dynamic_avoidance', 'bio_nav_planning_only',
    'v6_low_obstacle_isolation',
    'bio_nav_risk_only', 'bio_nav_tiebreak_risk',
    'attempt21_static_collection',
    'attempt22_reachability_shadow',
    'attempt23_global_prior',
    'attempt23_static_observer',
    'bio_nav_rgbd_risk_shadow', 'bio_nav_rgbd_risk_ab',
    'bio_nav_rgbd_risk_static_opt_in'})
COGNITIVE_PROFILES = frozenset({'M0', 'M1', 'M2', 'M3'})
COGNITIVE_GRAPH_MODES = frozenset({'gvg', 'shadow', 'hybrid', 'primary'})
_COGNITIVE_PROFILE_CONTRACT = {
    'M0': ('off', 'off', False),
    'M1': ('shadow', 'shadow', True),
    'M2': ('active', 'off', True),
    'M3': ('active', 'active', True),
}
_A21_COGNITIVE_CRITICS = (
    'ConstraintCritic', 'CostCritic', 'GoalCritic', 'PathAlignCritic',
    'PathFollowCritic', 'PathAngleCritic',
    'VelocityDeadbandCritic', 'CognitiveRiskCritic',
)


@dataclass(frozen=True)
class ModeSelection:
    """Normalized and validated bringup mode selection."""

    operation: str
    odometry_mode: str
    structure_tf_source: str
    posegraph_prefix: str
    occupancy_map_file: str
    localization_map_contract: str
    localization_owner: str
    route_graph_file: str
    map_manifest_file: str = ''
    map_version: str = ''
    map_bundle_sha256: str = ''


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


@dataclass(frozen=True)
class CognitiveProfile:
    """Executable Module2 local-planning write contract."""

    name: str
    obstacle_layer_mode: str
    risk_critic_mode: str
    module2_enabled: bool


def validate_cognitive_profile(value, modes_file):
    """Load M0--M3 from the shipped contract and reject semantic drift."""
    name = value.strip().upper()
    _require_choice('cognitive_profile', name, COGNITIVE_PROFILES)
    source = Path(modes_file).expanduser()
    if not source.is_file():
        raise ValueError(f'cognitive modes file does not exist: {source}')
    try:
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'invalid cognitive modes file {source}: {exc}') from exc
    root = _profile_mapping(document, 'modes root')
    profiles = _profile_mapping(
        root.get('cognitive_profiles'), 'cognitive_profiles')
    configured = _profile_mapping(
        profiles.get(name), f'cognitive_profiles.{name}')
    module2_enabled = configured.get('module2_enabled')
    if not isinstance(module2_enabled, bool):
        raise ValueError(
            f'cognitive_profiles.{name}.module2_enabled must be boolean')
    actual = (
        str(configured.get('obstacle_layer_mode', '')).strip().lower(),
        str(configured.get('risk_critic_mode', '')).strip().lower(),
        module2_enabled,
    )
    expected = _COGNITIVE_PROFILE_CONTRACT[name]
    if actual != expected:
        raise ValueError(
            f'cognitive_profiles.{name} violates M0-M3 contract: '
            f'expected {expected}, got {actual}')
    return CognitiveProfile(name, *actual)


def validate_cognitive_graph_mode(value):
    """Validate the graph experiment arm independently from M0--M3."""
    mode = value.strip().lower()
    _require_choice('cognitive_graph_mode', mode, COGNITIVE_GRAPH_MODES)
    return mode


def cognitive_nav2_parameters(profile):
    """
    Return the Phase-1 exact-node parameters for M0--M3.

    Phase 1 may subscribe in shadow, but it must not let an M2/M3 selection
    restore active cognitive writes before the base V6-GRID loop is closed.
    """
    obstacle_mode = (
        'shadow' if profile.obstacle_layer_mode == 'active'
        else profile.obstacle_layer_mode
    )
    critic_mode = (
        'shadow' if profile.risk_critic_mode == 'active'
        else profile.risk_critic_mode
    )
    return {
        'controller_server': {
            'ros__parameters': {
                'FollowPath': {
                    'critics': list(_A21_COGNITIVE_CRITICS),
                    'CognitiveRiskCritic': {
                        'mode': critic_mode,
                    },
                },
            },
        },
        'local_costmap': {
            'local_costmap': {
                'ros__parameters': {
                    'cognitive_obstacle_layer': {
                        'mode': obstacle_mode,
                    },
                },
            },
        },
        'global_costmap': {
            'global_costmap': {
                'ros__parameters': {
                    'cognitive_obstacle_layer': {
                        'mode': obstacle_mode,
                    },
                },
            },
        },
    }


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
    """
    Parse and validate an MPPI overlay before launch creates ROS nodes.

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
    localization_map_contract='posegraph_bundle',
    localization_owner='auto',
    route_graph_file='',
):
    """Reject combinations that violate TF and SLAM ownership contracts."""
    operation = operation.strip().lower()
    odometry_mode = odometry_mode.strip().lower()
    structure_tf_source = structure_tf_source.strip().lower()
    prefix = posegraph_prefix(posegraph_file)
    occupancy_map = map_file.strip()
    manifest_path = map_manifest_file.strip()
    map_contract = localization_map_contract.strip().lower()
    requested_localization_owner = localization_owner.strip().lower()
    route_graph = route_graph_file.strip()

    _require_choice('operation', operation, OPERATIONS)
    _require_choice('odometry_mode', odometry_mode, ODOMETRY_MODES)
    _require_choice(
        'structure_tf_source', structure_tf_source, STRUCTURE_TF_SOURCES)
    _require_choice(
        'localization_map_contract', map_contract,
        LOCALIZATION_MAP_CONTRACTS)
    _require_choice(
        'localization_owner', requested_localization_owner,
        LOCALIZATION_OWNERS)

    if odometry_mode == 'ideal' and structure_tf_source == 'rsp':
        raise ValueError(
            'ideal odometry is an Isaac-owned mode; structure_tf_source=rsp '
            'is reserved for the realistic/standard ROS ownership mode')

    localization_operation = operation in {'localization', 'navigation'}
    expected_localization_owner = (
        'ideal' if odometry_mode == 'ideal' else 'grid')
    if localization_operation:
        resolved_localization_owner = (
            expected_localization_owner
            if requested_localization_owner == 'auto'
            else requested_localization_owner
        )
        if resolved_localization_owner != expected_localization_owner:
            raise ValueError(
                f'localization_owner={resolved_localization_owner} conflicts '
                f'with odometry_mode={odometry_mode}; expected '
                f'{expected_localization_owner}')
    else:
        if requested_localization_owner != 'auto':
            raise ValueError(
                'localization_owner is valid only for localization or '
                'navigation mode')
        resolved_localization_owner = 'none'

    if map_contract == 'occupancy_only':
        if not localization_operation:
            raise ValueError(
                'localization_map_contract=occupancy_only is valid only for '
                'localization or navigation mode')
        if resolved_localization_owner != 'grid':
            raise ValueError(
                'localization_map_contract=occupancy_only requires '
                'localization_owner=grid')
        if prefix:
            raise ValueError(
                'posegraph_file must be empty for the occupancy_only '
                'localization map contract')
        if operation == 'navigation' and not route_graph:
            raise ValueError(
                'route_graph_file is required for the occupancy_only '
                'localization map contract')
    elif resolved_localization_owner == 'grid':
        raise ValueError(
            'localization_owner=grid requires '
            'localization_map_contract=occupancy_only')

    if operation == 'mapping' and prefix:
        raise ValueError(
            'posegraph_file is a localization input and must be empty in '
            'baseline mapping mode')
    saved_map_operation = operation in {
        'incremental_mapping', 'localization', 'navigation'}
    if saved_map_operation and map_contract == 'posegraph_bundle':
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
    if localization_operation:
        if not occupancy_map:
            raise ValueError(
                f'map_file is required for {operation} mode')
        if check_posegraph_files and not Path(occupancy_map).is_file():
            raise ValueError(
                f'occupancy map YAML does not exist: {occupancy_map}')
        if (operation == 'navigation'
                and map_contract == 'occupancy_only'
                and check_posegraph_files
                and not Path(route_graph).is_file()):
            raise ValueError(
                f'route graph does not exist: {route_graph}')
    elif occupancy_map:
        raise ValueError(
            f'map_file is not an input to {operation} mode')

    map_version = ''
    map_bundle_sha256 = ''
    if (saved_map_operation and map_contract == 'posegraph_bundle'
            and check_posegraph_files):
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
        localization_map_contract=map_contract,
        localization_owner=resolved_localization_owner,
        route_graph_file=route_graph,
        map_manifest_file=manifest_path,
        map_version=map_version,
        map_bundle_sha256=map_bundle_sha256,
    )


def _require_choice(name, value, choices):
    if value not in choices:
        expected = ', '.join(sorted(choices))
        raise ValueError(f'{name} must be one of [{expected}], got {value!r}')
