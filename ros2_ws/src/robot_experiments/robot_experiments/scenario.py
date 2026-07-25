"""Strict parser for static, dynamic, and incremental experiment scenarios."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re
from typing import Any, Mapping

from .configuration import (
    ConfigurationError,
    load_yaml_mapping,
    require_finite,
    require_mapping,
    require_string,
    require_vector,
)
from .metrics import (
    PLAN_INCREMENTAL_IMPROVEMENT_MIN_PERCENT,
    PLAN_ORIENTATION_TOLERANCE_RAD,
    PLAN_POSITION_TOLERANCE_M,
)
from .spawn_poses import SpawnPose
import math


SCENARIO_TYPES = frozenset({"static", "dynamic", "incremental"})
SCENARIO_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
DYNAMIC_MOTIONS = frozenset(
    {
        "crossing", "oncoming", "same_direction_slow", "local_bypass",
        "temporary_block", "g2_g3_exit", "g5_g1_crossing", "custom",
    }
)
DYNAMIC_GEOMETRY_TOLERANCE_M = 1.0e-4
DYNAMIC_DURATION_TOLERANCE_SEC = 1.0e-4


@dataclass(frozen=True)
class Goal:
    goal_id: str | None = field(compare=False)
    frame_id: str
    position: tuple[float, float]
    yaw_deg: float
    require_orientation: bool

    def as_dict(self) -> dict[str, object]:
        result: dict[str, object] = {
            "frame_id": self.frame_id,
            "position": list(self.position),
            "yaw_deg": self.yaw_deg,
            "require_orientation": self.require_orientation,
        }
        if self.goal_id is not None:
            result["id"] = self.goal_id
        return result


@dataclass(frozen=True)
class SuccessSettings:
    position_tolerance_m: float
    orientation_tolerance_deg: float
    final_linear_speed_mps: float
    final_angular_speed_radps: float
    final_still_duration_sec: float
    final_still_timeout_sec: float
    require_safety_observations: bool
    minimum_ground_truth_path_length_m: float
    minimum_reverse_distance_m: float
    maximum_reverse_distance_fraction: float
    minimum_curved_distance_fraction: float
    maximum_stopped_time_fraction: float


@dataclass(frozen=True)
class RunSelection:
    seed: int
    case_id: str | None = None
    variant_id: str | None = None
    appearance_profile_id: str | None = None


@dataclass(frozen=True)
class Scenario:
    scenario_id: str
    scenario_type: str
    seeds: tuple[int, ...]
    timeout_sec: float
    leg_timeout_sec: float
    spawn_pose_name: str
    map_version: str
    posegraph_version: str
    robot_config_file: str
    nav2_config_file: str
    dynamic_config_file: str | None
    appearance_config_file: str | None
    optimal_reference_file: str | None
    physics_dt: float
    rtf: float
    goal: Goal
    route: tuple[Goal, ...]
    success: SuccessSettings
    obstacles: Mapping[str, Any]
    obstacle_trajectories: tuple[Mapping[str, Any], ...]
    incremental_mapping: Mapping[str, Any] | None
    source_path: Path
    run_matrix: tuple[RunSelection, ...] = ()

    def resolve_path(self, configured_path: str) -> Path:
        candidate = Path(configured_path).expanduser()
        if candidate.is_absolute():
            return candidate
        return (self.source_path.parent / candidate).resolve()


@dataclass(frozen=True)
class _PhysicalObstacle:
    obstacle_id: str
    shape: str
    size: tuple[float, float, float]
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    speed: float
    repeat: bool
    coordinate_frame: str


def _positive(value: Any, location: str, *, allow_zero: bool = False) -> float:
    parsed = require_finite(value, location)
    if parsed < 0.0 or (parsed == 0.0 and not allow_zero):
        relation = "non-negative" if allow_zero else "positive"
        raise ConfigurationError(f"{location} must be {relation}")
    return parsed


def _fraction(value: Any, location: str) -> float:
    parsed = _positive(value, location, allow_zero=True)
    if parsed > 1.0:
        raise ConfigurationError(f"{location} must be between 0 and 1")
    return parsed


def _reject_unknown(
    values: Mapping[str, Any], allowed: set[str], location: str
) -> None:
    unknown = sorted(set(values) - allowed)
    if unknown:
        raise ConfigurationError(f"unknown {location} keys: {unknown}")


def _parse_seeds(runs: Mapping[str, Any]) -> tuple[int, ...]:
    values = runs.get("seeds")
    if not isinstance(values, list) or not values:
        raise ConfigurationError("scenario.runs.seeds must be a non-empty list")
    seeds: list[int] = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, int):
            raise ConfigurationError(f"scenario.runs.seeds[{index}] must be an integer")
        if value < 0:
            raise ConfigurationError(
                f"scenario.runs.seeds[{index}] must be non-negative"
            )
        seeds.append(value)
    if len(set(seeds)) != len(seeds):
        raise ConfigurationError("scenario.runs.seeds must not contain duplicates")
    return tuple(seeds)


def _parse_run_matrix(runs: Mapping[str, Any]) -> tuple[RunSelection, ...]:
    """Parse explicit case/variant rows used by the dynamic benchmark."""
    raw = runs.get("matrix")
    if raw is None:
        return tuple(RunSelection(seed) for seed in _parse_seeds(runs))
    if not isinstance(raw, list) or not raw:
        raise ConfigurationError("scenario.runs.matrix must be a non-empty list")
    rows: list[RunSelection] = []
    keys: set[tuple[int, str, str]] = set()
    for index, value in enumerate(raw):
        row = require_mapping(value, f"scenario.runs.matrix[{index}]")
        _reject_unknown(row, {"seed", "case_id", "variant_id", "appearance_profile_id"}, f"scenario.runs.matrix[{index}]")
        seed = row.get("seed")
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ConfigurationError(f"scenario.runs.matrix[{index}].seed must be non-negative integer")
        case_id = require_string(row.get("case_id"), f"scenario.runs.matrix[{index}].case_id")
        variant_id = require_string(row.get("variant_id"), f"scenario.runs.matrix[{index}].variant_id")
        appearance_profile = row.get("appearance_profile_id")
        if appearance_profile is not None:
            appearance_profile = require_string(
                appearance_profile,
                f"scenario.runs.matrix[{index}].appearance_profile_id",
            )
        key = seed, case_id, variant_id
        if key in keys: raise ConfigurationError("scenario.runs.matrix must not contain duplicate rows")
        keys.add(key); rows.append(RunSelection(seed, case_id, variant_id, appearance_profile))
    return tuple(rows)


def _validate_dynamic_trajectories(obstacles: Mapping[str, Any]) -> tuple[Mapping[str, Any], ...]:
    raw = obstacles.get("trajectories")
    if not isinstance(raw, list) or not raw:
        raise ConfigurationError("dynamic scenarios require obstacle trajectories")
    trajectories: list[Mapping[str, Any]] = []
    identifiers: set[str] = set()
    for index, item in enumerate(raw):
        trajectory = require_mapping(item, f"scenario.obstacles.trajectories[{index}]")
        _reject_unknown(
            trajectory,
            {"id", "motion", "shape", "repeat", "dimensions", "waypoints", "trigger_group"},
            f"scenario.obstacles.trajectories[{index}]",
        )
        identifier = require_string(trajectory.get("id"), f"trajectory[{index}].id")
        if identifier in identifiers:
            raise ConfigurationError(
                f"duplicate dynamic obstacle id {identifier!r}"
            )
        identifiers.add(identifier)
        motion = require_string(trajectory.get("motion"), f"trajectory[{index}].motion")
        if motion not in DYNAMIC_MOTIONS:
            raise ConfigurationError(f"unsupported dynamic motion {motion!r}")
        if trajectory.get("trigger_group") is not None:
            require_string(
                trajectory.get("trigger_group"),
                f"trajectory[{index}].trigger_group",
            )
        if trajectory.get("shape") != "box":
            raise ConfigurationError(
                f"trajectory[{index}].shape must be 'box'"
            )
        if not isinstance(trajectory.get("repeat"), bool):
            raise ConfigurationError(
                f"trajectory[{index}].repeat must be boolean"
            )
        dimensions = require_vector(
            trajectory.get("dimensions"), 2, f"trajectory[{index}].dimensions"
        )
        if min(dimensions) <= 0.0:
            raise ConfigurationError(
                f"trajectory[{index}].dimensions must be positive"
            )
        waypoints = trajectory.get("waypoints")
        if not isinstance(waypoints, list) or len(waypoints) < 2:
            raise ConfigurationError(f"trajectory[{index}].waypoints requires at least two entries")
        previous_time = -1.0
        for waypoint_index, waypoint_value in enumerate(waypoints):
            waypoint = require_mapping(
                waypoint_value, f"trajectory[{index}].waypoints[{waypoint_index}]"
            )
            _reject_unknown(
                waypoint,
                {"time_sec", "position"},
                f"trajectory[{index}].waypoints[{waypoint_index}]",
            )
            time_sec = _positive(
                waypoint.get("time_sec"),
                f"trajectory[{index}].waypoints[{waypoint_index}].time_sec",
                allow_zero=True,
            )
            require_vector(
                waypoint.get("position"),
                2,
                f"trajectory[{index}].waypoints[{waypoint_index}].position",
            )
            if time_sec <= previous_time:
                raise ConfigurationError("trajectory waypoint times must be strictly increasing")
            previous_time = time_sec
        trajectories.append(dict(trajectory))
    return tuple(trajectories)


def _validate_obstacles(
    obstacles: Mapping[str, Any], scenario_type: str
) -> tuple[Mapping[str, Any], ...]:
    _reject_unknown(
        obstacles, {"layout_id", "static", "trajectories"}, "scenario.obstacles"
    )
    require_string(obstacles.get("layout_id"), "scenario.obstacles.layout_id")
    static = obstacles.get("static")
    if not isinstance(static, list):
        raise ConfigurationError("scenario.obstacles.static must be a list")
    for index, item in enumerate(static):
        obstacle = require_mapping(item, f"scenario.obstacles.static[{index}]")
        _reject_unknown(obstacle, {"id"}, f"scenario.obstacles.static[{index}]")
        require_string(obstacle.get("id"), f"scenario.obstacles.static[{index}].id")
    trajectories = obstacles.get("trajectories")
    if not isinstance(trajectories, list):
        raise ConfigurationError("scenario.obstacles.trajectories must be a list")
    if scenario_type == "dynamic":
        return _validate_dynamic_trajectories(obstacles)
    if trajectories:
        raise ConfigurationError(
            f"{scenario_type} scenarios cannot contain dynamic trajectories"
        )
    return ()


def _parse_goal(value: Any, location: str) -> Goal:
    raw = require_mapping(value, location)
    _reject_unknown(
        raw,
        {"id", "frame_id", "position", "yaw_deg", "require_orientation"},
        location,
    )
    frame_id = require_string(raw.get("frame_id"), f"{location}.frame_id")
    if frame_id != "map":
        raise ConfigurationError(f"{location}.frame_id must be map")
    require_orientation = raw.get("require_orientation")
    if not isinstance(require_orientation, bool):
        raise ConfigurationError(
            f"{location}.require_orientation must be boolean"
        )
    identifier = raw.get("id")
    if identifier is not None:
        identifier = require_string(identifier, f"{location}.id")
    return Goal(
        goal_id=identifier,
        frame_id=frame_id,
        position=require_vector(raw.get("position"), 2, f"{location}.position"),
        yaw_deg=require_finite(raw.get("yaw_deg"), f"{location}.yaw_deg"),
        require_orientation=require_orientation,
    )


def _parse_route(value: Any, final_goal: Goal) -> tuple[Goal, ...]:
    if value is None:
        return ()
    if not isinstance(value, list) or len(value) < 2:
        raise ConfigurationError(
            "scenario.route must contain at least two map-frame poses"
        )
    route = tuple(
        _parse_goal(item, f"scenario.route[{index}]")
        for index, item in enumerate(value)
    )
    identifiers = [item.goal_id for item in route]
    if any(identifier is None for identifier in identifiers):
        route = tuple(
            Goal(
                goal_id=f"G{index}",
                frame_id=item.frame_id,
                position=item.position,
                yaw_deg=item.yaw_deg,
                require_orientation=item.require_orientation,
            )
            for index, item in enumerate(route, start=1)
        )
    elif len(set(identifiers)) != len(identifiers):
        raise ConfigurationError("scenario.route goal ids must be unique")
    for previous, current in zip(route, route[1:]):
        if math.dist(previous.position, current.position) <= 1.0e-6:
            raise ConfigurationError(
                "scenario.route cannot contain consecutive duplicate positions"
            )
    last = route[-1]
    if (
        last.frame_id != final_goal.frame_id
        or math.dist(last.position, final_goal.position) > 1.0e-9
        or abs(last.yaw_deg - final_goal.yaw_deg) > 1.0e-9
        or last.require_orientation != final_goal.require_orientation
    ):
        raise ConfigurationError(
            "scenario.route final pose must exactly match scenario.goal"
        )
    return route


def validate_navigation_runner_scenario(scenario: Scenario) -> None:
    """Reject workflow descriptors that the NavigateToPose runner cannot run."""
    if scenario.scenario_type == "incremental":
        raise ConfigurationError(
            "incremental scenarios are mapping workflow descriptors, not "
            "NavigateToPose trials; use incremental_mapping bringup and "
            "compare the saved map artifacts explicitly"
        )


def project_usd_xy_to_map(
    usd_position: tuple[float, float], spawn_pose: SpawnPose
) -> tuple[float, float]:
    """Project a USD/world XY point through a calibrated spawn-pose pair."""
    if not spawn_pose.map_calibrated:
        raise ConfigurationError(
            f"spawn pose {spawn_pose.name!r} is not calibrated"
        )
    usd_x, usd_y = (
        require_finite(usd_position[0], "USD position x"),
        require_finite(usd_position[1], "USD position y"),
    )
    rotation = math.radians(spawn_pose.map.yaw_deg - spawn_pose.usd.yaw_deg)
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    delta_x = usd_x - spawn_pose.usd.position[0]
    delta_y = usd_y - spawn_pose.usd.position[1]
    return (
        spawn_pose.map.position[0] + cosine * delta_x - sine * delta_y,
        spawn_pose.map.position[1] + sine * delta_x + cosine * delta_y,
    )


def _load_physical_dynamic_obstacles(
    path: str | Path,
) -> dict[str, _PhysicalObstacle]:
    source = Path(path).expanduser().resolve()
    document = load_yaml_mapping(source)
    version = document.get("schema_version")
    if version in {3, 4}:
        cases = document.get("cases")
        if not isinstance(cases, Mapping) or not cases:
            raise ConfigurationError(f"schema-v{version} physical dynamic obstacles require cases")
        parsed_v3: dict[str, _PhysicalObstacle] = {}
        for case_id, raw_case in cases.items():
            case = require_mapping(raw_case, f"physical dynamic case {case_id}")
            obstacle = require_mapping(case.get("obstacle"), f"physical dynamic case {case_id}.obstacle")
            identifier = require_string(obstacle.get("id"), f"physical dynamic case {case_id}.obstacle.id")
            waypoints = obstacle.get("waypoints")
            if identifier in parsed_v3 or not isinstance(waypoints, list) or len(waypoints) < 2:
                raise ConfigurationError(f"schema-v{version} physical dynamic obstacle id/waypoints is invalid")
            start, end = require_vector(waypoints[0], 3, f"{identifier}.start"), require_vector(waypoints[-1], 3, f"{identifier}.end")
            size = require_vector(obstacle.get("size"), 3, f"{identifier}.size")
            speed = _positive(obstacle.get("speed"), f"{identifier}.speed")
            if min(size) <= 0.0 or sum(math.dist(a, b) for a, b in zip(waypoints, waypoints[1:])) <= 0.0:
                raise ConfigurationError(f"schema-v{version} physical dynamic obstacle {identifier!r} is invalid")
            parsed_v3[identifier] = _PhysicalObstacle(identifier, "cube", tuple(size), tuple(start), tuple(end), speed, False, "map")
        return parsed_v3
    allowed_document = {"schema_version", "seed", "enabled", "obstacles"}
    if version == 2:
        allowed_document |= {"coordinate_frame", "spawn_pose_name"}
    _reject_unknown(document, allowed_document, "physical dynamic obstacle document")
    if version not in {1, 2}:
        raise ConfigurationError(
            "physical dynamic obstacle schema_version must be 1 or 2"
        )
    seed = document.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int):
        raise ConfigurationError("physical dynamic obstacle seed must be an integer")
    if not isinstance(document.get("enabled"), bool):
        raise ConfigurationError("physical dynamic obstacle enabled must be boolean")
    raw_obstacles = document.get("obstacles")
    if not isinstance(raw_obstacles, list) or not raw_obstacles:
        raise ConfigurationError(
            "physical dynamic obstacles must be a non-empty list"
        )

    coordinate_frame = document.get("coordinate_frame", "usd")
    if coordinate_frame not in {"usd", "map"}:
        raise ConfigurationError("physical dynamic obstacle coordinate_frame must be usd or map")
    parsed: dict[str, _PhysicalObstacle] = {}
    allowed_v1 = {
        "id",
        "shape",
        "size",
        "mass",
        "start",
        "end",
        "speed",
        "phase_jitter",
        "repeat",
    }
    allowed_v2 = {
        "id", "mode", "trigger_group", "size", "mass", "start", "end", "speed",
        "delay_sec", "jitter_sec", "post_motion",
    }
    for index, value in enumerate(raw_obstacles):
        location = f"physical dynamic obstacles[{index}]"
        obstacle = require_mapping(value, location)
        allowed = allowed_v1 if version == 1 else allowed_v2
        _reject_unknown(obstacle, allowed, location)
        missing = sorted(allowed - set(obstacle))
        if missing:
            raise ConfigurationError(f"missing {location} keys: {missing}")
        identifier = require_string(obstacle.get("id"), f"{location}.id")
        if identifier in parsed:
            raise ConfigurationError(
                f"duplicate physical dynamic obstacle id {identifier!r}"
            )
        shape = "cube" if version == 2 else require_string(obstacle.get("shape"), f"{location}.shape")
        if shape != "cube":
            raise ConfigurationError(
                f"physical dynamic obstacle {identifier!r} shape must be 'cube'"
            )
        size = require_vector(obstacle.get("size"), 3, f"{location}.size")
        if min(size) <= 0.0:
            raise ConfigurationError(
                f"physical dynamic obstacle {identifier!r} size must be positive"
            )
        mass = require_finite(obstacle.get("mass"), f"{location}.mass")
        if mass <= 0.0:
            raise ConfigurationError(
                f"physical dynamic obstacle {identifier!r} mass must be positive"
            )
        start = require_vector(obstacle.get("start"), 3, f"{location}.start")
        end = require_vector(obstacle.get("end"), 3, f"{location}.end")
        mode = obstacle.get("mode", "linear")
        if mode not in {"linear", "stationary"}:
            raise ConfigurationError(f"physical dynamic obstacle {identifier!r} mode is invalid")
        distance = math.dist(start, end)
        if mode == "linear" and distance <= 0.0:
            raise ConfigurationError(
                f"physical dynamic obstacle {identifier!r} path must be non-zero"
            )
        speed = require_finite(obstacle.get("speed"), f"{location}.speed")
        if speed < 0.0 or (mode == "linear" and speed <= 0.0):
            raise ConfigurationError(
                f"physical dynamic obstacle {identifier!r} speed must be positive"
            )
        phase_jitter = require_finite(
            obstacle.get("phase_jitter") if version == 1 else obstacle.get("jitter_sec"),
            f"{location}.phase_jitter",
        )
        if phase_jitter < 0.0:
            raise ConfigurationError(
                f"physical dynamic obstacle {identifier!r} phase_jitter "
                "must be non-negative"
            )
        repeat = obstacle.get("repeat", False)
        if not isinstance(repeat, bool):
            raise ConfigurationError(
                f"physical dynamic obstacle {identifier!r} repeat must be boolean"
            )
        parsed[identifier] = _PhysicalObstacle(
            obstacle_id=identifier,
            shape=shape,
            size=(size[0], size[1], size[2]),
            start=(start[0], start[1], start[2]),
            end=(end[0], end[1], end[2]),
            speed=speed,
            repeat=repeat,
            coordinate_frame=coordinate_frame,
        )
    return parsed


def _close_sequence(
    actual: tuple[float, ...],
    expected: tuple[float, ...],
    tolerance: float,
) -> bool:
    return len(actual) == len(expected) and all(
        math.isclose(
            actual_value,
            expected_value,
            rel_tol=1.0e-9,
            abs_tol=tolerance,
        )
        for actual_value, expected_value in zip(actual, expected)
    )


def validate_dynamic_physical_contract(
    scenario: Scenario,
    spawn_pose: SpawnPose,
    physical_config_path: str | Path,
) -> None:
    """Verify a dynamic scenario describes the referenced Isaac obstacles."""
    if scenario.scenario_type != "dynamic":
        return
    physical = _load_physical_dynamic_obstacles(physical_config_path)
    physical_schema_v3 = load_yaml_mapping(physical_config_path).get("schema_version") in {3, 4}
    declared = {
        require_string(item.get("id"), "dynamic trajectory id"): item
        for item in scenario.obstacle_trajectories
    }
    if set(physical) != set(declared):
        raise ConfigurationError(
            "physical dynamic obstacle IDs do not match scenario declarations: "
            f"scenario={tuple(sorted(declared))}, physical={tuple(sorted(physical))}"
        )

    for identifier in sorted(declared):
        trajectory = declared[identifier]
        obstacle = physical[identifier]
        scenario_shape = require_string(
            trajectory.get("shape"), f"trajectory {identifier!r}.shape"
        )
        if scenario_shape != "box" or obstacle.shape != "cube":
            raise ConfigurationError(
                f"dynamic obstacle {identifier!r} shape mismatch: "
                f"scenario={scenario_shape!r}, physical={obstacle.shape!r}"
            )
        scenario_repeat = trajectory.get("repeat")
        if not isinstance(scenario_repeat, bool):
            raise ConfigurationError(
                f"dynamic obstacle {identifier!r} repeat must be boolean"
            )
        if scenario_repeat != obstacle.repeat:
            raise ConfigurationError(
                f"dynamic obstacle {identifier!r} repeat mismatch: "
                f"scenario={scenario_repeat}, physical={obstacle.repeat}"
            )
        dimensions = require_vector(
            trajectory.get("dimensions"),
            2,
            f"trajectory {identifier!r}.dimensions",
        )
        if min(dimensions) <= 0.0:
            raise ConfigurationError(
                f"trajectory {identifier!r}.dimensions must be positive"
            )
        physical_dimensions = (obstacle.size[0], obstacle.size[1])
        if not _close_sequence(
            physical_dimensions,
            (dimensions[0], dimensions[1]),
            DYNAMIC_GEOMETRY_TOLERANCE_M,
        ):
            raise ConfigurationError(
                f"dynamic obstacle {identifier!r} XY dimensions mismatch: "
                f"scenario={dimensions}, physical={physical_dimensions}"
            )
        # Schema v3/v4 use acceleration-limited multi-segment trajectories and
        # therefore has no truthful fixed waypoint time in the ROS campaign.
        # Geometry/IDs remain verified here; kinematics live in Isaac YAML.
        if physical_schema_v3:
            continue

        raw_waypoints = trajectory.get("waypoints")
        if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
            raise ConfigurationError(
                f"trajectory {identifier!r} requires at least two waypoints"
            )
        first = require_mapping(
            raw_waypoints[0], f"trajectory {identifier!r} first waypoint"
        )
        last = require_mapping(
            raw_waypoints[-1], f"trajectory {identifier!r} last waypoint"
        )
        first_time = require_finite(
            first.get("time_sec"), f"trajectory {identifier!r} first time"
        )
        last_time = require_finite(
            last.get("time_sec"), f"trajectory {identifier!r} last time"
        )
        if first_time < 0.0 or last_time <= first_time:
            raise ConfigurationError(
                f"trajectory {identifier!r} endpoint times are invalid"
            )
        scenario_start = require_vector(
            first.get("position"),
            2,
            f"trajectory {identifier!r} first position",
        )
        scenario_end = require_vector(
            last.get("position"),
            2,
            f"trajectory {identifier!r} last position",
        )
        if obstacle.coordinate_frame == "map":
            physical_start = (obstacle.start[0], obstacle.start[1])
            physical_end = (obstacle.end[0], obstacle.end[1])
        else:
            physical_start = project_usd_xy_to_map(
                (obstacle.start[0], obstacle.start[1]), spawn_pose
            )
            physical_end = project_usd_xy_to_map(
                (obstacle.end[0], obstacle.end[1]), spawn_pose
            )
        if not _close_sequence(
            physical_start,
            (scenario_start[0], scenario_start[1]),
            DYNAMIC_GEOMETRY_TOLERANCE_M,
        ):
            raise ConfigurationError(
                f"dynamic obstacle {identifier!r} start endpoint mismatch: "
                f"scenario={scenario_start}, physical_map={physical_start}"
            )
        if not _close_sequence(
            physical_end,
            (scenario_end[0], scenario_end[1]),
            DYNAMIC_GEOMETRY_TOLERANCE_M,
        ):
            raise ConfigurationError(
                f"dynamic obstacle {identifier!r} end endpoint mismatch: "
                f"scenario={scenario_end}, physical_map={physical_end}"
            )

        physical_duration = math.dist(obstacle.start, obstacle.end) / obstacle.speed
        scenario_duration = last_time - first_time
        if not math.isclose(
            physical_duration,
            scenario_duration,
            rel_tol=1.0e-9,
            abs_tol=DYNAMIC_DURATION_TOLERANCE_SEC,
        ):
            raise ConfigurationError(
                f"dynamic obstacle {identifier!r} duration mismatch: "
                f"scenario={scenario_duration}, physical={physical_duration}"
            )


def validate_dynamic_runtime_contract(
    scenario: Scenario,
    *,
    runtime_enabled: bool,
    runtime_config_hash: str,
    runtime_obstacle_ids: tuple[str, ...],
    expected_config_hash: str | None,
) -> None:
    """Verify the Isaac process is running the physical obstacle set claimed."""
    if scenario.scenario_type == "static":
        if scenario.dynamic_config_file is None:
            if runtime_enabled:
                raise ConfigurationError(
                    "static scenario without a physical obstacle config requires Isaac dynamic obstacles to be disabled"
                )
            return
        if not runtime_enabled:
            raise ConfigurationError(
                "static physical-obstacle scenario requires Isaac --dynamic-obstacles"
            )
        if not expected_config_hash or runtime_config_hash != expected_config_hash:
            raise ConfigurationError(
                "Isaac static obstacle configuration hash does not match the scenario"
            )
        expected_ids = tuple(sorted(
            require_string(item.get("id"), "static obstacle id")
            for item in scenario.obstacles.get("static", [])
        ))
        if tuple(sorted(runtime_obstacle_ids)) != expected_ids:
            raise ConfigurationError(
                "Isaac static obstacle IDs do not match the scenario: "
                f"expected={expected_ids}, runtime={tuple(sorted(runtime_obstacle_ids))}"
            )
        return
    if scenario.scenario_type != "dynamic":
        return
    if not runtime_enabled:
        raise ConfigurationError(
            "dynamic scenario requires Isaac --dynamic-obstacles"
        )
    if not expected_config_hash or runtime_config_hash != expected_config_hash:
        raise ConfigurationError(
            "Isaac dynamic obstacle configuration hash does not match the "
            "scenario"
        )
    expected_ids = tuple(sorted(
        str(item["id"]) for item in scenario.obstacle_trajectories
    ))
    if tuple(sorted(runtime_obstacle_ids)) != expected_ids:
        raise ConfigurationError(
            "Isaac dynamic obstacle IDs do not match the scenario: "
            f"expected={expected_ids}, runtime={tuple(sorted(runtime_obstacle_ids))}"
        )


def load_scenario(path: str | Path) -> Scenario:
    source = Path(path).expanduser().resolve()
    document = load_yaml_mapping(source)
    _reject_unknown(document, {"schema_version", "scenario"}, "document")
    if document.get("schema_version") != 1:
        raise ConfigurationError("scenario schema_version must be 1")
    raw = require_mapping(document.get("scenario"), "scenario")
    _reject_unknown(
        raw,
        {
            "id",
            "type",
            "spawn_pose_name",
            "map_version",
            "posegraph_version",
            "configs",
            "simulation",
            "runs",
            "goal",
            "route",
            "success",
            "obstacles",
            "incremental_mapping",
        },
        "scenario",
    )
    scenario_id = require_string(raw.get("id"), "scenario.id")
    if SCENARIO_ID_PATTERN.fullmatch(scenario_id) is None:
        raise ConfigurationError(
            "scenario.id may contain only letters, digits, dot, underscore, and hyphen"
        )
    scenario_type = require_string(raw.get("type"), "scenario.type")
    if scenario_type not in SCENARIO_TYPES:
        raise ConfigurationError(f"scenario.type must be one of {sorted(SCENARIO_TYPES)}")

    configs = require_mapping(raw.get("configs"), "scenario.configs")
    simulation = require_mapping(raw.get("simulation"), "scenario.simulation")
    runs = require_mapping(raw.get("runs"), "scenario.runs")
    success_raw = require_mapping(raw.get("success"), "scenario.success")
    obstacles = require_mapping(raw.get("obstacles"), "scenario.obstacles")
    _reject_unknown(
        configs, {"robot", "nav2", "dynamic_obstacles", "optimal_reference", "appearance"}, "scenario.configs"
    )
    _reject_unknown(
        simulation, {"physics_dt", "rtf"}, "scenario.simulation"
    )
    _reject_unknown(runs, {"seeds", "matrix", "timeout_sec", "leg_timeout_sec"}, "scenario.runs")
    if "matrix" in runs and "seeds" in runs:
        raise ConfigurationError("scenario.runs must use either seeds or matrix, not both")
    _reject_unknown(
        success_raw,
        {
            "position_tolerance_m",
            "orientation_tolerance_deg",
            "final_linear_speed_mps",
            "final_angular_speed_radps",
            "final_still_duration_sec",
            "final_still_timeout_sec",
            "require_safety_observations",
            "minimum_ground_truth_path_length_m",
            "minimum_reverse_distance_m",
            "maximum_reverse_distance_fraction",
            "minimum_curved_distance_fraction",
            "maximum_stopped_time_fraction",
        },
        "scenario.success",
    )

    goal = _parse_goal(raw.get("goal"), "scenario.goal")
    route = _parse_route(raw.get("route"), goal)
    safety_required = success_raw.get("require_safety_observations", False)
    if not isinstance(safety_required, bool):
        raise ConfigurationError("scenario.success.require_safety_observations must be boolean")

    position_tolerance = _positive(
        success_raw.get("position_tolerance_m", PLAN_POSITION_TOLERANCE_M),
        "scenario.success.position_tolerance_m",
        allow_zero=True,
    )
    orientation_tolerance = _positive(
        success_raw.get("orientation_tolerance_deg", math.degrees(PLAN_ORIENTATION_TOLERANCE_RAD)),
        "scenario.success.orientation_tolerance_deg",
        allow_zero=True,
    )
    if position_tolerance > PLAN_POSITION_TOLERANCE_M:
        raise ConfigurationError("position tolerance cannot be looser than plan.md (0.25 m)")
    if orientation_tolerance > math.degrees(PLAN_ORIENTATION_TOLERANCE_RAD) + 1e-12:
        raise ConfigurationError("orientation tolerance cannot be looser than plan.md (10 deg)")

    trajectories = _validate_obstacles(obstacles, scenario_type)
    incremental: Mapping[str, Any] | None = None
    dynamic_config_file: str | None = None
    appearance_config_file: str | None = None
    optimal_reference_file: str | None = None
    if configs.get("optimal_reference") is not None:
        optimal_reference_file = require_string(
            configs.get("optimal_reference"), "scenario.configs.optimal_reference"
        )
    if configs.get("appearance") is not None:
        appearance_config_file = require_string(
            configs.get("appearance"), "scenario.configs.appearance"
        )
    configured_dynamic = configs.get("dynamic_obstacles")
    if scenario_type == "dynamic":
        dynamic_config_file = require_string(
            configured_dynamic, "scenario.configs.dynamic_obstacles"
        )
    elif configured_dynamic is not None:
        dynamic_config_file = require_string(
            configured_dynamic, "scenario.configs.dynamic_obstacles"
        )
        if not obstacles.get("static"):
            raise ConfigurationError(
                "static physical obstacle config requires scenario.obstacles.static"
            )

    if scenario_type == "incremental":
        incremental = require_mapping(raw.get("incremental_mapping"), "scenario.incremental_mapping")
        _reject_unknown(
            incremental,
            {
                "baseline_map_version",
                "changed_regions",
                "minimum_time_improvement_percent",
            },
            "scenario.incremental_mapping",
        )
        require_string(incremental.get("baseline_map_version"), "incremental_mapping.baseline_map_version")
        changed_regions = incremental.get("changed_regions")
        if not isinstance(changed_regions, list) or not changed_regions:
            raise ConfigurationError("incremental_mapping.changed_regions must be non-empty")
        parsed_regions = [
            require_string(value, f"incremental_mapping.changed_regions[{index}]")
            for index, value in enumerate(changed_regions)
        ]
        if len(set(parsed_regions)) != len(parsed_regions):
            raise ConfigurationError(
                "incremental_mapping.changed_regions must not contain duplicates"
            )
        minimum = _positive(
            incremental.get(
                "minimum_time_improvement_percent", PLAN_INCREMENTAL_IMPROVEMENT_MIN_PERCENT
            ),
            "incremental_mapping.minimum_time_improvement_percent",
            allow_zero=True,
        )
        if minimum < PLAN_INCREMENTAL_IMPROVEMENT_MIN_PERCENT:
            raise ConfigurationError("incremental improvement target cannot be below 30 percent")
    elif "incremental_mapping" in raw:
        raise ConfigurationError(
            "scenario.incremental_mapping is only valid for incremental scenarios"
        )

    run_matrix = _parse_run_matrix(runs)
    if appearance_config_file is not None and any(
        item.appearance_profile_id is None for item in run_matrix
    ):
        raise ConfigurationError(
            "scenarios with scenario.configs.appearance require appearance_profile_id for every run matrix row"
        )
    return Scenario(
        scenario_id=scenario_id,
        scenario_type=scenario_type,
        seeds=tuple(item.seed for item in run_matrix),
        timeout_sec=_positive(runs.get("timeout_sec"), "scenario.runs.timeout_sec"),
        leg_timeout_sec=_positive(
            runs.get("leg_timeout_sec", runs.get("timeout_sec")),
            "scenario.runs.leg_timeout_sec",
        ),
        spawn_pose_name=require_string(raw.get("spawn_pose_name"), "scenario.spawn_pose_name"),
        map_version=require_string(raw.get("map_version"), "scenario.map_version"),
        posegraph_version=require_string(raw.get("posegraph_version"), "scenario.posegraph_version"),
        robot_config_file=require_string(configs.get("robot"), "scenario.configs.robot"),
        nav2_config_file=require_string(configs.get("nav2"), "scenario.configs.nav2"),
        dynamic_config_file=dynamic_config_file,
        appearance_config_file=appearance_config_file,
        optimal_reference_file=optimal_reference_file,
        physics_dt=_positive(simulation.get("physics_dt"), "scenario.simulation.physics_dt"),
        rtf=_positive(simulation.get("rtf"), "scenario.simulation.rtf"),
        goal=goal,
        route=route,
        success=SuccessSettings(
            position_tolerance_m=position_tolerance,
            orientation_tolerance_deg=orientation_tolerance,
            final_linear_speed_mps=_positive(
                success_raw.get("final_linear_speed_mps", 0.02),
                "scenario.success.final_linear_speed_mps",
                allow_zero=True,
            ),
            final_angular_speed_radps=_positive(
                success_raw.get("final_angular_speed_radps", 0.05),
                "scenario.success.final_angular_speed_radps",
                allow_zero=True,
            ),
            final_still_duration_sec=_positive(
                success_raw.get("final_still_duration_sec", 1.0),
                "scenario.success.final_still_duration_sec",
            ),
            final_still_timeout_sec=_positive(
                success_raw.get("final_still_timeout_sec", 10.0),
                "scenario.success.final_still_timeout_sec",
            ),
            require_safety_observations=safety_required,
            minimum_ground_truth_path_length_m=_positive(
                success_raw.get("minimum_ground_truth_path_length_m", 0.0),
                "scenario.success.minimum_ground_truth_path_length_m",
                allow_zero=True,
            ),
            minimum_reverse_distance_m=_positive(
                success_raw.get("minimum_reverse_distance_m", 0.0),
                "scenario.success.minimum_reverse_distance_m",
                allow_zero=True,
            ),
            maximum_reverse_distance_fraction=_fraction(
                success_raw.get("maximum_reverse_distance_fraction", 1.0),
                "scenario.success.maximum_reverse_distance_fraction",
            ),
            minimum_curved_distance_fraction=_fraction(
                success_raw.get("minimum_curved_distance_fraction", 0.0),
                "scenario.success.minimum_curved_distance_fraction",
            ),
            maximum_stopped_time_fraction=_fraction(
                success_raw.get("maximum_stopped_time_fraction", 1.0),
                "scenario.success.maximum_stopped_time_fraction",
            ),
        ),
        obstacles=dict(obstacles),
        obstacle_trajectories=trajectories,
        incremental_mapping=dict(incremental) if incremental is not None else None,
        source_path=source,
        run_matrix=run_matrix,
    )
