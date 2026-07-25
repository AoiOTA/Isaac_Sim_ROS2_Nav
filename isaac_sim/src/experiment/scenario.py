"""Deterministic dynamic-obstacle scenario parsing and sampling.

Schema v4 keeps the calibrated one-actor cases while adding named ``case_sets``
for ordered full-route interactions.  Each actor remains independently armed
by its target goal and spatial gate; selecting a set never starts all actors at
once.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import random

from isaac_sim.src.yaml_utils import load_mapping, reject_unknown, require_keys, require_number, require_vector


@dataclass(frozen=True)
class ObstacleSpec:
    obstacle_id: str
    shape: str
    size: tuple[float, float, float]
    mass: float
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    speed: float
    phase_jitter: float
    repeat: bool
    mode: str = "linear"
    trigger_group: str | None = None
    delay_sec: float = 0.0
    post_motion: str = "hold"


@dataclass(frozen=True)
class RobotGate:
    axis: str
    threshold: float
    direction: str
    min_speed_mps: float
    x_range: tuple[float, float] | None = None
    max_distance_to_obstacle_start_m: float | None = None


@dataclass(frozen=True)
class DynamicVariant:
    variant_id: str
    seed: int | None = None
    start_delay_sec: float = 0.0
    dwell_sec: float = 0.0


@dataclass(frozen=True)
class DynamicCase:
    case_id: str
    obstacle: ObstacleSpec
    waypoints: tuple[tuple[float, float, float], ...]
    trigger_group: str
    gate: RobotGate
    max_acceleration: float
    variants: tuple[DynamicVariant, ...]

    def variant(self, variant_id: str | int | None) -> DynamicVariant:
        if variant_id is None:
            return self.variants[0]
        key = str(variant_id)
        for item in self.variants:
            if item.variant_id == key or item.variant_id == f"v{key}":
                return item
        raise ValueError(f"unknown dynamic obstacle variant {variant_id!r} for {self.case_id}")


@dataclass(frozen=True)
class DynamicScenario:
    seed: int
    enabled: bool
    obstacles: tuple[ObstacleSpec, ...]
    coordinate_frame: str = "usd"
    spawn_pose_name: str | None = None
    cases: dict[str, DynamicCase] = field(default_factory=dict)
    case_sets: dict[str, tuple[str, ...]] = field(default_factory=dict)
    guard_clearance_m: float = 0.05
    min_clearance_m: float = 0.10

    @property
    def is_case_matrix(self) -> bool:
        return bool(self.cases)

    def case(self, case_id: str | None = None) -> DynamicCase:
        if not self.cases:
            raise ValueError("legacy dynamic scenario has no named cases")
        if case_id is None:
            return next(iter(self.cases.values()))
        try:
            return self.cases[case_id]
        except KeyError as exc:
            raise ValueError(f"unknown dynamic obstacle case {case_id!r}") from exc

    def selected_cases(self, case_id: str | None = None) -> tuple[DynamicCase, ...]:
        """Return one calibrated case or an ordered schema-v4 case set."""
        if case_id is not None and case_id in self.case_sets:
            return tuple(self.cases[item] for item in self.case_sets[case_id])
        return (self.case(case_id),)

    def sampled_phases(self, seed: int | None = None) -> dict[str, float]:
        rng = random.Random(self.seed if seed is None else seed)
        return {item.obstacle_id: rng.uniform(-item.phase_jitter, item.phase_jitter) for item in self.obstacles}


def _vector(raw: object, size: int, context: str) -> tuple[float, ...]:
    return tuple(require_vector(raw, size, context=context))


def _load_case_matrix(data: dict[str, object], schema_version: int) -> DynamicScenario:
    allowed = {"schema_version", "seed", "enabled", "coordinate_frame", "spawn_pose_name", "safety", "cases"}
    if schema_version >= 4:
        allowed.add("case_sets")
    reject_unknown(data, allowed, context="dynamic scenario")
    require_keys(data, allowed, context="dynamic scenario")
    if not isinstance(data["seed"], int) or isinstance(data["seed"], bool):
        raise ValueError("dynamic scenario schema_version/seed is invalid")
    if not isinstance(data["enabled"], bool) or data["coordinate_frame"] != "map":
        raise ValueError(f"schema v{schema_version} requires enabled boolean and map coordinates")
    if not isinstance(data["spawn_pose_name"], str) or not data["spawn_pose_name"]:
        raise ValueError("map-coordinate dynamic scenario requires spawn_pose_name")
    safety = data["safety"]
    if not isinstance(safety, dict):
        raise ValueError("dynamic scenario safety must be a mapping")
    reject_unknown(safety, {"guard_clearance_m", "min_clearance_m"}, context="dynamic safety")
    require_keys(safety, {"guard_clearance_m", "min_clearance_m"}, context="dynamic safety")
    guard = require_number(safety["guard_clearance_m"], context="guard_clearance_m", positive=True)
    minimum = require_number(safety["min_clearance_m"], context="min_clearance_m", positive=True)
    if guard <= minimum:
        raise ValueError("guard_clearance_m must exceed min_clearance_m")
    raw_cases = data["cases"]
    if not isinstance(raw_cases, dict) or not raw_cases:
        raise ValueError(f"schema v{schema_version} cases must be a non-empty mapping")
    cases: dict[str, DynamicCase] = {}
    all_specs: list[ObstacleSpec] = []
    for case_id, raw in raw_cases.items():
        if not isinstance(case_id, str) or not case_id or not isinstance(raw, dict):
            raise ValueError("dynamic cases must have non-empty string ids and mappings")
        allowed_case = {"trigger_group", "gate", "obstacle", "variants"}
        reject_unknown(raw, allowed_case, context=f"dynamic case {case_id}")
        require_keys(raw, allowed_case, context=f"dynamic case {case_id}")
        group = raw["trigger_group"]
        if not isinstance(group, str) or not group:
            raise ValueError(f"{case_id}.trigger_group must be non-empty")
        gate_raw = raw["gate"]
        if not isinstance(gate_raw, dict):
            raise ValueError(f"{case_id}.gate must be a mapping")
        reject_unknown(gate_raw, {"axis", "threshold", "direction", "min_speed_mps", "x_range", "max_distance_to_obstacle_start_m"}, context=f"{case_id}.gate")
        require_keys(gate_raw, {"axis", "threshold", "direction", "min_speed_mps"}, context=f"{case_id}.gate")
        axis, direction = gate_raw["axis"], gate_raw["direction"]
        if axis not in {"x", "y"} or direction not in {"positive", "negative"}:
            raise ValueError(f"{case_id}.gate axis/direction is invalid")
        x_range = None
        if "x_range" in gate_raw:
            x_range = _vector(gate_raw["x_range"], 2, f"{case_id}.gate.x_range")
            if x_range[0] > x_range[1]:
                raise ValueError(f"{case_id}.gate.x_range is invalid")
        max_distance_to_obstacle_start_m = None
        if "max_distance_to_obstacle_start_m" in gate_raw:
            max_distance_to_obstacle_start_m = require_number(
                gate_raw["max_distance_to_obstacle_start_m"],
                context=f"{case_id}.gate.max_distance_to_obstacle_start_m",
                positive=True,
            )
        obstacle_raw = raw["obstacle"]
        if not isinstance(obstacle_raw, dict):
            raise ValueError(f"{case_id}.obstacle must be a mapping")
        allowed_obstacle = {"id", "size", "mass", "waypoints", "speed", "max_acceleration", "post_motion"}
        reject_unknown(obstacle_raw, allowed_obstacle, context=f"{case_id}.obstacle")
        require_keys(obstacle_raw, allowed_obstacle, context=f"{case_id}.obstacle")
        obstacle_id = obstacle_raw["id"]
        if not isinstance(obstacle_id, str) or not obstacle_id or any(s.obstacle_id == obstacle_id for s in all_specs):
            raise ValueError(f"invalid or duplicate obstacle id {obstacle_id!r}")
        waypoints_raw = obstacle_raw["waypoints"]
        if not isinstance(waypoints_raw, list) or len(waypoints_raw) < 2:
            raise ValueError(f"{case_id}.obstacle.waypoints needs at least two entries")
        waypoints = tuple(_vector(item, 3, f"{case_id}.obstacle.waypoints") for item in waypoints_raw)
        if any(a == b for a, b in zip(waypoints, waypoints[1:])):
            raise ValueError(f"{case_id}.obstacle.waypoints contains a zero-length segment")
        post_motion = obstacle_raw["post_motion"]
        if post_motion not in {"retire", "park"}:
            raise ValueError("schema v3 dynamic obstacles must retire or park after motion")
        spec = ObstacleSpec(obstacle_id, "cube", _vector(obstacle_raw["size"], 3, f"{case_id}.obstacle.size"),
                            require_number(obstacle_raw["mass"], context=f"{case_id}.obstacle.mass", positive=True),
                            waypoints[0], waypoints[-1], require_number(obstacle_raw["speed"], context=f"{case_id}.obstacle.speed", positive=True),
                            0.0, False, "linear", group, 0.0, post_motion)
        if min(spec.size) <= 0:
            raise ValueError(f"{case_id}.obstacle.size values must be positive")
        variants_raw = raw["variants"]
        if not isinstance(variants_raw, dict) or not variants_raw:
            raise ValueError(f"{case_id}.variants must be a non-empty mapping")
        variants: list[DynamicVariant] = []
        for variant_id, variant_raw in variants_raw.items():
            if not isinstance(variant_id, str) or not isinstance(variant_raw, dict):
                raise ValueError(f"{case_id}.variants is invalid")
            reject_unknown(variant_raw, {"seed", "start_delay_sec", "dwell_sec"}, context=f"{case_id}.{variant_id}")
            if "seed" in variant_raw and (not isinstance(variant_raw["seed"], int) or isinstance(variant_raw["seed"], bool)):
                raise ValueError(f"{case_id}.{variant_id}.seed is invalid")
            delay = require_number(variant_raw.get("start_delay_sec", 0.0), context=f"{case_id}.{variant_id}.start_delay_sec")
            dwell = require_number(variant_raw.get("dwell_sec", 0.0), context=f"{case_id}.{variant_id}.dwell_sec")
            if delay < 0 or dwell < 0 or dwell > 1.8:
                raise ValueError(f"{case_id}.{variant_id} delay/dwell is outside the safety contract")
            variants.append(DynamicVariant(variant_id, variant_raw.get("seed"), delay, dwell))
        cases[case_id] = DynamicCase(case_id, spec, waypoints, group,
                                     RobotGate(axis, require_number(gate_raw["threshold"], context=f"{case_id}.gate.threshold"), direction,
                                               require_number(gate_raw["min_speed_mps"], context=f"{case_id}.gate.min_speed_mps", positive=True), x_range,
                                               max_distance_to_obstacle_start_m),
                                     require_number(obstacle_raw["max_acceleration"], context=f"{case_id}.obstacle.max_acceleration", positive=True), tuple(variants))
        all_specs.append(spec)
    case_sets: dict[str, tuple[str, ...]] = {}
    if schema_version >= 4:
        raw_sets = data.get("case_sets", {})
        if not isinstance(raw_sets, dict):
            raise ValueError("schema v4 case_sets must be a mapping")
        for set_id, raw_ids in raw_sets.items():
            if not isinstance(set_id, str) or not set_id:
                raise ValueError("schema v4 case set id is invalid")
            if not isinstance(raw_ids, list) or not raw_ids:
                raise ValueError(f"case set {set_id!r} must contain one or more cases")
            identifiers = tuple(raw_ids)
            if (
                any(not isinstance(item, str) or item not in cases for item in identifiers)
                or len(set(identifiers)) != len(identifiers)
            ):
                raise ValueError(f"case set {set_id!r} references invalid or duplicate cases")
            groups = [cases[item].trigger_group for item in identifiers]
            if len(set(groups)) != len(groups):
                raise ValueError(f"case set {set_id!r} must use distinct trigger groups")
            case_sets[set_id] = identifiers
    return DynamicScenario(
        data["seed"], data["enabled"], tuple(all_specs), "map",
        data["spawn_pose_name"], cases, case_sets, guard, minimum,
    )


def load_dynamic_scenario(path: str | Path) -> DynamicScenario:
    data = load_mapping(path)
    version = data.get("schema_version")
    if version in {3, 4}:
        return _load_case_matrix(data, int(version))
    if version not in {1, 2}:
        raise ValueError("dynamic scenario schema_version/seed is invalid")
    top_level = {"schema_version", "seed", "enabled", "obstacles"}
    if version == 2:
        top_level |= {"coordinate_frame", "spawn_pose_name"}
    reject_unknown(data, top_level, context="dynamic scenario")
    require_keys(data, {"schema_version", "seed", "enabled", "obstacles"}, context="dynamic scenario")
    if not isinstance(data["seed"], int) or isinstance(data["seed"], bool):
        raise ValueError("dynamic scenario schema_version/seed is invalid")
    if not isinstance(data["enabled"], bool) or not isinstance(data["obstacles"], list):
        raise ValueError("dynamic scenario enabled/obstacles is invalid")
    coordinate_frame = data.get("coordinate_frame", "usd")
    if coordinate_frame not in {"usd", "map"}:
        raise ValueError("dynamic scenario coordinate_frame must be usd or map")
    spawn_pose_name = data.get("spawn_pose_name")
    if version == 2 and coordinate_frame == "map" and (not isinstance(spawn_pose_name, str) or not spawn_pose_name):
        raise ValueError("map-coordinate dynamic scenario requires spawn_pose_name")
    parsed: list[ObstacleSpec] = []
    identifiers: set[str] = set()
    for raw in data["obstacles"]:
        if not isinstance(raw, dict): raise ValueError("each dynamic obstacle must be a mapping")
        allowed = {"id", "shape", "size", "mass", "start", "end", "speed", "phase_jitter", "repeat"} if version == 1 else {"id", "mode", "trigger_group", "size", "mass", "start", "end", "speed", "delay_sec", "jitter_sec", "post_motion"}
        reject_unknown(raw, allowed, context="dynamic obstacle"); require_keys(raw, allowed, context="dynamic obstacle")
        identifier = raw["id"]
        if not isinstance(identifier, str) or not identifier or identifier in identifiers: raise ValueError(f"invalid or duplicate obstacle id {identifier!r}")
        identifiers.add(identifier)
        if version == 1 and raw["shape"] != "cube": raise ValueError("only deterministic cube obstacles are currently supported")
        size, start, end = _vector(raw["size"], 3, f"{identifier}.size"), _vector(raw["start"], 3, f"{identifier}.start"), _vector(raw["end"], 3, f"{identifier}.end")
        phase = require_number(raw["phase_jitter"] if version == 1 else raw["jitter_sec"], context=f"{identifier}.phase_jitter")
        mode = raw.get("mode", "linear")
        if min(size) <= 0: raise ValueError(f"{identifier}.size values must be positive")
        if mode not in {"linear", "stationary"}: raise ValueError(f"{identifier}.mode must be linear or stationary")
        if mode == "linear" and start == end: raise ValueError(f"{identifier} trajectory must have non-zero length")
        if phase < 0: raise ValueError(f"{identifier}.phase_jitter must be non-negative")
        repeat = raw.get("repeat", False)
        if not isinstance(repeat, bool): raise ValueError(f"{identifier}.repeat must be boolean")
        parsed.append(ObstacleSpec(identifier, "cube", size, require_number(raw["mass"], context=f"{identifier}.mass", positive=True), start, end,
                                   require_number(raw["speed"], context=f"{identifier}.speed", positive=(mode == "linear")), phase, repeat, mode, raw.get("trigger_group"),
                                   require_number(raw.get("delay_sec", 0.0), context=f"{identifier}.delay_sec"), raw.get("post_motion", "hold")))
    for item in parsed:
        if item.trigger_group is not None and (not isinstance(item.trigger_group, str) or not item.trigger_group): raise ValueError(f"{item.obstacle_id}.trigger_group must be a non-empty string")
        if item.delay_sec < 0 or item.post_motion not in {"hold", "retire"}: raise ValueError(f"invalid trigger policy for {item.obstacle_id}")
    return DynamicScenario(data["seed"], data["enabled"], tuple(parsed), coordinate_frame, spawn_pose_name)
