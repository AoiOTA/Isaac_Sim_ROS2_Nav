"""Deterministic dynamic-obstacle scenario parsing and sampling."""

from __future__ import annotations

from dataclasses import dataclass
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
class DynamicScenario:
    seed: int
    enabled: bool
    obstacles: tuple[ObstacleSpec, ...]
    coordinate_frame: str = "usd"
    spawn_pose_name: str | None = None

    def sampled_phases(self, seed: int | None = None) -> dict[str, float]:
        rng = random.Random(self.seed if seed is None else seed)
        return {
            obstacle.obstacle_id: rng.uniform(-obstacle.phase_jitter, obstacle.phase_jitter)
            for obstacle in self.obstacles
        }


def load_dynamic_scenario(path: str | Path) -> DynamicScenario:
    data = load_mapping(path)
    version = data.get("schema_version")
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
        if not isinstance(raw, dict):
            raise ValueError("each dynamic obstacle must be a mapping")
        allowed = {
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
        if version == 2:
            allowed = {
                "id", "mode", "trigger_group", "size", "mass", "start", "end",
                "speed", "delay_sec", "jitter_sec", "post_motion",
            }
        reject_unknown(raw, allowed, context="dynamic obstacle")
        require_keys(raw, allowed, context="dynamic obstacle")
        identifier = raw["id"]
        if not isinstance(identifier, str) or not identifier or identifier in identifiers:
            raise ValueError(f"invalid or duplicate obstacle id {identifier!r}")
        identifiers.add(identifier)
        if version == 1 and raw["shape"] != "cube":
            raise ValueError("only deterministic cube obstacles are currently supported")
        size = require_vector(raw["size"], 3, context=f"{identifier}.size")
        start = require_vector(raw["start"], 3, context=f"{identifier}.start")
        end = require_vector(raw["end"], 3, context=f"{identifier}.end")
        phase_jitter = require_number(raw["phase_jitter"] if version == 1 else raw["jitter_sec"], context=f"{identifier}.phase_jitter")
        if min(size) <= 0.0:
            raise ValueError(f"{identifier}.size values must be positive")
        mode = raw.get("mode", "linear")
        if mode not in {"linear", "stationary"}:
            raise ValueError(f"{identifier}.mode must be linear or stationary")
        if mode == "linear" and start == end:
            raise ValueError(f"{identifier} trajectory must have non-zero length")
        if phase_jitter < 0.0:
            raise ValueError(f"{identifier}.phase_jitter must be non-negative")
        repeat = raw.get("repeat", False)
        if not isinstance(repeat, bool):
            raise ValueError(f"{identifier}.repeat must be boolean")
        parsed.append(
            ObstacleSpec(
                obstacle_id=identifier,
                shape="cube",
                size=size,  # type: ignore[arg-type]
                mass=require_number(
                    raw["mass"], context=f"{identifier}.mass", positive=True
                ),
                start=start,  # type: ignore[arg-type]
                end=end,  # type: ignore[arg-type]
                speed=require_number(raw["speed"], context=f"{identifier}.speed", positive=(mode == "linear")),
                phase_jitter=phase_jitter,
                repeat=repeat,
                mode=mode,
                trigger_group=raw.get("trigger_group"),
                delay_sec=require_number(raw.get("delay_sec", 0.0), context=f"{identifier}.delay_sec"),
                post_motion=raw.get("post_motion", "hold"),
            )
        )
    for obstacle in parsed:
        if obstacle.trigger_group is not None and (not isinstance(obstacle.trigger_group, str) or not obstacle.trigger_group):
            raise ValueError(f"{obstacle.obstacle_id}.trigger_group must be a non-empty string")
        if obstacle.delay_sec < 0.0:
            raise ValueError(f"{obstacle.obstacle_id}.delay_sec must be non-negative")
        if obstacle.post_motion not in {"hold", "retire"}:
            raise ValueError(f"{obstacle.obstacle_id}.post_motion must be hold or retire")
    return DynamicScenario(data["seed"], data["enabled"], tuple(parsed), coordinate_frame, spawn_pose_name)
