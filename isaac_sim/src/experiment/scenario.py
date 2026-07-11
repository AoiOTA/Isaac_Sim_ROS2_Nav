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


@dataclass(frozen=True)
class DynamicScenario:
    seed: int
    enabled: bool
    obstacles: tuple[ObstacleSpec, ...]

    def sampled_phases(self, seed: int | None = None) -> dict[str, float]:
        rng = random.Random(self.seed if seed is None else seed)
        return {
            obstacle.obstacle_id: rng.uniform(-obstacle.phase_jitter, obstacle.phase_jitter)
            for obstacle in self.obstacles
        }


def load_dynamic_scenario(path: str | Path) -> DynamicScenario:
    data = load_mapping(path)
    reject_unknown(data, {"schema_version", "seed", "enabled", "obstacles"}, context="dynamic scenario")
    require_keys(data, {"schema_version", "seed", "enabled", "obstacles"}, context="dynamic scenario")
    if data["schema_version"] != 1 or not isinstance(data["seed"], int) or isinstance(data["seed"], bool):
        raise ValueError("dynamic scenario schema_version/seed is invalid")
    if not isinstance(data["enabled"], bool) or not isinstance(data["obstacles"], list):
        raise ValueError("dynamic scenario enabled/obstacles is invalid")
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
        reject_unknown(raw, allowed, context="dynamic obstacle")
        require_keys(raw, allowed, context="dynamic obstacle")
        identifier = raw["id"]
        if not isinstance(identifier, str) or not identifier or identifier in identifiers:
            raise ValueError(f"invalid or duplicate obstacle id {identifier!r}")
        identifiers.add(identifier)
        if raw["shape"] != "cube":
            raise ValueError("only deterministic cube obstacles are currently supported")
        size = require_vector(raw["size"], 3, context=f"{identifier}.size")
        start = require_vector(raw["start"], 3, context=f"{identifier}.start")
        end = require_vector(raw["end"], 3, context=f"{identifier}.end")
        phase_jitter = require_number(
            raw["phase_jitter"], context=f"{identifier}.phase_jitter"
        )
        if min(size) <= 0.0:
            raise ValueError(f"{identifier}.size values must be positive")
        if start == end:
            raise ValueError(f"{identifier} trajectory must have non-zero length")
        if phase_jitter < 0.0:
            raise ValueError(f"{identifier}.phase_jitter must be non-negative")
        repeat = raw["repeat"]
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
                speed=require_number(
                    raw["speed"], context=f"{identifier}.speed", positive=True
                ),
                phase_jitter=phase_jitter,
                repeat=repeat,
            )
        )
    return DynamicScenario(data["seed"], data["enabled"], tuple(parsed))
