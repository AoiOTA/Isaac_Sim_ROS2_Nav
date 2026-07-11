"""Strict YAML configuration helpers with no ROS dependencies."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


class ConfigurationError(ValueError):
    """Raised when a project YAML file violates its configuration contract."""


def load_yaml_mapping(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f"configuration file does not exist: {source}")
    try:
        document = yaml.safe_load(source.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigurationError(f"invalid YAML in {source}: {exc}") from exc
    if not isinstance(document, dict):
        raise ConfigurationError(f"{source} must contain a YAML mapping")
    return document


def require_mapping(value: Any, location: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ConfigurationError(f"{location} must be a mapping")
    return value


def require_string(value: Any, location: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ConfigurationError(f"{location} must be a non-empty string")
    return value


def require_finite(value: Any, location: str) -> float:
    if isinstance(value, bool):
        raise ConfigurationError(f"{location} must be a finite number")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigurationError(f"{location} must be a finite number") from exc
    if not math.isfinite(parsed):
        raise ConfigurationError(f"{location} must be a finite number")
    return parsed


def require_vector(value: Any, length: int, location: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ConfigurationError(f"{location} must contain {length} numbers")
    if len(value) != length:
        raise ConfigurationError(f"{location} must contain exactly {length} numbers")
    return tuple(require_finite(component, f"{location}[{index}]") for index, component in enumerate(value))
