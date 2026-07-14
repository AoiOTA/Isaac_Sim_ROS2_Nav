"""Small strict-YAML helpers shared by pure and runtime modules."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Iterable

import yaml


class YamlConfigError(ValueError):
    pass


class _UniqueKeySafeLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects silently shadowed mapping keys."""


def _construct_unique_mapping(
    loader: _UniqueKeySafeLoader,
    node: yaml.MappingNode,
    deep: bool = False,
) -> dict[Any, Any]:
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as exc:
            raise YamlConfigError("YAML mapping keys must be hashable") from exc
        if duplicate:
            raise YamlConfigError(f"duplicate YAML mapping key: {key!r}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def load_mapping(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.load(stream, Loader=_UniqueKeySafeLoader)
    if not isinstance(value, dict):
        raise YamlConfigError(f"{path} must contain a mapping")
    return value


def require_keys(data: dict[str, Any], required: Iterable[str], *, context: str) -> None:
    missing = sorted(set(required) - set(data))
    if missing:
        raise YamlConfigError(f"missing {context} keys: {missing}")


def reject_unknown(data: dict[str, Any], allowed: Iterable[str], *, context: str) -> None:
    unknown = sorted(set(data) - set(allowed))
    if unknown:
        raise YamlConfigError(f"unknown {context} keys: {unknown}")


def require_number(value: Any, *, context: str, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise YamlConfigError(f"{context} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise YamlConfigError(f"{context} must be finite")
    if positive and result <= 0:
        raise YamlConfigError(f"{context} must be positive")
    return result


def require_vector(value: Any, size: int, *, context: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != size:
        raise YamlConfigError(f"{context} must be a {size}-element list")
    return tuple(require_number(item, context=f"{context}[{index}]") for index, item in enumerate(value))
