"""Load the one editable A21 engineering-defaults file."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def load_engineering_defaults(path: str | Path) -> dict[str, Any]:
    source = Path(path).expanduser().resolve()
    data = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("engineering defaults must be a YAML mapping")
    if data.get("classification") != "engineering_defaults_only":
        raise ValueError("configuration is not marked engineering_defaults_only")
    required = {
        "graph",
        "footprint",
        "module2_edge_prior",
        "route_cost",
        "route_tracking",
        "metric_planning",
        "runtime_edges",
        "structural_updates",
    }
    missing = sorted(required.difference(data))
    if missing:
        raise ValueError(f"engineering defaults missing sections: {missing}")
    return data


__all__ = ["load_engineering_defaults"]
