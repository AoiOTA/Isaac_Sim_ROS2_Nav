"""Dump fixed-scene Module3 cognitive constraints as a small JSON payload."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .cognitive_constraints import (
    build_cognitive_constraints,
    cognitive_constraints_payload,
    fixed_scene_footprint_settings,
    load_fixed_scene_reachable_override,
    observed_adjacent_transition_report,
)
from .map_io import load_occupancy_map


def _parse_observed_h5(value: str) -> tuple[str, Path]:
    name, separator, raw_path = value.partition("=")
    if not separator or not name.strip() or not raw_path.strip():
        raise argparse.ArgumentTypeError("observed H5 must be NAME=PATH")
    return name.strip(), Path(raw_path).expanduser().resolve()


def dump_fixed_scene_constraints(
    *,
    map_yaml: str | Path,
    scene_config: str | Path,
    output: str | Path,
    include_witnesses: bool,
    observed_h5: list[tuple[str, Path]] | None = None,
) -> dict[str, object]:
    map_path = Path(map_yaml).expanduser().resolve()
    scene_path = Path(scene_config).expanduser().resolve()
    fixed_scene = load_fixed_scene_reachable_override(scene_path)
    footprint = fixed_scene_footprint_settings(fixed_scene)
    occupancy = load_occupancy_map(map_path, unknown_is_occupied=True)
    value = build_cognitive_constraints(
        occupancy,
        map_version=occupancy.map_version,
        graph_revision=0,
        footprint_settings=footprint,
        fixed_scene_override_file=scene_path,
    )
    payload = cognitive_constraints_payload(
        value, include_witnesses=include_witnesses
    )
    if observed_h5:
        import h5py

        reports: dict[str, object] = {}
        for name, path in observed_h5:
            with h5py.File(path, "r") as handle:
                reports[name] = observed_adjacent_transition_report(
                    value,
                    np.asarray(handle["labels_offline_only/state_id"][:]),
                    np.asarray(
                        handle["labels_offline_only/state_label_valid"][:]
                    ),
                )
        payload["observed_gt_transition_coverage"] = reports
    destination = Path(output).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return payload


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--scene-config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--include-witnesses", action="store_true")
    parser.add_argument(
        "--observed-h5",
        action="append",
        default=[],
        type=_parse_observed_h5,
        metavar="NAME=PATH",
        help="report GT-labelled adjacent-transition coverage without adding edges",
    )
    args = parser.parse_args(argv)
    payload = dump_fixed_scene_constraints(
        map_yaml=args.map_yaml,
        scene_config=args.scene_config,
        output=args.output,
        include_witnesses=args.include_witnesses,
        observed_h5=args.observed_h5,
    )
    print(
        json.dumps(
            {
                "map_id": payload["map_id"],
                "valid_states": len(payload["valid_state_ids"]),
                "directed_transitions": len(payload["verified_transitions"]),
                "output": str(Path(args.output).expanduser().resolve()),
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
