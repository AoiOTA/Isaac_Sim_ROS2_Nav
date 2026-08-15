#!/usr/bin/env python3
"""Audit recorded Smac paths for duplicate points and local cusps."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def diagnose(points: list[list[float]]) -> dict[str, float | int]:
    duplicate_steps = 0
    reversal_turns = 0
    maximum_turn_deg = 0.0
    for first, second in zip(points, points[1:]):
        if math.dist(first[:2], second[:2]) <= 1.0e-6:
            duplicate_steps += 1
    for first, second, third in zip(points, points[1:], points[2:]):
        incoming = (second[0] - first[0], second[1] - first[1])
        outgoing = (third[0] - second[0], third[1] - second[1])
        incoming_norm = math.hypot(*incoming)
        outgoing_norm = math.hypot(*outgoing)
        if incoming_norm <= 1.0e-9 or outgoing_norm <= 1.0e-9:
            continue
        cosine = (
            incoming[0] * outgoing[0] + incoming[1] * outgoing[1]
        ) / (incoming_norm * outgoing_norm)
        turn_deg = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
        maximum_turn_deg = max(maximum_turn_deg, turn_deg)
        if turn_deg >= 150.0:
            reversal_turns += 1
    return {
        "duplicate_steps": duplicate_steps,
        "reversal_turns": reversal_turns,
        "maximum_turn_deg": maximum_turn_deg,
    }


def audit(run_dir: Path) -> dict[str, object]:
    manifest = json.loads((run_dir / "run_manifest.json").read_text())
    plans = [diagnose(plan.get("points", [])) for plan in manifest["smac_plans"]]
    defective = [
        index for index, record in enumerate(plans)
        if record["duplicate_steps"] or record["reversal_turns"]
    ]
    return {
        "run_dir": str(run_dir.resolve()),
        "plan_count": len(plans),
        "defective_plan_count": len(defective),
        "defective_plan_indices": defective,
        "maximum_duplicate_steps": max(
            (int(record["duplicate_steps"]) for record in plans), default=0
        ),
        "maximum_reversal_turns": max(
            (int(record["reversal_turns"]) for record in plans), default=0
        ),
        "maximum_turn_deg": max(
            (float(record["maximum_turn_deg"]) for record in plans), default=0.0
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--fail-on-defect", action="store_true")
    args = parser.parse_args()
    report = {
        "schema": "attempt30_a21_smac_geometry_audit_v1",
        "runs": [audit(run_dir) for run_dir in args.run_dirs],
    }
    text = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(text)
    print(text, end="")
    has_defect = any(
        int(run["defective_plan_count"]) > 0 for run in report["runs"]
    )
    return 1 if args.fail_on_defect and has_defect else 0


if __name__ == "__main__":
    raise SystemExit(main())
