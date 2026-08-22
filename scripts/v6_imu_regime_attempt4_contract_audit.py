#!/usr/bin/env python3
"""Offline replication of the schema-2 capture contract against one capture.

Runs the analyzer's own binding/coverage helpers over every reset epoch and
prints EVERY check outcome instead of stopping at the first failure.  This is
a diagnostic tool only; it produces no scale authority.
"""

import json
import sys
from pathlib import Path

from robot_experiments.imu_regime_analysis import (
    _require_command_coverage,
    _require_zero_range,
    load_mcap,
    validate_benchmark_report,
    DiagnosticResources,
)


def main():
    run_dir = Path(sys.argv[1])
    report = json.loads((run_dir / "analysis/motion_report.json").read_text())
    streams = load_mcap(run_dir / "rosbag/flat20_motion")
    resources = DiagnosticResources.resolve() if hasattr(DiagnosticResources, "resolve") else None
    try:
        validate_benchmark_report(report, resources)
        print("report validation: OK")
    except Exception as exc:
        print("report validation:", type(exc).__name__, exc)

    resets = streams["/simulation/reset_event"]
    topics = ("/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel", "/cmd_vel_sim")
    entries = [report["stationary_reference"]] + list(report["primitives"])
    ends = [r.recorded_s for r in resets[1:]] + [float("inf")]
    for index, entry in enumerate(entries):
        reset = resets[index]
        epoch_end = ends[index]
        epoch = {
            topic: [
                s for s in streams[topic]
                if reset.recorded_s <= s.recorded_s < epoch_end
            ]
            for topic in topics
        }
        print(f"\n== {entry['id']} gen {reset.stamp_s:.3f}")
        schedules = entry.get("segment_schedule") or []
        first_start = float(schedules[0]["start_sim_s"])
        for topic in topics:
            try:
                r = _require_zero_range(
                    epoch[topic], start_s=reset.stamp_s, end_s=first_start,
                    topic=topic, minimum_samples=2, end_exclusive=True,
                )
                print(f"  HOLD {topic}: OK n={r['count']}")
            except Exception as exc:
                print(f"  HOLD {topic}: {getattr(exc, 'code', '?')} {exc}")
        zr = entry.get("final_zero_publish_receipt") or {}
        s0, s1 = float(zr.get("first_sim_s")), float(zr.get("last_sim_s"))
        expected = int(zr.get("publish_count", -1))
        nav = [s for s in epoch["/cmd_vel_nav"] if s0 <= s.stamp_s < s1]
        ok = "OK" if len(nav) == expected else f"MISMATCH {len(nav)} != {expected}"
        print(f"  settle nav count: {ok}")
        for topic in topics:
            try:
                r = _require_zero_range(
                    epoch[topic], start_s=s0, end_s=s1,
                    topic=topic, minimum_samples=2,
                )
                print(f"  settle {topic}: OK n={r['count']}")
            except Exception as exc:
                print(f"  settle {topic}: {getattr(exc, 'code', '?')} {exc}")
        for sched in schedules:
            a, b = float(sched["start_sim_s"]), float(sched["end_sim_s"])
            for topic in topics:
                try:
                    r = _require_command_coverage(
                        epoch[topic], start_s=a, end_s=b, topic=topic,
                        require_zero=entry["id"] == "stationary_reference",
                    )
                    print(f"  seg[{sched['segment_index']}] {topic}: OK n={r['count']}")
                except Exception as exc:
                    print(f"  seg[{sched['segment_index']}] {topic}: {getattr(exc, 'code', '?')} {exc}")
            in_window = [s for s in epoch["/cmd_vel_nav"] if a <= s.stamp_s < b]
            count_ok = "OK" if len(in_window) == int(sched["intent_publish_count"]) else (
                f"MISMATCH {len(in_window)} != {sched['intent_publish_count']}"
            )
            print(f"  seg[{sched['segment_index']}] nav count: {count_ok}")


if __name__ == "__main__":
    main()
