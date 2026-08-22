#!/usr/bin/env python3
"""V6 reset cold-boundary R5 — per-episode six-invariant evidence checker.

Reads one R5 session run directory (runner JSONL per episode, one
session-long MCAP, per-episode boundary ownership probes) and re-derives the
six minimal reset invariants from the recorded evidence, independently of
the runner's own fail-closed enforcement:

1. reset receipt seed/case/variant/pose/generation/odometry match;
2. post-reset /odom landing+span and /ground_truth/odom landing+span bounded;
3. zero /cmd_vel_sim and /cmd_vel between the reset and the first goal;
4. zero route progress/complete/result traffic before the first goal;
5. /ground_truth/odom subscribed only by the session recorder (boundary probe);
6. sole-publisher ownership at the warm boundary (boundary probe).

Plus the Option A re-arm chain: bridge epoch baseline -> +1 (physical) ->
+2 (bootstrap) per episode with fresh sessions, baseline rollover across
episodes, strictly increasing receipt generations, exactly one reset event
per episode window, and the coordinator "reset event without a pending gate
HOLD" error never logged.

usage: v6_reset_boundary_check.py RUN_DIR [--episodes 7201,7202,7203]
Exit 0 when every check passes, 2 otherwise.
"""

import argparse
import json
import math
import sys
from pathlib import Path

RESET_POSE_XY = (0.45, -5.35)
COMMAND_ZERO_TOLERANCE = 1.0e-3
LANDING_M = 0.10
SPAN_M = 0.10
ROUTE_TOPICS = (
    "/bio_nav/route_progress",
    "/bio_nav/route_goal_complete",
    "/bio_nav/route_goal_result",
)


def _load_jsonl(path):
    rows = []
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _read_mcap(bag_dir):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    series = {
        "odom": [],
        "gt": [],
        "cmd_vel": [],
        "cmd_vel_sim": [],
        "reset_event": [],
        "bridge": [],
        "b5": [],
        "gate_errors": [],
        "route": [],
    }
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        stamp_s = float(stamp) * 1.0e-9
        if topic in ("/odom", "/ground_truth/odom"):
            message = deserialize_message(data, get_message(topic_types[topic]))
            key = "odom" if topic == "/odom" else "gt"
            series[key].append(
                (stamp_s, float(message.pose.pose.position.x),
                 float(message.pose.pose.position.y))
            )
        elif topic in ("/cmd_vel", "/cmd_vel_sim"):
            message = deserialize_message(data, get_message(topic_types[topic]))
            nonzero = any(
                abs(float(value)) > COMMAND_ZERO_TOLERANCE
                for value in (message.linear.x, message.linear.y, message.angular.z)
            )
            series["cmd_vel" if topic == "/cmd_vel" else "cmd_vel_sim"].append(
                (stamp_s, nonzero)
            )
        elif topic == "/simulation/reset_event":
            series["reset_event"].append(stamp_s)
        elif topic in ROUTE_TOPICS:
            series["route"].append((stamp_s, topic))
        elif topic == "/diagnostics":
            message = deserialize_message(data, get_message(topic_types[topic]))
            for status in message.status:
                values = {
                    str(item.key): str(item.value) for item in status.values
                }
                if status.name == "bio_nav_ros_bridge":
                    series["bridge"].append((stamp_s, values))
                elif status.name == "bio_nav_localization_supervisor":
                    series["b5"].append((stamp_s, values))
        elif topic == "/rosout":
            message = deserialize_message(data, get_message(topic_types[topic]))
            text = getattr(message, "msg", "")
            if isinstance(text, str) and "reset event without a pending gate HOLD" in text:
                series["gate_errors"].append((stamp_s, text))
    return series


def _window(rows, start_s, end_s):
    return [row for row in rows if start_s <= row[0] <= end_s]


def _span(rows):
    if len(rows) < 2:
        return None
    xs = [row[1] for row in rows]
    ys = [row[2] for row in rows]
    return max(max(xs) - min(xs), max(ys) - min(ys))


def check_episode(seed, jsonl_path, boundary_path, series):
    events = _load_jsonl(jsonl_path)
    by_event = {}
    for row in events:
        by_event.setdefault(row.get("event"), []).append(row)

    start_rows = by_event.get("episode_start", [])
    receipt_rows = by_event.get("reset_receipt", [])
    result_rows = by_event.get("episode_result", [])
    goal_rows = by_event.get("route_goal_published", [])
    start_s = float(start_rows[0]["wall_time_ns"]) * 1.0e-9 if start_rows else 0.0
    receipt_s = (
        float(receipt_rows[0]["wall_time_ns"]) * 1.0e-9
        if receipt_rows else start_s
    )
    first_goal_s = (
        float(goal_rows[0]["wall_time_ns"]) * 1.0e-9
        if goal_rows else receipt_s
    )

    checks = {}
    result = result_rows[-1] if result_rows else {}
    checks["episode_succeeded"] = {
        "pass": result.get("state") == "SUCCEEDED",
        "state": result.get("state"),
        "stop_reason": result.get("stop_reason", ""),
        "completed_leg_ids": result.get("completed_leg_ids", []),
        "collision": result.get("collision"),
    }

    # 1. receipt match
    receipt = receipt_rows[0] if receipt_rows else {}
    checks["receipt_match"] = {
        "pass": bool(
            receipt
            and receipt.get("actual_seed") == seed
            and receipt.get("requested_seed") == seed
            and receipt.get("pose") == "long_route_start_g1"
            and receipt.get("case_id") == "static"
            and receipt.get("variant_id") == "v1"
            and isinstance(receipt.get("generation"), int)
            and not isinstance(receipt.get("generation"), bool)
            and receipt.get("generation") >= 1
            and receipt.get("odometry") == "realistic"
        ),
        "receipt": {
            key: receipt.get(key)
            for key in (
                "requested_seed", "actual_seed", "pose", "case_id",
                "variant_id", "generation", "odometry",
            )
        },
    }

    # 2. odometry + ground truth landing and span (first sample may straddle)
    odom = _window(series["odom"], receipt_s, first_goal_s)[1:]
    landing = math.hypot(odom[0][1], odom[0][2]) if odom else None
    span = _span(odom)
    gt = _window(series["gt"], receipt_s, first_goal_s)
    gt_landing = (
        math.hypot(gt[0][1] - RESET_POSE_XY[0], gt[0][2] - RESET_POSE_XY[1])
        if gt else None
    )
    gt_span = _span(gt)
    checks["odom_gt_landing"] = {
        "pass": bool(
            landing is not None and landing <= LANDING_M
            and span is not None and span <= SPAN_M
            and gt_landing is not None and gt_landing <= LANDING_M
            and (gt_span is None or gt_span <= SPAN_M)
        ),
        "odom_landing_m": landing,
        "odom_span_m": span,
        "odom_samples": len(odom),
        "gt_landing_m": gt_landing,
        "gt_span_m": gt_span,
        "gt_samples": len(gt),
    }

    # 3. no stale drive replay between reset and first goal
    nonzero_cmds = {
        key: sum(1 for row in _window(series[key], receipt_s, first_goal_s) if row[1])
        for key in ("cmd_vel", "cmd_vel_sim")
    }
    checks["no_stale_drive"] = {
        "pass": not nonzero_cmds["cmd_vel"] and not nonzero_cmds["cmd_vel_sim"],
        "nonzero_cmd_vel": nonzero_cmds["cmd_vel"],
        "nonzero_cmd_vel_sim": nonzero_cmds["cmd_vel_sim"],
    }

    # 4. no stale route traffic before this runner's first goal
    route = _window(series["route"], start_s, first_goal_s)
    checks["no_stale_route"] = {
        "pass": not route,
        "pre_goal_route_messages": len(route),
        "topics": sorted({topic for _stamp, topic in route}),
    }

    # 5/6. boundary ownership probe (recorded live between episodes)
    boundary = json.loads(boundary_path.read_text(encoding="utf-8"))
    checks["gt_firewall"] = {
        "pass": bool(boundary.get("ground_truth_firewall_pass")),
        "ground_truth_odom_max_subscribers": boundary.get(
            "ground_truth_odom_max_subscribers"
        ),
    }
    checks["publisher_ownership"] = {
        "pass": bool(boundary.get("publisher_ownership_pass")),
        "max_publishers": boundary.get("max_publishers"),
    }

    # Exactly one reset event inside this episode window.
    resets = _window(series["reset_event"], start_s, first_goal_s)
    checks["exactly_one_reset_event"] = {
        "pass": len(resets) == 1,
        "reset_events": len(resets),
    }

    # Re-arm chain: the in-window bridge diagnostic epoch/session sequence
    # must roll baseline -> baseline+1 -> baseline+2 with fresh sessions.
    bridge = _window(series["bridge"], start_s, first_goal_s)
    transitions = []
    for stamp_s, values in bridge:
        try:
            epoch = int(values.get("reset_epoch", ""))
        except ValueError:
            continue
        session = values.get("recurrent_session_id", "")
        if not session:
            continue
        if not transitions or transitions[-1][1:] != (epoch, session):
            transitions.append((stamp_s, epoch, session))
    baseline_epoch = transitions[0][1] if transitions else None
    rollover_ok = bool(
        len(transitions) >= 3
        and [row[1] for row in transitions[:3]]
        == [baseline_epoch, baseline_epoch + 1, baseline_epoch + 2]
        and len({row[2] for row in transitions[:3]}) == 3
    )
    checks["bridge_epoch_rollover"] = {
        "pass": rollover_ok,
        "baseline_epoch": baseline_epoch,
        "observed_epochs": [row[1] for row in transitions],
    }

    # B5 readiness string: warm episodes must show the seeded generation
    # string equal to the bridge baseline pair; the cold first episode shows
    # the waiting values instead.
    b5 = _window(series["b5"], start_s, receipt_s)
    raw_generations = [
        values.get("candidate_array_last_generation", "") for _s, values in b5
    ]
    baseline_match = False
    if baseline_epoch is not None and transitions:
        baseline_session = transitions[0][2]
        for text in raw_generations:
            fields = {}
            for item in str(text).split(","):
                key, _, value = item.partition("=")
                fields[key.strip()] = value.strip()
            if (
                fields.get("epoch") == str(baseline_epoch)
                and fields.get("session") == baseline_session
            ):
                baseline_match = True
                break
    cold_waiting = any(
        value in {"not_received", "waiting_after_physical_reset"}
        for value in raw_generations
    )
    checks["b5_readiness_string"] = {
        "pass": bool(baseline_match or cold_waiting),
        "baseline_match": baseline_match,
        "cold_waiting": cold_waiting,
        "observed_generations": sorted(set(raw_generations))[:6],
    }

    return {
        "seed": seed,
        "receipt_generation": receipt.get("generation"),
        "baseline_epoch": baseline_epoch,
        "checks": checks,
        "pass": all(item["pass"] for item in checks.values()),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--episodes", default="7201,7202,7203")
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    seeds = [int(item) for item in args.episodes.split(",") if item]

    series = _read_mcap(run_dir / "rosbag" / "r5_session")
    reports = [
        check_episode(
            seed,
            run_dir / "episodes" / f"episode_seed{seed}.jsonl",
            run_dir / "episodes" / f"boundary_seed{seed}.json",
            series,
        )
        for seed in seeds
    ]

    # Cross-episode re-arm chain: baseline of episode N+1 is the bootstrap
    # epoch of episode N; receipt generations strictly increase.
    baselines = [row["baseline_epoch"] for row in reports]
    chain_ok = all(
        later == earlier + 2 for earlier, later in zip(baselines, baselines[1:])
    ) and all(isinstance(value, int) for value in baselines)
    generations = [row["receipt_generation"] for row in reports]
    generations_ok = all(
        later > earlier for earlier, later in zip(generations, generations[1:])
    ) and all(isinstance(value, int) and not isinstance(value, bool)
              for value in generations)
    gate_errors = series["gate_errors"]

    summary = {
        "run_dir": str(run_dir),
        "episodes": reports,
        "rearm_chain": {
            "pass": chain_ok and generations_ok,
            "baseline_epochs": baselines,
            "receipt_generations": generations,
        },
        "gate_hold_error_absent": {
            "pass": not gate_errors,
            "occurrences": len(gate_errors),
        },
        "pass": bool(
            reports
            and all(row["pass"] for row in reports)
            and chain_ok
            and generations_ok
            and not gate_errors
        ),
    }
    output = run_dir / "analysis" / "boundary_checks.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"output": str(output), "pass": summary["pass"]}))
    return 0 if summary["pass"] else 2


if __name__ == "__main__":
    sys.exit(main())
