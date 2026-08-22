#!/usr/bin/env python3
"""V6 IMU regime Attempt4 goal-MCAP outcome metadata extractor.

Reads one finalized goal MCAP and writes the schema-1
``goal_mcap_outcome_metadata`` JSON consumed by ``imu_regime_analysis
--goal-evaluator``.  Every field is transcribed from the bag itself; the
caller-supplied ``--requested-seed`` must equal the bag receipt seed or the
helper exits nonzero without writing, so a requested/actual seed debt can
never be transcribed silently.
"""

import argparse
import json
import sys
from pathlib import Path


def _fail(message):
    print(f"goal_metadata: {message}", file=sys.stderr)
    return 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal-mcap", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--requested-seed", type=int, required=True)
    args = parser.parse_args()
    bag = args.goal_mcap.expanduser().resolve()

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
    except Exception as exc:
        return _fail(f"cannot open goal MCAP: {type(exc).__name__}: {exc}")
    try:
        rosbag2_py.ReadOrder(sort_by=rosbag2_py.ReadOrderSortBy.File, reverse=False)
    except Exception:
        pass
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}

    reset_events = 0
    receipt_logs = []
    terminals = []
    route_requests = []
    collision_any = False
    collision_count = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        stamp_s = float(stamp) * 1.0e-9
        if topic == "/simulation/reset_event":
            reset_events += 1
        elif topic == "/rosout":
            message = deserialize_message(data, get_message(topic_types[topic]))
            text = getattr(message, "msg", "")
            if isinstance(text, str) and "reset_receipt=" in text:
                tail = text.split("reset_receipt=", 1)[1]
                try:
                    value, _end = json.JSONDecoder().raw_decode(tail)
                except json.JSONDecodeError:
                    return _fail("malformed reset receipt log in goal MCAP")
                if isinstance(value, dict):
                    receipt_logs.append((stamp_s, value))
        elif topic == "/bio_nav/route_goal_complete":
            message = deserialize_message(data, get_message(topic_types[topic]))
            terminals.append((stamp_s, bool(message.data)))
        elif topic == "/bio_nav/route_goal":
            message = deserialize_message(data, get_message(topic_types[topic]))
            header = message.header
            pose = message.pose
            route_requests.append({
                "recorded_s": stamp_s,
                "header_stamp_s": (
                    float(header.stamp.sec) + float(header.stamp.nanosec) * 1.0e-9
                ),
                "frame_id": str(header.frame_id),
                "position_m": [
                    float(pose.position.x),
                    float(pose.position.y),
                    float(pose.position.z),
                ],
                "orientation_xyzw": [
                    float(pose.orientation.x),
                    float(pose.orientation.y),
                    float(pose.orientation.z),
                    float(pose.orientation.w),
                ],
            })
        elif topic == "/simulation/collision":
            message = deserialize_message(data, get_message(topic_types[topic]))
            collision_count += 1
            if bool(message.data):
                collision_any = True

    if reset_events != 1:
        return _fail(f"goal MCAP has {reset_events} reset events, expected 1")
    if len(receipt_logs) != 1:
        return _fail(f"goal MCAP has {len(receipt_logs)} reset receipt logs, expected 1")
    receipt = receipt_logs[0][1]
    actual_seed = receipt.get("seed")
    if not isinstance(actual_seed, int) or isinstance(actual_seed, bool):
        return _fail("bag receipt seed is not an int")
    if actual_seed != args.requested_seed:
        return _fail(
            f"requested seed {args.requested_seed} != bag receipt seed {actual_seed}"
        )
    generation = receipt.get("generation")
    pose = receipt.get("pose")
    if not isinstance(generation, int) or generation < 1:
        return _fail("bag receipt generation is invalid")
    if not isinstance(pose, str) or not pose:
        return _fail("bag receipt pose is invalid")
    if not terminals:
        return _fail("goal MCAP has no route_goal_complete terminal")
    if len(terminals) != 1 or not terminals[0][1]:
        return _fail(
            f"goal MCAP terminals are not exactly one success: {terminals}"
        )
    successful = [terminals[0][1]]
    if route_requests:
        # The production probe republishes the identical goal at 1 Hz until
        # the route ack lands; identical value identities (frame, position,
        # orientation) collapse into one logical request bound to the first
        # record.  Genuinely different request values cannot be transcribed.
        distinct = {
            (
                request["frame_id"],
                tuple(request["position_m"]),
                tuple(request["orientation_xyzw"]),
            )
            for request in route_requests
        }
        if len(distinct) != 1:
            return _fail(
                f"goal MCAP has {len(distinct)} distinct route request values "
                f"across {len(route_requests)} records, expected one logical request"
            )

    metadata = {
        "schema_version": 1,
        "source": "goal_mcap_outcome_metadata",
        "source_mcap": str(bag),
        "reset_receipt": {
            "requested_seed": int(args.requested_seed),
            "actual_seed": int(actual_seed),
            "generation": int(generation),
            "pose": pose,
        },
        "outcome": "SUCCEEDED" if successful else "FAILED",
        "collision_detected": bool(collision_any),
        "collision_message_count": collision_count,
    }
    if route_requests:
        metadata["route_goal_request"] = route_requests[0]
        metadata["route_request_recorded_count"] = len(route_requests)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({"output": str(args.output), "outcome": metadata["outcome"]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
