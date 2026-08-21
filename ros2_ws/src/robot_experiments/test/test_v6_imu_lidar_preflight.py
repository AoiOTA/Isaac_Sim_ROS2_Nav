from pathlib import Path
import struct
from types import SimpleNamespace

from robot_experiments.v6_imu_lidar_preflight import (
    TopicState,
    evaluate,
    load_contract,
    observe,
)


CONFIG = Path(__file__).resolve().parents[1] / "config/v6_imu_lidar_readiness.yaml"


def _stamp(value):
    return SimpleNamespace(sec=int(value), nanosec=int((value % 1.0) * 1e9))


def _cloud(stamp, xyz=(1.0, 2.0, 3.0)):
    fields = [
        SimpleNamespace(name=name, offset=index * 4, datatype=7, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(stamp)),
        fields=fields,
        point_step=12,
        width=1,
        height=1,
        is_bigendian=False,
        data=struct.pack("<fff", *xyz),
    )


def _scan(stamp, ranges=(1.0, float("inf"))):
    return SimpleNamespace(
        header=SimpleNamespace(stamp=_stamp(stamp)), ranges=list(ranges)
    )


def test_locked_contract_names_all_four_live_streams():
    path, contract = load_contract(CONFIG)
    assert path == CONFIG.resolve()
    assert contract["max_age_sec"] == 0.4
    assert contract["strict_header_stamps"] is True
    assert contract["require_finite_returns"] is True
    assert list(contract["topics"]) == [
        "/lidar/points_raw",
        "/lidar/points_scan",
        "/scan",
        "/scan_safety",
    ]


def test_all_streams_require_two_strict_fresh_messages_and_finite_returns():
    states = {topic: TopicState() for topic in (
        "/lidar/points_raw", "/lidar/points_scan", "/scan", "/scan_safety"
    )}
    for topic, state in states.items():
        pointcloud = topic.startswith("/lidar/")
        observe(state, _cloud(9.7) if pointcloud else _scan(9.7), pointcloud=pointcloud)
        observe(state, _cloud(9.8) if pointcloud else _scan(9.8), pointcloud=pointcloud)
    result = evaluate(states, now_s=10.0, min_messages=2, max_age_sec=0.4)
    assert result["verdict"] == "PASS"
    assert all(item["ready"] for item in result["topics"].values())


def test_zero_topic_duplicate_stamp_stale_and_nonfinite_each_stop():
    states = {topic: TopicState() for topic in (
        "/lidar/points_raw", "/lidar/points_scan", "/scan", "/scan_safety"
    )}
    observe(states["/lidar/points_raw"], _cloud(9.8), pointcloud=True)
    observe(states["/lidar/points_raw"], _cloud(9.8), pointcloud=True)
    observe(states["/lidar/points_scan"], _cloud(9.0), pointcloud=True)
    observe(states["/lidar/points_scan"], _cloud(9.1), pointcloud=True)
    observe(states["/scan"], _scan(9.8, (float("inf"),)), pointcloud=False)
    observe(states["/scan"], _scan(9.9, (float("nan"),)), pointcloud=False)
    result = evaluate(states, now_s=10.0, min_messages=2, max_age_sec=0.4)
    assert result["verdict"] == "STOP"
    assert result["topics"]["/lidar/points_raw"]["strict_stamps"] is False
    assert result["topics"]["/lidar/points_scan"]["age_sec"] > 0.4
    assert result["topics"]["/scan"]["finite_return_seen"] is False
    assert result["topics"]["/scan_safety"]["message_count"] == 0
