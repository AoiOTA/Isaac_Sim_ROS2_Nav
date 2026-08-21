from pathlib import Path
import struct
from types import SimpleNamespace

import pytest

from robot_experiments.v6_imu_lidar_preflight import (
    TopicState,
    _parse_args,
    create_preflight_node,
    evaluate,
    load_contract,
    observe,
    sim_time_now_s,
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


def test_zero_clock_and_system_time_age_fail_closed():
    states = {"/scan": TopicState()}
    observe(states["/scan"], _scan(9.7), pointcloud=False)
    observe(states["/scan"], _scan(9.8), pointcloud=False)

    zero = evaluate(states, now_s=0.0, min_messages=2, max_age_sec=0.4)
    assert zero["verdict"] == "STOP"
    assert zero["clock"] == {
        "ready": False, "now_s": 0.0, "required_type": "ROS_TIME"
    }
    assert zero["topics"]["/scan"]["age_sec"] is None

    system_time = evaluate(
        states, now_s=1_777_000_000.0, min_messages=2, max_age_sec=0.4
    )
    assert system_time["verdict"] == "STOP"
    assert system_time["topics"]["/scan"]["age_sec"] > 1_000_000_000.0


def test_entry_node_locks_ros_sim_time_and_rejects_disable():
    import rclpy
    from rclpy.clock import ClockType
    from rclpy.context import Context
    from rclpy.parameter import Parameter

    context = Context()
    rclpy.init(args=[], context=context)
    node = create_preflight_node(context=context)
    try:
        assert node.get_parameter("use_sim_time").value is True
        assert node.get_clock().clock_type == ClockType.ROS_TIME
        assert node.get_clock().now().nanoseconds == 0
        assert sim_time_now_s(node) == 0.0
        changed = node.set_parameters([Parameter("use_sim_time", value=False)])
        assert len(changed) == 1
        assert changed[0].successful is False
        assert node.get_parameter("use_sim_time").value is True
    finally:
        node.destroy_node()
        rclpy.shutdown(context=context)


def test_entry_cli_rejects_ros_arguments():
    with pytest.raises(SystemExit) as raised:
        _parse_args(["--ros-args", "-p", "use_sim_time:=false"])
    assert raised.value.code == 2


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
