from __future__ import annotations

from dataclasses import FrozenInstanceError
import json
from types import SimpleNamespace

import pytest

from isaac_sim.src.diagnostics.imu_regime_phase_trace import (
    ImuRegimePhaseTrace,
    SCHEMA,
    make_imu_graph_reader,
)
from isaac_sim.src.ground_truth import recorder as recorder_module
from isaac_sim.src.ground_truth.recorder import GroundTruthReceipt, GroundTruthRecorder


class _Robot:
    def __init__(self):
        self.pose_reads = 0
        self.velocity_reads = 0

    def get_world_pose(self):
        self.pose_reads += 1
        return (1.0, 2.0, 0.3), (1.0, 0.0, 0.0, 0.0)

    def get_base_velocities(self):
        self.velocity_reads += 1
        return (0.25, 0.0, 0.0), (0.0, 0.0, -0.5)


class _Attribute:
    def __init__(self, value):
        self.value = value
        self.get_count = 0

    def get(self):
        self.get_count += 1
        return self.value


def _assist():
    return SimpleNamespace(
        enabled=True,
        state=SimpleNamespace(
            target_linear_speed=0.25,
            target_yaw_rate=-0.5,
            last_applied_linear_speed=0.25,
            last_applied_yaw_rate=-0.4625,
        ),
    )


def test_trace_is_getter_only_buffers_and_flushes_on_limit_and_reset(tmp_path):
    attributes = {}

    def lookup(path):
        value = [0.0, 0.0, -0.5] if "ang" in path.lower() else 1.25
        attributes[path] = _Attribute(value)
        return attributes[path]

    graph_reader = make_imu_graph_reader(lookup)
    robot = _Robot()
    trace_path = tmp_path / "phase.jsonl"
    trace = ImuRegimePhaseTrace(trace_path, flush_loops=2)

    for loop, generation in ((0, 1), (1, 1), (2, 2)):
        trace.begin_loop(
            loop_sequence=loop,
            reset_generation=generation,
            simulation_time_s=1.0 + loop,
            before_app_monotonic_ns=100 + loop,
            robot=robot,
            motion_assist=_assist(),
        )
        trace.after_app(
            simulation_time_s=1.01 + loop,
            after_app_monotonic_ns=200 + loop,
            robot=robot,
            imu_graph=graph_reader(),
        )
        trace.after_assist(
            after_assist_monotonic_ns=300 + loop,
            robot=robot,
            motion_assist=_assist(),
            applied=True,
        )
        trace.finish_loop(
            before_ground_truth_monotonic_ns=400 + loop,
            after_ground_truth_monotonic_ns=500 + loop,
            ground_truth_receipt=GroundTruthReceipt(
                stamp_ns=1_000_000_000 + loop,
                simulation_time_s=1.0 + loop,
                position_xyz=(1.0, 2.0, 0.3),
                yaw_rad=0.2,
                post_assist_yaw_rate_radps=-0.4625,
            ),
        )
    trace.close()

    rows = [json.loads(line) for line in trace_path.read_text().splitlines()]
    assert rows[0] == {"kind": "manifest", "passive": True, "schema": SCHEMA}
    assert [row["loop_sequence"] for row in rows[1:]] == [0, 1, 2]
    assert rows[1]["pre_assist_body"]["yaw_rate_radps"] == -0.5
    assert rows[1]["post_assist_body"]["forward_speed_mps"] == 0.25
    assert rows[1]["assist"]["applied"] is True
    assert rows[1]["ground_truth_receipt"]["stamp_ns"] == 1_000_000_000
    assert rows[1]["imu_graph_after_app"]["read_imu_ang_vel"]["error"] is None
    assert all(attribute.get_count == 3 for attribute in attributes.values())
    assert not hasattr(robot, "set_base_velocities")


def test_missing_graph_attributes_are_observable_not_fabricated():
    def lookup(path):
        if "ReadIMU" in path:
            raise LookupError("missing node")
        return _Attribute(None)

    row = make_imu_graph_reader(lookup)()
    assert row["read_imu_ang_vel"]["value"] is None
    assert "missing node" in row["read_imu_ang_vel"]["error"]
    assert row["publish_imu_timestamp_s"] == {"value": None, "error": None}


def test_navigation_flag_is_passive_and_not_a_command_diagnostic_mode():
    source = (
        __import__("pathlib").Path(__file__).resolve().parents[1]
        / "apps/navigation_sim.py"
    ).read_text(encoding="utf-8")
    assert '"--imu-regime-phase-trace"' in source
    block = source.split("diagnostic_command_mode = any((", 1)[1].split("))", 1)[0]
    assert "imu_regime" not in block
    assert "app.update()" not in (
        __import__("pathlib").Path(__file__).resolve().parents[1]
        / "src/diagnostics/imu_regime_phase_trace.py"
    ).read_text(encoding="utf-8")


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Odometry:
    def __init__(self):
        vector = lambda: SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.child_frame_id = ""
        self.pose = SimpleNamespace(
            pose=SimpleNamespace(
                position=vector(),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            )
        )


class _Path:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.poses = []


def test_ground_truth_update_returns_frozen_receipt_without_changing_rate(monkeypatch):
    recorder = object.__new__(GroundTruthRecorder)
    recorder.config = SimpleNamespace(
        odom_hz=10.0,
        path_hz=1.0,
        frame_id="map",
        child_frame_id="base_link",
    )
    recorder.robot = _Robot()
    recorder.map_t_usd = SimpleNamespace(x=0.0, y=0.0, yaw=0.0)
    recorder._Odometry = _Odometry
    recorder._Path = _Path
    recorder._odom_publisher = _Publisher()
    recorder._path_publisher = _Publisher()
    recorder._path = _Path()
    recorder._last_odom_time = float("-inf")
    recorder._last_path_time = 1.0
    monkeypatch.setattr(
        recorder_module,
        "_stamp_from_seconds",
        lambda value: SimpleNamespace(sec=int(value), nanosec=int((value % 1) * 1e9)),
    )

    receipt = recorder.update(1.25)
    assert receipt == GroundTruthReceipt(
        stamp_ns=1_250_000_000,
        simulation_time_s=1.25,
        position_xyz=(1.0, 2.0, 0.3),
        yaw_rad=0.0,
        post_assist_yaw_rate_radps=-0.5,
    )
    assert len(recorder._odom_publisher.messages) == 1
    assert recorder.update(1.30) is None
    assert len(recorder._odom_publisher.messages) == 1
    with pytest.raises(FrozenInstanceError):
        receipt.yaw_rad = 1.0
