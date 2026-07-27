from __future__ import annotations

import json
from types import SimpleNamespace

from isaac_sim.src.diagnostics.odom_phase_trace import OdomPhaseScript, OdomPhaseTrace, SCHEMA


class _State:
    target_linear_speed = 0.5
    target_yaw_rate = -0.5
    last_command_at = 2.0
    last_applied_linear_speed = 0.5
    last_applied_yaw_rate = -0.49


class _Assist:
    state = _State()


class _Robot:
    def get_world_pose(self):
        return (1.0, 2.0, 0.0), (1.0, 0.0, 0.0, 0.0)

    def get_base_velocities(self):
        return (0.5, 0.0, 0.0), (0.0, 0.0, -0.5)


def test_phase_script_has_the_frozen_motion_sequence():
    script = OdomPhaseScript()
    assert script.command(4.9) == (0.0, 0.0, "idle")
    assert script.command(5.0) == (0.0, 0.0, "idle")
    assert script.command(10.1) == (0.5, 0.0, "straight")
    assert script.command(55.0) == (0.5, -0.5, "arc_right")
    assert script.command(60.0) is None
    assert script.complete(61.0)
    assert script.required_end_timecode(60.0) == 3480


def test_trace_is_append_only_and_records_default_velocity_contract(tmp_path):
    path = tmp_path / "phase.jsonl"
    trace = OdomPhaseTrace(path, stage_end_timecode=3480)
    trace.snapshot(
        phase="before_app_update",
        loop_sequence=7,
        simulation_time=1.25,
        robot=_Robot(),
        motion_assist=_Assist(),
        command=(0.5, -0.5, "arc_right"),
        odom_publish={
            "graph_epoch": 3,
            "loop_sequence": 7,
            "trigger_status": True,
            "evaluate_status": True,
            "loop_publish_count": 1,
        },
    )
    trace.record_odom_trigger({
        "graph_epoch": 3,
        "loop_sequence": 7,
        "trigger_status": True,
        "evaluate_status": True,
        "loop_publish_count": 1,
    }, simulation_time=1.25)
    trace.close()
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert rows[0] == {"kind": "manifest", "publish_raw_velocities": False, "schema": SCHEMA, "stage_end_timecode": 3480}
    assert rows[1]["phase"] == "before_app_update"
    assert rows[1]["motion_assist_target"] == [0.5, -0.5]
    assert rows[1]["motion_assist_applied"] == [0.5, -0.49]
    assert rows[1]["ideal_odom_publish"]["graph_epoch"] == 3
    assert rows[2]["kind"] == "ideal_odom_trigger"
    assert rows[2]["loop_publish_count"] == 1


def test_r2b_trace_associates_observed_odom_and_tf_by_trigger_stamp(tmp_path):
    path = tmp_path / "phase.jsonl"
    trace = OdomPhaseTrace(path)
    trace.record_odom_trigger({
        "graph_epoch": 4,
        "loop_sequence": 12,
        "trigger_status": True,
        "evaluate_status": True,
        "loop_publish_count": 1,
    }, simulation_time=1.25)
    stamp = SimpleNamespace(sec=1, nanosec=250_000_000)
    pose = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    twist = SimpleNamespace(
        linear=SimpleNamespace(x=0.5, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=0.25),
    )
    trace.record_odom(SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        pose=SimpleNamespace(pose=pose),
        twist=SimpleNamespace(twist=twist),
    ))
    trace.record_tf(SimpleNamespace(transforms=[SimpleNamespace(
        header=SimpleNamespace(stamp=stamp, frame_id="odom"),
        child_frame_id="base_link",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )]))
    trace.close()

    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert all(row["schema"] == SCHEMA for row in rows)
    odom = next(row for row in rows if row["kind"] == "odom_receive")
    transform = next(row for row in rows if row["kind"] == "tf_receive")
    assert odom["loop_sequence"] == transform["loop_sequence"] == 12
    assert odom["header_stamp_ns"] == transform["header_stamp_ns"] == 1_250_000_000
    assert odom["publisher_payload"] == {
        "position": [1.0, 2.0, 0.0],
        "yaw_rad": 0.0,
        "linear_xyz": [0.5, 0.0, 0.0],
        "angular_xyz": [0.0, 0.0, 0.25],
    }
