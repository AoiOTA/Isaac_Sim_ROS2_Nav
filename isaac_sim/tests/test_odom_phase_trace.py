from __future__ import annotations

import json

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
    )
    trace.close()
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert rows[0] == {"kind": "manifest", "publish_raw_velocities": False, "schema": SCHEMA, "stage_end_timecode": 3480}
    assert rows[1]["phase"] == "before_app_update"
    assert rows[1]["motion_assist_target"] == [0.5, -0.5]
    assert rows[1]["motion_assist_applied"] == [0.5, -0.49]
