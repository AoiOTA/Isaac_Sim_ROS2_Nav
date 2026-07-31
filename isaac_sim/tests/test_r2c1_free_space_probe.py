from __future__ import annotations

import json
from types import SimpleNamespace

from isaac_sim.src.diagnostics.r2c1_free_space_probe import (
    REQUIRED_CLEARANCE_M,
    R2C1Trace,
    SCHEMA,
    SegmentedFreeSpaceScript,
    is_leaf_collision_prim,
    minimum_xy_clearance,
    wheel_velocities,
)


class _State:
    target_linear_speed = 0.2
    target_yaw_rate = 0.5
    last_applied_linear_speed = 0.2
    last_applied_yaw_rate = 0.5


class _Assist:
    state = _State()


class _Robot:
    def get_world_pose(self):
        return (1.0, 2.0, 0.0), (1.0, 0.0, 0.0, 0.0)

    def get_base_velocities(self):
        return (0.2, 0.0, 0.0), (0.0, 0.0, 0.5)


def _message(*, stamp: int = 1_250_000_000):
    pose = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    twist = SimpleNamespace(
        linear=SimpleNamespace(x=0.2, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=0.5),
    )
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000)
        ),
        pose=SimpleNamespace(pose=pose),
        twist=SimpleNamespace(twist=twist),
    )


def test_segmented_probe_is_short_reset_separated_and_bounded():
    script = SegmentedFreeSpaceScript()
    assert [segment.segment_id for segment in script.segments] == [
        "idle", "straight", "spin_left", "spin_right", "arc_left", "arc_right",
    ]
    assert script.phase(0.0, script.segments[1]) == (0.0, 0.0, "settle")
    assert script.phase(2.0, script.segments[1]) == (0.2, 0.0, "action")
    assert script.phase(5.0, script.segments[1]) == (0.0, 0.0, "idle")
    assert script.segment_duration_s() == 6.0
    assert script.required_end_timecode(60.0) == 2280


def test_swept_clearance_rejects_a_static_obstacle_in_the_arc_envelope():
    segment = SegmentedFreeSpaceScript.segments[-1]
    clear = minimum_xy_clearance(
        start_xy=(0.0, 0.0), yaw_rad=0.0, segment=segment,
        obstacle_bounds_xy=[(0.35, -0.25, 0.50, -0.10)],
        footprint_radius_m=0.10,
    )
    assert clear < REQUIRED_CLEARANCE_M


def test_scene_sized_collision_parent_is_not_an_obstacle_candidate():
    token = object()

    class Prim:
        def __init__(self, collision: bool, descendants=()):
            self.collision = collision
            self.descendants = descendants

        def HasAPI(self, api):
            assert api is token
            return self.collision

    leaf = Prim(True)
    aggregate = Prim(True, (leaf,))
    assert not is_leaf_collision_prim(
        aggregate, collision_api=token,
        prim_range=lambda prim: (item for item in (prim, *prim.descendants)),
    )
    assert is_leaf_collision_prim(
        leaf, collision_api=token,
        prim_range=lambda prim: (item for item in (prim, *prim.descendants)),
    )


def test_wheel_velocity_capture_is_optional_but_not_silently_lost():
    robot = SimpleNamespace(
        articulation=SimpleNamespace(get_dof_velocities=lambda: [[1.0, -2.0]])
    )
    assert wheel_velocities(robot) == [1.0, -2.0]
    assert wheel_velocities(SimpleNamespace()) is None


def test_trace_preserves_reset_preflight_and_causal_callback_contract(tmp_path):
    path = tmp_path / "r2c1.jsonl"
    trace = R2C1Trace(path, manifest={"required_clearance_m": REQUIRED_CLEARANCE_M})
    trace.record_segment_reset(
        segment_index=1, segment_id="straight", reset_epoch=4,
        simulation_time_s=1.0, status="started",
    )
    trace.record_preflight(
        segment_index=1, segment_id="straight", clearance_m=0.6, valid=True,
    )
    after_app = trace.snapshot(
        phase="after_app_update", loop_sequence=12, reset_epoch=4,
        segment_index=1, segment_id="straight", segment_phase="action",
        simulation_time_s=1.25, robot=_Robot(), motion_assist=_Assist(),
    )
    receipt = {
        "loop_publish_count": 1,
        "source_payload": after_app,
        "publisher_payload": after_app,
    }
    trace.record_trigger(
        receipt, simulation_time_s=1.25, loop_sequence=12, reset_epoch=4,
        segment_index=1, segment_id="straight", segment_phase="action",
        post_assist_payload=after_app,
    )
    trace.record_odom(_message(stamp=1_249_999_999), arrival_loop_sequence=13)
    trace.record_realized_next(
        trigger_loop_sequence=12, reset_epoch=4, simulation_time_s=1.3,
        payload=after_app,
    )
    trace.record_collision(SimpleNamespace(data=True), reset_epoch=4)
    trace.record_segment_end(
        segment_index=1, segment_id="straight", reset_epoch=4, clearance_m=0.6,
    )
    trace.close()

    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert rows[0]["schema"] == SCHEMA
    assert rows[0]["kind"] == "manifest"
    trigger = next(row for row in rows if row["kind"] == "ideal_odom_trigger")
    observed = next(row for row in rows if row["kind"] == "odom_receive")
    assert trigger["loop_publish_count"] == 1
    assert trigger["segment_phase"] == "action"
    assert trigger["post_assist_payload"] == trigger["compute_odometry_payload"]
    assert observed["loop_sequence"] == 12
    assert observed["reset_epoch"] == 4
    assert observed["callback_latency_ticks"] == 1
    end = next(row for row in rows if row["kind"] == "segment_end")
    assert end["collision_count"] == 1


def test_reset_epoch_clears_latched_collision_before_next_segment(tmp_path):
    trace = R2C1Trace(tmp_path / "r2c1.jsonl", manifest={})
    trace.record_collision(SimpleNamespace(data=True), reset_epoch=1)
    assert trace.collision_count == 1
    trace.record_segment_reset(
        segment_index=2, segment_id="spin_left", reset_epoch=2,
        simulation_time_s=2.0, status="started",
    )
    assert trace.collision_count == 0
    assert trace.latest_collision is False
    trace.close()
