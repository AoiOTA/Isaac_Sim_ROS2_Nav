from __future__ import annotations

from dataclasses import dataclass, field, replace
from pathlib import Path

import pytest

from isaac_sim.src.bridge.reset_service import (
    InitialPoseRepublisher,
    ResetServiceError,
)
from isaac_sim.src.robot.reset import ResetError, ResetHooks, ResetManager, ResetRequest
from isaac_sim.src.robot.spawn_pose_manager import (
    SpawnPoseError,
    SpawnPoseManager,
    load_spawn_poses,
)


ROOT = Path(__file__).resolve().parents[2]


@dataclass
class FakeRobot:
    num_dof: int = 6
    calls: list[tuple[str, object]] = field(default_factory=list)

    def set_world_pose(self, position, orientation_wxyz):
        self.calls.append(("pose", (tuple(position), tuple(orientation_wxyz))))

    def set_base_velocities(self, linear, angular):
        self.calls.append(("base_velocity", (tuple(linear), tuple(angular))))

    def set_joint_velocities(self, values):
        self.calls.append(("joint_velocity", tuple(values)))

    def set_joint_velocity_targets(self, values):
        self.calls.append(("joint_target", tuple(values)))


@dataclass
class FakeSimulation:
    calls: list[str] = field(default_factory=list)

    def pause(self):
        self.calls.append("pause")

    def step(self, *, render):
        self.calls.append(f"step:{render}")

    def play(self):
        self.calls.append("play")


def test_spawn_reset_zeros_all_robot_state_and_gates_map_pose():
    robot = FakeRobot()
    poses = load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    manager = SpawnPoseManager(robot, poses)
    manager.apply_usd_pose("mapping_start")
    assert robot.calls[0][0] == "pose"
    assert robot.calls[-2:] == [
        ("joint_velocity", (0.0,) * 6),
        ("joint_target", (0.0,) * 6),
    ]
    map_pose = manager.get_map_pose("mapping_start", purpose="test localization")
    assert map_pose.position_stddev_m == pytest.approx(0.05)
    assert map_pose.yaw_stddev_deg == pytest.approx(5.0)

    uncalibrated = replace(
        poses["mapping_start"],
        map=replace(poses["mapping_start"].map, calibrated=False),
    )
    gated_manager = SpawnPoseManager(robot, {"mapping_start": uncalibrated})
    with pytest.raises(SpawnPoseError, match="calibrated=false"):
        gated_manager.get_map_pose("mapping_start", purpose="test localization")


def test_reset_manager_executes_every_hook_in_fixed_order():
    robot = FakeRobot()
    spawn = SpawnPoseManager(
        robot, load_spawn_poses(ROOT / "isaac_sim/configs/spawn_poses.yaml")
    )
    simulation = FakeSimulation()
    calls: list[str] = []
    hooks = ResetHooks(
        send_zero_velocity=lambda: calls.append("zero"),
        clear_controller_state=lambda: calls.append("controller"),
        reset_odometry=lambda mode: calls.append(f"odom:{mode}"),
        reset_ground_truth_path=lambda: calls.append("gt"),
        reset_dynamic_obstacles=lambda seed: calls.append(f"dynamic:{seed}"),
        clear_costmaps=lambda: calls.append("costmaps"),
        publish_map_initial_pose=lambda pose: calls.append(f"initial:{pose}"),
    )
    ResetManager(simulation, spawn, hooks).reset(
        ResetRequest("mapping_start", "mapping", "ideal", 123)
    )
    assert simulation.calls == ["pause", "step:False", "play"]
    assert calls == ["zero", "controller", "odom:ideal", "gt", "dynamic:123", "costmaps"]


def test_invalid_reset_request_is_rejected():
    with pytest.raises(ResetError, match="non-negative"):
        ResetRequest("mapping_start", "mapping", "ideal", -1)


def test_initial_pose_republisher_waits_for_post_reset_scans():
    published: list[str] = []
    republisher = InitialPoseRepublisher(
        published.append,
        publish_count=3,
    )

    republisher.schedule("mapping_start", after_stamp_s=10.0)
    assert republisher.pending
    republisher.observe_scan(9.9)
    republisher.observe_scan(10.0)
    assert published == []

    republisher.observe_scan(10.1)
    republisher.observe_scan(10.1)
    republisher.observe_scan(10.2)
    republisher.observe_scan(10.3)
    assert published == ["mapping_start"] * 3
    assert not republisher.pending


def test_initial_pose_republisher_replaces_pending_reset_and_validates_inputs():
    published: list[str] = []
    republisher = InitialPoseRepublisher(
        published.append,
        publish_count=1,
    )
    republisher.schedule("old_pose", after_stamp_s=1.0)
    republisher.schedule("mapping_start", after_stamp_s=2.0)
    republisher.observe_scan(2.1)
    assert published == ["mapping_start"]

    with pytest.raises(ResetServiceError, match="pose name"):
        republisher.schedule("", after_stamp_s=1.0)
    with pytest.raises(ResetServiceError, match="scan barrier"):
        republisher.schedule("mapping_start", after_stamp_s=float("nan"))
    with pytest.raises(ResetServiceError, match="scan stamp"):
        republisher.observe_scan(float("nan"))
    with pytest.raises(ResetServiceError, match="publish_count"):
        InitialPoseRepublisher(published.append, publish_count=0)
