from dataclasses import dataclass, field
from pathlib import Path

from isaac_sim.src.bridge.kidnap_service import KidnapController
from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseManager, load_spawn_poses


ROOT = Path(__file__).resolve().parents[2]


@dataclass
class FakeRobot:
    num_dof: int = 4
    calls: list[tuple[str, object]] = field(default_factory=list)

    def set_world_pose(self, position, orientation_wxyz):
        self.calls.append(("pose", (tuple(position), tuple(orientation_wxyz))))

    def set_base_velocities(self, linear, angular):
        self.calls.append(("base", (tuple(linear), tuple(angular))))

    def set_joint_velocities(self, values):
        self.calls.append(("joint", tuple(values)))

    def set_joint_velocity_targets(self, values):
        self.calls.append(("target", tuple(values)))


def _controller(mode="realistic"):
    robot = FakeRobot()
    manager = SpawnPoseManager(
        robot,
        load_spawn_poses(
            ROOT / "isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.spawn.yaml"
        ),
    )
    time = {"now": 0.0}
    controller = KidnapController(manager, odometry_mode=mode, clock=lambda: time["now"])
    return controller, robot, time


def test_kidnap_rejects_ideal_unarmed_unknown_unsafe_and_stale():
    ideal, _, time = _controller("ideal")
    ideal.set_armed(True)
    ideal.observe_cmd(0.0, 0.0, 0.0)
    time["now"] = 1.0
    assert not ideal.trigger("long_route_start_g5").success

    controller, robot, time = _controller()
    assert not controller.trigger("long_route_start_g5").success
    controller.set_armed(True)
    assert not controller.trigger("unknown").success
    controller.observe_cmd(0.0, 0.2, 0.0)
    time["now"] = 0.1
    assert not controller.trigger("long_route_start_g5").success
    controller.observe_cmd(0.2, 0.0, 0.0)
    time["now"] = 1.2
    assert not controller.trigger("long_route_start_g5").success
    assert robot.calls == []


def test_kidnap_is_one_shot_and_zeros_before_and_after_pose_without_reset_hooks():
    controller, robot, time = _controller()
    controller.set_armed(True)
    controller.observe_cmd(0.0, 0.0, 0.0)
    controller.observe_cmd(0.4, 0.0, 0.0)
    controller.observe_cmd(0.8, 0.0, 0.0)
    controller.observe_cmd(1.0, 0.0, 0.0)
    time["now"] = 1.0
    result = controller.trigger("long_route_start_g5")
    assert result.success
    assert "pose=long_route_start_g5" in result.message
    pose_index = next(index for index, call in enumerate(robot.calls) if call[0] == "pose")
    assert pose_index >= 3
    assert [name for name, _ in robot.calls[:3]] == ["base", "joint", "target"]
    assert [name for name, _ in robot.calls[-3:]] == ["base", "joint", "target"]
    assert not controller.trigger("long_route_start_g5").success
    assert sum(name == "pose" for name, _ in robot.calls) == 1


def test_disarm_rearms_a_fresh_one_shot_cycle():
    controller, robot, time = _controller()
    controller.set_armed(True)
    for stamp in (0.0, 0.4, 0.8, 1.0):
        controller.observe_cmd(stamp, 0.0, 0.0)
    time["now"] = 1.0
    assert controller.trigger("long_route_start_g5").success
    controller.set_armed(False)
    controller.set_armed(True)
    for stamp in (1.1, 1.5, 1.9, 2.1):
        controller.observe_cmd(stamp, 0.0, 0.0)
    time["now"] = 2.1
    assert controller.trigger("long_route_start_g5").success
    assert sum(name == "pose" for name, _ in robot.calls) == 2


def test_navigation_app_wires_the_service_without_reset_or_initialpose_publishers():
    app_source = (ROOT / "isaac_sim/apps/navigation_sim.py").read_text(encoding="utf-8")
    source = (ROOT / "isaac_sim/src/bridge/kidnap_service.py").read_text(encoding="utf-8")
    assert "KidnapServiceBridge(" in app_source
    assert 'service_name: str = "/simulation/kidnap"' in source
    assert "create_publisher" not in source
