"""Transactional reset orchestration with explicit, testable hooks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Protocol

from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseManager


class ResetError(RuntimeError):
    pass


class SimulationResetPort(Protocol):
    def pause(self) -> None: ...

    def step(self, *, render: bool) -> None: ...

    def play(self) -> None: ...


@dataclass(frozen=True)
class ResetHooks:
    send_zero_velocity: Callable[[], None]
    clear_controller_state: Callable[[], None]
    reset_odometry: Callable[[str], None]
    reset_ground_truth_path: Callable[[], None]
    reset_dynamic_obstacles: Callable[[int], None]
    clear_costmaps: Callable[[], None]
    publish_map_initial_pose: Callable[[str], None]

    def __post_init__(self) -> None:
        for name, hook in self.__dict__.items():
            if not callable(hook):
                raise ResetError(f"reset hook {name} must be callable")


@dataclass(frozen=True)
class ResetRequest:
    pose_name: str
    navigation_mode: str
    odometry_mode: str
    random_seed: int

    def __post_init__(self) -> None:
        if self.navigation_mode not in {"mapping", "localization"}:
            raise ResetError("navigation_mode must be mapping or localization")
        if self.odometry_mode not in {"ideal", "realistic"}:
            raise ResetError("odometry_mode must be ideal or realistic")
        if isinstance(self.random_seed, bool) or not isinstance(self.random_seed, int) or self.random_seed < 0:
            raise ResetError("random_seed must be a non-negative integer")


class ResetManager:
    def __init__(self, simulation: SimulationResetPort, spawn_manager: SpawnPoseManager, hooks: ResetHooks):
        self.simulation = simulation
        self.spawn_manager = spawn_manager
        self.hooks = hooks

    def reset(self, request: ResetRequest) -> None:
        """Execute a reset in a fixed order; every subsystem hook is mandatory."""

        # Validate everything that can be checked without side effects before
        # pausing or teleporting the robot.  Localization resets must never
        # discover an unknown/uncalibrated map pose after physical state moved.
        self.spawn_manager.get(request.pose_name)
        if request.navigation_mode == "localization":
            self.spawn_manager.get_map_pose(
                request.pose_name,
                purpose="localization reset",
            )

        self.simulation.pause()
        try:
            self.hooks.send_zero_velocity()
            self.hooks.clear_controller_state()
            self.spawn_manager.apply_usd_pose(request.pose_name)
            self.hooks.reset_odometry(request.odometry_mode)
            self.hooks.reset_ground_truth_path()
            self.hooks.reset_dynamic_obstacles(request.random_seed)
            self.hooks.clear_costmaps()
            self.simulation.step(render=False)
            if request.navigation_mode == "localization":
                # The concrete hook must apply the calibration gate before ROS publication.
                self.hooks.publish_map_initial_pose(request.pose_name)
        except Exception as exc:
            raise ResetError(f"reset failed before simulation resume: {exc}") from exc
        finally:
            self.simulation.play()
