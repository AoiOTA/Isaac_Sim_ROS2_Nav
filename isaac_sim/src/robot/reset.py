"""Transactional reset orchestration with explicit, testable hooks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Protocol

from isaac_sim.src.config import ResetStrategyConfig
from isaac_sim.src.robot.reset_strategy import (
    POSE_RESTORE_V1,
    SEPARATE_RECONTACT_0P20M_1STEP_V1,
    ResetStrategySpec,
    reset_strategy_spec,
)
from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseManager


class ResetError(RuntimeError):
    pass


class SimulationResetPort(Protocol):
    def pause(self) -> None: ...

    def step(self, *, render: bool) -> None: ...

    def play(self) -> None: ...


class WheelGroundContactPort(Protocol):
    def assert_all_wheels_separated(self, *, physics_dt_s: float) -> None: ...


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
    def __init__(
        self,
        simulation: SimulationResetPort,
        spawn_manager: SpawnPoseManager,
        hooks: ResetHooks,
        *,
        reset_strategy: ResetStrategyConfig,
        contact_probe: WheelGroundContactPort,
        physics_dt_s: float,
    ):
        if not callable(
            getattr(contact_probe, "assert_all_wheels_separated", None)
        ):
            raise ResetError(
                "contact_probe must provide assert_all_wheels_separated"
            )
        if (
            isinstance(physics_dt_s, bool)
            or not isinstance(physics_dt_s, (int, float))
            or not math.isfinite(physics_dt_s)
            or physics_dt_s <= 0.0
        ):
            raise ResetError("physics_dt_s must be finite and positive")
        self.simulation = simulation
        self.spawn_manager = spawn_manager
        self.hooks = hooks
        self.strategy: ResetStrategySpec = reset_strategy_spec(reset_strategy)
        self.contact_probe = contact_probe
        self.physics_dt_s = float(physics_dt_s)

    def _apply_physical_strategy(self, pose_name: str) -> None:
        if self.strategy.identifier == POSE_RESTORE_V1:
            self.spawn_manager.apply_usd_pose(pose_name)
            return
        if (
            self.strategy.identifier
            == SEPARATE_RECONTACT_0P20M_1STEP_V1
        ):
            self.spawn_manager.apply_usd_pose(
                pose_name,
                z_offset_m=self.strategy.lift_distance_m,
            )
            for _ in range(self.strategy.separation_step_count):
                self.simulation.step(render=False)
            self.contact_probe.assert_all_wheels_separated(
                physics_dt_s=self.physics_dt_s
            )
            # The contact-free step changes gravity-driven root/DOF state.  A
            # second complete restore is therefore mandatory before recontact.
            self.spawn_manager.apply_usd_pose(pose_name)
            return
        raise ResetError(
            f"unsupported validated reset strategy {self.strategy.identifier!r}"
        )

    def _recover_paused_spawn(self, pose_name: str) -> tuple[bool, list[str]]:
        errors: list[str] = []
        try:
            self.simulation.pause()
        except Exception as exc:
            errors.append(f"pause recovery: {type(exc).__name__}: {exc}")
            # Teleporting while the Timeline may still be advancing is less
            # safe than leaving the failed state untouched.  Do not claim the
            # simulation is paused when the recovery pause cannot be verified.
            return False, errors
        try:
            # Deliberately do not step here.  A failed transaction remains
            # paused and may be retried without manufacturing another contact.
            self.spawn_manager.apply_usd_pose(pose_name)
        except Exception as exc:
            errors.append(f"pose recovery: {type(exc).__name__}: {exc}")
        return True, errors

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

        try:
            self.simulation.pause()
            self.hooks.send_zero_velocity()
            self.hooks.clear_controller_state()
            self._apply_physical_strategy(request.pose_name)
            self.hooks.reset_odometry(request.odometry_mode)
            self.hooks.reset_ground_truth_path()
            self.hooks.reset_dynamic_obstacles(request.random_seed)
            # Costmaps belong to the saved-map navigation/localization stack.
            # Mapping mode intentionally has no Nav2 costmap servers, so
            # probing their reset services there only creates false warnings.
            if request.navigation_mode == "localization":
                self.hooks.clear_costmaps()
            for _ in range(self.strategy.recontact_step_count):
                self.simulation.step(render=False)
            if request.navigation_mode == "localization":
                # The concrete hook must apply the calibration gate before ROS publication.
                self.hooks.publish_map_initial_pose(request.pose_name)
            self.simulation.play()
        except Exception as exc:
            recovery_paused, recovery_errors = self._recover_paused_spawn(
                request.pose_name
            )
            recovery_detail = (
                f"; recovery errors={recovery_errors}" if recovery_errors else ""
            )
            recovery_state = (
                "simulation remains paused"
                if recovery_paused
                else (
                    "simulation pause could not be verified and no recovery "
                    "teleport was attempted"
                )
            )
            raise ResetError(
                f"reset failed; {recovery_state}: "
                f"{type(exc).__name__}: {exc}{recovery_detail}"
            ) from exc
