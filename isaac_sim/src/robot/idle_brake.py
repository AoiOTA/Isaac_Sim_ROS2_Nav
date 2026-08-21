"""Near-rest command watchdog that suppresses accumulated PhysX wheel creep."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable

from isaac_sim.src.robot.articulation_runtime import ArticulationPhysicsConfig


class IdleBrakeError(ValueError):
    pass


@dataclass
class IdleBrakeState:
    timeout_sec: float
    command_deadband: float
    last_command_at: float | None = None
    command_is_active: bool = False

    def __post_init__(self) -> None:
        if not math.isfinite(self.timeout_sec) or self.timeout_sec <= 0.0:
            raise IdleBrakeError("idle brake timeout must be positive")
        if not math.isfinite(self.command_deadband) or self.command_deadband <= 0.0:
            raise IdleBrakeError("idle brake command deadband must be positive")

    def observe(self, linear_x: float, angular_z: float, now: float) -> None:
        values = (float(linear_x), float(angular_z), float(now))
        if not all(math.isfinite(value) for value in values):
            raise IdleBrakeError("idle brake command values must be finite")
        self.last_command_at = values[2]
        self.command_is_active = (
            abs(values[0]) > self.command_deadband
            or abs(values[1]) > self.command_deadband
        )

    def should_brake(self, now: float) -> bool:
        now = float(now)
        if not math.isfinite(now):
            raise IdleBrakeError("idle brake time must be finite")
        if not self.command_is_active or self.last_command_at is None:
            return True
        age = now - self.last_command_at
        return age < 0.0 or age > self.timeout_sec

    def reset(self) -> None:
        self.last_command_at = None
        self.command_is_active = False


class IdleBrake:
    """ROS adapter; braking changes velocity state but never teleports pose."""

    def __init__(
        self,
        node,
        robot,
        settings: ArticulationPhysicsConfig,
        *,
        topic_name: str = "/cmd_vel_sim",
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        from geometry_msgs.msg import Twist

        self.robot = robot
        self.settings = settings
        self.clock = clock
        self.state = IdleBrakeState(
            timeout_sec=settings.idle_brake_command_timeout_sec,
            command_deadband=settings.idle_brake_command_deadband,
        )
        self._braking = False
        self._subscription = node.create_subscription(
            Twist, topic_name, self._command_callback, 1
        )

    def _command_callback(self, message) -> None:
        try:
            self.state.observe(
                message.linear.x,
                message.angular.z,
                self.clock(),
            )
        except IdleBrakeError:
            self.state.reset()

    def update(self) -> bool:
        if not self.state.should_brake(self.clock()):
            if self._braking:
                self.robot.wake_up()
                self._braking = False
            return False
        self.robot.zero_all_velocities()
        self.robot.put_to_sleep()
        self._braking = True
        return True

    def reset(self) -> None:
        self.state.reset()
        self.robot.zero_all_velocities()
        self.robot.put_to_sleep()
        self._braking = True
