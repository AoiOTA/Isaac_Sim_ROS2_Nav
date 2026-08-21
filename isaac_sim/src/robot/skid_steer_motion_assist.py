"""Bounded planar velocity tracking for four-wheel skid-steer simulation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

from isaac_sim.src.robot.articulation_runtime import (
    ArticulationPhysicsConfig,
)


class SkidSteerMotionAssistError(ValueError):
    pass


def _bounded_step(
    current: float,
    target: float,
    maximum_delta: float,
) -> float:
    error = target - current
    delta = max(-maximum_delta, min(maximum_delta, error))
    return current + delta


def _yaw_command_scale(linear_speed: float) -> float:
    """Keep in-place yaw exact while preserving the calibrated arc response."""

    speed = abs(float(linear_speed))
    if speed == 0.0:
        return 1.0
    blend = min(speed / 0.20, 1.0)
    return 0.925 + 0.075 * blend


@dataclass
class SkidSteerMotionAssistState:
    command_timeout_sec: float
    max_linear_acceleration: float
    max_angular_acceleration: float
    last_command_at: float | None = None
    target_linear_speed: float = 0.0
    target_yaw_rate: float = 0.0
    # Diagnostic-only observability for the default-off R2A3 phase trace.
    # These values are never consulted by the control law.
    last_applied_linear_speed: float | None = None
    last_applied_yaw_rate: float | None = None

    def __post_init__(self) -> None:
        values = (
            self.command_timeout_sec,
            self.max_linear_acceleration,
            self.max_angular_acceleration,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise SkidSteerMotionAssistError(
                "motion assist limits must be finite and positive"
            )

    def observe(
        self,
        linear_x: float,
        angular_z: float,
        now: float,
    ) -> None:
        values = (float(linear_x), float(angular_z), float(now))
        if not all(math.isfinite(value) for value in values):
            raise SkidSteerMotionAssistError(
                "motion assist command values must be finite"
            )
        self.target_linear_speed = values[0]
        self.target_yaw_rate = values[1]
        self.last_command_at = values[2]

    def next_command(
        self,
        current_linear_speed: float,
        current_yaw_rate: float,
        now: float,
        dt: float,
    ) -> tuple[float, float] | None:
        values = (
            float(current_linear_speed),
            float(current_yaw_rate),
            float(now),
            float(dt),
        )
        if not all(math.isfinite(value) for value in values):
            raise SkidSteerMotionAssistError(
                "motion assist state values must be finite"
            )
        if values[3] <= 0.0:
            raise SkidSteerMotionAssistError(
                "motion assist time step must be positive"
            )
        if self.last_command_at is None:
            return None
        age = values[2] - self.last_command_at
        if age < 0.0 or age > self.command_timeout_sec:
            return None
        return (
            _bounded_step(
                values[0],
                self.target_linear_speed,
                self.max_linear_acceleration * values[3],
            ),
            _bounded_step(
                values[1],
                self.target_yaw_rate,
                self.max_angular_acceleration * values[3],
            ),
        )

    def reset(self) -> None:
        self.last_command_at = None
        self.target_linear_speed = 0.0
        self.target_yaw_rate = 0.0


class SkidSteerMotionAssist:
    """ROS adapter correcting PhysX skid-steer curvature under-response.

    ``update`` runs after the current physics step has published its sensor
    samples.  Pure-yaw correction must therefore remain unit scale so it does
    not introduce a post-sensor IMU/ground-truth angular-rate disagreement.
    """

    def __init__(
        self,
        node,
        robot,
        settings: ArticulationPhysicsConfig,
        *,
        physics_dt: float,
        topic_name: str = "/cmd_vel_sim",
        clock: Callable[[], float],
    ) -> None:
        from geometry_msgs.msg import Twist

        self.robot = robot
        self.enabled = settings.motion_assist_enabled
        self.physics_dt = float(physics_dt)
        if not math.isfinite(self.physics_dt) or self.physics_dt <= 0.0:
            raise SkidSteerMotionAssistError(
                "motion assist physics time step must be positive"
            )
        self.clock = clock
        self.state = SkidSteerMotionAssistState(
            command_timeout_sec=settings.motion_assist_command_timeout_sec,
            max_linear_acceleration=(
                settings.motion_assist_max_linear_acceleration
            ),
            max_angular_acceleration=(
                settings.motion_assist_max_angular_acceleration
            ),
        )
        self._subscription = node.create_subscription(
            Twist,
            topic_name,
            self._command_callback,
            1,
        )

    def _command_callback(self, message) -> None:
        try:
            self.state.observe(
                message.linear.x,
                message.angular.z,
                self.clock(),
            )
        except SkidSteerMotionAssistError:
            self.state.reset()

    def update(self) -> bool:
        if not self.enabled:
            return False
        linear, angular = self.robot.get_base_velocities()
        _, orientation = self.robot.get_world_pose()
        w, x, y, z = orientation
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        forward_speed = linear[0] * cos_yaw + linear[1] * sin_yaw
        desired = self.state.next_command(
            forward_speed,
            angular[2],
            self.clock(),
            self.physics_dt,
        )
        if desired is None:
            return False

        desired_linear, desired_yaw_rate = desired
        desired_yaw_rate *= _yaw_command_scale(
            self.state.target_linear_speed
        )
        corrected_linear = [
            desired_linear * cos_yaw,
            desired_linear * sin_yaw,
            linear[2],
        ]
        corrected_angular = list(angular)
        corrected_angular[2] = desired_yaw_rate
        self.robot.set_base_velocities(
            corrected_linear,
            corrected_angular,
        )
        self.state.last_applied_linear_speed = desired_linear
        self.state.last_applied_yaw_rate = desired_yaw_rate
        return True

    def reset(self) -> None:
        self.state.reset()
        self.state.last_applied_linear_speed = None
        self.state.last_applied_yaw_rate = None
