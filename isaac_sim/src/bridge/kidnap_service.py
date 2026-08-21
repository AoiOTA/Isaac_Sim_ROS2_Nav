"""Fail-closed true-kidnap service for localization experiments.

Unlike ``/simulation/reset``, this service only moves the physical
articulation.  It never resets odometry/localization state and never publishes
reset or initial-pose events.  The external runner remains responsible for
cancelling any active goal before arming the service.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseError, SpawnPoseManager


class KidnapServiceError(RuntimeError):
    """Kidnap service configuration is invalid."""


@dataclass(frozen=True)
class KidnapResult:
    success: bool
    message: str


class KidnapController:
    """Pure safety guard and one-shot articulation move."""

    def __init__(
        self,
        spawn_manager: SpawnPoseManager,
        *,
        odometry_mode: str,
        clock: Callable[[], float],
        command_freshness_s: float = 0.5,
        zero_hold_s: float = 1.0,
        zero_speed_epsilon: float = 1.0e-6,
    ) -> None:
        if odometry_mode not in {"ideal", "realistic"}:
            raise KidnapServiceError("odometry_mode must be ideal or realistic")
        for value, name, allow_zero in (
            (command_freshness_s, "command_freshness_s", False),
            (zero_hold_s, "zero_hold_s", True),
            (zero_speed_epsilon, "zero_speed_epsilon", True),
        ):
            if (
                not math.isfinite(value)
                or value < 0.0
                or (not allow_zero and value == 0.0)
            ):
                qualifier = "non-negative" if allow_zero else "positive"
                raise KidnapServiceError(f"{name} must be finite and {qualifier}")
        self.spawn_manager = spawn_manager
        self.odometry_mode = odometry_mode
        self.clock = clock
        self.command_freshness_s = command_freshness_s
        self.zero_hold_s = zero_hold_s
        self.zero_speed_epsilon = zero_speed_epsilon
        self._armed = False
        self._consumed = False
        self._last_cmd_stamp_s: float | None = None
        self._zero_since_s: float | None = None

    @property
    def consumed(self) -> bool:
        return self._consumed

    def set_armed(self, armed: bool) -> None:
        if not isinstance(armed, bool):
            raise KidnapServiceError("armed must be boolean")
        if not armed:
            self._armed = False
            self._consumed = False
            return
        if not self._armed:
            self._consumed = False
        self._armed = True

    def observe_cmd(self, stamp_s: float, linear_x: float, angular_z: float) -> None:
        values = (stamp_s, linear_x, angular_z)
        if any(not math.isfinite(value) for value in values) or stamp_s < 0.0:
            raise KidnapServiceError("command observation must be finite with a non-negative stamp")
        speed = math.hypot(linear_x, angular_z)
        gap = (
            None
            if self._last_cmd_stamp_s is None
            else stamp_s - self._last_cmd_stamp_s
        )
        self._last_cmd_stamp_s = stamp_s
        if speed <= self.zero_speed_epsilon:
            if (
                self._zero_since_s is None
                or gap is None
                or gap < 0.0
                or gap > self.command_freshness_s
            ):
                self._zero_since_s = stamp_s
        else:
            self._zero_since_s = None

    def _zero_articulation(self) -> None:
        robot = self.spawn_manager.robot
        zeros = [0.0] * robot.num_dof
        robot.set_base_velocities([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        robot.set_joint_velocities(zeros)
        robot.set_joint_velocity_targets(zeros)

    def trigger(self, pose_name: str) -> KidnapResult:
        now = float(self.clock())
        if not math.isfinite(now) or now < 0.0:
            return KidnapResult(False, "kidnap rejected: invalid simulation clock")
        if self.odometry_mode != "realistic":
            return KidnapResult(False, "kidnap rejected: odometry_mode must be realistic")
        if not self._armed:
            return KidnapResult(False, "kidnap rejected: kidnap_armed=false")
        if self._consumed:
            return KidnapResult(False, "kidnap rejected: armed request already consumed")
        if not isinstance(pose_name, str) or not pose_name:
            return KidnapResult(False, "kidnap rejected: kidnap_pose_name is empty")
        try:
            self.spawn_manager.get(pose_name)
        except SpawnPoseError as exc:
            return KidnapResult(False, f"kidnap rejected: {exc}")
        if self._last_cmd_stamp_s is None or self._zero_since_s is None:
            return KidnapResult(False, "kidnap rejected: no zero /cmd_vel hold observed")
        age = now - self._last_cmd_stamp_s
        if age < 0.0 or age > self.command_freshness_s:
            return KidnapResult(False, f"kidnap rejected: /cmd_vel stale age={age:.3f}s")
        held = now - self._zero_since_s
        if held < self.zero_hold_s:
            return KidnapResult(False, f"kidnap rejected: zero /cmd_vel held {held:.3f}s")

        # Zero both before and after the physical move.  apply_usd_pose performs
        # the second zeroing; the explicit call also makes the precondition
        # independent of the pose-manager implementation.
        self._zero_articulation()
        self.spawn_manager.apply_usd_pose(pose_name)
        self._zero_articulation()
        self._consumed = True
        return KidnapResult(
            True,
            f"kidnap applied: pose={pose_name}; localization/odometry state unchanged",
        )


class KidnapServiceBridge:
    """ROS Trigger wrapper around :class:`KidnapController`."""

    def __init__(
        self,
        node: Any,
        spawn_manager: SpawnPoseManager,
        *,
        odometry_mode: str,
        simulation_time: Callable[[], float],
        service_name: str = "/simulation/kidnap",
    ) -> None:
        from geometry_msgs.msg import Twist
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_srvs.srv import Trigger

        self.node = node
        self._declare_parameter("kidnap_pose_name", "")
        self._declare_parameter("kidnap_armed", False)
        self._declare_parameter("kidnap_cmd_freshness_s", 0.5)
        self._declare_parameter("kidnap_zero_hold_s", 1.0)
        self._declare_parameter("kidnap_zero_speed_epsilon", 1.0e-6)
        self.controller = KidnapController(
            spawn_manager,
            odometry_mode=odometry_mode,
            clock=simulation_time,
            command_freshness_s=self._number_parameter("kidnap_cmd_freshness_s"),
            zero_hold_s=self._number_parameter("kidnap_zero_hold_s"),
            zero_speed_epsilon=self._number_parameter("kidnap_zero_speed_epsilon"),
        )
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self._subscription = node.create_subscription(
            Twist, "/cmd_vel", self._cmd_callback, qos
        )
        self._service = node.create_service(
            Trigger, service_name, self._trigger_callback
        )
        # Observe false->true arming transitions even when no Trigger request is
        # sent while disarmed, so one-shot state can be safely reused.
        self._arming_timer = node.create_timer(0.1, self._sync_arming)

    def _declare_parameter(self, name: str, default: Any) -> None:
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)

    def _number_parameter(self, name: str) -> float:
        value = self.node.get_parameter(name).value
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise KidnapServiceError(f"{name} must be numeric")
        return float(value)

    def _sync_arming(self) -> None:
        value = self.node.get_parameter("kidnap_armed").value
        if not isinstance(value, bool):
            self.controller.set_armed(False)
            return
        self.controller.set_armed(value)

    def _cmd_callback(self, message: Any) -> None:
        self.controller.observe_cmd(
            float(self.controller.clock()),
            float(message.linear.x),
            float(message.angular.z),
        )

    def _trigger_callback(self, _request: Any, response: Any) -> Any:
        self._sync_arming()
        pose_name = self.node.get_parameter("kidnap_pose_name").value
        result = self.controller.trigger(
            pose_name if isinstance(pose_name, str) else ""
        )
        response.success = result.success
        response.message = result.message
        return response

    def close(self) -> None:
        self.controller.set_armed(False)
        self.node.destroy_timer(self._arming_timer)
        self.node.destroy_service(self._service)
        self.node.destroy_subscription(self._subscription)
