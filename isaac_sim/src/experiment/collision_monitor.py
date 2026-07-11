"""Physics-contact collision observability for repeatable experiments."""

from __future__ import annotations

import math
from typing import Any


class CollisionMonitorError(RuntimeError):
    pass


class CollisionMonitor:
    """Publish chassis contact state without feeding it back into navigation."""

    def __init__(
        self,
        base_link_prim: str,
        node: Any,
        *,
        publish_hz: float = 20.0,
        sensor: Any | None = None,
    ) -> None:
        if not base_link_prim.startswith("/"):
            raise CollisionMonitorError("base_link_prim must be absolute")
        if not math.isfinite(publish_hz) or publish_hz <= 0.0:
            raise CollisionMonitorError("publish_hz must be finite and positive")

        from std_msgs.msg import Bool

        if sensor is None:
            from isaacsim.sensors.experimental.physics import Contact, ContactSensor

            sensor = ContactSensor(
                Contact.create(
                    f"{base_link_prim}/navigation_contact_sensor",
                    min_threshold=1.0,
                    max_threshold=1.0e9,
                    radius=-1.0,
                )
            )
        self._sensor = sensor
        self._Bool = Bool
        self._publisher = node.create_publisher(Bool, "/simulation/collision", 10)
        self._period = 1.0 / publish_hz
        self._last_publish_time = -math.inf
        self._latched_collision = False

    def reset(self) -> None:
        self._latched_collision = False
        self._last_publish_time = -math.inf
        self._publisher.publish(self._Bool(data=False))

    def update(self, simulation_time: float) -> None:
        if simulation_time - self._last_publish_time < self._period:
            return
        frame = self._sensor.get_data()
        collision = bool(frame.get("in_contact", False))
        self._latched_collision = self._latched_collision or collision
        self._publisher.publish(self._Bool(data=collision))
        self._last_publish_time = simulation_time

    @property
    def collision_detected(self) -> bool:
        return self._latched_collision
