"""Physics-contact collision observability for repeatable experiments."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from typing import Any


CONTACT_DIAGNOSTIC_SCHEMA = "bio_nav_collision_contact_diagnostic_v1"


class CollisionMonitorError(RuntimeError):
    pass


def _json_value(value: Any) -> Any:
    """Convert Isaac contact values without inventing a contact identity."""

    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    to_list = getattr(value, "tolist", None)
    if callable(to_list):
        return _json_value(to_list())
    if isinstance(value, Mapping):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, Sequence) and not isinstance(value, (bytes, bytearray, str)):
        return [_json_value(item) for item in value]
    return str(value)


def _contact_field(contact: Any, name: str) -> Any:
    if isinstance(contact, Mapping):
        return contact.get(name)
    return getattr(contact, name, None)


def contact_diagnostic(frame: Mapping[str, Any], simulation_time: float, *, raw_data_enabled: bool, raw_data_error: str) -> dict[str, Any]:
    """Return a telemetry-only payload for an already detected contact.

    ``body0`` and ``body1`` are copied exactly as provided by Isaac.  They may
    be numeric handles on a particular Isaac version, so consumers must not
    claim a human-readable prim name unless they independently decode them.
    """

    contacts = frame.get("contacts", []) if raw_data_enabled else []
    if not isinstance(contacts, Sequence) or isinstance(contacts, (bytes, bytearray, str)):
        contacts = []
    return {
        "schema": CONTACT_DIAGNOSTIC_SCHEMA,
        "simulation_time": _json_value(simulation_time),
        "sensor_time": _json_value(frame.get("time")),
        "physics_step": _json_value(frame.get("physics_step")),
        "in_contact": bool(frame.get("in_contact", False)),
        "force": _json_value(frame.get("force")),
        "number_of_contacts": _json_value(frame.get("number_of_contacts")),
        "raw_contact_data_enabled": raw_data_enabled,
        "raw_contact_data_error": raw_data_error,
        "contacts": [
            {
                "body0": _json_value(_contact_field(contact, "body0")),
                "body1": _json_value(_contact_field(contact, "body1")),
                "position": _json_value(_contact_field(contact, "position")),
                "normal": _json_value(_contact_field(contact, "normal")),
                "impulse": _json_value(_contact_field(contact, "impulse")),
                "time": _json_value(_contact_field(contact, "time")),
                "dt": _json_value(_contact_field(contact, "dt")),
            }
            for contact in contacts
        ],
    }


class CollisionMonitor:
    """Publish chassis contact state and read-only diagnostics, never a control input."""

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

        from std_msgs.msg import Bool, String

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
        self._raw_contact_data_enabled = False
        self._raw_contact_data_error = ""
        try:
            self._sensor.add_raw_contact_data_to_frame()
            self._raw_contact_data_enabled = True
        except AttributeError:
            self._raw_contact_data_error = "sensor_has_no_raw_contact_frame_api"
        except Exception as error:  # Diagnostic collection must not weaken the Bool safety gate.
            self._raw_contact_data_error = f"{type(error).__name__}: {error}"
        self._Bool = Bool
        self._String = String
        self._publisher = node.create_publisher(Bool, "/simulation/collision", 10)
        self._diagnostic_publisher = node.create_publisher(String, "/simulation/collision_diagnostics", 10)
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
        if collision:
            self._diagnostic_publisher.publish(
                self._String(
                    data=json.dumps(
                        contact_diagnostic(
                            frame,
                            simulation_time,
                            raw_data_enabled=self._raw_contact_data_enabled,
                            raw_data_error=self._raw_contact_data_error,
                        ),
                        sort_keys=True,
                        separators=(",", ":"),
                    )
                )
            )
        self._last_publish_time = simulation_time

    @property
    def collision_detected(self) -> bool:
        return self._latched_collision
