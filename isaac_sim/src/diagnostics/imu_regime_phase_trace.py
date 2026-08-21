"""Default-off, passive phase trace for the V6 IMU regime diagnostic."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
import json
import math
from pathlib import Path
from typing import Any, Callable


SCHEMA = "bio_nav_v6_imu_regime_phase_trace_v1"
_GRAPH_ATTRIBUTES = {
    "read_imu_ang_vel": "/World/Graphs/Sensors/ReadIMU.outputs:angVel",
    "read_imu_sensor_time_s": "/World/Graphs/Sensors/ReadIMU.outputs:sensorTime",
    "publish_imu_angular_velocity": "/World/Graphs/Sensors/PublishIMU.inputs:angularVelocity",
    "publish_imu_timestamp_s": "/World/Graphs/Sensors/PublishIMU.inputs:timeStamp",
}


def _validated_graph_value(key: str, value: Any) -> float | list[float] | None:
    """Keep missing values visible and reject malformed/non-finite values."""

    if value is None:
        return None
    if key in {"read_imu_ang_vel", "publish_imu_angular_velocity"}:
        try:
            result = [float(item) for item in value]
        except (TypeError, ValueError) as exc:
            raise ValueError("expected a 3-vector") from exc
        if len(result) != 3:
            raise ValueError(f"expected a 3-vector, got length {len(result)}")
        if not all(math.isfinite(item) for item in result):
            raise ValueError("3-vector contains a non-finite value")
        return result
    if isinstance(value, bool):
        raise ValueError("timestamp must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("timestamp is non-finite")
    return result


def _json_value(value: Any) -> Any:
    """Normalize runtime vector/scalar values without inventing missing data."""

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if is_dataclass(value):
        return _json_value(asdict(value))
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    try:
        return [_json_value(item) for item in value]
    except TypeError:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None


def body_velocity(robot: Any) -> dict[str, object]:
    """Read current forward speed and yaw rate; never mutate the articulation."""

    try:
        _position, orientation = robot.get_world_pose()
        linear, angular = robot.get_base_velocities()
        w, x, y, z = (float(item) for item in orientation)
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        forward = float(linear[0]) * math.cos(yaw) + float(linear[1]) * math.sin(yaw)
        yaw_rate = float(angular[2])
        if not math.isfinite(forward) or not math.isfinite(yaw_rate):
            raise ValueError("non-finite body velocity")
        return {"forward_speed_mps": forward, "yaw_rate_radps": yaw_rate, "error": None}
    except Exception as exc:
        return {
            "forward_speed_mps": None,
            "yaw_rate_radps": None,
            "error": f"{type(exc).__name__}: {exc}",
        }


def assist_state(motion_assist: Any, *, applied: bool | None = None) -> dict[str, object]:
    """Read diagnostic state already maintained by MotionAssist."""

    try:
        state = motion_assist.state
        return {
            "enabled": bool(motion_assist.enabled),
            "target": [float(state.target_linear_speed), float(state.target_yaw_rate)],
            "applied": None if applied is None else bool(applied),
            "last_applied": (
                None
                if state.last_applied_linear_speed is None or state.last_applied_yaw_rate is None
                else [float(state.last_applied_linear_speed), float(state.last_applied_yaw_rate)]
            ),
            "error": None,
        }
    except Exception as exc:
        return {
            "enabled": None,
            "target": None,
            "applied": None if applied is None else bool(applied),
            "last_applied": None,
            "error": f"{type(exc).__name__}: {exc}",
        }


def make_imu_graph_reader(attribute_lookup: Callable[[str], Any]) -> Callable[[], dict[str, object]]:
    """Create a getter-only OmniGraph reader with per-attribute error receipts."""

    attributes: dict[str, Any] = {}
    lookup_errors: dict[str, str] = {}
    for key, path in _GRAPH_ATTRIBUTES.items():
        try:
            attribute = attribute_lookup(path)
            if attribute is None:
                raise LookupError("attribute unavailable")
            attributes[key] = attribute
        except Exception as exc:
            lookup_errors[key] = f"{type(exc).__name__}: {exc}"

    def read() -> dict[str, object]:
        result: dict[str, object] = {}
        for key in _GRAPH_ATTRIBUTES:
            if key in lookup_errors:
                result[key] = {"value": None, "error": lookup_errors[key]}
                continue
            try:
                value = _validated_graph_value(key, attributes[key].get())
                result[key] = {"value": value, "error": None}
            except Exception as exc:
                result[key] = {
                    "value": None,
                    "error": f"{type(exc).__name__}: {exc}",
                }
        return result

    return read


class ImuRegimePhaseTrace:
    """Buffered JSONL writer. Callers provide already-observed loop boundaries."""

    def __init__(
        self,
        path: Path,
        *,
        flush_loops: int = 60,
        provenance: dict[str, object] | None = None,
    ) -> None:
        if isinstance(flush_loops, bool) or int(flush_loops) <= 0:
            raise ValueError("flush_loops must be positive")
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._output = self.path.open("x", encoding="utf-8")
        self._flush_loops = int(flush_loops)
        self._buffer: list[dict[str, object]] = []
        self._pending: dict[str, object] | None = None
        self._last_reset_generation: int | None = None
        manifest: dict[str, object] = {
            "schema": SCHEMA,
            "kind": "manifest",
            "passive": True,
        }
        if provenance is not None:
            manifest["provenance"] = _json_value(provenance)
        self._write_rows([manifest])

    def _write_rows(self, rows: list[dict[str, object]]) -> None:
        for row in rows:
            self._output.write(json.dumps(row, sort_keys=True) + "\n")
        self._output.flush()

    def flush(self) -> None:
        if self._buffer:
            self._write_rows(self._buffer)
            self._buffer.clear()

    def begin_loop(
        self,
        *,
        loop_sequence: int,
        reset_generation: int,
        simulation_time_s: float,
        before_app_monotonic_ns: int,
        robot: Any,
        motion_assist: Any,
    ) -> None:
        if self._pending is not None:
            raise RuntimeError("previous IMU regime trace loop was not completed")
        generation = int(reset_generation)
        if self._last_reset_generation is not None and generation != self._last_reset_generation:
            self.flush()
        self._last_reset_generation = generation
        self._pending = {
            "schema": SCHEMA,
            "kind": "loop",
            "loop_sequence": int(loop_sequence),
            "reset_generation": generation,
            "simulation_time_before_app_s": float(simulation_time_s),
            "before_app_monotonic_ns": int(before_app_monotonic_ns),
            "pre_app_body": body_velocity(robot),
            "pre_app_assist": assist_state(motion_assist),
        }

    def after_app(
        self,
        *,
        simulation_time_s: float,
        after_app_monotonic_ns: int,
        robot: Any,
        imu_graph: dict[str, object],
    ) -> None:
        if self._pending is None:
            raise RuntimeError("IMU regime trace loop was not started")
        self._pending.update({
            "simulation_time_after_app_s": float(simulation_time_s),
            "after_app_monotonic_ns": int(after_app_monotonic_ns),
            "pre_assist_body": body_velocity(robot),
            "imu_graph_after_app": _json_value(imu_graph),
        })

    def after_assist(
        self,
        *,
        after_assist_monotonic_ns: int,
        robot: Any,
        motion_assist: Any,
        applied: bool,
    ) -> None:
        if self._pending is None:
            raise RuntimeError("IMU regime trace loop was not started")
        self._pending.update({
            "after_assist_monotonic_ns": int(after_assist_monotonic_ns),
            "post_assist_body": body_velocity(robot),
            "assist": assist_state(motion_assist, applied=applied),
        })

    def finish_loop(
        self,
        *,
        before_ground_truth_monotonic_ns: int,
        after_ground_truth_monotonic_ns: int,
        ground_truth_receipt: Any | None,
        reset_generation_after_ground_truth: int | None = None,
    ) -> None:
        if self._pending is None:
            raise RuntimeError("IMU regime trace loop was not started")
        self._pending.update({
            "before_ground_truth_monotonic_ns": int(before_ground_truth_monotonic_ns),
            "after_ground_truth_monotonic_ns": int(after_ground_truth_monotonic_ns),
            "ground_truth_receipt": _json_value(ground_truth_receipt),
            "reset_generation_after_ground_truth": (
                None
                if reset_generation_after_ground_truth is None
                else int(reset_generation_after_ground_truth)
            ),
        })
        self._buffer.append(self._pending)
        self._pending = None
        if len(self._buffer) >= self._flush_loops:
            self.flush()

    def close(self) -> None:
        if self._pending is not None:
            # Preserve the partial observation without masking the runtime
            # exception that interrupted the loop.
            self._pending["incomplete"] = True
            self._buffer.append(self._pending)
            self._pending = None
        self.flush()
        if not self._output.closed:
            self._output.close()
