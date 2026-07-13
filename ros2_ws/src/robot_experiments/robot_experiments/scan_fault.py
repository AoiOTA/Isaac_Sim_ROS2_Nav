"""Deterministic LaserScan fault-injection state machine.

This module deliberately has no ROS imports so the safety-test behaviour can be
proved with ordinary unit tests.  The ROS adapter lives in
``scan_fault_bridge.py``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


class ScanFaultCommandError(ValueError):
    """Raised when a fault command is malformed or targets a stale epoch."""


@dataclass(frozen=True)
class ScanFaultDecision:
    """The action selected for one input scan."""

    forward: bool
    reason: str
    frame_id_override: str | None
    epoch: int
    epoch_changed: bool = False
    state_changed: bool = False


class ScanFaultController:
    """Apply one explicit scan fault at a time and isolate reset epochs."""

    MODES = frozenset(
        {"normal", "drop_next", "pause_for", "drop_all", "replace_frame_id"}
    )

    def __init__(self, *, rollback_tolerance_ns: int = 1_000) -> None:
        if isinstance(rollback_tolerance_ns, bool) or not isinstance(
            rollback_tolerance_ns, int
        ):
            raise ValueError("rollback_tolerance_ns must be an integer")
        if rollback_tolerance_ns < 0:
            raise ValueError("rollback_tolerance_ns must be non-negative")

        self.rollback_tolerance_ns = rollback_tolerance_ns
        self.epoch = 0
        self.mode = "normal"
        self.remaining = 0
        self.pause_until_s: float | None = None
        self.replacement_frame_id: str | None = None
        self.last_stamp_ns: int | None = None
        self.last_command = "normal"
        self.last_epoch_reason = "startup"
        self.command_sequence = 0

        self.total_received = 0
        self.total_forwarded = 0
        self.total_dropped = 0
        self.epoch_received = 0
        self.epoch_forwarded = 0
        self.epoch_dropped = 0

    @staticmethod
    def _validate_now(now_s: float) -> float:
        value = float(now_s)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError("now_s must be finite and non-negative")
        return value

    @staticmethod
    def _validate_stamp(stamp_ns: int) -> int:
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int):
            raise ValueError("stamp_ns must be an integer")
        if stamp_ns < 0:
            raise ValueError("stamp_ns must be non-negative")
        return stamp_ns

    def _set_normal(self) -> None:
        self.mode = "normal"
        self.remaining = 0
        self.pause_until_s = None
        self.replacement_frame_id = None

    def begin_new_epoch(self, reason: str = "reset_event") -> int:
        """Clear every active fault and make stale commands rejectable."""

        reason = str(reason).strip()
        if not reason:
            raise ValueError("epoch reason must not be empty")
        self.epoch += 1
        self._set_normal()
        self.last_stamp_ns = None
        self.last_epoch_reason = reason
        self.epoch_received = 0
        self.epoch_forwarded = 0
        self.epoch_dropped = 0
        return self.epoch

    @staticmethod
    def _check_keys(command: Mapping[str, Any], allowed: set[str]) -> None:
        unknown = set(command) - allowed
        if unknown:
            formatted = ", ".join(sorted(str(key) for key in unknown))
            raise ScanFaultCommandError(f"unknown command field(s): {formatted}")

    def apply_command(self, command: Mapping[str, Any], *, now_s: float) -> dict[str, Any]:
        """Atomically apply a JSON-compatible command mapping.

        Every command must include ``epoch`` so a command queued before Reset is
        rejected if it arrives after the new epoch begins.
        """

        now_s = self._validate_now(now_s)
        if not isinstance(command, Mapping):
            raise ScanFaultCommandError("command must be a JSON object")
        action = command.get("command")
        if not isinstance(action, str) or not action.strip():
            raise ScanFaultCommandError("command field must be a non-empty string")
        action = action.strip()
        if action == "resume":
            action = "normal"
        if action not in self.MODES:
            raise ScanFaultCommandError(f"unsupported command: {action}")

        if "epoch" not in command:
            raise ScanFaultCommandError("epoch is required for every command")
        expected_epoch = command["epoch"]
        if isinstance(expected_epoch, bool) or not isinstance(expected_epoch, int):
            raise ScanFaultCommandError("epoch must be an integer")
        if expected_epoch != self.epoch:
            raise ScanFaultCommandError(
                f"stale epoch {expected_epoch}; current epoch is {self.epoch}"
            )

        common = {"command", "epoch"}
        if action == "drop_next":
            self._check_keys(command, common | {"count"})
            count = command.get("count")
            if isinstance(count, bool) or not isinstance(count, int) or count < 1:
                raise ScanFaultCommandError("drop_next count must be a positive integer")
            self._set_normal()
            self.mode = action
            self.remaining = count
        elif action == "pause_for":
            self._check_keys(command, common | {"seconds"})
            if isinstance(command.get("seconds"), bool):
                raise ScanFaultCommandError(
                    "pause_for seconds must be finite and positive"
                )
            try:
                seconds = float(command.get("seconds"))
            except (TypeError, ValueError) as exc:
                raise ScanFaultCommandError(
                    "pause_for seconds must be finite and positive"
                ) from exc
            if not math.isfinite(seconds) or seconds <= 0.0:
                raise ScanFaultCommandError(
                    "pause_for seconds must be finite and positive"
                )
            self._set_normal()
            self.mode = action
            self.pause_until_s = now_s + seconds
        elif action == "replace_frame_id":
            self._check_keys(command, common | {"frame_id"})
            frame_id = command.get("frame_id")
            if not isinstance(frame_id, str) or not frame_id.strip():
                raise ScanFaultCommandError(
                    "replace_frame_id frame_id must be a non-empty string"
                )
            frame_id = frame_id.strip()
            if frame_id.startswith("/") or any(character.isspace() for character in frame_id):
                raise ScanFaultCommandError(
                    "replacement frame_id must be a relative TF frame without whitespace"
                )
            self._set_normal()
            self.mode = action
            self.replacement_frame_id = frame_id
        else:
            self._check_keys(command, common)
            self._set_normal()
            if action == "drop_all":
                self.mode = action

        self.command_sequence += 1
        self.last_command = str(command.get("command")).strip()
        return self.status(now_s=now_s)

    def process(self, *, stamp_ns: int, now_s: float) -> ScanFaultDecision:
        """Select whether to forward a scan and update observable counters."""

        stamp_ns = self._validate_stamp(stamp_ns)
        now_s = self._validate_now(now_s)
        epoch_changed = False
        if (
            self.last_stamp_ns is not None
            and stamp_ns + self.rollback_tolerance_ns < self.last_stamp_ns
        ):
            self.begin_new_epoch("scan_stamp_rollback")
            epoch_changed = True
        self.last_stamp_ns = stamp_ns

        self.total_received += 1
        self.epoch_received += 1
        state_changed = False
        frame_id_override = None

        if self.mode == "drop_next":
            self.remaining -= 1
            forward = False
            reason = "drop_next"
            if self.remaining == 0:
                self._set_normal()
                state_changed = True
        elif self.mode == "pause_for":
            assert self.pause_until_s is not None
            if now_s < self.pause_until_s:
                forward = False
                reason = "pause_for"
            else:
                self._set_normal()
                forward = True
                reason = "pause_complete"
                state_changed = True
        elif self.mode == "drop_all":
            forward = False
            reason = "drop_all"
        elif self.mode == "replace_frame_id":
            forward = True
            reason = "replace_frame_id"
            frame_id_override = self.replacement_frame_id
        else:
            forward = True
            reason = "normal"

        if forward:
            self.total_forwarded += 1
            self.epoch_forwarded += 1
        else:
            self.total_dropped += 1
            self.epoch_dropped += 1

        return ScanFaultDecision(
            forward=forward,
            reason=reason,
            frame_id_override=frame_id_override,
            epoch=self.epoch,
            epoch_changed=epoch_changed,
            state_changed=state_changed,
        )

    def status(self, *, now_s: float) -> dict[str, Any]:
        """Return a JSON-compatible snapshot for the status Topic."""

        now_s = self._validate_now(now_s)
        pause_remaining_s = 0.0
        if self.pause_until_s is not None:
            pause_remaining_s = max(0.0, self.pause_until_s - now_s)
        return {
            "epoch": self.epoch,
            "mode": self.mode,
            "remaining": self.remaining,
            "pause_remaining_s": pause_remaining_s,
            "replacement_frame_id": self.replacement_frame_id,
            "last_stamp_ns": self.last_stamp_ns,
            "last_command": self.last_command,
            "last_epoch_reason": self.last_epoch_reason,
            "command_sequence": self.command_sequence,
            "total": {
                "received": self.total_received,
                "forwarded": self.total_forwarded,
                "dropped": self.total_dropped,
            },
            "current_epoch": {
                "received": self.epoch_received,
                "forwarded": self.epoch_forwarded,
                "dropped": self.epoch_dropped,
            },
        }
