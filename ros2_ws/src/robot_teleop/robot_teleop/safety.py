"""Pure deadman and velocity-limiting policy for keyboard teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Callable


MAXIMUM_DEADMAN_TIMEOUT_SEC = 0.20


@dataclass(frozen=True)
class MotionCommand:
    """Planar velocity command independent of ROS message types."""

    linear_x: float = 0.0
    angular_z: float = 0.0

    @property
    def stopped(self) -> bool:
        """Return true for the canonical zero-velocity command."""
        return self.linear_x == 0.0 and self.angular_z == 0.0


STOP = MotionCommand()


@dataclass(frozen=True)
class TeleopConfig:
    """Validated mapping teleop limits and wall-time deadman timeout."""

    linear_speed: float = 0.50
    angular_speed: float = 0.80
    min_linear_speed: float = 0.10
    min_angular_speed: float = 0.20
    max_linear_speed: float = 1.00
    max_angular_speed: float = 1.50
    linear_speed_step: float = 0.05
    angular_speed_step: float = 0.10
    command_timeout_sec: float = 0.18

    def __post_init__(self) -> None:
        """Reject unsafe or non-finite policy values."""
        values = {
            'linear_speed': self.linear_speed,
            'angular_speed': self.angular_speed,
            'min_linear_speed': self.min_linear_speed,
            'min_angular_speed': self.min_angular_speed,
            'max_linear_speed': self.max_linear_speed,
            'max_angular_speed': self.max_angular_speed,
            'linear_speed_step': self.linear_speed_step,
            'angular_speed_step': self.angular_speed_step,
            'command_timeout_sec': self.command_timeout_sec,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self.command_timeout_sec > MAXIMUM_DEADMAN_TIMEOUT_SEC:
            raise ValueError(
                'command_timeout_sec must be no greater than '
                f'{MAXIMUM_DEADMAN_TIMEOUT_SEC:.2f}s')
        if self.min_linear_speed > self.max_linear_speed:
            raise ValueError(
                'min_linear_speed must be no greater than max_linear_speed')
        if self.min_angular_speed > self.max_angular_speed:
            raise ValueError(
                'min_angular_speed must be no greater than max_angular_speed')


@dataclass(frozen=True)
class SpeedFeedback:
    """One terminal-facing speed update produced by a non-motion key."""

    kind: str
    linear_speed: float
    angular_speed: float
    limits: tuple[str, ...] = ()


class TeleopController:
    """Translate key events into bounded commands with a wall-time lease."""

    _MOTION_KEYS = {
        'w': (1.0, 0.0),
        'up': (1.0, 0.0),
        's': (-1.0, 0.0),
        'down': (-1.0, 0.0),
        'a': (0.0, 1.0),
        'left': (0.0, 1.0),
        'd': (0.0, -1.0),
        'right': (0.0, -1.0),
    }
    _EXIT_KEYS = {'q', '\x03', '\x04', 'eof'}
    _HELP_KEYS = {'h', '?'}
    _SPEED_KEYS = {'+', '=', '-', ']', '[', '.', ',', '0'}

    def __init__(self, config: TeleopConfig) -> None:
        """Create a stopped controller from validated limits."""
        self.config = config
        self._lock = threading.RLock()
        self._default_linear_speed = self._clamp(
            config.linear_speed,
            config.min_linear_speed,
            config.max_linear_speed,
        )
        self._default_angular_speed = self._clamp(
            config.angular_speed,
            config.min_angular_speed,
            config.max_angular_speed,
        )
        self._linear_speed = self._default_linear_speed
        self._angular_speed = self._default_angular_speed
        self._command = STOP
        self._deadline: float | None = None
        self._last_now: float | None = None
        self._feedback: SpeedFeedback | None = None

    def handle_key(self, key: str, now: float) -> bool:
        """Apply one key and return true when the session must exit."""
        with self._lock:
            if not self._observe_time_unlocked(now):
                return False
            normalized = key.lower() if isinstance(key, str) else 'unknown'
            if normalized in self._EXIT_KEYS:
                self._stop_unlocked()
                return True
            if normalized == ' ':
                self._stop_unlocked()
                return False
            if normalized in self._HELP_KEYS:
                self._feedback = self._speed_feedback('help')
                return False
            if normalized in self._SPEED_KEYS:
                self._adjust_speed_unlocked(normalized)
                return False
            direction = self._MOTION_KEYS.get(normalized)
            if direction is None:
                # Unknown input must never prolong the previous velocity lease.
                self._stop_unlocked()
                return False
            self._command = MotionCommand(
                linear_x=direction[0] * self._linear_speed,
                angular_z=direction[1] * self._angular_speed,
            )
            self._deadline = now + self.config.command_timeout_sec
            return False

    def command(self, now: float) -> MotionCommand:
        """Return the leased command, or zero after the deadman deadline."""
        with self._lock:
            if not self._observe_time_unlocked(now):
                return STOP
            if self._deadline is not None and now >= self._deadline:
                self._stop_unlocked()
            return self._command

    def stop(self) -> MotionCommand:
        """Disarm the deadman lease and return the zero command."""
        with self._lock:
            return self._stop_unlocked()

    def speed_feedback(self) -> SpeedFeedback:
        """Return the current target speeds without consuming feedback."""
        with self._lock:
            return self._speed_feedback('status')

    def take_feedback(self) -> SpeedFeedback | None:
        """Consume the latest help/speed event for terminal presentation."""
        with self._lock:
            feedback = self._feedback
            self._feedback = None
            return feedback

    def _stop_unlocked(self) -> MotionCommand:
        self._command = STOP
        self._deadline = None
        return STOP

    def _observe_time_unlocked(self, now: float) -> bool:
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('wall monotonic time must be finite')
        if self._last_now is not None and now < self._last_now:
            self._stop_unlocked()
            self._last_now = now
            return False
        self._last_now = now
        return True

    def _adjust_speed_unlocked(self, key: str) -> None:
        previous_linear = self._linear_speed
        previous_angular = self._angular_speed
        if key == '0':
            requested_linear = self._default_linear_speed
            requested_angular = self._default_angular_speed
        else:
            linear_delta = 0.0
            angular_delta = 0.0
            if key in {'+', '='}:
                linear_delta = self.config.linear_speed_step
                angular_delta = self.config.angular_speed_step
            elif key == '-':
                linear_delta = -self.config.linear_speed_step
                angular_delta = -self.config.angular_speed_step
            elif key == ']':
                linear_delta = self.config.linear_speed_step
            elif key == '[':
                linear_delta = -self.config.linear_speed_step
            elif key == '.':
                angular_delta = self.config.angular_speed_step
            elif key == ',':
                angular_delta = -self.config.angular_speed_step
            requested_linear = previous_linear + linear_delta
            requested_angular = previous_angular + angular_delta

        self._linear_speed = self._clamp(
            requested_linear,
            self.config.min_linear_speed,
            self.config.max_linear_speed,
        )
        self._angular_speed = self._clamp(
            requested_angular,
            self.config.min_angular_speed,
            self.config.max_angular_speed,
        )
        limits = []
        if key != '0':
            if (requested_linear <= self.config.min_linear_speed
                    and requested_linear < previous_linear):
                limits.append('linear minimum')
            elif (requested_linear >= self.config.max_linear_speed
                    and requested_linear > previous_linear):
                limits.append('linear maximum')
            if (requested_angular <= self.config.min_angular_speed
                    and requested_angular < previous_angular):
                limits.append('angular minimum')
            elif (requested_angular >= self.config.max_angular_speed
                    and requested_angular > previous_angular):
                limits.append('angular maximum')

        # A tuning key never starts motion or extends its lease. If a lease is
        # already active, only its magnitude changes and its original deadline
        # remains authoritative.
        self._refresh_active_command_unlocked()
        self._feedback = SpeedFeedback(
            kind='reset' if key == '0' else 'speed',
            linear_speed=self._linear_speed,
            angular_speed=self._angular_speed,
            limits=tuple(limits),
        )

    def _refresh_active_command_unlocked(self) -> None:
        if self._command.stopped:
            return
        linear_direction = math.copysign(1.0, self._command.linear_x) \
            if self._command.linear_x else 0.0
        angular_direction = math.copysign(1.0, self._command.angular_z) \
            if self._command.angular_z else 0.0
        self._command = MotionCommand(
            linear_direction * self._linear_speed,
            angular_direction * self._angular_speed,
        )

    def _speed_feedback(self, kind: str) -> SpeedFeedback:
        return SpeedFeedback(
            kind=kind,
            linear_speed=self._linear_speed,
            angular_speed=self._angular_speed,
        )

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        # Rounding prevents repeated decimal key steps from accumulating a
        # visible 0.799999999 artefact while preserving deterministic limits.
        return round(min(max(float(value), minimum), maximum), 10)


class TeleopRuntime:
    """Connect the pure policy to a publisher with explicit final stopping."""

    def __init__(
        self,
        controller: TeleopController,
        publish: Callable[[MotionCommand], None],
    ) -> None:
        """Create a runtime around a controller and command sink."""
        if not callable(publish):
            raise ValueError('publish must be callable')
        self.controller = controller
        self._publish = publish
        self._closed = False
        self._lock = threading.RLock()

    def handle_key(self, key: str, now: float) -> bool:
        """Handle a key, publishing an immediate zero for stop and exit."""
        with self._lock:
            if self._closed:
                return True
            speeds_before = self.controller.speed_feedback()
            should_exit = self.controller.handle_key(key, now)
            command = self.controller.command(now)
            speeds_after = self.controller.speed_feedback()
            if should_exit or command.stopped:
                self._publish(STOP)
            elif (
                speeds_before.linear_speed != speeds_after.linear_speed
                or speeds_before.angular_speed != speeds_after.angular_speed
            ):
                # Apply an adjusted active command immediately. Its controller
                # deadline was deliberately not extended by the tuning key.
                self._publish(command)
            return should_exit

    def tick(self, now: float) -> MotionCommand:
        """Publish one fixed-rate command sample."""
        with self._lock:
            command = STOP if self._closed else self.controller.command(now)
            self._publish(command)
            return command

    def speed_feedback(self) -> SpeedFeedback:
        """Return the current target speeds for initial terminal output."""
        return self.controller.speed_feedback()

    def take_feedback(self) -> SpeedFeedback | None:
        """Consume a pending speed/help event from the pure controller."""
        return self.controller.take_feedback()

    def close(self) -> None:
        """Publish a final zero command exactly once."""
        with self._lock:
            if self._closed:
                return
            self.controller.stop()
            self._publish(STOP)
            self._closed = True
