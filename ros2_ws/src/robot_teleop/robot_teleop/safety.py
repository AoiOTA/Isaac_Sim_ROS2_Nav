"""Pure deadman and velocity-limiting policy for keyboard teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
import math
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

    linear_speed: float = 0.30
    angular_speed: float = 0.60
    max_linear_speed: float = 1.00
    max_angular_speed: float = 1.50
    command_timeout_sec: float = 0.18

    def __post_init__(self) -> None:
        """Reject unsafe or non-finite policy values."""
        values = {
            'linear_speed': self.linear_speed,
            'angular_speed': self.angular_speed,
            'max_linear_speed': self.max_linear_speed,
            'max_angular_speed': self.max_angular_speed,
            'command_timeout_sec': self.command_timeout_sec,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self.command_timeout_sec > MAXIMUM_DEADMAN_TIMEOUT_SEC:
            raise ValueError(
                'command_timeout_sec must be no greater than '
                f'{MAXIMUM_DEADMAN_TIMEOUT_SEC:.2f}s')


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

    def __init__(self, config: TeleopConfig) -> None:
        """Create a stopped controller from validated limits."""
        self.config = config
        self._linear_speed = min(
            config.linear_speed, config.max_linear_speed)
        self._angular_speed = min(
            config.angular_speed, config.max_angular_speed)
        self._command = STOP
        self._deadline: float | None = None
        self._last_now: float | None = None

    def handle_key(self, key: str, now: float) -> bool:
        """Apply one key and return true when the session must exit."""
        if not self._observe_time(now):
            return False
        normalized = key.lower()
        if normalized in self._EXIT_KEYS:
            self.stop()
            return True
        if normalized == ' ':
            self.stop()
            return False
        direction = self._MOTION_KEYS.get(normalized)
        if direction is None:
            # Unknown input must never prolong the previous velocity lease.
            self.stop()
            return False
        self._command = MotionCommand(
            linear_x=direction[0] * self._linear_speed,
            angular_z=direction[1] * self._angular_speed,
        )
        self._deadline = now + self.config.command_timeout_sec
        return False

    def command(self, now: float) -> MotionCommand:
        """Return the leased command, or zero after the deadman deadline."""
        if not self._observe_time(now):
            return STOP
        if self._deadline is not None and now >= self._deadline:
            self.stop()
        return self._command

    def stop(self) -> MotionCommand:
        """Disarm the deadman lease and return the zero command."""
        self._command = STOP
        self._deadline = None
        return STOP

    def _observe_time(self, now: float) -> bool:
        now = float(now)
        if not math.isfinite(now):
            raise ValueError('wall monotonic time must be finite')
        if self._last_now is not None and now < self._last_now:
            self.stop()
            self._last_now = now
            return False
        self._last_now = now
        return True


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

    def handle_key(self, key: str, now: float) -> bool:
        """Handle a key, publishing an immediate zero for stop and exit."""
        should_exit = self.controller.handle_key(key, now)
        if should_exit or self.controller.command(now).stopped:
            self._publish(STOP)
        return should_exit

    def tick(self, now: float) -> MotionCommand:
        """Publish one fixed-rate command sample."""
        command = self.controller.command(now)
        self._publish(command)
        return command

    def close(self) -> None:
        """Publish a final zero command exactly once."""
        if self._closed:
            return
        self.controller.stop()
        self._publish(STOP)
        self._closed = True
