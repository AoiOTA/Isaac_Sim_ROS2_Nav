"""Pure lifecycle decisions and bounded retry policy for Nav2 bringup."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Mapping


class LifecycleAction(str, Enum):
    """A safe manager-level action for a complete managed-node snapshot."""

    STARTUP = 'startup'
    RESUME = 'resume'
    ALREADY_ACTIVE = 'already_active'
    WAIT = 'wait'
    FAIL = 'fail'


@dataclass(frozen=True)
class LifecycleDecision:
    """Result of evaluating every Nav2 managed node together."""

    action: LifecycleAction
    reason: str


@dataclass(frozen=True)
class RetryPolicy:
    """Finite exponential backoff measured with wall-monotonic time."""

    max_attempts: int = 3
    initial_backoff: float = 0.50
    maximum_backoff: float = 2.00

    def __post_init__(self):
        if isinstance(self.max_attempts, bool) or self.max_attempts < 1:
            raise ValueError('max_attempts must be a positive integer')
        values = (self.initial_backoff, self.maximum_backoff)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('retry backoff values must be finite and positive')
        if self.maximum_backoff < self.initial_backoff:
            raise ValueError('maximum_backoff must be at least initial_backoff')

    def delay_after_failure(self, attempt):
        """Return the delay after a one-based failed attempt."""
        if isinstance(attempt, bool) or not isinstance(attempt, int) or attempt < 1:
            raise ValueError('attempt must be a positive integer')
        return min(
            self.maximum_backoff,
            self.initial_backoff * (2 ** (attempt - 1)),
        )

    def can_retry(self, attempts_made):
        """Return whether another manager command may be attempted."""
        return attempts_made < self.max_attempts


_TRANSITIONAL_STATES = {
    'configuring',
    'activating',
    'deactivating',
    'cleaningup',
    'shuttingdown',
}
_TERMINAL_STATES = {'finalized', 'errorprocessing', 'unknown'}


def lifecycle_decision(states: Mapping[str, str]) -> LifecycleDecision:
    """Choose only manager-level, idempotent actions from a full snapshot."""
    if not states:
        return LifecycleDecision(LifecycleAction.FAIL, 'no managed nodes')

    normalized = {name: str(state).lower() for name, state in states.items()}
    state_values = set(normalized.values())
    terminal = {
        name: state
        for name, state in normalized.items()
        if state in _TERMINAL_STATES or not state
    }
    if terminal:
        return LifecycleDecision(
            LifecycleAction.FAIL,
            'terminal or unknown lifecycle states: '
            + _format_states(terminal),
        )
    if state_values & _TRANSITIONAL_STATES:
        return LifecycleDecision(
            LifecycleAction.WAIT,
            'lifecycle transition in progress: ' + _format_states(normalized),
        )
    if state_values == {'unconfigured'}:
        return LifecycleDecision(
            LifecycleAction.STARTUP,
            'all managed nodes are unconfigured',
        )
    if state_values == {'inactive'}:
        return LifecycleDecision(
            LifecycleAction.RESUME,
            'all managed nodes are inactive',
        )
    if state_values == {'active'}:
        return LifecycleDecision(
            LifecycleAction.ALREADY_ACTIVE,
            'all managed nodes are already active',
        )
    return LifecycleDecision(
        LifecycleAction.FAIL,
        'unsafe mixed lifecycle states: ' + _format_states(normalized),
    )


def duplicate_names(names):
    """Return sorted duplicate fully-qualified node names."""
    seen = set()
    duplicates = set()
    for name in names:
        if name in seen:
            duplicates.add(name)
        seen.add(name)
    return sorted(duplicates)


def _format_states(states):
    return ', '.join(f'{name}={state}' for name, state in sorted(states.items()))
