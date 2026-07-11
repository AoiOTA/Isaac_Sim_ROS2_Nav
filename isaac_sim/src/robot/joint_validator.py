"""Wheel-joint mapping and direction validation independent of Isaac Sim."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping


class JointValidationError(RuntimeError):
    pass


@dataclass(frozen=True)
class JointGroups:
    front: tuple[str, str]
    rear: tuple[str, str]

    @property
    def all(self) -> tuple[str, str, str, str]:
        return self.front + self.rear


class JointValidator:
    def __init__(self, expected: Iterable[str], groups: JointGroups):
        self.expected = tuple(expected)
        self.groups = groups
        if len(self.expected) != 4 or len(set(self.expected)) != 4:
            raise JointValidationError("exactly four unique wheel joints are required")
        if set(self.groups.all) != set(self.expected):
            raise JointValidationError("front/rear groups must partition expected wheel joints")

    def validate(self, available_dofs: Iterable[str]) -> tuple[int, int, int, int]:
        available = tuple(available_dofs)
        duplicates = sorted({name for name in available if available.count(name) > 1})
        if duplicates:
            raise JointValidationError(f"duplicate DOF names reported by articulation: {duplicates}")
        missing = [name for name in self.expected if name not in available]
        if missing:
            raise JointValidationError(f"missing wheel DOFs {missing}; available={list(available)}")
        return tuple(available.index(name) for name in self.expected)  # type: ignore[return-value]


def validate_positive_wheel_directions(observed_forward_velocity: Mapping[str, float], expected: Iterable[str]) -> None:
    """Require every +1 rad/s single-wheel trial to trend toward robot +X."""

    expected_tuple = tuple(expected)
    missing = sorted(set(expected_tuple) - set(observed_forward_velocity))
    unknown = sorted(set(observed_forward_velocity) - set(expected_tuple))
    if missing or unknown:
        raise JointValidationError(f"wheel direction observations mismatch: missing={missing}, unknown={unknown}")
    wrong = {name: observed_forward_velocity[name] for name in expected_tuple if observed_forward_velocity[name] <= 0}
    if wrong:
        raise JointValidationError(
            f"positive joint velocity does not produce +X contact trend: {wrong}; fix USD joint frames"
        )
