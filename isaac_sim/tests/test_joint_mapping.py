from __future__ import annotations

import pytest

from isaac_sim.src.robot.joint_validator import (
    JointGroups,
    JointValidationError,
    JointValidator,
    validate_positive_wheel_directions,
)


WHEELS = ("fl", "fr", "rl", "rr")


def test_joint_mapping_uses_names_not_dof_order():
    validator = JointValidator(WHEELS, JointGroups(("fl", "fr"), ("rl", "rr")))
    assert validator.validate(("caster", "rr", "fl", "rl", "fr")) == (2, 4, 3, 1)


def test_missing_and_reversed_wheel_directions_fail_fast():
    validator = JointValidator(WHEELS, JointGroups(("fl", "fr"), ("rl", "rr")))
    with pytest.raises(JointValidationError, match="missing wheel DOFs"):
        validator.validate(("fl", "fr", "rl"))
    with pytest.raises(JointValidationError, match=r"does not produce \+X"):
        validate_positive_wheel_directions(
            {"fl": 0.1, "fr": 0.1, "rl": -0.1, "rr": 0.1}, WHEELS
        )


def test_all_positive_direction_observations_pass():
    validate_positive_wheel_directions({name: 0.01 for name in WHEELS}, WHEELS)
