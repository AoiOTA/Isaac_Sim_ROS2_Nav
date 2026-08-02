from dataclasses import dataclass
from pathlib import Path

import pytest

from robot_experiments.static_contact import (
    convex_contact_depth,
    load_robot_footprint,
    static_contact_summary,
)


@dataclass
class Pose:
    x: float
    y: float
    yaw_rad: float
    stamp_s: float


FOOTPRINT = ((0.255, 0.210), (0.255, -0.210), (-0.230, -0.210), (-0.230, 0.210))


def obstacle(x: float, y: float) -> dict:
    return {
        "id": "low_box",
        "position": [x, y, 0.08],
        "position_frame": "map",
        "size": [0.30, 0.30, 0.16],
        "retired": False,
    }


def test_sat_distinguishes_clear_and_contact() -> None:
    box = ((0.3, -0.1), (0.5, -0.1), (0.5, 0.1), (0.3, 0.1))
    clear = ((-0.2, -0.1), (0.2, -0.1), (0.2, 0.1), (-0.2, 0.1))
    contact = ((0.2, -0.1), (0.4, -0.1), (0.4, 0.1), (0.2, 0.1))
    assert convex_contact_depth(clear, box, numerical_margin_m=0.0) is None
    assert convex_contact_depth(contact, box, numerical_margin_m=0.0) == pytest.approx(0.1)


def test_default_contact_contract_accepts_any_positive_clearance() -> None:
    obstacle_box = ((0.3, -0.1), (0.5, -0.1), (0.5, 0.1), (0.3, 0.1))
    robot_with_1mm_clearance = (
        (-0.2, -0.1),
        (0.299, -0.1),
        (0.299, 0.1),
        (-0.2, 0.1),
    )
    assert convex_contact_depth(robot_with_1mm_clearance, obstacle_box) is None


def test_static_contact_catches_low_wheel_or_footprint_contact() -> None:
    result = static_contact_summary(
        [Pose(0.0, 0.0, 0.0, 1.0), Pose(0.4, 0.0, 0.0, 2.0)],
        [obstacle(0.65, 0.0)],
        FOOTPRINT,
    )
    assert result["observed"] is True
    assert result["contact_detected"] is True
    assert result["contacts"][0]["first_stamp_s"] == 2.0
    assert result["control_input"] is False


def test_static_contact_requires_both_pose_and_obstacle_evidence() -> None:
    assert static_contact_summary([], [obstacle(0.0, 0.0)], FOOTPRINT)["observed"] is False
    assert static_contact_summary([Pose(0.0, 0.0, 0.0, 1.0)], [], FOOTPRINT)["observed"] is False


def test_loads_checked_in_robot_footprint() -> None:
    root = Path(__file__).resolve().parents[4]
    footprint = load_robot_footprint(root / "isaac_sim/configs/robots/jackal.yaml")
    assert footprint == FOOTPRINT
