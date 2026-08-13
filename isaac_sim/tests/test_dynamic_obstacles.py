from __future__ import annotations

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from isaac_sim.src.experiment.scenario import load_dynamic_scenario
from isaac_sim.src.experiment.dynamic_obstacles import DynamicObstacleManager


ROOT = Path(__file__).resolve().parents[2]


def test_dynamic_obstacle_phases_are_seeded_and_repeatable():
    scenario = load_dynamic_scenario(ROOT / "isaac_sim/configs/experiments/dynamic.yaml")
    assert scenario.sampled_phases(123) == scenario.sampled_phases(123)
    assert scenario.sampled_phases(123) != scenario.sampled_phases(124)
    assert set(scenario.sampled_phases()) == {item.obstacle_id for item in scenario.obstacles}
    assert scenario.enabled is False


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("size", [0.5, 0.0, 1.0], "size values must be positive"),
        ("phase_jitter", -0.1, "must be non-negative"),
        ("speed", float("nan"), "must be finite"),
        ("repeat", "false", "repeat must be boolean"),
    ],
)
def test_invalid_dynamic_obstacle_physics_is_rejected(
    tmp_path, field, value, message
):
    source = ROOT / "isaac_sim/configs/experiments/dynamic.yaml"
    document = yaml.safe_load(source.read_text())
    document["obstacles"][0][field] = value
    target = tmp_path / "dynamic.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        load_dynamic_scenario(target)


def test_zero_length_dynamic_trajectory_is_rejected(tmp_path):
    source = ROOT / "isaac_sim/configs/experiments/dynamic.yaml"
    document = yaml.safe_load(source.read_text())
    document["obstacles"][0]["end"] = document["obstacles"][0]["start"]
    target = tmp_path / "dynamic.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="non-zero length"):
        load_dynamic_scenario(target)


def test_dynamic_baseline_uses_one_shot_trajectories():
    scenario = load_dynamic_scenario(
        ROOT / "isaac_sim/configs/experiments/dynamic.yaml"
    )
    assert scenario.obstacles
    assert all(not obstacle.repeat for obstacle in scenario.obstacles)


def test_dynamic_profile_is_smooth_and_honours_motion_limits():
    distance, max_speed, max_acceleration = 0.80, 0.40, 0.50
    start = DynamicObstacleManager._profile(
        distance, max_speed, max_acceleration, 0.0
    )
    _, _, duration = start
    middle = DynamicObstacleManager._profile(
        distance, max_speed, max_acceleration, duration / 2.0
    )
    finish = DynamicObstacleManager._profile(
        distance, max_speed, max_acceleration, duration
    )

    assert duration == pytest.approx(math.pi, abs=1e-6)
    assert start[:2] == pytest.approx((0.0, 0.0))
    assert middle[:2] == pytest.approx((distance / 2.0, max_speed))
    assert finish[:2] == pytest.approx((distance, 0.0))


def test_three_stage_case_set_selects_the_ordered_route_interactions():
    scenario = load_dynamic_scenario(
        ROOT / "isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml"
    )

    assert scenario.case_sets["full_route_three_stage"] == (
        "local_bypass", "g2_g3_exit", "g5_g1_crossing"
    )
    selected = scenario.selected_cases("full_route_three_stage")
    assert [item.obstacle.obstacle_id for item in selected] == [
        "local_bypass_actor", "g2_g3_exit_actor", "g5_g1_crossing_actor",
    ]
    assert [item.trigger_group for item in selected] == ["G2", "G3", "G1"]
    assert selected[0].waypoints[-1] == pytest.approx((-0.95, -0.20, 0.50))
    assert [item.obstacle.post_motion for item in selected] == [
        "park", "retire", "retire",
    ]
    # Actors retain the established full-height geometry and intersect the
    # 0.333 m LiDAR plane as well as the RGB-D obstacle band.
    for item in selected:
        lower = item.waypoints[0][2] - item.obstacle.size[2] / 2.0
        upper = item.waypoints[0][2] + item.obstacle.size[2] / 2.0
        assert item.obstacle.size[2] == pytest.approx(1.00)
        assert lower <= 0.333 < upper


def test_g2_g3_gate_triggers_southbound_at_y_2_6_in_the_narrow_lane():
    scenario = load_dynamic_scenario(
        ROOT / "isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml"
    )
    case = scenario.cases["g2_g3_exit"]
    manager = object.__new__(DynamicObstacleManager)

    assert case.gate.threshold == pytest.approx(2.60)
    assert manager._gate_passed(
        case, {"x": -0.40, "y": 2.59, "vy": -0.30, "speed": 0.30}
    )
    # Do not trigger before crossing the threshold, outside the calibrated
    # lane, or while travelling in the opposite direction.
    assert not manager._gate_passed(
        case, {"x": -0.40, "y": 2.61, "vy": -0.30, "speed": 0.30}
    )
    assert not manager._gate_passed(
        case, {"x": -0.60, "y": 2.40, "vy": -0.30, "speed": 0.30}
    )
    assert not manager._gate_passed(
        case, {"x": -0.40, "y": 2.40, "vy": 0.30, "speed": 0.30}
    )


def test_local_bypass_gate_arms_early_on_the_calibrated_northbound_lane():
    """Prevent the late gate that froze the actor in the robot swept arc."""
    scenario = load_dynamic_scenario(
        ROOT / "isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml"
    )
    case = scenario.cases["local_bypass"]
    manager = object.__new__(DynamicObstacleManager)

    assert case.gate.threshold == pytest.approx(-2.60)
    assert manager._gate_passed(
        case, {"x": -0.60, "y": -2.59, "vy": 0.30, "speed": 0.30}
    )
    # It remains direction-, speed-, and calibrated-lane-gated; merely
    # entering the broad room cannot re-arm this one-shot interaction.
    assert not manager._gate_passed(
        case, {"x": -0.60, "y": -2.61, "vy": 0.30, "speed": 0.30}
    )
    assert not manager._gate_passed(
        case, {"x": -0.80, "y": -2.50, "vy": 0.30, "speed": 0.30}
    )
    assert not manager._gate_passed(
        case, {"x": -0.60, "y": -2.50, "vy": -0.30, "speed": 0.30}
    )


def test_g5_g1_gate_arms_on_the_northbound_ingress_before_the_doorway():
    """Allow the full smooth crossing time before the G5->G1 conflict."""
    scenario = load_dynamic_scenario(
        ROOT / "isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml"
    )
    case = scenario.cases["g5_g1_crossing"]
    manager = object.__new__(DynamicObstacleManager)

    assert case.gate.threshold == pytest.approx(-2.90)
    assert case.gate.max_distance_to_obstacle_start_m == pytest.approx(2.00)
    assert manager._gate_passed(
        case, {"x": -2.03, "y": -2.89, "vy": 0.30, "speed": 0.30}
    )
    # It must still reject a pre-threshold pose, a pose outside the calibrated
    # ingress lane, and the later southbound doorway exit.
    assert not manager._gate_passed(
        case, {"x": -2.03, "y": -2.91, "vy": 0.30, "speed": 0.30}
    )
    assert not manager._gate_passed(
        case, {"x": -2.11, "y": -2.89, "vy": 0.30, "speed": 0.30}
    )
    assert not manager._gate_passed(
        case, {"x": -1.14, "y": -0.85, "vy": -0.30, "speed": 0.30}
    )


@pytest.mark.parametrize(
    ("case_set", "message"),
    [
        (["local_bypass", "local_bypass"], "invalid or duplicate"),
        (["local_bypass", "missing_case"], "invalid or duplicate"),
        (["crossing", "local_bypass"], "distinct trigger groups"),
    ],
)
def test_three_stage_case_set_rejects_invalid_members_or_goal_collisions(
    tmp_path, case_set, message
):
    source = ROOT / "isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml"
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    document["case_sets"]["bad"] = case_set
    target = tmp_path / "dynamic.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        load_dynamic_scenario(target)


def test_trigger_arms_a_selected_actor_exactly_once_and_keeps_it_hidden():
    manager = object.__new__(DynamicObstacleManager)
    events = []
    enabled = []
    runtime = SimpleNamespace(
        spec=SimpleNamespace(obstacle_id="g2_actor", trigger_group="G2"),
        state="waiting", retired=False, armed_at=None, gate_at=None,
    )
    manager._runtime = {"g2_actor": runtime}
    manager._selected_cases = (SimpleNamespace(obstacle=runtime.spec),)
    manager._case_by_obstacle_id = {"g2_actor": object()}
    manager.scenario = SimpleNamespace(is_case_matrix=True)
    manager._event = lambda kind, stamp, **detail: events.append(
        {"event": kind, "simulation_time": stamp, **detail}
    )
    manager._set_enabled = lambda item, value: enabled.append((item, value))

    assert manager.trigger("G2", 4.0) == ("g2_actor",)
    assert runtime.state == "armed"
    assert runtime.armed_at == 4.0
    assert manager.trigger("G2", 4.1) == ()
    assert enabled == []
    assert [event["event"] for event in events] == ["armed"]


def test_trigger_activation_starts_selected_actor_before_goal_publication():
    manager = object.__new__(DynamicObstacleManager)
    events = []
    enabled = []
    runtime = SimpleNamespace(
        spec=SimpleNamespace(obstacle_id="focus_actor", trigger_group="FOCUS"),
        state="waiting", retired=False, armed_at=None, gate_at=None,
        motion_at=None,
    )
    case = SimpleNamespace(obstacle=runtime.spec, activation="trigger")
    manager._runtime = {"focus_actor": runtime}
    manager._selected_cases = (case,)
    manager._case_by_obstacle_id = {"focus_actor": case}
    manager.scenario = SimpleNamespace(is_case_matrix=True)
    manager._event = lambda kind, stamp, **detail: events.append(kind)
    manager._set_enabled = lambda item, value: enabled.append((item, value))

    assert manager.trigger("FOCUS", 4.0) == ("focus_actor",)
    assert runtime.state == "moving"
    assert runtime.motion_at == 4.0
    assert enabled == [(runtime, True)]
    assert events == ["armed", "motion_start"]


def test_goal_completion_retires_only_armed_or_parked_matching_actor():
    manager = object.__new__(DynamicObstacleManager)
    events = []
    enabled = []
    manager._events = events
    manager._event = lambda kind, stamp, **detail: events.append(
        {"event": kind, "simulation_time": stamp, **detail}
    )
    manager._set_enabled = lambda runtime, value: enabled.append(
        (runtime.spec.obstacle_id, value)
    )
    parked = SimpleNamespace(
        spec=SimpleNamespace(obstacle_id="g2_actor", trigger_group="G3"),
        state="parked", retired=False, velocity_mps=0.0,
    )
    waiting = SimpleNamespace(
        spec=SimpleNamespace(obstacle_id="g5_actor", trigger_group="G1"),
        state="waiting", retired=False, velocity_mps=0.0,
    )
    manager._runtime = {"g2_actor": parked, "g5_actor": waiting}
    manager._selected_cases = ()
    manager._case_by_obstacle_id = {}

    assert manager.complete("G3", 12.5) == ("g2_actor",)
    assert parked.state == "retired"
    assert parked.retired
    assert enabled == [("g2_actor", False)]
    assert events[-1]["event"] == "goal_reached_retire"
    # A repeated completion request is harmless, and a waiting future actor
    # cannot disappear before its own target has been sent.
    assert manager.complete("G3", 13.0) == ()
    assert manager.complete("G1", 13.0) == ()
    assert waiting.state == "waiting"


def test_three_stage_actor_footprints_keep_wall_clearance_in_occupancy_grid():
    """Sample the swept 0.40 m footprint plus the required 0.05 m margin."""
    from PIL import Image

    scenario = load_dynamic_scenario(
        ROOT / "isaac_sim/configs/experiments/kujiale_long_range_dynamic.yaml"
    )
    import yaml

    map_yaml = ROOT / "data/maps/occupancy/warehouse_new.yaml"
    map_data = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image = Image.open(map_yaml.parent / map_data["image"]).convert("L")
    origin_x, origin_y, _ = map_data["origin"]
    resolution = float(map_data["resolution"])
    width, height = image.size
    cases = scenario.selected_cases("full_route_three_stage")
    for case in cases:
        start, end = case.waypoints[0], case.waypoints[-1]
        for sample in range(101):
            ratio = sample / 100.0
            center_x = start[0] + ratio * (end[0] - start[0])
            center_y = start[1] + ratio * (end[1] - start[1])
            # ±0.25 m is the half-size plus the configured 0.05 m wall
            # clearance.  Stay infinitesimally inside that closed boundary:
            # a PGM cell quantizes the wall line itself to a whole 5 cm cell,
            # so treating a point exactly on the 5 cm limit as a collision
            # would incorrectly reject a geometrically valid equality case.
            for x_index in range(11):
                for y_index in range(11):
                    x = center_x - 0.249 + x_index * 0.0498
                    y = center_y - 0.249 + y_index * 0.0498
                    column = int((x - origin_x) / resolution)
                    row = height - 1 - int((y - origin_y) / resolution)
                    assert 0 <= column < width and 0 <= row < height
                    # warehouse_new uses 254 for verified free cells.
                    assert image.getpixel((column, row)) >= 250, (
                        case.case_id, x, y
                    )
