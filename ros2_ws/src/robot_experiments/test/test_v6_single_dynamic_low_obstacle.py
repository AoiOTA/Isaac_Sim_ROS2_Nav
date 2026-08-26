import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from robot_experiments import v6_low_obstacle_causal as causal
from robot_experiments.v6_low_obstacle_causal import RecordedMessage
from robot_experiments.v6_single_dynamic_low_obstacle import (
    DYNAMIC_STATE_TOPIC,
    _actor_resolver,
    actor_timeline,
    build_plan,
    evaluate_evidence,
    load_experiment,
    old_position_clearance,
)


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
CONFIG = PACKAGE / "config/v6_single_dynamic_low_obstacle.yaml"


def _state(stamp_ns, state, x, events=()):
    payload = {
        "schema_version": 4,
        "obstacles": [{
            "id": "v6_dynamic_low_box_solo",
            "state": state,
            "position": [x, -0.35, 0.08],
            "position_frame": "map",
            "velocity_mps": 0.25 if state == "moving" else 0.0,
            "size": [0.30, 0.30, 0.16],
        }],
        "events": list(events),
    }
    return RecordedMessage(
        DYNAMIC_STATE_TOPIC, stamp_ns,
        SimpleNamespace(data=json.dumps(payload)),
    )


def _grid(value):
    width, height, resolution = 30, 20, 0.10
    data = [0] * (width * height)
    for row in range(5, 9):
        for column in range(6, 10):
            data[row * width + column] = value
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width, height=height, resolution=resolution,
            origin=SimpleNamespace(position=SimpleNamespace(x=-2.0, y=-1.0)),
        ),
        data=data,
    )


def _typed_source(stamp_ns, sequence, obstacles):
    return RecordedMessage(
        "/bio_nav/module2/cognitive_obstacles", stamp_ns,
        {
            "header": {"stamp": stamp_ns, "frame_id": "map"},
            "sequence": sequence,
            "input_healthy": True,
            "module2_healthy": True,
            "observation_valid": True,
            "trusted_write": True,
            "obstacles": obstacles,
        },
    )


def _layer_status(scope, stamp_ns, sequence, *, active_cells=0, reason=""):
    return RecordedMessage(
        "/bio_nav/cognitive_obstacle_layer/status", stamp_ns,
        {
            "consumer": f"/{scope}_costmap:cognitive",
            "source_sequence": sequence,
            "active_cell_count": active_cells,
            "raised_cell_count": 0,
            "maximum_cost": 0,
            "maximum_cost_increase": 0,
            "fallback_reason": reason,
        },
    )


def _outside_grid():
    return SimpleNamespace(
        info=SimpleNamespace(
            width=4, height=4, resolution=0.10,
            origin=SimpleNamespace(position=SimpleNamespace(x=10.0, y=10.0)),
        ),
        data=[0] * 16,
    )


def _sample(center_x, candidate_x):
    rectangle = [center_x - 0.15, -0.50, center_x + 0.15, -0.20]
    error = abs(candidate_x - center_x)
    return {
        "stamp_ns": int((center_x + 2.0) * 1.0e9),
        "frame_id": "map",
        "physical_obstacle": {
            "id": "v6_dynamic_low_box_solo",
            "center": [center_x, -0.35],
            "size": [0.30, 0.30, 0.16],
            "z_bounds": [0.0, 0.16],
            "robot_radius_m": 0.33,
        },
        "scan_valid": True,
        "scan_point_count": 50,
        "scan_hits_in_obstacle_footprints": 0,
        "depth_observation_valid": True,
        "depth_observation_reason": "observed",
        "depth_point_count": 20,
        "depth_hits_in_obstacle_bounds": 4,
        "rgbd_obstacle_footprints": [{
            "id": "v6_dynamic_low_box_solo",
            "center": [center_x, -0.35, 0.08],
            "size": [0.30, 0.30, 0.16],
            "rectangle": rectangle,
            "source": "physical_low_box_aabb_depth_hits",
            "point_count": 4,
            "hit_count": 4,
        }],
        "typed_obstacles": [{
            "accepted": True, "x": candidate_x, "y": -0.35,
            "radius_m": 0.18, "observed_spatial_error_m": error,
        }],
    }


def _evidence(arm_label="M3"):
    active = arm_label != "M1"
    critic = arm_label == "M3"
    return {
        "requested_arm_label": arm_label,
        "producer_stop_invoked": False,
        "dynamic_actor": {
            "states": ["armed", "moving", "parked"],
            "events": ["armed", "motion_start", "motion_complete", "park"],
            "position_span_m": 0.80,
        },
        "synchronized_samples": [
            _sample(-1.25, -1.24), _sample(-0.45, -0.46),
        ],
        "layer": {
            "global": {"applied_count": 3 if active else 0,
                       "cells": 9 if active else 0,
                       "active_cells": 9 if active else 0,
                       "max_cost_increase": 100 if active else 0},
            "local": {"applied_count": 3 if active else 0,
                      "cells": 7 if active else 0,
                      "active_cells": 7 if active else 0,
                      "max_cost_increase": 100 if active else 0},
        },
        "critic": {
            "applied": critic, "applied_count": 3 if critic else 0,
            "cost_delta_nonzero_count": 3 if critic else 0,
        },
        "action": {"state": "SUCCEEDED", "terminal_zero_confirmed": True,
                   "stop_reason": ""},
        "route": {"goal_results": [{"code": 0}]},
        "passive": {"success": True, "collision": False},
        "cmd_vel": [
            {"stamp_ns": 0, "linear_x": 0.2, "angular_z": 0.0},
            {"stamp_ns": 100_000_000, "linear_x": 0.2, "angular_z": 0.1},
        ],
        "old_position_clearance": {
            "source": {
                "present": True,
                "geometry_cleared": True,
                "actor_intersects_old_aabb": False,
                "candidate_intersects_old_aabb": False,
            },
            "consumers": {
                "global": {
                    "status_present": True,
                    "status_current_or_newer": True,
                    "private_layer_cleared": True,
                },
                "local": {
                    "status_present": True,
                    "status_current_or_newer": True,
                    "private_layer_cleared": True,
                },
            },
        },
    }


def test_plan_uses_one_dynamic_actor_m3_primary_and_explicit_m2_fallback(tmp_path):
    experiment = load_experiment(CONFIG)
    m3 = build_plan(experiment, "M3", tmp_path)
    fallback = build_plan(experiment, "M2-fallback", tmp_path)

    assert m3["module3_mode"] == "M3"
    assert m3["fallback_only"] is False
    assert m3["recommended_order"] == ["M1", "M3"]
    assert m3["fallback_arm"] == "M2-fallback"
    assert "--dynamic-obstacles" in m3["commands"]["scene"]
    assert str(experiment.identity["obstacle_config"]) in m3["commands"]["scene"]
    assert m3["commands"]["stack"][1] == "M3"
    assert "producer_stop" not in m3["commands"]
    assert fallback["module3_mode"] == "M2"
    assert fallback["fallback_only"] is True
    assert fallback["commands"]["stack"][1] == "M2"


def test_actor_timeline_tracks_armed_moving_parked_and_events():
    rows = actor_timeline([
        _state(1, "armed", -1.25, ({"event": "armed", "obstacle_id": "v6_dynamic_low_box_solo"},)),
        _state(2, "moving", -0.95, ({"event": "motion_start", "obstacle_id": "v6_dynamic_low_box_solo"},)),
        _state(3, "parked", -0.45, ({"event": "park", "obstacle_id": "v6_dynamic_low_box_solo"},)),
    ], "v6_dynamic_low_box_solo")

    assert [row["state"] for row in rows] == ["armed", "moving", "parked"]
    assert rows[-1]["position"] == pytest.approx([-0.45, -0.35, 0.08])
    assert rows[1]["events"][0]["event"] == "motion_start"


def test_actor_timeline_uses_ordered_sim_clock_instead_of_wall_receive_time():
    wall_ns = 1_800_000_000_000_000_000
    sim_ns = 12_000_000_000
    clock = RecordedMessage(
        "/clock", wall_ns,
        SimpleNamespace(clock=SimpleNamespace(sec=12, nanosec=0)),
    )
    state = _state(wall_ns + 50_000_000, "moving", -0.95)
    records = list(causal._latest_clock_stamped_records(
        [clock, state], {DYNAMIC_STATE_TOPIC}
    ))
    rows = actor_timeline(records, "v6_dynamic_low_box_solo")
    typed = RecordedMessage(
        "/bio_nav/module2/cognitive_obstacles", wall_ns + 70_000_000,
        SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=50_000_000)
        )),
    )

    assert rows[0]["stamp_ns"] == sim_ns
    assert causal._message_stamp_ns(typed) == sim_ns + 50_000_000
    assert _actor_resolver(rows, {
        "id": "v6_dynamic_low_box_solo", "robot_radius_m": 0.33,
    })(causal._message_stamp_ns(typed)) is not None


def test_background_master_cost_does_not_create_cognitive_old_cell_ghost():
    timeline = [
        {"stamp_ns": 0, "state": "moving", "position": [-1.25, -0.35, 0.08],
         "size": [0.30, 0.30, 0.16]},
        {"stamp_ns": 1_000_000_000, "state": "moving", "position": [-0.90, -0.35, 0.08],
         "size": [0.30, 0.30, 0.16]},
    ]
    records = []
    for topic in ("/global_costmap/costmap", "/local_costmap/costmap"):
        records.extend([
            RecordedMessage(topic, 500_000_000, _grid(99)),
            RecordedMessage(topic, 1_600_000_000, _grid(99)),
            RecordedMessage(topic, 2_600_000_000, _grid(99)),
        ])
    records.extend([
        _typed_source(1_200_000_000, 7, [{
            "id": "v6_dynamic_low_box_solo", "pose_xy_m": [-0.45, -0.35],
            "radius_m": 0.18, "confidence": 0.9,
        }]),
        _typed_source(2_000_000_000, 8, []),
        _layer_status("global", 2_100_000_000, 8),
        _layer_status("local", 2_100_000_000, 8),
    ])
    result = old_position_clearance(
        records, timeline,
        {"start": [-1.25, -0.35, 0.08], "size": [0.30, 0.30, 0.16]},
        10,
    )

    assert result["vacated_stamp_ns"] == 1_000_000_000
    assert result["source"]["candidate_count"] == 0
    assert result["source"]["geometry_cleared"] is True
    assert result["consumers"]["global"]["combined_master_old_aabb_max_after"] == 99
    assert result["consumers"]["local"]["combined_master_old_aabb_max_after"] == 99
    assert result["consumers"]["global"]["private_layer_cleared"] is True
    evidence = _evidence("M3")
    evidence["old_position_clearance"] = result
    assert evaluate_evidence(load_experiment(CONFIG), "M3", evidence)["verdict"] == "PASS"


def test_local_old_aabb_outside_window_passes_with_current_no_cells_status():
    timeline = [
        {"stamp_ns": 1_000_000_000, "state": "moving", "position": [-0.90, -0.35, 0.08],
         "size": [0.30, 0.30, 0.16]},
    ]
    records = [
        RecordedMessage("/global_costmap/costmap", 1_500_000_000, _grid(99)),
        RecordedMessage("/local_costmap/costmap", 1_500_000_000, _outside_grid()),
        _typed_source(2_000_000_000, 8, []),
        _layer_status("global", 2_100_000_000, 8),
        _layer_status(
            "local", 2_100_000_000, 8, reason="rejection_reason=no_costmap_cells"
        ),
    ]
    result = old_position_clearance(
        records, timeline,
        {"start": [-1.25, -0.35, 0.08], "size": [0.30, 0.30, 0.16]},
        10,
    )

    assert result["consumers"]["local"]["post_vacated_sample_count"] == 0
    assert result["consumers"]["local"]["private_layer_cleared"] is True
    evidence = _evidence("M3")
    evidence["old_position_clearance"] = result
    assert evaluate_evidence(load_experiment(CONFIG), "M3", evidence)["verdict"] == "PASS"


@pytest.mark.parametrize("arm_label", ["M1", "M3", "M2-fallback"])
def test_focused_fake_evidence_passes_each_supported_arm(arm_label):
    experiment = load_experiment(CONFIG)
    result = evaluate_evidence(experiment, arm_label, _evidence(arm_label))
    assert result["verdict"] == "PASS", result["reasons"]


@pytest.mark.parametrize(
    ("case", "reason"),
    [
        ("source_intersects", "old_source_candidate_geometry_intersects_old_aabb"),
        ("missing", "local_old_position_consumer_status_missing"),
        ("stale", "local_old_position_consumer_status_stale_or_older"),
        ("active", "local_old_position_private_cells_remain"),
    ],
)
def test_active_evidence_rejects_unattributed_or_uncleared_old_cells(case, reason):
    experiment = load_experiment(CONFIG)
    evidence = _evidence("M3")
    clearance = evidence["old_position_clearance"]
    if case == "source_intersects":
        clearance["source"]["geometry_cleared"] = False
        clearance["source"]["candidate_intersects_old_aabb"] = True
    elif case == "missing":
        clearance["consumers"]["local"]["status_present"] = False
    elif case == "stale":
        clearance["consumers"]["local"]["status_current_or_newer"] = False
    else:
        clearance["consumers"]["local"]["private_layer_cleared"] = False
    result = evaluate_evidence(experiment, "M3", evidence)
    assert result["verdict"] == "FAIL"
    assert reason in result["reasons"]
