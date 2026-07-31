from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "scripts" / "analyze_collision_evidence.py"
SPEC = importlib.util.spec_from_file_location(
    "analyze_collision_evidence", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
ANALYZER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYZER)


def _pose(stamp, x=0.0, y=0.0, yaw=0.0):
    return ANALYZER.Sample(stamp, {
        "x": x,
        "y": y,
        "yaw_rad": yaw,
    })


def _costmap(stamp, lethal=True):
    data = [0] * 100
    if lethal:
        data[5 * 10 + 5] = 254
    return ANALYZER.Sample(stamp, {
        "frame_id": "odom",
        "size_x": 10,
        "size_y": 10,
        "resolution_m": 0.1,
        "origin": {
            "x": -0.5,
            "y": -0.5,
            "yaw_rad": 0.0,
        },
        "data": tuple(data),
    })


def test_footprint_summary_detects_lethal_cell_and_boundary():
    summary = ANALYZER.costmap_footprint_summary(
        _costmap(4.0), _pose(4.1))

    assert summary is not None
    assert summary["lethal_cell_count"] == 1
    assert summary["cost_ge_253_known_count"] == 1
    assert ANALYZER.point_in_polygon(
        0.260,
        0.0,
        ANALYZER.transform_polygon(_pose(0.0).value),
    )


def test_trajectory_uses_only_a_costmap_not_newer_than_the_path():
    trajectory = ANALYZER.Sample(5.0, {
        "frame_id": "odom",
        "poses": [
            {"x": 0.0, "y": 0.0, "yaw_rad": 0.0},
        ],
    })
    summary = ANALYZER.trajectory_cost_summary(
        trajectory,
        [_costmap(4.0, lethal=False), _costmap(6.0, lethal=True)],
    )

    assert summary is not None
    assert summary["costmap_stamp_s"] == 4.0
    assert summary["costmap_is_not_newer_than_trajectory"] is True
    assert summary["footprint_collision_point_count"] == 0


def test_markdown_keeps_missing_old_topics_explicit():
    markdown = ANALYZER.render_markdown({
        "schema": "bio_nav_collision_forensics_v2",
        "collision_topic": {
            "first_true_stamp_s": 43.316666666,
            "true_count": 1,
            "contact_identity": "unknown_bool_topic_only",
        },
        "causal_assessment": {
            "status":
                "supports_static_wall_plus_legacy_scan_nearfield_blind_zone",
            "static_map_overlap": True,
            "minimum_actor_center_distance_m": 2.6,
            "legacy_scan_blind_zone": True,
        },
        "unavailable_in_frozen_evidence": ["/scan_safety"],
    })

    assert "43.316666666" in markdown
    assert "/scan_safety" in markdown
    assert "不得用新运行补写旧证据" in markdown
