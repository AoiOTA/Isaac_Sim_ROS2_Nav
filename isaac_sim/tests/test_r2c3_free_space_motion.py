from __future__ import annotations

import json
from types import SimpleNamespace

from isaac_sim.apps.navigation_sim import _parser
from isaac_sim.src.diagnostics.r2c1_free_space_probe import (
    R2C1Trace,
    SCHEMA as R2C1_SCHEMA,
    SegmentedFreeSpaceScript,
)
from isaac_sim.src.diagnostics.r2c2_free_space_envelope import (
    SegmentAssessment,
)
from isaac_sim.src.diagnostics.r2c3_free_space_motion import (
    SCHEMA,
    SOURCE_MESH_BOUNDS_TOLERANCE_M,
    evaluate_envelope_preflight,
)


FALLBACKS = ("/Root/door_handle_1", "/Root/door_handle_2")


def _odom_message(stamp_ns: int):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            )
        ),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0
                ),
            )
        ),
        twist=SimpleNamespace(
            twist=SimpleNamespace(
                linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )
        ),
    )


def _assessment(**overrides) -> SegmentAssessment:
    values = {
        "segment_id": "arc_left",
        "support_coverage": 1.0,
        "support_height_variation_m": 0.0,
        "minimum_clearance_m": 0.456,
        "closest_path": "/Root/wall",
        "valid": True,
    }
    values.update(overrides)
    return SegmentAssessment(**values)


def _classified(*, delta: float = 0.0):
    return [
        {
            "path": path,
            "classification": "LATERAL_CANDIDATE",
            "bounds_source": "INVISIBLE_COLLISION_SUBTREE_FALLBACK",
            "fallback_bounds_source_delta_m": delta,
        }
        for path in FALLBACKS
    ] + [
        {
            "path": "/Root/floor",
            "classification": "SUPPORT",
            "bounds_source": "VISIBLE_WORLD_BBOX",
            "fallback_bounds_source_delta_m": None,
        },
        {
            "path": "/Root/ceiling",
            "classification": "OVERHEAD",
            "bounds_source": "VISIBLE_WORLD_BBOX",
            "fallback_bounds_source_delta_m": None,
        },
    ]


def test_r2c3_frozen_six_segment_command_contract():
    assert [
        (item.segment_id, item.linear_x, item.angular_z)
        for item in SegmentedFreeSpaceScript.segments
    ] == [
        ("idle", 0.0, 0.0),
        ("straight", 0.2, 0.0),
        ("spin_left", 0.0, 0.5),
        ("spin_right", 0.0, -0.5),
        ("arc_left", 0.2, 0.5),
        ("arc_right", 0.2, -0.5),
    ]
    assert SegmentedFreeSpaceScript.settle_s == 2.0
    assert SegmentedFreeSpaceScript.action_s == 3.0
    assert SegmentedFreeSpaceScript.idle_s == 1.0


def test_r2c3_preflight_accepts_only_the_frozen_fallback_set():
    result = evaluate_envelope_preflight(
        assessment=_assessment(),
        classified_colliders=_classified(),
        frozen_fallback_paths=FALLBACKS,
    )
    assert result.valid
    assert result.fallback_paths == tuple(sorted(FALLBACKS))
    assert result.support_coverage == 1.0
    assert result.minimum_clearance_m == 0.456

    unknown = _classified()
    unknown[0]["path"] = "/Root/unknown"
    assert not evaluate_envelope_preflight(
        assessment=_assessment(),
        classified_colliders=unknown,
        frozen_fallback_paths=FALLBACKS,
    ).valid


def test_r2c3_source_mesh_bound_threshold_is_fail_closed():
    at_boundary = evaluate_envelope_preflight(
        assessment=_assessment(),
        classified_colliders=_classified(
            delta=SOURCE_MESH_BOUNDS_TOLERANCE_M
        ),
        frozen_fallback_paths=FALLBACKS,
    )
    assert at_boundary.valid
    over = evaluate_envelope_preflight(
        assessment=_assessment(),
        classified_colliders=_classified(
            delta=SOURCE_MESH_BOUNDS_TOLERANCE_M + 1.0e-12
        ),
        frozen_fallback_paths=FALLBACKS,
    )
    assert not over.valid


def test_r2c3_support_clearance_disabled_and_invalid_contracts_fail_closed():
    assert not evaluate_envelope_preflight(
        assessment=_assessment(support_coverage=60.0 / 61.0, valid=False),
        classified_colliders=_classified(),
        frozen_fallback_paths=FALLBACKS,
    ).valid
    assert not evaluate_envelope_preflight(
        assessment=_assessment(
            support_height_variation_m=0.0300001,
            valid=False,
        ),
        classified_colliders=_classified(),
        frozen_fallback_paths=FALLBACKS,
    ).valid
    assert not evaluate_envelope_preflight(
        assessment=_assessment(minimum_clearance_m=0.199999, valid=False),
        classified_colliders=_classified(),
        frozen_fallback_paths=FALLBACKS,
    ).valid
    invalid = _classified()
    invalid.append({
        "path": "/Root/bad",
        "classification": "INVALID",
        "bounds_source": "UNRESOLVED",
        "fallback_bounds_source_delta_m": None,
    })
    assert not evaluate_envelope_preflight(
        assessment=_assessment(),
        classified_colliders=invalid,
        frozen_fallback_paths=FALLBACKS,
    ).valid


def test_r2c3_trace_schema_is_explicit_and_r2c1_history_stays_unchanged(
    tmp_path,
):
    r2c3_path = tmp_path / "r2c3.jsonl"
    trace = R2C1Trace(r2c3_path, manifest={"probe": "r2c3"}, schema=SCHEMA)
    trace.write({"kind": "segment_envelope_preflight", "valid": True})
    trace.close()
    r2c3_rows = [
        json.loads(line)
        for line in r2c3_path.read_text(encoding="utf-8").splitlines()
    ]
    assert {row["schema"] for row in r2c3_rows} == {SCHEMA}

    r2c1_path = tmp_path / "r2c1.jsonl"
    old_trace = R2C1Trace(r2c1_path, manifest={"probe": "r2c1"})
    old_trace.close()
    assert (
        json.loads(r2c1_path.read_text(encoding="utf-8"))["schema"]
        == R2C1_SCHEMA
    )


def test_r2c3_registers_callback_context_before_graph_publication(tmp_path):
    path = tmp_path / "race.jsonl"
    trace = R2C1Trace(path, manifest={}, schema=SCHEMA)
    context = {
        "simulation_time_s": 1.25,
        "loop_sequence": 12,
        "reset_epoch": 4,
        "segment_index": 2,
        "segment_id": "spin_left",
        "segment_phase": "action",
    }
    trace.register_trigger_context(**context)
    # The dedicated executor may deliver here, before trigger() returns.
    trace.record_odom(
        _odom_message(1_250_000_000),
        arrival_loop_sequence=12,
    )
    payload = {
        "position": [0.0, 0.0, 0.0],
        "yaw_rad": 0.0,
        "linear_xyz": [0.0, 0.0, 0.0],
        "angular_xyz": [0.0, 0.0, 0.0],
    }
    trace.record_trigger(
        {
            "loop_publish_count": 1,
            "trigger_status": True,
            "evaluate_status": True,
            "source_payload": payload,
            "publisher_payload": payload,
        },
        post_assist_payload=payload,
        **context,
    )
    trace.close()
    rows = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
    ]
    callback = next(row for row in rows if row["kind"] == "odom_receive")
    assert callback["loop_sequence"] == 12
    assert callback["reset_epoch"] == 4
    assert callback["callback_latency_ticks"] == 0


def test_r2c3_cli_requires_explicit_trace_and_bounds_paths():
    args = _parser().parse_args([
        "--r2c3-free-space-motion-trace",
        "/tmp/r2c3.jsonl",
        "--r2c3-collision-bounds-config",
        "/tmp/r2c3.yaml",
    ])
    assert str(args.r2c3_free_space_motion_trace).endswith("r2c3.jsonl")
    assert str(args.r2c3_collision_bounds_config).endswith("r2c3.yaml")
