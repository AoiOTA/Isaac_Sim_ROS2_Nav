"""Tests for strict, offline contact-profile A/B aggregation."""

import hashlib
import json
import math
from pathlib import Path

import pytest

from robot_experiments.configuration import ConfigurationError
from robot_experiments.contact_ab_analysis import (
    COMPLETE_MATRIX_ENVIRONMENTS,
    COMPLETE_MATRIX_ENVIRONMENT_TOPOLOGIES,
    COMPLETE_MATRIX_PROFILES,
    analyse_contact_ab,
    main,
)


RADIUS_M = 0.098
MOTION_PROFILE_ID = "jackal_skid_steer_ab_v1"
WHEELS = {
    "front_left": "front_left_wheel_joint",
    "front_right": "front_right_wheel_joint",
    "rear_left": "rear_left_wheel_joint",
    "rear_right": "rear_right_wheel_joint",
}
SEGMENTS = (
    ("rotate_left_360", "rotate_left", 0.0, 0.4, 2.0 * math.pi / 0.4),
    ("rotate_right_360", "rotate_right", 0.0, -0.4, 2.0 * math.pi / 0.4),
    ("forward_3m", "forward", 0.5, 0.0, 3.0 / 0.5),
    ("backward_2m", "backward", -0.3, 0.0, 2.0 / 0.3),
    ("arc_left_5s", "arc_left", 0.4, 0.4, 5.0),
    ("arc_right_5s", "arc_right", 0.4, -0.4, 5.0),
)


def _material(path: str) -> dict[str, object]:
    return {
        "material_path": path,
        "static_friction": 0.2,
        "dynamic_friction": 0.2,
        "restitution": 0.0,
        "friction_combine_mode": "average",
        "restitution_combine_mode": "average",
        "friction_combine_mode_authored": True,
        "restitution_combine_mode_authored": True,
    }


def _runtime_provenance(
    *,
    environment: str = "Warehouse",
    contact_profile: str = "legacy_baseline",
    dirty: bool = False,
) -> dict[str, object]:
    wheel_colliders = [
        f"/World/Robot/wheel_{index}/collider" for index in range(4)
    ]
    ground_colliders = ["/Root/GroundPlane/CollisionPlane"]
    if environment == "Warehouse":
        ground_colliders.extend(
            f"/Root/Warehouse/FloorDecal_{index:02d}" for index in range(31)
        )
    wheel_material_path = "/World/Looks/WheelPhysics"
    mode = (
        "explicit_material"
        if contact_profile == "explicit_material"
        else "threshold_only"
        if contact_profile.startswith("threshold_")
        else "legacy_baseline"
    )
    explicit = mode == "explicit_material"
    threshold = mode != "legacy_baseline"
    wheel_material = _material(wheel_material_path)
    if not explicit:
        wheel_material.update(
            friction_combine_mode=None,
            restitution_combine_mode=None,
            friction_combine_mode_authored=False,
            restitution_combine_mode_authored=False,
        )
    ground_material = _material("/World/Looks/GroundPhysics") if explicit else None
    wheel_bindings = [
        {
            "collider_path": path,
            "direct_physics_material_path": wheel_material_path,
            "effective_physics_material_path": wheel_material_path,
        }
        for path in wheel_colliders
    ]
    ground_bindings = [
        {
            "collider_path": path,
            "direct_physics_material_path": (
                "/World/Looks/GroundPhysics" if explicit else None
            ),
            "effective_physics_material_path": (
                "/World/Looks/GroundPhysics" if explicit else None
            ),
        }
        for path in ground_colliders
    ]
    return {
        "verified": True,
        "schema_version": 3,
        "robot": {
            "config": {"path": "/repo/jackal.yaml", "sha256": "a" * 64},
            "asset": {"path": "/repo/jackal_nav.usda", "sha256": "b" * 64},
            "solver": {
                "position_iterations": 32,
                "velocity_iterations": 4,
                "stage_articulation_usd_readback_verified": True,
            },
        },
        "environment": {
            "id": environment,
            "project_stage": {
                "path": f"/repo/{environment}_scene.usda",
                "sha256": ("c" if environment == "Warehouse" else "8") * 64,
            },
            "source_asset": {
                "path": f"/assets/{environment}.usd",
                "sha256": ("d" if environment == "Warehouse" else "9") * 64,
            },
            "asset_root": "/assets/Isaac/6.0",
            "asset_version": "6.0",
            "composed_root_layer_sha256": (
                "e" if environment == "Warehouse" else "7"
            ) * 64,
        },
        "simulation": {
            "navigation_mode": "mapping",
            "odometry_mode": "ideal",
            "physics_hz": 60.0,
        },
        "contact": {
            "profile_path": f"/repo/configs/physics/{contact_profile}.yaml",
            "profile_sha256": hashlib.sha256(contact_profile.encode()).hexdigest(),
            "profile_id": contact_profile,
            "profile_mode": mode,
            "overlay_identifier": f"anon:0x123:contact_{contact_profile}.usda",
            # The anonymous overlay contains environment collider opinions, so
            # identical profiles legitimately produce different layer bytes.
            "overlay_sha256": (
                "2" if environment == "Warehouse" else "1"
            ) * 64,
            "explicit_materials": explicit,
            "thresholds_authored": threshold,
            "scene": {
                "physics_scene_path": "/PhysicsScene",
                "friction_correlation_distance": (
                    0.025 if "corr_0p025" in contact_profile else 0.00025
                ),
                "friction_offset_threshold": (
                    0.04 if "offset_0p04" in contact_profile else 0.0004
                ),
                "friction_type": "patch" if environment == "SimplePlane" else None,
            },
            "collider_contract": {
                "wheel_joint_names": list(WHEELS.values()),
                "wheel_expected_count": 4,
                "ground_required_prim_paths": ground_colliders,
                "ground_semantic_classes": (
                    ["floor_decal"] if environment == "Warehouse" else []
                ),
                "ground_expected_enabled_count": len(ground_colliders),
            },
            "wheel_colliders": wheel_colliders,
            "ground_colliders": ground_colliders,
            "wheel_bindings": wheel_bindings,
            "ground_bindings": ground_bindings,
            "wheel_material": wheel_material,
            "ground_material": ground_material,
            "stage_usd_readback_verified": True,
        },
        "git": {
            "commit": "f" * 40,
            "branch": "codex/navigation-quality-fidelity",
            "dirty": dirty,
        },
    }


def _upgrade_runtime_provenance_to_v4(report):
    provenance = report["runtime_provenance"]
    provenance["schema_version"] = 4
    provenance["robot"]["kinematics"] = {
        "profile_id": "jackal_legacy_geometric_v1",
        "lifecycle": "stable_baseline",
        "wheel_radius_m": RADIUS_M,
        "wheel_width_m": 0.040,
        "geometric_track_width_m": 0.37559,
        "effective_track_width_m": 0.37559,
        "controller_contract_verified": True,
    }
    return report


def _collider_paths_sha256(paths):
    canonical = json.dumps(
        sorted(paths),
        separators=(",", ":"),
        allow_nan=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _upgrade_runtime_provenance_to_v5(report, topology=None):
    _upgrade_runtime_provenance_to_v4(report)
    provenance = report["runtime_provenance"]
    provenance["schema_version"] = 5
    environment = provenance["environment"]["id"]
    if topology is None:
        topology = (
            "simple_plane_only1_v1"
            if environment == "SimplePlane"
            else "warehouse_combined32_v1"
        )
    expected_environment = {
        "simple_plane_only1_v1": "SimplePlane",
        "warehouse_combined32_v1": "Warehouse",
        "warehouse_plane_only1_v1": "Warehouse",
    }[topology]
    assert environment == expected_environment

    contact = provenance["contact"]
    source_colliders = sorted(contact["ground_colliders"])
    plane = "/Root/GroundPlane/CollisionPlane"
    if topology == "warehouse_plane_only1_v1":
        target_colliders = [plane]
        operation = "disable_non_target_colliders"
    else:
        target_colliders = list(source_colliders)
        operation = "preserve_source_colliders"
    disabled_colliders = sorted(
        set(source_colliders) - set(target_colliders)
    )
    contact["ground_colliders"] = target_colliders
    contact["ground_bindings"] = [
        binding
        for binding in contact["ground_bindings"]
        if binding["collider_path"] in set(target_colliders)
    ]
    contact["collider_contract"].update(
        {
            "ground_required_prim_paths": [plane],
            "ground_semantic_classes": (
                ["floor_decal"]
                if topology == "warehouse_combined32_v1"
                else []
            ),
            "ground_expected_enabled_count": len(target_colliders),
        }
    )
    contact["overlay_sha256"] = hashlib.sha256(
        f"{environment}:{topology}:{contact['profile_id']}".encode("utf-8")
    ).hexdigest()
    source_asset = provenance["environment"]["source_asset"]
    provenance["ground_topology"] = {
        "profile_path": f"/repo/configs/ground_topologies/{topology}.yaml",
        "profile_sha256": hashlib.sha256(topology.encode("utf-8")).hexdigest(),
        "profile_id": topology,
        "environment_id": environment,
        "operation": operation,
        "source_asset_path": source_asset["path"],
        "source_asset_sha256": source_asset["sha256"],
        "overlay_identifier": f"anon:0x456:ground_topology_{topology}.usda",
        "overlay_sha256": hashlib.sha256(
            f"overlay:{topology}".encode("utf-8")
        ).hexdigest(),
        "source_colliders": source_colliders,
        "source_collider_count": len(source_colliders),
        "source_collider_paths_sha256": _collider_paths_sha256(
            source_colliders
        ),
        "target_colliders": target_colliders,
        "target_collider_count": len(target_colliders),
        "target_collider_paths_sha256": _collider_paths_sha256(
            target_colliders
        ),
        "disabled_colliders": disabled_colliders,
        "disabled_collider_count": len(disabled_colliders),
        "disabled_collider_paths_sha256": _collider_paths_sha256(
            disabled_colliders
        ),
        "stage_usd_readback_verified": True,
    }
    return report


def _segment(
    specification: tuple[str, str, float, float, float],
    *,
    scale: float,
) -> dict[str, object]:
    segment_id, motion, linear, angular, duration = specification
    observed_duration_ns = round(duration * 1_000_000_000)
    observed_duration = observed_duration_ns / 1_000_000_000
    yaw_change = angular * observed_duration * scale
    longitudinal = linear * observed_duration * scale
    lateral = 0.01 * scale * (-1.0 if "right" in motion else 1.0)
    if motion == "rotate_left":
        left_rate, right_rate = -4.0 * scale, 4.0 * scale
    elif motion == "rotate_right":
        left_rate, right_rate = 4.0 * scale, -4.0 * scale
    else:
        left_rate = right_rate = linear / RADIUS_M * scale
    rates = {
        WHEELS["front_left"]: left_rate,
        WHEELS["rear_left"]: left_rate,
        WHEELS["front_right"]: right_rate,
        WHEELS["rear_right"]: right_rate,
    }
    return {
        "segment_id": segment_id,
        "motion": motion,
        "tier": "ab",
        "result": "complete",
        "command": {
            "linear_x_mps": linear,
            "angular_z_radps": angular,
            "configured_duration_sec": duration,
            "observed_duration_sec": observed_duration,
            "publish_count": 100,
            "start_stamp_ns": 1_000_000_000,
            "end_stamp_ns": 1_000_000_000 + observed_duration_ns,
        },
        "pose": {
            "start": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            "end": {"x_m": longitudinal, "y_m": lateral, "yaw_rad": yaw_change},
            "trajectory_length_m": abs(longitudinal) + abs(lateral),
            "net_displacement_m": math.hypot(longitudinal, lateral),
            "longitudinal_displacement_m": longitudinal,
            "expected_longitudinal_displacement_m": linear * observed_duration,
            "longitudinal_error_m": longitudinal - linear * observed_duration,
            "lateral_displacement_m": lateral,
            "lateral_drift_m": lateral if motion in {"forward", "backward"} else None,
            "translation_drift_m": (
                math.hypot(longitudinal, lateral)
                if motion in {"rotate_left", "rotate_right"}
                else None
            ),
        },
        "yaw": {
            "change_rad": yaw_change,
            "expected_change_rad": angular * observed_duration,
            "error_rad": yaw_change - angular * observed_duration,
        },
        "actual_velocity": {
            "linear_x_mps": {"mean": linear * scale},
            "linear_y_mps": {"mean": lateral / duration},
            "linear_speed_mps": {"mean": abs(linear) * scale},
            "angular_z_radps": {"mean": angular * scale},
        },
        "stopping": {
            "stopped": True,
            "stationary_onset_after_command_sec": 0.05 * scale,
            "confirmed_after_command_sec": 0.55 * scale,
        },
        "wheels": {
            "all_directions_match": True,
            "per_wheel": {
                name: {
                    "direction": "positive" if rate > 0 else "negative",
                    "expected_direction": "positive" if rate > 0 else "negative",
                    "direction_matches": True,
                    "speed_radps": {
                        "mean": rate,
                        "minimum": rate,
                        "maximum": rate,
                    },
                }
                for name, rate in rates.items()
            },
        },
        "sample_counts": {
            "odom_command": 10,
            "odom_total": 20,
            "joint_states_command": 10,
            "joint_states_total": 20,
        },
        "reset": {"success": True},
        "invalid_message_counts": {"odom": 0, "joint_states": 0},
        "timestamp_integrity": {},
    }


def _report(
    *,
    environment: str = "Warehouse",
    contact_profile: str = "legacy_baseline",
    scale: float = 1.0,
    dirty: bool = False,
) -> dict[str, object]:
    configuration_segments = [
        {
            "segment_id": segment_id,
            "motion": motion,
            "tier": "ab",
            "linear_x_mps": linear,
            "angular_z_radps": angular,
            "duration_sec": duration,
        }
        for segment_id, motion, linear, angular, duration in SEGMENTS
    ]
    return {
        "schema_version": 1,
        "diagnostic": "four_wheel_chassis_motion_baseline",
        "profile_id": MOTION_PROFILE_ID,
        "environment_id": environment,
        "odometry_mode": "ideal",
        "config_file": "/repo/motion_skid_steer_ab.yaml",
        "config_sha256": "3" * 64,
        "output_file": f"/reports/{environment}_{contact_profile}_{scale}.json",
        "started_at_utc": f"2026-07-14T00:00:{scale:05.2f}+00:00",
        "completed_at_utc": "2026-07-14T00:01:00+00:00",
        "configuration": {
            "schema_version": 1,
            "profile_id": MOTION_PROFILE_ID,
            "topics": {},
            "reset": {},
            "sampling": {},
            "limits": {},
            "stop": {"wheel_velocity_threshold_radps": 0.2},
            "wheels": WHEELS,
            "segments": configuration_segments,
        },
        "runtime_provenance": _runtime_provenance(
            environment=environment,
            contact_profile=contact_profile,
            dirty=dirty,
        ),
        "segments": [_segment(specification, scale=scale) for specification in SEGMENTS],
        "timestamp_integrity": {},
        "safety": {"safe_zero_burst_attempted": True},
        "result": "success",
        "failure_reason": "",
        "failed_segments": [],
    }


def _write(path: Path, report: dict[str, object]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, sort_keys=True), encoding="utf-8")
    return path


def _three_reports(
    directory: Path,
    *,
    environment: str = "Warehouse",
    contact_profile: str = "legacy_baseline",
) -> list[Path]:
    return [
        _write(
            directory / f"{environment}_{contact_profile}_{index}.json",
            _report(
                environment=environment,
                contact_profile=contact_profile,
                scale=scale,
            ),
        )
        for index, scale in enumerate((0.98, 1.0, 1.02))
    ]


def _three_v5_reports(
    directory: Path,
    *,
    environment: str = "Warehouse",
    topology: str = "warehouse_combined32_v1",
    contact_profile: str = "legacy_baseline",
) -> list[Path]:
    return [
        _write(
            directory / f"{environment}_{topology}_{contact_profile}_{index}.json",
            _upgrade_runtime_provenance_to_v5(
                _report(
                    environment=environment,
                    contact_profile=contact_profile,
                    scale=scale,
                ),
                topology,
            ),
        )
        for index, scale in enumerate((0.98, 1.0, 1.02))
    ]


def test_three_repeats_produce_audited_group_metrics(tmp_path):
    """Three unique reports produce every required group summary."""
    paths = _three_reports(tmp_path)

    report = analyse_contact_ab(paths, RADIUS_M)

    assert report["schema_version"] == 1
    assert report["report_type"] == "contact_ab_analysis"
    assert report["analysis_valid"] is True
    assert report["selection_policy"][
        "required_runtime_provenance_schema"
    ] == 3
    assert report["selection"]["excluded"] == []
    group = report["groups"]["Warehouse::legacy_baseline"]
    assert group["repeat_count"] == 3
    assert len(group["input_reports"]) == 3
    assert all(len(item["sha256"]) == 64 for item in group["input_reports"])
    assert all(
        len(item["canonical_sha256"]) == 64 for item in group["input_reports"]
    )
    assert group["segments"]["rotate_left_360"]["yaw_gain"]["count"] == 3
    assert group["segments"]["rotate_left_360"]["center_drift_m"]["count"] == 3
    assert group["segments"]["forward_3m"]["distance_error_m"]["count"] == 3
    assert group["segments"]["backward_2m"]["lateral_drift_m"]["count"] == 3
    assert group["segments"]["arc_left_5s"]["yaw_gain"]["count"] == 3
    assert group["segments"]["arc_right_5s"]["lateral_displacement_m"]["count"] == 3
    assert group["stop_latency"]["confirmed_sec"]["count"] == 18
    assert group["rotation_symmetry"]["yaw_gain_signed_difference"]["count"] == 3
    assert group["effective_track_m"]["overall"]["count"] == 6
    assert "best_profile" not in json.dumps(report)
    assert report["locked_inputs"]["wheel_radius_m"] == RADIUS_M
    assert report["environment_contracts"]["Warehouse"]["environment"]["id"] == (
        "Warehouse"
    )
    assert report["profile_contracts"]["legacy_baseline"]["profile_id"] == (
        "legacy_baseline"
    )
    assert all(
        len(item["canonical_sha256"]) == 64
        for item in report["selection"]["included"]
    )


def test_dirty_report_is_excluded_and_marks_audit_invalid(tmp_path):
    """A dirty runtime identity is visible and invalidates the audit."""
    paths = _three_reports(tmp_path)
    dirty = _write(tmp_path / "dirty.json", _report(scale=1.03, dirty=True))

    report = analyse_contact_ab([*paths, dirty], RADIUS_M)

    assert report["analysis_valid"] is False
    assert report["counts"]["excluded_reports"] == 1
    assert report["selection"]["excluded"][0]["reasons"][0]["code"] == "git_dirty"


def test_global_lock_and_group_contact_contract_mismatches_are_fatal(tmp_path):
    """Identity drift is fatal instead of becoming another A/B variable."""
    paths = _three_reports(tmp_path)
    different_robot = _report(scale=1.03)
    different_robot["runtime_provenance"]["robot"]["asset"]["sha256"] = "6" * 64
    robot_path = _write(tmp_path / "different_robot.json", different_robot)
    with pytest.raises(ConfigurationError, match="global input lock mismatch"):
        analyse_contact_ab([*paths, robot_path], RADIUS_M)

    different_scene = _report(scale=1.03)
    different_scene["runtime_provenance"]["contact"]["scene"][
        "friction_offset_threshold"
    ] = 0.04
    scene_path = _write(tmp_path / "different_scene.json", different_scene)
    with pytest.raises(ConfigurationError, match="contact contract mismatch"):
        analyse_contact_ab([*paths, scene_path], RADIUS_M)

    different_overlay = _report(scale=1.03)
    different_overlay["runtime_provenance"]["contact"]["overlay_sha256"] = "4" * 64
    overlay_path = _write(tmp_path / "different_overlay.json", different_overlay)
    with pytest.raises(ConfigurationError, match="contact contract mismatch"):
        analyse_contact_ab([*paths, overlay_path], RADIUS_M)


def test_v4_is_auditable_but_cannot_mix_with_historical_v3(tmp_path):
    v4_paths = [
        _write(
            tmp_path / f"v4_{index}.json",
            _upgrade_runtime_provenance_to_v4(_report(scale=scale)),
        )
        for index, scale in enumerate((0.99, 1.0, 1.01), start=1)
    ]
    report = analyse_contact_ab(v4_paths, RADIUS_M)
    assert report["analysis_valid"] is True
    assert report["selection_policy"][
        "required_runtime_provenance_schema"
    ] == 4
    assert report["locked_inputs"]["robot"]["kinematics"][
        "effective_track_width_m"
    ] == 0.37559

    historical_v3 = _write(
        tmp_path / "historical_v3.json", _report(scale=1.02)
    )
    with pytest.raises(
        ConfigurationError,
        match=r"mixed runtime provenance schemas.*\[3, 4\]",
    ):
        analyse_contact_ab([*v4_paths, historical_v3], RADIUS_M)


def test_v4_wheel_radius_is_selected_from_and_locked_to_provenance(tmp_path):
    candidate_radius = 0.1
    paths = []
    for index, scale in enumerate((0.99, 1.0, 1.01), start=1):
        document = _upgrade_runtime_provenance_to_v4(
            _report(scale=scale)
        )
        document["runtime_provenance"]["robot"]["kinematics"][
            "wheel_radius_m"
        ] = candidate_radius
        paths.append(_write(tmp_path / f"candidate_{index}.json", document))

    report = analyse_contact_ab(paths, candidate_radius)

    assert report["analysis_valid"] is True
    assert report["locked_inputs"]["wheel_radius_m"] == candidate_radius
    assert report["locked_inputs"]["robot"]["kinematics"][
        "wheel_radius_m"
    ] == candidate_radius
    assert report["selection_policy"][
        "required_runtime_provenance_schema"
    ] == 4

    with pytest.raises(
        ConfigurationError,
        match="runtime provenance wheel_radius_m does not match the selected "
        "robot",
    ):
        analyse_contact_ab(paths, RADIUS_M)


def test_v5_separates_ground_topology_from_environment_and_contact(tmp_path):
    combined = _three_v5_reports(tmp_path / "combined")
    plane_only = _three_v5_reports(
        tmp_path / "plane_only",
        topology="warehouse_plane_only1_v1",
    )

    report = analyse_contact_ab(
        [*combined, *plane_only],
        RADIUS_M,
        expected_environments=("Warehouse",),
        expected_topologies=(
            "warehouse_combined32_v1",
            "warehouse_plane_only1_v1",
        ),
        expected_profiles=("legacy_baseline",),
    )

    assert report["schema_version"] == 2
    assert report["analysis_valid"] is True
    assert report["selection_policy"][
        "required_runtime_provenance_schema"
    ] == 5
    assert report["counts"]["groups"] == 2
    assert set(report["groups"]) == {
        "Warehouse::warehouse_combined32_v1::legacy_baseline",
        "Warehouse::warehouse_plane_only1_v1::legacy_baseline",
    }
    assert set(report["topology_contracts"]) == {
        "Warehouse::warehouse_combined32_v1",
        "Warehouse::warehouse_plane_only1_v1",
    }
    assert "ground_colliders" not in report["environment_contracts"][
        "Warehouse"
    ]
    plane_group = report["groups"][
        "Warehouse::warehouse_plane_only1_v1::legacy_baseline"
    ]
    assert plane_group["ground_topology_id"] == (
        "warehouse_plane_only1_v1"
    )
    assert plane_group["ground_topology_contract"]["ground_topology"][
        "target_collider_count"
    ] == 1


def test_v5_topology_identity_drift_is_fatal_within_one_topology(tmp_path):
    paths = _three_v5_reports(tmp_path)
    document = json.loads(paths[1].read_text(encoding="utf-8"))
    document["runtime_provenance"]["ground_topology"][
        "overlay_sha256"
    ] = "6" * 64
    _write(paths[1], document)

    with pytest.raises(
        ConfigurationError,
        match="ground topology contract mismatch",
    ):
        analyse_contact_ab(paths, RADIUS_M)


def test_v5_rejects_a_shipped_topology_in_the_wrong_environment(tmp_path):
    legal = _three_v5_reports(tmp_path / "legal")
    forged = _three_v5_reports(
        tmp_path / "forged",
        environment="SimplePlane",
        topology="simple_plane_only1_v1",
    )
    for path in forged:
        document = json.loads(path.read_text(encoding="utf-8"))
        topology = document["runtime_provenance"]["ground_topology"]
        topology["profile_id"] = "warehouse_combined32_v1"
        topology["profile_path"] = (
            "/repo/configs/ground_topologies/"
            "warehouse_combined32_v1.yaml"
        )
        topology["profile_sha256"] = "6" * 64
        _write(path, document)

    report = analyse_contact_ab([*legal, *forged], RADIUS_M)

    assert report["analysis_valid"] is False
    assert report["counts"]["excluded_reports"] == 3
    assert all(
        item["reasons"][0]["code"] == "invalid_runtime_provenance"
        and "shipped ground topology/environment pair is invalid"
        in item["reasons"][0]["detail"]
        for item in report["selection"]["excluded"]
    )


def test_v5_topology_ab_locks_contact_inputs_outside_topology(tmp_path):
    combined = _three_v5_reports(tmp_path / "combined")
    plane_only = _three_v5_reports(
        tmp_path / "plane_only",
        topology="warehouse_plane_only1_v1",
    )
    for path in plane_only:
        document = json.loads(path.read_text(encoding="utf-8"))
        document["runtime_provenance"]["contact"]["scene"][
            "friction_offset_threshold"
        ] = 0.123
        _write(path, document)

    with pytest.raises(
        ConfigurationError,
        match="topology A/B contact invariant mismatch",
    ):
        analyse_contact_ab([*combined, *plane_only], RADIUS_M)


def test_v5_source_collider_discovery_is_locked_across_topologies(tmp_path):
    combined = _three_v5_reports(tmp_path / "combined")
    plane_only = _three_v5_reports(
        tmp_path / "plane_only",
        topology="warehouse_plane_only1_v1",
    )
    for path in plane_only:
        document = json.loads(path.read_text(encoding="utf-8"))
        topology = document["runtime_provenance"]["ground_topology"]
        removed = topology["disabled_colliders"][-1]
        replace_source = [
            collider
            for collider in topology["source_colliders"]
            if collider != removed
        ]
        replace_disabled = [
            collider
            for collider in topology["disabled_colliders"]
            if collider != removed
        ]
        topology["source_colliders"] = replace_source
        topology["source_collider_count"] = len(replace_source)
        topology["source_collider_paths_sha256"] = _collider_paths_sha256(
            replace_source
        )
        topology["disabled_colliders"] = replace_disabled
        topology["disabled_collider_count"] = len(replace_disabled)
        topology["disabled_collider_paths_sha256"] = _collider_paths_sha256(
            replace_disabled
        )
        _write(path, document)

    with pytest.raises(
        ConfigurationError,
        match="environment contract mismatch",
    ):
        analyse_contact_ab([*combined, *plane_only], RADIUS_M)


def test_v5_env_profile_selectors_cross_observed_topologies(tmp_path):
    combined_legacy = _three_v5_reports(tmp_path / "combined_legacy")
    plane_explicit = _three_v5_reports(
        tmp_path / "plane_explicit",
        topology="warehouse_plane_only1_v1",
        contact_profile="explicit_material",
    )

    with pytest.raises(
        ConfigurationError,
        match="required contact A/B groups are missing",
    ):
        analyse_contact_ab(
            [*combined_legacy, *plane_explicit],
            RADIUS_M,
            expected_environments=("Warehouse",),
            expected_profiles=("legacy_baseline", "explicit_material"),
        )


def test_v5_cannot_mix_with_v4_and_topology_selectors_reject_history(
    tmp_path,
):
    v5_paths = _three_v5_reports(tmp_path / "v5")
    v4 = _write(
        tmp_path / "v4.json",
        _upgrade_runtime_provenance_to_v4(_report(scale=1.03)),
    )
    with pytest.raises(
        ConfigurationError,
        match=r"mixed runtime provenance schemas.*\[4, 5\]",
    ):
        analyse_contact_ab([*v5_paths, v4], RADIUS_M)

    with pytest.raises(
        ConfigurationError,
        match="expected_topologies requires runtime provenance schema 5",
    ):
        analyse_contact_ab(
            _three_reports(tmp_path / "historical"),
            RADIUS_M,
            expected_topologies=("warehouse_combined32_v1",),
        )


def test_environment_and_cross_environment_profile_contracts_are_fatal(tmp_path):
    """Environment identity and profile configuration cannot drift between groups."""
    warehouse = _three_reports(tmp_path / "warehouse")

    changed_environment = _report(contact_profile="explicit_material", scale=1.03)
    changed_environment["runtime_provenance"]["environment"]["source_asset"][
        "sha256"
    ] = "6" * 64
    environment_path = _write(
        tmp_path / "changed_environment.json", changed_environment
    )
    with pytest.raises(ConfigurationError, match="environment contract mismatch"):
        analyse_contact_ab([*warehouse, environment_path], RADIUS_M)

    simple = _three_reports(
        tmp_path / "simple",
        environment="SimplePlane",
        contact_profile="legacy_baseline",
    )
    for index, path in enumerate(simple):
        document = json.loads(path.read_text())
        document["runtime_provenance"]["contact"]["profile_sha256"] = "6" * 64
        _write(path, document)
    with pytest.raises(ConfigurationError, match="profile contract mismatch"):
        analyse_contact_ab([*warehouse, *simple], RADIUS_M)


def test_environment_specific_overlay_bytes_are_not_cross_environment_locked(tmp_path):
    """One profile may author different collider overlay bytes per environment."""
    warehouse = _three_reports(tmp_path / "warehouse")
    simple = _three_reports(tmp_path / "simple", environment="SimplePlane")

    report = analyse_contact_ab([*warehouse, *simple], RADIUS_M)

    assert report["analysis_valid"] is True
    assert set(report["environment_contracts"]) == {"Warehouse", "SimplePlane"}
    assert set(report["profile_contracts"]) == {"legacy_baseline"}


def test_historical_composed_root_hash_keeps_profile_group_scope(tmp_path):
    """Do not reinterpret the published v3/v4 offline grouping contract."""
    legacy = _three_reports(tmp_path / "legacy")
    explicit = _three_reports(
        tmp_path / "explicit",
        contact_profile="explicit_material",
    )

    for path in explicit:
        document = json.loads(path.read_text(encoding="utf-8"))
        document["runtime_provenance"]["environment"][
            "composed_root_layer_sha256"
        ] = "4" * 64
        _write(path, document)

    analysis = analyse_contact_ab([*legacy, *explicit], RADIUS_M)

    assert analysis["analysis_valid"] is True
    assert "composed_root_layer_sha256" not in analysis[
        "environment_contracts"
    ]["Warehouse"]["environment"]
    assert analysis["groups"]["Warehouse::explicit_material"][
        "contact_contract"
    ]["composed_root_layer_sha256"] == "4" * 64


@pytest.mark.parametrize("treatment", ["contact", "topology"])
def test_v5_composed_root_hash_is_locked_per_treatment_group(
    tmp_path,
    treatment,
):
    """Runtime-derived RootLayer bytes may vary, but group repeats may not."""
    baseline = _three_v5_reports(tmp_path / "baseline")
    if treatment == "contact":
        changed = _three_v5_reports(
            tmp_path / "changed",
            contact_profile="explicit_material",
        )
    else:
        changed = _three_v5_reports(
            tmp_path / "changed",
            topology="warehouse_plane_only1_v1",
        )

    for path in changed:
        document = json.loads(path.read_text(encoding="utf-8"))
        document["runtime_provenance"]["environment"][
            "composed_root_layer_sha256"
        ] = "4" * 64
        _write(path, document)

    analysis = analyse_contact_ab([*baseline, *changed], RADIUS_M)
    assert analysis["analysis_valid"] is True
    assert "composed_root_layer_sha256" not in analysis[
        "environment_contracts"
    ]["Warehouse"]["environment"]
    assert {
        group["contact_contract"]["composed_root_layer_sha256"]
        for group in analysis["groups"].values()
    } == {"e" * 64, "4" * 64}

    drifted = json.loads(changed[1].read_text(encoding="utf-8"))
    drifted["runtime_provenance"]["environment"][
        "composed_root_layer_sha256"
    ] = "5" * 64
    _write(changed[1], drifted)
    with pytest.raises(ConfigurationError, match="contact contract mismatch"):
        analyse_contact_ab([*baseline, *changed], RADIUS_M)


def test_exact_six_segment_protocol_is_fail_closed(tmp_path):
    """A command mutation is excluded with the affected segment identified."""
    paths = _three_reports(tmp_path)
    wrong = _report(scale=1.03)
    wrong["segments"][0]["command"]["angular_z_radps"] = 0.41
    wrong_path = _write(tmp_path / "wrong_protocol.json", wrong)

    report = analyse_contact_ab([*paths, wrong_path], RADIUS_M)

    assert report["analysis_valid"] is False
    reason = report["selection"]["excluded"][0]["reasons"][0]
    assert reason["code"] == "invalid_motion_protocol"
    assert "rotate_left_360" in reason["detail"]


def test_tick_overshoot_drives_expected_yaw_and_gain(tmp_path):
    """Gain uses the observed command interval, not configured duration."""
    paths = _three_reports(tmp_path)
    expected_gain = 1.1
    for path in paths:
        document = json.loads(path.read_text())
        segment = document["segments"][0]
        observed_ns = round((SEGMENTS[0][4] + 1.0 / 60.0) * 1_000_000_000)
        observed = observed_ns / 1_000_000_000
        expected = SEGMENTS[0][3] * observed
        segment["command"]["observed_duration_sec"] = observed
        segment["command"]["end_stamp_ns"] = (
            segment["command"]["start_stamp_ns"] + observed_ns
        )
        segment["yaw"]["expected_change_rad"] = expected
        segment["yaw"]["change_rad"] = expected * expected_gain
        segment["yaw"]["error_rad"] = segment["yaw"]["change_rad"] - expected
        _write(path, document)

    report = analyse_contact_ab(paths, RADIUS_M)

    mean_gain = report["groups"]["Warehouse::legacy_baseline"]["segments"][
        "rotate_left_360"
    ]["yaw_gain"]["mean"]
    assert mean_gain == pytest.approx(expected_gain)


@pytest.mark.parametrize(
    ("field", "bad_value"),
    (
        ("yaw.error_rad", 123.0),
        ("pose.expected_longitudinal_displacement_m", 123.0),
        ("pose.longitudinal_error_m", 123.0),
    ),
)
def test_derived_motion_fields_must_match_observed_duration(
    tmp_path, field, bad_value
):
    """Producer-derived expected/error fields are independently recomputed."""
    paths = _three_reports(tmp_path)
    wrong = _report(scale=1.03)
    target = wrong["segments"][2]
    parent, child = field.split(".")
    target[parent][child] = bad_value
    wrong_path = _write(tmp_path / f"wrong_{child}.json", wrong)

    report = analyse_contact_ab([*paths, wrong_path], RADIUS_M)

    reason = report["selection"]["excluded"][0]["reasons"][0]
    assert reason["code"] == "invalid_motion_protocol"
    assert child in reason["detail"]


@pytest.mark.parametrize(
    ("mutation", "expected_detail"),
    (
        ("aggregate", "all_directions_match"),
        ("per_wheel_flag", "direction_matches"),
        ("expected_direction", "expected_direction"),
        ("measured_direction", ".direction"),
        ("mean_sign", "speed_radps.mean sign"),
    ),
)
def test_wheel_direction_contract_is_fail_closed(
    tmp_path, mutation, expected_detail
):
    """Every wheel must agree with both aggregate and commanded direction evidence."""
    paths = _three_reports(tmp_path)
    wrong = _report(scale=1.03)
    wheels = wrong["segments"][0]["wheels"]
    wheel = wheels["per_wheel"][WHEELS["front_left"]]
    if mutation == "aggregate":
        wheels["all_directions_match"] = False
    elif mutation == "per_wheel_flag":
        wheel["direction_matches"] = False
    elif mutation == "expected_direction":
        wheel["expected_direction"] = "positive"
    elif mutation == "mean_sign":
        wheel["speed_radps"]["mean"] = 10.0
    else:
        wheel["direction"] = "positive"
    wrong_path = _write(tmp_path / f"wrong_wheel_{mutation}.json", wrong)

    report = analyse_contact_ab([*paths, wrong_path], RADIUS_M)

    reason = report["selection"]["excluded"][0]["reasons"][0]
    assert reason["code"] == "invalid_motion_protocol"
    assert expected_detail in reason["detail"]


def test_rotation_accepts_internally_consistent_mixed_transient(tmp_path):
    """Pure rotation may retain a deadband-crossing transient with a dominant sign."""
    paths = _three_reports(tmp_path)
    report_with_transient = _report(scale=1.03)
    wheels = report_with_transient["segments"][0]["wheels"]
    wheel = wheels["per_wheel"][WHEELS["front_left"]]
    wheel["direction"] = "mixed"
    wheel["direction_matches"] = False
    wheel["speed_radps"].update({"minimum": -4.2, "maximum": 0.31})
    wheels["all_directions_match"] = False
    transient_path = _write(tmp_path / "rotation_mixed_transient.json", report_with_transient)

    analysis = analyse_contact_ab([*paths, transient_path], RADIUS_M)

    assert analysis["counts"]["included_reports"] == 4
    assert analysis["selection"]["excluded"] == []


@pytest.mark.parametrize(
    ("mutation", "expected_detail"),
    (
        ("non_rotation", "only valid for a pure rotation"),
        ("zero_mean", "mean sign"),
        ("missing_negative_range", "cross both sides"),
        ("missing_positive_range", "cross both sides"),
        ("flag_true", "direction_matches is inconsistent"),
        ("aggregate_true", "all_directions_match is inconsistent"),
    ),
)
def test_mixed_wheel_direction_contract_remains_fail_closed(
    tmp_path, mutation, expected_detail
):
    """Mixed evidence is accepted only when every producer invariant is provable."""
    paths = _three_reports(tmp_path)
    wrong = _report(scale=1.03)
    segment_index = 2 if mutation == "non_rotation" else 0
    wheels = wrong["segments"][segment_index]["wheels"]
    wheel = wheels["per_wheel"][WHEELS["front_left"]]
    wheel["direction"] = "mixed"
    wheel["direction_matches"] = False
    wheels["all_directions_match"] = False
    if segment_index == 0:
        wheel["speed_radps"].update({"minimum": -4.2, "maximum": 0.31})
    else:
        wheel["speed_radps"].update({"minimum": -0.31, "maximum": 5.3})
    if mutation == "zero_mean":
        wheel["speed_radps"]["mean"] = 0.0
    elif mutation == "missing_negative_range":
        wheel["speed_radps"]["minimum"] = -0.19
        wheel["speed_radps"]["mean"] = -0.1
    elif mutation == "missing_positive_range":
        wheel["speed_radps"]["maximum"] = 0.19
    elif mutation == "flag_true":
        wheel["direction_matches"] = True
    elif mutation == "aggregate_true":
        wheels["all_directions_match"] = True
    wrong_path = _write(tmp_path / f"wrong_mixed_{mutation}.json", wrong)

    analysis = analyse_contact_ab([*paths, wrong_path], RADIUS_M)

    reason = analysis["selection"]["excluded"][0]["reasons"][0]
    assert reason["code"] == "invalid_motion_protocol"
    assert expected_detail in reason["detail"]


@pytest.mark.parametrize(
    ("mutation", "bad_value", "expected_detail"),
    (
        ("opposite_extreme", 0.31, "reported negative direction classification"),
        ("non_string_direction", [], ".direction must be a string"),
    ),
)
def test_reported_direction_must_match_speed_extrema(
    tmp_path, mutation, bad_value, expected_detail
):
    """Reported direction cannot contradict the producer's extrema classification."""
    paths = _three_reports(tmp_path)
    wrong = _report(scale=1.03)
    wheel = wrong["segments"][0]["wheels"]["per_wheel"][WHEELS["front_left"]]
    if mutation == "opposite_extreme":
        wheel["speed_radps"]["maximum"] = bad_value
    else:
        wheel["direction"] = bad_value
        wheel["direction_matches"] = False
        wrong["segments"][0]["wheels"]["all_directions_match"] = False
    wrong_path = _write(tmp_path / f"wrong_direction_{mutation}.json", wrong)

    analysis = analyse_contact_ab([*paths, wrong_path], RADIUS_M)

    reason = analysis["selection"]["excluded"][0]["reasons"][0]
    assert reason["code"] == "invalid_motion_protocol"
    assert expected_detail in reason["detail"]


def test_zero_direction_deadband_matches_producer_configuration(tmp_path):
    """The producer permits a zero direction deadband, so the audit does too."""
    paths = _three_reports(tmp_path)
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        document["configuration"]["stop"]["wheel_velocity_threshold_radps"] = 0.0
        _write(path, document)

    analysis = analyse_contact_ab(paths, RADIUS_M)

    assert analysis["analysis_valid"] is True
    assert analysis["counts"]["included_reports"] == 3


def test_speed_mean_range_allows_producer_roundoff(tmp_path):
    """A producer-computed mean may exceed an equal endpoint by one float ulp."""
    paths = _three_reports(tmp_path)
    rounded = _report(scale=1.03)
    wheel = rounded["segments"][0]["wheels"]["per_wheel"][WHEELS["front_right"]]
    samples = [0.4, 0.4, 0.4]
    producer_mean = sum(samples) / len(samples)
    assert producer_mean > max(samples)
    wheel["speed_radps"].update(
        {"mean": producer_mean, "minimum": min(samples), "maximum": max(samples)}
    )
    rounded_path = _write(tmp_path / "producer_roundoff.json", rounded)

    analysis = analyse_contact_ab([*paths, rounded_path], RADIUS_M)

    assert analysis["counts"]["included_reports"] == 4
    assert analysis["selection"]["excluded"] == []


def test_no_valid_reports_error_includes_first_exclusion_detail(tmp_path):
    """A failed one-report smoke gives the actionable protocol reason."""
    wrong = _report()
    wrong["segments"][0]["wheels"]["all_directions_match"] = False
    wrong_path = _write(tmp_path / "only_invalid_report.json", wrong)

    with pytest.raises(
        ConfigurationError,
        match=r"first exclusion \[invalid_motion_protocol\].*all_directions_match",
    ):
        analyse_contact_ab([wrong_path], RADIUS_M)


def test_motion_and_contact_wheel_joint_contracts_must_match(tmp_path):
    """Motion metrics and Stage collider evidence must identify the same wheels."""
    paths = _three_reports(tmp_path)
    wrong = _report(scale=1.03)
    wrong["runtime_provenance"]["contact"]["collider_contract"][
        "wheel_joint_names"
    ] = ["unrelated_front_left", "unrelated_front_right", "other_a", "other_b"]
    wrong_path = _write(tmp_path / "wrong_contact_wheels.json", wrong)

    report = analyse_contact_ab([*paths, wrong_path], RADIUS_M)

    reason = report["selection"]["excluded"][0]["reasons"][0]
    assert reason["code"] == "invalid_motion_protocol"
    assert "configured wheels" in reason["detail"]


def test_complete_matrix_requires_twelve_groups_with_three_unique_reports(tmp_path):
    """The shipped full-matrix option expands to twelve complete groups."""
    paths = []
    for environment in COMPLETE_MATRIX_ENVIRONMENTS:
        for profile in COMPLETE_MATRIX_PROFILES:
            group_directory = tmp_path / environment / profile
            group_directory.mkdir(parents=True)
            paths.extend(
                _three_reports(
                    group_directory,
                    environment=environment,
                    contact_profile=profile,
                )
            )

    report = analyse_contact_ab(
        paths,
        RADIUS_M,
        require_complete_matrix=True,
    )

    assert report["analysis_valid"] is True
    assert len(report["groups"]) == 12
    assert report["matrix"]["complete"] is True
    assert report["matrix"]["missing_groups"] == []


def test_v5_complete_matrix_requires_eighteen_legal_topology_groups(tmp_path):
    paths = []
    for environment, topology in COMPLETE_MATRIX_ENVIRONMENT_TOPOLOGIES:
        for profile in COMPLETE_MATRIX_PROFILES:
            paths.extend(
                _three_v5_reports(
                    tmp_path / environment / topology / profile,
                    environment=environment,
                    topology=topology,
                    contact_profile=profile,
                )
            )

    report = analyse_contact_ab(
        paths,
        RADIUS_M,
        require_complete_matrix=True,
    )

    assert report["schema_version"] == 2
    assert report["analysis_valid"] is True
    assert report["counts"] == {
        "input_reports": 54,
        "included_reports": 54,
        "excluded_reports": 0,
        "groups": 18,
    }
    assert report["matrix"]["complete"] is True
    assert len(report["matrix"]["required_groups"]) == 18
    assert report["matrix"]["missing_groups"] == []


def test_cli_exit_two_atomically_writes_exclusion_audit(tmp_path):
    """Nonfatal exclusions are durable but return a non-success status."""
    paths = _three_reports(tmp_path)
    dirty = _write(tmp_path / "dirty.json", _report(scale=1.03, dirty=True))
    output = tmp_path / "audit.json"

    status = main(
        [
            *(str(path) for path in paths),
            str(dirty),
            "--wheel-radius",
            str(RADIUS_M),
            "--output",
            str(output),
        ]
    )

    assert status == 2
    assert json.loads(output.read_text())["analysis_valid"] is False
    assert not list(tmp_path.glob(".audit.json.*.tmp"))


def test_expected_selectors_reject_unselected_groups(tmp_path):
    """Explicit selectors audit rather than silently absorb other groups."""
    paths = _three_reports(tmp_path)
    other = _three_reports(
        tmp_path / "other",
        environment="SimplePlane",
        contact_profile="legacy_baseline",
    )

    report = analyse_contact_ab(
        [*paths, *other],
        RADIUS_M,
        expected_environments=("Warehouse",),
        expected_profiles=("legacy_baseline",),
    )

    assert report["analysis_valid"] is False
    assert set(report["groups"]) == {"Warehouse::legacy_baseline"}
    assert {item["reasons"][0]["code"] for item in report["selection"]["excluded"]} == {
        "unexpected_environment"
    }


def test_duplicate_content_cannot_satisfy_minimum_repeats(tmp_path):
    """Whitespace/key ordering cannot masquerade as an independent repeat."""
    first, second, _ = _three_reports(tmp_path)
    duplicate = tmp_path / "duplicate.json"
    duplicate.write_text(
        json.dumps(json.loads(first.read_text()), indent=4), encoding="utf-8"
    )

    with pytest.raises(ConfigurationError, match="unique report contents"):
        analyse_contact_ab([first, second, duplicate], RADIUS_M)


@pytest.mark.parametrize(
    ("mutation", "expected_detail"),
    (
        ("report_float", "schema_version must be integer 1"),
        ("report_bool", "schema_version must be integer 1"),
        ("config_float", "configuration.schema_version"),
        ("runtime_float", "runtime_provenance.schema_version"),
        ("runtime_bool", "runtime_provenance.schema_version"),
        ("runtime_unknown", "runtime_provenance keys"),
        ("configuration_unknown", "configuration keys"),
    ),
)
def test_schema_and_required_shapes_are_exact(tmp_path, mutation, expected_detail):
    """JSON equality coercions and unmodelled provenance are rejected."""
    paths = _three_reports(tmp_path)
    wrong = _report(scale=1.03)
    if mutation == "report_float":
        wrong["schema_version"] = 1.0
    elif mutation == "report_bool":
        wrong["schema_version"] = True
    elif mutation == "config_float":
        wrong["configuration"]["schema_version"] = 1.0
    elif mutation == "runtime_float":
        wrong["runtime_provenance"]["schema_version"] = 3.0
    elif mutation == "runtime_bool":
        wrong["runtime_provenance"]["schema_version"] = True
    elif mutation == "runtime_unknown":
        wrong["runtime_provenance"]["unlocked_extension"] = "ignored"
    else:
        wrong["configuration"]["unlocked_extension"] = "ignored"
    wrong_path = _write(tmp_path / f"wrong_schema_{mutation}.json", wrong)

    report = analyse_contact_ab([*paths, wrong_path], RADIUS_M)

    reason = report["selection"]["excluded"][0]["reasons"][0]
    assert reason["code"] == "invalid_motion_protocol"
    assert expected_detail in reason["detail"]


def test_extreme_finite_samples_that_overflow_statistics_are_configuration_errors(
    tmp_path,
):
    """Finite inputs may not leak infinity from aggregate arithmetic."""
    paths = _three_reports(tmp_path)
    for index, path in enumerate(paths):
        document = json.loads(path.read_text())
        value = 1e308 if index != 1 else -1e308
        segment = document["segments"][2]
        segment["pose"]["lateral_displacement_m"] = value
        segment["pose"]["lateral_drift_m"] = value
        _write(path, document)

    with pytest.raises(ConfigurationError, match="statistics"):
        analyse_contact_ab(paths, RADIUS_M)


def test_extreme_timestamp_is_excluded_without_leaking_overflow(tmp_path):
    """Unbounded JSON integers cannot escape the analyzer as a traceback."""
    paths = _three_reports(tmp_path)
    wrong = _report(scale=1.03)
    wrong["segments"][0]["command"]["end_stamp_ns"] = 10**1000
    wrong_path = _write(tmp_path / "huge_timestamp.json", wrong)

    report = analyse_contact_ab([*paths, wrong_path], RADIUS_M)

    assert report["analysis_valid"] is False
    reason = report["selection"]["excluded"][0]["reasons"][0]
    assert reason["code"] == "invalid_motion_protocol"
    assert "timestamp interval is too large" in reason["detail"]


def test_arc_metric_is_named_displacement_not_drift(tmp_path):
    """Arc lateral offset is reported without implying a zero-offset target."""
    report = analyse_contact_ab(_three_reports(tmp_path), RADIUS_M)

    group = report["groups"]["Warehouse::legacy_baseline"]
    assert "lateral_displacement_m" in group["segments"]["arc_left_5s"]
    assert "lateral_drift_m" not in group["segments"]["arc_left_5s"]
    assert (
        "absolute_lateral_displacement_difference_m" in group["arc_symmetry"]
    )


@pytest.mark.parametrize("radius", (0.1, 0.097999, True, float("nan")))
def test_historical_v3_wheel_radius_is_the_canonical_jackal_value(
    tmp_path,
    radius,
):
    """Historical v3 evidence cannot silently rescale effective track."""
    with pytest.raises(
        ConfigurationError,
        match="canonical Jackal wheel radius|must be a finite number",
    ):
        analyse_contact_ab(_three_reports(tmp_path), radius)
