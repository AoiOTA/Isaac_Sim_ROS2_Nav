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
    validate_physical_acceptance_accounting,
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
    odometry_mode: str = "ideal",
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
            "odometry_mode": odometry_mode,
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


def _upgrade_motion_report_to_v2(report):
    report["schema_version"] = 2
    for segment in report["segments"]:
        command = segment["command"]
        start_stamp = command["start_stamp_ns"]
        end_stamp = command["end_stamp_ns"]
        steady_start_stamp = start_stamp + (end_stamp - start_stamp) // 2
        mean_yaw_rate = segment["actual_velocity"]["angular_z_radps"]["mean"]
        sample_count = 5
        segment["actual_velocity"]["steady_state_window"] = {
            "schema_version": 1,
            "definition": "final_half_of_command_interval",
            "start_stamp_ns": steady_start_stamp,
            "end_stamp_ns": end_stamp,
            "observed_duration_sec": (
                end_stamp - steady_start_stamp
            ) / 1_000_000_000,
            "sample_count": sample_count,
            "angular_z_radps": {
                "sample_count": sample_count,
                "mean": mean_yaw_rate,
                "mean_abs": abs(mean_yaw_rate),
                "minimum": mean_yaw_rate,
                "maximum": mean_yaw_rate,
                "peak_abs": abs(mean_yaw_rate),
                "rmse": abs(mean_yaw_rate),
            },
        }
    return report


def _constant_speed_distribution(value, sample_count):
    magnitude = abs(value)
    return {
        "sample_count": sample_count,
        "mean": value,
        "mean_abs": magnitude,
        "minimum": value,
        "maximum": value,
        "peak_abs": magnitude,
        "rmse": magnitude,
    }


def _upgrade_motion_report_to_v3(report):
    _upgrade_motion_report_to_v2(report)
    report["schema_version"] = 3
    report["configuration"]["sampling"]["max_sample_age_sec"] = 0.5
    report["configuration"]["sampling"]["zero_publish_count"] = 5
    report["configuration"]["reset"] = {
        "service": "/simulation/reset",
        "service_timeout_sec": 5.0,
        "recovery_timeout_sec": 30.0,
        "settle_duration_sec": 0.5,
    }
    deadband = report["configuration"]["stop"][
        "wheel_velocity_threshold_radps"
    ]
    session_timestamp_counts = {
        "clock": 0,
        "odom": 0,
        "joint_states": 0,
    }
    for segment_index, segment in enumerate(report["segments"], start=1):
        command = segment["command"]
        start_stamp = command["start_stamp_ns"]
        end_stamp = command["end_stamp_ns"]
        steady_start_stamp = start_stamp + (end_stamp - start_stamp) // 2
        steady_duration = (end_stamp - steady_start_stamp) / 1_000_000_000
        steady_sample_count = math.ceil(steady_duration / 0.25) + 1
        maximum_gap = steady_duration / (steady_sample_count - 1)
        command_sample_count = steady_sample_count * 2 - 1
        total_sample_count = command_sample_count + 2
        segment["sample_counts"].update(
            {
                "odom_command": command_sample_count,
                "odom_total": total_sample_count,
                "joint_states_command": command_sample_count,
                "joint_states_total": total_sample_count,
            }
        )
        odom_command_count = command_sample_count
        for velocity_name, distribution in list(
            segment["actual_velocity"].items()
        ):
            if velocity_name == "steady_state_window":
                continue
            segment["actual_velocity"][velocity_name] = (
                _constant_speed_distribution(
                    distribution["mean"], odom_command_count
                )
            )
        yaw_window = segment["actual_velocity"]["steady_state_window"]
        mean_yaw_rate = yaw_window["angular_z_radps"]["mean"]
        yaw_window.update(
            {
                "boundary_semantics": "closed_interval",
                "sample_count": steady_sample_count,
                "first_sample_stamp_ns": steady_start_stamp,
                "last_sample_stamp_ns": end_stamp,
                "maximum_inter_sample_gap_sec": maximum_gap,
                "angular_z_radps": _constant_speed_distribution(
                    mean_yaw_rate, steady_sample_count
                ),
            }
        )
        whole_sample_count = segment["sample_counts"]["joint_states_command"]
        steady_per_wheel = {}
        for joint_name, wheel in segment["wheels"]["per_wheel"].items():
            speed = wheel["speed_radps"]["mean"]
            wheel["speed_radps"] = _constant_speed_distribution(
                speed, whole_sample_count
            )
            direction = wheel["direction"]
            steady_per_wheel[joint_name] = {
                "direction": direction,
                "expected_direction": wheel["expected_direction"],
                "direction_matches": wheel["direction_matches"],
                "direction_sample_counts": {
                    "positive_above_deadband": (
                        steady_sample_count if speed > deadband else 0
                    ),
                    "negative_below_deadband": (
                        steady_sample_count if speed < -deadband else 0
                    ),
                    "within_deadband": (
                        steady_sample_count
                        if -deadband <= speed <= deadband
                        else 0
                    ),
                },
                "speed_radps": _constant_speed_distribution(
                    speed, steady_sample_count
                ),
            }
        segment["wheels"]["steady_state_window"] = {
            "schema_version": 1,
            "definition": "final_half_of_command_interval",
            "boundary_semantics": "closed_interval",
            "start_stamp_ns": steady_start_stamp,
            "end_stamp_ns": end_stamp,
            "observed_duration_sec": steady_duration,
            "sample_count": steady_sample_count,
            "first_sample_stamp_ns": steady_start_stamp,
            "last_sample_stamp_ns": end_stamp,
            "maximum_inter_sample_gap_sec": maximum_gap,
            "classification_deadband_radps": deadband,
            "all_directions_match": all(
                wheel["direction_matches"]
                for wheel in steady_per_wheel.values()
            ),
            "per_wheel": steady_per_wheel,
        }
        stop_start_stamp = end_stamp + 50_000_000
        stop_end_stamp = end_stamp + 550_000_000
        segment["stopping"]["stationary_evidence"] = {
            "schema_version": 1,
            "definition": (
                "dual_stream_continuously_stationary_after_zero_command"
            ),
            "boundary_semantics": "closed_interval",
            "start_stamp_ns": stop_start_stamp,
            "end_stamp_ns": stop_end_stamp,
            "observed_duration_sec": 0.5,
            "max_sample_age_sec": 0.5,
            "streams": {
                source: {
                    "sample_count": 2,
                    "first_sample_stamp_ns": stop_start_stamp,
                    "last_sample_stamp_ns": stop_end_stamp,
                    "maximum_inter_sample_gap_sec": 0.5,
                }
                for source in ("odom", "joint_states")
            },
        }
        segment["invalid_message_counts"] = {
            "odom": 0,
            "joint_states": 0,
        }
        segment["timestamp_integrity"] = {
            source: {
                "sample_count": total_sample_count,
                "first_stamp_ns": start_stamp,
                "last_stamp_ns": stop_end_stamp,
                "regression_count": 0,
                "duplicate_count": 0,
                "monotonic_unique": True,
            }
            for source in ("clock", "odom", "joint_states")
        }
        segment["reset"] = {
            "service": "/simulation/reset",
            "response_message": (
                "simulation reset transaction complete; "
                f"reset_metadata_v1={segment_index}:0"
            ),
            "reset_generation": segment_index,
            "reset_boundary_clock_ns": 0,
            "service_latency_wall_sec": 0.01,
            "recovery_latency_wall_sec": 0.1,
            "clock_before_ns": 1_000_000_000,
            "clock_after_ns": 100_000_000,
            "clock_rollback_observed": True,
            "fresh_clock_received": True,
            "fresh_odom_received": True,
            "fresh_joint_states_received": True,
            "stationary_settle_duration_sec": 0.5,
            "recovery_observation_counts": {
                "coherent_group_not_ready": 0,
                "pre_boundary_group": 0,
                "not_stationary": 0,
                "stationary": 2,
                "coherent_without_time_progress": 0,
                "observation_regression": 0,
                "receive_timestamp_regression": 0,
                "coherent_timestamp_regression": 0,
            },
            "recovery_violation_counts": {
                "streams_fresh": 0,
                "stream:odom_not_stale": 0,
                "stream:odom_not_too_far_ahead": 0,
                "stream:joint_states_not_stale": 0,
                "stream:joint_states_not_too_far_ahead": 0,
                "wall_streams_fresh": 0,
                "odom_linear_speed": 0,
                "odom_angular_speed": 0,
                **{f"wheel:{joint_name}": 0 for joint_name in WHEELS.values()},
            },
            "recovery_peak_observed": {
                "odom_linear_speed_mps": 0.0,
                "odom_angular_speed_radps": 0.0,
                "wheel_abs_speed_radps": {
                    joint_name: 0.0 for joint_name in WHEELS.values()
                },
                "sim_age_sec": {
                    source: {"minimum": 0.0, "maximum": 0.01}
                    for source in ("odom", "joint_states")
                },
            },
            "credited_stamp_high_watermarks_ns": {
                "clock": 100_000_000,
                "odom": 100_000_000,
                "joint_states": 100_000_000,
            },
            "received_stamp_high_watermarks_ns": {
                "clock": 100_000_000,
                "odom": 100_000_000,
                "joint_states": 100_000_000,
            },
            "longest_stationary_duration_sec": 0.5,
        }
        for source in session_timestamp_counts:
            session_timestamp_counts[source] += total_sample_count
    report["timestamp_integrity"] = {
        source: {
            "sample_count": sample_count,
            "first_stamp_ns": 1_000_000_000,
            "last_stamp_ns": report["segments"][-1]["timestamp_integrity"][
                source
            ]["last_stamp_ns"],
            "regression_count": len(report["segments"]) - 1,
            "duplicate_count": 0,
            "monotonic_unique": False,
        }
        for source, sample_count in session_timestamp_counts.items()
    }
    report["safety"] = {
        "exclusive_non_reset_cmd_vel_owner_enforced": True,
        "authorized_reset_safety_publishers": ["/isaac_navigation_sim"],
        "cmd_vel_subscription_count": 1,
        "safe_zero_burst_attempted": True,
        "zero_publish_count": 5,
    }
    return report


def _collider_paths_sha256(paths):
    canonical = json.dumps(
        sorted(paths),
        separators=(",", ":"),
        allow_nan=False,
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _upgrade_runtime_provenance_to_v5(
    report,
    topology=None,
    *,
    report_schema_version=3,
):
    if report_schema_version == 3:
        _upgrade_motion_report_to_v3(report)
    elif report_schema_version == 2:
        _upgrade_motion_report_to_v2(report)
    else:
        assert report_schema_version == 1
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


def _upgrade_runtime_provenance_to_v6(
    report,
    *,
    strategy_id="pose_restore_v1",
    topology=None,
):
    _upgrade_runtime_provenance_to_v5(report, topology)
    provenance = report["runtime_provenance"]
    provenance["schema_version"] = 6
    semantics = {
        "pose_restore_v1": (0.0, 0),
        "separate_recontact_0p20m_1step_v1": (0.2, 1),
    }
    lift_distance_m, separation_step_count = semantics[strategy_id]
    ground_paths = provenance["ground_topology"]["target_colliders"]
    provenance["simulation"]["reset_strategy"] = {
        "schema_version": 1,
        "id": strategy_id,
        "lift_distance_m": lift_distance_m,
        "separation_step_count": separation_step_count,
        "recontact_step_count": 1,
        "contact_probe": {
            "schema_version": 1,
            "enabled": True,
            "wheel_bindings": [
                {
                    "joint_name": joint_name,
                    "wheel_link_path": f"/World/Robot/wheel_{index}",
                }
                for index, joint_name in enumerate(WHEELS.values())
            ],
            "wheel_count": 4,
            "ground_filter_paths": list(ground_paths),
            "ground_filter_count": len(ground_paths),
            "max_contact_count": 128,
            "report_threshold_n": 0.0,
            "stage_usd_readback_verified": True,
        },
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
            "lateral_drift_m": (
                lateral
                if motion
                in {"forward", "backward", "rotate_left", "rotate_right"}
                else None
            ),
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
            "stationary_onset_after_command_sec": 0.05,
            "confirmed_after_command_sec": 0.55,
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
    odometry_mode: str = "ideal",
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
        "odometry_mode": odometry_mode,
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
            "stop": {
                "linear_velocity_threshold_mps": 0.02,
                "angular_velocity_threshold_radps": 0.05,
                "wheel_velocity_threshold_radps": 0.2,
                "stable_duration_sec": 0.5,
            },
            "wheels": WHEELS,
            "segments": configuration_segments,
        },
        "runtime_provenance": _runtime_provenance(
            environment=environment,
            contact_profile=contact_profile,
            odometry_mode=odometry_mode,
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
    report["output_file"] = str(path.resolve())
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
    odometry_mode: str = "ideal",
    report_schema_version: int = 3,
) -> list[Path]:
    return [
        _write(
            directory / f"{environment}_{topology}_{contact_profile}_{index}.json",
            _upgrade_runtime_provenance_to_v5(
                _report(
                    environment=environment,
                    contact_profile=contact_profile,
                    odometry_mode=odometry_mode,
                    scale=scale,
                ),
                topology,
                report_schema_version=report_schema_version,
            ),
        )
        for index, scale in enumerate((0.98, 1.0, 1.02))
    ]


def _three_v6_reports(
    directory: Path,
    *,
    strategy_id: str,
    environment: str = "SimplePlane",
    topology: str = "simple_plane_only1_v1",
    contact_profile: str = "threshold_corr_0p00025_offset_0p04",
) -> list[Path]:
    return [
        _write(
            directory / f"{strategy_id}_{index}.json",
            _upgrade_runtime_provenance_to_v6(
                _report(
                    environment=environment,
                    contact_profile=contact_profile,
                    scale=scale,
                ),
                strategy_id=strategy_id,
                topology=topology,
            ),
        )
        for index, scale in enumerate((0.98, 1.0, 1.02))
    ]


def _three_physical_acceptance_reports(
    directory: Path,
    *,
    contact_profile: str = "legacy_baseline",
    odometry_mode: str = "ideal",
) -> list[Path]:
    return _three_v5_reports(
        directory,
        environment="SimplePlane",
        topology="simple_plane_only1_v1",
        contact_profile=contact_profile,
        odometry_mode=odometry_mode,
    )


def _set_yaw_gain(segment, gain):
    expected = segment["yaw"]["expected_change_rad"]
    measured = expected * gain
    segment["yaw"]["change_rad"] = measured
    segment["yaw"]["error_rad"] = measured - expected
    segment["pose"]["end"]["yaw_rad"] = measured
    whole_distribution = segment["actual_velocity"]["angular_z_radps"]
    whole_rate = segment["command"]["angular_z_radps"] * gain
    if "sample_count" in whole_distribution:
        whole_distribution.update(
            _constant_speed_distribution(
                whole_rate, whole_distribution["sample_count"]
            )
        )
    else:
        whole_distribution["mean"] = whole_rate
    steady_state_window = segment["actual_velocity"].get(
        "steady_state_window"
    )
    if steady_state_window is not None:
        steady_rate = segment["command"]["angular_z_radps"] * gain
        steady_distribution = steady_state_window["angular_z_radps"]
        steady_distribution.update(
            {
                "mean": steady_rate,
                "mean_abs": abs(steady_rate),
                "minimum": steady_rate,
                "maximum": steady_rate,
                "peak_abs": abs(steady_rate),
                "rmse": abs(steady_rate),
            }
        )


def _set_pose_displacements(segment, *, longitudinal=None, lateral=None):
    pose = segment["pose"]
    start = pose["start"]
    current_longitudinal = pose["longitudinal_displacement_m"]
    current_lateral = pose["lateral_displacement_m"]
    longitudinal = (
        current_longitudinal if longitudinal is None else longitudinal
    )
    lateral = current_lateral if lateral is None else lateral
    yaw = start["yaw_rad"]
    delta_x = math.cos(yaw) * longitudinal - math.sin(yaw) * lateral
    delta_y = math.sin(yaw) * longitudinal + math.cos(yaw) * lateral
    pose["end"]["x_m"] = start["x_m"] + delta_x
    pose["end"]["y_m"] = start["y_m"] + delta_y
    net = math.hypot(delta_x, delta_y)
    pose["trajectory_length_m"] = abs(longitudinal) + abs(lateral)
    pose["net_displacement_m"] = net
    pose["longitudinal_displacement_m"] = longitudinal
    pose["longitudinal_error_m"] = (
        longitudinal - pose["expected_longitudinal_displacement_m"]
    )
    pose["lateral_displacement_m"] = lateral
    if segment["motion"] in {
        "forward",
        "backward",
        "rotate_left",
        "rotate_right",
    }:
        pose["lateral_drift_m"] = lateral
    else:
        pose["lateral_drift_m"] = None
    pose["translation_drift_m"] = (
        net
        if segment["motion"] in {"rotate_left", "rotate_right"}
        else None
    )


def _set_schema3_wheel_classification(
    segment,
    joint_name,
    direction,
    *,
    steady,
):
    container = (
        segment["wheels"]["steady_state_window"]
        if steady
        else segment["wheels"]
    )
    wheel = container["per_wheel"][joint_name]
    sample_count = wheel["speed_radps"]["sample_count"]
    if direction == "positive":
        distribution = _constant_speed_distribution(1.0, sample_count)
        positive, negative, within = sample_count, 0, 0
    elif direction == "negative":
        distribution = _constant_speed_distribution(-1.0, sample_count)
        positive, negative, within = 0, sample_count, 0
    elif direction == "stationary":
        distribution = _constant_speed_distribution(0.0, sample_count)
        positive, negative, within = 0, 0, sample_count
    else:
        assert direction == "mixed"
        positive = sample_count // 2
        negative = sample_count - positive
        within = 0
        distribution = {
            "sample_count": sample_count,
            "mean": (positive - negative) / sample_count,
            "mean_abs": 1.0,
            "minimum": -1.0,
            "maximum": 1.0,
            "peak_abs": 1.0,
            "rmse": 1.0,
        }
    wheel["direction"] = direction
    wheel["direction_matches"] = direction == wheel["expected_direction"]
    wheel["speed_radps"] = distribution
    if steady:
        wheel["direction_sample_counts"] = {
            "positive_above_deadband": positive,
            "negative_below_deadband": negative,
            "within_deadband": within,
        }
    container["all_directions_match"] = all(
        observation["direction_matches"]
        for observation in container["per_wheel"].values()
    )


def test_three_repeats_produce_audited_group_metrics(tmp_path):
    """Three unique reports produce every required group summary."""
    paths = _three_reports(tmp_path)

    report = analyse_contact_ab(paths, RADIUS_M)

    assert report["schema_version"] == 1
    assert "physical_acceptance" not in report
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

    assert report["schema_version"] == 4
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


def test_v6_reset_strategies_are_distinct_audited_physical_groups(tmp_path):
    pose = _three_v6_reports(
        tmp_path / "pose", strategy_id="pose_restore_v1"
    )
    separate = _three_v6_reports(
        tmp_path / "separate",
        strategy_id="separate_recontact_0p20m_1step_v1",
    )

    report = analyse_contact_ab(
        [*pose, *separate],
        RADIUS_M,
        expected_environments=("SimplePlane",),
        expected_topologies=("simple_plane_only1_v1",),
        expected_reset_strategies=(
            "reset-v1-pose_restore_v1",
            "reset-v1-separate_recontact_0p20m_1step_v1",
        ),
        expected_profiles=("threshold_corr_0p00025_offset_0p04",),
    )
    validate_physical_acceptance_accounting(report, expected_repeats=3)

    pose_group = (
        "SimplePlane::simple_plane_only1_v1::reset-v1-pose_restore_v1::"
        "threshold_corr_0p00025_offset_0p04"
    )
    separate_group = (
        "SimplePlane::simple_plane_only1_v1::"
        "reset-v1-separate_recontact_0p20m_1step_v1::"
        "threshold_corr_0p00025_offset_0p04"
    )
    assert report["schema_version"] == 5
    assert report["analysis_valid"] is True
    assert set(report["groups"]) == {pose_group, separate_group}
    assert report["selection_policy"][
        "required_runtime_provenance_schema"
    ] == 6
    assert report["selection_policy"]["expected_reset_strategies"] == [
        "reset-v1-pose_restore_v1",
        "reset-v1-separate_recontact_0p20m_1step_v1",
    ]
    assert "reset_strategy" not in report["locked_inputs"]["simulation"]
    assert set(report["reset_strategy_definitions"]) == {
        "reset-v1-pose_restore_v1",
        "reset-v1-separate_recontact_0p20m_1step_v1",
    }
    assert set(report["reset_contact_probe_contracts"]) == {
        "SimplePlane::simple_plane_only1_v1"
    }
    assert report["groups"][pose_group]["reset_strategy_id"] == (
        "pose_restore_v1"
    )
    assert report["groups"][separate_group]["reset_strategy_id"] == (
        "separate_recontact_0p20m_1step_v1"
    )
    assert all(
        item["runtime_provenance_schema_version"] == 6
        and item["reset_strategy_schema_version"] == 1
        and item["reset_strategy_token"].startswith("reset-v1-")
        for item in report["selection"]["included"]
    )
    acceptance = report["physical_acceptance"]
    assert acceptance["schema_version"] == 3
    assert acceptance["policy_id"] == "skid_steer_plan_8_7_v3"
    assert acceptance["applicability"][
        "required_runtime_provenance_schema"
    ] == 6
    assert set(acceptance["applicable_groups"]) == {
        pose_group,
        separate_group,
    }


def test_v6_reset_contact_probe_is_locked_across_strategy_arms(tmp_path):
    pose = _three_v6_reports(
        tmp_path / "pose", strategy_id="pose_restore_v1"
    )
    separate = _three_v6_reports(
        tmp_path / "separate",
        strategy_id="separate_recontact_0p20m_1step_v1",
    )
    changed = json.loads(separate[0].read_text(encoding="utf-8"))
    changed["runtime_provenance"]["simulation"]["reset_strategy"][
        "contact_probe"
    ]["wheel_bindings"][0]["wheel_link_path"] = (
        "/World/Robot/wheel_0/collider"
    )
    _write(separate[0], changed)

    with pytest.raises(
        ConfigurationError,
        match="reset contact-probe invariant mismatch",
    ):
        analyse_contact_ab([*pose, *separate], RADIUS_M)


def test_v6_expected_reset_matrix_rejects_a_missing_arm(tmp_path):
    pose = _three_v6_reports(
        tmp_path / "pose", strategy_id="pose_restore_v1"
    )

    with pytest.raises(
        ConfigurationError,
        match="expected reset strategies are missing",
    ):
        analyse_contact_ab(
            pose,
            RADIUS_M,
            expected_reset_strategies=(
                "reset-v1-pose_restore_v1",
                "reset-v1-separate_recontact_0p20m_1step_v1",
            ),
        )


def test_v6_physical_accounting_rejects_reset_identity_tampering(tmp_path):
    paths = _three_v6_reports(
        tmp_path, strategy_id="pose_restore_v1"
    )
    report = analyse_contact_ab(paths, RADIUS_M)
    report["selection"]["included"][0]["reset_strategy_token"] = (
        "reset-v1-separate_recontact_0p20m_1step_v1"
    )

    with pytest.raises(ConfigurationError, match="reset_strategy_token"):
        validate_physical_acceptance_accounting(report, expected_repeats=3)


def test_v6_physical_accounting_rejects_probe_contract_tampering(tmp_path):
    paths = _three_v6_reports(
        tmp_path, strategy_id="pose_restore_v1"
    )
    report = analyse_contact_ab(paths, RADIUS_M)
    probe_contract = next(
        iter(report["reset_contact_probe_contracts"].values())
    )
    probe_contract["wheel_count"] = 999

    with pytest.raises(
        ConfigurationError,
        match="reset_contact_probe_contracts",
    ):
        validate_physical_acceptance_accounting(report, expected_repeats=3)


def test_v6_physical_accounting_rejects_group_reset_contract_tampering(
    tmp_path,
):
    paths = _three_v6_reports(
        tmp_path, strategy_id="pose_restore_v1"
    )
    report = analyse_contact_ab(paths, RADIUS_M)
    group = next(iter(report["groups"].values()))
    group["reset_strategy_contract"]["lift_distance_m"] = 999.0

    with pytest.raises(
        ConfigurationError,
        match="reset_strategy_contract",
    ):
        validate_physical_acceptance_accounting(report, expected_repeats=3)


def test_v5_physical_acceptance_is_every_repeat_and_non_ranking(tmp_path):
    paths = _three_physical_acceptance_reports(tmp_path)

    report = analyse_contact_ab(paths, RADIUS_M)
    validate_physical_acceptance_accounting(report, expected_repeats=3)

    acceptance = report["physical_acceptance"]
    group_id = "SimplePlane::simple_plane_only1_v1::legacy_baseline"
    group = acceptance["groups"][group_id]
    assert acceptance["schema_version"] == 2
    assert acceptance["policy_id"] == "skid_steer_plan_8_7_v2"
    assert acceptance["evaluation_basis"] == "every_repeat"
    assert acceptance["ranking_policy"] == "none; pass/fail only"
    assert acceptance["applicability"] == {
        "required_motion_report_schema": 3,
        "required_runtime_provenance_schema": 5,
        "required_environment_id": "SimplePlane",
        "required_ground_topology_id": "simple_plane_only1_v1",
        "required_odometry_mode": "ideal",
        "minimum_unique_repeats_per_group": 3,
    }
    assert acceptance["steady_state_measurement_basis"] == (
        "actual_velocity.steady_state_window.angular_z_radps.mean over the "
        "final_half_of_command_interval window"
    )
    assert acceptance["wheel_direction_measurement_basis"] == (
        "wheels.steady_state_window.per_wheel[*].direction_matches over the "
        "final_half_of_command_interval window"
    )
    assert acceptance["thresholds"] == {
        "forward_abs_lateral_drift_max_m": 0.05,
        "backward_abs_lateral_drift_max_m": 0.08,
        "rotation_center_drift_max_m": 0.10,
        "rotation_center_drift_asymmetry_ratio_max": 0.20,
        "rotation_mean_yaw_rate_absolute_error_fraction_max": 0.10,
        "stop_stable_duration_min_sec": 0.5,
        "stop_linear_velocity_threshold_max_mps": 0.02,
        "stop_angular_velocity_threshold_max_radps": 0.05,
        "stop_wheel_velocity_threshold_max_radps": 0.20,
    }
    assert acceptance["applicable_groups"] == [group_id]
    assert acceptance["not_applicable_groups"] == []
    assert acceptance["passing_groups"] == [group_id]
    assert acceptance["failed_groups"] == []
    assert acceptance["all_applicable_groups_passed"] is True
    assert group["applicable"] is True
    assert group["not_applicable_reasons"] == []
    assert group["passed"] is True
    assert group["repeat_count"] == 3
    assert group["failed_checks"] == []
    assert len(group["repeat_results"]) == 3
    assert all(result["passed"] for result in group["repeat_results"])
    assert all(
        len(result["checks"]) == 18
        for result in group["repeat_results"]
    )
    assert report["groups"][group_id]["runtime_provenance_schema"] == 5
    assert report["groups"][group_id]["odometry_mode"] == "ideal"
    for result in group["repeat_results"]:
        wheel_check = result["checks"]["wheel_direction_contract"]
        assert wheel_check["steady_state_window_schema_version"] == 1
        assert wheel_check["validated_segment_count"] == 6
        assert wheel_check["validated_wheel_observation_count"] == 24
        assert set(wheel_check["segments"]) == {
            segment_id for segment_id, *_ in SEGMENTS
        }
        assert wheel_check["failed_observations"] == []
        assert wheel_check["passed"] is True
    assert report["method"]["ranking_policy"].startswith("none;")
    assert "best_profile" not in json.dumps(report)


def test_schema3_first_half_mixed_evidence_is_included_but_steady_gate_passes(
    tmp_path,
):
    paths = _three_physical_acceptance_reports(tmp_path)
    changed = json.loads(paths[0].read_text(encoding="utf-8"))
    _set_schema3_wheel_classification(
        changed["segments"][0],
        WHEELS["front_left"],
        "mixed",
        steady=False,
    )
    _write(paths[0], changed)

    report = analyse_contact_ab(paths, RADIUS_M)
    validate_physical_acceptance_accounting(report, expected_repeats=3)

    group = next(iter(report["physical_acceptance"]["groups"].values()))
    assert report["counts"]["included_reports"] == 3
    assert report["selection"]["excluded"] == []
    assert group["passed"] is True
    assert all(
        result["checks"]["wheel_direction_contract"]["passed"] is True
        for result in group["repeat_results"]
    )


@pytest.mark.parametrize("observed_direction", ("mixed", "positive", "stationary"))
def test_schema3_steady_direction_mismatch_is_included_and_fails_gate(
    tmp_path,
    observed_direction,
):
    paths = _three_physical_acceptance_reports(tmp_path)
    changed = json.loads(paths[1].read_text(encoding="utf-8"))
    _set_schema3_wheel_classification(
        changed["segments"][0],
        WHEELS["front_left"],
        observed_direction,
        steady=True,
    )
    _write(paths[1], changed)

    report = analyse_contact_ab(paths, RADIUS_M)
    validate_physical_acceptance_accounting(report, expected_repeats=3)

    group = next(iter(report["physical_acceptance"]["groups"].values()))
    assert report["counts"]["included_reports"] == 3
    assert report["selection"]["excluded"] == []
    assert group["passed"] is False
    assert group["failed_checks"] == ["wheel_direction_contract"]
    failed_repeat = next(
        result
        for result in group["repeat_results"]
        if result["checks"]["wheel_direction_contract"]["passed"] is False
    )
    wheel_check = failed_repeat["checks"]["wheel_direction_contract"]
    assert wheel_check["failed_observations"] == [
        f"rotate_left_360::{WHEELS['front_left']}"
    ]
    assert failed_repeat["failed_checks"] == ["wheel_direction_contract"]


def test_v5_physical_acceptance_rejects_one_repeat_not_the_mean(tmp_path):
    paths = _three_physical_acceptance_reports(tmp_path)
    changed = json.loads(paths[1].read_text(encoding="utf-8"))
    forward = changed["segments"][2]
    _set_pose_displacements(forward, lateral=0.051)
    _write(paths[1], changed)

    report = analyse_contact_ab(paths, RADIUS_M)

    group_id = "SimplePlane::simple_plane_only1_v1::legacy_baseline"
    group = report["physical_acceptance"]["groups"][group_id]
    assert report["analysis_valid"] is True
    assert report["groups"][group_id]["segments"]["forward_3m"][
        "lateral_drift_m"
    ]["mean"] < 0.05
    assert report["physical_acceptance"][
        "all_applicable_groups_passed"
    ] is False
    assert report["physical_acceptance"]["failed_groups"] == [group_id]
    assert group["passed"] is False
    assert group["failed_checks"] == ["forward_abs_lateral_drift_m"]
    assert [result["passed"] for result in group["repeat_results"]] == [
        True,
        False,
        True,
    ]
    assert group["repeat_results"][1]["failed_checks"] == [
        "forward_abs_lateral_drift_m"
    ]
    assert group["checks"]["forward_abs_lateral_drift_m"] == {
        "passed_repeats": 2,
        "failed_repeats": 1,
        "all_repeats_passed": False,
    }


@pytest.mark.parametrize("center_drift", (0.0, 0.10))
def test_v5_physical_acceptance_includes_exact_boundaries_and_zero_symmetry(
    tmp_path,
    center_drift,
):
    paths = _three_physical_acceptance_reports(tmp_path)
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        forward = document["segments"][2]
        _set_pose_displacements(forward, lateral=0.05)
        backward = document["segments"][3]
        _set_pose_displacements(backward, lateral=-0.08)
        for index, gain in ((0, 0.90), (1, 1.10)):
            _set_pose_displacements(
                document["segments"][index],
                longitudinal=0.0,
                lateral=center_drift,
            )
            _set_yaw_gain(document["segments"][index], gain)
        _write(path, document)

    report = analyse_contact_ab(paths, RADIUS_M)

    group = next(iter(report["physical_acceptance"]["groups"].values()))
    assert group["passed"] is True
    assert all(
        result["checks"]["rotation_center_drift_asymmetry_ratio"][
            "observed"
        ]
        == 0.0
        for result in group["repeat_results"]
    )
    assert all(
        result["checks"]["stop_window.rotate_left_360"][
            "observed_stable_duration_sec"
        ]
        == pytest.approx(0.5)
        for result in group["repeat_results"]
    )
    assert all(
        result["checks"][
            "rotate_left_mean_yaw_rate_absolute_error_fraction"
        ]["passed"]
        and result["checks"][
            "rotate_right_mean_yaw_rate_absolute_error_fraction"
        ]["passed"]
        for result in group["repeat_results"]
    )


def test_v5_physical_acceptance_uses_steady_state_yaw_rate_not_yaw_gain(
    tmp_path,
):
    paths = _three_physical_acceptance_reports(tmp_path)
    changed = json.loads(paths[1].read_text(encoding="utf-8"))
    yaw_distribution = changed["segments"][0]["actual_velocity"][
        "steady_state_window"
    ]["angular_z_radps"]
    yaw_distribution.update(
        _constant_speed_distribution(0.2, yaw_distribution["sample_count"])
    )
    full_yaw_distribution = changed["segments"][0]["actual_velocity"][
        "angular_z_radps"
    ]
    full_yaw_distribution.update(
        _constant_speed_distribution(
            0.2, full_yaw_distribution["sample_count"]
        )
    )
    _write(paths[1], changed)

    report = analyse_contact_ab(paths, RADIUS_M)

    group_id = "SimplePlane::simple_plane_only1_v1::legacy_baseline"
    analysis_group = report["groups"][group_id]
    acceptance_group = report["physical_acceptance"]["groups"][group_id]
    check_id = "rotate_left_mean_yaw_rate_absolute_error_fraction"
    assert analysis_group["segments"]["rotate_left_360"]["yaw_gain"][
        "mean"
    ] == pytest.approx(1.0)
    assert acceptance_group["passed"] is False
    assert acceptance_group["failed_checks"] == [check_id]
    failed_check = acceptance_group["repeat_results"][1]["checks"][check_id]
    assert failed_check == {
        "observed": 0.5,
        "maximum": 0.10,
        "passed": False,
        "commanded_yaw_rate_radps": 0.4,
        "steady_state_mean_yaw_rate_radps": 0.2,
        "steady_state_measurement_basis": (
            "actual_velocity.steady_state_window.angular_z_radps.mean"
        ),
    }


@pytest.mark.parametrize("report_schema_version", (1, 2))
def test_v5_legacy_motion_reports_are_aggregated_but_gate_is_not_applicable(
    tmp_path,
    report_schema_version,
):
    paths = _three_v5_reports(
        tmp_path,
        environment="SimplePlane",
        topology="simple_plane_only1_v1",
        report_schema_version=report_schema_version,
    )

    report = analyse_contact_ab(paths, RADIUS_M)

    group_id = "SimplePlane::simple_plane_only1_v1::legacy_baseline"
    acceptance = report["physical_acceptance"]
    assert report["analysis_valid"] is True
    assert {
        item["report_schema_version"]
        for item in report["groups"][group_id]["input_reports"]
    } == {report_schema_version}
    assert acceptance["groups"][group_id]["applicable"] is False
    assert acceptance["groups"][group_id]["passed"] is None
    assert acceptance["groups"][group_id]["not_applicable_reasons"] == [
        "motion_report_schema_not_3"
    ]
    assert acceptance["all_applicable_groups_passed"] is None


@pytest.mark.parametrize(
    "mutation",
    (
        "missing_window",
        "unknown_window_key",
        "wrong_window_schema",
        "wrong_definition",
        "wrong_start",
        "wrong_end",
        "wrong_duration",
        "zero_sample_count",
        "distribution_count_mismatch",
        "nonfinite_mean_shape",
        "unknown_distribution_key",
    ),
)
def test_v3_motion_report_yaw_steady_state_window_is_strict(tmp_path, mutation):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    actual_velocity = document["segments"][0]["actual_velocity"]
    window = actual_velocity["steady_state_window"]
    if mutation == "missing_window":
        del actual_velocity["steady_state_window"]
    elif mutation == "unknown_window_key":
        window["unlocked"] = True
    elif mutation == "wrong_window_schema":
        window["schema_version"] = 2
    elif mutation == "wrong_definition":
        window["definition"] = "last_sample_only"
    elif mutation == "wrong_start":
        window["start_stamp_ns"] += 1
    elif mutation == "wrong_end":
        window["end_stamp_ns"] -= 1
    elif mutation == "wrong_duration":
        window["observed_duration_sec"] += 0.001
    elif mutation == "zero_sample_count":
        window["sample_count"] = 0
    elif mutation == "distribution_count_mismatch":
        window["angular_z_radps"]["sample_count"] += 1
    elif mutation == "nonfinite_mean_shape":
        window["angular_z_radps"]["mean"] = "not-a-number"
    else:
        assert mutation == "unknown_distribution_key"
        window["angular_z_radps"]["unlocked"] = 0.4
    _write(path, document)

    with pytest.raises(ConfigurationError, match="steady_state_window"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize(
    "mutation",
    (
        "start",
        "end",
        "duration",
        "count",
        "deadband",
        "direction_sample_counts",
        "distribution",
        "classification",
        "expected_direction",
        "direction_matches",
        "aggregate",
    ),
)
def test_schema3_wheel_steady_state_window_is_fail_closed(tmp_path, mutation):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    window = document["segments"][0]["wheels"]["steady_state_window"]
    wheel = window["per_wheel"][WHEELS["front_left"]]
    if mutation == "start":
        window["start_stamp_ns"] += 1
    elif mutation == "end":
        window["end_stamp_ns"] -= 1
    elif mutation == "duration":
        window["observed_duration_sec"] += 0.001
    elif mutation == "count":
        window["sample_count"] += 1
    elif mutation == "deadband":
        window["classification_deadband_radps"] += 0.001
    elif mutation == "direction_sample_counts":
        counts = wheel["direction_sample_counts"]
        counts["negative_below_deadband"] = 0
        counts["positive_above_deadband"] = window["sample_count"]
    elif mutation == "distribution":
        wheel["speed_radps"]["sample_count"] += 1
    elif mutation == "classification":
        wheel["direction"] = "positive"
        wheel["direction_matches"] = False
        window["all_directions_match"] = False
    elif mutation == "expected_direction":
        wheel["expected_direction"] = "positive"
        wheel["direction_matches"] = False
        window["all_directions_match"] = False
    elif mutation == "direction_matches":
        wheel["direction_matches"] = False
        window["all_directions_match"] = False
    else:
        assert mutation == "aggregate"
        window["all_directions_match"] = False
    _write(path, document)

    with pytest.raises(ConfigurationError, match="steady_state_window"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize("stream", ("odom", "wheel"))
@pytest.mark.parametrize("gap_case", ("below_average", "above_span"))
def test_schema3_steady_window_gap_must_be_mathematically_feasible(
    tmp_path, stream, gap_case
):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    segment = document["segments"][0]
    window = (
        segment["actual_velocity"]["steady_state_window"]
        if stream == "odom"
        else segment["wheels"]["steady_state_window"]
    )
    span = (
        window["last_sample_stamp_ns"] - window["first_sample_stamp_ns"]
    ) / 1_000_000_000
    average = span / (window["sample_count"] - 1)
    window["maximum_inter_sample_gap_sec"] = (
        average / 2.0 if gap_case == "below_average" else span + 0.001
    )
    _write(path, document)

    with pytest.raises(ConfigurationError, match="maximum_inter_sample_gap"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize(
    "mutation",
    (
        "max_sample_age",
        "unknown_sample_count",
        "nonpositive_sample_count",
        "total_below_command",
        "full_odom_count",
        "full_wheel_count",
        "steady_odom_exceeds_command",
        "steady_wheel_exceeds_command",
    ),
)
def test_schema3_sampling_counts_are_bound_to_the_fixed_protocol(
    tmp_path, mutation
):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    segment = document["segments"][0]
    counts = segment["sample_counts"]
    if mutation == "max_sample_age":
        document["configuration"]["sampling"]["max_sample_age_sec"] = 0.5001
    elif mutation == "unknown_sample_count":
        counts["unlocked"] = 1
    elif mutation == "nonpositive_sample_count":
        counts["odom_command"] = 0
    elif mutation == "total_below_command":
        counts["odom_total"] = counts["odom_command"] - 1
    elif mutation == "full_odom_count":
        segment["actual_velocity"]["linear_x_mps"]["sample_count"] += 1
    elif mutation == "full_wheel_count":
        segment["wheels"]["per_wheel"][WHEELS["front_left"]][
            "speed_radps"
        ]["sample_count"] += 1
    elif mutation == "steady_odom_exceeds_command":
        segment["actual_velocity"]["steady_state_window"]["sample_count"] = (
            counts["odom_command"] + 1
        )
    else:
        assert mutation == "steady_wheel_exceeds_command"
        segment["wheels"]["steady_state_window"]["sample_count"] = (
            counts["joint_states_command"] + 1
        )
    _write(path, document)

    with pytest.raises(ConfigurationError):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize(
    ("mutation", "message"),
    (
        ("output_path", "canonical source path"),
        ("invalid_started", "ISO-8601"),
        ("naive_started", "must be UTC"),
        ("non_utc_completed", "must be UTC"),
        ("reversed", "precedes started_at_utc"),
    ),
)
def test_schema3_report_path_and_time_identity_is_fail_closed(
    tmp_path, mutation, message
):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    if mutation == "output_path":
        document["output_file"] = "/reports/forged.json"
    elif mutation == "invalid_started":
        document["started_at_utc"] = "not-a-time"
    elif mutation == "naive_started":
        document["started_at_utc"] = "2026-07-14T00:00:00"
    elif mutation == "non_utc_completed":
        document["completed_at_utc"] = "2026-07-14T08:01:00+08:00"
    else:
        assert mutation == "reversed"
        document["started_at_utc"] = "2026-07-14T00:02:00+00:00"
        document["completed_at_utc"] = "2026-07-14T00:01:00+00:00"
    path.write_text(json.dumps(document, sort_keys=True), encoding="utf-8")

    with pytest.raises(ConfigurationError, match=message):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize(
    "mutation",
    (
        "forward_endpoint",
        "rotation_endpoint",
        "endpoint_shape",
        "trajectory_shorter_than_net",
    ),
)
def test_pose_metrics_are_bound_to_finite_endpoints(tmp_path, mutation):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    if mutation == "forward_endpoint":
        document["segments"][2]["pose"]["end"]["y_m"] += 10.0
    elif mutation == "rotation_endpoint":
        document["segments"][0]["pose"]["end"]["x_m"] += 10.0
    elif mutation == "endpoint_shape":
        document["segments"][2]["pose"]["start"]["unlocked"] = 0.0
    else:
        assert mutation == "trajectory_shorter_than_net"
        document["segments"][2]["pose"]["trajectory_length_m"] = 0.0
    _write(path, document)

    with pytest.raises(ConfigurationError, match="pose"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


def test_pose_endpoint_yaw_must_match_reported_yaw_change_modulo_turn(tmp_path):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    document["segments"][2]["pose"]["end"]["yaw_rad"] = 1.234
    _write(path, document)

    with pytest.raises(ConfigurationError, match="endpoint yaw"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


def test_schema3_steady_distribution_must_be_a_full_window_subset(tmp_path):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    distribution = document["segments"][0]["actual_velocity"][
        "angular_z_radps"
    ]
    distribution.update(
        _constant_speed_distribution(0.01, distribution["sample_count"])
    )
    _write(path, document)

    with pytest.raises(ConfigurationError, match="subset|extrema"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


def test_schema3_total_counts_cover_command_and_stop_windows(tmp_path):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    segment = document["segments"][0]
    for source in ("odom", "joint_states"):
        total_key = f"{source}_total"
        command_count = segment["sample_counts"][f"{source}_command"]
        removed = segment["sample_counts"][total_key] - command_count
        segment["sample_counts"][total_key] = command_count
        segment["timestamp_integrity"][source]["sample_count"] = command_count
        document["timestamp_integrity"][source]["sample_count"] -= removed
    _write(path, document)

    with pytest.raises(ConfigurationError, match="command and stationary"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize(
    "mutation",
    (
        "invalid_nonzero",
        "invalid_unknown",
        "timestamp_unknown",
        "timestamp_regression",
        "timestamp_duplicate",
        "timestamp_count_mismatch",
        "monotonic_contradiction",
        "steady_outside_tracker",
        "top_count_too_small",
        "top_monotonic_contradiction",
    ),
)
def test_schema3_timestamp_and_invalid_evidence_is_fail_closed(
    tmp_path, mutation
):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    segment = document["segments"][0]
    odom_tracker = segment["timestamp_integrity"]["odom"]
    if mutation == "invalid_nonzero":
        segment["invalid_message_counts"]["odom"] = 1
    elif mutation == "invalid_unknown":
        segment["invalid_message_counts"]["unlocked"] = 0
    elif mutation == "timestamp_unknown":
        odom_tracker["unlocked"] = 0
    elif mutation == "timestamp_regression":
        odom_tracker["regression_count"] = 1
        odom_tracker["monotonic_unique"] = False
    elif mutation == "timestamp_duplicate":
        odom_tracker["duplicate_count"] = 1
        odom_tracker["monotonic_unique"] = False
    elif mutation == "timestamp_count_mismatch":
        odom_tracker["sample_count"] += 1
    elif mutation == "monotonic_contradiction":
        odom_tracker["monotonic_unique"] = False
    elif mutation == "steady_outside_tracker":
        odom_tracker["last_stamp_ns"] = segment["actual_velocity"][
            "steady_state_window"
        ]["last_sample_stamp_ns"] - 1
    elif mutation == "top_count_too_small":
        document["timestamp_integrity"]["odom"]["sample_count"] = 1
    else:
        assert mutation == "top_monotonic_contradiction"
        document["timestamp_integrity"]["odom"]["monotonic_unique"] = True
    _write(path, document)

    with pytest.raises(ConfigurationError):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize(
    "mutation",
    (
        "not_mapping",
        "unknown_key",
        "owner_false",
        "zero_burst_false",
        "no_subscription",
        "wrong_zero_count",
        "empty_reset_publishers",
        "duplicate_reset_publishers",
    ),
)
def test_schema3_safety_evidence_is_fail_closed(tmp_path, mutation):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    safety = document["safety"]
    if mutation == "not_mapping":
        document["safety"] = "garbage"
    elif mutation == "unknown_key":
        safety["unlocked"] = True
    elif mutation == "owner_false":
        safety["exclusive_non_reset_cmd_vel_owner_enforced"] = False
    elif mutation == "zero_burst_false":
        safety["safe_zero_burst_attempted"] = False
    elif mutation == "no_subscription":
        safety["cmd_vel_subscription_count"] = 0
    elif mutation == "wrong_zero_count":
        safety["zero_publish_count"] += 1
    elif mutation == "empty_reset_publishers":
        safety["authorized_reset_safety_publishers"] = []
    else:
        assert mutation == "duplicate_reset_publishers"
        safety["authorized_reset_safety_publishers"] *= 2
    _write(path, document)

    with pytest.raises(ConfigurationError):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize(
    "mutation",
    (
        "missing_key",
        "wrong_service",
        "boolean_generation",
        "fresh_false",
        "wrong_settle",
        "boundary_not_followed",
        "received_below_credited",
        "rollback_contradiction",
        "observation_unknown",
        "duplicate_generation",
        "violation_unknown",
        "peak_unknown",
        "command_before_recovery",
        "tracker_before_received",
    ),
)
def test_schema3_reset_evidence_is_fail_closed(tmp_path, mutation):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    segment = document["segments"][0]
    reset = segment["reset"]
    if mutation == "missing_key":
        del reset["response_message"]
    elif mutation == "wrong_service":
        reset["service"] = "/another/reset"
    elif mutation == "boolean_generation":
        reset["reset_generation"] = True
    elif mutation == "fresh_false":
        reset["fresh_joint_states_received"] = False
    elif mutation == "wrong_settle":
        reset["stationary_settle_duration_sec"] = 0.499
    elif mutation == "boundary_not_followed":
        reset["reset_boundary_clock_ns"] = reset["clock_after_ns"]
    elif mutation == "received_below_credited":
        reset["received_stamp_high_watermarks_ns"]["odom"] = (
            reset["credited_stamp_high_watermarks_ns"]["odom"] - 1
        )
    elif mutation == "rollback_contradiction":
        reset["clock_rollback_observed"] = False
    elif mutation == "observation_unknown":
        reset["recovery_observation_counts"]["unlocked"] = 0
    elif mutation == "duplicate_generation":
        document["segments"][1]["reset"]["reset_generation"] = reset[
            "reset_generation"
        ]
    elif mutation == "violation_unknown":
        reset["recovery_violation_counts"]["garbage"] = 0
    elif mutation == "peak_unknown":
        reset["recovery_peak_observed"]["garbage"] = True
    elif mutation == "command_before_recovery":
        after = segment["command"]["start_stamp_ns"] + 1
        reset["clock_before_ns"] = after + 1
        reset["clock_after_ns"] = after
        reset["clock_rollback_observed"] = True
        reset["credited_stamp_high_watermarks_ns"]["clock"] = after
        reset["received_stamp_high_watermarks_ns"]["clock"] = after
    else:
        assert mutation == "tracker_before_received"
        reset["received_stamp_high_watermarks_ns"]["odom"] = (
            segment["timestamp_integrity"]["odom"]["first_stamp_ns"] + 1
        )
    _write(path, document)

    with pytest.raises(ConfigurationError):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize(
    "mutation",
    (
        "missing_evidence",
        "wrong_boundary",
        "end_mismatch",
        "sample_count_one",
        "gap_above_limit",
        "first_before_window",
        "last_not_after_start",
        "wrong_max_age",
    ),
)
def test_schema3_stationary_evidence_is_fail_closed(tmp_path, mutation):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    stopping = document["segments"][0]["stopping"]
    evidence = stopping["stationary_evidence"]
    stream = evidence["streams"]["odom"]
    if mutation == "missing_evidence":
        del stopping["stationary_evidence"]
    elif mutation == "wrong_boundary":
        evidence["boundary_semantics"] = "open_interval"
    elif mutation == "end_mismatch":
        evidence["end_stamp_ns"] -= 1
    elif mutation == "sample_count_one":
        stream["sample_count"] = 1
    elif mutation == "gap_above_limit":
        stream["maximum_inter_sample_gap_sec"] = 0.501
    elif mutation == "first_before_window":
        stream["first_sample_stamp_ns"] = evidence["start_stamp_ns"] - 1
    elif mutation == "last_not_after_start":
        stream["last_sample_stamp_ns"] = evidence["start_stamp_ns"]
    else:
        assert mutation == "wrong_max_age"
        evidence["max_sample_age_sec"] = 0.499
    _write(path, document)

    with pytest.raises(ConfigurationError):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


@pytest.mark.parametrize(
    "mutation", ("within_with_positive_extrema", "all_positive_min", "all_negative_max")
)
def test_schema3_direction_counts_must_be_feasible_for_extrema(
    tmp_path, mutation
):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    window = document["segments"][0]["wheels"]["steady_state_window"]
    joint_name = (
        WHEELS["front_left"]
        if mutation == "all_negative_max"
        else WHEELS["front_right"]
    )
    wheel = window["per_wheel"][joint_name]
    sample_count = window["sample_count"]
    if mutation == "within_with_positive_extrema":
        wheel["direction_sample_counts"] = {
            "positive_above_deadband": sample_count - 1,
            "negative_below_deadband": 0,
            "within_deadband": 1,
        }
    elif mutation == "all_positive_min":
        wheel["speed_radps"]["minimum"] = 0.0
    else:
        wheel["speed_radps"]["maximum"] = 0.0
    _write(path, document)

    with pytest.raises(
        ConfigurationError, match="direction_sample_counts|impossible"
    ):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


def test_schema3_direction_counts_must_be_feasible_for_reported_moments(
    tmp_path,
):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    window = document["segments"][2]["wheels"]["steady_state_window"]
    wheel = window["per_wheel"][WHEELS["front_left"]]
    sample_count = window["sample_count"]
    assert sample_count == 13
    wheel["direction_sample_counts"] = {
        "positive_above_deadband": 1,
        "negative_below_deadband": 0,
        "within_deadband": sample_count - 1,
    }
    # These moments uniquely require twelve 1.0 samples and one 0.0 sample,
    # which is the opposite of the claimed direction buckets.
    wheel["speed_radps"] = {
        "sample_count": sample_count,
        "mean": 12.0 / sample_count,
        "mean_abs": 12.0 / sample_count,
        "minimum": 0.0,
        "maximum": 1.0,
        "peak_abs": 1.0,
        "rmse": math.sqrt(12.0 / sample_count),
    }
    _write(path, document)

    with pytest.raises(ConfigurationError, match="reported mean moment"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


def test_schema3_direction_counts_jointly_constrain_abs_and_square_moments(
    tmp_path,
):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    window = document["segments"][2]["wheels"]["steady_state_window"]
    wheel = window["per_wheel"][WHEELS["front_left"]]
    sample_count = window["sample_count"]
    assert sample_count == 13
    wheel["direction_sample_counts"] = {
        "positive_above_deadband": 1,
        "negative_below_deadband": 0,
        "within_deadband": sample_count - 1,
    }
    # The moments encode [1.0, 0.21, 0.0 x 11].  The 0.21 sample is above
    # the 0.2 deadband, so no distribution with only one positive bucket can
    # realize both the reported absolute and square sums.
    wheel["speed_radps"] = {
        "sample_count": sample_count,
        "mean": 1.21 / sample_count,
        "mean_abs": 1.21 / sample_count,
        "minimum": 0.0,
        "maximum": 1.0,
        "peak_abs": 1.0,
        "rmse": math.sqrt(1.0441 / sample_count),
    }
    _write(path, document)

    with pytest.raises(ConfigurationError, match="jointly realize"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


def test_schema3_distribution_moments_must_be_feasible_for_extrema(tmp_path):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    distribution = document["segments"][0]["actual_velocity"][
        "steady_state_window"
    ]["angular_z_radps"]
    distribution.update(
        {
            "mean": 0.4,
            "mean_abs": 0.4,
            "minimum": -100.0,
            "maximum": 100.0,
            "peak_abs": 100.0,
            "rmse": 0.4,
        }
    )
    _write(path, document)

    with pytest.raises(ConfigurationError, match="impossible"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


def test_schema3_distribution_moments_must_be_jointly_feasible(tmp_path):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    distribution = document["segments"][0]["actual_velocity"][
        "steady_state_window"
    ]["angular_z_radps"]
    sample_count = distribution["sample_count"]
    distribution.update(
        {
            "mean": 0.4,
            "mean_abs": 0.4,
            "minimum": -1.0,
            "maximum": 1.0,
            "peak_abs": 1.0,
            "rmse": 0.5,
        }
    )
    assert sample_count > 2
    _write(path, document)

    with pytest.raises(ConfigurationError, match="interior"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


def test_schema3_distribution_moments_respect_asymmetric_extrema(tmp_path):
    path = _three_physical_acceptance_reports(tmp_path)[0]
    document = json.loads(path.read_text(encoding="utf-8"))
    distribution = document["segments"][0]["actual_velocity"]["linear_y_mps"]
    sample_count = 65
    distribution.update(
        {
            "sample_count": sample_count,
            "mean": 5.4 / sample_count,
            "mean_abs": 7.4 / sample_count,
            "minimum": -1.0,
            "maximum": 0.1,
            "peak_abs": 1.0,
            "rmse": math.sqrt(3.01 / sample_count),
        }
    )
    document["segments"][0]["sample_counts"]["odom_command"] = sample_count
    _write(path, document)

    with pytest.raises(ConfigurationError, match="interior moments"):
        analyse_contact_ab([path], RADIUS_M, min_repeats=1)


def test_v5_physical_acceptance_marks_warehouse_group_not_applicable(
    tmp_path,
):
    paths = _three_v5_reports(tmp_path)

    report = analyse_contact_ab(paths, RADIUS_M)
    validate_physical_acceptance_accounting(report, expected_repeats=3)

    acceptance = report["physical_acceptance"]
    group_id = "Warehouse::warehouse_combined32_v1::legacy_baseline"
    group = acceptance["groups"][group_id]
    assert acceptance["applicable_groups"] == []
    assert acceptance["not_applicable_groups"] == [group_id]
    assert acceptance["passing_groups"] == []
    assert acceptance["failed_groups"] == []
    assert acceptance["all_applicable_groups_passed"] is None
    assert group == {
        "applicable": False,
        "passed": None,
        "not_applicable_reasons": [
            "environment_not_SimplePlane",
            "ground_topology_not_simple_plane_only1_v1",
        ],
        "repeat_count": 3,
        "checks": {},
        "failed_checks": [],
        "repeat_results": [],
    }


@pytest.mark.parametrize(
    "tamper_case",
    (
        "acceptance_group_missing",
        "partition_overlap",
        "acceptance_repeat_count_bool",
        "analysis_input_report_removed",
        "repeat_identity_sha_swap",
        "repeat_index_gap",
        "repeat_check_passed_non_bool",
        "repeat_failed_checks_drift",
        "repeat_check_removed",
        "required_check_removed_everywhere",
        "observed_threshold_contradiction",
        "leaf_limit_drift",
        "yaw_evidence_drift",
        "stop_duration_drift",
        "aggregate_count_drift",
        "group_passed_drift",
        "forged_not_applicable",
        "top_threshold_drift",
        "top_policy_drift",
        "top_applicability_drift",
        "top_steady_basis_drift",
        "selection_identity_drift",
        "matrix_observed_drift",
        "coordinated_group_identity_forgery",
        "forged_odometry_not_applicable",
        "top_verdict_drift",
    ),
)
def test_physical_acceptance_accounting_rejects_applicable_tampering(
    tmp_path,
    tamper_case,
):
    report = analyse_contact_ab(
        _three_physical_acceptance_reports(tmp_path),
        RADIUS_M,
    )
    document = json.loads(json.dumps(report))
    acceptance = document["physical_acceptance"]
    group_id = acceptance["applicable_groups"][0]
    group = acceptance["groups"][group_id]
    first_repeat = group["repeat_results"][0]
    first_check_id = sorted(first_repeat["checks"])[0]

    if tamper_case == "acceptance_group_missing":
        del acceptance["groups"][group_id]
    elif tamper_case == "partition_overlap":
        acceptance["failed_groups"].append(group_id)
    elif tamper_case == "acceptance_repeat_count_bool":
        group["repeat_count"] = True
    elif tamper_case == "analysis_input_report_removed":
        document["groups"][group_id]["input_reports"].pop()
    elif tamper_case == "repeat_identity_sha_swap":
        repeat_a, repeat_b = group["repeat_results"][:2]
        repeat_a["report_sha256"], repeat_b["report_sha256"] = (
            repeat_b["report_sha256"],
            repeat_a["report_sha256"],
        )
    elif tamper_case == "repeat_index_gap":
        group["repeat_results"][1]["repeat_index"] = 3
    elif tamper_case == "repeat_check_passed_non_bool":
        first_repeat["checks"][first_check_id]["passed"] = 1
    elif tamper_case == "repeat_failed_checks_drift":
        first_repeat["failed_checks"] = [first_check_id]
    elif tamper_case == "repeat_check_removed":
        del first_repeat["checks"][first_check_id]
    elif tamper_case == "required_check_removed_everywhere":
        for repeat in group["repeat_results"]:
            del repeat["checks"][first_check_id]
        del group["checks"][first_check_id]
    elif tamper_case == "observed_threshold_contradiction":
        first_repeat["checks"]["forward_abs_lateral_drift_m"][
            "observed"
        ] = 0.051
    elif tamper_case == "leaf_limit_drift":
        first_repeat["checks"]["forward_abs_lateral_drift_m"][
            "maximum"
        ] = 0.06
    elif tamper_case == "yaw_evidence_drift":
        first_repeat["checks"][
            "rotate_left_mean_yaw_rate_absolute_error_fraction"
        ]["steady_state_mean_yaw_rate_radps"] = 0.2
    elif tamper_case == "stop_duration_drift":
        first_repeat["checks"]["stop_window.forward_3m"][
            "observed_stable_duration_sec"
        ] = 0.6
    elif tamper_case == "aggregate_count_drift":
        group["checks"][first_check_id]["passed_repeats"] = 2
    elif tamper_case == "group_passed_drift":
        group["passed"] = False
    elif tamper_case == "forged_not_applicable":
        group.update(
            {
                "applicable": False,
                "passed": None,
                "not_applicable_reasons": ["motion_report_schema_not_3"],
                "checks": {},
                "failed_checks": [],
                "repeat_results": [],
            }
        )
        acceptance["applicable_groups"] = []
        acceptance["not_applicable_groups"] = [group_id]
        acceptance["passing_groups"] = []
        acceptance["failed_groups"] = []
        acceptance["all_applicable_groups_passed"] = None
    elif tamper_case == "top_threshold_drift":
        acceptance["thresholds"]["forward_abs_lateral_drift_max_m"] = 0.06
    elif tamper_case == "top_policy_drift":
        acceptance["policy_id"] = "forged"
    elif tamper_case == "top_applicability_drift":
        acceptance["applicability"]["required_odometry_mode"] = "realistic"
    elif tamper_case == "top_steady_basis_drift":
        acceptance["steady_state_measurement_basis"] = "whole command"
    elif tamper_case == "selection_identity_drift":
        document["selection"]["included"][0]["sha256"] = "0" * 64
    elif tamper_case == "matrix_observed_drift":
        document["matrix"]["observed_groups"] = []
    elif tamper_case == "coordinated_group_identity_forgery":
        forged_group_id = (
            "Warehouse::warehouse_combined32_v1::legacy_baseline"
        )
        analysis_group = document["groups"].pop(group_id)
        analysis_group["environment_id"] = "Warehouse"
        analysis_group["ground_topology_id"] = "warehouse_combined32_v1"
        document["groups"][forged_group_id] = analysis_group
        forged_acceptance_group = acceptance["groups"].pop(group_id)
        forged_acceptance_group.update(
            {
                "applicable": False,
                "passed": None,
                "not_applicable_reasons": [
                    "environment_not_SimplePlane",
                    "ground_topology_not_simple_plane_only1_v1",
                ],
                "checks": {},
                "failed_checks": [],
                "repeat_results": [],
            }
        )
        acceptance["groups"][forged_group_id] = forged_acceptance_group
        acceptance["applicable_groups"] = []
        acceptance["not_applicable_groups"] = [forged_group_id]
        acceptance["passing_groups"] = []
        acceptance["failed_groups"] = []
        acceptance["all_applicable_groups_passed"] = None
    elif tamper_case == "forged_odometry_not_applicable":
        document["groups"][group_id]["odometry_mode"] = "realistic"
        group.update(
            {
                "applicable": False,
                "passed": None,
                "not_applicable_reasons": ["odometry_mode_not_ideal"],
                "checks": {},
                "failed_checks": [],
                "repeat_results": [],
            }
        )
        acceptance["applicable_groups"] = []
        acceptance["not_applicable_groups"] = [group_id]
        acceptance["passing_groups"] = []
        acceptance["failed_groups"] = []
        acceptance["all_applicable_groups_passed"] = None
    else:
        assert tamper_case == "top_verdict_drift"
        acceptance["all_applicable_groups_passed"] = None

    with pytest.raises(ConfigurationError):
        validate_physical_acceptance_accounting(
            document,
            expected_repeats=3,
        )


@pytest.mark.parametrize(
    "tamper_case",
    (
        "empty_reasons",
        "non_null_passed",
        "forged_checks",
        "forged_repeat_results",
        "non_null_top_verdict",
    ),
)
def test_physical_acceptance_accounting_rejects_not_applicable_tampering(
    tmp_path,
    tamper_case,
):
    report = analyse_contact_ab(_three_v5_reports(tmp_path), RADIUS_M)
    document = json.loads(json.dumps(report))
    acceptance = document["physical_acceptance"]
    group_id = acceptance["not_applicable_groups"][0]
    group = acceptance["groups"][group_id]

    if tamper_case == "empty_reasons":
        group["not_applicable_reasons"] = []
    elif tamper_case == "non_null_passed":
        group["passed"] = False
    elif tamper_case == "forged_checks":
        group["checks"] = {"forged": {"all_repeats_passed": True}}
    elif tamper_case == "forged_repeat_results":
        group["repeat_results"] = [{"forged": True}]
    else:
        assert tamper_case == "non_null_top_verdict"
        acceptance["all_applicable_groups_passed"] = False

    with pytest.raises(ConfigurationError):
        validate_physical_acceptance_accounting(
            document,
            expected_repeats=3,
        )


def test_physical_acceptance_accounting_rejects_required_group_shrink(
    tmp_path,
):
    report = analyse_contact_ab(
        _three_physical_acceptance_reports(tmp_path),
        RADIUS_M,
        expected_environments=("SimplePlane",),
        expected_topologies=("simple_plane_only1_v1",),
        expected_profiles=("legacy_baseline",),
    )
    validate_physical_acceptance_accounting(report, expected_repeats=3)
    assert report["matrix"]["required_groups"] == report["matrix"][
        "observed_groups"
    ]
    document = json.loads(json.dumps(report))
    document["matrix"]["required_groups"] = []

    with pytest.raises(ConfigurationError, match="required_groups"):
        validate_physical_acceptance_accounting(
            document,
            expected_repeats=3,
        )


@pytest.mark.parametrize(
    "mutation",
    (
        "missing_segment",
        "missing_wheel",
        "direction",
        "expected_direction",
        "direction_matches",
        "failed_observations",
        "passed",
    ),
)
def test_physical_accounting_rejects_wheel_leaf_tampering(tmp_path, mutation):
    report = analyse_contact_ab(
        _three_physical_acceptance_reports(tmp_path),
        RADIUS_M,
    )
    document = json.loads(json.dumps(report))
    group_id = document["physical_acceptance"]["applicable_groups"][0]
    wheel_check = document["physical_acceptance"]["groups"][group_id][
        "repeat_results"
    ][0]["checks"]["wheel_direction_contract"]
    segment = wheel_check["segments"]["rotate_left_360"]
    observation = segment["per_wheel"][WHEELS["front_left"]]
    if mutation == "missing_segment":
        del wheel_check["segments"]["rotate_left_360"]
    elif mutation == "missing_wheel":
        del segment["per_wheel"][WHEELS["front_left"]]
    elif mutation == "direction":
        observation["direction"] = "positive"
    elif mutation == "expected_direction":
        observation["expected_direction"] = "positive"
    elif mutation == "direction_matches":
        observation["direction_matches"] = False
    elif mutation == "failed_observations":
        wheel_check["failed_observations"] = [
            f"rotate_left_360::{WHEELS['front_left']}"
        ]
    else:
        assert mutation == "passed"
        wheel_check["passed"] = False

    with pytest.raises(ConfigurationError):
        validate_physical_acceptance_accounting(
            document,
            expected_repeats=3,
        )


@pytest.mark.parametrize(
    ("identity_field", "repeat_field"),
    (("sha256", "report_sha256"), ("canonical_sha256", "canonical_sha256")),
)
def test_physical_accounting_revalidates_source_hashes(
    tmp_path, identity_field, repeat_field
):
    paths = _three_physical_acceptance_reports(tmp_path)
    document = analyse_contact_ab(paths, RADIUS_M)
    group_id = document["physical_acceptance"]["applicable_groups"][0]
    target_path = str(paths[0].resolve())
    selection = next(
        item
        for item in document["selection"]["included"]
        if item["path"] == target_path
    )
    group_report = next(
        item
        for item in document["groups"][group_id]["input_reports"]
        if item["path"] == target_path
    )
    repeat = next(
        item
        for item in document["physical_acceptance"]["groups"][group_id][
            "repeat_results"
        ]
        if item["report_path"] == target_path
    )
    selection[identity_field] = "0" * 64
    group_report[identity_field] = "0" * 64
    repeat[repeat_field] = "0" * 64

    with pytest.raises(ConfigurationError, match="source report"):
        validate_physical_acceptance_accounting(document, expected_repeats=3)


def test_physical_accounting_rejects_coordinated_source_global_lock_change(
    tmp_path,
):
    paths = _three_physical_acceptance_reports(tmp_path)
    document = analyse_contact_ab(paths, RADIUS_M)
    target_path = str(paths[0].resolve())
    changed = json.loads(paths[0].read_text(encoding="utf-8"))
    changed["runtime_provenance"]["git"]["commit"] = "e" * 40
    _write(paths[0], changed)
    content = paths[0].read_bytes()
    raw_sha256 = hashlib.sha256(content).hexdigest()
    canonical_sha256 = hashlib.sha256(
        json.dumps(
            json.loads(content),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    group_id = document["physical_acceptance"]["applicable_groups"][0]
    selection = next(
        item
        for item in document["selection"]["included"]
        if item["path"] == target_path
    )
    group_report = next(
        item
        for item in document["groups"][group_id]["input_reports"]
        if item["path"] == target_path
    )
    repeat = next(
        item
        for item in document["physical_acceptance"]["groups"][group_id][
            "repeat_results"
        ]
        if item["report_path"] == target_path
    )
    selection.update(
        {"sha256": raw_sha256, "canonical_sha256": canonical_sha256}
    )
    group_report.update(
        {"sha256": raw_sha256, "canonical_sha256": canonical_sha256}
    )
    repeat.update(
        {
            "report_sha256": raw_sha256,
            "canonical_sha256": canonical_sha256,
        }
    )

    with pytest.raises(ConfigurationError, match="global identity"):
        validate_physical_acceptance_accounting(document, expected_repeats=3)


def test_physical_accounting_requires_original_source_reports(tmp_path):
    paths = _three_physical_acceptance_reports(tmp_path)
    report = analyse_contact_ab(paths, RADIUS_M)
    paths[0].unlink()

    with pytest.raises(ConfigurationError, match="canonical regular report file"):
        validate_physical_acceptance_accounting(report, expected_repeats=3)


def test_physical_accounting_rejects_coordinated_schema_not_applicable_forgery(
    tmp_path,
):
    report = analyse_contact_ab(
        _three_physical_acceptance_reports(tmp_path), RADIUS_M
    )
    document = json.loads(json.dumps(report))
    acceptance = document["physical_acceptance"]
    group_id = acceptance["applicable_groups"][0]
    for item in document["selection"]["included"]:
        item["report_schema_version"] = 2
    for item in document["groups"][group_id]["input_reports"]:
        item["report_schema_version"] = 2
    acceptance["groups"][group_id].update(
        {
            "applicable": False,
            "passed": None,
            "not_applicable_reasons": ["motion_report_schema_not_3"],
            "checks": {},
            "failed_checks": [],
            "repeat_results": [],
        }
    )
    acceptance["applicable_groups"] = []
    acceptance["not_applicable_groups"] = [group_id]
    acceptance["passing_groups"] = []
    acceptance["failed_groups"] = []
    acceptance["all_applicable_groups_passed"] = None

    with pytest.raises(ConfigurationError, match="identity no longer matches"):
        validate_physical_acceptance_accounting(document, expected_repeats=3)


def test_physical_accounting_rejects_coordinated_wheel_fail_to_pass_forgery(
    tmp_path,
):
    paths = _three_physical_acceptance_reports(tmp_path)
    changed = json.loads(paths[1].read_text(encoding="utf-8"))
    joint_name = WHEELS["front_left"]
    _set_schema3_wheel_classification(
        changed["segments"][0], joint_name, "positive", steady=True
    )
    _write(paths[1], changed)
    document = analyse_contact_ab(paths, RADIUS_M)
    acceptance = document["physical_acceptance"]
    group_id = acceptance["applicable_groups"][0]
    group = acceptance["groups"][group_id]
    repeat = next(
        item
        for item in group["repeat_results"]
        if "wheel_direction_contract" in item["failed_checks"]
    )
    check = repeat["checks"]["wheel_direction_contract"]
    observation = check["segments"]["rotate_left_360"]["per_wheel"][
        joint_name
    ]
    observation["direction"] = observation["expected_direction"]
    observation["direction_matches"] = True
    check["segments"]["rotate_left_360"]["all_directions_match"] = True
    check["failed_observations"] = []
    check["passed"] = True
    repeat["failed_checks"] = []
    repeat["passed"] = True
    group["checks"]["wheel_direction_contract"] = {
        "passed_repeats": 3,
        "failed_repeats": 0,
        "all_repeats_passed": True,
    }
    group["failed_checks"] = []
    group["passed"] = True
    acceptance["passing_groups"] = [group_id]
    acceptance["failed_groups"] = []
    acceptance["all_applicable_groups_passed"] = True

    with pytest.raises(ConfigurationError, match="revalidated source reports"):
        validate_physical_acceptance_accounting(document, expected_repeats=3)


def test_v5_physical_acceptance_requires_ideal_odometry(tmp_path):
    paths = _three_physical_acceptance_reports(
        tmp_path,
        odometry_mode="realistic",
    )

    report = analyse_contact_ab(paths, RADIUS_M)

    acceptance = report["physical_acceptance"]
    group_id = "SimplePlane::simple_plane_only1_v1::legacy_baseline"
    group = acceptance["groups"][group_id]
    assert group["applicable"] is False
    assert group["passed"] is None
    assert group["not_applicable_reasons"] == ["odometry_mode_not_ideal"]
    assert acceptance["all_applicable_groups_passed"] is None


def test_v5_physical_acceptance_requires_three_unique_repeats(tmp_path):
    paths = _three_physical_acceptance_reports(tmp_path)

    report = analyse_contact_ab(paths[:1], RADIUS_M, min_repeats=1)
    validate_physical_acceptance_accounting(report, expected_repeats=1)

    acceptance = report["physical_acceptance"]
    group_id = "SimplePlane::simple_plane_only1_v1::legacy_baseline"
    group = acceptance["groups"][group_id]
    assert group["applicable"] is False
    assert group["passed"] is None
    assert group["repeat_count"] == 1
    assert group["not_applicable_reasons"] == [
        "fewer_than_3_unique_repeats"
    ]
    assert acceptance["all_applicable_groups_passed"] is None


@pytest.mark.parametrize(
    ("field", "value", "failed_check"),
    (
        (
            "stable_duration_sec",
            0.499,
            "stop_config.stable_duration_sec",
        ),
        (
            "linear_velocity_threshold_mps",
            0.0201,
            "stop_config.linear_velocity_threshold_mps",
        ),
        (
            "angular_velocity_threshold_radps",
            0.0501,
            "stop_config.angular_velocity_threshold_radps",
        ),
        (
            "wheel_velocity_threshold_radps",
            0.2001,
            "stop_config.wheel_velocity_threshold_radps",
        ),
    ),
)
def test_v5_physical_acceptance_rejects_unsafe_stop_configuration(
    tmp_path,
    field,
    value,
    failed_check,
):
    paths = _three_physical_acceptance_reports(tmp_path)
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        document["configuration"]["stop"][field] = value
        if field == "wheel_velocity_threshold_radps":
            for segment in document["segments"]:
                segment["wheels"]["steady_state_window"][
                    "classification_deadband_radps"
                ] = value
        _write(path, document)

    report = analyse_contact_ab(paths, RADIUS_M)

    group = next(iter(report["physical_acceptance"]["groups"].values()))
    assert report["analysis_valid"] is True
    assert group["passed"] is False
    assert failed_check in group["failed_checks"]
    assert all(
        failed_check in result["failed_checks"]
        for result in group["repeat_results"]
    )


def test_v5_physical_acceptance_rejects_short_stop_window_in_one_repeat(
    tmp_path,
):
    paths = _three_physical_acceptance_reports(tmp_path)
    changed = json.loads(paths[2].read_text(encoding="utf-8"))
    stopping = changed["segments"][5]["stopping"]
    stopping["confirmed_after_command_sec"] = (
        stopping["stationary_onset_after_command_sec"] + 0.499
    )
    stationary_evidence = stopping["stationary_evidence"]
    stationary_evidence["end_stamp_ns"] = (
        changed["segments"][5]["command"]["end_stamp_ns"] + 549_000_000
    )
    stationary_evidence["observed_duration_sec"] = 0.499
    for stream in stationary_evidence["streams"].values():
        stream["last_sample_stamp_ns"] = stationary_evidence["end_stamp_ns"]
        stream["maximum_inter_sample_gap_sec"] = 0.499
    _write(paths[2], changed)

    report = analyse_contact_ab(paths, RADIUS_M)

    group = next(iter(report["physical_acceptance"]["groups"].values()))
    failed_check = "stop_window.arc_right_5s"
    assert group["passed"] is False
    assert group["failed_checks"] == [failed_check]
    assert [result["passed"] for result in group["repeat_results"]] == [
        True,
        True,
        False,
    ]
    assert group["repeat_results"][2]["failed_checks"] == [failed_check]


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
        segment["pose"]["end"]["yaw_rad"] = segment["yaw"]["change_rad"]
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
    validate_physical_acceptance_accounting(report, expected_repeats=3)

    assert report["schema_version"] == 4
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
    acceptance = report["physical_acceptance"]
    assert set(acceptance["groups"]) == set(report["groups"])
    assert len(acceptance["applicable_groups"]) == 6
    assert len(acceptance["not_applicable_groups"]) == 12
    assert all(
        group_id.startswith("SimplePlane::simple_plane_only1_v1::")
        for group_id in acceptance["applicable_groups"]
    )


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
        ("report_float", "schema_version must be integer 1, 2, or 3"),
        ("report_bool", "schema_version must be integer 1, 2, or 3"),
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
        _set_pose_displacements(segment, lateral=value)
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
