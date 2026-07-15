from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from isaac_sim.apps.wheel_direction_diagnostic import _contact_slice
from isaac_sim.src.robot.wheel_direction_diagnostic import (
    TrialObservation,
    WheelDirectionDiagnosticError,
    center_of_mass_world,
    contact_point_velocity_world,
    evaluate_trial_set,
    load_wheel_direction_config,
    rotate_local_to_world,
    rotate_world_to_local,
    spin_contact_velocity_world,
    summarize_trial,
    write_json_atomic,
)


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "isaac_sim/configs/diagnostics/wheel_direction.yaml"
WHEELS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)


def test_runtime_app_uses_physics_material_purpose_and_enabled_colliders_only():
    source = (
        ROOT / "isaac_sim/apps/wheel_direction_diagnostic.py"
    ).read_text(encoding="utf-8")

    material_reader = source.split("def _physics_material_snapshot", 1)[1].split(
        "def _wheel_collider_snapshot", 1
    )[0]
    assert 'ComputeBoundMaterial("physics")' in material_reader
    assert "ComputeBoundMaterial()" in material_reader
    assert material_reader.index('ComputeBoundMaterial("physics")') < material_reader.index(
        "ComputeBoundMaterial()"
    )

    collider_reader = source.split("def _wheel_collider_snapshot", 1)[1].split(
        "def _to_float_tuple", 1
    )[0]
    assert "prim.IsActive()" in collider_reader
    assert "prim.HasAPI(UsdPhysics.CollisionAPI)" in collider_reader
    assert "GetCollisionEnabledAttr().Get()" in collider_reader
    assert "collision_enabled is False" in collider_reader


def test_runtime_authors_contact_reports_before_physx_and_filters_all_ground():
    source = (
        ROOT / "isaac_sim/apps/wheel_direction_diagnostic.py"
    ).read_text(encoding="utf-8")
    author = source.split("def _author_wheel_contact_reports", 1)[1].split(
        "def _validate_ground_collision", 1
    )[0]
    assert "stage.GetSessionLayer()" in author
    assert "PhysxSchema.PhysxContactReportAPI.Apply(prim)" in author
    assert "CreateThresholdAttr()" in author
    assert "threshold.Set(0.0)" in author

    runtime = source.split("def run(", 1)[1]
    assert runtime.index("_author_wheel_contact_reports(") < runtime.index(
        "PhysicsSetup(config.simulation).apply(stage, app)"
    )
    assert "ground_paths = resolve_ground_colliders(stage, config)" in runtime
    assert "reset_contact_probe = WheelGroundContactProbe(" in runtime
    assert "ground_filter_paths=ground_paths" in runtime
    assert "reset_contact_probe.initialize(app)" in runtime
    assert "wheel_view = reset_contact_probe.view" in runtime
    assert "reset_contact_probe.provenance_snapshot(" in runtime
    cleanup = runtime.split("finally:", 1)[1]
    assert cleanup.index("write_json_atomic(output_path, report)") < cleanup.index(
        "app.close("
    )
    assert 'exit_code=0 if report["result"] == "success" else 1' in cleanup

    sampler = source.split("def _sample(", 1)[1].split(
        "def _set_single_wheel_target", 1
    )[0]
    assert "matrix_normal_force_world" in sampler
    assert "vector_subtract(" in sampler
    assert "normal_force_world, matrix_normal_force_world" in sampler
    assert '"normal_force_matrix_trial_frame_n"' in sampler
    assert '"normal_force_reconstruction_error_n"' in sampler
    assert '"net_contact_force_trial_frame_n"' not in sampler


def test_contact_slice_concatenates_every_filter_for_one_sensor():
    values = np.asarray([[10.0], [20.0], [30.0], [40.0], [50.0]])
    counts = np.asarray([[1, 0, 2], [0, 1, 1]])
    starts = np.asarray([[0, 1, 3], [0, 1, 2]])

    selected, count = _contact_slice(values, counts, starts, 0)

    assert count == 3
    assert selected.tolist() == [[10.0], [40.0], [50.0]]


def test_default_direction_config_is_strict_and_warehouse_bound():
    config = load_wheel_direction_config(CONFIG)

    assert config.environment_id == "Warehouse"
    assert config.ground_collision_prim == "/Root/GroundPlane/CollisionPlane"
    assert config.protocol.command_rad_s == pytest.approx(1.0)
    assert config.protocol.max_contact_count == 128
    assert config.thresholds.active_rate_min_rad_s == pytest.approx(0.75)
    assert (
        config.thresholds.inactive_rate_p95_advisory_max_rad_s
        == pytest.approx(0.10)
    )
    assert config.thresholds.spin_velocity_opposition_min_m_s == pytest.approx(
        0.04
    )


def test_direction_config_rejects_unknown_and_unsafe_values(tmp_path: Path):
    source = CONFIG.read_text(encoding="utf-8")
    unknown = tmp_path / "unknown.yaml"
    unknown.write_text(source + "unknown: true\n", encoding="utf-8")
    with pytest.raises(ValueError, match="unknown"):
        load_wheel_direction_config(unknown)

    unsafe = tmp_path / "unsafe.yaml"
    unsafe.write_text(
        source.replace("environment_id: Warehouse", "environment_id: ../Warehouse"),
        encoding="utf-8",
    )
    with pytest.raises(WheelDirectionDiagnosticError, match="path-safe"):
        load_wheel_direction_config(unsafe)


def test_quaternion_rotation_and_local_com_transform_are_explicit():
    # +90 degrees about Z maps local +X to world +Y.
    q = (2**-0.5, 0.0, 0.0, 2**-0.5)
    world = rotate_local_to_world((1.0, 0.0, 0.0), q)
    assert world == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
    assert rotate_world_to_local(world, q) == pytest.approx(
        (1.0, 0.0, 0.0), abs=1e-12
    )
    assert center_of_mass_world((4.0, 2.0, 1.0), q, (1.0, 0.0, 0.0)) == pytest.approx(
        (4.0, 3.0, 1.0), abs=1e-12
    )


def test_positive_wheel_spin_has_negative_bottom_surface_velocity():
    # Positive wheel rotation is +Y; the bottom contact radius is -Z.
    # omega x radius therefore points -X while ground friction drives +X.
    surface = contact_point_velocity_world(
        (0.02, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, -0.098),
        (0.0, 0.0, 0.0),
    )
    spin = spin_contact_velocity_world(
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, -0.098),
        (0.0, 0.0, 0.0),
    )

    assert spin == pytest.approx((-0.098, 0.0, 0.0))
    assert surface == pytest.approx((-0.078, 0.0, 0.0))


def _observation(
    *,
    phase: str,
    step: int,
    wheel_index: int,
    command: float,
    position_x: float,
    velocity_x: float,
    wrong_spin: bool = False,
    inactive_rate: float = 0.0,
) -> TrialObservation:
    sign = 1.0 if command > 0.0 else -1.0
    targets = [0.0] * 4
    rates = [inactive_rate] * 4
    if phase == "drive":
        targets[wheel_index] = command
        rates[wheel_index] = sign * 0.95
    else:
        rates = [0.0] * 4
    spin = sign * 0.08 if wrong_spin else -sign * 0.08
    return TrialObservation(
        phase=phase,
        step_index=step,
        simulation_time_s=step / 60.0,
        joint_targets_rad_s=tuple(targets),  # type: ignore[arg-type]
        joint_rates_rad_s=tuple(rates),  # type: ignore[arg-type]
        active_contact_count=1,
        active_normal_force_n=10.0,
        active_spin_velocity_x_m_s=spin,
        active_surface_velocity_x_m_s=0.005 * sign,
        active_friction_force_x_n=0.2 * sign,
        normal_force_consistency_error_n=0.01,
        base_position_x_m=position_x,
        base_velocity_x_m_s=velocity_x,
        base_acceleration_x_m_s2=(0.1 * sign if phase == "drive" else 0.0),
    )


def _passing_trial(wheel_index: int, command: float):
    sign = 1.0 if command > 0.0 else -1.0
    observations = [
        _observation(
            phase="baseline",
            step=step,
            wheel_index=wheel_index,
            command=command,
            position_x=0.0,
            velocity_x=0.0,
        )
        for step in range(3)
    ]
    observations.extend(
        _observation(
            phase="drive",
            step=3 + step,
            wheel_index=wheel_index,
            command=command,
            position_x=sign * (step + 1) * 0.001,
            velocity_x=sign * 0.01,
        )
        for step in range(4)
    )
    config = load_wheel_direction_config(CONFIG)
    return summarize_trial(
        observations,
        wheel_name=WHEELS[wheel_index],
        wheel_index=wheel_index,
        command_rad_s=command,
        physics_dt_s=1.0 / 60.0,
        thresholds=config.thresholds,
    )


@pytest.mark.parametrize("command", [1.0, -1.0])
@pytest.mark.parametrize("wheel_index", range(4))
def test_trial_summary_accepts_mirrored_single_wheel_direction(
    wheel_index: int, command: float
):
    summary = _passing_trial(wheel_index, command)

    assert summary["passed"] is True
    assert all(summary["gates"].values())
    assert summary["expected_body_direction"] == (
        "+X" if command > 0.0 else "-X"
    )


def test_trial_summary_rejects_wrong_joint_axis_spin_direction():
    config = load_wheel_direction_config(CONFIG)
    observations = [
        _observation(
            phase="baseline",
            step=0,
            wheel_index=0,
            command=1.0,
            position_x=0.0,
            velocity_x=0.0,
        )
    ] + [
        _observation(
            phase="drive",
            step=step + 1,
            wheel_index=0,
            command=1.0,
            position_x=0.001 * (step + 1),
            velocity_x=0.01,
            wrong_spin=True,
        )
        for step in range(4)
    ]

    summary = summarize_trial(
        observations,
        wheel_name=WHEELS[0],
        wheel_index=0,
        command_rad_s=1.0,
        physics_dt_s=1.0 / 60.0,
        thresholds=config.thresholds,
    )

    assert summary["passed"] is False
    assert summary["gates"]["spin_opposes_forward"] is False


def test_passive_inactive_wheel_motion_is_advisory_not_direction_failure():
    config = load_wheel_direction_config(CONFIG)
    observations = [
        _observation(
            phase="baseline",
            step=0,
            wheel_index=0,
            command=1.0,
            position_x=0.0,
            velocity_x=0.0,
        )
    ] + [
        _observation(
            phase="drive",
            step=step + 1,
            wheel_index=0,
            command=1.0,
            position_x=0.001 * (step + 1),
            velocity_x=0.01,
            inactive_rate=0.2,
        )
        for step in range(4)
    ]

    summary = summarize_trial(
        observations,
        wheel_name=WHEELS[0],
        wheel_index=0,
        command_rad_s=1.0,
        physics_dt_s=1.0 / 60.0,
        thresholds=config.thresholds,
    )

    advisory = summary["advisories"]["inactive_wheel_motion"]
    assert summary["passed"] is True
    assert advisory["within_advisory_limit"] is False
    assert advisory["p95_max_rad_s"] == pytest.approx(0.2)
    assert advisory["ratio_to_active_rate_median"] == pytest.approx(0.2 / 0.95)
    assert summary["warnings"]


def test_nonzero_inactive_wheel_target_remains_a_hard_failure():
    config = load_wheel_direction_config(CONFIG)
    observations = [
        _observation(
            phase="baseline",
            step=0,
            wheel_index=0,
            command=1.0,
            position_x=0.0,
            velocity_x=0.0,
        )
    ] + [
        _observation(
            phase="drive",
            step=step + 1,
            wheel_index=0,
            command=1.0,
            position_x=0.001 * (step + 1),
            velocity_x=0.01,
        )
        for step in range(4)
    ]
    observations[1] = replace(
        observations[1],
        joint_targets_rad_s=(1.0, 0.2, 0.0, 0.0),
    )

    summary = summarize_trial(
        observations,
        wheel_name=WHEELS[0],
        wheel_index=0,
        command_rad_s=1.0,
        physics_dt_s=1.0 / 60.0,
        thresholds=config.thresholds,
    )

    assert summary["passed"] is False
    assert summary["gates"]["target_readback"] is False


def test_normal_force_consistency_does_not_use_friction_as_tolerance_scale():
    config = load_wheel_direction_config(CONFIG)
    observations = [
        _observation(
            phase="baseline",
            step=0,
            wheel_index=0,
            command=1.0,
            position_x=0.0,
            velocity_x=0.0,
        )
    ] + [
        replace(
            _observation(
                phase="drive",
                step=step + 1,
                wheel_index=0,
                command=1.0,
                position_x=0.001 * (step + 1),
                velocity_x=0.01,
            ),
            active_friction_force_x_n=100.0,
            normal_force_consistency_error_n=3.0,
        )
        for step in range(4)
    ]

    summary = summarize_trial(
        observations,
        wheel_name=WHEELS[0],
        wheel_index=0,
        command_rad_s=1.0,
        physics_dt_s=1.0 / 60.0,
        thresholds=config.thresholds,
    )

    assert summary["passed"] is False
    assert summary["gates"]["normal_force_api_consistency"] is False


def test_complete_eight_trial_matrix_requires_every_wheel_and_sign():
    config = load_wheel_direction_config(CONFIG)
    summaries = [
        _passing_trial(wheel_index, command)
        for wheel_index in range(4)
        for command in (1.0, -1.0)
    ]

    result = evaluate_trial_set(
        summaries,
        wheel_order=WHEELS,
        thresholds=config.thresholds,
    )

    assert result["passed"] is True
    assert result["failed_trials"] == []
    assert result["positive_commands_correspond_to_body_positive_x"] is True

    with pytest.raises(WheelDirectionDiagnosticError, match="missing"):
        evaluate_trial_set(
            summaries[:-1],
            wheel_order=WHEELS,
            thresholds=config.thresholds,
        )


def test_symmetry_is_advisory_not_a_direction_gate():
    config = load_wheel_direction_config(CONFIG)
    summaries = [
        _passing_trial(wheel_index, command)
        for wheel_index in range(4)
        for command in (1.0, -1.0)
    ]
    changed = dict(summaries[0])
    changed_metrics = dict(changed["metrics"])
    changed_metrics["friction_force_signed_median_n"] = 10.0
    changed["metrics"] = changed_metrics
    summaries[0] = changed

    result = evaluate_trial_set(
        summaries,
        wheel_order=WHEELS,
        thresholds=config.thresholds,
    )

    assert result["passed"] is True
    assert result["warnings"]


def test_atomic_report_replaces_existing_json_and_rejects_nonfinite(
    tmp_path: Path,
):
    output = tmp_path / "wheel.json"
    output.write_text("partial", encoding="utf-8")

    write_json_atomic(output, {"schema_version": 1, "result": "success"})

    assert json.loads(output.read_text(encoding="utf-8")) == {
        "schema_version": 1,
        "result": "success",
    }
    assert list(tmp_path.glob(".wheel.json.*.tmp")) == []

    with pytest.raises(WheelDirectionDiagnosticError, match="NaN"):
        write_json_atomic(output, {"value": float("nan")})
    assert json.loads(output.read_text(encoding="utf-8"))["result"] == "success"
