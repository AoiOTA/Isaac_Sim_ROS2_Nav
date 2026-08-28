from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from isaac_sim.src.config import ConfigError, load_project_config
from isaac_sim.src.robot.articulation_runtime import (
    ArticulationRuntimeError,
    load_articulation_physics_config,
)
from isaac_sim.src.sensors.sensor_factory import SensorConfigError, _load_lidar
from isaac_sim.src.yaml_utils import YamlConfigError
from isaac_sim.src.stage.physics_setup import PhysicsSetup, pacing_plan
from isaac_sim.graphs.sensor_graph import lidar_graph_spec


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "isaac_sim/configs/project.yaml"
ASSET_ROOT = Path("/home/lyb/isaacsim_assets/Assets/Isaac/6.0")


def _environment(**updates: str) -> dict[str, str]:
    values = {
        "PROJECT_ROOT": str(ROOT),
        "ISAAC_ASSET_ROOT": str(ASSET_ROOT),
    }
    values.update(updates)
    return values


def test_default_project_contract_loads_strictly():
    config = load_project_config(CONFIG, _environment())
    assert config.schema_version == 1
    assert config.environment.composition == "sublayer"
    assert config.simulation.expected_physics_scene == "/PhysicsScene"
    assert config.simulation.structure_tf_source == "isaac"
    assert config.simulation.pacing_mode == "realtime"
    assert config.simulation.target_realtime_factor == pytest.approx(1.0)
    assert config.simulation.stage_readiness_timeout_s == pytest.approx(420.0)
    assert config.robot.default_prim == "jackal"
    assert config.third_person_camera.enabled is True
    assert config.third_person_camera.prim_name == "third_person_camera"
    assert config.third_person_camera.distance_m == pytest.approx(3.2)
    assert config.third_person_camera.height_m == pytest.approx(2.2)
    assert config.robot.wheel_joints == (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    )
    assert config.ros2.rmw_implementation == "rmw_fastrtps_cpp"
    stability = load_articulation_physics_config(config.files.robot)
    assert stability.solver_position_iterations == 32
    assert stability.solver_velocity_iterations == 4
    assert stability.sleep_threshold == pytest.approx(0.005)
    assert stability.stabilization_threshold == pytest.approx(0.001)
    assert stability.wheel_static_friction_effort == pytest.approx(0.0)
    assert stability.wheel_dynamic_friction_effort == pytest.approx(0.0)
    assert stability.wheel_viscous_friction_coefficient == pytest.approx(0.0)
    assert stability.idle_brake_command_timeout_sec == pytest.approx(0.25)
    assert stability.motion_assist_enabled is True
    assert stability.motion_assist_command_timeout_sec == pytest.approx(0.25)
    assert stability.motion_assist_max_linear_acceleration \
        == pytest.approx(6.0)
    assert stability.motion_assist_max_angular_acceleration \
        == pytest.approx(30.0)


def test_lidar_sensor_frame_contract_rejects_unknown_and_world_values(tmp_path):
    source = ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml"
    document = yaml.safe_load(source.read_text(encoding="utf-8"))

    document["legacy_world_frame"] = True
    candidate = tmp_path / "lidar.yaml"
    candidate.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(YamlConfigError, match="unknown lidar config keys"):
        _load_lidar(candidate)

    document.pop("legacy_world_frame")
    document["frame_id"] = "world"
    candidate.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(SensorConfigError, match="frame_id must be rtx_lidar"):
        _load_lidar(candidate)

    document["frame_id"] = "rtx_lidar"
    document["output_frame"] = "WORLD"
    candidate.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(SensorConfigError, match="output_frame must be SENSOR"):
        _load_lidar(candidate)

    document["output_frame"] = "SENSOR"
    document["ros_frame_parent"] = "base_link"
    candidate.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(SensorConfigError, match="parent must be lidar_link"):
        _load_lidar(candidate)

    document["ros_frame_parent"] = "lidar_link"
    document["ros_frame_translation"] = [0.0, 0.1, 0.0]
    candidate.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(SensorConfigError, match="translation must be zero"):
        _load_lidar(candidate)

    document["ros_frame_translation"] = [0.0, 0.0, 0.0]
    document["ros_frame_rotation_xyzw"] = [0.0, 0.0, -0.7071067811865475, 0.7071067811865476]
    candidate.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(SensorConfigError, match=r"rotation must be \+90 degrees"):
        _load_lidar(candidate)


def test_nested_environment_overrides_are_typed():
    config = load_project_config(
        CONFIG,
        _environment(
            ISAAC_NAV__SIMULATION__ODOMETRY_MODE="realistic",
            ISAAC_NAV__SIMULATION__STRUCTURE_TF_SOURCE="rsp",
            ISAAC_NAV__SIMULATION__HEADLESS="true",
            ISAAC_NAV__SIMULATION__MAX_FRAMES="17",
            ISAAC_NAV__SIMULATION__PACING_MODE="unbounded",
            ISAAC_NAV__SIMULATION__TARGET_REALTIME_FACTOR="1.25",
            ISAAC_NAV__SIMULATION__RENDERING_HZ="30",
            ISAAC_NAV__SIMULATION__STAGE_READINESS_TIMEOUT_S="12.5",
            ISAAC_NAV__ROS2__DOMAIN_ID="42",
            ISAAC_NAV__THIRD_PERSON_CAMERA__ENABLED="false",
            ISAAC_NAV__THIRD_PERSON_CAMERA__DISTANCE_M="4.75",
        ),
    )
    assert config.simulation.odometry_mode == "realistic"
    assert config.simulation.structure_tf_source == "rsp"
    assert config.simulation.headless is True
    assert config.simulation.max_frames == 17
    assert config.simulation.pacing_mode == "unbounded"
    assert config.simulation.target_realtime_factor == pytest.approx(1.25)
    assert config.simulation.rendering_hz == pytest.approx(30.0)
    assert config.simulation.stage_readiness_timeout_s == pytest.approx(12.5)
    assert config.ros2.domain_id == 42
    assert config.third_person_camera.enabled is False
    assert config.third_person_camera.distance_m == pytest.approx(4.75)


def test_mixed_compute_amcl_mode_parses_only_with_isaac_structure_tf():
    config = load_project_config(
        CONFIG,
        _environment(ISAAC_NAV__SIMULATION__ODOMETRY_MODE="mixed"),
    )
    assert config.simulation.odometry_mode == "mixed"
    assert config.simulation.structure_tf_source == "isaac"

    with pytest.raises(ConfigError, match="mixed odometry requires"):
        load_project_config(
            CONFIG,
            _environment(
                ISAAC_NAV__SIMULATION__ODOMETRY_MODE="mixed",
                ISAAC_NAV__SIMULATION__STRUCTURE_TF_SOURCE="rsp",
            ),
        )


def test_invalid_third_person_camera_settings_are_rejected():
    with pytest.raises(ConfigError, match="distance_m must be a positive"):
        load_project_config(
            CONFIG,
            _environment(
                ISAAC_NAV__THIRD_PERSON_CAMERA__DISTANCE_M="-1.0"
            ),
        )
    with pytest.raises(ConfigError, match="height_m must be a non-negative"):
        load_project_config(
            CONFIG,
            _environment(ISAAC_NAV__THIRD_PERSON_CAMERA__HEIGHT_M="-0.1"),
        )


def test_realtime_and_unbounded_pacing_keep_fixed_simulation_dt():
    config = load_project_config(CONFIG, _environment())
    realtime = pacing_plan(config.simulation)
    unbounded = pacing_plan(replace(
        config.simulation,
        pacing_mode="unbounded",
        target_realtime_factor=2.0,
    ))

    assert realtime.timeline_hz == 60.0
    assert realtime.wall_loop_hz == 60.0
    assert realtime.target_realtime_factor == 1.0
    assert unbounded.timeline_hz == 60.0
    assert unbounded.wall_loop_hz is None
    assert PhysicsSetup(config.simulation).plan == realtime


@pytest.mark.parametrize(
    ("name", "value", "message"),
    [
        ("PACING_MODE", "turbo", "pacing_mode"),
        ("TARGET_REALTIME_FACTOR", "0", "positive"),
        ("TARGET_REALTIME_FACTOR", ".nan", "positive"),
    ],
)
def test_invalid_pacing_configuration_fails_before_kit(name, value, message):
    with pytest.raises(ConfigError, match=message):
        load_project_config(
            CONFIG,
            _environment(**{f"ISAAC_NAV__SIMULATION__{name}": value}),
        )


@pytest.mark.parametrize("value", ["0", "-1", ".inf", ".nan"])
def test_invalid_stage_readiness_timeout_fails_before_kit(value):
    with pytest.raises(ConfigError, match="stage_readiness_timeout_s.*positive"):
        load_project_config(
            CONFIG,
            _environment(
                ISAAC_NAV__SIMULATION__STAGE_READINESS_TIMEOUT_S=value
            ),
        )


def test_unknown_override_is_rejected():
    with pytest.raises(ConfigError, match="unknown key"):
        load_project_config(
            CONFIG,
            _environment(ISAAC_NAV__SIMULATION__TYPO="true"),
        )


def test_ideal_rsp_tf_ownership_is_rejected():
    with pytest.raises(ConfigError, match="ideal odometry requires"):
        load_project_config(
            CONFIG,
            _environment(ISAAC_NAV__SIMULATION__STRUCTURE_TF_SOURCE="rsp"),
        )


def test_custom_project_template_requires_real_asset_inputs():
    template = ROOT / "isaac_sim/configs/custom_robot.project.yaml"
    with pytest.raises(ConfigError, match="CUSTOM_ROBOT_USD"):
        load_project_config(template, _environment())


def test_custom_robot_runtime_template_uses_the_live_schema():
    custom = ROOT / "isaac_sim/configs/robots/custom_robot.yaml"
    with pytest.raises(ValueError, match="robot.controller.*must be numeric"):
        from isaac_sim.graphs.control_graph import load_controller_config

        load_controller_config(custom)
    with pytest.raises(
        (ValueError, ArticulationRuntimeError),
        match=(
            "robot.physics.*"
            "(must be numeric|must be an integer|must be boolean)"
        ),
    ):
        load_articulation_physics_config(custom)
