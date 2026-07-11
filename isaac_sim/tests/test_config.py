from __future__ import annotations

from pathlib import Path

import pytest

from isaac_sim.src.config import ConfigError, load_project_config
from isaac_sim.src.robot.articulation_runtime import (
    load_articulation_physics_config,
)


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
    assert config.robot.default_prim == "jackal"
    assert config.robot.wheel_joints == (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    )
    assert config.ros2.rmw_implementation == "rmw_fastrtps_cpp"
    stability = load_articulation_physics_config(config.files.robot)
    assert stability.sleep_threshold == pytest.approx(0.005)
    assert stability.stabilization_threshold == pytest.approx(0.001)
    assert stability.wheel_static_friction_effort == pytest.approx(0.0)
    assert stability.wheel_dynamic_friction_effort == pytest.approx(0.0)
    assert stability.wheel_viscous_friction_coefficient == pytest.approx(0.0)
    assert stability.idle_brake_command_timeout_sec == pytest.approx(0.25)


def test_nested_environment_overrides_are_typed():
    config = load_project_config(
        CONFIG,
        _environment(
            ISAAC_NAV__SIMULATION__ODOMETRY_MODE="realistic",
            ISAAC_NAV__SIMULATION__STRUCTURE_TF_SOURCE="rsp",
            ISAAC_NAV__SIMULATION__HEADLESS="true",
            ISAAC_NAV__SIMULATION__MAX_FRAMES="17",
            ISAAC_NAV__ROS2__DOMAIN_ID="42",
        ),
    )
    assert config.simulation.odometry_mode == "realistic"
    assert config.simulation.structure_tf_source == "rsp"
    assert config.simulation.headless is True
    assert config.simulation.max_frames == 17
    assert config.ros2.domain_id == 42


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
    with pytest.raises(ValueError, match="robot.physics.*must be numeric"):
        load_articulation_physics_config(custom)
