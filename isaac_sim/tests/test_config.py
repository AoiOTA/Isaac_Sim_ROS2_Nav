from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from isaac_sim.src.config import ConfigError, load_project_config
from isaac_sim.src.robot.articulation_runtime import (
    ArticulationRuntime,
    ArticulationRuntimeError,
    load_articulation_physics_config,
)
from isaac_sim.src.sensors.sensor_factory import _load_lidar
from isaac_sim.src.stage.physics_setup import FramePacer, pacing_plan
from isaac_sim.graphs.sensor_graph import lidar_graph_spec


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "isaac_sim/configs/project.yaml"
ASSET_ROOT = Path("/home/lyb/isaacsim_assets/Assets/Isaac/6.0")


class _SolverValues:
    def __init__(self, value: int):
        self._values = [value]

    def numpy(self):
        return self

    def reshape(self, shape):
        assert shape in {-1, (-1,)}
        return self._values


class _FakeArticulation:
    def __init__(self, readback: tuple[int, int]):
        self.readback = readback

    def set_solver_iteration_counts(self, position, velocity):
        self.requested = (position, velocity)

    def get_solver_iteration_counts(self):
        return _SolverValues(self.readback[0]), _SolverValues(self.readback[1])

    def set_sleep_thresholds(self, values):
        pass

    def set_stabilization_thresholds(self, values):
        pass

    def set_dof_friction_properties(self, **values):
        pass


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
    assert config.environment.identifier == "Warehouse"
    assert config.environment.composition == "sublayer"
    assert config.environment.ground_colliders.required_prim_paths == (
        "/Root/GroundPlane/CollisionPlane",
    )
    assert config.environment.ground_colliders.semantic_classes == (
        "floor_decal",
    )
    assert config.environment.ground_colliders.expected_enabled_count == 32
    assert config.files.contact_profile == (
        ROOT / "isaac_sim/configs/physics/legacy_baseline.yaml"
    )
    assert config.simulation.expected_physics_scene == "/PhysicsScene"
    assert config.simulation.structure_tf_source == "isaac"
    assert config.simulation.pacing_mode == "realtime"
    assert config.simulation.target_realtime_factor == pytest.approx(1.0)
    assert config.robot.default_prim == "jackal"
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
    lidar = _load_lidar(config.files.lidar)
    assert tuple(lidar["render_product_resolution"]) == (1.0, 1.0)
    lidar_values = dict(lidar_graph_spec(config, "/Render/Test").values)
    assert lidar_values["PointCloudConfig.inputs:outputIntensity"] is False
    assert lidar_values["PointCloudConfig.inputs:outputTimestamp"] is False
    assert lidar_values["PointCloudPublisher.inputs:fullScan"] is False


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
            ISAAC_NAV__ROS2__DOMAIN_ID="42",
        ),
    )
    assert config.simulation.odometry_mode == "realistic"
    assert config.simulation.structure_tf_source == "rsp"
    assert config.simulation.headless is True
    assert config.simulation.max_frames == 17
    assert config.simulation.pacing_mode == "unbounded"
    assert config.simulation.target_realtime_factor == pytest.approx(1.25)
    assert config.ros2.domain_id == 42


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


def test_frame_pacer_limits_rate_without_catch_up_bursts():
    now = [0.0]
    sleeps: list[float] = []

    def clock() -> float:
        return now[0]

    def sleep(duration: float) -> None:
        sleeps.append(duration)
        now[0] += duration

    pacer = FramePacer(60.0, clock=clock, sleep=sleep)
    pacer.wait_after_frame()
    assert sleeps == [pytest.approx(1.0 / 60.0)]

    # A frame that exceeded its deadline resets cadence instead of issuing a
    # burst of unpaced physics frames.
    now[0] += 1.0
    pacer.wait_after_frame()
    assert len(sleeps) == 1
    pacer.wait_after_frame()
    assert sleeps[-1] == pytest.approx(1.0 / 60.0)


def test_unbounded_frame_pacer_never_sleeps():
    sleeps: list[float] = []
    FramePacer(None, sleep=sleeps.append).wait_after_frame()
    assert sleeps == []


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


def test_unknown_override_is_rejected():
    with pytest.raises(ConfigError, match="unknown key"):
        load_project_config(
            CONFIG,
            _environment(ISAAC_NAV__SIMULATION__TYPO="true"),
        )


@pytest.mark.parametrize("value", ["", "bad/id", "-leading"])
def test_environment_id_must_be_path_safe(value):
    with pytest.raises(ConfigError, match="environment.id.*path-safe"):
        load_project_config(
            CONFIG,
            _environment(ISAAC_NAV__ENVIRONMENT__ID=value),
        )


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("REQUIRED_PRIM_PATHS", "[]", "non-empty list"),
        ("REQUIRED_PRIM_PATHS", "[relative]", "absolute USD prim path"),
        ("SEMANTIC_CLASSES", "[bad/id]", "path-safe identifiers"),
        ("EXPECTED_ENABLED_COUNT", "true", "integer no smaller"),
        ("EXPECTED_ENABLED_COUNT", "0", "integer no smaller"),
    ],
)
def test_ground_collider_resolver_schema_is_strict(field, value, message):
    with pytest.raises(ConfigError, match=message):
        load_project_config(
            CONFIG,
            _environment(
                **{
                    "ISAAC_NAV__ENVIRONMENT__GROUND_COLLIDERS__"
                    f"{field}": value
                }
            ),
        )


def test_simple_plane_uses_an_independent_project_stage():
    warehouse = load_project_config(CONFIG, _environment())
    simple = load_project_config(
        ROOT / "isaac_sim/configs/simple_plane.project.yaml",
        _environment(),
    )
    assert simple.environment.identifier == "SimplePlane"
    assert simple.environment.project_stage.is_file()
    assert simple.environment.project_stage != warehouse.environment.project_stage
    assert simple.environment.ground_colliders.expected_enabled_count == 1


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


@pytest.mark.parametrize(
    ("field", "original", "value"),
    [
        ("solver_position_iterations", "32", "0"),
        ("solver_position_iterations", "32", "256"),
        ("solver_velocity_iterations", "4", "-1"),
        ("solver_velocity_iterations", "4", "4.0"),
        ("solver_velocity_iterations", "4", "true"),
    ],
)
def test_solver_iteration_counts_require_schema_range(
    tmp_path, field, original, value
):
    source = (ROOT / "isaac_sim/configs/robots/jackal.yaml").read_text()
    source = source.replace(
        f"{field}: {original}",
        f"{field}: {value}",
        1,
    )
    candidate = tmp_path / "robot.yaml"
    candidate.write_text(source)
    with pytest.raises(
        ArticulationRuntimeError,
        match=f"{field}.*integer in.*1, 255",
    ):
        load_articulation_physics_config(candidate)


@pytest.mark.parametrize("readback", [(32, 4), (32, 16)])
def test_runtime_solver_configuration_requires_matching_readback(readback):
    settings = load_articulation_physics_config(
        ROOT / "isaac_sim/configs/robots/jackal.yaml"
    )
    runtime = ArticulationRuntime("/World/Robot", "/World/Robot/base", None)
    runtime._articulation = _FakeArticulation(readback)
    if readback == (32, 4):
        assert runtime.configure_stability(settings) == (32, 4)
    else:
        with pytest.raises(
            ArticulationRuntimeError,
            match="solver readback does not match",
        ):
            runtime.configure_stability(settings)
