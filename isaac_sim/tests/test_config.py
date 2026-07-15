from __future__ import annotations

from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
import sys
from types import ModuleType

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


class _DofValues:
    def __init__(self, values):
        self._values = [list(values)]

    def numpy(self):
        return self._values


class _FakeArticulation:
    def __init__(
        self,
        readback: tuple[int, int],
        dof_positions=(0.1, -0.2, 0.3, -0.4),
    ):
        self.readback = readback
        self.num_dofs = len(dof_positions)
        self.dof_positions = tuple(dof_positions)
        self.dof_position_updates = []
        self.dof_velocities = (5.0,) * self.num_dofs
        self.dof_velocity_targets = (6.0,) * self.num_dofs
        self.dof_efforts = (7.0,) * self.num_dofs
        self.linear_velocity = (1.0, 2.0, 3.0)
        self.angular_velocity = (4.0, 5.0, 6.0)

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

    def get_dof_positions(self):
        return _DofValues(self.dof_positions)

    def set_dof_positions(self, values):
        self.dof_position_updates.append(values)
        self.dof_positions = tuple(values[0])

    def get_dof_velocities(self):
        return _DofValues(self.dof_velocities)

    def set_dof_velocities(self, values):
        self.dof_velocities = tuple(values[0])

    def get_dof_velocity_targets(self):
        return _DofValues(self.dof_velocity_targets)

    def set_dof_velocity_targets(self, values):
        self.dof_velocity_targets = tuple(values[0])

    def get_dof_efforts(self):
        return _DofValues(self.dof_efforts)

    def set_dof_efforts(self, values):
        self.dof_efforts = tuple(values[0])

    def get_velocities(self):
        return (
            _DofValues(self.linear_velocity),
            _DofValues(self.angular_velocity),
        )

    def set_velocities(self, *, linear_velocities, angular_velocities):
        self.linear_velocity = tuple(linear_velocities[0])
        self.angular_velocity = tuple(angular_velocities[0])


def _environment(**updates: str) -> dict[str, str]:
    values = {
        "PROJECT_ROOT": str(ROOT),
        "ISAAC_ASSET_ROOT": str(ASSET_ROOT),
    }
    values.update(updates)
    return values


def test_default_project_contract_loads_strictly():
    config = load_project_config(CONFIG, _environment())
    assert config.schema_version == 2
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
    assert config.simulation.reset_strategy.schema_version == 1
    assert config.simulation.reset_strategy.identifier == "pose_restore_v1"
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
            ISAAC_NAV__SIMULATION__RESET_STRATEGY__ID=(
                "separate_recontact_0p20m_1step_v1"
            ),
            ISAAC_NAV__ROS2__DOMAIN_ID="42",
        ),
    )
    assert config.simulation.odometry_mode == "realistic"
    assert config.simulation.structure_tf_source == "rsp"
    assert config.simulation.headless is True
    assert config.simulation.max_frames == 17
    assert config.simulation.pacing_mode == "unbounded"
    assert config.simulation.target_realtime_factor == pytest.approx(1.25)
    assert config.simulation.reset_strategy.identifier == (
        "separate_recontact_0p20m_1step_v1"
    )
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


@pytest.mark.parametrize(
    ("strategy_schema", "strategy_id", "message"),
    [
        (2, "pose_restore_v1", "reset_strategy.schema_version must be 1"),
        (True, "pose_restore_v1", "reset_strategy.schema_version must be 1"),
        (1, "unknown", "reset_strategy.id must be one of"),
        (1, 7, "reset_strategy.id must be one of"),
    ],
)
def test_reset_strategy_mapping_is_strict(
    tmp_path, strategy_schema, strategy_id, message
):
    source = CONFIG.read_text(encoding="utf-8")
    if source.startswith("schema_version: 1\n"):
        source = source.replace("schema_version: 1", "schema_version: 2", 1)
    valid_strategy = (
        "  reset_strategy:\n"
        "    schema_version: 1\n"
        "    id: pose_restore_v1\n"
    )
    if valid_strategy not in source:
        source = source.replace(
            "simulation:\n",
            "simulation:\n" + valid_strategy,
            1,
        )
    invalid_strategy = (
        "  reset_strategy:\n"
        f"    schema_version: {str(strategy_schema).lower()}\n"
        f"    id: {strategy_id}\n"
    )
    source = source.replace(valid_strategy, invalid_strategy, 1)
    candidate = tmp_path / "project.yaml"
    candidate.write_text(source, encoding="utf-8")

    with pytest.raises(ConfigError, match=message):
        load_project_config(candidate, _environment())


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
    with pytest.raises(
        ValueError,
        match="robot.kinematics_profile_id must match",
    ):
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


def test_runtime_restores_captured_nonzero_dof_positions_with_readback():
    articulation = _FakeArticulation((32, 4))
    runtime = ArticulationRuntime("/World/Robot", "/World/Robot/base", None)
    runtime._articulation = articulation
    runtime._initial_dof_positions = runtime._read_dof_positions(
        context="test initial state"
    )

    articulation.dof_positions = (9.0, 8.0, 7.0, 6.0)
    runtime.restore_initial_joint_state()
    articulation.dof_positions = (9.0, 8.0, 7.0, 6.0)
    articulation.dof_velocities = (5.0,) * 4
    articulation.dof_velocity_targets = (6.0,) * 4
    articulation.dof_efforts = (7.0,) * 4
    runtime.restore_initial_joint_state()

    expected = [[0.1, -0.2, 0.3, -0.4]]
    assert articulation.dof_position_updates == [expected, expected]
    assert articulation.dof_positions == (0.1, -0.2, 0.3, -0.4)
    assert articulation.dof_velocities == (0.0,) * 4
    assert articulation.dof_velocity_targets == (0.0,) * 4
    assert articulation.dof_efforts == (0.0,) * 4


def test_runtime_base_velocity_adapter_has_finite_three_vector_readback():
    articulation = _FakeArticulation((32, 4))
    runtime = ArticulationRuntime("/World/Robot", "/World/Robot/base", None)
    runtime._articulation = articulation

    runtime.set_base_velocities((0.1, 0.2, 0.3), (0.4, 0.5, 0.6))

    assert runtime.get_base_velocities() == (
        (0.1, 0.2, 0.3),
        (0.4, 0.5, 0.6),
    )
    articulation.angular_velocity = (0.0, float("nan"), 0.0)
    with pytest.raises(ArticulationRuntimeError, match="non-finite"):
        runtime.get_base_velocities()


def _root_pose_runtime_harness(monkeypatch, physics_view):
    events = []

    @contextmanager
    def use_backend(name, *, raise_on_unsupported, raise_on_fallback):
        events.append(
            (
                "backend",
                name,
                raise_on_unsupported,
                raise_on_fallback,
            )
        )
        yield
        events.append(("backend_exit", name))

    class FakeRootView:
        def set_world_poses(self, *, positions, orientations):
            events.append(("pose", positions, orientations))

    class FakePhysicsInterface:
        def flush_changes(self):
            events.append(("flush",))

    backend_module = ModuleType(
        "isaacsim.core.experimental.utils.backend"
    )
    backend_module.use_backend = use_backend
    simulation_module = ModuleType("isaacsim.core.simulation_manager")
    simulation_module.SimulationManager = type(
        "SimulationManager",
        (),
        {
            "get_physics_simulation_view": staticmethod(
                lambda: physics_view
            )
        },
    )
    physics_module = ModuleType("omni.physics.core")
    physics_module.get_physics_simulation_interface = (
        lambda: FakePhysicsInterface()
    )
    monkeypatch.setitem(
        sys.modules,
        "isaacsim.core.experimental.utils.backend",
        backend_module,
    )
    monkeypatch.setitem(
        sys.modules,
        "isaacsim.core.simulation_manager",
        simulation_module,
    )
    monkeypatch.setitem(sys.modules, "omni.physics.core", physics_module)

    runtime = ArticulationRuntime(
        "/World/Robot",
        "/World/Robot/base",
        None,
    )
    runtime._base_link_view = FakeRootView()
    return runtime, events


def test_runtime_root_pose_uses_persistent_usd_source_before_kinematic_sync(
    monkeypatch,
):
    class FakePhysicsView:
        is_valid = True

        def update_articulations_kinematic(self):
            events.append(("sync",))
            return True

    runtime, events = _root_pose_runtime_harness(
        monkeypatch,
        FakePhysicsView(),
    )
    runtime.set_world_pose((4.0, 0.0, 0.2635), (1.0, 0.0, 0.0, 0.0))

    assert events == [
        ("backend", "usd", True, True),
        (
            "pose",
            [[4.0, 0.0, 0.2635]],
            [[1.0, 0.0, 0.0, 0.0]],
        ),
        ("backend_exit", "usd"),
        ("flush",),
        ("sync",),
    ]


def test_runtime_root_pose_fails_closed_when_physics_sync_is_unavailable(
    monkeypatch,
):
    class FakePhysicsView:
        def __init__(self, *, valid, sync_result):
            self.is_valid = valid
            self.sync_result = sync_result
            self.sync_calls = 0

        def update_articulations_kinematic(self):
            self.sync_calls += 1
            return self.sync_result

    for physics_view, message, expected_sync_calls in (
        (None, "physics simulation view is unavailable", None),
        (
            FakePhysicsView(valid=False, sync_result=True),
            "physics simulation view is unavailable",
            0,
        ),
        (
            FakePhysicsView(valid=True, sync_result=False),
            "articulation kinematic synchronization failed",
            1,
        ),
    ):
        runtime, _ = _root_pose_runtime_harness(
            monkeypatch,
            physics_view,
        )
        with pytest.raises(ArticulationRuntimeError, match=message):
            runtime.set_world_pose(
                (4.0, 0.0, 0.2635),
                (1.0, 0.0, 0.0, 0.0),
            )
        if expected_sync_calls is not None:
            assert physics_view.sync_calls == expected_sync_calls


def test_runtime_root_pose_rejects_uninitialized_root_view_before_imports():
    runtime = ArticulationRuntime(
        "/World/Robot",
        "/World/Robot/base",
        None,
    )
    with pytest.raises(
        ArticulationRuntimeError,
        match="physics root rigid-body view is not initialized",
    ):
        runtime.set_world_pose(
            (4.0, 0.0, 0.2635),
            (1.0, 0.0, 0.0, 0.0),
        )


def test_runtime_rejects_nonfinite_or_mismatched_dof_position_readback():
    runtime = ArticulationRuntime("/World/Robot", "/World/Robot/base", None)
    runtime._articulation = _FakeArticulation((32, 4), (0.0, float("nan")))
    with pytest.raises(ArticulationRuntimeError, match="non-finite"):
        runtime._read_dof_positions(context="test state")

    runtime._initial_dof_positions = (0.0, 1.0)
    runtime._articulation = _FakeArticulation((32, 4), (0.0, 1.0))

    def ignore_update(values):
        runtime._articulation.dof_position_updates.append(values)

    runtime._articulation.set_dof_positions = ignore_update
    runtime._articulation.dof_positions = (2.0, 3.0)
    with pytest.raises(ArticulationRuntimeError, match="does not match"):
        runtime.restore_initial_joint_state()


@pytest.mark.parametrize(
    ("state_attribute", "setter_name", "label"),
    [
        ("dof_velocities", "set_dof_velocities", "velocity"),
        (
            "dof_velocity_targets",
            "set_dof_velocity_targets",
            "velocity target",
        ),
        ("dof_efforts", "set_dof_efforts", "effort"),
    ],
)
def test_runtime_rejects_nonzero_dynamic_state_readback(
    state_attribute,
    setter_name,
    label,
):
    articulation = _FakeArticulation((32, 4), (0.0, 0.0))
    runtime = ArticulationRuntime("/World/Robot", "/World/Robot/base", None)
    runtime._articulation = articulation
    runtime._initial_dof_positions = (0.0, 0.0)

    def ignore_update(values):
        del values

    setattr(articulation, setter_name, ignore_update)
    setattr(articulation, state_attribute, (1.0, -1.0))
    with pytest.raises(
        ArticulationRuntimeError,
        match=rf"{label} readback is not zero",
    ):
        runtime.restore_initial_joint_state()
