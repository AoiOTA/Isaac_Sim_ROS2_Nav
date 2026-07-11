#!/usr/bin/env python3
"""Standalone Isaac Sim 6.0.1 application for ROS 2 navigation.

Configuration is parsed before Kit starts so ROS domain/RMW settings are in
place before the bridge extension loads.  All Isaac imports remain below the
``SimulationApp`` construction boundary.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import os
from pathlib import Path
import sys
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from isaac_sim.graphs.control_graph import control_graph_spec
from isaac_sim.graphs.odometry_graph import ideal_odometry_graph_spec
from isaac_sim.graphs.sensor_graph import core_sensor_graph_spec, lidar_graph_spec
from isaac_sim.graphs.tf_graph import structure_tf_graph_spec
from isaac_sim.src.bridge.tf_ownership import expected_tf_owners
from isaac_sim.src.config import (
    ProjectConfig,
    configure_process_environment,
    load_project_config,
)
from isaac_sim.src.experiment.scenario import load_dynamic_scenario
from isaac_sim.src.robot.spawn_pose_manager import (
    load_spawn_poses,
    require_map_calibration,
)
from isaac_sim.src.robot.articulation_runtime import (
    load_articulation_physics_config,
)
from isaac_sim.src.sensors.sensor_factory import _load_camera, _load_imu, _load_lidar
from isaac_sim.src.stage.asset_validator import validate_robot_articulation
from isaac_sim.src.stage.asset_validator import validate_sensor_frames
from isaac_sim.src.stage.physics_setup import (
    find_all_physics_scenes,
    validate_stage_units,
    validate_up_axis,
)
from isaac_sim.src.stage.scene_composer import SceneComposer


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the Isaac Sim + ROS 2 Jackal navigation simulation"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "isaac_sim/configs/project.yaml",
        help="project YAML configuration",
    )
    parser.add_argument(
        "--mode",
        choices=("ideal", "realistic"),
        help="override simulation.odometry_mode",
    )
    parser.add_argument(
        "--navigation-mode",
        choices=("mapping", "localization"),
        help="override simulation.navigation_mode",
    )
    parser.add_argument(
        "--structure-tf-source",
        choices=("isaac", "rsp"),
        help="override simulation.structure_tf_source",
    )
    parser.add_argument(
        "--headless",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="override the configured GUI/headless mode",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="stop after N render updates (0 means unlimited)",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate files, USD composition, contracts, and calibration without starting Kit",
    )
    parser.add_argument(
        "--dynamic-obstacles",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="enable or disable the configured deterministic dynamic obstacle set",
    )
    return parser


def _apply_cli_overrides(args: argparse.Namespace) -> None:
    if args.mode is not None:
        os.environ["ISAAC_NAV__SIMULATION__ODOMETRY_MODE"] = args.mode
    if args.navigation_mode is not None:
        os.environ["ISAAC_NAV__SIMULATION__NAVIGATION_MODE"] = args.navigation_mode
    if args.structure_tf_source is not None:
        os.environ["ISAAC_NAV__SIMULATION__STRUCTURE_TF_SOURCE"] = (
            args.structure_tf_source
        )
    if args.headless is not None:
        os.environ["ISAAC_NAV__SIMULATION__HEADLESS"] = "true" if args.headless else "false"
    if args.max_steps is not None:
        if args.max_steps < 0:
            raise ValueError("--max-steps must be non-negative")
        os.environ["ISAAC_NAV__SIMULATION__MAX_FRAMES"] = str(args.max_steps)


def validate_configuration(config: ProjectConfig) -> tuple[object, object]:
    """Validate configuration without importing USD/Kit modules."""

    config.require_runtime_paths()
    poses = load_spawn_poses(config.spawn.poses_file)
    if config.spawn.selected not in poses:
        raise ValueError(
            f"unknown selected spawn pose {config.spawn.selected!r}; available={sorted(poses)}"
        )
    selected_pose = poses[config.spawn.selected]
    if config.simulation.navigation_mode == "localization":
        require_map_calibration(selected_pose, "localization startup")
    if config.ground_truth.enabled:
        require_map_calibration(selected_pose, "map-frame Ground Truth publication")

    lidar = _load_lidar(config.files.lidar)
    imu = _load_imu(config.files.imu)
    _load_camera(config.files.camera)
    load_articulation_physics_config(config.files.robot)
    specifications = [
        control_graph_spec(config),
        core_sensor_graph_spec(config, str(imu["sensor_prim"])),
        lidar_graph_spec(config, "/Render/ValidationProduct"),
    ]
    if config.simulation.structure_tf_source == "isaac":
        specifications.append(structure_tf_graph_spec(config))
    if config.simulation.odometry_mode == "ideal":
        specifications.append(ideal_odometry_graph_spec(config))
    for specification in specifications:
        specification.validate()
    if lidar["topic_name"] != "/lidar/points_raw":
        raise ValueError("the navigation baseline requires /lidar/points_raw")
    expected_tf_owners(
        config.simulation.odometry_mode,
        config.simulation.structure_tf_source,
    )
    dynamic_scenario = load_dynamic_scenario(config.files.dynamic_obstacles)
    return selected_pose, dynamic_scenario


def validate_composed_stage(config: ProjectConfig, stage: object) -> None:
    """Validate a composed Stage after Kit has started (or in validate-only mode)."""

    scenes = find_all_physics_scenes(stage)
    if len(scenes) != 1:
        paths = [str(prim.GetPath()) for prim in scenes]
        raise ValueError(f"composed Stage must contain exactly one PhysicsScene, got {paths}")
    if str(scenes[0].GetPath()) != config.simulation.expected_physics_scene:
        raise ValueError(
            f"PhysicsScene is {scenes[0].GetPath()}, expected {config.simulation.expected_physics_scene}"
        )
    validate_stage_units(stage, 1.0)
    validate_up_axis(stage, "Z")
    validate_robot_articulation(
        stage,
        config.robot.articulation_root,
        config.robot.base_link_prim,
        config.robot.wheel_joints,
    )
    validate_sensor_frames(stage, config.robot.base_link_prim)


def validate_project(config: ProjectConfig) -> tuple[object, object]:
    """Perform complete validation for the no-Kit ``--validate-only`` path."""

    selected_pose, dynamic_scenario = validate_configuration(config)
    stage = SceneComposer(config).compose(save=False)
    validate_composed_stage(config, stage)
    return selected_pose, dynamic_scenario


def _enable_extensions(app: object, extension_ids: Sequence[str]) -> None:
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    for extension_id in extension_ids:
        manager.set_extension_enabled_immediate(extension_id, True)
        if not manager.is_extension_enabled(extension_id):
            raise RuntimeError(f"required Isaac extension could not be enabled: {extension_id}")
    app.update()


def run(config: ProjectConfig, selected_pose: object, dynamic_scenario: object) -> None:
    configure_process_environment(config)

    from isaacsim import SimulationApp

    original_argv = sys.argv[:]
    try:
        # SimulationApp otherwise forwards this application's argparse flags
        # to Kit as if they were native settings.
        sys.argv = [sys.argv[0]]
        app = SimulationApp(
            {
                "headless": config.simulation.headless,
                "renderer": config.simulation.renderer,
            }
        )
    finally:
        sys.argv = original_argv
    runtime = None
    node = None
    rclpy_started = False
    try:
        _enable_extensions(app, config.extensions)

        # Runtime composition uses omni.usd so sensors and OmniGraph operate on
        # exactly the same Stage object.
        stage = SceneComposer(config).compose(save=False)
        app.update()
        validate_composed_stage(config, stage)

        import rclpy
        from rcl_interfaces.msg import ParameterDescriptor
        from rclpy.node import Node
        from rclpy.parameter import Parameter

        rclpy.init(args=[])
        rclpy_started = True
        node = Node(
            "isaac_navigation_sim",
            namespace=config.ros2.namespace,
            parameter_overrides=[Parameter("use_sim_time", value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        read_only = ParameterDescriptor(read_only=True)
        node.declare_parameter(
            "dynamic_obstacles_enabled",
            bool(dynamic_scenario.enabled),
            read_only,
        )
        node.declare_parameter(
            "dynamic_obstacles_config_sha256",
            hashlib.sha256(config.files.dynamic_obstacles.read_bytes()).hexdigest(),
            read_only,
        )
        node.declare_parameter(
            "dynamic_obstacle_ids",
            [obstacle.obstacle_id for obstacle in dynamic_scenario.obstacles],
            read_only,
        )

        from isaac_sim.src.experiment.collision_monitor import CollisionMonitor
        from isaac_sim.src.experiment.dynamic_obstacles import DynamicObstacleManager
        from isaac_sim.src.robot.articulation_runtime import (
            ArticulationRuntime,
            load_articulation_physics_config,
        )
        from isaac_sim.src.robot.joint_validator import JointGroups, JointValidator
        from isaac_sim.src.robot.idle_brake import IdleBrake
        from isaac_sim.src.robot.reset import ResetHooks, ResetManager, ResetRequest
        from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseManager
        from isaac_sim.src.sensors.sensor_factory import SensorFactory
        from isaac_sim.src.stage.physics_setup import PhysicsSetup
        from isaacsim.core.simulation_manager import SimulationManager

        articulation_settings = load_articulation_physics_config(
            config.files.robot
        )
        runtime = PhysicsSetup(config.simulation).apply(stage, app)
        sensors = SensorFactory(config).create_all()
        dynamic_manager = DynamicObstacleManager(stage, dynamic_scenario)
        collision_monitor = CollisionMonitor(config.robot.base_link_prim, node)
        runtime.reset()

        robot = ArticulationRuntime(
            config.robot.articulation_root,
            config.robot.base_link_prim,
            app,
        )
        robot.initialize()
        robot.configure_stability(articulation_settings)
        JointValidator(
            config.robot.wheel_joints,
            JointGroups(config.robot.front_wheel_joints, config.robot.rear_wheel_joints),
        ).validate(robot.get_dof_names())
        spawn_manager = SpawnPoseManager(robot, load_spawn_poses(config.spawn.poses_file))
        spawn_manager.apply_usd_pose(config.spawn.selected)
        idle_brake = IdleBrake(
            node,
            robot,
            articulation_settings,
            clock=lambda: float(SimulationManager.get_simulation_time()),
        )

        from isaac_sim.src.bridge.ros_graph_builder import RosGraphBuilder

        graph_references: dict[str, object] = {
            "all": RosGraphBuilder(config, sensors).build()
        }

        from isaac_sim.src.bridge.reset_service import ResetServiceBridge
        from isaac_sim.src.ground_truth.recorder import GroundTruthRecorder

        reset_bridge = ResetServiceBridge(
            node,
            spawn_manager,
            default_pose_name=config.spawn.selected,
            navigation_mode=config.simulation.navigation_mode,
            odometry_mode=config.simulation.odometry_mode,
            simulation_time=lambda: float(
                SimulationManager.get_simulation_time()
            ),
        )
        ground_truth = (
            GroundTruthRecorder(config.ground_truth, robot, node, selected_pose)
            if config.ground_truth.enabled
            else None
        )

        def clear_controller_state() -> None:
            from isaac_sim.graphs.control_graph import build_control_graph

            idle_brake.reset()
            graph_references["control"] = build_control_graph(config)

        def reset_odometry(mode: str) -> None:
            if mode != config.simulation.odometry_mode:
                raise RuntimeError(
                    f"reset requested odometry={mode}, running mode={config.simulation.odometry_mode}"
                )
            reset_bridge.reset_ros_odometry(mode)
            if mode == "ideal":
                from isaac_sim.graphs.odometry_graph import build_odometry_graph

                graph_references["odometry"] = build_odometry_graph(config)

        def reset_ground_truth_path() -> None:
            collision_monitor.reset()
            if ground_truth is not None:
                ground_truth.reset_path()

        hooks = ResetHooks(
            send_zero_velocity=reset_bridge.send_zero_velocity,
            clear_controller_state=clear_controller_state,
            reset_odometry=reset_odometry,
            reset_ground_truth_path=reset_ground_truth_path,
            reset_dynamic_obstacles=dynamic_manager.reset,
            clear_costmaps=reset_bridge.clear_costmaps,
            publish_map_initial_pose=reset_bridge.publish_map_initial_pose,
        )
        reset_manager = ResetManager(runtime, spawn_manager, hooks)
        reset_bridge.bind(reset_manager)
        reset_manager.reset(
            ResetRequest(
                pose_name=config.spawn.selected,
                navigation_mode=config.simulation.navigation_mode,
                odometry_mode=config.simulation.odometry_mode,
                random_seed=dynamic_scenario.seed,
            )
        )

        max_frames = config.simulation.max_frames
        frame = 0
        node.get_logger().info(
            "Isaac navigation simulation ready: "
            f"navigation={config.simulation.navigation_mode}, "
            f"odometry={config.simulation.odometry_mode}, "
            f"structure_tf={config.simulation.structure_tf_source}, "
            f"spawn={config.spawn.selected}, dynamic={dynamic_scenario.enabled}, "
            f"max_frames={max_frames or 'unlimited'}"
        )
        while app.is_running() and (max_frames == 0 or frame < max_frames):
            app.update()
            rclpy.spin_once(node, timeout_sec=0.0)
            simulation_time = float(SimulationManager.get_simulation_time())
            dynamic_manager.update(simulation_time)
            collision_monitor.update(simulation_time)
            idle_brake.update()
            if ground_truth is not None:
                ground_truth.update(simulation_time)
            frame += 1
    finally:
        if runtime is not None:
            try:
                runtime.stop()
            except Exception as exc:
                print(f"warning: failed to stop simulation cleanly: {exc}", file=sys.stderr)
        if node is not None:
            node.destroy_node()
        if rclpy_started:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        app.close()


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    _apply_cli_overrides(args)
    config = load_project_config(args.config)
    configure_process_environment(config)
    selected_pose, dynamic_scenario = validate_configuration(config)
    if args.dynamic_obstacles is not None:
        dynamic_scenario = replace(
            dynamic_scenario, enabled=bool(args.dynamic_obstacles)
        )
    calibration = "calibrated" if selected_pose.map.calibrated else "uncalibrated"
    if args.validate_only:
        # This process exits immediately after validation, so importing pxr
        # without Kit is safe here.
        stage = SceneComposer(config).compose(save=False)
        validate_composed_stage(config, stage)
        print(
            "validation: PASS "
            f"(navigation={config.simulation.navigation_mode}, "
            f"odometry={config.simulation.odometry_mode}, "
            f"structure_tf={config.simulation.structure_tf_source}, "
            f"spawn={config.spawn.selected}, "
            f"dynamic_obstacles={dynamic_scenario.enabled}, {calibration})"
        )
        return 0
    run(config, selected_pose, dynamic_scenario)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
