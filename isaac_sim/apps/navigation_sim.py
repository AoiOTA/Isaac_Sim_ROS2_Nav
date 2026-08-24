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
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
from typing import Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from isaac_sim.graphs.control_graph import control_graph_spec
from isaac_sim.graphs.odometry_graph import ideal_odometry_graph_spec
from isaac_sim.graphs.sensor_graph import (
    core_sensor_graph_spec,
    lidar_graph_spec,
    lio_lidar_graph_spec,
)
from isaac_sim.graphs.tf_graph import structure_tf_graph_spec
from isaac_sim.src.bridge.tf_ownership import expected_tf_owners
from isaac_sim.src.config import (
    ProjectConfig,
    configure_process_environment,
    load_project_config,
)
from isaac_sim.src.experiment.scenario import load_dynamic_scenario
from isaac_sim.src.experiment.appearance import load_appearance_profiles
from isaac_sim.src.environment_selection import (
    DEFAULT_ENVIRONMENT_ROOT,
    resolve_environment_usd,
    resolve_spawn_poses_file,
    runtime_project_stage,
)
from isaac_sim.src.robot.spawn_pose_manager import (
    load_spawn_poses,
    require_map_calibration,
)
from isaac_sim.src.robot.articulation_runtime import (
    load_articulation_physics_config,
)
from isaac_sim.src.sensors.sensor_factory import (
    CAMERA_PROFILE_NAMES,
    LIO_LIDAR_PROFILE_NAMES,
    _load_camera,
    _load_imu,
    _load_lidar,
    resolve_camera_selection,
    resolve_lio_lidar_config,
)
from isaac_sim.src.stage.asset_validator import validate_robot_articulation
from isaac_sim.src.stage.asset_validator import validate_sensor_frames
from isaac_sim.src.stage.physics_setup import (
    find_all_physics_scenes,
    validate_stage_units,
    validate_up_axis,
)
from isaac_sim.src.stage.scene_composer import SceneComposer


V6_IMU_REGIME_GRID_USD = Path(
    "/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Grid/"
    "default_environment.usd"
).resolve()
V6_IMU_REGIME_SPAWN_FILE = (
    PROJECT_ROOT
    / "isaac_sim/configs/environments/v6_calibration_flat_20m.spawn.yaml"
).resolve()
V6_IMU_REGIME_DIAGNOSTIC_CONFIG = (
    PROJECT_ROOT
    / "ros2_ws/src/robot_experiments/config/v6_imu_regime_diagnostic.yaml"
).resolve()
V6_IMU_REGIME_FEATURE_CONFIG = (
    PROJECT_ROOT
    / "isaac_sim/configs/experiments/v6_calibration_grid_features.yaml"
).resolve()


def imu_regime_trace_provenance(
    config: ProjectConfig,
    dynamic_scenario: object,
    diagnostic_config_path: Path = V6_IMU_REGIME_DIAGNOSTIC_CONFIG,
) -> dict[str, object]:
    """Fail closed when the trace is not the bounded flat20 experiment."""

    requirements = {
        "environment_usd": str(V6_IMU_REGIME_GRID_USD),
        "spawn_poses_file": str(config.spawn.poses_file.resolve()),
        "diagnostic_config_file": str(diagnostic_config_path.resolve()),
        "spawn_pose": "flat20_start",
        "odometry_mode": "realistic",
        "navigation_mode": "mapping",
        "obstacle_authoring_enabled": True,
        "obstacle_config_file": str(config.files.dynamic_obstacles.resolve()),
        "obstacle_config_id": "v6_calibration_grid_features",
        "obstacle_seed": 20260821,
        "obstacle_count": 7,
        "moving_obstacle_count": 0,
        "ground_truth_enabled": True,
    }
    moving_obstacle_count = sum(
        item.mode != "stationary"
        or item.start != item.end
        or item.speed != 0.0
        for item in dynamic_scenario.obstacles
    )
    actual = {
        "environment_usd": str(config.environment.source_asset.resolve()),
        "spawn_poses_file": str(config.spawn.poses_file.resolve()),
        "diagnostic_config_file": str(diagnostic_config_path.resolve()),
        "spawn_pose": config.spawn.selected,
        "odometry_mode": config.simulation.odometry_mode,
        "navigation_mode": config.simulation.navigation_mode,
        "obstacle_authoring_enabled": bool(dynamic_scenario.enabled),
        "obstacle_config_file": str(config.files.dynamic_obstacles.resolve()),
        "obstacle_config_id": "v6_calibration_grid_features",
        "obstacle_seed": dynamic_scenario.seed,
        "obstacle_count": len(dynamic_scenario.obstacles),
        "moving_obstacle_count": moving_obstacle_count,
        "ground_truth_enabled": bool(config.ground_truth.enabled),
    }
    mismatches = {
        key: {"expected": expected, "actual": actual[key]}
        for key, expected in requirements.items()
        if actual[key] != expected
    }
    resource_errors = []
    for label, candidate, canonical in (
        ("spawn_poses_file", config.spawn.poses_file.resolve(), V6_IMU_REGIME_SPAWN_FILE),
        ("diagnostic_config_file", diagnostic_config_path.resolve(), V6_IMU_REGIME_DIAGNOSTIC_CONFIG),
        ("obstacle_config_file", config.files.dynamic_obstacles.resolve(), V6_IMU_REGIME_FEATURE_CONFIG),
    ):
        try:
            if candidate.read_bytes() != canonical.read_bytes():
                resource_errors.append(f"{label}_content_mismatch")
        except OSError as exc:
            resource_errors.append(f"{label}_unreadable:{exc}")
    if mismatches or resource_errors:
        raise ValueError(
            "V6 IMU regime trace requires the locked flat20 configuration: "
            + json.dumps(
                {"field_mismatches": mismatches, "resource_errors": resource_errors},
                sort_keys=True,
            )
        )
    return {
        "contract": "v6_imu_regime_flat20_features_v2",
        **actual,
    }


def effective_reset_seed(dynamic_seed: int | None, scenario_seed: int) -> int:
    """Resolve the one seed used by startup and the first Trigger reset."""

    seed = scenario_seed if dynamic_seed is None else dynamic_seed
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("effective reset seed must be a non-negative integer")
    return seed


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
        "--stage-readiness-timeout-s",
        type=float,
        help="override the bounded Kit stage readiness timeout in seconds",
    )
    parser.add_argument(
        "--pacing-mode",
        choices=("realtime", "unbounded"),
        help="wall-clock pacing; unbounded must be selected explicitly",
    )
    parser.add_argument(
        "--target-rtf",
        type=float,
        help="target simulation-time / steady-wall-time ratio in realtime mode",
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
    parser.add_argument(
        "--dynamic-obstacle-config",
        type=Path,
        help="override the physical obstacle schema for a frozen experiment campaign",
    )
    parser.add_argument(
        "--dynamic-case-id",
        type=str,
        default="",
        help="select one named schema-v3/v4 dynamic case or schema-v4 case set",
    )
    parser.add_argument(
        "--dynamic-variant-id",
        type=str,
        default="",
        help="select one named variant for every selected dynamic case",
    )
    parser.add_argument(
        "--dynamic-seed",
        type=int,
        default=None,
        help="override the scenario reset seed without mutating its YAML",
    )
    parser.add_argument(
        "--third-person-camera",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="enable or disable the GUI third-person robot-follow camera",
    )
    parser.add_argument(
        "--camera-profile",
        choices=CAMERA_PROFILE_NAMES,
        help="front Camera publication profile selected before Isaac starts",
    )
    parser.add_argument(
        "--lio-lidar-profile",
        choices=LIO_LIDAR_PROFILE_NAMES,
        help=(
            "optional FAST-LIO2 input producer profile selected before Isaac "
            "starts; defaults to the configured off profile"
        ),
    )
    parser.add_argument(
        "--environment-usd",
        type=str,
        help=(
            "environment USD absolute path, path relative to --environment-root, "
            "or unique filename below that root"
        ),
    )
    parser.add_argument(
        "--environment-root",
        type=Path,
        default=None,
        help=(
            "directory searched recursively for --environment-usd filenames "
            f"(default: {DEFAULT_ENVIRONMENT_ROOT})"
        ),
    )
    parser.add_argument(
        "--spawn-poses-file",
        type=Path,
        default=None,
        help="spawn-pose YAML for the selected environment",
    )
    parser.add_argument(
        "--spawn-pose",
        type=str,
        default=None,
        help="named pose selected from the spawn-pose YAML",
    )
    parser.add_argument(
        "--appearance-config",
        type=Path,
        default=PROJECT_ROOT / "isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml",
        help="fixed appearance-profile YAML applied through an anonymous USD Session Layer",
    )
    parser.add_argument(
        "--appearance-profile",
        type=str,
        default="baseline",
        help="initial appearance profile ID; the runner may switch it between resets",
    )
    parser.add_argument(
        "--odom-phase-trace",
        type=Path,
        default=None,
        help=(
            "write a default-off Stage 2.2-R2B Isaac-only odometry phase "
            "trace and run its fixed command sequence"
        ),
    )
    parser.add_argument(
        "--imu-regime-phase-trace",
        type=Path,
        default=None,
        help="write a passive, default-off V6 IMU/MotionAssist/GT phase trace",
    )
    parser.add_argument(
        "--imu-regime-diagnostic-config",
        type=Path,
        default=V6_IMU_REGIME_DIAGNOSTIC_CONFIG,
        help="resolved installed/source V6 IMU diagnostic YAML recorded in trace provenance",
    )
    parser.add_argument(
        "--r2c1-free-space-trace",
        type=Path,
        default=None,
        help=(
            "write a default-off Stage 2.2-R2C1 reset-separated free-space "
            "odometry probe trace; requires the frozen Kujiale mapping_start "
            "diagnostic configuration"
        ),
    )
    parser.add_argument(
        "--r2c2-free-space-envelope",
        type=Path,
        default=None,
        help=(
            "write a default-off Stage 2.2-R2C2 no-motion 3D free-space "
            "envelope trace at the frozen Kujiale mapping_start configuration"
        ),
    )
    parser.add_argument(
        "--r2c2a-free-space-envelope",
        type=Path,
        default=None,
        help=(
            "write a default-off Stage 2.2-R2C2A no-motion 3D free-space "
            "envelope trace with frozen invisible-collider bounds fallback"
        ),
    )
    parser.add_argument(
        "--r2c2a-collision-bounds-config",
        type=Path,
        default=None,
        help="frozen R2C2A collision-bounds fallback configuration",
    )
    parser.add_argument(
        "--r2c3-free-space-motion-trace",
        type=Path,
        default=None,
        help=(
            "write a default-off Stage 2.2-R2C3 envelope-bound, "
            "reset-separated free-space motion and odometry trace"
        ),
    )
    parser.add_argument(
        "--r2c3-collision-bounds-config",
        type=Path,
        default=None,
        help="frozen R2C3 collision-bounds fallback configuration",
    )
    parser.add_argument(
        "--r2d2-live-pose-delta-trace",
        type=Path,
        default=None,
        help=(
            "write a default-off Stage 2.2-R2D2 armed live Shadow trace; "
            "requires the frozen R2D2 diagnostic configuration"
        ),
    )
    parser.add_argument(
        "--r2d2-collision-bounds-config",
        type=Path,
        default=None,
        help="frozen R2D2 collision-bounds fallback configuration",
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
    if args.stage_readiness_timeout_s is not None:
        os.environ["ISAAC_NAV__SIMULATION__STAGE_READINESS_TIMEOUT_S"] = str(
            args.stage_readiness_timeout_s
        )
    if args.third_person_camera is not None:
        os.environ["ISAAC_NAV__THIRD_PERSON_CAMERA__ENABLED"] = (
            "true" if args.third_person_camera else "false"
        )
    if args.spawn_pose is not None:
        os.environ["ISAAC_NAV__SPAWN__SELECTED"] = args.spawn_pose

    if args.environment_usd is not None:
        environment_root = (
            args.environment_root
            or Path(
                os.environ.get(
                    "ISAAC_NAV_ENVIRONMENT_ROOT",
                    str(DEFAULT_ENVIRONMENT_ROOT),
                )
            )
        )
        source_asset = resolve_environment_usd(
            args.environment_usd,
            environment_root,
        )
        spawn_poses = resolve_spawn_poses_file(
            source_asset,
            explicit=args.spawn_poses_file,
            repository_profiles=PROJECT_ROOT / "isaac_sim/configs/environments",
        )
        runtime_dir = Path(
            os.environ.get(
                "ISAAC_NAV_RUNTIME_DIR",
                f"/tmp/isaac_sim_ros2_nav_{os.getuid()}",
            )
        )
        project_stage = runtime_project_stage(source_asset, runtime_dir)
        os.environ["ISAAC_NAV__ENVIRONMENT__SOURCE_ASSET"] = str(source_asset)
        os.environ["ISAAC_NAV__ENVIRONMENT__PROJECT_STAGE"] = str(project_stage)
        os.environ["ISAAC_NAV__SPAWN__POSES_FILE"] = str(spawn_poses)
        # Custom environments use the same robot-relative camera contract as
        # the built-in scene.  The committed pose is below a normal indoor
        # ceiling, so GUI runs may safely inherit the configured default.
    elif args.spawn_poses_file is not None:
        spawn_poses = args.spawn_poses_file.expanduser().resolve()
        if not spawn_poses.is_file():
            raise ValueError(f"spawn poses file not found: {spawn_poses}")
        os.environ["ISAAC_NAV__SPAWN__POSES_FILE"] = str(spawn_poses)


def validate_configuration(
    config: ProjectConfig,
    camera_profile: str | None = None,
    lio_lidar_profile: str | None = None,
) -> tuple[object, object, object, object]:
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
    lio_config = resolve_lio_lidar_config(lidar, lio_lidar_profile)
    imu = _load_imu(config.files.imu)
    camera_selection = resolve_camera_selection(
        _load_camera(config.files.camera),
        camera_profile,
        headless=config.simulation.headless,
    )
    load_articulation_physics_config(config.files.robot)
    specifications = [
        control_graph_spec(config),
        core_sensor_graph_spec(config, str(imu["sensor_prim"])),
        lidar_graph_spec(config, "/Render/ValidationProduct"),
    ]
    if lio_config is not None:
        specifications.append(
            lio_lidar_graph_spec(
                config, "/Render/LioValidationProduct", lio_config
            )
        )
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
    return selected_pose, dynamic_scenario, camera_selection, lio_config


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


def validate_project(config: ProjectConfig) -> tuple[object, object, object, object]:
    """Perform complete validation for the no-Kit ``--validate-only`` path."""

    (
        selected_pose,
        dynamic_scenario,
        camera_selection,
        lio_config,
    ) = validate_configuration(config)
    stage = SceneComposer(config).compose(save=False)
    validate_composed_stage(config, stage)
    return selected_pose, dynamic_scenario, camera_selection, lio_config


def _enable_extensions(app: object, extension_ids: Sequence[str]) -> None:
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    for extension_id in extension_ids:
        manager.set_extension_enabled_immediate(extension_id, True)
        if not manager.is_extension_enabled(extension_id):
            raise RuntimeError(f"required Isaac extension could not be enabled: {extension_id}")
    app.update()


def _simulation_app_config(config: ProjectConfig) -> dict[str, object]:
    return {
        "headless": config.simulation.headless,
        "renderer": config.simulation.renderer,
        "multi_gpu": False,
        "extra_args": [
            "--/renderer/raytracingMotion/enabled=true",
            "--/renderer/raytracingMotion/enableHydraEngineMasking=true",
            "--/renderer/raytracingMotion/enabledForHydraEngines=0,1,2,3",
            "--/rtx/rendering/perSensorTickTlas=true",
            "--/rtx/hydra/supportMultiTickRate=true",
            (
                "--/persistent/simulation/minFrameRate="
                f"{int(round(config.simulation.physics_hz))}"
            ),
        ],
    }


def run(
    config: ProjectConfig,
    selected_pose: object,
    dynamic_scenario: object,
    camera_selection: object,
    appearance_profiles: object,
    initial_appearance_profile: str,
    dynamic_case_id: str = "",
    dynamic_variant_id: str = "",
    dynamic_seed: int | None = None,
    odom_phase_trace_path: Path | None = None,
    r2c1_free_space_trace_path: Path | None = None,
    r2c2_free_space_envelope_path: Path | None = None,
    r2c2a_free_space_envelope_path: Path | None = None,
    r2c2a_collision_bounds_config_path: Path | None = None,
    r2c3_free_space_motion_trace_path: Path | None = None,
    r2c3_collision_bounds_config_path: Path | None = None,
    r2d2_live_pose_delta_trace_path: Path | None = None,
    r2d2_collision_bounds_config_path: Path | None = None,
    imu_regime_phase_trace_path: Path | None = None,
    imu_regime_diagnostic_config_path: Path = V6_IMU_REGIME_DIAGNOSTIC_CONFIG,
    lio_config: object = None,
) -> None:
    configure_process_environment(config)

    from isaacsim import SimulationApp

    original_argv = sys.argv[:]
    try:
        # SimulationApp otherwise forwards this application's argparse flags
        # to Kit as if they were native settings.
        sys.argv = [sys.argv[0]]
        app = SimulationApp(_simulation_app_config(config))
    finally:
        sys.argv = original_argv
    runtime = None
    sensors = None
    camera_graph_paths: tuple[str, ...] = ()
    node = None
    reset_bridge = None
    reset_stop_gate = None
    kidnap_bridge = None
    appearance_manager = None
    paired_appearance_capture = None
    imu_regime_phase_trace = None
    odom_phase_trace = None
    r2c1_trace = None
    r2c2_trace = None
    r2c2a_trace = None
    r2c1_observer_node = None
    r2c1_observer_executor = None
    r2c1_observer_thread = None
    rclpy_started = False
    failed = False
    try:
        _enable_extensions(app, config.extensions)

        from isaac_sim.src.stage.physics_setup import (
            PhysicsSetup,
            prepare_pacing,
        )

        prepare_pacing(config.simulation)

        # Runtime composition uses omni.usd so sensors and OmniGraph operate on
        # exactly the same Stage object.
        stage = SceneComposer(config).compose(save=False)
        # A referenced robot asset can still be resolving on the first Kit
        # update after reopening the persistent project layer.  Explicitly
        # load the composed stage before validating/creating the camera so a
        # transient unresolved reference cannot make the canonical camera
        # frames appear missing on a restart.
        stage.Load()
        probe_end_timecode = None
        if (
            odom_phase_trace_path is not None
            or r2c1_free_space_trace_path is not None
            or r2c2_free_space_envelope_path is not None
            or r2c2a_free_space_envelope_path is not None
            or r2c3_free_space_motion_trace_path is not None
            or r2d2_live_pose_delta_trace_path is not None
        ):
            # The warehouse Stage's normal end code is shorter than the
            # frozen 61-second probe.  This transient extension is scoped to
            # the default-off diagnostic mode and is never saved to USD.
            if (
                r2c2_free_space_envelope_path is not None
                or r2c2a_free_space_envelope_path is not None
            ):
                required_end = int(math.ceil(5.0 * config.simulation.rendering_hz))
            elif (
                r2c1_free_space_trace_path is not None
                or r2c3_free_space_motion_trace_path is not None
                or r2d2_live_pose_delta_trace_path is not None
            ):
                from isaac_sim.src.diagnostics.r2c1_free_space_probe import (
                    SegmentedFreeSpaceScript,
                )
                required_end = SegmentedFreeSpaceScript.required_end_timecode(
                    config.simulation.rendering_hz
                )
            else:
                from isaac_sim.src.diagnostics.odom_phase_trace import OdomPhaseScript
                required_end = OdomPhaseScript.required_end_timecode(
                    config.simulation.rendering_hz
                )
            probe_end_timecode = max(int(stage.GetEndTimeCode()), required_end)
            stage.SetEndTimeCode(probe_end_timecode)
        for _ in range(3):
            app.update()
        # Configure coherent Timeline/RunLoop/Fabric periods before the first
        # post-composition app update creates Fabric history caches.
        runtime = PhysicsSetup(config.simulation).apply(stage, app)
        app.update()
        validate_composed_stage(config, stage)
        camera_enabled = bool(
            config.third_person_camera.enabled
            and not config.simulation.headless
        )
        third_person_camera = None
        if camera_enabled:
            from isaac_sim.src.visualization.third_person_camera import (
                ThirdPersonCamera,
            )

            third_person_camera = ThirdPersonCamera(
                stage,
                config.robot.base_link_prim,
                config.third_person_camera,
            )

        import rclpy
        from rcl_interfaces.msg import ParameterDescriptor
        from rclpy.node import Node
        from rclpy.parameter import Parameter

        rclpy.init(args=[])
        rclpy_started = True
        parameter_overrides = [Parameter("use_sim_time", value=True)]
        if dynamic_case_id:
            parameter_overrides.append(
                Parameter("dynamic_case_id", value=dynamic_case_id)
            )
        if dynamic_variant_id:
            parameter_overrides.append(
                Parameter("dynamic_variant_id", value=dynamic_variant_id)
            )
        node = Node(
            "isaac_navigation_sim",
            namespace=config.ros2.namespace,
            parameter_overrides=parameter_overrides,
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
        node.declare_parameter(
            "third_person_camera_enabled", camera_enabled, read_only
        )
        node.declare_parameter(
            "lio_lidar_profile",
            "off" if lio_config is None else lio_config["profile_name"],
            read_only,
        )
        node.declare_parameter(
            "third_person_camera_prim_path",
            (
                f"{config.robot.base_link_prim}/"
                f"{config.third_person_camera.prim_name}"
            ),
            read_only,
        )
        node.declare_parameter(
            "environment_usd",
            str(config.environment.source_asset),
            read_only,
        )
        node.declare_parameter(
            "spawn_poses_file",
            str(config.spawn.poses_file),
            read_only,
        )
        node.declare_parameter(
            "physics_hz", float(config.simulation.physics_hz), read_only
        )
        node.declare_parameter(
            "rendering_hz", float(config.simulation.rendering_hz), read_only
        )

        from isaac_sim.src.bridge.reset_stop_gate import ResetStopGate

        diagnostic_command_mode = any((
            odom_phase_trace_path is not None,
            r2c1_free_space_trace_path is not None,
            r2c3_free_space_motion_trace_path is not None,
            r2d2_live_pose_delta_trace_path is not None,
        ))
        reset_stop_gate = ResetStopGate(
            node,
            input_topic=(
                "/cmd_vel_diagnostic"
                if diagnostic_command_mode
                else "/cmd_vel"
            ),
        )
        effective_seed = effective_reset_seed(dynamic_seed, dynamic_scenario.seed)

        from isaac_sim.src.experiment.appearance import AppearanceManager

        appearance_manager = AppearanceManager(stage, appearance_profiles)
        appearance_manager.bind_ros(node, initial_appearance_profile)

        from isaac_sim.src.experiment.collision_monitor import CollisionMonitor
        from isaac_sim.src.experiment.dynamic_obstacles import DynamicObstacleManager
        from isaac_sim.src.robot.articulation_runtime import (
            ArticulationRuntime,
            load_articulation_physics_config,
        )
        from isaac_sim.src.robot.joint_validator import JointGroups, JointValidator
        from isaac_sim.src.robot.idle_brake import IdleBrake
        from isaac_sim.src.robot.skid_steer_motion_assist import (
            SkidSteerMotionAssist,
        )
        from isaac_sim.src.robot.reset import ResetHooks, ResetManager, ResetRequest
        from isaac_sim.src.robot.spawn_pose_manager import SpawnPoseManager
        from isaac_sim.src.sensors.sensor_factory import SensorFactory
        from isaacsim.core.simulation_manager import SimulationManager

        articulation_settings = load_articulation_physics_config(
            config.files.robot
        )
        sensors = SensorFactory(
            config,
            camera_profile=camera_selection.profile.name,
            lio_lidar_profile=(
                None if lio_config is None else lio_config["profile_name"]
            ),
        ).create_all()
        if sensors.cameras:
            from isaac_sim.src.experiment.paired_appearance import (
                PairedAppearanceCapture,
            )

            paired_camera = sensors.cameras[0]
            paired_appearance_capture = PairedAppearanceCapture(
                node=node,
                render_product=paired_camera.render_product,
                appearance_manager=appearance_manager,
                appearance_profiles=appearance_profiles,
                width=paired_camera.width,
                height=paired_camera.height,
            )
        if dynamic_scenario.coordinate_frame == "map" and not selected_pose.map.calibrated:
            raise ValueError("map-coordinate obstacles require a calibrated selected spawn pose")
        def map_to_usd(position):
            """Map-frame obstacle coordinates through the calibrated spawn pair."""
            yaw = math.radians(selected_pose.usd.yaw_deg - selected_pose.map.yaw_deg)
            delta_x = position[0] - selected_pose.map.position[0]
            delta_y = position[1] - selected_pose.map.position[1]
            return (
                selected_pose.usd.position[0] + math.cos(yaw) * delta_x - math.sin(yaw) * delta_y,
                selected_pose.usd.position[1] + math.sin(yaw) * delta_x + math.cos(yaw) * delta_y,
                position[2],
            )

        def usd_to_map(position):
            """Inverse of map_to_usd for GUI-adjusted obstacle capture."""
            yaw = math.radians(selected_pose.usd.yaw_deg - selected_pose.map.yaw_deg)
            delta_x = position[0] - selected_pose.usd.position[0]
            delta_y = position[1] - selected_pose.usd.position[1]
            return (
                selected_pose.map.position[0] + math.cos(yaw) * delta_x + math.sin(yaw) * delta_y,
                selected_pose.map.position[1] - math.sin(yaw) * delta_x + math.cos(yaw) * delta_y,
                position[2],
            )

        dynamic_manager = DynamicObstacleManager(
            stage, dynamic_scenario, map_to_usd=map_to_usd, usd_to_map=usd_to_map
        )
        dynamic_manager.bind_ros(
            node, lambda: float(SimulationManager.get_simulation_time())
        )
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
        collision_monitor = CollisionMonitor(config.robot.base_link_prim, node)
        spawn_manager = SpawnPoseManager(robot, load_spawn_poses(config.spawn.poses_file))
        spawn_manager.apply_usd_pose(config.spawn.selected)
        camera_binding_reported = False
        if third_person_camera is not None:
            camera_binding_reported = third_person_camera.viewport_bound
            node.get_logger().info(
                "third-person camera created: "
                f"prim={third_person_camera.camera_path}, "
                f"viewport={'bound' if camera_binding_reported else 'pending'}"
            )
        idle_brake = IdleBrake(
            node,
            robot,
            articulation_settings,
            topic_name="/cmd_vel_sim",
            clock=lambda: float(SimulationManager.get_simulation_time()),
        )
        motion_assist = SkidSteerMotionAssist(
            node,
            robot,
            articulation_settings,
            physics_dt=1.0 / config.simulation.physics_hz,
            topic_name="/cmd_vel_sim",
            clock=lambda: float(SimulationManager.get_simulation_time()),
        )

        from isaac_sim.src.bridge.ros_graph_builder import RosGraphBuilder

        graph_handles = RosGraphBuilder(config, sensors).build()
        graph_references: dict[str, object] = {"all": graph_handles}
        if graph_handles.odometry is not None:
            graph_references["odometry"] = graph_handles.odometry
        odom_graph_epoch = 0
        camera_graph_paths = tuple(
            camera.graph_path for camera in sensors.cameras
        )

        imu_regime_graph_reader = None
        if imu_regime_phase_trace_path is not None:
            import omni.graph.core as og
            from isaac_sim.src.diagnostics.imu_regime_phase_trace import (
                ImuRegimePhaseTrace,
                make_imu_graph_reader,
            )

            imu_regime_phase_trace = ImuRegimePhaseTrace(
                imu_regime_phase_trace_path,
                provenance=imu_regime_trace_provenance(
                    config, dynamic_scenario, imu_regime_diagnostic_config_path
                ),
            )
            imu_regime_graph_reader = make_imu_graph_reader(
                og.Controller.attribute
            )

        odom_phase_script = None
        odom_phase_publisher = None
        if odom_phase_trace_path is not None:
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import Odometry
            from tf2_msgs.msg import TFMessage
            from isaac_sim.src.diagnostics.odom_phase_trace import (
                OdomPhaseScript,
                OdomPhaseTrace,
            )

            odom_phase_trace = OdomPhaseTrace(
                odom_phase_trace_path,
                stage_end_timecode=probe_end_timecode,
            )
            odom_phase_script = OdomPhaseScript()
            odom_phase_publisher = node.create_publisher(
                Twist, "/cmd_vel_diagnostic", 1
            )
            node.create_subscription(
                Odometry,
                "/odom",
                odom_phase_trace.record_odom,
                10,
            )
            node.create_subscription(TFMessage, "/tf", odom_phase_trace.record_tf, 10)

        r2c1_script = None
        r2c1_publisher = None
        r2c1_state: dict[str, object] | None = None
        if (
            r2c1_free_space_trace_path is not None
            or r2c3_free_space_motion_trace_path is not None
            or r2d2_live_pose_delta_trace_path is not None
        ):
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from nav_msgs.msg import Odometry
            from std_msgs.msg import Bool
            from tf2_msgs.msg import TFMessage
            from rclpy.executors import SingleThreadedExecutor
            from isaac_sim.src.diagnostics.r2c1_free_space_probe import (
                REQUIRED_CLEARANCE_M,
                R2C1Trace,
                SegmentedFreeSpaceScript,
                is_leaf_collision_prim,
                minimum_xy_clearance,
            )

            from pxr import Usd, UsdGeom, UsdPhysics
            r2d2_mode = r2d2_live_pose_delta_trace_path is not None
            r2c3_mode = r2c3_free_space_motion_trace_path is not None or r2d2_mode
            r2c3_preflight = None
            trace_schema = None
            trace_path = r2c1_free_space_trace_path
            trace_manifest: dict[str, object]
            collider_trace_rows: list[dict[str, object]] = []

            if not r2c3_mode:
                # Preserve the historical R2C1 two-dimensional preflight
                # exactly.  R2C3 uses the separate three-dimensional branch
                # below and never changes old trace behavior.
                cache = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
                )
                static_bounds_xy: list[tuple[float, float, float, float]] = []
                aggregate_collision_parent_count = 0
                for prim in Usd.PrimRange(stage.GetPseudoRoot()):
                    prim_path = str(prim.GetPath())
                    if prim_path.startswith(config.robot.articulation_root):
                        continue
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):
                        continue
                    if not is_leaf_collision_prim(
                        prim,
                        collision_api=UsdPhysics.CollisionAPI,
                        prim_range=Usd.PrimRange,
                    ):
                        aggregate_collision_parent_count += 1
                        continue
                    bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                    lower, upper = bounds.GetMin(), bounds.GetMax()
                    if float(upper[2]) - float(lower[2]) <= 0.12:
                        continue
                    static_bounds_xy.append((
                        float(lower[0]), float(lower[1]),
                        float(upper[0]), float(upper[1]),
                    ))
                start_position, start_orientation = robot.get_world_pose()
                start_yaw = math.atan2(
                    2.0 * (
                        start_orientation[0] * start_orientation[3]
                        + start_orientation[1] * start_orientation[2]
                    ),
                    1.0 - 2.0 * (
                        start_orientation[2] ** 2
                        + start_orientation[3] ** 2
                    ),
                )
                clearance_by_segment = {
                    segment.segment_id: minimum_xy_clearance(
                        start_xy=(
                            float(start_position[0]),
                            float(start_position[1]),
                        ),
                        yaw_rad=start_yaw,
                        segment=segment,
                        obstacle_bounds_xy=static_bounds_xy,
                    )
                    for segment in SegmentedFreeSpaceScript.segments
                }
                trace_manifest = {
                    "environment_source_asset": str(
                        config.environment.source_asset
                    ),
                    "environment_project_stage": str(
                        config.environment.project_stage
                    ),
                    "spawn_pose_name": config.spawn.selected,
                    "spawn_poses_sha256": hashlib.sha256(
                        config.spawn.poses_file.read_bytes()
                    ).hexdigest(),
                    "config_sha256": hashlib.sha256(
                        repr(config).encode("utf-8")
                    ).hexdigest(),
                    "robot_config_sha256": hashlib.sha256(
                        config.files.robot.read_bytes()
                    ).hexdigest(),
                    "physics_hz": config.simulation.physics_hz,
                    "rendering_hz": config.simulation.rendering_hz,
                    "physics_dt_s": 1.0 / config.simulation.physics_hz,
                    "render_dt_s": 1.0 / config.simulation.rendering_hz,
                    "dynamic_obstacles_enabled": bool(dynamic_scenario.enabled),
                    "nav2_enabled": False,
                    "module2_enabled": False,
                    "camera_enabled": False,
                    "scene": "kujiale",
                    "spawn": "mapping_start",
                    "appearance_profile": "baseline",
                    "seed": "stage2_2_r2c1_frozen_seed",
                    "reset_random_seed": dynamic_scenario.seed,
                    "dedicated_delivery_executor": True,
                    "delivery_recorder_mode": "dedicated",
                    "required_clearance_m": REQUIRED_CLEARANCE_M,
                    "static_collision_bound_count": len(static_bounds_xy),
                    "aggregate_collision_parent_excluded_count": (
                        aggregate_collision_parent_count
                    ),
                    "swept_clearance_m": clearance_by_segment,
                    "probe_segments": [
                        segment.segment_id
                        for segment in SegmentedFreeSpaceScript.segments
                    ],
                }
            else:
                from isaac_sim.src.diagnostics.r2c1_free_space_probe import (
                    yaw_from_wxyz,
                )
                from isaac_sim.src.diagnostics.r2c2_free_space_envelope import (
                    Bounds3D,
                    Collider,
                    SUPPORT_HEIGHT_VARIATION_M,
                    assess_envelope,
                )
                from isaac_sim.src.diagnostics.r2c2a_invisible_collision_bounds import (
                    load_collision_bounds_config,
                    resolve_collision_bounds,
                )
                from isaac_sim.src.diagnostics.r2c3_free_space_motion import (
                    FROZEN_SEED,
                    SCHEMA as R2C3_SCHEMA,
                    evaluate_envelope_preflight,
                )
                from isaac_sim.src.diagnostics.r2d2_live_pose_delta import (
                    FROZEN_SEED as R2D2_FROZEN_SEED,
                    SCHEMA as R2D2_SCHEMA,
                )
                from isaac_sim.src.yaml_utils import load_mapping

                selected_collision_bounds_path = (
                    r2d2_collision_bounds_config_path
                    if r2d2_mode
                    else r2c3_collision_bounds_config_path
                )
                if selected_collision_bounds_path is None:
                    raise RuntimeError("R2C3/R2D2 requires a collision-bounds configuration")
                bounds_config = load_collision_bounds_config(selected_collision_bounds_path)
                if (
                    bounds_config.source_asset_name
                    != config.environment.source_asset.name
                ):
                    raise RuntimeError(
                        "R2C3 collision-bounds configuration does not match "
                        "the source USD"
                    )
                robot_geometry = load_mapping(config.files.robot)
                footprint = robot_geometry.get("footprint")
                wheel_radius = robot_geometry.get("wheel_radius")
                if (
                    not isinstance(footprint, list)
                    or not isinstance(wheel_radius, (int, float))
                ):
                    raise RuntimeError(
                        "R2C3 requires robot footprint and wheel_radius"
                    )
                footprint_points = [
                    [float(value) for value in point] for point in footprint
                ]
                purposes = [
                    UsdGeom.Tokens.default_,
                    UsdGeom.Tokens.render,
                    UsdGeom.Tokens.proxy,
                    UsdGeom.Tokens.guide,
                ]
                normal_bbox_cache = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(), purposes, False, False
                )
                invisible_bbox_cache = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(), purposes, False, True
                )

                def r2c3_bounds_for(cache, prim) -> Bounds3D:
                    bounds = cache.ComputeWorldBound(
                        prim
                    ).ComputeAlignedRange()
                    lower, upper = bounds.GetMin(), bounds.GetMax()
                    return Bounds3D(
                        float(lower[0]),
                        float(lower[1]),
                        float(lower[2]),
                        float(upper[0]),
                        float(upper[1]),
                        float(upper[2]),
                    )

                def r2c3_collision_enabled(prim) -> bool:
                    attribute = UsdPhysics.CollisionAPI(
                        prim
                    ).GetCollisionEnabledAttr()
                    value = attribute.Get() if attribute.IsValid() else True
                    return bool(True if value is None else value)

                def r2c3_finite_transform(prim) -> bool:
                    matrix = UsdGeom.Xformable(
                        prim
                    ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    return all(
                        math.isfinite(float(value))
                        for row in matrix
                        for value in row
                    )

                def r2c3_descendant_facts(
                    prim,
                ) -> tuple[tuple[str, ...], tuple[str, ...], bool, str]:
                    gprims: list[str] = []
                    nested_collisions: list[str] = []
                    visibility: list[str] = []
                    finite = True
                    for descendant in Usd.PrimRange(prim):
                        if descendant == prim:
                            continue
                        if descendant.HasAPI(UsdPhysics.CollisionAPI):
                            nested_collisions.append(
                                str(descendant.GetPath())
                            )
                        if descendant.IsActive() and descendant.IsA(
                            UsdGeom.Gprim
                        ):
                            gprims.append(str(descendant.GetPath()))
                            imageable = UsdGeom.Imageable(descendant)
                            visibility.append(
                                str(
                                    imageable.ComputeVisibility(
                                        Usd.TimeCode.Default()
                                    )
                                )
                            )
                            finite = (
                                finite
                                and r2c3_bounds_for(
                                    invisible_bbox_cache, descendant
                                ).finite()
                                and r2c3_finite_transform(descendant)
                            )
                    effective_visibility = (
                        ",".join(sorted(set(visibility)))
                        if visibility
                        else "none"
                    )
                    return (
                        tuple(gprims),
                        tuple(nested_collisions),
                        finite,
                        effective_visibility,
                    )

                r2c3_colliders: list[Collider] = []
                r2c3_metadata: dict[str, dict[str, object]] = {}
                source_leaf_count = 0
                aggregate_count = 0
                for prim in Usd.PrimRange(stage.GetPseudoRoot()):
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):
                        continue
                    path = str(prim.GetPath())
                    aggregate = not is_leaf_collision_prim(
                        prim,
                        collision_api=UsdPhysics.CollisionAPI,
                        prim_range=Usd.PrimRange,
                    )
                    if path.startswith(config.robot.articulation_root):
                        continue
                    if aggregate:
                        aggregate_count += 1
                        bounds = r2c3_bounds_for(normal_bbox_cache, prim)
                        r2c3_colliders.append(
                            Collider(
                                path,
                                bounds,
                                r2c3_collision_enabled(prim),
                                aggregate=True,
                            )
                        )
                        r2c3_metadata[path] = {
                            "bounds_source": "AGGREGATE_EXCLUDED",
                            "source_gprim_paths": [],
                            "effective_visibility": "not_applicable",
                            "fallback_reason": (
                                "AGGREGATE_COLLISION_PARENT"
                            ),
                            "collision_schema_noncanonical": (
                                not prim.IsA(UsdGeom.Gprim)
                            ),
                            "fallback_bounds_source_delta_m": None,
                        }
                        continue
                    if path.startswith("/Root/"):
                        source_leaf_count += 1
                    (
                        gprims,
                        nested_collisions,
                        descendants_finite,
                        effective_visibility,
                    ) = r2c3_descendant_facts(prim)
                    resolution = resolve_collision_bounds(
                        path=path,
                        primary_bounds=r2c3_bounds_for(
                            normal_bbox_cache, prim
                        ),
                        fallback_bounds=r2c3_bounds_for(
                            invisible_bbox_cache, prim
                        ),
                        collision_enabled=r2c3_collision_enabled(prim),
                        is_leaf_collision=True,
                        active_gprim_paths=gprims,
                        descendant_collision_paths=nested_collisions,
                        descendants_finite=descendants_finite,
                        effective_visibility=effective_visibility,
                        collision_schema_noncanonical=(
                            not prim.IsA(UsdGeom.Gprim)
                        ),
                        config=bounds_config,
                    )
                    fields = resolution.trace_fields()
                    if (
                        resolution.bounds_source
                        == "INVISIBLE_COLLISION_SUBTREE_FALLBACK"
                    ):
                        source_bounds = r2c3_bounds_for(
                            invisible_bbox_cache,
                            stage.GetPrimAtPath(gprims[0]),
                        )
                        source_delta = (
                            max(
                                abs(left - right)
                                for left, right in zip(
                                    (
                                        resolution.bounds.min_x,
                                        resolution.bounds.min_y,
                                        resolution.bounds.min_z,
                                        resolution.bounds.max_x,
                                        resolution.bounds.max_y,
                                        resolution.bounds.max_z,
                                    ),
                                    (
                                        source_bounds.min_x,
                                        source_bounds.min_y,
                                        source_bounds.min_z,
                                        source_bounds.max_x,
                                        source_bounds.max_y,
                                        source_bounds.max_z,
                                    ),
                                )
                            )
                            if source_bounds.finite()
                            else math.inf
                        )
                        fields[
                            "fallback_bounds_source_delta_m"
                        ] = source_delta
                    else:
                        fields["fallback_bounds_source_delta_m"] = None
                    r2c3_colliders.append(
                        Collider(
                            path,
                            resolution.bounds,
                            r2c3_collision_enabled(prim),
                        )
                    )
                    r2c3_metadata[path] = fields

                frozen_fallback_paths = tuple(
                    item.collision_prim for item in bounds_config.fallbacks
                )
                clearance_by_segment = {
                    segment.segment_id: math.nan
                    for segment in SegmentedFreeSpaceScript.segments
                }
                trace_path = (
                    r2d2_live_pose_delta_trace_path
                    if r2d2_mode
                    else r2c3_free_space_motion_trace_path
                )
                trace_schema = R2D2_SCHEMA if r2d2_mode else R2C3_SCHEMA
                trace_manifest = {
                    "environment_source_asset": str(
                        config.environment.source_asset
                    ),
                    "environment_source_sha256": hashlib.sha256(
                        config.environment.source_asset.read_bytes()
                    ).hexdigest(),
                    "environment_project_stage": str(
                        config.environment.project_stage
                    ),
                    "spawn_pose_name": config.spawn.selected,
                    "spawn_poses_sha256": hashlib.sha256(
                        config.spawn.poses_file.read_bytes()
                    ).hexdigest(),
                    "config_sha256": hashlib.sha256(
                        repr(config).encode("utf-8")
                    ).hexdigest(),
                    "robot_config_sha256": hashlib.sha256(
                        config.files.robot.read_bytes()
                    ).hexdigest(),
                    "collision_bounds_config_path": str(selected_collision_bounds_path),
                    "collision_bounds_config_sha256": hashlib.sha256(
                        selected_collision_bounds_path.read_bytes()
                    ).hexdigest(),
                    "collision_bounds_config_schema": (
                        "bio_nav_stage2_2_r2c2a_collision_bounds_config_v1"
                    ),
                    "fallback_mappings": [
                        {
                            "collision_prim": item.collision_prim,
                            "source_gprim": item.source_gprim,
                        }
                        for item in bounds_config.fallbacks
                    ],
                    "physics_hz": config.simulation.physics_hz,
                    "rendering_hz": config.simulation.rendering_hz,
                    "physics_dt_s": 1.0 / config.simulation.physics_hz,
                    "render_dt_s": 1.0 / config.simulation.rendering_hz,
                    "dynamic_obstacles_enabled": bool(dynamic_scenario.enabled),
                    "nav2_enabled": False,
                    "module2_enabled": False,
                    "module2_shadow_expected": bool(r2d2_mode),
                    "camera_enabled": bool(r2d2_mode),
                    "camera_profile": (
                        camera_selection.profile.name if r2d2_mode else "off"
                    ),
                    "scene": "kujiale",
                    "spawn": "mapping_start",
                    "appearance_profile": "baseline",
                    "seed": R2D2_FROZEN_SEED if r2d2_mode else FROZEN_SEED,
                    "r2d2_mode": bool(r2d2_mode),
                    "reset_random_seed": dynamic_scenario.seed,
                    "dedicated_delivery_executor": True,
                    "delivery_recorder_mode": "dedicated",
                    "required_clearance_m": REQUIRED_CLEARANCE_M,
                    "support_height_variation_m": (
                        SUPPORT_HEIGHT_VARIATION_M
                    ),
                    "wheel_radius_m": float(wheel_radius),
                    "footprint": footprint_points,
                    "source_leaf_collision_count": source_leaf_count,
                    "aggregate_collision_parent_excluded_count": (
                        aggregate_count
                    ),
                    "probe_segments": [
                        segment.segment_id
                        for segment in SegmentedFreeSpaceScript.segments
                    ],
                }
                for collider in r2c3_colliders:
                    collider_trace_rows.append({
                        "kind": "collider_bounds",
                        "path": collider.path,
                        "bounds": collider.bounds.as_dict(),
                        "enabled": collider.enabled,
                        "aggregate": collider.aggregate,
                        **r2c3_metadata[collider.path],
                    })

                def capture_r2c3_preflight(
                    segment_id: str,
                ):
                    position, orientation = robot.get_world_pose()
                    yaw = yaw_from_wxyz(
                        tuple(float(item) for item in orientation)
                    )
                    current_cache = UsdGeom.BBoxCache(
                        Usd.TimeCode.Default(), purposes, False, False
                    )
                    robot_bounds: list[Bounds3D] = []
                    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
                        if not prim.HasAPI(UsdPhysics.CollisionAPI):
                            continue
                        path = str(prim.GetPath())
                        if not path.startswith(
                            config.robot.articulation_root
                        ):
                            continue
                        if is_leaf_collision_prim(
                            prim,
                            collision_api=UsdPhysics.CollisionAPI,
                            prim_range=Usd.PrimRange,
                        ):
                            robot_bounds.append(
                                r2c3_bounds_for(current_cache, prim)
                            )
                    if (
                        not robot_bounds
                        or not all(item.finite() for item in robot_bounds)
                    ):
                        raise RuntimeError(
                            "R2C3 robot collision envelope is unavailable"
                        )
                    robot_envelope = Bounds3D(
                        min(item.min_x for item in robot_bounds),
                        min(item.min_y for item in robot_bounds),
                        min(item.min_z for item in robot_bounds),
                        max(item.max_x for item in robot_bounds),
                        max(item.max_y for item in robot_bounds),
                        max(item.max_z for item in robot_bounds),
                    )
                    support_plane_z = (
                        float(position[2]) - float(wheel_radius)
                    )
                    classified, assessments = assess_envelope(
                        footprint=footprint_points,
                        start_x=float(position[0]),
                        start_y=float(position[1]),
                        start_yaw=yaw,
                        support_plane_z=support_plane_z,
                        robot_max_z=robot_envelope.max_z,
                        colliders=r2c3_colliders,
                    )
                    for item in classified:
                        item.update(r2c3_metadata[item["path"]])
                    assessment = next(
                        item
                        for item in assessments
                        if item.segment_id == segment_id
                    )
                    preflight = evaluate_envelope_preflight(
                        assessment=assessment,
                        classified_colliders=classified,
                        frozen_fallback_paths=frozen_fallback_paths,
                    )
                    classification_sha256 = hashlib.sha256(
                        json.dumps(
                            classified,
                            sort_keys=True,
                            separators=(",", ":"),
                        ).encode("utf-8")
                    ).hexdigest()
                    return (
                        preflight,
                        robot_envelope,
                        support_plane_z,
                        [float(item) for item in position],
                        yaw,
                        classification_sha256,
                    )

                r2c3_preflight = capture_r2c3_preflight

            if trace_path is None:
                raise RuntimeError("free-space diagnostic trace path missing")
            r2c1_trace = R2C1Trace(
                trace_path,
                manifest=trace_manifest,
                **({} if trace_schema is None else {"schema": trace_schema}),
            )
            for row in collider_trace_rows:
                r2c1_trace.write(row)
            r2c1_script = SegmentedFreeSpaceScript()
            r2c1_publisher = node.create_publisher(
                Twist, "/cmd_vel_diagnostic", 1
            )
            # R2D2 runs without a SLAM/AMCL scan publisher.  Its diagnostic
            # therefore publishes the same calibrated Module3 map pose that a
            # localization reset would publish, after each reset completes.
            # This is an R2D2-only seed for the Shadow Bridge, never a Nav2
            # localization correction or control input.
            r2d2_initialpose_publisher = (
                node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
                if r2d2_mode
                else None
            )
            r2c1_state = {
                "segment_index": -1,
                "segment_started_at": None,
                "pending_reset": None,
                "reset_epoch": 0,
                "active": False,
                "clearance_by_segment": clearance_by_segment,
                "last_trigger": None,
                "observer_loop_sequence": -1,
                "r2c3_mode": r2c3_mode,
                "r2d2_mode": r2d2_mode,
                "armed": not r2d2_mode,
                "preflight": r2c3_preflight,
                "preflight_done": not r2c3_mode,
            }
            if r2d2_mode:
                from std_srvs.srv import Trigger
                from isaac_sim.src.diagnostics.r2d2_live_pose_delta import ARM_SERVICE

                def arm_r2d2(_request, response):
                    if bool(r2c1_state["armed"]):
                        response.success = False
                        response.message = "R2D2 diagnostic is already armed"
                        return response
                    r2c1_state["armed"] = True
                    r2c1_trace.write({
                        "kind": "armed",
                        "reset_epoch": int(r2c1_state["reset_epoch"]),
                        "service": ARM_SERVICE,
                    })
                    response.success = True
                    response.message = "R2D2 live Shadow diagnostic armed"
                    return response

                node.create_service(Trigger, ARM_SERVICE, arm_r2d2)
            # A dedicated diagnostic node/executor prevents the main loop's
            # non-blocking spin from becoming a sampling/receipt bottleneck.
            r2c1_observer_node = Node(
                (
                    "isaac_r2d2_odom_observer"
                    if r2d2_mode
                    else "isaac_r2c3_odom_observer"
                    if r2c3_mode
                    else "isaac_r2c1_odom_observer"
                ),
                namespace=config.ros2.namespace,
                parameter_overrides=[Parameter("use_sim_time", value=True)],
                automatically_declare_parameters_from_overrides=True,
            )
            r2c1_observer_executor = SingleThreadedExecutor()
            r2c1_observer_executor.add_node(r2c1_observer_node)
            r2c1_observer_node.create_subscription(
                Odometry, "/odom",
                lambda message: r2c1_trace.record_odom(
                    message,
                    arrival_loop_sequence=int(r2c1_state["observer_loop_sequence"]),
                ), 100,
            )
            if r2d2_mode:
                from sensor_msgs.msg import CameraInfo, Image
                from rclpy.qos import qos_profile_sensor_data

                def r2d2_camera_receipt(message, kind: str) -> None:
                    stamp = message.header.stamp
                    r2c1_trace.write({
                        "kind": kind,
                        "reset_epoch": int(r2c1_state["reset_epoch"]),
                        "stamp_ns": int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
                        "width": int(getattr(message, "width", 0)),
                        "height": int(getattr(message, "height", 0)),
                    })

                r2c1_observer_node.create_subscription(
                    Image, "/camera/front/image_raw",
                    lambda message: r2d2_camera_receipt(message, "camera_receive"),
                    qos_profile_sensor_data,
                )
                r2c1_observer_node.create_subscription(
                    CameraInfo, "/camera/front/camera_info",
                    lambda message: r2d2_camera_receipt(message, "camera_info_receive"),
                    qos_profile_sensor_data,
                )
            r2c1_observer_node.create_subscription(
                TFMessage, "/tf",
                lambda message: r2c1_trace.record_tf(
                    message,
                    arrival_loop_sequence=int(r2c1_state["observer_loop_sequence"]),
                ), 100,
            )
            r2c1_observer_node.create_subscription(
                Bool, "/simulation/collision",
                lambda message: r2c1_trace.record_collision(
                    message, reset_epoch=int(r2c1_state["reset_epoch"])
                ), 100,
            )
            import threading
            r2c1_observer_thread = threading.Thread(
                target=r2c1_observer_executor.spin,
                name=(
                    "r2d2-odom-observer"
                    if r2d2_mode
                    else "r2c3-odom-observer"
                    if r2c3_mode
                    else "r2c1-odom-observer"
                ),
                daemon=True,
            )
            r2c1_observer_thread.start()

        from isaac_sim.src.bridge.reset_service import ResetServiceBridge
        from isaac_sim.src.ground_truth.recorder import GroundTruthRecorder

        reset_bridge = ResetServiceBridge(
            node,
            spawn_manager,
            default_pose_name=config.spawn.selected,
            navigation_mode=config.simulation.navigation_mode,
            odometry_mode=config.simulation.odometry_mode,
            default_reset_seed=effective_seed,
            simulation_time=lambda: float(
                SimulationManager.get_simulation_time()
            ),
            reset_stop_gate=reset_stop_gate,
            external_recovery_release_required=(
                config.simulation.navigation_mode == "localization"
                and not diagnostic_command_mode
            ),
        )
        from isaac_sim.src.bridge.kidnap_service import KidnapServiceBridge

        kidnap_bridge = KidnapServiceBridge(
            node,
            spawn_manager,
            odometry_mode=config.simulation.odometry_mode,
            simulation_time=lambda: float(
                SimulationManager.get_simulation_time()
            ),
        )

        def dynamic_robot_state() -> dict[str, float]:
            """Robot state in map coordinates for the dynamic safety gate."""
            position, orientation = robot.get_world_pose()
            linear, _ = robot.get_base_velocities()
            mapped = usd_to_map(position)
            # Transform the world linear vector through the same calibrated
            # inverse planar rotation as usd_to_map (translation cancels).
            yaw = math.radians(selected_pose.usd.yaw_deg - selected_pose.map.yaw_deg)
            vx = math.cos(yaw) * linear[0] + math.sin(yaw) * linear[1]
            vy = -math.sin(yaw) * linear[0] + math.cos(yaw) * linear[1]
            return {
                "x": mapped[0], "y": mapped[1], "vx": vx, "vy": vy,
                # Circumscribed radius of the configured 0.255 x 0.210 m
                # footprint.  Do not underestimate this in the actor guard.
                "speed": math.hypot(vx, vy), "footprint_radius": 0.33,
            }
        ground_truth = (
            GroundTruthRecorder(config.ground_truth, robot, node, selected_pose)
            if config.ground_truth.enabled
            else None
        )

        def clear_controller_state() -> None:
            from isaac_sim.graphs.control_graph import build_control_graph

            idle_brake.reset()
            motion_assist.reset()
            graph_references["control"] = build_control_graph(config)

        def reset_odometry(mode: str) -> None:
            nonlocal odom_graph_epoch
            if mode != config.simulation.odometry_mode:
                raise RuntimeError(
                    f"reset requested odometry={mode}, running mode={config.simulation.odometry_mode}"
                )
            reset_bridge.reset_ros_odometry(mode)
            if mode == "ideal":
                from isaac_sim.graphs.odometry_graph import build_odometry_graph

                previous = graph_references.get("odometry")
                if previous is not None:
                    previous.retire()
                odom_graph_epoch += 1
                graph_references["odometry"] = build_odometry_graph(
                    config, epoch=odom_graph_epoch
                )

        def reset_ground_truth_path() -> None:
            collision_monitor.reset()
            if ground_truth is not None:
                ground_truth.reset_path()

        hooks = ResetHooks(
            send_zero_velocity=reset_bridge.send_zero_velocity,
            clear_controller_state=clear_controller_state,
            reset_odometry=reset_odometry,
            reset_ground_truth_path=reset_ground_truth_path,
            reset_dynamic_obstacles=lambda seed: dynamic_manager.reset(
                seed,
                str(node.get_parameter("dynamic_case_id").value) or None,
                str(node.get_parameter("dynamic_variant_id").value) or None,
            ),
            clear_costmaps=reset_bridge.clear_costmaps,
            publish_map_initial_pose=reset_bridge.publish_map_initial_pose,
        )
        reset_manager = ResetManager(runtime, spawn_manager, hooks)
        reset_bridge.bind(reset_manager)
        startup_reset = reset_bridge.start_reset(
            ResetRequest(
                pose_name=config.spawn.selected,
                navigation_mode=config.simulation.navigation_mode,
                odometry_mode=config.simulation.odometry_mode,
                random_seed=(
                    effective_seed
                ),
                dynamic_case_id=dynamic_case_id,
                dynamic_variant_id=dynamic_variant_id,
            )
        )
        if startup_reset.finished and startup_reset.errors:
            raise RuntimeError(
                "startup reset transaction failed: "
                f"{startup_reset.errors}"
            )
        actual_spawn_position, _ = robot.get_world_pose()
        spawn_position_error_m = math.dist(
            actual_spawn_position,
            selected_pose.usd.position,
        )
        if spawn_position_error_m > 0.02:
            raise RuntimeError(
                "startup reset did not place the live articulation at the "
                f"selected spawn: error_m={spawn_position_error_m:.6f}, "
                f"expected={selected_pose.usd.position}, "
                f"actual={actual_spawn_position}"
            )

        r2c2_state: dict[str, object] | None = None
        if r2c2_free_space_envelope_path is not None:
            from isaac_sim.src.diagnostics.r2c1_free_space_probe import (
                is_leaf_collision_prim,
                yaw_from_wxyz,
            )
            from isaac_sim.src.diagnostics.r2c2_free_space_envelope import (
                Bounds3D,
                Collider,
                EnvelopeTrace,
                REQUIRED_CLEARANCE_M,
                SUPPORT_HEIGHT_VARIATION_M,
                assess_envelope,
            )
            from isaac_sim.src.yaml_utils import load_mapping
            from pxr import Usd, UsdGeom, UsdPhysics

            robot_geometry = load_mapping(config.files.robot)
            footprint = robot_geometry.get("footprint")
            wheel_radius = robot_geometry.get("wheel_radius")
            if not isinstance(footprint, list) or not isinstance(wheel_radius, (int, float)):
                raise RuntimeError("R2C2 requires robot footprint and wheel_radius")
            footprint_points = [[float(value) for value in point] for point in footprint]
            r2c2_trace = EnvelopeTrace(
                r2c2_free_space_envelope_path,
                manifest={
                    "environment_source_asset": str(config.environment.source_asset),
                    "environment_project_stage": str(config.environment.project_stage),
                    "spawn_pose_name": config.spawn.selected,
                    "spawn_poses_sha256": hashlib.sha256(config.spawn.poses_file.read_bytes()).hexdigest(),
                    "config_sha256": hashlib.sha256(repr(config).encode("utf-8")).hexdigest(),
                    "robot_config_sha256": hashlib.sha256(config.files.robot.read_bytes()).hexdigest(),
                    "physics_hz": config.simulation.physics_hz,
                    "rendering_hz": config.simulation.rendering_hz,
                    "dynamic_obstacles_enabled": bool(dynamic_scenario.enabled),
                    "nav2_enabled": False,
                    "module2_enabled": False,
                    "camera_enabled": False,
                    "scene": "kujiale",
                    "spawn": "mapping_start",
                    "appearance_profile": "baseline",
                    "required_clearance_m": REQUIRED_CLEARANCE_M,
                    "support_height_variation_m": SUPPORT_HEIGHT_VARIATION_M,
                    "wheel_radius_m": float(wheel_radius),
                    "footprint": footprint_points,
                },
            )

            def bounds_for(prim) -> Bounds3D:
                # Jackal's replacement wheel colliders intentionally use the
                # USD ``guide`` purpose.  A default-only BBoxCache silently
                # drops them and makes the physical collision envelope appear
                # absent.  R2C2 is an audit of collision geometry, so include
                # every geometry purpose rather than only render/default data.
                bounds = UsdGeom.BBoxCache(
                    Usd.TimeCode.Default(),
                    [
                        UsdGeom.Tokens.default_,
                        UsdGeom.Tokens.render,
                        UsdGeom.Tokens.proxy,
                        UsdGeom.Tokens.guide,
                    ],
                ).ComputeWorldBound(prim).ComputeAlignedRange()
                lower, upper = bounds.GetMin(), bounds.GetMax()
                return Bounds3D(
                    float(lower[0]), float(lower[1]), float(lower[2]),
                    float(upper[0]), float(upper[1]), float(upper[2]),
                )

            def collision_enabled(prim) -> bool:
                attribute = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
                value = attribute.Get() if attribute.IsValid() else True
                return bool(True if value is None else value)

            def capture_r2c2_envelope() -> None:
                position, orientation = robot.get_world_pose()
                yaw = yaw_from_wxyz(tuple(float(item) for item in orientation))
                robot_bounds: list[Bounds3D] = []
                colliders: list[Collider] = []
                aggregate_paths: list[str] = []
                for prim in Usd.PrimRange(stage.GetPseudoRoot()):
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):
                        continue
                    path = str(prim.GetPath())
                    aggregate = not is_leaf_collision_prim(
                        prim, collision_api=UsdPhysics.CollisionAPI, prim_range=Usd.PrimRange
                    )
                    if path.startswith(config.robot.articulation_root):
                        if not aggregate:
                            robot_bounds.append(bounds_for(prim))
                        continue
                    if aggregate:
                        aggregate_paths.append(path)
                        colliders.append(Collider(path, bounds_for(prim), collision_enabled(prim), aggregate=True))
                    else:
                        colliders.append(Collider(path, bounds_for(prim), collision_enabled(prim)))
                if not robot_bounds or not all(item.finite() for item in robot_bounds):
                    raise RuntimeError("R2C2 robot collision envelope is unavailable")
                robot_envelope = Bounds3D(
                    min(item.min_x for item in robot_bounds), min(item.min_y for item in robot_bounds), min(item.min_z for item in robot_bounds),
                    max(item.max_x for item in robot_bounds), max(item.max_y for item in robot_bounds), max(item.max_z for item in robot_bounds),
                )
                support_plane_z = float(position[2]) - float(wheel_radius)
                classified, assessments = assess_envelope(
                    footprint=footprint_points, start_x=float(position[0]), start_y=float(position[1]), start_yaw=yaw,
                    support_plane_z=support_plane_z, robot_max_z=robot_envelope.max_z,
                    colliders=colliders,
                )
                if any(item["classification"] in {"INVALID", "DISABLED"} for item in classified):
                    receipt = "STATIC_COLLIDER_CLASSIFICATION_INVALID"
                elif not all(item.support_coverage == 1.0 and item.support_height_variation_m <= SUPPORT_HEIGHT_VARIATION_M for item in assessments):
                    receipt = "SUPPORT_SURFACE_CONTRACT_INVALID"
                elif not all(item.minimum_clearance_m >= REQUIRED_CLEARANCE_M for item in assessments):
                    receipt = "SWEEP_CLEARANCE_INSUFFICIENT"
                else:
                    receipt = "FREE_SPACE_ENVELOPE_VALID"
                r2c2_trace.record(
                    robot_envelope=robot_envelope, support_plane_z=support_plane_z,
                    colliders=classified, assessments=assessments, receipt=receipt,
                )
                r2c2_trace.write({
                    "schema": "bio_nav_stage2_2_r2c2_free_space_envelope_v1", "kind": "capture",
                    "robot_position": [float(item) for item in position], "robot_yaw_rad": yaw,
                    "aggregate_collision_paths": aggregate_paths,
                })

            r2c2_state = {"settle_until": None, "captured": False, "capture": capture_r2c2_envelope}

        r2c2a_state: dict[str, object] | None = None
        if r2c2a_free_space_envelope_path is not None:
            if r2c2a_collision_bounds_config_path is None:
                raise RuntimeError("R2C2A requires a collision-bounds configuration")
            from isaac_sim.src.diagnostics.r2c1_free_space_probe import (
                is_leaf_collision_prim,
                yaw_from_wxyz,
            )
            from isaac_sim.src.diagnostics.r2c2_free_space_envelope import (
                Bounds3D,
                Collider,
                EnvelopeTrace,
                REQUIRED_CLEARANCE_M,
                SUPPORT_HEIGHT_VARIATION_M,
                assess_envelope,
            )
            from isaac_sim.src.diagnostics.r2c2a_invisible_collision_bounds import (
                SCHEMA as R2C2A_SCHEMA,
                load_collision_bounds_config,
                resolve_collision_bounds,
            )
            from isaac_sim.src.yaml_utils import load_mapping
            from pxr import Usd, UsdGeom, UsdPhysics

            bounds_config = load_collision_bounds_config(r2c2a_collision_bounds_config_path)
            if bounds_config.source_asset_name != config.environment.source_asset.name:
                raise RuntimeError("R2C2A collision-bounds configuration does not match the source USD")
            robot_geometry = load_mapping(config.files.robot)
            footprint = robot_geometry.get("footprint")
            wheel_radius = robot_geometry.get("wheel_radius")
            if not isinstance(footprint, list) or not isinstance(wheel_radius, (int, float)):
                raise RuntimeError("R2C2A requires robot footprint and wheel_radius")
            footprint_points = [[float(value) for value in point] for point in footprint]
            r2c2a_trace = EnvelopeTrace(
                r2c2a_free_space_envelope_path,
                schema=R2C2A_SCHEMA,
                manifest={
                    "environment_source_asset": str(config.environment.source_asset),
                    "environment_project_stage": str(config.environment.project_stage),
                    "spawn_pose_name": config.spawn.selected,
                    "spawn_poses_sha256": hashlib.sha256(config.spawn.poses_file.read_bytes()).hexdigest(),
                    "config_sha256": hashlib.sha256(repr(config).encode("utf-8")).hexdigest(),
                    "robot_config_sha256": hashlib.sha256(config.files.robot.read_bytes()).hexdigest(),
                    "collision_bounds_config_path": str(r2c2a_collision_bounds_config_path),
                    "collision_bounds_config_sha256": hashlib.sha256(r2c2a_collision_bounds_config_path.read_bytes()).hexdigest(),
                    "collision_bounds_config_schema": "bio_nav_stage2_2_r2c2a_collision_bounds_config_v1",
                    "fallback_mappings": [
                        {"collision_prim": item.collision_prim, "source_gprim": item.source_gprim}
                        for item in bounds_config.fallbacks
                    ],
                    "physics_hz": config.simulation.physics_hz,
                    "rendering_hz": config.simulation.rendering_hz,
                    "dynamic_obstacles_enabled": bool(dynamic_scenario.enabled),
                    "nav2_enabled": False,
                    "module2_enabled": False,
                    "camera_enabled": False,
                    "scene": "kujiale",
                    "spawn": "mapping_start",
                    "appearance_profile": "baseline",
                    "required_clearance_m": REQUIRED_CLEARANCE_M,
                    "support_height_variation_m": SUPPORT_HEIGHT_VARIATION_M,
                    "wheel_radius_m": float(wheel_radius),
                    "footprint": footprint_points,
                },
            )
            purposes = [
                UsdGeom.Tokens.default_, UsdGeom.Tokens.render,
                UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide,
            ]
            normal_bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes, False, False)
            invisible_bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes, False, True)

            def bounds_for(cache, prim) -> Bounds3D:
                bounds = cache.ComputeWorldBound(prim).ComputeAlignedRange()
                lower, upper = bounds.GetMin(), bounds.GetMax()
                return Bounds3D(
                    float(lower[0]), float(lower[1]), float(lower[2]),
                    float(upper[0]), float(upper[1]), float(upper[2]),
                )

            def collision_enabled(prim) -> bool:
                attribute = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr()
                value = attribute.Get() if attribute.IsValid() else True
                return bool(True if value is None else value)

            def finite_transform(prim) -> bool:
                matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                return all(math.isfinite(float(value)) for row in matrix for value in row)

            def descendant_facts(prim) -> tuple[tuple[str, ...], tuple[str, ...], bool, str]:
                gprims: list[str] = []
                nested_collisions: list[str] = []
                visibility: list[str] = []
                finite = True
                for descendant in Usd.PrimRange(prim):
                    if descendant == prim:
                        continue
                    if descendant.HasAPI(UsdPhysics.CollisionAPI):
                        nested_collisions.append(str(descendant.GetPath()))
                    if descendant.IsActive() and descendant.IsA(UsdGeom.Gprim):
                        gprims.append(str(descendant.GetPath()))
                        imageable = UsdGeom.Imageable(descendant)
                        visibility.append(str(imageable.ComputeVisibility(Usd.TimeCode.Default())))
                        finite = finite and bounds_for(invisible_bbox_cache, descendant).finite() and finite_transform(descendant)
                effective_visibility = ",".join(sorted(set(visibility))) if visibility else "none"
                return tuple(gprims), tuple(nested_collisions), finite, effective_visibility

            def capture_r2c2a_envelope() -> None:
                position, orientation = robot.get_world_pose()
                yaw = yaw_from_wxyz(tuple(float(item) for item in orientation))
                robot_bounds: list[Bounds3D] = []
                colliders: list[Collider] = []
                metadata: dict[str, dict[str, object]] = {}
                invalid_bounds = False
                aggregate_paths: list[str] = []
                source_leaf_count = 0
                for prim in Usd.PrimRange(stage.GetPseudoRoot()):
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):
                        continue
                    path = str(prim.GetPath())
                    aggregate = not is_leaf_collision_prim(
                        prim, collision_api=UsdPhysics.CollisionAPI, prim_range=Usd.PrimRange
                    )
                    if path.startswith(config.robot.articulation_root):
                        if not aggregate:
                            robot_bounds.append(bounds_for(normal_bbox_cache, prim))
                        continue
                    if aggregate:
                        aggregate_paths.append(path)
                        colliders.append(Collider(path, bounds_for(normal_bbox_cache, prim), collision_enabled(prim), aggregate=True))
                        metadata[path] = {
                            "bounds_source": "AGGREGATE_EXCLUDED", "source_gprim_paths": [],
                            "effective_visibility": "not_applicable", "fallback_reason": "AGGREGATE_COLLISION_PARENT",
                            "collision_schema_noncanonical": not prim.IsA(UsdGeom.Gprim),
                        }
                        continue
                    if path.startswith("/Root/"):
                        source_leaf_count += 1
                    gprims, nested_collisions, descendants_finite, effective_visibility = descendant_facts(prim)
                    resolution = resolve_collision_bounds(
                        path=path, primary_bounds=bounds_for(normal_bbox_cache, prim),
                        fallback_bounds=bounds_for(invisible_bbox_cache, prim),
                        collision_enabled=collision_enabled(prim), is_leaf_collision=True,
                        active_gprim_paths=gprims, descendant_collision_paths=nested_collisions,
                        descendants_finite=descendants_finite, effective_visibility=effective_visibility,
                        collision_schema_noncanonical=not prim.IsA(UsdGeom.Gprim), config=bounds_config,
                    )
                    fields = resolution.trace_fields()
                    if resolution.bounds_source == "INVISIBLE_COLLISION_SUBTREE_FALLBACK":
                        source_bounds = bounds_for(invisible_bbox_cache, stage.GetPrimAtPath(gprims[0]))
                        source_delta = max(
                            abs(left - right)
                            for left, right in zip(
                                (resolution.bounds.min_x, resolution.bounds.min_y, resolution.bounds.min_z, resolution.bounds.max_x, resolution.bounds.max_y, resolution.bounds.max_z),
                                (source_bounds.min_x, source_bounds.min_y, source_bounds.min_z, source_bounds.max_x, source_bounds.max_y, source_bounds.max_z),
                            )
                        ) if source_bounds.finite() else math.inf
                        fields["fallback_bounds_source_delta_m"] = source_delta
                        invalid_bounds = invalid_bounds or not math.isfinite(source_delta) or source_delta > 1.0e-6
                    else:
                        fields["fallback_bounds_source_delta_m"] = None
                    invalid_bounds = invalid_bounds or not resolution.valid
                    colliders.append(Collider(path, resolution.bounds, collision_enabled(prim)))
                    metadata[path] = fields
                if not robot_bounds or not all(item.finite() for item in robot_bounds):
                    raise RuntimeError("R2C2A robot collision envelope is unavailable")
                robot_envelope = Bounds3D(
                    min(item.min_x for item in robot_bounds), min(item.min_y for item in robot_bounds), min(item.min_z for item in robot_bounds),
                    max(item.max_x for item in robot_bounds), max(item.max_y for item in robot_bounds), max(item.max_z for item in robot_bounds),
                )
                support_plane_z = float(position[2]) - float(wheel_radius)
                classified, assessments = assess_envelope(
                    footprint=footprint_points, start_x=float(position[0]), start_y=float(position[1]), start_yaw=yaw,
                    support_plane_z=support_plane_z, robot_max_z=robot_envelope.max_z, colliders=colliders,
                )
                for item in classified:
                    item.update(metadata[item["path"]])
                if invalid_bounds:
                    receipt = "COLLISION_BOUNDS_FALLBACK_INVALID"
                elif any(item["classification"] in {"INVALID", "DISABLED"} for item in classified):
                    receipt = "STATIC_COLLIDER_CLASSIFICATION_INVALID"
                elif not all(item.support_coverage == 1.0 and item.support_height_variation_m <= SUPPORT_HEIGHT_VARIATION_M for item in assessments):
                    receipt = "SUPPORT_SURFACE_CONTRACT_INVALID"
                elif not all(item.minimum_clearance_m >= REQUIRED_CLEARANCE_M for item in assessments):
                    receipt = "SWEEP_CLEARANCE_INSUFFICIENT"
                else:
                    receipt = "FREE_SPACE_ENVELOPE_VALID"
                r2c2a_trace.record(
                    robot_envelope=robot_envelope, support_plane_z=support_plane_z,
                    colliders=classified, assessments=assessments, receipt=receipt,
                )
                r2c2a_trace.write({
                    "schema": R2C2A_SCHEMA, "kind": "capture",
                    "robot_position": [float(item) for item in position], "robot_yaw_rad": yaw,
                    "aggregate_collision_paths": aggregate_paths, "source_leaf_collision_count": source_leaf_count,
                    "fallback_collision_count": sum(1 for item in classified if item["bounds_source"] == "INVISIBLE_COLLISION_SUBTREE_FALLBACK"),
                    "primary_collision_count": sum(1 for item in classified if item["bounds_source"] == "VISIBLE_WORLD_BBOX"),
                })

            r2c2a_state = {"settle_until": None, "captured": False, "capture": capture_r2c2a_envelope}

        max_frames = config.simulation.max_frames
        frame = 0
        node.get_logger().info(
            "Isaac navigation simulation ready: "
            f"navigation={config.simulation.navigation_mode}, "
            f"odometry={config.simulation.odometry_mode}, "
            f"structure_tf={config.simulation.structure_tf_source}, "
            f"environment={config.environment.source_asset.name}, "
            f"spawn={config.spawn.selected}, dynamic={dynamic_scenario.enabled}, "
            f"dynamic_config={config.files.dynamic_obstacles.name}, "
            f"ground_truth={config.ground_truth.enabled}, "
            f"camera={camera_selection.profile.name}, "
            f"appearance={appearance_manager.active_profile_id}, "
            f"pacing={config.simulation.pacing_mode}, "
            f"target_rtf={config.simulation.target_realtime_factor:.3f}, "
            f"max_frames={max_frames or 'unlimited'}"
        )
        while (
            app.is_running()
            and rclpy.ok()
            and (max_frames == 0 or frame < max_frames)
        ):
            if imu_regime_phase_trace is not None:
                imu_regime_phase_trace.begin_loop(
                    loop_sequence=frame,
                    reset_generation=reset_stop_gate.generation,
                    simulation_time_s=float(
                        SimulationManager.get_simulation_time()
                    ),
                    before_app_monotonic_ns=time.monotonic_ns(),
                    robot=robot,
                    motion_assist=motion_assist,
                )
            if r2c1_state is not None and r2c1_script is not None:
                r2c1_state["observer_loop_sequence"] = frame
                if (
                    not bool(r2c1_state["active"])
                    and r2c1_state["pending_reset"] is None
                    and bool(r2c1_state["armed"])
                    and int(r2c1_state["segment_index"]) + 1 < len(r2c1_script.segments)
                ):
                    segment_index = int(r2c1_state["segment_index"]) + 1
                    segment = r2c1_script.segments[segment_index]
                    if not bool(r2c1_state["r2c3_mode"]):
                        clearance_m = float(
                            r2c1_state["clearance_by_segment"][
                                segment.segment_id
                            ]
                        )
                        valid = clearance_m >= REQUIRED_CLEARANCE_M
                        r2c1_trace.record_preflight(
                            segment_index=segment_index,
                            segment_id=segment.segment_id,
                            clearance_m=clearance_m,
                            valid=valid,
                        )
                        if not valid:
                            raise RuntimeError(
                                "R2C1 free-space preflight failed: "
                                f"clearance={clearance_m:.3f}m < "
                                f"{REQUIRED_CLEARANCE_M:.3f}m"
                            )
                    r2c1_state["segment_index"] = segment_index
                    r2c1_state["reset_epoch"] = int(r2c1_state["reset_epoch"]) + 1
                    transaction = reset_bridge.start_reset(
                        ResetRequest(
                            pose_name="mapping_start",
                            navigation_mode=config.simulation.navigation_mode,
                            odometry_mode=config.simulation.odometry_mode,
                            random_seed=dynamic_scenario.seed,
                        )
                    )
                    r2c1_state["pending_reset"] = transaction
                    r2c1_trace.record_segment_reset(
                        segment_index=segment_index,
                        segment_id=segment.segment_id,
                        reset_epoch=int(r2c1_state["reset_epoch"]),
                        simulation_time_s=float(SimulationManager.get_simulation_time()),
                        status="started",
                    )
            if odom_phase_trace is not None:
                odom_phase_trace.snapshot(
                    phase="before_app_update",
                    loop_sequence=frame,
                    simulation_time=float(SimulationManager.get_simulation_time()),
                    robot=robot,
                    motion_assist=motion_assist,
                    command=None,
                )
            app.update()
            after_app_monotonic_ns = time.monotonic_ns()
            try:
                rclpy.spin_once(node, timeout_sec=0.0)
            except Exception:
                # A single Ctrl+C reaches Isaac and the ROS launch trees at
                # the same time.  If another handler already shut the shared
                # rclpy context down, leave the simulation loop cleanly.
                if not rclpy.ok():
                    break
                raise
            if startup_reset.finished and startup_reset.errors:
                raise RuntimeError(
                    "startup reset transaction failed: "
                    f"{startup_reset.errors}"
                )
            simulation_time = float(SimulationManager.get_simulation_time())
            if imu_regime_phase_trace is not None:
                imu_regime_phase_trace.after_app(
                    simulation_time_s=simulation_time,
                    after_app_monotonic_ns=after_app_monotonic_ns,
                    robot=robot,
                    imu_graph=(
                        imu_regime_graph_reader()
                        if imu_regime_graph_reader is not None
                        else {}
                    ),
                )
            if paired_appearance_capture is not None and startup_reset.finished:
                paired_appearance_capture.update(simulation_time)
            if r2c2_state is not None and startup_reset.finished:
                if r2c2_state["settle_until"] is None:
                    r2c2_state["settle_until"] = simulation_time + 2.0
                elif not bool(r2c2_state["captured"]) and simulation_time >= float(r2c2_state["settle_until"]):
                    capture = r2c2_state["capture"]
                    if not callable(capture):
                        raise RuntimeError("R2C2 envelope capture is unavailable")
                    capture()
                    r2c2_state["captured"] = True
            if r2c2a_state is not None and startup_reset.finished:
                if r2c2a_state["settle_until"] is None:
                    r2c2a_state["settle_until"] = simulation_time + 2.0
                elif not bool(r2c2a_state["captured"]) and simulation_time >= float(r2c2a_state["settle_until"]):
                    capture = r2c2a_state["capture"]
                    if not callable(capture):
                        raise RuntimeError("R2C2A envelope capture is unavailable")
                    capture()
                    r2c2a_state["captured"] = True
            r2c1_after_app_payload = None
            if r2c1_state is not None and r2c1_script is not None:
                pending = r2c1_state["pending_reset"]
                if pending is not None and pending.finished:
                    if pending.errors:
                        raise RuntimeError(f"R2C1 segment reset failed: {pending.errors}")
                    segment = r2c1_script.segments[int(r2c1_state["segment_index"])]
                    r2c1_state["pending_reset"] = None
                    r2c1_state["segment_started_at"] = simulation_time
                    r2c1_state["active"] = True
                    r2c1_state["preflight_done"] = not bool(
                        r2c1_state["r2c3_mode"]
                    )
                    if r2d2_initialpose_publisher is not None:
                        initialpose = PoseWithCovarianceStamped()
                        initialpose.header.stamp = node.get_clock().now().to_msg()
                        initialpose.header.frame_id = "map"
                        initialpose.pose.pose.position.x = float(selected_pose.map.position[0])
                        initialpose.pose.pose.position.y = float(selected_pose.map.position[1])
                        yaw = math.radians(float(selected_pose.map.yaw_deg))
                        initialpose.pose.pose.orientation.z = math.sin(yaw * 0.5)
                        initialpose.pose.pose.orientation.w = math.cos(yaw * 0.5)
                        initialpose.pose.covariance[0] = float(selected_pose.map.position_stddev_m) ** 2
                        initialpose.pose.covariance[7] = float(selected_pose.map.position_stddev_m) ** 2
                        initialpose.pose.covariance[35] = math.radians(float(selected_pose.map.yaw_stddev_deg)) ** 2
                        r2d2_initialpose_publisher.publish(initialpose)
                        r2c1_trace.write({
                            "kind": "r2d2_initialpose_published",
                            "reset_epoch": int(r2c1_state["reset_epoch"]),
                            "stamp_ns": int(initialpose.header.stamp.sec) * 1_000_000_000 + int(initialpose.header.stamp.nanosec),
                            "frame_id": "map",
                        })
                    r2c1_trace.record_segment_reset(
                        segment_index=int(r2c1_state["segment_index"]),
                        segment_id=segment.segment_id,
                        reset_epoch=int(r2c1_state["reset_epoch"]),
                        simulation_time_s=simulation_time,
                        status="complete",
                    )
                if bool(r2c1_state["active"]):
                    segment = r2c1_script.segments[int(r2c1_state["segment_index"])]
                    elapsed = simulation_time - float(r2c1_state["segment_started_at"])
                    _, _, segment_phase = r2c1_script.phase(elapsed, segment)
                    r2c1_after_app_payload = r2c1_trace.snapshot(
                        phase="after_app_update", loop_sequence=frame,
                        reset_epoch=int(r2c1_state["reset_epoch"]),
                        segment_index=int(r2c1_state["segment_index"]),
                        segment_id=segment.segment_id, segment_phase=segment_phase,
                        simulation_time_s=simulation_time, robot=robot,
                        motion_assist=motion_assist,
                    )
                    previous = r2c1_state["last_trigger"]
                    if previous is not None:
                        r2c1_trace.record_realized_next(
                            trigger_loop_sequence=int(previous["loop_sequence"]),
                            reset_epoch=int(previous["reset_epoch"]),
                            simulation_time_s=simulation_time,
                            payload=r2c1_after_app_payload,
                        )
                        r2c1_state["last_trigger"] = None
                    if (
                        bool(r2c1_state["r2c3_mode"])
                        and not bool(r2c1_state["preflight_done"])
                        and elapsed >= r2c1_script.settle_s
                    ):
                        capture = r2c1_state["preflight"]
                        if not callable(capture):
                            raise RuntimeError(
                                "R2C3 envelope preflight is unavailable"
                            )
                        (
                            preflight,
                            robot_envelope,
                            support_plane_z,
                            robot_position,
                            robot_yaw,
                            classification_sha256,
                        ) = capture(segment.segment_id)
                        r2c1_state["clearance_by_segment"][
                            segment.segment_id
                        ] = preflight.minimum_clearance_m
                        r2c1_trace.write({
                            "kind": "robot_envelope",
                            "segment_index": int(
                                r2c1_state["segment_index"]
                            ),
                            "segment_id": segment.segment_id,
                            "reset_epoch": int(
                                r2c1_state["reset_epoch"]
                            ),
                            "bounds": robot_envelope.as_dict(),
                            "support_plane_z": support_plane_z,
                            "robot_position": robot_position,
                            "robot_yaw_rad": robot_yaw,
                        })
                        r2c1_trace.write({
                            "kind": "segment_envelope_preflight",
                            "segment_index": int(
                                r2c1_state["segment_index"]
                            ),
                            "reset_epoch": int(
                                r2c1_state["reset_epoch"]
                            ),
                            "classification_sha256": (
                                classification_sha256
                            ),
                            **preflight.trace_fields(),
                        })
                        r2c1_trace.record_preflight(
                            segment_index=int(
                                r2c1_state["segment_index"]
                            ),
                            segment_id=segment.segment_id,
                            clearance_m=preflight.minimum_clearance_m,
                            valid=preflight.valid,
                        )
                        r2c1_state["preflight_done"] = True
                        if not preflight.valid:
                            raise RuntimeError(
                                "R2C3 envelope preflight failed for "
                                f"{segment.segment_id}: "
                                f"coverage={preflight.support_coverage:.6f}, "
                                "height_variation="
                                f"{preflight.support_height_variation_m:.6f}m, "
                                "clearance="
                                f"{preflight.minimum_clearance_m:.6f}m, "
                                "fallback_delta="
                                f"{preflight.fallback_source_delta_max_m:.6g}m"
                            )
            command = None
            if odom_phase_script is not None and odom_phase_publisher is not None:
                command = odom_phase_script.command(simulation_time)
                if command is not None:
                    from geometry_msgs.msg import Twist

                    message = Twist()
                    message.linear.x = command[0]
                    message.angular.z = command[1]
                    odom_phase_publisher.publish(message)
            if odom_phase_trace is not None:
                odom_phase_trace.snapshot(
                    phase="after_app_update",
                    loop_sequence=frame,
                    simulation_time=simulation_time,
                    robot=robot,
                    motion_assist=motion_assist,
                    command=command,
                )
            r2c1_segment_phase = None
            if r2c1_state is not None and r2c1_script is not None and bool(r2c1_state["active"]):
                segment = r2c1_script.segments[int(r2c1_state["segment_index"])]
                elapsed = simulation_time - float(r2c1_state["segment_started_at"])
                linear_x, angular_z, r2c1_segment_phase = r2c1_script.phase(elapsed, segment)
                if elapsed >= r2c1_script.segment_duration_s():
                    r2c1_trace.record_segment_end(
                        segment_index=int(r2c1_state["segment_index"]),
                        segment_id=segment.segment_id,
                        reset_epoch=int(r2c1_state["reset_epoch"]),
                        clearance_m=float(r2c1_state["clearance_by_segment"][segment.segment_id]),
                    )
                    r2c1_state["active"] = False
                    r2c1_segment_phase = None
                elif r2c1_publisher is not None:
                    from geometry_msgs.msg import Twist
                    message = Twist()
                    message.linear.x = linear_x
                    message.angular.z = angular_z
                    r2c1_publisher.publish(message)
            dynamic_manager.update(simulation_time, dynamic_robot_state())
            collision_monitor.update(simulation_time)
            motion_assist_applied = False
            if not idle_brake.update():
                motion_assist_applied = motion_assist.update()
            if imu_regime_phase_trace is not None:
                imu_regime_phase_trace.after_assist(
                    after_assist_monotonic_ns=time.monotonic_ns(),
                    robot=robot,
                    motion_assist=motion_assist,
                    applied=motion_assist_applied,
                )
            if odom_phase_trace is not None:
                odom_phase_trace.snapshot(
                    phase="after_motion_assist_update",
                    loop_sequence=frame,
                    simulation_time=simulation_time,
                    robot=robot,
                    motion_assist=motion_assist,
                    command=command,
                )
            r2c1_post_assist_payload = None
            if r2c1_state is not None and r2c1_script is not None and bool(r2c1_state["active"]):
                segment = r2c1_script.segments[int(r2c1_state["segment_index"])]
                r2c1_post_assist_payload = r2c1_trace.snapshot(
                    phase="after_motion_assist_update", loop_sequence=frame,
                    reset_epoch=int(r2c1_state["reset_epoch"]),
                    segment_index=int(r2c1_state["segment_index"]),
                    segment_id=segment.segment_id,
                    segment_phase=str(r2c1_segment_phase or "idle"),
                    simulation_time_s=simulation_time, robot=robot,
                    motion_assist=motion_assist,
                )
            odom_publish = None
            should_publish_ideal_odom = (
                config.simulation.odometry_mode == "ideal"
                and r2c2_state is None
                and r2c2a_state is None
                and (r2c1_state is None or bool(r2c1_state["active"]))
            )
            if should_publish_ideal_odom:
                ideal_odom = graph_references.get("odometry")
                if ideal_odom is None:
                    raise RuntimeError("ideal odometry graph is unavailable after motion assist")
                if (
                    r2c1_state is not None
                    and r2c1_script is not None
                    and bool(r2c1_state["r2c3_mode"])
                ):
                    segment = r2c1_script.segments[
                        int(r2c1_state["segment_index"])
                    ]
                    r2c1_trace.register_trigger_context(
                        simulation_time_s=simulation_time,
                        loop_sequence=frame,
                        reset_epoch=int(r2c1_state["reset_epoch"]),
                        segment_index=int(r2c1_state["segment_index"]),
                        segment_id=segment.segment_id,
                        segment_phase=str(r2c1_segment_phase or "idle"),
                    )
                odom_publish = ideal_odom.trigger(frame)
                if odom_phase_trace is not None:
                    odom_phase_trace.record_odom_trigger(
                        odom_publish, simulation_time=simulation_time
                    )
                    # Drain now so normal probe rows carry the originating
                    # trigger loop.  Header-stamp association stays active
                    # for delayed DDS callbacks and fails closed if it differs.
                    rclpy.spin_once(node, timeout_sec=0.0)
                    odom_phase_trace.snapshot(
                        phase="after_odom_trigger",
                        loop_sequence=frame,
                        simulation_time=simulation_time,
                        robot=robot,
                        motion_assist=motion_assist,
                        command=command,
                        odom_publish=odom_publish,
                    )
                if r2c1_state is not None and r2c1_script is not None:
                    segment = r2c1_script.segments[int(r2c1_state["segment_index"])]
                    r2c1_trace.record_trigger(
                        odom_publish, simulation_time_s=simulation_time,
                        loop_sequence=frame, reset_epoch=int(r2c1_state["reset_epoch"]),
                        segment_index=int(r2c1_state["segment_index"]),
                        segment_id=segment.segment_id,
                        segment_phase=str(r2c1_segment_phase or "idle"),
                        post_assist_payload=(r2c1_post_assist_payload or r2c1_after_app_payload or {}),
                    )
                    r2c1_state["last_trigger"] = {
                        "loop_sequence": frame,
                        "reset_epoch": int(r2c1_state["reset_epoch"]),
                    }
            ground_truth_receipt = None
            before_ground_truth_monotonic_ns = time.monotonic_ns()
            if ground_truth is not None:
                ground_truth_receipt = ground_truth.update(simulation_time)
            if imu_regime_phase_trace is not None:
                imu_regime_phase_trace.finish_loop(
                    before_ground_truth_monotonic_ns=(
                        before_ground_truth_monotonic_ns
                    ),
                    after_ground_truth_monotonic_ns=time.monotonic_ns(),
                    ground_truth_receipt=ground_truth_receipt,
                    reset_generation_after_ground_truth=(
                        reset_stop_gate.generation
                    ),
                )
            if third_person_camera is not None:
                third_person_camera.bind_viewport()
                if (
                    third_person_camera.viewport_bound
                    and not camera_binding_reported
                ):
                    node.get_logger().info(
                        "third-person camera bound to the active Isaac viewport"
                    )
                    camera_binding_reported = True
            frame += 1
            if r2c2_state is not None and bool(r2c2_state["captured"]):
                break
            if r2c2a_state is not None and bool(r2c2a_state["captured"]):
                break
            if (
                r2c1_state is not None and r2c1_script is not None
                and int(r2c1_state["segment_index"]) + 1 >= len(r2c1_script.segments)
                and not bool(r2c1_state["active"])
                and r2c1_state["pending_reset"] is None
            ):
                break
            if odom_phase_script is not None and odom_phase_script.complete(simulation_time):
                break
    except Exception:
        # Kit's fast-shutdown path terminates Python with os._exit().  Print
        # initialization/runtime failures before close() so their traceback and
        # nonzero status are not replaced by a successful shutdown message.
        failed = True
        traceback.print_exc()
        raise
    finally:
        if paired_appearance_capture is not None:
            try:
                paired_appearance_capture.close()
            except Exception as exc:
                print(
                    f"warning: failed to close paired appearance capture cleanly: {exc}",
                    file=sys.stderr,
                )
        if r2c1_observer_executor is not None:
            try:
                r2c1_observer_executor.shutdown(timeout_sec=2.0)
            except Exception as exc:
                print(
                    f"warning: failed to stop R2C1 observer executor cleanly: {exc}",
                    file=sys.stderr,
                )
        if r2c1_observer_thread is not None:
            r2c1_observer_thread.join(timeout=2.0)
        if r2c1_observer_node is not None:
            try:
                r2c1_observer_node.destroy_node()
            except Exception as exc:
                print(
                    f"warning: failed to destroy R2C1 observer node cleanly: {exc}",
                    file=sys.stderr,
                )
        if runtime is not None:
            try:
                runtime.stop()
            except Exception as exc:
                print(f"warning: failed to stop simulation cleanly: {exc}", file=sys.stderr)
        if camera_graph_paths:
            try:
                from isaac_sim.graphs.camera_graph import destroy_camera_graphs

                destroy_camera_graphs(camera_graph_paths)
                # Camera Helper writer detach is queued by graph destruction.
                # Drain Kit work before deleting its Render Product owner.
                app.update()
                app.update()
            except Exception as exc:
                print(
                    f"warning: failed to destroy Camera graphs cleanly: {exc}",
                    file=sys.stderr,
                )
        if sensors is not None:
            try:
                sensors.close_camera_resources()
                app.update()
            except Exception as exc:
                print(
                    f"warning: failed to release Camera resources cleanly: {exc}",
                    file=sys.stderr,
                )
        if kidnap_bridge is not None:
            try:
                kidnap_bridge.close()
            except Exception as exc:
                print(
                    f"warning: failed to close kidnap bridge cleanly: {exc}",
                    file=sys.stderr,
                )
        if reset_bridge is not None:
            try:
                reset_bridge.close()
            except Exception as exc:
                print(
                    f"warning: failed to close reset bridge cleanly: {exc}",
                    file=sys.stderr,
                )
        if reset_stop_gate is not None:
            try:
                reset_stop_gate.close()
            except Exception as exc:
                print(
                    f"warning: failed to close reset stop gate cleanly: {exc}",
                    file=sys.stderr,
                )
        if appearance_manager is not None:
            try:
                appearance_manager.close()
            except Exception as exc:
                print(
                    f"warning: failed to remove appearance session layer cleanly: {exc}",
                    file=sys.stderr,
                )
        if node is not None:
            node.destroy_node()
        if odom_phase_trace is not None:
            odom_phase_trace.close()
        if imu_regime_phase_trace is not None:
            imu_regime_phase_trace.close()
        if r2c1_trace is not None:
            r2c1_trace.close()
        if r2c2_trace is not None:
            r2c2_trace.close()
        if r2c2a_trace is not None:
            r2c2a_trace.close()
        if rclpy_started:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        app.close(exit_code=1 if failed else 0)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    diagnostic_modes = [
        args.odom_phase_trace is not None,
        args.r2c1_free_space_trace is not None,
        args.r2c2_free_space_envelope is not None,
        args.r2c2a_free_space_envelope is not None,
        args.r2c3_free_space_motion_trace is not None,
        args.r2d2_live_pose_delta_trace is not None,
    ]
    if sum(diagnostic_modes) > 1:
        raise ValueError(
            "odom phase, R2C1, R2C2, R2C2A, R2C3 and R2D2 diagnostic modes "
            "are mutually exclusive"
        )
    if args.r2c2a_collision_bounds_config is not None and args.r2c2a_free_space_envelope is None:
        raise ValueError("--r2c2a-collision-bounds-config requires --r2c2a-free-space-envelope")
    if args.r2c2a_free_space_envelope is not None:
        if args.r2c2a_collision_bounds_config is None:
            raise ValueError("R2C2A requires --r2c2a-collision-bounds-config")
        if not args.r2c2a_collision_bounds_config.expanduser().resolve().is_file():
            raise ValueError(f"R2C2A collision-bounds config not found: {args.r2c2a_collision_bounds_config}")
    if (
        args.r2c3_collision_bounds_config is not None
        and args.r2c3_free_space_motion_trace is None
    ):
        raise ValueError(
            "--r2c3-collision-bounds-config requires "
            "--r2c3-free-space-motion-trace"
        )
    if args.r2c3_free_space_motion_trace is not None:
        if args.r2c3_collision_bounds_config is None:
            raise ValueError(
                "R2C3 requires --r2c3-collision-bounds-config"
            )
        if not (
            args.r2c3_collision_bounds_config.expanduser().resolve().is_file()
        ):
            raise ValueError(
                "R2C3 collision-bounds config not found: "
                f"{args.r2c3_collision_bounds_config}"
            )
    if (
        args.r2d2_collision_bounds_config is not None
        and args.r2d2_live_pose_delta_trace is None
    ):
        raise ValueError(
            "--r2d2-collision-bounds-config requires "
            "--r2d2-live-pose-delta-trace"
        )
    if args.r2d2_live_pose_delta_trace is not None:
        if args.r2d2_collision_bounds_config is None:
            raise ValueError("R2D2 requires --r2d2-collision-bounds-config")
        if not args.r2d2_collision_bounds_config.expanduser().resolve().is_file():
            raise ValueError(
                "R2D2 collision-bounds config not found: "
                f"{args.r2d2_collision_bounds_config}"
            )
    _apply_cli_overrides(args)
    config = load_project_config(args.config)
    appearance_config = args.appearance_config.expanduser().resolve()
    if not appearance_config.is_file():
        raise ValueError(f"appearance config not found: {appearance_config}")
    appearance_profiles = load_appearance_profiles(appearance_config)
    appearance_profiles.require(args.appearance_profile)
    if args.dynamic_obstacle_config is not None:
        obstacle_config = args.dynamic_obstacle_config.expanduser().resolve()
        if not obstacle_config.is_file():
            raise ValueError(f"dynamic obstacle config not found: {obstacle_config}")
        config = replace(
            config, files=replace(config.files, dynamic_obstacles=obstacle_config)
        )
    configure_process_environment(config)
    (
        selected_pose,
        dynamic_scenario,
        camera_selection,
        lio_config,
    ) = validate_configuration(
        config, args.camera_profile, args.lio_lidar_profile
    )
    if args.dynamic_obstacles is not None:
        dynamic_scenario = replace(
            dynamic_scenario, enabled=bool(args.dynamic_obstacles)
        )
    if args.imu_regime_phase_trace is not None:
        imu_regime_trace_provenance(
            config,
            dynamic_scenario,
            args.imu_regime_diagnostic_config.expanduser().resolve(),
        )
    if (
        args.r2c1_free_space_trace is not None
        or args.r2c2_free_space_envelope is not None
        or args.r2c2a_free_space_envelope is not None
        or args.r2c3_free_space_motion_trace is not None
        or args.r2d2_live_pose_delta_trace is not None
    ):
        if config.simulation.odometry_mode != "ideal":
            raise ValueError("R2C diagnostic modes require --mode ideal")
        if config.spawn.selected != "mapping_start":
            raise ValueError("R2C diagnostic modes require --spawn-pose mapping_start")
        if config.environment.source_asset.name != "kujiale_0026_A_to_B_door_open.usd":
            raise ValueError("R2C diagnostic modes require the frozen Kujiale source USD")
        if dynamic_scenario.enabled:
            raise ValueError("R2C diagnostic modes require --no-dynamic-obstacles")
        expected_camera_profile = (
            "monitoring"
            if args.r2d2_live_pose_delta_trace is not None
            else "off"
        )
        if camera_selection.profile.name != expected_camera_profile:
            raise ValueError(
                "R2D2 requires --camera-profile monitoring"
                if args.r2d2_live_pose_delta_trace is not None
                else "R2C diagnostic modes require --camera-profile off"
            )
        if config.third_person_camera.enabled:
            raise ValueError("R2C diagnostic modes require --no-third-person-camera")
        if args.appearance_profile != "baseline":
            raise ValueError("R2C diagnostic modes require --appearance-profile baseline")
    calibration = "calibrated" if selected_pose.map.calibrated else "uncalibrated"
    if args.validate_only:
        # This process exits immediately after validation, so importing pxr
        # without Kit is safe here.
        stage = SceneComposer(config).compose(save=False)
        validate_composed_stage(config, stage)
        if config.third_person_camera.enabled:
            from isaac_sim.src.visualization.third_person_camera import (
                ThirdPersonCamera,
            )

            ThirdPersonCamera(
                stage,
                config.robot.base_link_prim,
                config.third_person_camera,
                activate_viewport=False,
            )
        print(
            "validation: PASS "
            f"(navigation={config.simulation.navigation_mode}, "
            f"odometry={config.simulation.odometry_mode}, "
            f"structure_tf={config.simulation.structure_tf_source}, "
            f"environment={config.environment.source_asset.name}, "
            f"spawn={config.spawn.selected}, "
            f"dynamic_obstacles={dynamic_scenario.enabled}, "
            f"third_person_camera={config.third_person_camera.enabled}, "
            f"lio_lidar={('off' if lio_config is None else lio_config['profile_name'])}, "
            f"{calibration})"
        )
        return 0
    run(
        config,
        selected_pose,
        dynamic_scenario,
        camera_selection,
        appearance_profiles,
        args.appearance_profile,
        args.dynamic_case_id,
        args.dynamic_variant_id,
        args.dynamic_seed,
        None if args.odom_phase_trace is None else args.odom_phase_trace.expanduser().resolve(),
        None if args.r2c1_free_space_trace is None else args.r2c1_free_space_trace.expanduser().resolve(),
        None if args.r2c2_free_space_envelope is None else args.r2c2_free_space_envelope.expanduser().resolve(),
        None if args.r2c2a_free_space_envelope is None else args.r2c2a_free_space_envelope.expanduser().resolve(),
        None if args.r2c2a_collision_bounds_config is None else args.r2c2a_collision_bounds_config.expanduser().resolve(),
        None if args.r2c3_free_space_motion_trace is None else args.r2c3_free_space_motion_trace.expanduser().resolve(),
        None if args.r2c3_collision_bounds_config is None else args.r2c3_collision_bounds_config.expanduser().resolve(),
        None if args.r2d2_live_pose_delta_trace is None else args.r2d2_live_pose_delta_trace.expanduser().resolve(),
        None if args.r2d2_collision_bounds_config is None else args.r2d2_collision_bounds_config.expanduser().resolve(),
        (
            None
            if args.imu_regime_phase_trace is None
            else args.imu_regime_phase_trace.expanduser().resolve()
        ),
        args.imu_regime_diagnostic_config.expanduser().resolve(),
        lio_config=lio_config,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
