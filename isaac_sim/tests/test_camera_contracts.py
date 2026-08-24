from __future__ import annotations

import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

from isaac_sim.apps.navigation_sim import (
    _apply_cli_overrides,
    _parser,
    _simulation_app_config,
    run,
)
from isaac_sim.graphs.camera_graph import (
    camera_graph_spec,
    destroy_camera_graphs,
    stereo_camera_graph_spec,
    validate_camera_ros_contract,
)
from isaac_sim.graphs.ros_contract import load_topics
from isaac_sim.src.config import load_project_config
from isaac_sim.src.sensors.sensor_factory import (
    CAMERA_PROFILE_NAMES,
    CameraRuntime,
    SensorConfigError,
    _load_camera,
    resolve_camera_selection,
)


ROOT = Path(__file__).resolve().parents[2]


def _config():
    return load_project_config(
        ROOT / "isaac_sim/configs/project.yaml",
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": "/home/lyb/isaacsim_assets/Assets/Isaac/6.0",
        },
    )


def _camera_runtime(
    camera_config,
    profile="monitoring",
    render_product=None,
    camera_name="front",
):
    camera = camera_config.cameras[camera_name]
    selected = camera_config.profiles[profile]
    return CameraRuntime(
        name=camera.name,
        profile_name=selected.name,
        sensor=object(),
        camera_prim_path=camera.sensor_prim,
        render_product=render_product or object(),
        render_product_path=f"/Render/Camera{camera.name.title()}",
        graph_path=(
            "/World/Graphs/ROS2StereoCamera"
            if profile == "stereo_vio"
            else "/World/Graphs/ROS2CameraFront"
        ),
        optical_frame=camera.optical_frame,
        node_namespace=camera.node_namespace,
        rgb=camera.rgb,
        camera_info=camera.camera_info,
        depth=camera.depth,
        depth_points=camera.depth_points,
        depth_points_enabled=(
            selected.depth_points_enabled and camera.depth_points.enabled
        ),
        width=selected.width,
        height=selected.height,
        publish_rate_hz=selected.publish_rate_hz,
    )


def test_camera_schema_profiles_and_front_contract_load_strictly():
    camera = _load_camera(_config().files.camera)

    assert tuple(camera.profiles) == CAMERA_PROFILE_NAMES
    assert camera.default_profile == "monitoring"
    assert camera.primary_camera == "front"
    assert (
        camera.profiles["monitoring"].width,
        camera.profiles["monitoring"].height,
        camera.profiles["monitoring"].publish_rate_hz,
    ) == (640, 360, 15.0)
    assert (
        camera.profiles["standard"].width,
        camera.profiles["standard"].height,
        camera.profiles["standard"].publish_rate_hz,
    ) == (640, 480, 20.0)
    assert (
        camera.profiles["high_quality"].width,
        camera.profiles["high_quality"].height,
        camera.profiles["high_quality"].publish_rate_hz,
    ) == (1280, 720, 30.0)
    assert (
        camera.profiles["rgbd_navigation"].width,
        camera.profiles["rgbd_navigation"].height,
        camera.profiles["rgbd_navigation"].publish_rate_hz,
        camera.profiles["rgbd_navigation"].depth_points_enabled,
    ) == (320, 180, 10.0, True)
    assert (
        camera.profiles["stereo_vio"].width,
        camera.profiles["stereo_vio"].height,
        camera.profiles["stereo_vio"].publish_rate_hz,
        camera.profiles["stereo_vio"].depth_points_enabled,
    ) == (640, 360, 20.0, False)
    front = camera.cameras["front"]
    assert front.sensor_prim.endswith(
        "/camera_front_link/camera_front_optical_frame/camera_front_sensor"
    )
    assert front.optical_frame == "camera_front_optical_frame"
    assert front.node_namespace == "/camera/front"
    assert front.rgb.encoding == "rgb8"
    assert front.rgb.queue_size == front.camera_info.queue_size == 2
    assert front.depth.enabled is True
    assert front.depth_points.enabled is True
    assert front.depth_points.topic_name == "depth/points"
    assert front.depth_points.queue_size == 2
    left = camera.cameras["left"]
    right = camera.cameras["right"]
    assert left.optical_frame == "camera_left_optical_frame"
    assert right.optical_frame == "camera_right_optical_frame"
    assert left.node_namespace == "/camera/left"
    assert right.node_namespace == "/camera/right"
    assert left.depth.enabled is True
    assert right.depth.enabled is False
    assert left.depth_points.enabled is right.depth_points.enabled is False
    for field in (
        "clipping_range_m", "projection", "focal_length_mm",
        "horizontal_aperture_mm", "vertical_aperture_mode",
        "focus_distance_m", "exposure_enabled", "exposure_time_s",
        "exposure_responsivity", "exposure_f_stop",
    ):
        assert getattr(left, field) == getattr(right, field)


def test_camera_defaults_are_gui_monitoring_and_headless_off():
    camera = _load_camera(_config().files.camera)

    gui = resolve_camera_selection(camera, None, headless=False)
    headless = resolve_camera_selection(camera, None, headless=True)
    explicit = resolve_camera_selection(camera, "standard", headless=True)
    stereo = resolve_camera_selection(camera, "stereo_vio", headless=True)

    assert gui.profile.name == "monitoring"
    assert tuple(item.name for item in gui.cameras) == ("front",)
    assert headless.profile.name == "off" and headless.cameras == ()
    assert explicit.profile.name == "standard"
    assert tuple(item.name for item in explicit.cameras) == ("front",)
    assert stereo.profile.name == "stereo_vio"
    assert tuple(item.name for item in stereo.cameras) == ("left", "right")
    assert "front" not in {item.name for item in stereo.cameras}
    with pytest.raises(SensorConfigError, match="unknown camera profile"):
        resolve_camera_selection(camera, "turbo", headless=False)


def test_camera_schema_rejects_profile_drift_and_unknown_keys(tmp_path):
    source = yaml.safe_load(_config().files.camera.read_text(encoding="utf-8"))
    source["profiles"]["monitoring"]["publish_rate_hz"] = 12.0
    profile_drift = tmp_path / "profile_drift.yaml"
    profile_drift.write_text(yaml.safe_dump(source), encoding="utf-8")
    with pytest.raises(SensorConfigError, match="verified contract"):
        _load_camera(profile_drift)

    source["profiles"]["monitoring"]["publish_rate_hz"] = 15.0
    source["cameras"]["front"]["typo"] = True
    unknown = tmp_path / "unknown.yaml"
    unknown.write_text(yaml.safe_dump(source), encoding="utf-8")
    with pytest.raises(ValueError, match="unknown .* keys"):
        _load_camera(unknown)


def test_camera_graph_publishes_rgb_info_and_depth_from_one_render_product():
    config = _config()
    camera_config = _load_camera(config.files.camera)
    runtime = _camera_runtime(camera_config)
    contract = validate_camera_ros_contract(config, camera_config)
    spec = camera_graph_spec(config, runtime)
    spec.validate()

    nodes = dict(spec.nodes)
    values = dict(spec.values)
    assert nodes == {
        "OnPlaybackTick": "omni.graph.action.OnPlaybackTick",
        "PublishRGB": "isaacsim.ros2.bridge.ROS2CameraHelper",
        "PublishCameraInfo": "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
        "PublishDepth": "isaacsim.ros2.bridge.ROS2CameraHelper",
    }
    assert values["PublishRGB.inputs:renderProductPath"] == (
        values["PublishCameraInfo.inputs:renderProductPath"]
    )
    assert values["PublishDepth.inputs:renderProductPath"] == (
        values["PublishRGB.inputs:renderProductPath"]
    )
    assert values["PublishRGB.inputs:type"] == "rgb"
    assert values["PublishRGB.inputs:useSystemTime"] is False
    assert values["PublishCameraInfo.inputs:useSystemTime"] is False
    assert values["PublishDepth.inputs:useSystemTime"] is False
    assert values["PublishRGB.inputs:frameId"] == contract.optical_frame
    assert values["PublishCameraInfo.inputs:frameId"] == contract.optical_frame
    assert values["PublishDepth.inputs:frameId"] == contract.optical_frame
    assert values["PublishRGB.inputs:queueSize"] == 2
    assert values["PublishCameraInfo.inputs:queueSize"] == 2
    assert values["PublishDepth.inputs:queueSize"] == 2
    assert values["PublishDepth.inputs:type"] == "depth"
    assert '"depth":2' in contract.qos_profile
    assert '"reliability":"bestEffort"' in contract.qos_profile
    assert '"durability":"volatile"' in contract.qos_profile


def test_rgbd_navigation_graph_publishes_depth_points_from_shared_render_product():
    config = _config()
    camera_config = _load_camera(config.files.camera)
    runtime = _camera_runtime(camera_config, profile="rgbd_navigation")
    spec = camera_graph_spec(config, runtime)
    spec.validate()

    nodes = dict(spec.nodes)
    values = dict(spec.values)
    assert nodes["PublishDepthPoints"] == "isaacsim.ros2.bridge.ROS2CameraHelper"
    assert values["PublishDepthPoints.inputs:renderProductPath"] == (
        values["PublishRGB.inputs:renderProductPath"]
    )
    assert values["PublishDepthPoints.inputs:type"] == "depth_pcl"
    assert values["PublishDepthPoints.inputs:topicName"] == "depth/points"
    assert values["PublishDepthPoints.inputs:frameId"] == "camera_front_optical_frame"
    assert values["PublishDepthPoints.inputs:queueSize"] == 2
    assert values["PublishDepthPoints.inputs:useSystemTime"] is False


def test_stereo_graph_has_independent_branches_one_tick_and_paired_info():
    config = _config()
    camera_config = _load_camera(config.files.camera)
    left = _camera_runtime(camera_config, "stereo_vio", camera_name="left")
    right = _camera_runtime(camera_config, "stereo_vio", camera_name="right")
    spec = stereo_camera_graph_spec(config, (left, right))
    spec.validate()

    nodes = dict(spec.nodes)
    values = dict(spec.values)
    assert left.camera_prim_path != right.camera_prim_path
    assert left.render_product_path != right.render_product_path
    assert list(nodes.values()).count("omni.graph.action.OnPlaybackTick") == 1
    assert nodes == {
        "OnPlaybackTick": "omni.graph.action.OnPlaybackTick",
        "PublishLeftRGB": "isaacsim.ros2.bridge.ROS2CameraHelper",
        "PublishRightRGB": "isaacsim.ros2.bridge.ROS2CameraHelper",
        "PublishLeftDepth": "isaacsim.ros2.bridge.ROS2CameraHelper",
        "PublishStereoCameraInfo": (
            "isaacsim.ros2.bridge.ROS2CameraInfoHelper"
        ),
    }
    assert set(spec.connections) == {
        ("OnPlaybackTick.outputs:tick", f"{node}.inputs:execIn")
        for node in nodes if node != "OnPlaybackTick"
    }
    assert values["PublishLeftRGB.inputs:renderProductPath"] == (
        values["PublishLeftDepth.inputs:renderProductPath"]
    )
    assert values["PublishRightRGB.inputs:renderProductPath"] != (
        values["PublishLeftRGB.inputs:renderProductPath"]
    )
    assert "PublishRightDepth" not in nodes
    assert values["PublishStereoCameraInfo.inputs:renderProductPath"] == (
        left.render_product_path
    )
    assert values[
        "PublishStereoCameraInfo.inputs:renderProductPathRight"
    ] == right.render_product_path
    assert values["PublishStereoCameraInfo.inputs:frameId"] == (
        "camera_left_optical_frame"
    )
    assert values["PublishStereoCameraInfo.inputs:frameIdRight"] == (
        "camera_right_optical_frame"
    )
    assert values["PublishStereoCameraInfo.inputs:topicName"] == (
        "/camera/left/camera_info"
    )
    assert values["PublishStereoCameraInfo.inputs:topicNameRight"] == (
        "/camera/right/camera_info"
    )
    assert left.width == right.width == 640
    assert left.height == right.height == 360
    assert left.publish_rate_hz == right.publish_rate_hz == 20.0


def test_stereo_topics_frames_and_static_projection_baseline_contract():
    config = _config()
    camera_config = _load_camera(config.files.camera)
    topics = load_topics(config.files.topics)
    left_contract = validate_camera_ros_contract(config, camera_config, "left")
    right_contract = validate_camera_ros_contract(config, camera_config, "right")

    assert left_contract.image_topic == topics["camera_left_image"]
    assert left_contract.camera_info_topic == topics["camera_left_info"]
    assert left_contract.depth_topic == topics["camera_left_depth"]
    assert right_contract.image_topic == topics["camera_right_image"]
    assert right_contract.camera_info_topic == topics["camera_right_info"]
    assert right_contract.depth_topic is None
    assert left_contract.optical_frame == topics["frames"]["camera_left_optical"]
    assert right_contract.optical_frame == topics["frames"]["camera_right_optical"]

    robot = yaml.safe_load(config.files.robot.read_text(encoding="utf-8"))
    translations = {
        (item["parent"], item["child"]): item["translation"]
        for item in robot["static_transforms"]
    }
    left_y = translations[("camera_link", "camera_left_link")][1]
    right_y = translations[("camera_link", "camera_right_link")][1]
    baseline_m = left_y - right_y
    assert (left_y, right_y, baseline_m) == pytest.approx((0.060, -0.060, 0.12))
    fx_px = (
        camera_config.cameras["left"].focal_length_mm
        / camera_config.cameras["left"].horizontal_aperture_mm
        * camera_config.profiles["stereo_vio"].width
    )
    expected_left_p3 = 0.0
    expected_right_p3 = -fx_px * baseline_m
    assert expected_left_p3 == 0.0
    assert expected_right_p3 == pytest.approx(-fx_px * 0.12)


def test_camera_render_product_owner_releases_exactly_once():
    class RenderProduct:
        def __init__(self):
            self.destroy_calls = 0

        def destroy(self):
            self.destroy_calls += 1

    camera_config = _load_camera(_config().files.camera)
    owner = RenderProduct()
    runtime = _camera_runtime(camera_config, render_product=owner)

    runtime.release_render_product()
    runtime.release_render_product()

    assert owner.destroy_calls == 1


def test_stereo_render_product_owners_release_independently_exactly_once():
    class RenderProduct:
        def __init__(self):
            self.destroy_calls = 0

        def destroy(self):
            self.destroy_calls += 1

    camera_config = _load_camera(_config().files.camera)
    left_owner = RenderProduct()
    right_owner = RenderProduct()
    left = _camera_runtime(
        camera_config, "stereo_vio", left_owner, camera_name="left"
    )
    right = _camera_runtime(
        camera_config, "stereo_vio", right_owner, camera_name="right"
    )

    for runtime in (left, right):
        runtime.release_render_product()
        runtime.release_render_product()

    assert left_owner.destroy_calls == right_owner.destroy_calls == 1


def test_stereo_shared_graph_cleanup_removes_graph_once(monkeypatch):
    removed = []

    class Prim:
        def IsValid(self):
            return True

    stage = SimpleNamespace(
        GetPrimAtPath=lambda _path: Prim(),
        RemovePrim=lambda path: removed.append(path),
    )
    usd = SimpleNamespace(
        get_context=lambda: SimpleNamespace(get_stage=lambda: stage)
    )
    omni = SimpleNamespace(usd=usd)
    monkeypatch.setitem(sys.modules, "omni", omni)
    monkeypatch.setitem(sys.modules, "omni.usd", usd)

    graph_path = "/World/Graphs/ROS2StereoCamera"
    destroy_camera_graphs((graph_path, graph_path))

    assert removed == [graph_path]


def test_camera_cli_accepts_only_named_profiles():
    parser = _parser()

    assert parser.parse_args([]).camera_profile is None
    assert parser.parse_args(
        ["--headless", "--camera-profile", "standard"]
    ).camera_profile == "standard"
    assert parser.parse_args(
        ["--headless", "--camera-profile", "stereo_vio"]
    ).camera_profile == "stereo_vio"
    with pytest.raises(SystemExit):
        parser.parse_args(["--camera-profile", "turbo"])


def test_stage_readiness_cli_uses_typed_config_override(monkeypatch):
    args = _parser().parse_args(
        ["--stage-readiness-timeout-s", "135.5"]
    )
    monkeypatch.delenv(
        "ISAAC_NAV__SIMULATION__STAGE_READINESS_TIMEOUT_S", raising=False
    )

    _apply_cli_overrides(args)

    assert (
        os.environ["ISAAC_NAV__SIMULATION__STAGE_READINESS_TIMEOUT_S"]
        == "135.5"
    )


def test_simulation_app_enables_supported_multitick_sensor_settings_early():
    launch = _simulation_app_config(_config())

    assert launch["multi_gpu"] is False
    assert launch["extra_args"] == [
        "--/renderer/raytracingMotion/enabled=true",
        "--/renderer/raytracingMotion/enableHydraEngineMasking=true",
        "--/renderer/raytracingMotion/enabledForHydraEngines=0,1,2,3",
        "--/rtx/rendering/perSensorTickTlas=true",
        "--/rtx/hydra/supportMultiTickRate=true",
        "--/persistent/simulation/minFrameRate=60",
    ]


def test_run_passes_single_gpu_launch_contract_to_simulation_app(monkeypatch):
    captured = {}

    class StopAfterConstruction(RuntimeError):
        pass

    class FakeSimulationApp:
        def __init__(self, launch):
            captured.update(launch)
            raise StopAfterConstruction

    monkeypatch.setitem(
        sys.modules,
        "isaacsim",
        SimpleNamespace(SimulationApp=FakeSimulationApp),
    )
    with pytest.raises(StopAfterConstruction):
        run(_config(), None, None, None, None, "baseline")

    assert captured["multi_gpu"] is False
    assert captured == _simulation_app_config(_config())
