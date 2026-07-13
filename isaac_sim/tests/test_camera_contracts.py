from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from isaac_sim.apps.navigation_sim import _parser, _simulation_app_config
from isaac_sim.graphs.camera_graph import (
    camera_graph_spec,
    validate_camera_ros_contract,
)
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


def _camera_runtime(camera_config, profile="monitoring", render_product=None):
    camera = camera_config.cameras[camera_config.primary_camera]
    selected = camera_config.profiles[profile]
    return CameraRuntime(
        name=camera.name,
        profile_name=selected.name,
        sensor=object(),
        camera_prim_path=camera.sensor_prim,
        render_product=render_product or object(),
        render_product_path="/Render/CameraFront",
        graph_path="/World/Graphs/ROS2CameraFront",
        optical_frame=camera.optical_frame,
        node_namespace=camera.node_namespace,
        rgb=camera.rgb,
        camera_info=camera.camera_info,
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
    front = camera.cameras["front"]
    assert front.sensor_prim.endswith(
        "/camera_front_link/camera_front_optical_frame/camera_front_sensor"
    )
    assert front.optical_frame == "camera_front_optical_frame"
    assert front.node_namespace == "/camera/front"
    assert front.rgb.encoding == "rgb8"
    assert front.rgb.queue_size == front.camera_info.queue_size == 2
    assert front.depth.enabled is False


def test_camera_defaults_are_gui_monitoring_and_headless_off():
    camera = _load_camera(_config().files.camera)

    gui = resolve_camera_selection(camera, None, headless=False)
    headless = resolve_camera_selection(camera, None, headless=True)
    explicit = resolve_camera_selection(camera, "standard", headless=True)

    assert gui.profile.name == "monitoring" and gui.camera is not None
    assert headless.profile.name == "off" and headless.camera is None
    assert explicit.profile.name == "standard" and explicit.camera is not None
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


def test_camera_graph_publishes_rgb_and_info_from_one_render_product():
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
    }
    assert values["PublishRGB.inputs:renderProductPath"] == (
        values["PublishCameraInfo.inputs:renderProductPath"]
    )
    assert values["PublishRGB.inputs:type"] == "rgb"
    assert values["PublishRGB.inputs:useSystemTime"] is False
    assert values["PublishCameraInfo.inputs:useSystemTime"] is False
    assert values["PublishRGB.inputs:frameId"] == contract.optical_frame
    assert values["PublishCameraInfo.inputs:frameId"] == contract.optical_frame
    assert values["PublishRGB.inputs:queueSize"] == 2
    assert values["PublishCameraInfo.inputs:queueSize"] == 2
    assert '"depth":2' in contract.qos_profile
    assert '"reliability":"bestEffort"' in contract.qos_profile
    assert '"durability":"volatile"' in contract.qos_profile


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


def test_camera_cli_accepts_only_named_profiles():
    parser = _parser()

    assert parser.parse_args([]).camera_profile is None
    assert parser.parse_args(
        ["--headless", "--camera-profile", "standard"]
    ).camera_profile == "standard"
    with pytest.raises(SystemExit):
        parser.parse_args(["--camera-profile", "turbo"])


def test_simulation_app_enables_supported_multitick_sensor_settings_early():
    launch = _simulation_app_config(_config())

    assert launch["multi_gpu"] is False
    assert launch["extra_args"] == [
        "--/rtx/hydra/supportMultiTickRate=true",
        "--/persistent/simulation/minFrameRate=60",
    ]
