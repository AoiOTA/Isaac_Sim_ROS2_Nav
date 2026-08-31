from __future__ import annotations

import inspect
import hashlib
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

from isaac_sim.apps import navigation_sim
from isaac_sim.apps.navigation_sim import (
    _apply_cli_overrides,
    _create_paired_appearance_capture,
    _parser,
    _publish_viewport_runtime_attestation,
    _simulation_app_config,
    _validate_viewport_command_contract,
    _verify_default_viewport_updates,
    _verify_rtx_descriptor_sets,
    run,
)
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
        depth=camera.depth,
        depth_points=camera.depth_points,
        depth_points_enabled=selected.depth_points_enabled,
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

    assert parser.allow_abbrev is False
    assert parser.parse_args([]).camera_profile is None
    assert parser.parse_args([]).paired_appearance_capture is False
    assert (
        inspect.signature(run)
        .parameters["paired_appearance_capture_enabled"]
        .default
        is False
    )
    assert (
        inspect.signature(run)
        .parameters["disable_viewport_updates"]
        .default
        is False
    )
    assert parser.parse_args([]).disable_dlss is False
    assert parser.parse_args([]).disable_viewport_updates is False
    assert parser.parse_args([]).rtx_descriptor_sets is None
    assert parser.parse_args(["--disable-dlss"]).disable_dlss is True
    assert parser.parse_args(["--no-disable-dlss"]).disable_dlss is False
    assert (
        parser.parse_args(
            ["--disable-viewport-updates"]
        ).disable_viewport_updates
        is True
    )
    assert (
        parser.parse_args(
            ["--no-disable-viewport-updates"]
        ).disable_viewport_updates
        is False
    )
    assert (
        parser.parse_args(
            ["--paired-appearance-capture"]
        ).paired_appearance_capture
        is True
    )
    assert (
        parser.parse_args(
            ["--no-paired-appearance-capture"]
        ).paired_appearance_capture
        is False
    )
    assert parser.parse_args(
        ["--rtx-descriptor-sets", "20000"]
    ).rtx_descriptor_sets == 20000
    assert parser.parse_args(
        ["--headless", "--camera-profile", "standard"]
    ).camera_profile == "standard"
    with pytest.raises(SystemExit):
        parser.parse_args(["--camera-profile", "turbo"])
    for invalid in ("0", "-1", "not-an-int"):
        with pytest.raises(SystemExit):
            parser.parse_args(["--rtx-descriptor-sets", invalid])
    for abbreviated in ("--rtx-descriptor-set", "--rtx-desc"):
        with pytest.raises(SystemExit):
            parser.parse_args([abbreviated, "10000"])
    with pytest.raises(SystemExit):
        parser.parse_args(["--disable-viewport-update"])


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
    dlss_off_args = [
        "--/rtx/post/aa/op=0",
        "--/rtx-defaults/post/aa/op=0",
        "--/rtx-transient/post/aa/limitedOps=false",
    ]

    assert launch["multi_gpu"] is False
    assert launch["disable_viewport_updates"] is False
    assert "anti_aliasing" not in launch
    assert launch["extra_args"] == [
        "--/renderer/raytracingMotion/enabled=true",
        "--/renderer/raytracingMotion/enableHydraEngineMasking=true",
        "--/renderer/raytracingMotion/enabledForHydraEngines=0,1,2,3",
        "--/rtx/rendering/perSensorTickTlas=true",
        "--/rtx/hydra/supportMultiTickRate=true",
        "--/persistent/simulation/minFrameRate=60",
    ]
    assert all(arg not in launch["extra_args"] for arg in dlss_off_args)

    dlss_disabled = _simulation_app_config(_config(), disable_dlss=True)
    assert dlss_disabled["anti_aliasing"] == 0
    assert dlss_disabled["extra_args"] == launch["extra_args"] + dlss_off_args

    descriptor_override = _simulation_app_config(
        _config(), rtx_descriptor_sets=20000
    )
    assert descriptor_override["extra_args"] == launch["extra_args"] + [
        "--/rtx/descriptorSets=20000"
    ]
    assert sum(
        argument.startswith("--/rtx/descriptorSets=")
        for argument in descriptor_override["extra_args"]
    ) == 1

    viewport_disabled = _simulation_app_config(
        _config(), disable_viewport_updates=True
    )
    assert viewport_disabled["disable_viewport_updates"] is True
    assert viewport_disabled["extra_args"] == launch["extra_args"]


def test_rtx_descriptor_sets_verification_reports_and_fails_closed(
    monkeypatch, capsys
):
    class Settings:
        def __init__(self, applied):
            self.applied = applied

        def get_as_int(self, path):
            assert path == "/rtx/descriptorSets"
            return self.applied

    applied = Settings(20000)
    monkeypatch.setitem(
        sys.modules,
        "carb",
        SimpleNamespace(
            settings=SimpleNamespace(get_settings=lambda: applied)
        ),
    )

    _verify_rtx_descriptor_sets(20000)
    assert capsys.readouterr().out == (
        "RTX_DESCRIPTOR_SETS requested=20000 applied=20000\n"
    )

    applied.applied = 10000
    with pytest.raises(RuntimeError, match="requested=20000 applied=10000"):
        _verify_rtx_descriptor_sets(20000)
    assert capsys.readouterr().out == (
        "RTX_DESCRIPTOR_SETS requested=20000 applied=10000\n"
    )


def _install_fake_active_viewport(monkeypatch, viewport):
    monkeypatch.setitem(sys.modules, "omni", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "omni.kit", SimpleNamespace())
    monkeypatch.setitem(
        sys.modules, "omni.kit.viewport", SimpleNamespace()
    )
    monkeypatch.setitem(
        sys.modules,
        "omni.kit.viewport.utility",
        SimpleNamespace(get_active_viewport=lambda: viewport),
    )


@pytest.mark.parametrize(
    ("requested_disabled", "observed_enabled"),
    ((False, True), (True, False)),
)
def test_viewport_verification_reports_live_match_for_both_arms(
    monkeypatch, capsys, requested_disabled, observed_enabled
):
    _install_fake_active_viewport(
        monkeypatch, SimpleNamespace(updates_enabled=observed_enabled)
    )

    _verify_default_viewport_updates(
        headless=True, requested_disabled=requested_disabled
    )

    assert capsys.readouterr().out == (
        "DEFAULT_VIEWPORT_UPDATES headless=True "
        f"requested_disabled={requested_disabled!r} "
        f"observed_enabled={observed_enabled!r} match=True\n"
    )


@pytest.mark.parametrize("observed_enabled", (None, 0, "false", True))
def test_viewport_verification_fails_closed_on_missing_nonbool_or_mismatch(
    monkeypatch, capsys, observed_enabled
):
    viewport = (
        None
        if observed_enabled is None
        else SimpleNamespace(updates_enabled=observed_enabled)
    )
    _install_fake_active_viewport(monkeypatch, viewport)

    with pytest.raises(RuntimeError, match="viewport updates contract mismatch"):
        _verify_default_viewport_updates(
            headless=True, requested_disabled=True
        )

    assert "match=False" in capsys.readouterr().out


def test_viewport_verification_rejects_gui_disable(monkeypatch, capsys):
    _install_fake_active_viewport(
        monkeypatch, SimpleNamespace(updates_enabled=False)
    )

    with pytest.raises(RuntimeError, match="headless=False"):
        _verify_default_viewport_updates(
            headless=False, requested_disabled=True
        )

    assert "match=False" in capsys.readouterr().out


def test_viewport_runtime_attestation_is_atomic_0600_and_process_bound(
    tmp_path, monkeypatch
):
    winner = tmp_path / "winner.json"
    winner.write_text(json.dumps({"winner": {"viewport_arm": "B"}}))
    output = tmp_path / "viewport_runtime_attestation.json"
    winner_digest = hashlib.sha256(winner.read_bytes()).hexdigest()
    monkeypatch.setattr(
        navigation_sim,
        "_self_process_identity",
        lambda: {
            "pid": 123,
            "pgid": 120,
            "start_ticks": 456,
            "boot_id": "boot",
            "cmdline_sha256": "c" * 64,
            "executable": "/python",
        },
    )
    monkeypatch.setattr(
        navigation_sim,
        "_process_cmdline_tokens",
        lambda _pid: [
            "/python", str(Path(navigation_sim.__file__).resolve()),
            "--disable-viewport-updates",
            "--viewport-arm-identity", "B",
            "--viewport-runtime-attestation", str(output.resolve()),
            "--viewport-winner-manifest", str(winner.resolve()),
            "--viewport-winner-manifest-sha256", winner_digest,
            "--viewport-run-root", str(tmp_path.resolve()),
            "--viewport-scene", "rivermark:static",
            "--viewport-launcher", str(Path(__file__).resolve()),
        ],
    )
    payload = _publish_viewport_runtime_attestation(
        output_path=output,
        instance_uuid="550e8400-e29b-41d4-a716-446655440000",
        start_wall_time_ns=123456789,
        scene="rivermark:static",
        run_root=tmp_path,
        launcher_path=Path(__file__),
        winner_manifest_path=winner,
        winner_manifest_sha256=winner_digest,
        readbacks=(
            {
                "phase": "post_construction",
                "requested_disabled": True,
                "observed_enabled": False,
                "match": True,
            },
            {
                "phase": "pre_ready",
                "requested_disabled": True,
                "observed_enabled": False,
                "match": True,
            },
        ),
    )
    assert output.stat().st_mode & 0o777 == 0o600
    assert json.loads(output.read_text()) == payload
    assert payload["pid"] == 123
    assert payload["module3"]["head"]
    with pytest.raises(RuntimeError, match="invalid or reused"):
        _publish_viewport_runtime_attestation(
            output_path=output,
            instance_uuid=payload["instance_uuid"],
            start_wall_time_ns=123456789,
            scene="rivermark:static",
            run_root=tmp_path,
            launcher_path=Path(__file__),
            winner_manifest_path=winner,
            winner_manifest_sha256=winner_digest,
            readbacks=payload["readbacks"],
        )


@pytest.mark.parametrize(
    "mutation",
    ("arm_a", "missing_disable", "attestation", "digest", "run_root", "scene", "launcher"),
)
def test_viewport_command_contract_rejects_arm_missing_flag_or_wrong_binding(
    tmp_path, mutation
):
    output = tmp_path / "viewport_runtime_attestation.json"
    winner = tmp_path / "winner.json"
    winner.write_text('{"winner":{"viewport_arm":"B"}}')
    digest = hashlib.sha256(winner.read_bytes()).hexdigest()
    launcher = tmp_path / "launcher.sh"
    launcher.write_text("#!/bin/sh\n")
    values = {
        "--viewport-arm-identity": "B",
        "--viewport-runtime-attestation": str(output.resolve()),
        "--viewport-winner-manifest": str(winner.resolve()),
        "--viewport-winner-manifest-sha256": digest,
        "--viewport-run-root": str(tmp_path.resolve()),
        "--viewport-scene": "rivermark:static",
        "--viewport-launcher": str(launcher.resolve()),
    }
    tokens = [str(Path(navigation_sim.__file__).resolve()), "--disable-viewport-updates"]
    for flag, value in values.items():
        tokens.extend([flag, value])
    if mutation == "arm_a":
        tokens[tokens.index("--viewport-arm-identity") + 1] = "A"
    elif mutation == "missing_disable":
        tokens.remove("--disable-viewport-updates")
    else:
        flag = {
            "attestation": "--viewport-runtime-attestation",
            "digest": "--viewport-winner-manifest-sha256",
            "run_root": "--viewport-run-root",
            "scene": "--viewport-scene",
            "launcher": "--viewport-launcher",
        }[mutation]
        tokens[tokens.index(flag) + 1] = "wrong"
    with pytest.raises(RuntimeError, match="viewport runtime command"):
        _validate_viewport_command_contract(
            tokens,
            output_path=output,
            winner_manifest_path=winner,
            winner_manifest_sha256=digest,
            run_root=tmp_path,
            scene="rivermark:static",
            launcher_path=launcher,
        )


def test_viewport_verification_runs_after_construction_and_before_ready():
    source = inspect.getsource(run)

    first = source.index("_verify_default_viewport_updates(")
    ready = source.index('"Isaac navigation simulation ready: "')
    second = source.rindex("_verify_default_viewport_updates(", 0, ready)

    assert first < second < ready


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
        run(
            _config(),
            None,
            None,
            None,
            None,
            "baseline",
            disable_dlss=True,
            rtx_descriptor_sets=20000,
            disable_viewport_updates=True,
        )

    assert captured["multi_gpu"] is False
    assert captured["anti_aliasing"] == 0
    assert captured["disable_viewport_updates"] is True
    assert captured == _simulation_app_config(
        _config(),
        disable_dlss=True,
        rtx_descriptor_sets=20000,
        disable_viewport_updates=True,
    )


def test_paired_appearance_capture_is_default_off_and_lazy(monkeypatch):
    def reject_import(name, *args, **kwargs):
        if name == "isaac_sim.src.experiment.paired_appearance":
            raise AssertionError("paired appearance capture imported while disabled")
        return original_import(name, *args, **kwargs)

    original_import = __import__
    monkeypatch.setattr("builtins.__import__", reject_import)

    assert _create_paired_appearance_capture(
        enabled=False,
        sensors=SimpleNamespace(cameras=[object()]),
        node=object(),
        appearance_manager=object(),
        appearance_profiles=object(),
    ) is None


def test_paired_appearance_capture_explicit_enable_uses_front_camera(monkeypatch):
    captured = {}

    class FakeCapture:
        def __init__(self, **kwargs):
            captured.update(kwargs)

    monkeypatch.setitem(
        sys.modules,
        "isaac_sim.src.experiment.paired_appearance",
        SimpleNamespace(PairedAppearanceCapture=FakeCapture),
    )
    camera = SimpleNamespace(
        render_product=object(), width=320, height=180
    )
    node = object()
    appearance_manager = object()
    appearance_profiles = object()

    result = _create_paired_appearance_capture(
        enabled=True,
        sensors=SimpleNamespace(cameras=[camera]),
        node=node,
        appearance_manager=appearance_manager,
        appearance_profiles=appearance_profiles,
    )

    assert isinstance(result, FakeCapture)
    assert captured == {
        "node": node,
        "render_product": camera.render_product,
        "appearance_manager": appearance_manager,
        "appearance_profiles": appearance_profiles,
        "width": 320,
        "height": 180,
    }


def test_paired_appearance_capture_enabled_without_camera_fails_closed():
    with pytest.raises(
        RuntimeError,
        match="requires an enabled camera profile",
    ):
        _create_paired_appearance_capture(
            enabled=True,
            sensors=SimpleNamespace(cameras=[]),
            node=object(),
            appearance_manager=object(),
            appearance_profiles=object(),
        )


def test_run_closes_simulation_app_when_descriptor_verification_fails(
    monkeypatch
):
    closed = []

    class FakeSimulationApp:
        def __init__(self, _launch):
            pass

        def close(self, *, exit_code):
            closed.append(exit_code)

    monkeypatch.setitem(
        sys.modules,
        "isaacsim",
        SimpleNamespace(SimulationApp=FakeSimulationApp),
    )
    monkeypatch.setattr(
        navigation_sim,
        "_verify_default_viewport_updates",
        lambda **_kwargs: {
            "requested_disabled": False,
            "observed_enabled": True,
            "match": True,
        },
    )
    monkeypatch.setattr(
        navigation_sim,
        "_verify_rtx_descriptor_sets",
        lambda _requested: (_ for _ in ()).throw(RuntimeError("mismatch")),
    )

    with pytest.raises(RuntimeError, match="mismatch"):
        run(
            _config(),
            None,
            None,
            None,
            None,
            "baseline",
            rtx_descriptor_sets=20000,
        )

    assert closed == [1]


def test_run_closes_simulation_app_when_viewport_verification_fails(
    monkeypatch
):
    closed = []

    class FakeSimulationApp:
        def __init__(self, launch):
            assert launch["disable_viewport_updates"] is True

        def close(self, *, exit_code):
            closed.append(exit_code)

    monkeypatch.setitem(
        sys.modules,
        "isaacsim",
        SimpleNamespace(SimulationApp=FakeSimulationApp),
    )
    monkeypatch.setattr(
        navigation_sim,
        "_verify_default_viewport_updates",
        lambda **_kwargs: (_ for _ in ()).throw(
            RuntimeError("viewport mismatch")
        ),
    )

    with pytest.raises(RuntimeError, match="viewport mismatch"):
        run(
            _config(),
            None,
            None,
            None,
            None,
            "baseline",
            disable_viewport_updates=True,
        )

    assert closed == [1]


@pytest.mark.parametrize(
    (
        "argv",
        "expected_localization_owner",
        "expected_descriptor_sets",
        "expected_paired_capture",
        "expected_viewport_disabled",
    ),
    (
        (["--disable-dlss"], "auto", None, False, False),
        (
            [
                "--disable-dlss",
                "--disable-viewport-updates",
                "--localization-owner",
                "ideal",
                "--rtx-descriptor-sets",
                "20000",
                "--paired-appearance-capture",
            ],
            "ideal",
            20000,
            True,
            True,
        ),
    ),
)
def test_main_passes_runtime_contract_directly_to_run(
    monkeypatch,
    argv,
    expected_localization_owner,
    expected_descriptor_sets,
    expected_paired_capture,
    expected_viewport_disabled,
):
    selected_pose = SimpleNamespace(map=SimpleNamespace(calibrated=True))
    captured = {}

    monkeypatch.setattr(
        navigation_sim, "load_project_config", lambda _path: _config())

    def fake_validate_configuration(
        _config, _profile, *, localization_owner
    ):
        captured["localization_owner"] = localization_owner
        return selected_pose, object(), object()

    monkeypatch.setattr(
        navigation_sim, "validate_configuration", fake_validate_configuration
    )

    def fake_run(*_args, **kwargs):
        captured["run_kwargs"] = kwargs

    monkeypatch.setattr(navigation_sim, "run", fake_run)

    assert navigation_sim.main(argv) == 0
    assert captured == {
        "localization_owner": expected_localization_owner,
        "run_kwargs": {
            "disable_dlss": True,
            "rtx_descriptor_sets": expected_descriptor_sets,
                "paired_appearance_capture_enabled": expected_paired_capture,
                "disable_viewport_updates": expected_viewport_disabled,
                "viewport_arm_identity": None,
                "viewport_runtime_attestation_path": None,
                "viewport_winner_manifest_path": None,
                "viewport_winner_manifest_sha256": None,
                "viewport_run_root": None,
                "viewport_scene": None,
                "viewport_launcher_path": None,
            },
    }


def test_targeted_teaching_wrapper_enables_paired_capture_only_for_isaac():
    wrapper = (
        ROOT / "scripts/run_module1_targeted_teaching_kujiale.sh"
    ).read_text(encoding="utf-8")
    case_body = wrapper.split('case "${component}" in', 1)[1]
    isaac_body, remaining = case_body.split("  ros)\n", 1)
    ros_body = remaining.split("  manifest)\n", 1)[0]

    assert isaac_body.count("--paired-appearance-capture") == 1
    assert 'isaac "$@" --paired-appearance-capture' in isaac_body
    assert "--paired-appearance-capture" not in ros_body
    assert wrapper.count("--paired-appearance-capture") == 1
