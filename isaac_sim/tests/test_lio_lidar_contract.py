from __future__ import annotations

import ast
from pathlib import Path
import runpy

import pytest

from isaac_sim.apps.navigation_sim import _parser
from isaac_sim.graphs.sensor_graph import lio_lidar_graph_spec
from isaac_sim.src.config import load_project_config
from isaac_sim.src.sensors.sensor_factory import (
    LIO_LIDAR_PROFILE_NAMES,
    LIO_LIDAR_AUX_OUTPUT_LEVEL,
    SensorConfigError,
    _create_lio_lidar,
    _load_lidar,
    resolve_lio_lidar_config,
    validate_lio_lidar_registry_selection,
)


ROOT = Path(__file__).resolve().parents[2]
INSTALLED_LIDAR_REGISTRY = Path(
    "/home/lyb/miniconda3/envs/isaacsim/lib/python3.12/site-packages/isaacsim/"
    "exts/isaacsim.sensors.experimental.rtx/isaacsim/sensors/experimental/"
    "rtx/impl/rtx_lidar_configs.py"
)
INSTALLED_LIDAR_API = INSTALLED_LIDAR_REGISTRY.with_name("lidar.py")


def _config():
    return load_project_config(
        ROOT / "isaac_sim/configs/project.yaml",
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": "/home/lyb/isaacsim_assets/Assets/Isaac/6.0",
        },
    )


def test_existing_2d_rplidar_contract_is_unchanged_and_lio_defaults_off():
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")

    assert lidar["config"] == "RPLIDAR_S2E"
    assert lidar["topic_name"] == "/lidar/points_raw"
    assert lidar["frame_id"] == "rtx_world"
    assert lidar["output_frame"] == "WORLD"
    assert lidar["motion_compensation"] == "COMPENSATED"
    assert lidar["lio"]["default_profile"] == "off"
    assert tuple(lidar["lio"]["profiles"]) == LIO_LIDAR_PROFILE_NAMES[1:]
    assert LIO_LIDAR_PROFILE_NAMES == (
        "off",
        "OS1_REV6_32ch10hz512res",
        "OS1_REV6_128ch10hz512res",
    )
    assert resolve_lio_lidar_config(lidar, None) is None
    assert _parser().parse_args([]).lio_lidar_profile is None


@pytest.mark.parametrize(
    ("profile", "channels"),
    [
        ("OS1_REV6_32ch10hz512res", 32),
        ("OS1_REV6_128ch10hz512res", 128),
    ],
)
def test_os1_profiles_have_exact_sensor_local_noncompensated_contract(
    profile, channels
):
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")
    lio = resolve_lio_lidar_config(lidar, profile)

    assert _parser().parse_args(
        ["--lio-lidar-profile", profile]
    ).lio_lidar_profile == profile
    assert lio is not None
    assert lio["profile_name"] == profile
    assert lio["config"] == "OS1"
    assert lio["variant"] == profile
    assert lio["channels"] == channels
    assert lio["tick_rate"] == 10.0
    assert lio["horizontal_resolution"] == 512
    assert lio["range_m"] == (0.3, 120.0)
    assert lio["topic_name"] == "/lio/points_raw_isaac"
    assert lio["frame_id"] == "lio_lidar_link"
    assert lio["output_frame"] == "SENSOR"
    assert lio["motion_compensation"] == "NONCOMPENSATED"

    spec = lio_lidar_graph_spec(_config(), "/Render/Lio", lio)
    spec.validate()
    values = dict(spec.values)
    assert values["PointCloudConfig.inputs:outputIntensity"] is True
    assert values["PointCloudConfig.inputs:outputTimestamp"] is True
    assert values["PointCloudConfig.inputs:outputChannelId"] is True
    assert values["PointCloudPublisher.inputs:topicName"] == (
        "/lio/points_raw_isaac"
    )
    assert values["PointCloudPublisher.inputs:frameId"] == "lio_lidar_link"
    assert '"reliability":"bestEffort"' in values[
        "PointCloudPublisher.inputs:qosProfile"
    ]


def test_os1_128_changes_only_profile_variant_and_channel_count():
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")
    os1_32 = resolve_lio_lidar_config(lidar, "OS1_REV6_32ch10hz512res")
    os1_128 = resolve_lio_lidar_config(lidar, "OS1_REV6_128ch10hz512res")

    assert os1_32 is not None
    assert os1_128 is not None
    differing = {
        key for key in os1_32 if os1_32[key] != os1_128[key]
    }
    assert differing == {"profile_name", "variant", "channels"}
    assert os1_128["sensor_prim"] == os1_32["sensor_prim"]
    assert os1_128["topic_name"] == os1_32["topic_name"]
    assert os1_128["frame_id"] == os1_32["frame_id"]


def test_unknown_lio_profile_still_fails_before_kit():
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")

    with pytest.raises(SensorConfigError, match="unknown LIO LiDAR profile"):
        resolve_lio_lidar_config(lidar, "missing")
    with pytest.raises(SystemExit):
        _parser().parse_args(["--lio-lidar-profile", "missing"])


def _installed_lidar_registry():
    return runpy.run_path(str(INSTALLED_LIDAR_REGISTRY))[
        "SUPPORTED_LIDAR_CONFIGS"
    ]


@pytest.mark.parametrize("profile", LIO_LIDAR_PROFILE_NAMES[1:])
def test_os1_config_and_variant_resolve_in_installed_isaac_registry(profile):
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")
    lio = resolve_lio_lidar_config(lidar, profile)

    assert lio is not None
    assert validate_lio_lidar_registry_selection(
        lio, _installed_lidar_registry()
    ) == "/Isaac/Sensors/Ouster/OS1/OS1.usd"


def test_installed_lidar_create_accepts_full_aux_output_level():
    module = ast.parse(INSTALLED_LIDAR_API.read_text(encoding="utf-8"))
    lidar_class = next(
        node
        for node in module.body
        if isinstance(node, ast.ClassDef) and node.name == "Lidar"
    )
    create = next(
        node
        for node in lidar_class.body
        if isinstance(node, ast.FunctionDef) and node.name == "create"
    )
    keyword_defaults = dict(
        zip(
            (argument.arg for argument in create.args.kwonlyargs),
            create.args.kw_defaults,
            strict=True,
        )
    )
    assert ast.literal_eval(keyword_defaults["aux_output_level"]) == "NONE"

    valid_levels = next(
        node.value
        for node in lidar_class.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name)
            and target.id == "_VALID_AUX_OUTPUT_LEVELS"
            for target in node.targets
        )
    )
    assert LIO_LIDAR_AUX_OUTPUT_LEVEL == "FULL"
    assert LIO_LIDAR_AUX_OUTPUT_LEVEL in ast.literal_eval(valid_levels)


@pytest.mark.parametrize(
    "profile", LIO_LIDAR_PROFILE_NAMES[1:]
)
def test_lio_factory_passes_full_aux_config_and_variant_as_exact_arguments(
    profile
):
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")
    lio = resolve_lio_lidar_config(lidar, profile)
    calls = []
    sentinel = object()

    class FakeLidar:
        @staticmethod
        def create(**kwargs):
            calls.append(kwargs)
            return sentinel

    assert lio is not None
    assert _create_lio_lidar(
        FakeLidar, lio, _installed_lidar_registry()
    ) is sentinel
    assert calls == [
        {
            "path": "/World/Robots/Jackal/base_link/lio_lidar_link/rtx_lidar",
            "config": "OS1",
            "variant": profile,
            "aux_output_level": "FULL",
            "tick_rate": 10.0,
            "accumulate_outputs": True,
            "attributes": {
                "omni:sensor:Core:outputFrameOfReference": "SENSOR",
                "omni:sensor:Core:outputMotionCompensationState": (
                    "NONCOMPENSATED"
                ),
            },
        }
    ]


@pytest.mark.parametrize(
    ("config", "variant", "message"),
    [
        ("OS9", "OS1_REV6_32ch10hz512res", "config 'OS9' is not installed"),
        ("OS1", "missing", "variant 'missing' is not installed for config 'OS1'"),
    ],
)
def test_invalid_lio_registry_selection_fails_with_named_values(
    config, variant, message
):
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")
    lio = resolve_lio_lidar_config(lidar, "OS1_REV6_32ch10hz512res")

    assert lio is not None
    invalid = dict(lio, config=config, variant=variant)
    with pytest.raises(SensorConfigError, match=message):
        validate_lio_lidar_registry_selection(
            invalid, _installed_lidar_registry()
        )


@pytest.mark.parametrize(
    ("profile", "channels"),
    [
        ("OS1_REV6_32ch10hz512res", 32),
        ("OS1_REV6_128ch10hz512res", 128),
    ],
)
def test_os1_assets_declare_the_committed_profile_geometry(profile, channels):
    asset = Path(
        "/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Sensors/"
        f"Ouster/OS1/{profile}.usda"
    ).read_text(encoding="utf-8")
    assert f"uint omni:sensor:Core:numberOfChannels = {channels}" in asset
    assert "uint omni:sensor:Core:scanRateBaseHz = 10" in asset
    assert "uint omni:sensor:Core:patternFiringRateHz = 5120" in asset
    assert "float omni:sensor:Core:nearRangeM = 0.3" in asset
    assert "float omni:sensor:Core:farRangeM = 120" in asset


def test_lio_mount_and_imu_extrinsics_are_explicit_and_identity_rotated():
    import yaml

    robot = yaml.safe_load(
        (ROOT / "isaac_sim/configs/robots/jackal.yaml").read_text(
            encoding="utf-8"
        )
    )
    transforms = {item["child"]: item for item in robot["static_transforms"]}
    assert transforms["lio_lidar_link"]["translation"] == [0.120, 0.0, 0.333]
    assert transforms["imu_link"]["translation"] == [0.012, 0.002, 0.067]
    assert transforms["lio_lidar_link"]["rotation_xyzw"] == [
        0.0, 0.0, 0.0, 1.0
    ]
    assert transforms["imu_link"]["rotation_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    # IMU -> LiDAR, matching the initial FAST-LIO2 extrinsic convention.
    assert [
        transforms["lio_lidar_link"]["translation"][axis]
        - transforms["imu_link"]["translation"][axis]
        for axis in range(3)
    ] == [0.108, -0.002, 0.266]
