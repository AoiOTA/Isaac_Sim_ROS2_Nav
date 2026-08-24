from __future__ import annotations

from pathlib import Path
import runpy

import pytest

from isaac_sim.apps.navigation_sim import _parser
from isaac_sim.graphs.sensor_graph import lio_lidar_graph_spec
from isaac_sim.src.config import load_project_config
from isaac_sim.src.sensors.sensor_factory import (
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
    assert resolve_lio_lidar_config(lidar, None) is None
    assert _parser().parse_args([]).lio_lidar_profile is None


def test_os1_profile_has_exact_sensor_local_noncompensated_contract():
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")
    lio = resolve_lio_lidar_config(lidar, "OS1_REV6_32ch10hz512res")

    assert lio is not None
    assert lio["profile_name"] == "OS1_REV6_32ch10hz512res"
    assert lio["config"] == "OS1"
    assert lio["variant"] == "OS1_REV6_32ch10hz512res"
    assert lio["channels"] == 32
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


def _installed_lidar_registry():
    return runpy.run_path(str(INSTALLED_LIDAR_REGISTRY))[
        "SUPPORTED_LIDAR_CONFIGS"
    ]


def test_os1_config_and_variant_resolve_in_installed_isaac_registry():
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")
    lio = resolve_lio_lidar_config(lidar, "OS1_REV6_32ch10hz512res")

    assert lio is not None
    assert validate_lio_lidar_registry_selection(
        lio, _installed_lidar_registry()
    ) == "/Isaac/Sensors/Ouster/OS1/OS1.usd"


def test_lio_factory_passes_config_and_variant_as_separate_exact_arguments():
    lidar = _load_lidar(ROOT / "isaac_sim/configs/sensors/lidar_3d.yaml")
    lio = resolve_lio_lidar_config(lidar, "OS1_REV6_32ch10hz512res")
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
            "variant": "OS1_REV6_32ch10hz512res",
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


def test_os1_asset_declares_the_committed_profile_geometry():
    asset = Path(
        "/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Sensors/"
        "Ouster/OS1/OS1_REV6_32ch10hz512res.usda"
    ).read_text(encoding="utf-8")
    assert "uint omni:sensor:Core:numberOfChannels = 32" in asset
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
