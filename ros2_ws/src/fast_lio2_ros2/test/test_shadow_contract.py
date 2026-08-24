import math
import re
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from xml.etree import ElementTree

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read(relative_path):
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


def _inline_yaml_numbers(config, key):
    match = re.search(rf"{re.escape(key)}:\s*\[([^\]]+)\]", config)
    assert match is not None
    return [float(value.strip()) for value in match.group(1).split(",")]


def _mat_vec(matrix, vector):
    return [
        sum(matrix[row * 3 + column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def test_pinned_gpl_source_and_dependencies():
    origin = _read("ORIGIN.md")
    assert "2fffc570a25d0df172720bac034fbdb6a13d2162" in origin
    assert "e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4" in origin
    assert _read("LICENSE").lstrip().startswith("GNU GENERAL PUBLIC LICENSE")
    package = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    assert package.findtext("license") == "GPL-2.0"

    compile_surface = "\n".join(
        _read(path)
        for path in (
            "CMakeLists.txt",
            "package.xml",
            "src/preprocess.h",
        )
    ).lower()
    assert "pcl_ros" not in compile_surface
    assert "livox_ros_driver" not in compile_surface
    assert "custommsg" not in compile_surface


def test_shadow_defaults_topics_frames_and_ordering():
    config = _read("config/ouster_shadow.yaml")
    for expected in (
        "lid_topic: /lio/points_raw",
        "imu_topic: /imu/data",
        "odom_topic: /lio/odom_shadow",
        "map_frame: lio_map_shadow",
        "body_frame: base_link",
        "scan_rate: 10",
        "blind: 0.3",
        "extrinsic_T: [0.108, -0.002, 0.266]",
        "extrinsic_est_en: false",
        "publish_tf: false",
        "path_en: false",
        "map_en: false",
        "scan_publish_en: false",
        "scan_bodyframe_pub_en: false",
        "pcd_save_en: false",
    ):
        assert expected in config

    rotation = _inline_yaml_numbers(config, "extrinsic_R")
    translation = _inline_yaml_numbers(config, "extrinsic_T")
    assert rotation == [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    assert translation == [0.108, -0.002, 0.266]
    assert (
        "extrinsic_R: [0.0, -1.0, 0.0,\n"
        "                    1.0,  0.0, 0.0,\n"
        "                    0.0,  0.0, 1.0]"
    ) in config

    rows = [rotation[offset:offset + 3] for offset in range(0, 9, 3)]
    gram = [
        sum(rows[k][i] * rows[k][j] for k in range(3))
        for i in range(3)
        for j in range(3)
    ]
    determinant = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    assert gram == [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    assert determinant == 1.0
    assert _mat_vec(rotation, [1.0, 0.0, 0.0]) == [0.0, 1.0, 0.0]
    assert _mat_vec(rotation, [0.0, 1.0, 0.0]) == [-1.0, 0.0, 0.0]
    assert math.atan2(rotation[3], rotation[0]) == math.pi / 2.0

    source = _read("src/laserMapping.cpp")
    odom_function = source[source.index("void publish_odometry"):source.index("void publish_path")]
    assert odom_function.index("pose.covariance") < odom_function.index("publish(odomAftMapped)")
    assert odom_function.index("twist.covariance") < odom_function.index("publish(odomAftMapped)")
    assert "if (tf_br)" in odom_function
    assert "if (publish_tf)" in source
    identity_fallback = (
        "vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, "
        "0.0, 0.0, 1.0}"
    )
    assert source.count(identity_fallback) == 2
    assert "vector<double>{0.0, -1.0" not in source


def test_launch_description_expands_and_is_default_off():
    launch_path = PACKAGE_ROOT / "launch" / "shadow.launch.py"
    spec = spec_from_file_location("fast_lio2_shadow_launch", launch_path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()

    declarations = {
        entity.name: entity
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]
    assert declarations["enabled"].default_value[0].text == "false"
    assert declarations["planar_imu_enabled"].default_value[0].text == "false"
    assert declarations["publish_tf"].default_value[0].text == "false"
    assert declarations["input_cloud_topic"].default_value[0].text == "/lio/points_raw"
    assert declarations["output_odom_topic"].default_value[0].text == "/lio/odom_shadow"
    assert len(nodes) == 2
    assert all(node.condition is not None for node in nodes)

    launch = _read("launch/shadow.launch.py")
    assert "mapping.extrinsic_R" not in launch
    assert "mapping.extrinsic_T" not in launch
    assert "static_transform_publisher" not in launch
    assert "tf2_ros" not in launch

    sensor_urdf = (
        PACKAGE_ROOT.parent / "robot_description/urdf/jackal_sensors.xacro"
    ).read_text(encoding="utf-8")
    frame_call = re.search(
        r'<xacro:fixed_frame\s+prefix="\$\{prefix\}" '
        r'parent="base_link" child="lio_lidar_link"\s+([^>]*)/>',
        sensor_urdf,
    )
    assert frame_call is not None
    assert 'xyz="0.120 0.000 0.333"' in frame_call.group(1)
    assert "rpy=" not in frame_call.group(1)


def test_planar_imu_remap_is_explicit_and_diagnostics_are_bounded():
    launch = _read("launch/shadow.launch.py")
    assert '"\'/imu/lio\' if \'"' in launch
    assert 'LaunchConfiguration("input_imu_topic")' in launch
    assert 'package="robot_odometry"' in launch

    source = _read("src/laserMapping.cpp")
    assert "scan_begin=%.9f scan_end=%.9f" in source
    assert "nearest_neighbor=%lu" in source
    assert "plane_fit=%lu" in source
    assert "residual=%lu" in source
    assert "1000000000LL" in source
    assert 'pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5' in source
    assert 'esti_plane(pabcd, points_near, 0.1f)' in source
    assert 'if (s > 0.9)' in source
