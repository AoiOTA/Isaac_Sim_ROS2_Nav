from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from xml.etree import ElementTree

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _read(relative_path):
    return (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")


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

    source = _read("src/laserMapping.cpp")
    odom_function = source[source.index("void publish_odometry"):source.index("void publish_path")]
    assert odom_function.index("pose.covariance") < odom_function.index("publish(odomAftMapped)")
    assert odom_function.index("twist.covariance") < odom_function.index("publish(odomAftMapped)")
    assert "if (tf_br)" in odom_function
    assert "if (publish_tf)" in source


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
    assert declarations["publish_tf"].default_value[0].text == "false"
    assert declarations["input_cloud_topic"].default_value[0].text == "/lio/points_raw"
    assert declarations["output_odom_topic"].default_value[0].text == "/lio/odom_shadow"
    assert len(nodes) == 1
    assert nodes[0].condition is not None
