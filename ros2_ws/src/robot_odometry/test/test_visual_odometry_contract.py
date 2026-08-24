from pathlib import Path

import numpy as np
import pytest
from robot_odometry.depth_float_to_uint16_node import convert_depth_image
from sensor_msgs.msg import Image
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE_ROOT / 'config' / 'cuvslam_rgbd_shadow.yaml'
LAUNCH = PACKAGE_ROOT / 'launch' / 'visual_odometry.launch.py'


def test_cuvslam_rgbd_shadow_parameters_are_isolated():
    document = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))
    parameters = document['visual_slam_shadow']['ros__parameters']

    assert parameters == {
        'use_sim_time': True,
        'tracking_mode': 2,
        'num_cameras': 1,
        'min_num_images': 1,
        'depth_camera_id': 0,
        'depth_scale_factor': 1000.0,
        'camera_optical_frames': ['camera_front_optical_frame'],
        'base_frame': 'base_link',
        'odom_frame': 'visual_odom_shadow',
        'map_frame': 'visual_map_shadow',
        'enable_localization_n_mapping': False,
        'enable_slam_visualization': False,
        'enable_landmarks_view': False,
        'enable_observations_view': False,
        'rectified_images': False,
        'image_qos': 'SENSOR_DATA',
        'sync_matching_threshold_ms': 10.0,
        'image_jitter_threshold_ms': 20.0,
        'publish_map_to_odom_tf': False,
        'publish_odom_to_base_tf': False,
        'override_publishing_stamp': False,
    }


def test_visual_shadow_uses_one_aligned_rgbd_camera_and_explicit_outputs():
    source = LAUNCH.read_text(encoding='utf-8')
    for contract in (
        "plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode'",
        "('visual_slam/image_0', '/camera/front/image_raw')",
        "('visual_slam/camera_info_0', '/camera/front/camera_info')",
        "('visual_slam/depth_0', '/camera/front/depth/image_uint16')",
        "('visual_slam/tracking/odometry', '/visual/odom_shadow')",
        "('visual_slam/status', '/visual/status')",
        "executable='depth_float_to_uint16'",
        "executable='component_container'",
    ):
        assert contract in source
    assert 'visual_slam/imu' not in source


def _depth_message(values):
    message = Image()
    message.header.stamp.sec = 41
    message.header.stamp.nanosec = 123456789
    message.header.frame_id = 'camera_front_optical_frame'
    message.height = 2
    message.width = len(values) // 2
    message.encoding = '32FC1'
    message.is_bigendian = 0
    message.step = message.width * 4
    message.data = np.asarray(values, dtype='<f4').tobytes()
    return message


def test_depth_conversion_values_invalids_clamp_and_header_stamp():
    source = _depth_message([
        0.3028, 0.8495, np.nan, np.inf,
        -1.0, 0.0, 65.5354, 80.0,
    ])

    converted = convert_depth_image(source)

    assert converted.encoding == '16UC1'
    assert converted.is_bigendian == 0
    assert converted.step == converted.width * 2
    assert np.frombuffer(converted.data, dtype='<u2').tolist() == [
        303, 850, 0, 0, 0, 0, 65535, 65535,
    ]
    assert converted.header.frame_id == source.header.frame_id
    assert converted.header.stamp.sec == 41
    assert converted.header.stamp.nanosec == 123456789


@pytest.mark.parametrize(
    ('mutation', 'error'),
    [
        (lambda message: setattr(message, 'encoding', '16UC1'), 'expected 32FC1'),
        (lambda message: setattr(message, 'is_bigendian', 1), 'big-endian'),
        (lambda message: setattr(message, 'step', 20), 'contiguous step'),
        (lambda message: setattr(message, 'data', b'\x00'), 'depth bytes'),
    ],
)
def test_depth_conversion_rejects_noncontract_input(mutation, error):
    message = _depth_message([1.0, 2.0, 3.0, 4.0])
    mutation(message)

    with pytest.raises(ValueError, match=error):
        convert_depth_image(message)


def test_installed_isaac_ros_45_visual_slam_component_is_discoverable():
    prefix = Path('/opt/ros/jazzy')
    package = prefix / 'share' / 'isaac_ros_visual_slam'
    version = yaml.safe_load(
        (package / 'version_info.yaml').read_text(encoding='utf-8'))
    component_resource = (
        prefix / 'share' / 'ament_index' / 'resource_index'
        / 'rclcpp_components' / 'isaac_ros_visual_slam'
    )

    assert version['version'] == '4.5.0'
    assert (prefix / 'lib' / 'libvisual_slam_node.so').is_file()
    assert (prefix / 'lib' / 'rclcpp_components'
            / 'component_container').is_file()
    assert ('nvidia::isaac_ros::visual_slam::VisualSlamNode;'
            'lib/libvisual_slam_node.so') in component_resource.read_text(
                encoding='utf-8')
