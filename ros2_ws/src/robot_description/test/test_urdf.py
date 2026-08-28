from pathlib import Path
import xml.etree.ElementTree as ET

import xacro


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_FILE = PACKAGE_ROOT / 'urdf' / 'jackal.urdf.xacro'


def _robot_root(prefix=''):
    document = xacro.process_file(
        str(XACRO_FILE), mappings={'prefix': prefix})
    return ET.fromstring(document.toxml())


def test_required_links_and_joints_are_present():
    root = _robot_root()
    links = {element.attrib['name'] for element in root.findall('link')}
    joints = {element.attrib['name'] for element in root.findall('joint')}

    assert {
        'base_link',
        'front_left_wheel_link',
        'front_right_wheel_link',
        'rear_left_wheel_link',
        'rear_right_wheel_link',
        'lidar_link',
        'imu_link',
        'camera_link',
        'camera_front_link',
        'camera_front_optical_frame',
        'camera_left_link',
        'camera_right_link',
        'camera_left_optical_frame',
        'camera_right_optical_frame',
    } <= links
    assert {
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint',
    } <= joints

    for name in (
        'front_left_wheel_joint',
        'front_right_wheel_joint',
        'rear_left_wheel_joint',
        'rear_right_wheel_joint',
    ):
        joint = next(
            element for element in root.findall('joint')
            if element.attrib['name'] == name)
        assert joint.find('origin').attrib['xyz'].endswith(' 0.0345')
        # USD axis X rotated +90 degrees around Z is base-frame +Y.
        assert joint.find('axis').attrib['xyz'] == '0 1 0'


def test_description_does_not_claim_navigation_or_truth_frames():
    root = _robot_root()
    links = {element.attrib['name'] for element in root.findall('link')}
    assert links.isdisjoint({'world', 'map', 'odom', 'ground_truth_base_link'})


def test_sensor_and_optical_joints_are_fixed():
    root = _robot_root()
    joints = {element.attrib['name']: element for element in root.findall('joint')}
    expected = {
        'lidar_link_joint',
        'imu_link_joint',
        'camera_link_joint',
        'camera_front_link_joint',
        'camera_front_optical_frame_joint',
        'camera_left_link_joint',
        'camera_right_link_joint',
        'camera_left_optical_frame_joint',
        'camera_right_optical_frame_joint',
    }
    assert all(joints[name].attrib['type'] == 'fixed' for name in expected)

    for name in ('camera_front_optical_frame_joint',
                 'camera_left_optical_frame_joint',
                 'camera_right_optical_frame_joint'):
        origin = joints[name].find('origin')
        assert origin is not None
        assert origin.attrib['rpy'] == '-1.57079632679 0 -1.57079632679'


def test_lidar_mount_has_one_parent_and_is_reachable_from_base():
    for prefix in ('', 'robot_'):
        root = _robot_root(prefix)
        joints = root.findall('joint')
        lidar_joint = [
            joint for joint in joints
            if joint.find('child').attrib['link'] == f'{prefix}lidar_link'
        ]

        assert len(lidar_joint) == 1
        assert lidar_joint[0].attrib['type'] == 'fixed'
        assert lidar_joint[0].find('parent').attrib['link'] \
            == f'{prefix}base_link'
        origin = lidar_joint[0].find('origin')
        assert origin.attrib['xyz'] == '0.120 0.000 0.333'
        assert origin.attrib['rpy'] == '0 0 0'

        parent_by_child = {
            joint.find('child').attrib['link']:
            joint.find('parent').attrib['link']
            for joint in joints
        }
        frame = f'{prefix}lidar_link'
        while frame != f'{prefix}base_link':
            frame = parent_by_child[frame]


def test_description_only_mode_does_not_use_a_tf_broadcaster():
    publisher_source = (
        PACKAGE_ROOT / 'scripts' / 'robot_description_publisher.py').read_text()
    assert 'TransformBroadcaster' not in publisher_source
    assert "'/robot_description'" in publisher_source
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'description.launch.py').read_text()
    assert "DeclareLaunchArgument(\n            'publish_tf'" in launch_source
    assert "'xacro_file'" in launch_source
    assert "xacro_file = LaunchConfiguration('xacro_file')" in launch_source


def test_description_publisher_treats_external_shutdown_as_clean_exit():
    publisher_source = (
        PACKAGE_ROOT / 'scripts' / 'robot_description_publisher.py').read_text()
    assert 'except (KeyboardInterrupt, ExternalShutdownException):' \
        in publisher_source
    assert 'except RuntimeError as error:' in publisher_source
    assert "'context is not valid' not in str(error)" in publisher_source
    assert 'raise' in publisher_source
    assert publisher_source.index('node.destroy_node()') \
        < publisher_source.index('rclpy.shutdown()')
