import math
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest
from robot_odometry.imu_calibration import ImuYawCalibration
from robot_odometry.imu_calibration import ImuYawCalibrationConfig
from robot_odometry.imu_calibration_node import ImuCalibrationNode
from sensor_msgs.msg import Imu
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Logger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)


def _message(stamp_ns, angular_z=1.0):
    message = Imu()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = 'imu_link'
    message.orientation.x = 0.1
    message.orientation.y = 0.2
    message.orientation.z = 0.3
    message.orientation.w = 0.9
    message.orientation_covariance = [float(index) for index in range(9)]
    message.angular_velocity.x = 0.4
    message.angular_velocity.y = 0.5
    message.angular_velocity.z = angular_z
    message.angular_velocity_covariance = [
        0.01 * (index + 1) for index in range(9)]
    message.linear_acceleration.x = 1.1
    message.linear_acceleration.y = 1.2
    message.linear_acceleration.z = 9.7
    message.linear_acceleration_covariance = [
        0.02 * (index + 1) for index in range(9)]
    return message


def _adapter(config=None):
    publisher = _Publisher()
    logger = _Logger()
    adapter = SimpleNamespace(
        _calibration=ImuYawCalibration(
            config or ImuYawCalibrationConfig()),
        _diagnostic_interval=1000,
        _publisher=publisher,
        get_logger=lambda: logger,
    )
    adapter._report_diagnostics = MethodType(
        ImuCalibrationNode._report_diagnostics, adapter)
    return adapter, publisher, logger


@pytest.mark.parametrize(
    ('raw', 'expected'),
    [
        (0.0, 0.0),
        (0.25, 0.23235),
        (-0.25, -0.23235),
        (0.50, 0.46470),
        (-0.50, -0.46470),
        (1.081260, 1.081260 * 0.9294),
    ],
)
def test_v6_scale_matches_six_live_style_yaw_rates(raw, expected):
    calibration = ImuYawCalibration(ImuYawCalibrationConfig())
    result = calibration.calibrate(1_000_000_000, raw)

    assert result.accepted is True
    assert result.angular_velocity_z == pytest.approx(expected)


def test_message_contract_preserves_raw_fields_and_replaces_only_yaw_rate_covariance():
    adapter, publisher, _logger = _adapter(ImuYawCalibrationConfig(
        yaw_scale=0.9,
        yaw_bias_rad_s=0.1,
        yaw_variance=0.0002,
    ))
    raw = _message(2_000_000_123, angular_z=1.1)
    raw_covariance = list(raw.angular_velocity_covariance)

    ImuCalibrationNode._raw_callback(adapter, raw)

    assert len(publisher.messages) == 1
    corrected = publisher.messages[0]
    assert corrected is not raw
    assert corrected.header == raw.header
    assert corrected.orientation == raw.orientation
    assert list(corrected.orientation_covariance) == list(
        raw.orientation_covariance)
    assert corrected.linear_acceleration == raw.linear_acceleration
    assert list(corrected.linear_acceleration_covariance) == list(
        raw.linear_acceleration_covariance)
    assert corrected.angular_velocity.x == raw.angular_velocity.x
    assert corrected.angular_velocity.y == raw.angular_velocity.y
    assert corrected.angular_velocity.z == pytest.approx(0.9)
    assert list(corrected.angular_velocity_covariance[:8]) == (
        raw_covariance[:8])
    assert corrected.angular_velocity_covariance[8] == pytest.approx(0.0002)
    assert raw.angular_velocity.z == pytest.approx(1.1)
    assert list(raw.angular_velocity_covariance) == raw_covariance


def test_duplicate_backward_and_nonfinite_samples_fail_closed():
    adapter, publisher, logger = _adapter()

    for message in (
        _message(3_000_000_000),
        _message(3_000_000_000),
        _message(2_999_999_999),
        _message(3_100_000_000, math.nan),
    ):
        ImuCalibrationNode._raw_callback(adapter, message)

    assert len(publisher.messages) == 1
    assert adapter._calibration.counters == {
        'accepted': 1,
        'nonfinite': 1,
        'backward': 1,
        'duplicate': 1,
    }
    assert len(logger.warnings) == 3


def test_explicit_reset_accepts_a_new_lower_positive_stamp():
    calibration = ImuYawCalibration(ImuYawCalibrationConfig())
    assert calibration.calibrate(5_000_000_000, 0.2).accepted
    assert not calibration.calibrate(1_000_000_000, 0.2).accepted

    calibration.reset_stamp()

    assert calibration.calibrate(1_000_000_000, 0.2).accepted


@pytest.mark.parametrize(
    'kwargs',
    [
        {'yaw_scale': 0.49},
        {'yaw_scale': 1.51},
        {'yaw_scale': math.nan},
        {'yaw_bias_rad_s': math.inf},
        {'yaw_variance': 0.0},
        {'yaw_variance': math.nan},
    ],
)
def test_invalid_parameters_fail_closed(kwargs):
    with pytest.raises(ValueError):
        ImuYawCalibrationConfig(**kwargs)


def test_calibrated_default_and_identity_rollback_profiles_are_explicit():
    def parameters(filename):
        document = yaml.safe_load(
            (PACKAGE_ROOT / 'config' / filename).read_text())
        return document['imu_yaw_calibrator']['ros__parameters']

    calibrated = parameters('imu_calibration.yaml')
    identity = parameters('imu_calibration_identity.yaml')

    assert calibrated['yaw_scale'] == pytest.approx(0.9294)
    assert calibrated['yaw_bias_rad_s'] == pytest.approx(0.0)
    assert calibrated['yaw_variance'] == pytest.approx(1.0e-4)
    assert identity['yaw_scale'] == pytest.approx(1.0)
    assert identity['yaw_bias_rad_s'] == calibrated['yaw_bias_rad_s']
    assert identity['yaw_variance'] == calibrated['yaw_variance']


def test_node_topics_default_to_legacy_and_accept_parameter_endpoints():
    source = (
        PACKAGE_ROOT / 'robot_odometry' / 'imu_calibration_node.py'
    ).read_text(encoding='utf-8')

    assert "declare_parameter('input_topic', '/imu/data_raw')" in source
    assert "declare_parameter('output_topic', '/imu/data')" in source
    assert "str(self.get_parameter('input_topic').value)" in source
    assert "str(self.get_parameter('output_topic').value)" in source
    assert "Imu, '/imu/data_raw'" not in source
    assert "Imu, '/imu/data'" not in source


def test_vio_profile_reuses_legacy_calibration_with_separate_topics():
    legacy = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'imu_calibration.yaml').read_text()
    )['imu_yaw_calibrator']['ros__parameters']
    vio = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'imu_vio_calibration.yaml').read_text()
    )['imu_vio_calibrator']['ros__parameters']

    assert vio['input_topic'] == '/imu/vio_raw'
    assert vio['output_topic'] == '/imu/vio'
    for key in (
        'use_sim_time', 'yaw_scale', 'yaw_bias_rad_s', 'yaw_variance',
        'diagnostic_interval',
    ):
        assert vio[key] == legacy[key]


def test_two_calibrator_instances_keep_independent_stamp_state():
    legacy, legacy_publisher, _ = _adapter()
    vio, vio_publisher, _ = _adapter()

    ImuCalibrationNode._raw_callback(legacy, _message(2_000_000_000))
    ImuCalibrationNode._raw_callback(vio, _message(1_000_000_000))
    ImuCalibrationNode._raw_callback(legacy, _message(1_500_000_000))
    ImuCalibrationNode._raw_callback(vio, _message(1_500_000_000))

    assert len(legacy_publisher.messages) == 1
    assert len(vio_publisher.messages) == 2
    assert legacy._calibration.counters['backward'] == 1
    assert vio._calibration.counters['backward'] == 0
