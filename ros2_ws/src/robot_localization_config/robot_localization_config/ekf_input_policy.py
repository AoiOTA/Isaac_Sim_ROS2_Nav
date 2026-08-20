"""Fail-closed classification of EKF sensor input topics."""

from collections.abc import Mapping
from pathlib import Path
import re

import yaml


_INPUT_KEY = re.compile(r'^(odom|pose|twist|imu)\d+$')
_CANONICAL_SHADOW_INPUTS = {'/wheel/odom', '/imu/data'}
_LIDAR_MARKERS = ('lidar', 'laser', 'rf2o', 'scan_match')


def _ros_parameters(document):
    if not isinstance(document, Mapping):
        raise ValueError('EKF YAML must contain a mapping')
    blocks = [
        value['ros__parameters']
        for value in document.values()
        if isinstance(value, Mapping) and 'ros__parameters' in value
    ]
    if len(blocks) != 1 or not isinstance(blocks[0], Mapping):
        raise ValueError('EKF YAML must contain exactly one ros__parameters block')
    return blocks[0]


def ekf_uses_lidar(params_file):
    """Return whether a known EKF config references a LiDAR odometry input."""
    path = Path(params_file)
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ValueError(f'cannot parse EKF params file {path}: {exc}') from exc

    sensor_inputs = []
    for key, value in _ros_parameters(document).items():
        if not _INPUT_KEY.fullmatch(str(key)):
            continue
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f'EKF input {key} must name a topic')
        topic = '/' + value.strip().lstrip('/')
        sensor_inputs.append(topic)

    if not sensor_inputs:
        raise ValueError('EKF YAML contains no recognized sensor inputs')

    uses_lidar = False
    for topic in sensor_inputs:
        if topic in _CANONICAL_SHADOW_INPUTS:
            continue
        if any(marker in topic.lower() for marker in _LIDAR_MARKERS):
            uses_lidar = True
            continue
        raise ValueError(f'unrecognized EKF sensor input topic: {topic}')
    return uses_lidar


def validate_lidar_gate(params_file, lidar_odometry_validated):
    """Classify the loaded file and reject unvalidated LiDAR fusion."""
    uses_lidar = ekf_uses_lidar(params_file)
    if uses_lidar and not lidar_odometry_validated:
        raise ValueError(
            'loaded EKF params reference LiDAR odometry; explicitly set '
            'lidar_odometry_validated:=true only after calibration')
    return uses_lidar
