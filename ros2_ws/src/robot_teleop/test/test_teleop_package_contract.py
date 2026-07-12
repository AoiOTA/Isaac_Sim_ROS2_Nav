"""Package, configuration, and terminal-adapter contract tests."""

from pathlib import Path

from robot_teleop.keyboard_teleop import decode_keypresses
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_terminal_arrow_sequences_and_control_keys_are_decoded():
    """Raw terminal escape sequences retain all required controls."""
    assert decode_keypresses(b'w\x1b[A\x1b[B\x1b[C\x1b[D\x03\x04') == [
        'w', 'up', 'down', 'right', 'left', '\x03', '\x04']


def test_safe_defaults_and_package_install_contract():
    """Installed YAML defaults stay inside project safety limits."""
    document = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'teleop.yaml').read_text())
    params = document['keyboard_teleop']['ros__parameters']

    assert params['cmd_vel_topic'] == '/cmd_vel'
    assert params['command_timeout_sec'] <= 0.20
    assert params['publish_rate_hz'] >= 10.0
    assert params['linear_speed'] <= params['max_linear_speed']
    assert params['angular_speed'] <= params['max_angular_speed']

    setup_source = (PACKAGE_ROOT / 'setup.py').read_text()
    assert "glob('config/*.yaml')" in setup_source
    assert 'keyboard_teleop = robot_teleop.keyboard_teleop:main' \
        in setup_source


def test_node_uses_wall_monotonic_time_and_finally_stops():
    """The ROS adapter is independent of `/clock` and always closes safely."""
    source = (
        PACKAGE_ROOT / 'robot_teleop' / 'keyboard_teleop.py').read_text()

    assert 'time.monotonic()' in source
    assert 'node.runtime.close()' in source
    assert "topic != '/cmd_vel'" in source
    assert 'create_timer' not in source
