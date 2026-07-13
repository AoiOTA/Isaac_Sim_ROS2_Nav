from pathlib import Path

import pytest

from robot_bringup.interactive_policy import parse_bool
from robot_bringup.interactive_policy import resolve_interactive_selection
from robot_bringup.interactive_policy import teleop_terminal_command


def _share(tmp_path):
    rviz = tmp_path / 'rviz'
    rviz.mkdir()
    for name in ('mapping.rviz', 'localization.rviz', 'navigation.rviz'):
        (rviz / name).write_text('Panels: []\n', encoding='utf-8')
    return tmp_path


@pytest.mark.parametrize('value', ['true', 'TRUE', '1', 'yes', 'on'])
def test_parse_bool_accepts_true_values(value):
    assert parse_bool(value, 'flag') is True


@pytest.mark.parametrize('value', ['false', 'FALSE', '0', 'no', 'off'])
def test_parse_bool_accepts_false_values(value):
    assert parse_bool(value, 'flag') is False


def test_parse_bool_rejects_ambiguous_value():
    with pytest.raises(ValueError, match='flag must be true or false'):
        parse_bool('sometimes', 'flag')


@pytest.mark.parametrize('operation', ['mapping', 'incremental_mapping'])
def test_mapping_defaults_to_rviz_and_teleop(operation, tmp_path):
    selection = resolve_interactive_selection(
        operation=operation,
        interactive='true',
        use_rviz='true',
        rviz_config='auto',
        use_teleop='auto',
        robot_description_share=_share(tmp_path),
    )

    assert selection.use_rviz is True
    assert selection.rviz_config.endswith('/rviz/mapping.rviz')
    assert selection.use_teleop is True


@pytest.mark.parametrize('operation', ['localization', 'navigation'])
def test_localization_and_navigation_never_auto_start_teleop(
        operation, tmp_path):
    selection = resolve_interactive_selection(
        operation=operation,
        interactive='true',
        use_rviz='true',
        rviz_config='auto',
        use_teleop='auto',
        robot_description_share=_share(tmp_path),
    )

    assert selection.use_teleop is False
    assert selection.rviz_config.endswith(f'/rviz/{operation}.rviz')


def test_headless_disables_both_processes_without_resolving_config(tmp_path):
    selection = resolve_interactive_selection(
        operation='navigation',
        interactive='false',
        use_rviz='true',
        rviz_config='/missing/config.rviz',
        use_teleop='true',
        robot_description_share=tmp_path,
    )

    assert selection.use_rviz is False
    assert selection.rviz_config == ''
    assert selection.use_teleop is False


def test_navigation_rejects_explicit_mapping_teleop(tmp_path):
    with pytest.raises(ValueError, match='unsafe for navigation'):
        resolve_interactive_selection(
            operation='navigation',
            interactive='true',
            use_rviz='false',
            rviz_config='auto',
            use_teleop='true',
            robot_description_share=tmp_path,
        )


def test_custom_rviz_config_is_preserved(tmp_path):
    custom = tmp_path / 'custom.rviz'
    custom.write_text('Panels: []\n', encoding='utf-8')

    selection = resolve_interactive_selection(
        operation='localization',
        interactive='true',
        use_rviz='true',
        rviz_config=str(custom),
        use_teleop='false',
        robot_description_share=tmp_path,
    )

    assert selection.rviz_config == str(custom)


def test_missing_rviz_config_fails_before_launch(tmp_path):
    with pytest.raises(ValueError, match='RViz config does not exist'):
        resolve_interactive_selection(
            operation='mapping',
            interactive='true',
            use_rviz='true',
            rviz_config='auto',
            use_teleop='auto',
            robot_description_share=Path(tmp_path),
        )


def test_teleop_prefers_waiting_gnome_terminal():
    command = teleop_terminal_command(
        '/project/scripts/run_teleop.sh',
        find_executable=lambda name: (
            '/usr/bin/gnome-terminal' if name == 'gnome-terminal' else None),
    )

    assert command == [
        '/usr/bin/gnome-terminal',
        '--wait',
        '--title=Isaac Nav Mapping Teleop',
        '--',
        '/project/scripts/run_teleop.sh',
    ]


def test_teleop_terminal_forwards_speed_overrides_without_shell_parsing():
    command = teleop_terminal_command(
        '/project/scripts/run_teleop.sh',
        arguments=(
            'linear_speed:=0.45',
            'angular_speed:=0.75',
        ),
        find_executable=lambda name: (
            '/usr/bin/gnome-terminal' if name == 'gnome-terminal' else None),
    )

    assert command == [
        '/usr/bin/gnome-terminal',
        '--wait',
        '--title=Isaac Nav Mapping Teleop',
        '--',
        '/project/scripts/run_teleop.sh',
        'linear_speed:=0.45',
        'angular_speed:=0.75',
    ]


def test_teleop_falls_back_to_xterm():
    command = teleop_terminal_command(
        '/project/scripts/run_teleop.sh',
        find_executable=lambda name: (
            '/usr/bin/xterm' if name == 'xterm' else None),
    )

    assert command[-2:] == ['-e', '/project/scripts/run_teleop.sh']


def test_teleop_requires_supported_terminal():
    with pytest.raises(ValueError, match='requires gnome-terminal'):
        teleop_terminal_command(
            '/project/scripts/run_teleop.sh',
            find_executable=lambda _name: None,
        )
