from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_plugin_uses_project_owned_class_name():
    plugin = (PACKAGE_ROOT / 'plugins_description.xml').read_text(
        encoding='utf-8')
    assert 'robot_rviz_plugins/Navigation 2 Safe' in plugin
    assert 'robot_rviz_plugins::Nav2Panel' in plugin


def test_initial_thread_has_cooperative_shutdown():
    header = (PACKAGE_ROOT / 'include' / 'robot_rviz_plugins'
              / 'nav2_panel.hpp').read_text(encoding='utf-8')
    source = (PACKAGE_ROOT / 'src' / 'nav2_panel.cpp').read_text(
        encoding='utf-8')
    assert 'isInterruptionRequested()' in header
    assert 'rclcpp::ok()' in header
    assert 'requestInterruption()' in source
    assert 'wait()' in source


def test_panel_owns_qtconcurrent_tasks_and_guards_timer_spin():
    header = (PACKAGE_ROOT / 'include' / 'robot_rviz_plugins'
              / 'nav2_panel.hpp').read_text(encoding='utf-8')
    source = (PACKAGE_ROOT / 'src' / 'nav2_panel.cpp').read_text(
        encoding='utf-8')
    assert 'QFutureSynchronizer<void> async_tasks_' in header
    assert 'async_tasks_.addFuture' in source
    assert 'async_tasks_.waitForFinished()' in source
    assert '!rclcpp::ok()' in source


def test_status_callbacks_guard_empty_arrays_and_loop_parsing():
    header = (PACKAGE_ROOT / 'include' / 'robot_rviz_plugins'
              / 'nav2_panel.hpp').read_text(encoding='utf-8')
    source = (PACKAGE_ROOT / 'src' / 'nav2_panel.cpp').read_text(
        encoding='utf-8')
    assert 'bool parseLoopValue(std::string & loop, int & value);' in header
    assert source.count('if (msg->status_list.empty())') == 2
    assert source.count('msg->status_list.back().status') == 2
    assert source.count('stoi(') == 2  # validation plus the guarded parser
    assert 'parseLoopValue(loop_value, parsed_loop_value)' in source


def test_upstream_provenance_and_license_are_present():
    notice = (PACKAGE_ROOT / 'NOTICE.md').read_text(encoding='utf-8')
    assert 'nav2_rviz_plugins' in notice
    assert '1.3.12' in notice
    assert (PACKAGE_ROOT / 'LICENSE').is_file()
