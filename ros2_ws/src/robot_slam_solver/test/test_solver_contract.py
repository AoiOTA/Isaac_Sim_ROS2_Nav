from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_solver_is_isolated_configurable_and_lgpl():
    source = (PACKAGE_ROOT / 'src' / 'ceres_solver.cpp').read_text()
    plugin = (PACKAGE_ROOT / 'solver_plugins.xml').read_text()
    notice = (PACKAGE_ROOT / 'NOTICE.md').read_text()

    assert 'options_.num_threads = 50' not in source
    assert 'ceres_num_threads' in source
    assert 'std::thread::hardware_concurrency()' in source
    assert 'requested ceres_num_threads=%ld exceeds' in source
    assert 'node->set_parameter(' in source
    assert 'robot_slam_solver::ConfigurableCeresSolver' in plugin
    assert 'base_class_type="karto::ScanSolver"' in plugin
    assert 'ec8f7635dea317b531c419f798f87d90a336f32e' in notice
    assert (PACKAGE_ROOT / 'LICENSE').read_text().startswith(
        '                  GNU LESSER GENERAL PUBLIC LICENSE')
