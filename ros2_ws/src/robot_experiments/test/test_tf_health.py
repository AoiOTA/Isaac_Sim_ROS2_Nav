from robot_experiments.experiment_runner import ExperimentRunner


def _runner(clock_s: float) -> ExperimentRunner:
    runner = object.__new__(ExperimentRunner)
    runner._lock_started_at = None
    runner._collision_lock_timeout_sec = 1.0
    runner._collision_monitor_locked = False
    runner._tf_ever_available = True
    runner._last_tf_stamp_s = 10.0
    runner._tf_gap_tolerance_sec = 1.0
    runner._tf_interrupted = False
    runner._localization_lost = False
    runner._lookup_fresh_map_to_odom = lambda: None
    runner._clock_seconds = lambda: clock_s
    return runner


def test_tf_health_uses_ros_time_not_process_wall_time():
    runner = _runner(10.5)
    runner._update_health()
    assert runner._tf_interrupted is False
    assert runner._localization_lost is False


def test_tf_health_latches_a_real_ros_time_gap():
    runner = _runner(11.0)
    runner._update_health()
    assert runner._tf_interrupted is True
    assert runner._localization_lost is True
