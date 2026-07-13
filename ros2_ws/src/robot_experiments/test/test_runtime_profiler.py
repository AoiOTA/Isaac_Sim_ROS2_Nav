from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

from robot_experiments.runtime_profiler import (
    TF_PAIRS,
    TOPIC_NAMES,
    _cpu_utilization,
    _gpu_delta,
    _infer_ros_operation,
    _process_delta,
    _proc_stat,
    _registered_process_members,
    _registered_process_snapshot,
    camera_stamp_report,
    distribution,
    percentile,
    RtfMetric,
    TopicMetric,
)


def test_plan_required_topics_and_tf_chains_are_profiled():
    assert set(TOPIC_NAMES) == {
        "clock",
        "pointcloud",
        "scan",
        "imu",
        "joint_states",
        "wheel_odom",
        "odom",
        "global_plan",
        "local_plan",
        "cmd_vel_nav",
        "cmd_vel_smoothed",
        "cmd_vel",
        "camera_image",
        "camera_info",
    }
    assert {
        "map->odom",
        "odom->base_link",
        "map->base_link",
        "base_link->lidar_link",
        "base_link->imu_link",
        "base_link->camera_front_link",
        "camera_front_link->camera_front_optical_frame",
    } <= set(TF_PAIRS)


def test_percentiles_are_interpolated_and_empty_is_explicit():
    assert percentile([], 0.5) is None
    assert percentile([1.0], 0.99) == 1.0
    assert percentile([1.0, 2.0, 3.0, 4.0], 0.50) == 2.5
    assert distribution([1.0, 2.0, 3.0])["p95"] == pytest.approx(2.9)
    with pytest.raises(ValueError, match="quantile"):
        percentile([1.0], 1.1)


def test_topic_metric_reports_hz_age_bandwidth_and_stamp_anomalies():
    metric = TopicMetric()
    metric.observe(
        arrival_s=10.0,
        stamp_s=1.0,
        simulation_s=1.1,
        size_bytes=100,
    )
    metric.observe(
        arrival_s=10.1,
        stamp_s=1.1,
        simulation_s=1.15,
        size_bytes=120,
    )
    metric.observe(
        arrival_s=10.2,
        stamp_s=1.1,
        simulation_s=1.05,
        size_bytes=80,
    )
    metric.observe(
        arrival_s=10.3,
        stamp_s=0.9,
        simulation_s=1.2,
        size_bytes=100,
    )

    report = metric.report()
    assert report["samples"] == 4
    assert report["wall_hz"] == pytest.approx(10.0)
    assert report["bytes_total"] == 400
    assert report["duplicate_stamps"] == 1
    assert report["rollback_stamps"] == 1
    assert report["future_stamps"] == 1
    assert report["age_s"]["p99"] is not None
    assert report["payload_throughput_estimate_bytes_per_s"] \
        == pytest.approx(4000.0 / 3.0)


def test_rtf_excludes_reset_rollback_but_records_epoch():
    metric = RtfMetric()
    metric.observe(1.0, 10.0)
    metric.observe(2.0, 11.0)
    metric.observe(3.0, 0.0)
    metric.observe(4.0, 1.0)

    report = metric.report()
    assert report["measured"] == pytest.approx(1.0)
    assert report["simulation_delta_s"] == pytest.approx(2.0)
    assert report["steady_wall_delta_s"] == pytest.approx(2.0)
    assert report["epoch_rollbacks"] == 1


def test_rtf_collapses_duplicate_clock_messages_before_distribution():
    metric = RtfMetric()
    metric.observe(10.0, 1.0)
    metric.observe(10.001, 1.0)
    metric.observe(11.0, 2.0)
    metric.observe(11.001, 2.0)

    report = metric.report()
    assert report["samples"] == 4
    assert report["unique_samples"] == 2
    assert report["measured"] == pytest.approx(1.0 / 1.001)
    assert report["instantaneous"]["p50"] == pytest.approx(1.0)


def test_rtf_includes_trailing_wall_stall():
    metric = RtfMetric()
    metric.observe(10.0, 1.0)
    metric.observe(11.0, 2.0)
    metric.observe(16.0, 2.0)

    report = metric.report()

    assert report["measured"] == pytest.approx(1.0 / 6.0)
    assert report["steady_wall_delta_s"] == pytest.approx(6.0)


def test_camera_stamp_matching_counts_duplicate_messages():
    report = camera_stamp_report(
        [1.0, 1.0, 2.0, 4.0],
        [1.0, 2.0, 2.0, 3.0],
    )

    assert report["exact_stamp_matches"] == 2
    assert report["image_without_camera_info"] == 2
    assert report["camera_info_without_image"] == 2
    assert report["image_exact_match_ratio"] == pytest.approx(0.5)
    assert report["image_camera_info_nearest_stamp_delta_s"]["max"] == 1.0


def test_registered_process_cpu_delta_uses_one_core_percent():
    before = {
        "isaac": {
            "pid": 10,
            "process_group": 10,
            "member_count": 2,
            "member_pids": [10, 11],
            "member_cpu_seconds": {"10:100": 1.0, "11:110": 2.0},
            "aggregation": "process_group",
            "leader_start_ticks": 100,
            "cpu_seconds": 3.0,
            "rss_bytes": 100,
        },
        "stale": {"state": "stale"},
    }
    after = {
        "isaac": {
            "pid": 10,
            "process_group": 10,
            "member_count": 2,
            "member_pids": [10, 11],
            "member_cpu_seconds": {"10:100": 2.0, "11:110": 4.0},
            "aggregation": "process_group",
            "leader_start_ticks": 100,
            "cpu_seconds": 5.0,
            "rss_bytes": 200,
            "state": "R",
        },
        "stale": {"state": "stale"},
    }

    report = _process_delta(before, after, 4.0)
    assert report["isaac"]["cpu_percent_one_core"] == pytest.approx(75.0)
    assert report["isaac"]["rss_bytes"] == 200
    assert report["isaac"]["process_group"] == 10
    assert report["isaac"]["member_count"] == 2
    assert report["isaac"]["aggregation"] == "process_group"
    assert report["isaac"]["cpu_sample_member_set_stable"] is True
    assert report["stale"]["cpu_percent_one_core"] is None


def test_registered_process_cpu_delta_is_withheld_when_members_change():
    identity = {
        "pid": 10,
        "process_group": 10,
        "leader_start_ticks": 100,
        "aggregation": "process_group_and_descendants",
    }
    before = {
        "ros": {
            **identity,
            "member_cpu_seconds": {"10:100": 1.0, "11:110": 2.0},
        }
    }
    after = {
        "ros": {
            **identity,
            "member_cpu_seconds": {"10:100": 2.0, "12:120": 0.5},
        }
    }

    report = _process_delta(before, after, 2.0)["ros"]

    assert report["cpu_percent_one_core"] is None
    assert report["cpu_sample_member_set_stable"] is False
    assert report["cpu_sample_removed_members"] == ["11:110"]
    assert report["cpu_sample_added_members"] == ["12:120"]


@pytest.mark.parametrize(
    ("arguments", "expected"),
    [
        (["bash", "/repo/scripts/run_ros.sh", "navigation"], "navigation"),
        (["ros2", "launch", "robot_bringup", "mapping_bringup.launch.py"], "mapping"),
        (["ros2", "launch", "pkg", "x.py", "operation:=localization"], "localization"),
        (["bash", "/repo/scripts/run_ros.sh", "invalid"], None),
    ],
)
def test_runtime_operation_infers_supervisor_and_launch_cmdlines(arguments, expected):
    assert _infer_ros_operation(arguments) == expected


def test_registered_process_snapshot_aggregates_dedicated_group(tmp_path: Path):
    process = subprocess.Popen(
        ["bash", "-c", "sleep 30 & wait"],
        start_new_session=True,
    )
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            snapshot = _proc_stat(process.pid)
            group_members = 0
            for child in Path("/proc").iterdir():
                if not child.name.isdigit():
                    continue
                try:
                    group_members += (
                        _proc_stat(int(child.name))["process_group"]
                        == process.pid
                    )
                except (OSError, ValueError):
                    continue
            if (
                snapshot["process_group"] == process.pid
                and group_members >= 2
            ):
                break
            time.sleep(0.01)
        metadata = tmp_path / "ros.pid"
        metadata.write_text(
            "\n".join([
                f"pid={process.pid}",
                f"process_group={process.pid}",
                f"leader_start_ticks={snapshot['start_ticks']}",
                "boot_id=" + Path(
                    "/proc/sys/kernel/random/boot_id"
                ).read_text(encoding="utf-8").strip(),
            ]) + "\n",
            encoding="utf-8",
        )

        report = _registered_process_snapshot(tmp_path)["ros"]

        assert report["pid"] == process.pid
        assert report["process_group"] == process.pid
        assert report["leader_start_ticks"] == snapshot["start_ticks"]
        assert report["member_count"] >= 2
        assert process.pid in report["member_pids"]
        assert report["member_cpu_seconds"]
    finally:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)


def test_registered_members_add_same_user_descendants_without_unrelated_processes():
    def process(pid, parent, group, uid=1000):
        return {
            "pid": pid,
            "parent_pid": parent,
            "process_group": group,
            "uid": uid,
        }

    processes = {
        100: process(100, 1, 100),
        101: process(101, 100, 100),
        200: process(200, 100, 200),
        201: process(201, 200, 200),
        300: process(300, 1, 300),
        400: process(400, 100, 400, uid=2000),
        401: process(401, 400, 400),
    }

    members = _registered_process_members(
        processes,
        leader_pid=100,
        process_group=100,
        include_descendants=True,
        owner_uid=1000,
    )

    assert {member["pid"] for member in members} == {100, 101, 200, 201}


def test_registered_ros_supervisor_aggregates_cross_pgid_descendants(
        tmp_path: Path):
    child_pid_file = tmp_path / "launch.pid"
    process = subprocess.Popen(
        [
            "bash",
            "-c",
            f'setsid bash -c "sleep 30 & wait" & '
            f'echo $! > "{child_pid_file}"; wait',
        ],
        start_new_session=True,
    )
    unrelated = subprocess.Popen(["sleep", "30"], start_new_session=True)
    try:
        deadline = time.monotonic() + 2.0
        while not child_pid_file.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        child_pid = int(child_pid_file.read_text(encoding="utf-8"))
        snapshot = _proc_stat(process.pid)
        (tmp_path / "ros.pid").write_text(
            "\n".join([
                f"pid={process.pid}",
                f"process_group={process.pid}",
                f"leader_start_ticks={snapshot['start_ticks']}",
                "boot_id=" + Path(
                    "/proc/sys/kernel/random/boot_id"
                ).read_text(encoding="utf-8").strip(),
                "component=ros",
            ]) + "\n",
            encoding="utf-8",
        )

        report = _registered_process_snapshot(tmp_path)["ros"]

        assert report["aggregation"] == "process_group_and_descendants"
        assert child_pid in report["member_pids"]
        assert unrelated.pid not in report["member_pids"]
        assert report["member_count"] >= 3
    finally:
        for child in (process, unrelated):
            try:
                os.killpg(child.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        process.wait(timeout=5.0)
        unrelated.wait(timeout=5.0)


def test_cpu_and_gpu_deltas_are_explicit():
    assert _cpu_utilization(
        {"cpu": (100, 20), "cpu0": (50, 10)},
        {"cpu": (200, 60), "cpu0": (100, 20)},
    ) == {"cpu": 60.0, "cpu0": 80.0}
    delta = _gpu_delta(
        {"devices": [{"name": "gpu", "memory_used_mib": 100.0}]},
        {"devices": [{"name": "gpu", "memory_used_mib": 125.0}]},
    )
    assert delta["devices"][0]["memory_used_mib"] == 25.0
