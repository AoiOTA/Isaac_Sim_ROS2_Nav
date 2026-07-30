"""Measure simulation RTF, ROS timing, TF lag, and host load."""

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import platform
import statistics
import subprocess
import time
from typing import Any


TOPIC_NAMES = {
    "clock": "/clock",
    "pointcloud": "/lidar/points_raw",
    "pointcloud_safety": "/lidar/points_scan",
    "scan": "/scan",
    "scan_safety": "/scan_safety",
    "imu": "/imu/data",
    "joint_states": "/joint_states",
    "wheel_odom": "/wheel/odom",
    "odom": "/odom",
    "global_plan": "/plan",
    "local_plan": "/optimal_trajectory",
    "cmd_vel_nav": "/cmd_vel_nav",
    "cmd_vel_smoothed": "/cmd_vel_smoothed",
    "cmd_vel": "/cmd_vel",
    "camera_image": "/camera/front/image_raw",
    "camera_info": "/camera/front/camera_info",
}
TOPIC_TYPES = {
    "clock": "rosgraph_msgs/msg/Clock",
    "pointcloud": "sensor_msgs/msg/PointCloud2",
    "pointcloud_safety": "sensor_msgs/msg/PointCloud2",
    "scan": "sensor_msgs/msg/LaserScan",
    "scan_safety": "sensor_msgs/msg/LaserScan",
    "imu": "sensor_msgs/msg/Imu",
    "joint_states": "sensor_msgs/msg/JointState",
    "wheel_odom": "nav_msgs/msg/Odometry",
    "odom": "nav_msgs/msg/Odometry",
    "global_plan": "nav_msgs/msg/Path",
    "local_plan": "nav_msgs/msg/Path",
    "cmd_vel_nav": "geometry_msgs/msg/Twist",
    "cmd_vel_smoothed": "geometry_msgs/msg/Twist",
    "cmd_vel": "geometry_msgs/msg/Twist",
    "camera_image": "sensor_msgs/msg/Image",
    "camera_info": "sensor_msgs/msg/CameraInfo",
}
TF_PAIRS = {
    "map->odom": ("map", "odom"),
    "odom->base_link": ("odom", "base_link"),
    "map->base_link": ("map", "base_link"),
    "base_link->lidar_link": ("base_link", "lidar_link"),
    "base_link->imu_link": ("base_link", "imu_link"),
    "base_link->camera_front_link": ("base_link", "camera_front_link"),
    "camera_front_link->camera_front_optical_frame": (
        "camera_front_link",
        "camera_front_optical_frame",
    ),
    "base_link->camera_front_optical_frame": (
        "base_link",
        "camera_front_optical_frame",
    ),
}
MAX_DURATION_S = 3600.0
LOG_PATTERNS = {
    "controller_missed_rate": ("control loop missed", "missed its desired rate"),
    "collision_invalid_source": ("invalid source", "source timeout"),
    "future_extrapolation": ("future extrapolation", "extrapolation into the future"),
    "tf_old_data": ("tf_old_data", "earlier than all the data"),
    "queue_full": ("queue is full", "message filter dropping"),
    "failed_progress": ("failed to make progress",),
    "optimizer_reset": ("optimizer reset", "resetting optimizer"),
    "costmap_clear": ("clearing costmap", "clear entire costmap"),
    "goal_succeeded": ("goal succeeded", "succeeded"),
    "goal_failed": ("goal failed", "failed with result"),
}


def percentile(values: list[float], quantile: float) -> float | None:
    """Return a linearly interpolated percentile without third-party packages."""
    if not values:
        return None
    if not 0.0 <= quantile <= 1.0:
        raise ValueError("quantile must be within [0, 1]")
    ordered = sorted(float(value) for value in values)
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def distribution(values: list[float]) -> dict[str, float | int | None]:
    """Summarize a metric with the percentiles required by the run plan."""
    return {
        "samples": len(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values) if values else None,
        "mean": statistics.fmean(values) if values else None,
    }


@dataclass
class TopicMetric:
    """Timing and estimated ROS-payload evidence for one ROS Topic."""

    arrivals: list[float] = field(default_factory=list)
    stamps: list[float] = field(default_factory=list)
    ages: list[float] = field(default_factory=list)
    bytes_total: int = 0
    duplicate_stamps: int = 0
    rollback_stamps: int = 0
    future_stamps: int = 0

    def observe(
        self,
        *,
        arrival_s: float,
        stamp_s: float | None,
        simulation_s: float | None,
        size_bytes: int,
    ) -> None:
        if self.arrivals and arrival_s < self.arrivals[-1]:
            raise ValueError("steady arrival time regressed")
        self.arrivals.append(float(arrival_s))
        self.bytes_total += max(0, int(size_bytes))
        if stamp_s is None:
            return
        stamp_s = float(stamp_s)
        if self.stamps:
            if stamp_s == self.stamps[-1]:
                self.duplicate_stamps += 1
            elif stamp_s < self.stamps[-1]:
                self.rollback_stamps += 1
        self.stamps.append(stamp_s)
        if simulation_s is not None and stamp_s > 0.0:
            age = float(simulation_s) - stamp_s
            self.ages.append(age)
            if age < -1e-6:
                self.future_stamps += 1

    def report(self) -> dict[str, Any]:
        arrival_periods = [
            current - previous
            for previous, current in zip(self.arrivals, self.arrivals[1:])
        ]
        stamp_periods = [
            current - previous
            for previous, current in zip(self.stamps, self.stamps[1:])
            if current > previous
        ]
        wall_span = (
            self.arrivals[-1] - self.arrivals[0]
            if len(self.arrivals) >= 2
            else 0.0
        )
        return {
            "samples": len(self.arrivals),
            "wall_hz": (
                (len(self.arrivals) - 1) / wall_span
                if wall_span > 0.0
                else None
            ),
            "arrival_period_s": distribution(arrival_periods),
            "stamp_period_s": distribution(stamp_periods),
            "age_s": distribution(self.ages),
            "lag_s": distribution([age for age in self.ages if age >= 0.0]),
            "lead_s": distribution([-age for age in self.ages if age < 0.0]),
            "age_reference": "latest_received_clock",
            "bytes_total": self.bytes_total,
            "payload_throughput_estimate_bytes_per_s": (
                self.bytes_total / wall_span if wall_span > 0.0 else None
            ),
            "duplicate_stamps": self.duplicate_stamps,
            "rollback_stamps": self.rollback_stamps,
            "future_stamps": self.future_stamps,
        }


@dataclass
class RtfMetric:
    """Measure positive simulation deltas against steady-wall deltas."""

    observations: list[tuple[float, float]] = field(default_factory=list)
    rollbacks: int = 0

    def observe(self, steady_s: float, simulation_s: float) -> None:
        if self.observations and steady_s < self.observations[-1][0]:
            raise ValueError("steady clock regressed")
        if self.observations and simulation_s < self.observations[-1][1]:
            self.rollbacks += 1
        self.observations.append((float(steady_s), float(simulation_s)))

    def report(self) -> dict[str, Any]:
        unique_observations = []
        for observation in self.observations:
            if (
                unique_observations
                and observation[1] == unique_observations[-1][1]
            ):
                continue
            unique_observations.append(observation)
        ratios = []
        simulation_delta = 0.0
        wall_delta = 0.0
        epochs: list[list[tuple[float, float]]] = []
        for observation in self.observations:
            if epochs and observation[1] < epochs[-1][-1][1]:
                epochs.append([observation])
            elif epochs:
                epochs[-1].append(observation)
            else:
                epochs.append([observation])
        for epoch in epochs:
            if len(epoch) < 2:
                continue
            epoch_wall = epoch[-1][0] - epoch[0][0]
            epoch_simulation = epoch[-1][1] - epoch[0][1]
            if epoch_wall > 0.0 and epoch_simulation >= 0.0:
                wall_delta += epoch_wall
                simulation_delta += epoch_simulation
        for (wall_a, sim_a), (wall_b, sim_b) in zip(
            unique_observations, unique_observations[1:]
        ):
            delta_wall = wall_b - wall_a
            delta_sim = sim_b - sim_a
            if delta_wall <= 0.0 or delta_sim < 0.0:
                continue
            ratios.append(delta_sim / delta_wall)
        return {
            "samples": len(self.observations),
            "unique_samples": len(unique_observations),
            "measured": (
                simulation_delta / wall_delta if wall_delta > 0.0 else None
            ),
            "instantaneous": distribution(ratios),
            "simulation_delta_s": simulation_delta,
            "steady_wall_delta_s": wall_delta,
            "epoch_rollbacks": self.rollbacks,
        }


def camera_stamp_report(
    image_stamps: list[float], info_stamps: list[float]
) -> dict[str, Any]:
    images = sorted(float(stamp) for stamp in image_stamps)
    infos = sorted(float(stamp) for stamp in info_stamps)
    nearest_deltas = []
    if images:
        for stamp in infos:
            insertion = bisect_left(images, stamp)
            candidates = []
            if insertion < len(images):
                candidates.append(images[insertion])
            if insertion > 0:
                candidates.append(images[insertion - 1])
            nearest_deltas.append(min(abs(stamp - item) for item in candidates))
    image_counts = Counter(images)
    info_counts = Counter(infos)
    exact_matches = sum((image_counts & info_counts).values())
    return {
        "image_camera_info_nearest_stamp_delta_s": distribution(
            nearest_deltas
        ),
        "exact_stamp_matches": exact_matches,
        "image_without_camera_info": sum((image_counts - info_counts).values()),
        "camera_info_without_image": sum((info_counts - image_counts).values()),
        "image_exact_match_ratio": (
            exact_matches / len(images) if images else None
        ),
    }


def _stamp_seconds(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _read_cpu_times() -> tuple[int, int]:
    fields = Path("/proc/stat").read_text(encoding="utf-8").splitlines()[0].split()
    values = [int(value) for value in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def _read_cpu_times_by_core() -> dict[str, tuple[int, int]]:
    result = {}
    for line in Path("/proc/stat").read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if not fields or not fields[0].startswith("cpu"):
            continue
        if fields[0] != "cpu" and not fields[0][3:].isdigit():
            continue
        values = [int(value) for value in fields[1:]]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        result[fields[0]] = (sum(values), idle)
    return result


def _cpu_utilization(
    before: dict[str, tuple[int, int]],
    after: dict[str, tuple[int, int]],
) -> dict[str, float | None]:
    result = {}
    for name in sorted(set(before) | set(after)):
        first = before.get(name)
        last = after.get(name)
        if first is None or last is None:
            result[name] = None
            continue
        total_delta = last[0] - first[0]
        idle_delta = last[1] - first[1]
        result[name] = (
            100.0 * (total_delta - idle_delta) / total_delta
            if total_delta > 0
            else None
        )
    return result


def _cpu_model() -> str | None:
    for line in Path("/proc/cpuinfo").read_text(encoding="utf-8").splitlines():
        if line.startswith("model name"):
            return line.partition(":")[2].strip() or None
    return None


def _read_memory() -> dict[str, int | float | None]:
    values = {}
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        name, raw = line.split(":", 1)
        values[name] = int(raw.strip().split()[0]) * 1024
    total = values.get("MemTotal", 0)
    available = values.get("MemAvailable", 0)
    swap_total = values.get("SwapTotal", 0)
    swap_free = values.get("SwapFree", 0)
    return {
        "total_bytes": total,
        "available_bytes": available,
        "used_percent": (
            100.0 * (total - available) / total if total else None
        ),
        "swap_total_bytes": swap_total,
        "swap_free_bytes": swap_free,
        "swap_used_percent": (
            100.0 * (swap_total - swap_free) / swap_total
            if swap_total
            else 0.0
        ),
    }


def _cpu_frequency_summary() -> dict[str, float | int | None]:
    values = []
    for path in Path("/sys/devices/system/cpu").glob(
        "cpu[0-9]*/cpufreq/scaling_cur_freq"
    ):
        try:
            values.append(float(path.read_text(encoding="utf-8").strip()) / 1000.0)
        except (OSError, ValueError):
            continue
    return {
        "samples": len(values),
        "minimum_mhz": min(values) if values else None,
        "mean_mhz": statistics.fmean(values) if values else None,
        "maximum_mhz": max(values) if values else None,
    }


def _cpu_scaling_snapshot() -> dict[str, Any]:
    per_cpu = {}
    for cpu_path in sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*")):
        cpufreq = cpu_path / "cpufreq"
        if not cpufreq.is_dir():
            continue

        def read(name: str) -> str | None:
            try:
                return (cpufreq / name).read_text(encoding="utf-8").strip()
            except OSError:
                return None

        def mhz(name: str) -> float | None:
            value = read(name)
            try:
                return float(value) / 1000.0 if value is not None else None
            except ValueError:
                return None

        per_cpu[cpu_path.name] = {
            "governor": read("scaling_governor"),
            "driver": read("scaling_driver"),
            "energy_performance_preference": read(
                "energy_performance_preference"
            ),
            "current_mhz": mhz("scaling_cur_freq"),
            "minimum_mhz": mhz("scaling_min_freq"),
            "maximum_mhz": mhz("scaling_max_freq"),
        }
    return {
        "per_cpu": per_cpu,
        "governors": sorted({
            value["governor"]
            for value in per_cpu.values()
            if value["governor"]
        }),
        "drivers": sorted({
            value["driver"]
            for value in per_cpu.values()
            if value["driver"]
        }),
    }


def _temperature_snapshot() -> dict[str, Any]:
    readings = []
    for hwmon in sorted(Path("/sys/class/hwmon").glob("hwmon*")):
        try:
            device = (hwmon / "name").read_text(encoding="utf-8").strip()
        except OSError:
            device = hwmon.name
        for input_path in sorted(hwmon.glob("temp*_input")):
            stem = input_path.name.removesuffix("_input")
            try:
                value_c = float(
                    input_path.read_text(encoding="utf-8").strip()
                ) / 1000.0
            except (OSError, ValueError):
                continue
            try:
                label = (hwmon / f"{stem}_label").read_text(
                    encoding="utf-8"
                ).strip()
            except OSError:
                label = stem
            readings.append({
                "device": device,
                "label": label,
                "temperature_c": value_c,
            })
    values = [item["temperature_c"] for item in readings]
    return {
        "readings": readings,
        "maximum_c": max(values) if values else None,
    }


def _throttle_snapshot() -> dict[str, Any]:
    counters = {}
    for path in sorted(Path("/sys/devices/system/cpu").glob(
        "cpu[0-9]*/thermal_throttle/*_throttle_count"
    )):
        try:
            counters[str(path)] = int(
                path.read_text(encoding="utf-8").strip()
            )
        except (OSError, ValueError):
            continue
    return {
        "available": bool(counters),
        "counters": counters,
        "total": sum(counters.values()),
    }


def _gpu_snapshot() -> dict[str, Any]:
    command = [
        "nvidia-smi",
        (
            "--query-gpu=name,driver_version,utilization.gpu,"
            "utilization.memory,memory.used,memory.total,pstate,power.draw,"
            "temperature.gpu,clocks.gr,clocks.mem"
        ),
        "--format=csv,noheader,nounits",
    ]
    try:
        result = subprocess.run(
            command,
            text=True,
            capture_output=True,
            check=False,
            timeout=3.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"available": False, "reason": str(exc)}
    if result.returncode != 0:
        return {
            "available": False,
            "reason": result.stderr.strip() or f"exit {result.returncode}",
        }
    rows = []
    for line in result.stdout.splitlines():
        parts = [part.strip() for part in line.split(",")]
        if len(parts) != 11:
            continue
        try:
            rows.append({
                "name": parts[0],
                "driver_version": parts[1],
                "utilization_percent": float(parts[2]),
                "memory_utilization_percent": float(parts[3]),
                "memory_used_mib": float(parts[4]),
                "memory_total_mib": float(parts[5]),
                "pstate": parts[6],
                "power_w": float(parts[7]),
                "temperature_c": float(parts[8]),
                "graphics_clock_mhz": float(parts[9]),
                "memory_clock_mhz": float(parts[10]),
            })
        except ValueError:
            continue
    process_rows = []
    try:
        process_result = subprocess.run(
            [
                "nvidia-smi",
                "--query-compute-apps=pid,process_name,used_gpu_memory",
                "--format=csv,noheader,nounits",
            ],
            text=True,
            capture_output=True,
            check=False,
            timeout=3.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        process_result = None
    if process_result is not None and process_result.returncode == 0:
        for line in process_result.stdout.splitlines():
            parts = [part.strip() for part in line.split(",", 2)]
            if len(parts) != 3:
                continue
            try:
                process_rows.append({
                    "pid": int(parts[0]),
                    "process_name": parts[1],
                    "used_gpu_memory_mib": float(parts[2]),
                })
            except ValueError:
                continue
    return {
        "available": bool(rows),
        "devices": rows,
        "compute_processes": process_rows,
        "reason": None if rows else "nvidia-smi returned no parseable rows",
    }


def _gpu_delta(before: dict[str, Any], after: dict[str, Any]) -> dict[str, Any]:
    numeric_fields = (
        "utilization_percent",
        "memory_utilization_percent",
        "memory_used_mib",
        "power_w",
        "temperature_c",
        "graphics_clock_mhz",
        "memory_clock_mhz",
    )
    device_deltas = []
    for first, last in zip(
        before.get("devices", []), after.get("devices", [])
    ):
        device_deltas.append({
            "name": last.get("name", first.get("name")),
            **{
                field: last[field] - first[field]
                for field in numeric_fields
                if isinstance(first.get(field), (int, float))
                and isinstance(last.get(field), (int, float))
            },
        })
    return {"devices": device_deltas}


def _git_metadata(root: Path) -> dict[str, Any]:
    def git(*arguments: str) -> str | None:
        result = subprocess.run(
            ["git", "-C", str(root), *arguments],
            text=True,
            capture_output=True,
            check=False,
            timeout=3.0,
        )
        return result.stdout.strip() if result.returncode == 0 else None

    status = git("status", "--porcelain")
    return {
        "commit": git("rev-parse", "HEAD"),
        "branch": git("branch", "--show-current"),
        "dirty": bool(status) if status is not None else None,
    }


def _proc_stat(pid: int) -> dict[str, Any]:
    raw = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    closing_parenthesis = raw.rfind(")")
    if closing_parenthesis < 0:
        raise ValueError(f"invalid /proc/{pid}/stat")
    fields = raw[closing_parenthesis + 2:].split()
    if len(fields) < 22:
        raise ValueError(f"short /proc/{pid}/stat")
    return {
        "pid": pid,
        "state": fields[0],
        "parent_pid": int(fields[1]),
        "process_group": int(fields[2]),
        "cpu_ticks": int(fields[11]) + int(fields[12]),
        "start_ticks": int(fields[19]),
        "rss_pages": int(fields[21]),
        "uid": Path(f"/proc/{pid}").stat().st_uid,
    }


def _registered_process_members(
    processes: dict[int, dict[str, Any]],
    *,
    leader_pid: int,
    process_group: int,
    include_descendants: bool,
    owner_uid: int,
) -> list[dict[str, Any]]:
    """Select the registered PGID plus authenticated supervisor descendants."""
    selected = {
        pid for pid, process in processes.items()
        if process["process_group"] == process_group
    }
    if include_descendants:
        children: dict[int, list[int]] = defaultdict(list)
        for pid, process in processes.items():
            if process["uid"] == owner_uid:
                children[process["parent_pid"]].append(pid)
        pending = [leader_pid]
        descendants = {leader_pid}
        while pending:
            parent = pending.pop()
            for child in children.get(parent, []):
                if child not in descendants:
                    descendants.add(child)
                    pending.append(child)
        selected.update(descendants)
    return [processes[pid] for pid in sorted(selected)]


def _registered_process_snapshot(runtime_dir: Path) -> dict[str, Any]:
    result = {}
    ticks_per_second = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
    page_size = os.sysconf("SC_PAGE_SIZE")
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(
        encoding="utf-8"
    ).strip()
    for metadata in sorted(runtime_dir.glob("*.pid")):
        values = {}
        try:
            for line in metadata.read_text(encoding="utf-8").splitlines():
                key, separator, value = line.partition("=")
                if separator:
                    values[key] = value
            pid = int(values["pid"])
            process_group = int(values.get("process_group", pid))
            if values.get("boot_id", boot_id) != boot_id:
                raise OSError("runtime metadata belongs to another boot")
            if process_group != pid:
                raise OSError("runtime does not have a dedicated process group")
            processes = {}
            for process_path in Path("/proc").iterdir():
                if not process_path.name.isdigit():
                    continue
                try:
                    stat = _proc_stat(int(process_path.name))
                except (OSError, ValueError, IndexError):
                    continue
                processes[stat["pid"]] = stat
            leader = processes.get(pid)
            if leader is None or leader["state"] == "Z":
                raise OSError(f"registered leader {pid} is not running")
            expected_start = values.get("leader_start_ticks")
            if (
                expected_start is not None
                and int(expected_start) != leader["start_ticks"]
            ):
                raise OSError("registered leader identity changed")
            component = values.get("component", metadata.stem)
            if component != metadata.stem:
                raise OSError("runtime component identity changed")
            authenticated_supervisor = (
                component == "ros"
                and values.get("boot_id") == boot_id
                and expected_start is not None
                and leader["uid"] == os.geteuid()
            )
            members = _registered_process_members(
                processes,
                leader_pid=pid,
                process_group=process_group,
                include_descendants=authenticated_supervisor,
                owner_uid=leader["uid"],
            )
            states = {
                state: sum(member["state"] == state for member in members)
                for state in sorted({member["state"] for member in members})
            }
            active_members = [
                member for member in members if member["state"] != "Z"
            ]
            zombie_members = [
                member for member in members if member["state"] == "Z"
            ]
            result[metadata.stem] = {
                "pid": pid,
                "process_group": process_group,
                "leader_start_ticks": leader["start_ticks"],
                "member_count": len(active_members),
                "member_pids": sorted(
                    member["pid"] for member in active_members
                ),
                "member_cpu_seconds": {
                    f"{member['pid']}:{member['start_ticks']}": (
                        member["cpu_ticks"] / ticks_per_second
                    )
                    for member in active_members
                },
                "aggregation": (
                    "process_group_and_descendants"
                    if authenticated_supervisor else "process_group"
                ),
                "zombie_member_count": len(zombie_members),
                "zombie_member_pids": sorted(
                    member["pid"] for member in zombie_members
                ),
                "cpu_seconds": sum(
                    member["cpu_ticks"] for member in active_members
                ) / ticks_per_second,
                "rss_bytes": sum(
                    member["rss_pages"] for member in active_members
                ) * page_size,
                "state": leader["state"],
                "states": states,
            }
        except (KeyError, OSError, ValueError, IndexError):
            result[metadata.stem] = {"state": "stale"}
    return result


def _registered_process_cmdlines(runtime_dir: Path) -> dict[str, list[str]]:
    result = {}
    for metadata in sorted(runtime_dir.glob("*.pid")):
        try:
            values = dict(
                line.split("=", 1)
                for line in metadata.read_text(encoding="utf-8").splitlines()
                if "=" in line
            )
            pid = int(values["pid"])
            raw = Path(f"/proc/{pid}/cmdline").read_bytes()
            result[metadata.stem] = [
                item.decode("utf-8", errors="replace")
                for item in raw.split(b"\0")
                if item
            ]
        except (KeyError, OSError, ValueError):
            continue
    return result


def _infer_ros_operation(arguments: list[str]) -> str | None:
    prefix = "operation:="
    explicit = next(
        (item[len(prefix):] for item in arguments if item.startswith(prefix)),
        None,
    )
    if explicit:
        return explicit
    operations = {"mapping", "incremental_mapping", "localization", "navigation"}
    for index, item in enumerate(arguments[:-1]):
        if Path(item).name == "run_ros.sh" and arguments[index + 1] in operations:
            return arguments[index + 1]
    for item in arguments:
        if item.endswith("_bringup.launch.py"):
            return Path(item).name.removesuffix("_bringup.launch.py")
    return None


def _runtime_metadata(
    project_root: Path,
    runtime_dir: Path,
    scaling: dict[str, Any],
) -> dict[str, Any]:
    import yaml

    config_path = Path(os.environ.get(
        "ISAAC_NAV_PROJECT_CONFIG",
        project_root / "isaac_sim/configs/project.yaml",
    ))
    try:
        project = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        simulation = project.get("simulation", {})
    except (OSError, yaml.YAMLError, AttributeError):
        simulation = {}
    commands = _registered_process_cmdlines(runtime_dir)
    isaac = commands.get("isaac", [])
    ros = commands.get("ros", [])

    def option(arguments: list[str], name: str) -> str | None:
        try:
            return arguments[arguments.index(name) + 1]
        except (ValueError, IndexError):
            return None

    def launch_option(arguments: list[str], name: str) -> str | None:
        prefix = f"{name}:="
        return next(
            (item[len(prefix):] for item in arguments if item.startswith(prefix)),
            None,
        )

    headless = bool(simulation.get("headless", False))
    if "--headless" in isaac:
        headless = True
    elif "--no-headless" in isaac:
        headless = False
    operation = _infer_ros_operation(ros)
    posegraph_file = launch_option(ros, "posegraph_file")
    map_file = launch_option(ros, "map_file")
    posegraph_version = (
        Path(posegraph_file).name.removesuffix(".posegraph")
        if posegraph_file
        else None
    )
    map_version = Path(map_file).stem if map_file else posegraph_version
    camera_profile = option(isaac, "--camera-profile")
    if camera_profile is None:
        camera_profile = "off" if headless else "monitoring"
    governors = scaling.get("governors", [])
    cpu_mode = governors[0] if len(governors) == 1 else governors
    return {
        "operation": operation,
        "map_version": map_version,
        "posegraph_version": posegraph_version,
        "initial_pose_source": launch_option(ros, "initial_pose_source"),
        "odometry_mode": (
            option(isaac, "--mode")
            or launch_option(ros, "odometry_mode")
            or simulation.get("odometry_mode")
        ),
        "structure_tf_source": (
            option(isaac, "--structure-tf-source")
            or launch_option(ros, "structure_tf_source")
            or simulation.get("structure_tf_source")
        ),
        "pacing_mode": (
            option(isaac, "--pacing-mode")
            or simulation.get("pacing_mode")
        ),
        "target_rtf": (
            option(isaac, "--target-rtf")
            or simulation.get("target_realtime_factor")
        ),
        "cpu_mode": cpu_mode,
        "runtime_mode": "headless" if headless else "gui",
        "camera_profile": camera_profile,
        "camera_rviz_enabled": "rviz" in commands,
        "nav2_profile": launch_option(ros, "nav2_profile") or "stable",
        "nav2_profile_params_file": launch_option(
            ros, "nav2_profile_params_file"
        ),
        "ceres_num_threads": launch_option(ros, "ceres_num_threads") or "12",
        "registered_process_cmdlines": commands,
    }


def _process_delta(
    before: dict[str, Any],
    after: dict[str, Any],
    elapsed_s: float,
) -> dict[str, Any]:
    result = {}
    for component in sorted(set(before) | set(after)):
        first = before.get(component, {})
        last = after.get(component, {})
        cpu_delta = None
        first_member_cpu = first.get("member_cpu_seconds")
        last_member_cpu = last.get("member_cpu_seconds")
        member_set_stable = False
        removed_members: list[str] = []
        added_members: list[str] = []
        if isinstance(first_member_cpu, dict) and isinstance(last_member_cpu, dict):
            first_identities = set(first_member_cpu)
            last_identities = set(last_member_cpu)
            member_set_stable = first_identities == last_identities
            removed_members = sorted(first_identities - last_identities)
            added_members = sorted(last_identities - first_identities)
        if (
            elapsed_s > 0.0
            and member_set_stable
            and first.get("pid") == last.get("pid")
            and first.get("process_group") == last.get("process_group")
            and first.get("leader_start_ticks")
            == last.get("leader_start_ticks")
        ):
            member_deltas = [
                last_member_cpu[identity] - first_member_cpu[identity]
                for identity in sorted(first_member_cpu)
            ]
            if all(delta >= 0.0 for delta in member_deltas):
                cpu_delta = 100.0 * sum(member_deltas) / elapsed_s
        result[component] = {
            "pid": last.get("pid", first.get("pid")),
            "process_group": last.get(
                "process_group", first.get("process_group")
            ),
            "leader_start_ticks": last.get(
                "leader_start_ticks", first.get("leader_start_ticks")
            ),
            "member_count": last.get(
                "member_count", first.get("member_count")
            ),
            "member_pids": last.get(
                "member_pids", first.get("member_pids", [])
            ),
            "aggregation": last.get(
                "aggregation", first.get("aggregation")
            ),
            "cpu_sample_member_set_stable": member_set_stable,
            "cpu_sample_removed_members": removed_members,
            "cpu_sample_added_members": added_members,
            "zombie_member_count": last.get(
                "zombie_member_count", first.get("zombie_member_count", 0)
            ),
            "zombie_member_pids": last.get(
                "zombie_member_pids", first.get("zombie_member_pids", [])
            ),
            "cpu_percent_one_core": cpu_delta,
            "rss_bytes": last.get("rss_bytes"),
            "state": last.get("state", first.get("state", "unknown")),
            "states": last.get("states", first.get("states", {})),
        }
    return result


class RuntimeProfiler:
    """ROS adapter kept separate from the pure metric accumulators."""

    def __init__(self, node: Any, *, sample_period_s: float) -> None:
        from rclpy.duration import Duration
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry, Path
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import (
            CameraInfo,
            Image,
            Imu,
            JointState,
            LaserScan,
            PointCloud2,
        )
        from rcl_interfaces.msg import Log
        from tf2_ros import Buffer, TransformListener

        self.node = node
        self.metrics = {name: TopicMetric() for name in TOPIC_NAMES}
        self.rtf = RtfMetric()
        self.latest_simulation_s: float | None = None
        self.log_counts = defaultdict(int)
        self.tf_ages = {name: [] for name in TF_PAIRS}
        self.tf_future = defaultdict(int)
        self.tf_failures = defaultdict(int)
        self.tf_static = defaultdict(bool)
        self.image_stamps: list[float] = []
        self.info_stamps: list[float] = []
        self.camera_message_contracts: dict[str, dict[str, Any]] = {}
        self.goal_states: dict[str, int] = {}
        self.goal_started: dict[str, float] = {}
        self.goal_durations: list[float] = []
        self.goal_results = defaultdict(int)
        self._subscriptions = []
        reliable = QoSProfile(depth=20)
        reliable.reliability = ReliabilityPolicy.RELIABLE
        sensor_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        rosout = QoSProfile(
            depth=1000,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            lifespan=Duration(seconds=10.0),
        )

        self._subscriptions.extend([
            node.create_subscription(
                Clock,
                TOPIC_NAMES["clock"],
                self._clock_callback,
                sensor_qos,
            ),
            node.create_subscription(
                PointCloud2,
                TOPIC_NAMES["pointcloud"],
                lambda message: self._message_callback(
                    "pointcloud", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                PointCloud2,
                TOPIC_NAMES["pointcloud_safety"],
                lambda message: self._message_callback(
                    "pointcloud_safety", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                LaserScan,
                TOPIC_NAMES["scan"],
                lambda message: self._message_callback("scan", message),
                sensor_qos,
            ),
            node.create_subscription(
                LaserScan,
                TOPIC_NAMES["scan_safety"],
                lambda message: self._message_callback(
                    "scan_safety", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                Imu,
                TOPIC_NAMES["imu"],
                lambda message: self._message_callback("imu", message),
                sensor_qos,
            ),
            node.create_subscription(
                JointState,
                TOPIC_NAMES["joint_states"],
                lambda message: self._message_callback(
                    "joint_states", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                Odometry,
                TOPIC_NAMES["wheel_odom"],
                lambda message: self._message_callback(
                    "wheel_odom", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                Odometry,
                TOPIC_NAMES["odom"],
                lambda message: self._message_callback("odom", message),
                sensor_qos,
            ),
            node.create_subscription(
                Path,
                TOPIC_NAMES["global_plan"],
                lambda message: self._message_callback(
                    "global_plan", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                Path,
                TOPIC_NAMES["local_plan"],
                lambda message: self._message_callback(
                    "local_plan", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                Twist,
                TOPIC_NAMES["cmd_vel_nav"],
                lambda message: self._message_callback(
                    "cmd_vel_nav", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                Twist,
                TOPIC_NAMES["cmd_vel_smoothed"],
                lambda message: self._message_callback(
                    "cmd_vel_smoothed", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                Twist,
                TOPIC_NAMES["cmd_vel"],
                lambda message: self._message_callback("cmd_vel", message),
                sensor_qos,
            ),
            node.create_subscription(
                Image,
                TOPIC_NAMES["camera_image"],
                lambda message: self._message_callback(
                    "camera_image", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                CameraInfo,
                TOPIC_NAMES["camera_info"],
                lambda message: self._message_callback(
                    "camera_info", message
                ),
                sensor_qos,
            ),
            node.create_subscription(
                Log, "/rosout", self._log_callback, rosout
            ),
            node.create_subscription(
                GoalStatusArray,
                "/navigate_to_pose/_action/status",
                self._goal_status_callback,
                reliable,
            ),
        ])
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.tf_timer = node.create_timer(sample_period_s, self._sample_tf)

    def reset_measurements(self) -> None:
        self.metrics = {name: TopicMetric() for name in TOPIC_NAMES}
        self.rtf = RtfMetric()
        self.log_counts.clear()
        self.tf_ages = {name: [] for name in TF_PAIRS}
        self.tf_future.clear()
        self.tf_failures.clear()
        self.tf_static.clear()
        self.image_stamps.clear()
        self.info_stamps.clear()
        self.camera_message_contracts.clear()
        self.goal_states.clear()
        self.goal_started.clear()
        self.goal_durations.clear()
        self.goal_results.clear()

    def _serialized_size(self, message: Any) -> int:
        # Full CDR serialization of megapixel Images and PointCloud2 data in
        # the subscription callback can itself cause dropped sensor samples.
        # Count their dominant variable payload directly and reserve a small
        # fixed allowance for headers/field metadata.
        data = getattr(message, "data", None)
        if data is not None and hasattr(data, "__len__"):
            return len(data) + 256
        ranges = getattr(message, "ranges", None)
        if ranges is not None:
            intensities = getattr(message, "intensities", ())
            return 256 + 4 * (len(ranges) + len(intensities))
        poses = getattr(message, "poses", None)
        if poses is not None:
            return 128 + 400 * len(poses)
        from rclpy.serialization import serialize_message

        try:
            return len(serialize_message(message))
        except Exception:
            return 0

    def _clock_callback(self, message: Any) -> None:
        steady = time.monotonic()
        simulation = _stamp_seconds(message.clock)
        self.latest_simulation_s = simulation
        self.rtf.observe(steady, simulation)
        self.metrics["clock"].observe(
            arrival_s=steady,
            stamp_s=simulation,
            simulation_s=simulation,
            size_bytes=self._serialized_size(message),
        )

    def _message_callback(self, name: str, message: Any) -> None:
        header = getattr(message, "header", None)
        stamp = (
            _stamp_seconds(header.stamp)
            if header is not None
            else None
        )
        self.metrics[name].observe(
            arrival_s=time.monotonic(),
            stamp_s=stamp,
            simulation_s=self.latest_simulation_s,
            size_bytes=self._serialized_size(message),
        )
        if name == "camera_image":
            if stamp is not None:
                self.image_stamps.append(stamp)
            self.camera_message_contracts[name] = {
                "frame_id": getattr(header, "frame_id", None),
                "encoding": getattr(message, "encoding", None),
                "width": getattr(message, "width", None),
                "height": getattr(message, "height", None),
                "step": getattr(message, "step", None),
            }
        elif name == "camera_info":
            if stamp is not None:
                self.info_stamps.append(stamp)
            self.camera_message_contracts[name] = {
                "frame_id": getattr(header, "frame_id", None),
                "width": getattr(message, "width", None),
                "height": getattr(message, "height", None),
                "distortion_model": getattr(
                    message, "distortion_model", None
                ),
                "k": list(getattr(message, "k", ())),
                "p": list(getattr(message, "p", ())),
            }

    def _goal_status_callback(self, message: Any) -> None:
        now = time.monotonic()
        for item in message.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            status = int(item.status)
            previous = self.goal_states.get(goal_id)
            if status == previous:
                continue
            self.goal_states[goal_id] = status
            if status in (1, 2, 3):
                self.goal_started.setdefault(goal_id, now)
            elif status in (4, 5, 6):
                started = self.goal_started.pop(goal_id, None)
                if started is not None:
                    self.goal_durations.append(now - started)
                result_names = {4: "succeeded", 5: "canceled", 6: "aborted"}
                self.goal_results[result_names[status]] += 1

    def _log_callback(self, message: Any) -> None:
        text = str(message.msg).lower()
        for name, patterns in LOG_PATTERNS.items():
            if any(pattern in text for pattern in patterns):
                self.log_counts[name] += 1

    def _sample_tf(self) -> None:
        if self.latest_simulation_s is None:
            return
        from rclpy.time import Time
        from tf2_ros import TransformException

        for name, (target, source) in TF_PAIRS.items():
            try:
                transform = self.tf_buffer.lookup_transform(
                    target, source, Time()
                )
            except TransformException:
                self.tf_failures[name] += 1
                continue
            stamp = _stamp_seconds(transform.header.stamp)
            if stamp == 0.0:
                self.tf_static[name] = True
                continue
            age = self.latest_simulation_s - stamp
            self.tf_ages[name].append(age)
            if age < -1e-6:
                self.tf_future[name] += 1

    def _endpoint_report(self, topic: str) -> dict[str, Any]:
        def convert(endpoint: Any) -> dict[str, Any]:
            qos = endpoint.qos_profile
            return {
                "node_name": endpoint.node_name,
                "node_namespace": endpoint.node_namespace,
                "reliability": str(qos.reliability),
                "durability": str(qos.durability),
                "history": str(qos.history),
                "depth": qos.depth,
            }

        publishers = self.node.get_publishers_info_by_topic(topic)
        subscriptions = self.node.get_subscriptions_info_by_topic(topic)
        return {
            "publisher_count": len(publishers),
            "subscription_count": len(subscriptions),
            "external_subscription_count": sum(
                endpoint.node_name != self.node.get_name()
                for endpoint in subscriptions
            ),
            "publishers": [convert(item) for item in publishers],
            "subscriptions": [convert(item) for item in subscriptions],
        }

    def report(self) -> dict[str, Any]:
        topics = {}
        for name, topic in TOPIC_NAMES.items():
            topics[name] = self.metrics[name].report()
            topics[name]["topic"] = topic
            topics[name]["type"] = TOPIC_TYPES[name]
            topics[name]["endpoints"] = self._endpoint_report(topic)
            topics[name]["payload_size_estimator"] = (
                "variable_payload_plus_header_estimate"
                if name in {
                    "pointcloud", "pointcloud_safety", "scan", "scan_safety",
                    "global_plan", "local_plan", "camera_image",
                }
                else "ros_cdr_serialized"
            )
        return {
            "rtf": self.rtf.report(),
            "topics": topics,
            "tf": {
                name: {
                    "target": target,
                    "source": source,
                    "age_s": distribution(self.tf_ages[name]),
                    "lag_s": distribution([
                        age for age in self.tf_ages[name] if age >= 0.0
                    ]),
                    "lead_s": distribution([
                        -age for age in self.tf_ages[name] if age < 0.0
                    ]),
                    "future_stamps": self.tf_future[name],
                    "lookup_failures": self.tf_failures[name],
                    "static": self.tf_static[name],
                }
                for name, (target, source) in TF_PAIRS.items()
            },
            "camera": {
                **camera_stamp_report(self.image_stamps, self.info_stamps),
                "latest_message_contracts": self.camera_message_contracts,
            },
            "nav2_log_counts": {
                name: self.log_counts[name] for name in LOG_PATTERNS
            },
            "navigation": {
                "controller_actual_hz": topics["cmd_vel_nav"]["wall_hz"],
                "goal_results": dict(sorted(self.goal_results.items())),
                "goal_duration_s": distribution(self.goal_durations),
                "active_goal_count": len(self.goal_started),
            },
        }

    def close(self) -> None:
        self.tf_timer.cancel()
        unregister = getattr(self.tf_listener, "unregister", None)
        if callable(unregister):
            unregister()


def _arguments(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--warmup", type=float, default=2.0)
    parser.add_argument("--sample-period", type=float, default=0.05)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", default="runtime")
    arguments = parser.parse_args(argv)
    for name in ("duration", "sample_period"):
        value = getattr(arguments, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if arguments.duration > MAX_DURATION_S:
        parser.error(f"--duration must not exceed {MAX_DURATION_S:g} seconds")
    if not math.isfinite(arguments.warmup) or arguments.warmup < 0.0:
        parser.error("--warmup must be non-negative")
    return arguments


def _spin_for(node: Any, duration_s: float) -> None:
    import rclpy

    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(
            node,
            timeout_sec=min(0.10, max(0.0, deadline - time.monotonic())),
        )


def _runtime_ros_snapshot(node: Any, timeout_s: float = 1.5) -> dict[str, Any]:
    import rclpy
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.parameter import parameter_value_to_python

    lifecycle_nodes = (
        "map_server",
        "slam_toolbox",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
    )
    lifecycle = {}
    clients = []
    pending: list[tuple[str, Any]] = []
    for name in lifecycle_nodes:
        client = node.create_client(GetState, f"/{name}/get_state")
        clients.append(client)
        if client.wait_for_service(timeout_sec=0.02):
            pending.append((name, client.call_async(GetState.Request())))
        else:
            lifecycle[name] = "unavailable"

    parameter_requests = {
        "controller_server": (
            "controller_frequency",
            "FollowPath.batch_size",
            "FollowPath.time_steps",
            "FollowPath.model_dt",
        ),
        "slam_toolbox": ("solver_plugin", "ceres_num_threads"),
        "collision_monitor": ("scan.source_timeout",),
        "velocity_smoother": ("smoothing_frequency",),
    }
    list_pending: list[tuple[str, Any]] = []
    for name in parameter_requests:
        client = node.create_client(
            ListParameters, f"/{name}/list_parameters"
        )
        clients.append(client)
        if client.wait_for_service(timeout_sec=0.02):
            request = ListParameters.Request()
            request.depth = 0
            list_pending.append((name, client.call_async(request)))

    deadline = time.monotonic() + timeout_s
    discovery_futures = [future for _, future in pending]
    discovery_futures.extend(future for _, future in list_pending)
    while (
        rclpy.ok()
        and time.monotonic() < deadline
        and any(not future.done() for future in discovery_futures)
    ):
        rclpy.spin_once(node, timeout_sec=0.05)

    available_parameters: dict[str, set[str]] = {}
    for name, future in list_pending:
        if future.done() and future.exception() is None:
            available_parameters[name] = set(future.result().result.names)

    parameters: dict[str, dict[str, Any]] = {}
    parameter_pending: list[tuple[str, str, Any]] = []
    for name, requested_parameters in parameter_requests.items():
        parameters[name] = {}
        available = available_parameters.get(name)
        if available is None:
            for parameter in requested_parameters:
                parameters[name][parameter] = {"error": "service_unavailable"}
            continue
        client = node.create_client(
            GetParameters, f"/{name}/get_parameters"
        )
        clients.append(client)
        if not client.wait_for_service(timeout_sec=0.02):
            for parameter in requested_parameters:
                parameters[name][parameter] = {"error": "service_unavailable"}
            continue
        for parameter in requested_parameters:
            if parameter not in available:
                parameters[name][parameter] = {"available": False}
                continue
            request = GetParameters.Request()
            request.names = [parameter]
            parameter_pending.append(
                (name, parameter, client.call_async(request))
            )

    deadline = time.monotonic() + timeout_s
    all_futures = [future for _, _, future in parameter_pending]
    while (
        rclpy.ok()
        and time.monotonic() < deadline
        and any(not future.done() for future in all_futures)
    ):
        rclpy.spin_once(node, timeout_sec=0.05)

    for name, future in pending:
        if not future.done() or future.exception() is not None:
            lifecycle[name] = "timeout"
            continue
        lifecycle[name] = future.result().current_state.label

    for name, parameter, future in parameter_pending:
        if not future.done() or future.exception() is not None:
            parameters[name][parameter] = {"error": "timeout"}
            continue
        values = list(future.result().values)
        parameters[name][parameter] = (
            parameter_value_to_python(values[0]) if values else None
        )
    for client in clients:
        node.destroy_client(client)
    return {
        "lifecycle": lifecycle,
        "parameters": parameters,
    }


def main(argv: list[str] | None = None) -> int:
    arguments = _arguments(argv)
    import rclpy
    from rclpy.node import Node

    rclpy.init(args=[])
    node = Node("runtime_profiler")
    profiler = RuntimeProfiler(node, sample_period_s=arguments.sample_period)
    project_root = Path(
        os.environ.get("PROJECT_ROOT", Path(__file__).resolve().parents[4])
    ).resolve()
    runtime_dir = Path(os.environ.get(
        "ISAAC_NAV_RUNTIME_DIR",
        f"/tmp/isaac_sim_ros2_nav_{os.getuid()}",
    ))
    started = time.monotonic()
    try:
        _spin_for(node, arguments.warmup)
        ros_snapshot = _runtime_ros_snapshot(node)
        profiler.reset_measurements()
        cpu_before = _read_cpu_times_by_core()
        scaling_before = _cpu_scaling_snapshot()
        throttle_before = _throttle_snapshot()
        gpu_before = _gpu_snapshot()
        process_before = _registered_process_snapshot(runtime_dir)
        runtime_metadata = _runtime_metadata(
            project_root, runtime_dir, scaling_before
        )
        started = time.monotonic()
        _spin_for(node, arguments.duration)
        finished = time.monotonic()
        cpu_after = _read_cpu_times_by_core()
        cpu_usage = _cpu_utilization(cpu_before, cpu_after)
        scaling_after = _cpu_scaling_snapshot()
        throttle_after = _throttle_snapshot()
        gpu_after = _gpu_snapshot()
        elapsed = finished - started
        runtime_report = profiler.report()
        runtime_report["navigation"].update({
            "lifecycle": ros_snapshot["lifecycle"],
            "parameters": ros_snapshot["parameters"],
            "controller_target_hz": ros_snapshot["parameters"].get(
                "controller_server", {}
            ).get("controller_frequency"),
        })
        report = {
            "schema_version": 1,
            "metadata": {
                "created_at": datetime.now(timezone.utc).isoformat(),
                "label": arguments.label,
                "requested_duration_s": arguments.duration,
                "measured_duration_s": elapsed,
                "warmup_s": arguments.warmup,
                "hostname": platform.node(),
                "platform": platform.platform(),
                "python": platform.python_version(),
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION"),
                "git": _git_metadata(project_root),
                **runtime_metadata,
            },
            **runtime_report,
            "system": {
                "cpu_model": _cpu_model(),
                "logical_cpu_count": os.cpu_count(),
                "cpu_percent": cpu_usage.get("cpu"),
                "per_core_cpu_percent": {
                    name: value
                    for name, value in cpu_usage.items()
                    if name != "cpu"
                },
                "cpu_frequency": _cpu_frequency_summary(),
                "cpu_scaling_start": scaling_before,
                "cpu_scaling_end": scaling_after,
                "load_average": list(os.getloadavg()),
                "memory": _read_memory(),
                "temperature": _temperature_snapshot(),
                "throttling": {
                    "start": throttle_before,
                    "end": throttle_after,
                    "counter_delta": (
                        throttle_after["total"] - throttle_before["total"]
                    ),
                },
                "registered_processes": _process_delta(
                    process_before,
                    _registered_process_snapshot(runtime_dir),
                    elapsed,
                ),
            },
            "gpu": {
                **gpu_after,
                "sampling": "start_and_end_snapshots",
                "measurement_start": gpu_before,
                "measurement_end": gpu_after,
                "measurement_delta": _gpu_delta(gpu_before, gpu_after),
            },
        }
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        temporary_output = arguments.output.with_name(
            f".{arguments.output.name}.{os.getpid()}.tmp"
        )
        try:
            temporary_output.write_text(
                json.dumps(report, indent=2, sort_keys=True, allow_nan=False)
                + "\n",
                encoding="utf-8",
            )
            os.replace(temporary_output, arguments.output)
        finally:
            temporary_output.unlink(missing_ok=True)
        print(str(arguments.output.resolve()))
        return 0
    finally:
        profiler.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
