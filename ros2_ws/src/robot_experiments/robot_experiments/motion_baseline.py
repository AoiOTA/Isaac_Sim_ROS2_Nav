"""Pure-Python configuration and metrics for chassis motion baselines.

The ROS adapter deliberately lives in :mod:`motion_baseline_runner`.  Keeping
all configuration and measurement logic here makes the acceptance contract
testable without ROS 2 or Isaac Sim.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from itertools import groupby
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from .configuration import (
    ConfigurationError,
    load_yaml_mapping,
    require_finite,
    require_mapping,
    require_string,
)
from .metrics import path_length, wrap_angle


REQUIRED_MOTION_KINDS = (
    "forward",
    "backward",
    "rotate_left",
    "rotate_right",
)
MOTION_KINDS = REQUIRED_MOTION_KINDS + ("arc_left", "arc_right")
NANOSECONDS_PER_SECOND = 1_000_000_000


def _reject_unknown(mapping: Mapping[str, Any], allowed: set[str], location: str) -> None:
    unknown = sorted(set(mapping) - allowed)
    if unknown:
        raise ConfigurationError(
            f"{location} contains unknown keys: {', '.join(unknown)}"
        )


def _positive(value: Any, location: str, *, allow_zero: bool = False) -> float:
    parsed = require_finite(value, location)
    if parsed < 0.0 or (parsed == 0.0 and not allow_zero):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ConfigurationError(f"{location} must be {qualifier}")
    return parsed


def _positive_integer(value: Any, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ConfigurationError(f"{location} must be a positive integer")
    return value


def _absolute_name(value: Any, location: str) -> str:
    parsed = require_string(value, location).strip()
    if not parsed.startswith("/") or parsed == "/" or "//" in parsed:
        raise ConfigurationError(f"{location} must be an absolute ROS name")
    return parsed


@dataclass(frozen=True)
class TopicSettings:
    cmd_vel: str
    clock: str
    odom: str
    joint_states: str

    def as_dict(self) -> dict[str, str]:
        return asdict(self)


@dataclass(frozen=True)
class ResetSettings:
    service: str
    service_timeout_sec: float
    recovery_timeout_sec: float
    settle_duration_sec: float

    def as_dict(self) -> dict[str, str | float]:
        return asdict(self)


@dataclass(frozen=True)
class SamplingSettings:
    publish_rate_hz: float
    command_wall_timeout_sec: float
    max_sample_age_sec: float
    max_future_skew_sec: float
    zero_publish_count: int
    zero_publish_interval_sec: float

    def as_dict(self) -> dict[str, float | int]:
        return asdict(self)


@dataclass(frozen=True)
class CommandLimits:
    max_abs_linear_mps: float
    max_abs_angular_radps: float
    max_segment_duration_sec: float

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class StopSettings:
    linear_velocity_threshold_mps: float
    angular_velocity_threshold_radps: float
    wheel_velocity_threshold_radps: float
    stable_duration_sec: float
    timeout_sec: float

    def as_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class WheelLayout:
    front_left: str
    front_right: str
    rear_left: str
    rear_right: str

    @property
    def ordered_names(self) -> tuple[str, str, str, str]:
        return (
            self.front_left,
            self.front_right,
            self.rear_left,
            self.rear_right,
        )

    @property
    def left_names(self) -> tuple[str, str]:
        return self.front_left, self.rear_left

    @property
    def right_names(self) -> tuple[str, str]:
        return self.front_right, self.rear_right

    def as_dict(self) -> dict[str, str]:
        return asdict(self)


@dataclass(frozen=True)
class MotionSegment:
    segment_id: str
    motion: str
    tier: str
    linear_x_mps: float
    angular_z_radps: float
    duration_sec: float

    def __post_init__(self) -> None:
        if self.motion not in MOTION_KINDS:
            raise ConfigurationError(
                f"segment {self.segment_id!r} has unsupported motion {self.motion!r}"
            )
        linear = require_finite(self.linear_x_mps, f"{self.segment_id}.linear_x_mps")
        angular = require_finite(
            self.angular_z_radps, f"{self.segment_id}.angular_z_radps"
        )
        _positive(self.duration_sec, f"{self.segment_id}.duration_sec")
        expected = {
            "forward": linear > 0.0 and angular == 0.0,
            "backward": linear < 0.0 and angular == 0.0,
            "rotate_left": linear == 0.0 and angular > 0.0,
            "rotate_right": linear == 0.0 and angular < 0.0,
            "arc_left": linear > 0.0 and angular > 0.0,
            "arc_right": linear > 0.0 and angular < 0.0,
        }
        if not expected[self.motion]:
            raise ConfigurationError(
                f"segment {self.segment_id!r} command signs do not match {self.motion}"
            )

    def as_dict(self) -> dict[str, str | float]:
        return asdict(self)


@dataclass(frozen=True)
class MotionBaselineConfig:
    schema_version: int
    profile_id: str
    topics: TopicSettings
    reset: ResetSettings
    sampling: SamplingSettings
    limits: CommandLimits
    stop: StopSettings
    wheels: WheelLayout
    segments: tuple[MotionSegment, ...]

    def as_dict(self) -> dict[str, object]:
        return {
            "schema_version": self.schema_version,
            "profile_id": self.profile_id,
            "topics": self.topics.as_dict(),
            "reset": self.reset.as_dict(),
            "sampling": self.sampling.as_dict(),
            "limits": self.limits.as_dict(),
            "stop": self.stop.as_dict(),
            "wheels": self.wheels.as_dict(),
            "segments": [segment.as_dict() for segment in self.segments],
        }


def _parse_topics(value: Any) -> TopicSettings:
    mapping = require_mapping(value, "topics")
    required = {"cmd_vel", "clock", "odom", "joint_states"}
    _reject_unknown(mapping, required, "topics")
    missing = sorted(required - set(mapping))
    if missing:
        raise ConfigurationError(f"topics is missing keys: {', '.join(missing)}")
    settings = TopicSettings(
        cmd_vel=_absolute_name(mapping["cmd_vel"], "topics.cmd_vel"),
        clock=_absolute_name(mapping["clock"], "topics.clock"),
        odom=_absolute_name(mapping["odom"], "topics.odom"),
        joint_states=_absolute_name(mapping["joint_states"], "topics.joint_states"),
    )
    if len(set(settings.as_dict().values())) != 4:
        raise ConfigurationError("motion baseline topic names must be unique")
    return settings


def _parse_reset(value: Any) -> ResetSettings:
    mapping = require_mapping(value, "reset")
    required = {
        "service",
        "service_timeout_sec",
        "recovery_timeout_sec",
        "settle_duration_sec",
    }
    _reject_unknown(mapping, required, "reset")
    missing = sorted(required - set(mapping))
    if missing:
        raise ConfigurationError(f"reset is missing keys: {', '.join(missing)}")
    return ResetSettings(
        service=_absolute_name(mapping["service"], "reset.service"),
        service_timeout_sec=_positive(
            mapping["service_timeout_sec"], "reset.service_timeout_sec"
        ),
        recovery_timeout_sec=_positive(
            mapping["recovery_timeout_sec"], "reset.recovery_timeout_sec"
        ),
        settle_duration_sec=_positive(
            mapping["settle_duration_sec"],
            "reset.settle_duration_sec",
            allow_zero=True,
        ),
    )


def _parse_sampling(value: Any) -> SamplingSettings:
    mapping = require_mapping(value, "sampling")
    required = {
        "publish_rate_hz",
        "command_wall_timeout_sec",
        "max_sample_age_sec",
        "max_future_skew_sec",
        "zero_publish_count",
        "zero_publish_interval_sec",
    }
    _reject_unknown(mapping, required, "sampling")
    missing = sorted(required - set(mapping))
    if missing:
        raise ConfigurationError(f"sampling is missing keys: {', '.join(missing)}")
    settings = SamplingSettings(
        publish_rate_hz=_positive(mapping["publish_rate_hz"], "sampling.publish_rate_hz"),
        command_wall_timeout_sec=_positive(
            mapping["command_wall_timeout_sec"], "sampling.command_wall_timeout_sec"
        ),
        max_sample_age_sec=_positive(
            mapping["max_sample_age_sec"], "sampling.max_sample_age_sec"
        ),
        max_future_skew_sec=_positive(
            mapping["max_future_skew_sec"],
            "sampling.max_future_skew_sec",
            allow_zero=True,
        ),
        zero_publish_count=_positive_integer(
            mapping["zero_publish_count"], "sampling.zero_publish_count"
        ),
        zero_publish_interval_sec=_positive(
            mapping["zero_publish_interval_sec"],
            "sampling.zero_publish_interval_sec",
            allow_zero=True,
        ),
    )
    if settings.max_future_skew_sec > min(settings.max_sample_age_sec, 0.05):
        raise ConfigurationError(
            "sampling.max_future_skew_sec must not exceed 0.05 seconds or "
            "sampling.max_sample_age_sec"
        )
    return settings


def _parse_stop(value: Any) -> StopSettings:
    mapping = require_mapping(value, "stop")
    required = {
        "linear_velocity_threshold_mps",
        "angular_velocity_threshold_radps",
        "wheel_velocity_threshold_radps",
        "stable_duration_sec",
        "timeout_sec",
    }
    _reject_unknown(mapping, required, "stop")
    missing = sorted(required - set(mapping))
    if missing:
        raise ConfigurationError(f"stop is missing keys: {', '.join(missing)}")
    return StopSettings(
        linear_velocity_threshold_mps=_positive(
            mapping["linear_velocity_threshold_mps"],
            "stop.linear_velocity_threshold_mps",
            allow_zero=True,
        ),
        angular_velocity_threshold_radps=_positive(
            mapping["angular_velocity_threshold_radps"],
            "stop.angular_velocity_threshold_radps",
            allow_zero=True,
        ),
        wheel_velocity_threshold_radps=_positive(
            mapping["wheel_velocity_threshold_radps"],
            "stop.wheel_velocity_threshold_radps",
            allow_zero=True,
        ),
        stable_duration_sec=_positive(
            mapping["stable_duration_sec"], "stop.stable_duration_sec"
        ),
        timeout_sec=_positive(mapping["timeout_sec"], "stop.timeout_sec"),
    )


def _parse_limits(value: Any) -> CommandLimits:
    mapping = require_mapping(value, "limits")
    required = {
        "max_abs_linear_mps",
        "max_abs_angular_radps",
        "max_segment_duration_sec",
    }
    _reject_unknown(mapping, required, "limits")
    missing = sorted(required - set(mapping))
    if missing:
        raise ConfigurationError(f"limits is missing keys: {', '.join(missing)}")
    return CommandLimits(
        max_abs_linear_mps=_positive(
            mapping["max_abs_linear_mps"], "limits.max_abs_linear_mps"
        ),
        max_abs_angular_radps=_positive(
            mapping["max_abs_angular_radps"], "limits.max_abs_angular_radps"
        ),
        max_segment_duration_sec=_positive(
            mapping["max_segment_duration_sec"],
            "limits.max_segment_duration_sec",
        ),
    )


def _parse_wheels(value: Any) -> WheelLayout:
    mapping = require_mapping(value, "wheels")
    required = {"front_left", "front_right", "rear_left", "rear_right"}
    _reject_unknown(mapping, required, "wheels")
    missing = sorted(required - set(mapping))
    if missing:
        raise ConfigurationError(f"wheels is missing keys: {', '.join(missing)}")
    layout = WheelLayout(
        **{
            key: require_string(mapping[key], f"wheels.{key}").strip()
            for key in sorted(required)
        }
    )
    if len(set(layout.ordered_names)) != 4:
        raise ConfigurationError("exactly four unique wheel joint names are required")
    return layout


def _parse_segments(value: Any) -> tuple[MotionSegment, ...]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence) or not value:
        raise ConfigurationError("segments must be a non-empty sequence")
    segments: list[MotionSegment] = []
    allowed = {
        "id",
        "motion",
        "tier",
        "linear_x_mps",
        "angular_z_radps",
        "duration_sec",
    }
    for index, raw in enumerate(value):
        location = f"segments[{index}]"
        mapping = require_mapping(raw, location)
        _reject_unknown(mapping, allowed, location)
        missing = sorted(allowed - set(mapping))
        if missing:
            raise ConfigurationError(f"{location} is missing keys: {', '.join(missing)}")
        segment_id = require_string(mapping["id"], f"{location}.id").strip()
        if Path(segment_id).name != segment_id or segment_id in {".", ".."}:
            raise ConfigurationError(f"{location}.id must be one path-safe name")
        segments.append(
            MotionSegment(
                segment_id=segment_id,
                motion=require_string(mapping["motion"], f"{location}.motion").strip(),
                tier=require_string(mapping["tier"], f"{location}.tier").strip(),
                linear_x_mps=require_finite(
                    mapping["linear_x_mps"], f"{location}.linear_x_mps"
                ),
                angular_z_radps=require_finite(
                    mapping["angular_z_radps"], f"{location}.angular_z_radps"
                ),
                duration_sec=_positive(mapping["duration_sec"], f"{location}.duration_sec"),
            )
        )
    identifiers = [segment.segment_id for segment in segments]
    if len(set(identifiers)) != len(identifiers):
        raise ConfigurationError("segment ids must be unique")
    missing_motion = sorted(
        set(REQUIRED_MOTION_KINDS) - {segment.motion for segment in segments}
    )
    if missing_motion:
        raise ConfigurationError(
            "segments must cover forward, backward, rotate_left, and rotate_right; "
            f"missing: {', '.join(missing_motion)}"
        )
    return tuple(segments)


def load_motion_baseline_config(path: str | Path) -> MotionBaselineConfig:
    """Load a strict, complete chassis motion-baseline profile."""
    document = load_yaml_mapping(path)
    allowed = {
        "schema_version",
        "profile_id",
        "topics",
        "reset",
        "sampling",
        "limits",
        "stop",
        "wheels",
        "segments",
    }
    _reject_unknown(document, allowed, "motion baseline configuration")
    missing = sorted(allowed - set(document))
    if missing:
        raise ConfigurationError(
            f"motion baseline configuration is missing keys: {', '.join(missing)}"
        )
    if document["schema_version"] != 1:
        raise ConfigurationError("schema_version must be exactly 1")
    config = MotionBaselineConfig(
        schema_version=1,
        profile_id=require_string(document["profile_id"], "profile_id").strip(),
        topics=_parse_topics(document["topics"]),
        reset=_parse_reset(document["reset"]),
        sampling=_parse_sampling(document["sampling"]),
        limits=_parse_limits(document["limits"]),
        stop=_parse_stop(document["stop"]),
        wheels=_parse_wheels(document["wheels"]),
        segments=_parse_segments(document["segments"]),
    )
    longest_command = max(segment.duration_sec for segment in config.segments)
    if config.sampling.command_wall_timeout_sec <= longest_command:
        raise ConfigurationError(
            "sampling.command_wall_timeout_sec must exceed every simulated command duration"
        )
    if not 10.0 <= config.sampling.publish_rate_hz <= 100.0:
        raise ConfigurationError("sampling.publish_rate_hz must be within [10, 100]")
    if not 3 <= config.sampling.zero_publish_count <= 100:
        raise ConfigurationError("sampling.zero_publish_count must be within [3, 100]")
    if config.sampling.zero_publish_interval_sec > 0.2:
        raise ConfigurationError(
            "sampling.zero_publish_interval_sec must not exceed 0.2"
        )
    for segment in config.segments:
        if abs(segment.linear_x_mps) > config.limits.max_abs_linear_mps:
            raise ConfigurationError(
                f"segment {segment.segment_id!r} linear speed exceeds limits"
            )
        if abs(segment.angular_z_radps) > config.limits.max_abs_angular_radps:
            raise ConfigurationError(
                f"segment {segment.segment_id!r} angular speed exceeds limits"
            )
        if segment.duration_sec > config.limits.max_segment_duration_sec:
            raise ConfigurationError(
                f"segment {segment.segment_id!r} duration exceeds limits"
            )
    if config.stop.timeout_sec <= config.stop.stable_duration_sec:
        raise ConfigurationError("stop.timeout_sec must exceed stop.stable_duration_sec")
    return config


def _finite_number(value: float, location: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{location} must be finite")
    return parsed


def _stamp_ns(value: int, location: str = "stamp_ns") -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{location} must be a non-negative integer")
    return value


@dataclass(frozen=True)
class OdomSample:
    stamp_ns: int
    x_m: float
    y_m: float
    yaw_rad: float
    linear_x_mps: float
    linear_y_mps: float
    angular_z_radps: float

    def __post_init__(self) -> None:
        _stamp_ns(self.stamp_ns)
        for field, value in asdict(self).items():
            if field != "stamp_ns":
                _finite_number(value, field)

    def pose_dict(self) -> dict[str, float]:
        return {"x_m": self.x_m, "y_m": self.y_m, "yaw_rad": self.yaw_rad}


@dataclass(frozen=True)
class JointSample:
    stamp_ns: int
    velocities_radps: tuple[tuple[str, float], ...]

    def __post_init__(self) -> None:
        _stamp_ns(self.stamp_ns)
        names = [name for name, _ in self.velocities_radps]
        if len(set(names)) != len(names):
            raise ValueError("joint sample names must be unique")
        for name, velocity in self.velocities_radps:
            if not isinstance(name, str) or not name:
                raise ValueError("joint names must be non-empty strings")
            _finite_number(velocity, f"velocity[{name}]")

    @classmethod
    def from_mapping(cls, stamp_ns: int, velocities: Mapping[str, float]) -> "JointSample":
        return cls(
            stamp_ns=stamp_ns,
            velocities_radps=tuple(
                (str(name), float(value)) for name, value in velocities.items()
            ),
        )

    def velocity_map(self) -> dict[str, float]:
        return dict(self.velocities_radps)


class TimestampTracker:
    """Count timestamp duplicates and regressions in message receipt order."""

    def __init__(self) -> None:
        self.sample_count = 0
        self.regression_count = 0
        self.duplicate_count = 0
        self.first_stamp_ns: int | None = None
        self.last_stamp_ns: int | None = None

    def observe(self, stamp_ns: int) -> None:
        stamp = _stamp_ns(stamp_ns)
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp
        if self.last_stamp_ns is not None:
            if stamp < self.last_stamp_ns:
                self.regression_count += 1
            elif stamp == self.last_stamp_ns:
                self.duplicate_count += 1
        self.last_stamp_ns = stamp
        self.sample_count += 1

    def as_dict(self) -> dict[str, int | None | bool]:
        return {
            "sample_count": self.sample_count,
            "first_stamp_ns": self.first_stamp_ns,
            "last_stamp_ns": self.last_stamp_ns,
            "regression_count": self.regression_count,
            "duplicate_count": self.duplicate_count,
            "monotonic_unique": (
                self.regression_count == 0 and self.duplicate_count == 0
            ),
        }


def _distribution(values: Iterable[float]) -> dict[str, int | float]:
    parsed = [_finite_number(value, "sample") for value in values]
    if not parsed:
        raise ValueError("at least one finite sample is required")
    mean = sum(parsed) / len(parsed)
    return {
        "sample_count": len(parsed),
        "mean": mean,
        "mean_abs": sum(abs(value) for value in parsed) / len(parsed),
        "minimum": min(parsed),
        "maximum": max(parsed),
        "peak_abs": max(abs(value) for value in parsed),
        "rmse": math.sqrt(sum(value * value for value in parsed) / len(parsed)),
    }


def _unwrapped_yaw_change(samples: Sequence[OdomSample]) -> float:
    return sum(
        wrap_angle(current.yaw_rad - previous.yaw_rad)
        for previous, current in zip(samples, samples[1:])
    )


def _classify_direction(values: Sequence[float], deadband: float) -> str:
    moving = [value for value in values if abs(value) > deadband]
    if not moving:
        return "stationary"
    if all(value > 0.0 for value in moving):
        return "positive"
    if all(value < 0.0 for value in moving):
        return "negative"
    return "mixed"


def expected_wheel_directions(
    motion: str, wheels: WheelLayout
) -> dict[str, str]:
    if motion not in MOTION_KINDS:
        raise ValueError(f"unsupported motion: {motion}")
    if motion == "forward":
        return {name: "positive" for name in wheels.ordered_names}
    if motion == "backward":
        return {name: "negative" for name in wheels.ordered_names}
    if motion in {"arc_left", "arc_right"}:
        # The A/B arc contract uses v=0.4 m/s and |w|=0.4 rad/s, for which
        # both sides roll forward while the inner side runs more slowly.
        return {name: "positive" for name in wheels.ordered_names}
    left, right = (
        ("negative", "positive")
        if motion == "rotate_left"
        else ("positive", "negative")
    )
    return {
        **{name: left for name in wheels.left_names},
        **{name: right for name in wheels.right_names},
    }


@dataclass(frozen=True)
class StopDetection:
    stopped: bool
    stationary_onset_after_command_sec: float | None
    confirmed_after_command_sec: float | None
    stationary_evidence: Mapping[str, object] | None

    def as_dict(self) -> dict[str, object]:
        return asdict(self)


def detect_stopping(
    command_end_ns: int,
    odom_samples: Sequence[OdomSample],
    joint_samples: Sequence[JointSample],
    wheels: WheelLayout,
    settings: StopSettings,
    max_sample_age_sec: float,
) -> StopDetection:
    """Find a dual-stream, continuously evidenced stationary interval."""
    end_ns = _stamp_ns(command_end_ns, "command_end_ns")
    if isinstance(max_sample_age_sec, bool) or not isinstance(
        max_sample_age_sec, (int, float)
    ):
        raise ValueError("max_sample_age_sec must be a finite number")
    sample_age_limit = _finite_number(
        max_sample_age_sec, "max_sample_age_sec"
    )
    if sample_age_limit <= 0.0:
        raise ValueError("max_sample_age_sec must be positive")
    max_sample_age_ns = round(
        sample_age_limit * NANOSECONDS_PER_SECOND
    )
    for source, samples in (
        ("odometry", odom_samples),
        ("joint-state", joint_samples),
    ):
        stamps = [sample.stamp_ns for sample in samples]
        if any(
            current <= previous
            for previous, current in zip(stamps, stamps[1:])
        ):
            raise ValueError(
                f"{source} stop-detection timestamps must be strictly increasing"
            )

    events: list[tuple[int, int, object]] = []
    events.extend(
        (sample.stamp_ns, 0, sample)
        for sample in odom_samples
        if sample.stamp_ns >= end_ns
    )
    events.extend(
        (sample.stamp_ns, 1, sample)
        for sample in joint_samples
        if sample.stamp_ns >= end_ns
    )
    events.sort(key=lambda item: (item[0], item[1]))
    latest_odom: OdomSample | None = None
    latest_joint: JointSample | None = None
    stationary_since_ns: int | None = None
    supporting_stamps: dict[str, list[int]] = {
        "odom": [],
        "joint_states": [],
    }
    stable_ns = round(settings.stable_duration_sec * NANOSECONDS_PER_SECOND)
    required_wheels = set(wheels.ordered_names)

    def clear_candidate() -> None:
        nonlocal stationary_since_ns, supporting_stamps
        stationary_since_ns = None
        supporting_stamps = {"odom": [], "joint_states": []}

    def start_candidate(stamp_ns: int) -> None:
        nonlocal stationary_since_ns, supporting_stamps
        assert latest_odom is not None
        assert latest_joint is not None
        stationary_since_ns = stamp_ns
        supporting_stamps = {
            "odom": [latest_odom.stamp_ns],
            "joint_states": [latest_joint.stamp_ns],
        }

    for stamp, simultaneous in groupby(events, key=lambda item: item[0]):
        # Apply every message at one timestamp before evaluating stillness.  An
        # odometry message must not confirm a stop immediately before a wheel
        # message at the same simulation instant reports continued motion.
        updated_sources: set[str] = set()
        for _, event_type, sample in simultaneous:
            if event_type == 0:
                assert isinstance(sample, OdomSample)
                latest_odom = sample
                updated_sources.add("odom")
            else:
                assert isinstance(sample, JointSample)
                latest_joint = sample
                updated_sources.add("joint_states")
        stationary = False
        if latest_odom is not None and latest_joint is not None:
            velocities = latest_joint.velocity_map()
            streams_fresh = (
                stamp - latest_odom.stamp_ns <= max_sample_age_ns
                and stamp - latest_joint.stamp_ns <= max_sample_age_ns
            )
            stationary = streams_fresh and required_wheels <= set(velocities) and (
                math.hypot(
                    latest_odom.linear_x_mps, latest_odom.linear_y_mps
                )
                <= settings.linear_velocity_threshold_mps
                and abs(latest_odom.angular_z_radps)
                <= settings.angular_velocity_threshold_radps
                and all(
                    abs(velocities[name]) <= settings.wheel_velocity_threshold_radps
                    for name in required_wheels
                )
            )
        if not stationary:
            clear_candidate()
            continue
        if stationary_since_ns is None:
            start_candidate(stamp)
        else:
            latest_stamps = {
                "odom": latest_odom.stamp_ns,
                "joint_states": latest_joint.stamp_ns,
            }
            continuity_broken = any(
                source in updated_sources
                and latest_stamps[source] - supporting_stamps[source][-1]
                > max_sample_age_ns
                for source in supporting_stamps
            )
            if continuity_broken:
                start_candidate(stamp)
            else:
                for source in updated_sources:
                    current = latest_stamps[source]
                    if current > supporting_stamps[source][-1]:
                        supporting_stamps[source].append(current)

        assert stationary_since_ns is not None
        supported_through_ns = min(
            latest_odom.stamp_ns, latest_joint.stamp_ns
        )
        if supported_through_ns - stationary_since_ns < stable_ns:
            continue
        window_stamps = {
            source: [
                sample_stamp
                for sample_stamp in stamps
                if stationary_since_ns
                <= sample_stamp
                <= supported_through_ns
            ]
            for source, stamps in supporting_stamps.items()
        }
        if any(len(stamps) < 2 for stamps in window_stamps.values()):
            continue
        if all(
            max(
                current - previous
                for previous, current in zip(stamps, stamps[1:])
            )
            <= max_sample_age_ns
            for stamps in window_stamps.values()
        ):

            def stream_evidence(stamps: Sequence[int]) -> dict[str, object]:
                maximum_gap_ns = max(
                    current - previous
                    for previous, current in zip(stamps, stamps[1:])
                )
                return {
                    "sample_count": len(stamps),
                    "first_sample_stamp_ns": stamps[0],
                    "last_sample_stamp_ns": stamps[-1],
                    "maximum_inter_sample_gap_sec": (
                        maximum_gap_ns / NANOSECONDS_PER_SECOND
                    ),
                }

            return StopDetection(
                stopped=True,
                stationary_onset_after_command_sec=(
                    stationary_since_ns - end_ns
                )
                / NANOSECONDS_PER_SECOND,
                confirmed_after_command_sec=(supported_through_ns - end_ns)
                / NANOSECONDS_PER_SECOND,
                stationary_evidence={
                    "schema_version": 1,
                    "definition": (
                        "dual_stream_continuously_stationary_after_zero_command"
                    ),
                    "boundary_semantics": "closed_interval",
                    "start_stamp_ns": stationary_since_ns,
                    "end_stamp_ns": supported_through_ns,
                    "observed_duration_sec": (
                        supported_through_ns - stationary_since_ns
                    )
                    / NANOSECONDS_PER_SECOND,
                    "max_sample_age_sec": sample_age_limit,
                    "streams": {
                        source: stream_evidence(stamps)
                        for source, stamps in window_stamps.items()
                    },
                },
            )
    return StopDetection(False, None, None, None)


def analyse_motion_segment(
    segment: MotionSegment,
    command_start_ns: int,
    command_end_ns: int,
    odom_samples: Sequence[OdomSample],
    joint_samples: Sequence[JointSample],
    wheels: WheelLayout,
    stop_settings: StopSettings,
    *,
    command_publish_count: int,
    max_sample_age_sec: float,
    timestamp_integrity: Mapping[str, Mapping[str, object]],
) -> dict[str, object]:
    """Build one strict-JSON-compatible motion segment result."""
    start_ns = _stamp_ns(command_start_ns, "command_start_ns")
    end_ns = _stamp_ns(command_end_ns, "command_end_ns")
    if end_ns <= start_ns:
        raise ValueError("command_end_ns must be later than command_start_ns")
    if isinstance(command_publish_count, bool) or command_publish_count <= 0:
        raise ValueError("command_publish_count must be positive")
    if isinstance(max_sample_age_sec, bool) or not isinstance(
        max_sample_age_sec, (int, float)
    ):
        raise ValueError("max_sample_age_sec must be a finite number")
    sample_age_limit = _finite_number(
        max_sample_age_sec, "max_sample_age_sec"
    )
    if sample_age_limit <= 0.0:
        raise ValueError("max_sample_age_sec must be positive")

    command_odom = [
        sample for sample in odom_samples if start_ns <= sample.stamp_ns <= end_ns
    ]
    command_joints = [
        sample for sample in joint_samples if start_ns <= sample.stamp_ns <= end_ns
    ]
    if not command_odom:
        raise ValueError("no odometry samples overlap the command interval")
    if not command_joints:
        raise ValueError("no joint-state samples overlap the command interval")
    steady_state_start_ns = start_ns + (end_ns - start_ns) // 2
    steady_state_odom = [
        sample
        for sample in command_odom
        if steady_state_start_ns <= sample.stamp_ns <= end_ns
    ]
    steady_state_joints = [
        sample
        for sample in command_joints
        if steady_state_start_ns <= sample.stamp_ns <= end_ns
    ]
    if not steady_state_odom:
        raise ValueError("no odometry samples overlap the steady-state window")
    if len(steady_state_odom) < 2:
        raise ValueError(
            "odometry steady-state window requires at least two samples"
        )
    if not steady_state_joints:
        raise ValueError("no joint-state samples overlap the steady-state window")
    if len(steady_state_joints) < 2:
        raise ValueError(
            "joint-state steady-state window requires at least two samples"
        )
    steady_odom_stamps = [sample.stamp_ns for sample in steady_state_odom]
    if any(
        current <= previous
        for previous, current in zip(
            steady_odom_stamps, steady_odom_stamps[1:]
        )
    ):
        raise ValueError(
            "odometry steady-state window timestamps must be strictly increasing"
        )
    first_odom_lag_sec = (
        steady_odom_stamps[0] - steady_state_start_ns
    ) / NANOSECONDS_PER_SECOND
    last_odom_lag_sec = (
        end_ns - steady_odom_stamps[-1]
    ) / NANOSECONDS_PER_SECOND
    maximum_odom_gap_sec = max(
        current - previous
        for previous, current in zip(
            steady_odom_stamps, steady_odom_stamps[1:]
        )
    ) / NANOSECONDS_PER_SECOND
    if first_odom_lag_sec > sample_age_limit:
        raise ValueError(
            "first odometry steady-state sample exceeds max_sample_age_sec"
        )
    if last_odom_lag_sec > sample_age_limit:
        raise ValueError(
            "last odometry steady-state sample exceeds max_sample_age_sec"
        )
    if maximum_odom_gap_sec > sample_age_limit:
        raise ValueError(
            "odometry steady-state sample gap exceeds max_sample_age_sec"
        )

    steady_joint_stamps = [sample.stamp_ns for sample in steady_state_joints]
    if any(
        current <= previous
        for previous, current in zip(
            steady_joint_stamps, steady_joint_stamps[1:]
        )
    ):
        raise ValueError(
            "joint-state steady-state window timestamps must be strictly increasing"
        )
    first_joint_lag_sec = (
        steady_joint_stamps[0] - steady_state_start_ns
    ) / NANOSECONDS_PER_SECOND
    last_joint_lag_sec = (
        end_ns - steady_joint_stamps[-1]
    ) / NANOSECONDS_PER_SECOND
    maximum_joint_gap_sec = max(
        current - previous
        for previous, current in zip(
            steady_joint_stamps, steady_joint_stamps[1:]
        )
    ) / NANOSECONDS_PER_SECOND
    if first_joint_lag_sec > sample_age_limit:
        raise ValueError(
            "first joint-state steady-state sample exceeds max_sample_age_sec"
        )
    if last_joint_lag_sec > sample_age_limit:
        raise ValueError(
            "last joint-state steady-state sample exceeds max_sample_age_sec"
        )
    if maximum_joint_gap_sec > sample_age_limit:
        raise ValueError(
            "joint-state steady-state sample gap exceeds max_sample_age_sec"
        )
    for sample in command_joints:
        missing = sorted(set(wheels.ordered_names) - set(sample.velocity_map()))
        if missing:
            raise ValueError(
                f"joint sample at {sample.stamp_ns} is missing: {', '.join(missing)}"
            )

    start = command_odom[0]
    end = command_odom[-1]
    delta_x = end.x_m - start.x_m
    delta_y = end.y_m - start.y_m
    longitudinal = math.cos(start.yaw_rad) * delta_x + math.sin(start.yaw_rad) * delta_y
    lateral = -math.sin(start.yaw_rad) * delta_x + math.cos(start.yaw_rad) * delta_y
    yaw_change = _unwrapped_yaw_change(command_odom)
    observed_duration = (end_ns - start_ns) / NANOSECONDS_PER_SECOND
    expected_longitudinal = segment.linear_x_mps * observed_duration
    expected_yaw_change = segment.angular_z_radps * observed_duration
    pure_or_straight = segment.motion in REQUIRED_MOTION_KINDS
    pure_rotation = segment.motion in {"rotate_left", "rotate_right"}

    expected_directions = expected_wheel_directions(segment.motion, wheels)

    def wheel_direction_report(
        samples: Sequence[JointSample],
        *,
        include_sample_counts: bool = False,
    ) -> tuple[dict[str, object], bool]:
        report: dict[str, object] = {}
        for name in wheels.ordered_names:
            values = [sample.velocity_map()[name] for sample in samples]
            direction = _classify_direction(
                values, stop_settings.wheel_velocity_threshold_radps
            )
            wheel: dict[str, object] = {
                "direction": direction,
                "expected_direction": expected_directions[name],
                "direction_matches": direction == expected_directions[name],
                "speed_radps": _distribution(values),
            }
            if include_sample_counts:
                deadband = stop_settings.wheel_velocity_threshold_radps
                wheel["direction_sample_counts"] = {
                    "positive_above_deadband": sum(
                        value > deadband for value in values
                    ),
                    "negative_below_deadband": sum(
                        value < -deadband for value in values
                    ),
                    "within_deadband": sum(
                        -deadband <= value <= deadband for value in values
                    ),
                }
            report[name] = wheel
        all_directions_match = all(
            bool(wheel["direction_matches"])
            for wheel in report.values()
            if isinstance(wheel, Mapping)
        )
        return report, all_directions_match

    wheel_report, direction_matches = wheel_direction_report(command_joints)
    steady_wheel_report, steady_direction_matches = wheel_direction_report(
        steady_state_joints,
        include_sample_counts=True,
    )

    stop = detect_stopping(
        end_ns,
        odom_samples,
        joint_samples,
        wheels,
        stop_settings,
        sample_age_limit,
    )
    trajectory_points = [(sample.x_m, sample.y_m) for sample in command_odom]
    max_radial_displacement_from_start = _finite_number(
        max(
            math.hypot(sample.x_m - start.x_m, sample.y_m - start.y_m)
            for sample in command_odom
        ),
        "max_radial_displacement_from_start_m",
    )
    return {
        "segment_id": segment.segment_id,
        "motion": segment.motion,
        "tier": segment.tier,
        "result": "complete" if stop.stopped else "stop_timeout",
        "command": {
            "linear_x_mps": segment.linear_x_mps,
            "angular_z_radps": segment.angular_z_radps,
            "configured_duration_sec": segment.duration_sec,
            "observed_duration_sec": observed_duration,
            "publish_count": command_publish_count,
            "start_stamp_ns": start_ns,
            "end_stamp_ns": end_ns,
        },
        "sample_counts": {
            "odom_command": len(command_odom),
            "odom_total": len(odom_samples),
            "joint_states_command": len(command_joints),
            "joint_states_total": len(joint_samples),
        },
        "pose": {
            "start": start.pose_dict(),
            "end": end.pose_dict(),
            "trajectory_length_m": path_length(trajectory_points),
            "max_radial_displacement_from_start_m": (
                max_radial_displacement_from_start
            ),
            "net_displacement_m": math.hypot(delta_x, delta_y),
            "longitudinal_displacement_m": longitudinal,
            "expected_longitudinal_displacement_m": expected_longitudinal,
            "longitudinal_error_m": longitudinal - expected_longitudinal,
            "lateral_displacement_m": lateral,
            "lateral_drift_m": lateral if pure_or_straight else None,
            "translation_drift_m": (
                math.hypot(delta_x, delta_y) if pure_rotation else None
            ),
        },
        "yaw": {
            "change_rad": yaw_change,
            "expected_change_rad": expected_yaw_change,
            "error_rad": yaw_change - expected_yaw_change,
        },
        "actual_velocity": {
            "linear_x_mps": _distribution(
                sample.linear_x_mps for sample in command_odom
            ),
            "linear_y_mps": _distribution(
                sample.linear_y_mps for sample in command_odom
            ),
            "linear_speed_mps": _distribution(
                math.hypot(sample.linear_x_mps, sample.linear_y_mps)
                for sample in command_odom
            ),
            "angular_z_radps": _distribution(
                sample.angular_z_radps for sample in command_odom
            ),
            "steady_state_window": {
                "schema_version": 1,
                "definition": "final_half_of_command_interval",
                "boundary_semantics": "closed_interval",
                "start_stamp_ns": steady_state_start_ns,
                "end_stamp_ns": end_ns,
                "observed_duration_sec": (
                    end_ns - steady_state_start_ns
                )
                / NANOSECONDS_PER_SECOND,
                "sample_count": len(steady_state_odom),
                "first_sample_stamp_ns": steady_odom_stamps[0],
                "last_sample_stamp_ns": steady_odom_stamps[-1],
                "maximum_inter_sample_gap_sec": maximum_odom_gap_sec,
                "angular_z_radps": _distribution(
                    sample.angular_z_radps for sample in steady_state_odom
                ),
            },
        },
        "stopping": stop.as_dict(),
        "wheels": {
            "all_directions_match": direction_matches,
            "per_wheel": wheel_report,
            "steady_state_window": {
                "schema_version": 1,
                "definition": "final_half_of_command_interval",
                "boundary_semantics": "closed_interval",
                "start_stamp_ns": steady_state_start_ns,
                "end_stamp_ns": end_ns,
                "observed_duration_sec": (
                    end_ns - steady_state_start_ns
                )
                / NANOSECONDS_PER_SECOND,
                "sample_count": len(steady_state_joints),
                "first_sample_stamp_ns": steady_joint_stamps[0],
                "last_sample_stamp_ns": steady_joint_stamps[-1],
                "maximum_inter_sample_gap_sec": maximum_joint_gap_sec,
                "classification_deadband_radps": (
                    stop_settings.wheel_velocity_threshold_radps
                ),
                "all_directions_match": steady_direction_matches,
                "per_wheel": steady_wheel_report,
            },
        },
        "timestamp_integrity": {
            name: dict(report) for name, report in timestamp_integrity.items()
        },
    }
