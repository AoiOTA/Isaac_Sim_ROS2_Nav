from __future__ import annotations

import math
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest
import yaml

from robot_experiments.motion_benchmark import (
    estimated_state_is_fresh,
    evaluate_motion_primitive,
    load_motion_config,
    MotionDispatchBarrier,
    MotionBenchmarkError,
    MotionBenchmarkNode,
    MotionPrimitive,
    MotionSafetyStop,
    MotionSample,
    MotionSegment,
    MotionThresholds,
    ResetStopGateStatus,
    StationaryReference,
    StampedObservation,
    validate_motion_dispatch,
    wait_for_motion_dispatch_barrier,
)


CONFIG = Path(__file__).resolve().parents[1] / "config/motion_benchmark.yaml"


def test_motion_benchmark_is_upstream_of_final_command_authority():
    source = (
        Path(__file__).resolve().parents[1]
        / "robot_experiments/motion_benchmark.py"
    ).read_text(encoding="utf-8")
    assert 'Twist, "/cmd_vel_nav", reliable' in source
    assert 'Twist, "/cmd_vel", reliable' not in source


def _gate_status(generation: int, held: bool, received_at: float):
    return ResetStopGateStatus(
        generation=generation,
        held=held,
        eligible_generation=generation if held else None,
        received_at=received_at,
    )


def test_motion_dispatch_waits_for_delayed_same_generation_release():
    barrier = MotionDispatchBarrier(7, reset_started_at=1.0, settle_sec=0.1)
    assert not barrier.observe(
        gate_status=_gate_status(7, True, 1.1),
        collision_monitor_active=True,
        estimated_state_ready=True,
        now=1.1,
    )
    assert not barrier.observe(
        gate_status=_gate_status(7, False, 1.2),
        collision_monitor_active=True,
        estimated_state_ready=True,
        now=1.2,
    )
    assert barrier.observe(
        gate_status=_gate_status(7, False, 1.3),
        collision_monitor_active=True,
        estimated_state_ready=True,
        now=1.3,
    )


def test_motion_dispatch_never_release_times_out():
    now = [0.0]
    held = _gate_status(3, True, 0.1)
    barrier = MotionDispatchBarrier(3, reset_started_at=0.0, settle_sec=0.0)

    def spin_once(_timeout):
        now[0] += 0.05

    with pytest.raises(TimeoutError, match="gate_released=False"):
        wait_for_motion_dispatch_barrier(
            barrier,
            spin_once=spin_once,
            snapshot=lambda: (held, True, True),
            timeout_sec=0.2,
            monotonic=lambda: now[0],
        )


def test_motion_dispatch_rejects_wrong_reset_generation():
    barrier = MotionDispatchBarrier(8, reset_started_at=2.0, settle_sec=0.0)
    with pytest.raises(RuntimeError, match="generation mismatch"):
        barrier.observe(
            gate_status=_gate_status(9, False, 2.1),
            collision_monitor_active=True,
            estimated_state_ready=True,
            now=2.1,
        )


def test_motion_dispatch_success_requires_collision_monitor_and_estimated_state():
    barrier = MotionDispatchBarrier(5, reset_started_at=1.0, settle_sec=0.0)
    released = _gate_status(5, False, 1.1)
    assert not barrier.observe(
        gate_status=released,
        collision_monitor_active=False,
        estimated_state_ready=True,
        now=1.1,
    )
    assert not barrier.observe(
        gate_status=released,
        collision_monitor_active=True,
        estimated_state_ready=False,
        now=1.2,
    )
    assert barrier.observe(
        gate_status=released,
        collision_monitor_active=True,
        estimated_state_ready=True,
        now=1.3,
    )


def _observation(received_at, stamp_s, progressed_at=None):
    return StampedObservation(
        received_at=received_at,
        stamp_s=stamp_s,
        progressed_at=(received_at if progressed_at is None else progressed_at),
    )


def test_motion_dispatch_snapshot_once_becomes_stale_before_settle():
    now = [0.0]
    released = _gate_status(6, False, 0.01)
    one_shot = _observation(0.05, 10.0)
    barrier = MotionDispatchBarrier(6, reset_started_at=0.0, settle_sec=0.30)

    def spin_once(_timeout):
        now[0] += 0.05

    def snapshot():
        ready = estimated_state_is_fresh(
            clock=one_shot,
            odometry=one_shot,
            transform=one_shot,
            reset_started_at=0.0,
            now=now[0],
            max_age_sec=0.10,
            stamp_coherence_sec=0.10,
        )
        return released, True, ready

    with pytest.raises(TimeoutError, match="estimated_state_ready=False"):
        wait_for_motion_dispatch_barrier(
            barrier,
            spin_once=spin_once,
            snapshot=snapshot,
            timeout_sec=0.50,
            monotonic=lambda: now[0],
        )


@pytest.mark.parametrize("stale_stream", ["odometry", "transform"])
def test_estimated_state_rejects_stale_odom_or_tf(stale_stream):
    streams = {
        "clock": _observation(0.95, 10.0),
        "odometry": _observation(0.95, 10.0),
        "transform": _observation(0.95, 10.0),
    }
    streams[stale_stream] = _observation(0.50, 10.0)
    assert not estimated_state_is_fresh(
        **streams,
        reset_started_at=0.0,
        now=1.0,
        max_age_sec=0.20,
        stamp_coherence_sec=0.20,
    )


def test_estimated_state_rejects_stamp_clock_incoherence():
    assert not estimated_state_is_fresh(
        clock=_observation(0.95, 10.0),
        odometry=_observation(0.95, 9.0),
        transform=_observation(0.95, 10.0),
        reset_started_at=0.0,
        now=1.0,
        max_age_sec=0.20,
        stamp_coherence_sec=0.20,
    )


def test_motion_dispatch_passes_with_continuously_advancing_coherent_state():
    now = [0.0]
    released = _gate_status(11, False, 0.01)
    barrier = MotionDispatchBarrier(11, reset_started_at=0.0, settle_sec=0.15)

    def spin_once(_timeout):
        now[0] += 0.05

    def snapshot():
        current = _observation(now[0], 20.0 + now[0])
        return (
            released,
            True,
            estimated_state_is_fresh(
                clock=current,
                odometry=current,
                transform=current,
                reset_started_at=0.0,
                now=now[0],
                max_age_sec=0.10,
                stamp_coherence_sec=0.10,
            ),
        )

    wait_for_motion_dispatch_barrier(
        barrier,
        spin_once=spin_once,
        snapshot=snapshot,
        timeout_sec=0.50,
        monotonic=lambda: now[0],
    )


def test_dispatch_rechecks_collision_monitor_after_settle():
    with pytest.raises(MotionSafetyStop, match="collision_monitor_inactive"):
        validate_motion_dispatch(
            generation=4,
            reset_started_at=1.0,
            gate_status=_gate_status(4, False, 1.1),
            collision_monitor_active=False,
            estimated_state_ready=True,
        )


def test_final_collision_query_replaces_cached_active_with_inactive():
    calls = []
    response = SimpleNamespace(current_state=SimpleNamespace(id=2))
    future = SimpleNamespace()
    client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: True,
        call_async=lambda request: calls.append(request) or future,
    )
    node = SimpleNamespace(
        _collision_state_future=None,
        _collision_state_client=client,
        _collision_monitor_active=True,
        _collision_state_received_at=0.0,
        _wait_future=lambda value, timeout: response,
    )

    assert not MotionBenchmarkNode._query_collision_monitor_active(node)
    assert len(calls) == 1
    assert not node._collision_monitor_active


def test_gate_returning_to_hold_during_settle_is_stop_condition():
    barrier = MotionDispatchBarrier(12, reset_started_at=1.0, settle_sec=0.2)
    assert not barrier.observe(
        gate_status=_gate_status(12, False, 1.1),
        collision_monitor_active=True,
        estimated_state_ready=True,
        now=1.1,
    )
    with pytest.raises(RuntimeError, match="returned to HOLD"):
        barrier.observe(
            gate_status=_gate_status(12, True, 1.2),
            collision_monitor_active=True,
            estimated_state_ready=True,
            now=1.2,
        )


def test_sim_clock_freeze_publishes_zero_and_records_stop(monkeypatch):
    published = []
    primitive = MotionPrimitive("freeze", (MotionSegment(1.0, 0.2, 0.0),))
    node = SimpleNamespace(
        _config=SimpleNamespace(
            primitives=(primitive,),
            reset_seed=3,
            spawn_pose_name="mapping_start",
            command_rate_hz=20.0,
            reset_settle_sec=0.60,
            state_freshness_sec=0.25,
            stamp_coherence_sec=0.50,
            thresholds=_thresholds(),
            sim_clock_stall_timeout_sec=0.10,
        ),
        _clock_observation=_observation(1.0, 5.0),
        _reset_receipts=[],
        _current_reset_receipt=None,
        _samples=[],
        _collision_detected=False,
        _recording=False,
        _publish=lambda linear, angular: published.append((linear, angular)),
        _reset=lambda seed: {"seed": seed, "generation": 1},
        _settle=lambda: None,
        get_logger=lambda: SimpleNamespace(info=lambda message: None, error=lambda message: None),
    )
    node._stop = MethodType(MotionBenchmarkNode._stop, node)
    node._play_segment = lambda index, segment: (
        MotionBenchmarkNode._assert_sim_clock_live(node)
    )
    monkeypatch.setattr(
        "robot_experiments.motion_benchmark.time.monotonic", lambda: 1.2
    )

    report = MotionBenchmarkNode.run(node)

    assert report["stopped"]
    assert not report["passed"]
    assert report["primitives"][0]["outcome"] == "STOP"
    assert report["primitives"][0]["failure_reasons"] == [
        "sim_clock_stalled_during_motion"
    ]
    assert published and all(command == (0.0, 0.0) for command in published)
    assert report["reset_settle_sec"] == 0.60
    assert report["state_freshness_sec"] == 0.25
    assert report["stamp_coherence_sec"] == 0.50
    assert report["sim_clock_stall_timeout_sec"] == 0.10
    assert report["dispatch_barrier_timeout_sec"] == 15.0
    assert report["collision_monitor_state_freshness_sec"] == 0.25
    assert report["collision_monitor_query_timeout_sec"] == 2.0
    assert report["collision_monitor_required_state"] == "active"
    assert report["reset_stop_gate_generation_match_required"] is True


def test_second_primitive_dispatch_stop_has_fresh_state_and_receipt(monkeypatch):
    published = []
    primitives = (
        MotionPrimitive("first", (MotionSegment(1.0, 0.2, 0.0),)),
        MotionPrimitive("second", (MotionSegment(1.0, 0.2, 0.0),)),
    )
    node = SimpleNamespace(
        _config=SimpleNamespace(
            primitives=primitives,
            reset_seed=41,
            spawn_pose_name="mapping_start",
            command_rate_hz=20.0,
            reset_settle_sec=0.60,
            final_settle_sec=0.80,
            steady_window_sec=0.20,
            state_freshness_sec=0.25,
            stamp_coherence_sec=0.50,
            sim_clock_stall_timeout_sec=0.50,
            thresholds=_thresholds(),
        ),
        _reset_receipts=[],
        _current_reset_receipt=None,
        _samples=[object()],
        _collision_detected=True,
        _recording=True,
        _segment_index=99,
        _segment_started_at=99.0,
        _command_linear=99.0,
        _command_angular=99.0,
        _gate_status=None,
        _gate_status_error=None,
        _collision_state_future=None,
        _collision_monitor_active=False,
        _collision_state_received_at=None,
        _publish=lambda linear, angular: published.append((linear, angular)),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            error=lambda message: None,
        ),
    )

    def receipt(seed, generation):
        return {
            "requested_seed": seed,
            "actual_seed": seed,
            "generation": generation,
            "pose": "mapping_start",
            "odometry": "estimated",
            "case_id": "",
            "variant_id": "",
            "full_response": (
                "reset complete; reset_receipt="
                f'{{"seed":{seed},"generation":{generation},'
                '"pose":"mapping_start","odometry":"estimated",'
                '"case_id":"","variant_id":""}'
            ),
        }

    current_seed = [None]

    class IsaacParameters:
        @staticmethod
        def wait_for_services(timeout_sec):
            return True

        @staticmethod
        def set_parameters(parameters):
            current_seed[0] = next(
                parameter.value
                for parameter in parameters
                if parameter.name == "reset_seed"
            )
            return SimpleNamespace(
                results=[SimpleNamespace(successful=True)]
            )

    class ResetClient:
        @staticmethod
        def wait_for_service(timeout_sec):
            return True

        @staticmethod
        def call_async(request):
            seed = current_seed[0]
            generation = seed - 40
            return SimpleNamespace(
                success=True,
                message=receipt(seed, generation)["full_response"],
            )

    def wait_for_dispatch(barrier, **kwargs):
        node._gate_status = _gate_status(
            barrier.generation,
            False,
            barrier.reset_started_at + 0.01,
        )
        if barrier.generation == 2:
            raise TimeoutError("gate held")

    node._isaac_parameters = IsaacParameters()
    node._reset_client = ResetClient()
    node._wait_future = lambda future, timeout: future
    node._spin_once = lambda timeout: None
    node._query_collision_monitor_active = lambda: True
    node._estimated_state_ready = lambda **kwargs: True
    node._reset = MethodType(MotionBenchmarkNode._reset, node)
    node._stop = MethodType(MotionBenchmarkNode._stop, node)
    monkeypatch.setattr(
        "robot_experiments.motion_benchmark.wait_for_motion_dispatch_barrier",
        wait_for_dispatch,
    )

    def play_segment(index, segment):
        node._samples.append(
            MotionSample(
                received_at=1.0,
                stamp_s=1.0,
                x=0.0,
                y=0.0,
                yaw=0.0,
                linear_speed=segment.linear_x,
                angular_speed=segment.angular_z,
                segment_index=index,
                segment_elapsed=0.5,
                command_linear=segment.linear_x,
                command_angular=segment.angular_z,
            )
        )
        node._collision_detected = True

    node._play_segment = play_segment
    node._settle = lambda: None

    report = MotionBenchmarkNode.run(node)

    first, stopped = report["primitives"]
    assert first["id"] == "first"
    assert first["sample_count"] == 1
    assert first["collision_detected"] is True
    assert first["reset_receipt"] == receipt(41, 1)
    assert stopped["id"] == "second"
    assert stopped["outcome"] == "STOP"
    assert stopped["sample_count"] == 0
    assert stopped["collision_detected"] is False
    assert stopped["reset_receipt"] == receipt(42, 2)
    assert report["reset_receipts"] == [receipt(41, 1), receipt(42, 2)]


def test_motion_benchmark_reset_publishes_hold_zeros(monkeypatch):
    """The post-reset dispatch barrier wait must emit command-rate zeros."""
    import time as real_time

    published = []
    node = SimpleNamespace(
        _config=SimpleNamespace(
            primitives=(),
            reset_seed=41,
            spawn_pose_name="mapping_start",
            command_rate_hz=20.0,
            reset_settle_sec=0.60,
            final_settle_sec=0.80,
            steady_window_sec=0.20,
            state_freshness_sec=0.25,
            stamp_coherence_sec=0.50,
            sim_clock_stall_timeout_sec=0.50,
            thresholds=None,
        ),
        _reset_receipts=[],
        _current_reset_receipt=None,
        _gate_status=None,
        _gate_status_error=None,
        _collision_state_future=None,
        _collision_monitor_active=True,
        _collision_state_received_at=None,
        _publish=lambda linear, angular: published.append((linear, angular)),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            error=lambda message: None,
        ),
    )
    receipt_message = (
        'reset complete; reset_receipt={"seed":41,"generation":7,'
        '"pose":"mapping_start","odometry":"estimated",'
        '"case_id":"","variant_id":""}'
    )

    class IsaacParameters:
        @staticmethod
        def wait_for_services(timeout_sec):
            return True

        @staticmethod
        def set_parameters(parameters):
            return SimpleNamespace(
                results=[SimpleNamespace(successful=True)]
            )

    class ResetClient:
        @staticmethod
        def wait_for_service(timeout_sec):
            return True

        @staticmethod
        def call_async(request):
            return SimpleNamespace(success=True, message=receipt_message)

    captured = {}

    def fake_wait(barrier, **kwargs):
        node._gate_status = _gate_status(
            barrier.generation,
            False,
            barrier.reset_started_at + 0.01,
        )
        captured["spin_once"] = kwargs["spin_once"]

    node._isaac_parameters = IsaacParameters()
    node._reset_client = ResetClient()
    node._wait_future = lambda future, timeout: future
    node._spin_once = lambda timeout: None
    node._query_collision_monitor_active = lambda: True
    node._estimated_state_ready = lambda **kwargs: True
    node._stop = MethodType(MotionBenchmarkNode._stop, node)
    node._reset = MethodType(MotionBenchmarkNode._reset, node)
    monkeypatch.setattr(
        "robot_experiments.motion_benchmark.wait_for_motion_dispatch_barrier",
        fake_wait,
    )

    receipt = node._reset(41)
    assert receipt["generation"] == 7
    spin_once = captured["spin_once"]
    baseline = len(published)
    spin_once(0.0)
    assert len(published) == baseline + 1
    spin_once(0.0)
    assert len(published) == baseline + 1
    real_time.sleep(1.0 / node._config.command_rate_hz + 0.02)
    spin_once(0.0)
    assert len(published) == baseline + 2
    assert all(call == (0.0, 0.0) for call in published)


def test_motion_benchmark_config_covers_required_primitives():
    config = load_motion_config(CONFIG)
    identifiers = {primitive.identifier for primitive in config.primitives}
    assert {
        "spin_left",
        "spin_right",
        "forward_circle_left",
        "forward_circle_right",
        "reverse_circle_left",
        "reverse_circle_right",
        "reverse_straight",
        "forward_sharp_slalom",
        "reverse_sharp_slalom",
        "rapid_spin_reversal",
    } <= identifiers
    assert config.command_rate_hz >= 20.0
    assert config.stationary_reference is None


def test_optional_stationary_reference_has_own_reset_zero_and_report():
    published = []
    reference = StationaryReference("stationary_reference", 10.0, 8609)
    receipt = {
        "requested_seed": 8609, "actual_seed": 8609,
        "generation": 4, "pose": "flat20_start",
    }
    node = SimpleNamespace(
        _config=SimpleNamespace(
            stationary_reference=reference,
            command_rate_hz=20.0,
            final_settle_sec=0.8,
        ),
        _samples=[], _collision_detected=False, _recording=False,
        _segment_index=-1, _segment_started_at=0.0,
        _command_linear=0.0, _command_angular=0.0,
        _current_reset_receipt=None, _clock_s=0.0,
        _publish=lambda linear, angular: published.append((linear, angular)),
        _assert_sim_clock_live=lambda: None,
    )

    def reset(seed):
        assert seed == 8609
        node._current_reset_receipt = receipt
        return receipt

    def spin(_timeout):
        node._clock_s += 0.1
        if node._recording:
            node._samples.append(MotionSample(
                received_at=node._clock_s, stamp_s=node._clock_s,
                x=0.0, y=0.0, yaw=0.0, linear_speed=0.0, angular_speed=0.0,
                segment_index=-1, segment_elapsed=node._clock_s,
                command_linear=0.0, command_angular=0.0,
            ))

    node._reset = reset
    node._spin_once = spin
    node._settle = MethodType(MotionBenchmarkNode._settle, node)
    result = MotionBenchmarkNode._stationary_reference(node)
    assert result["passed"] is True
    assert result["reset_seed"] == 8609
    assert result["reset_receipt"] == receipt
    assert result["measured_duration_sec"] >= 9.9
    assert result["collision_detected"] is False
    assert result["final_zero_published"] is True
    schedule = result["segment_schedule"]
    assert len(schedule) == 1
    assert schedule[0]["segment_index"] == 0
    assert schedule[0]["start_sim_s"] == 0.0
    assert 10.0 <= schedule[0]["end_sim_s"] <= 10.11
    assert schedule[0]["expected_duration_s"] == 10.0
    assert schedule[0]["command_linear_mps"] == 0.0
    assert schedule[0]["command_angular_radps"] == 0.0
    assert schedule[0]["intent_publish_count"] > 0
    assert schedule[0]["completion"] == "COMPLETED"
    assert schedule[0]["truncated"] is False
    assert result["final_zero_publish_receipt"]["publish_count"] > 0
    assert result["zero_command_count"] == (
        schedule[0]["intent_publish_count"]
        + result["final_zero_publish_receipt"]["publish_count"]
    )
    assert (
        result["final_zero_publish_receipt"]["last_sim_s"]
        - result["final_zero_publish_receipt"]["first_sim_s"]
        >= 0.8
    )
    assert published and set(published) == {(0.0, 0.0)}


def test_play_segment_records_immutable_truncated_schedule_on_stop(monkeypatch):
    published = []
    node = SimpleNamespace(
        _config=SimpleNamespace(command_rate_hz=20.0),
        _clock_s=1.0,
        _segment_schedule=[],
        _publish=lambda linear, angular: published.append((linear, angular)),
        _assert_sim_clock_live=lambda: None,
    )
    calls = [0]

    def spin(_timeout):
        calls[0] += 1
        node._clock_s += 0.05
        if calls[0] == 3:
            raise MotionSafetyStop("forced_stop")

    node._spin_once = spin
    monkeypatch.setattr(
        "robot_experiments.motion_benchmark.time.monotonic",
        lambda: 10.0 + calls[0] * 0.1,
    )
    with pytest.raises(MotionSafetyStop, match="forced_stop"):
        MotionBenchmarkNode._play_segment(
            node, 0, MotionSegment(1.0, 0.2, -0.5)
        )
    receipt = node._segment_schedule[0]
    assert receipt["segment_index"] == 0
    assert receipt["start_sim_s"] == 1.0
    assert receipt["end_sim_s"] == pytest.approx(1.15)
    assert receipt["expected_duration_s"] == 1.0
    assert receipt["command_linear_mps"] == 0.2
    assert receipt["command_angular_radps"] == -0.5
    assert receipt["intent_publish_count"] > 0
    assert receipt["completion"] == "TRUNCATED"
    assert receipt["truncated"] is True


def test_motion_benchmark_rejects_duplicate_primitive_ids(tmp_path):
    document = yaml.safe_load(CONFIG.read_text())
    document["primitives"].append(document["primitives"][0])
    target = tmp_path / "motion.yaml"
    target.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(MotionBenchmarkError, match="duplicate"):
        load_motion_config(target)


def _thresholds() -> MotionThresholds:
    return MotionThresholds(
        linear_mae_mps=0.06,
        angular_mae_radps=0.12,
        radius_relative_error_percent=20.0,
        tracking_fraction=0.85,
        transition_latency_sec=0.45,
        overshoot_ratio=1.25,
        wrong_direction_fraction=0.05,
    )


def test_motion_evaluation_accepts_smooth_forward_arc():
    primitive = MotionPrimitive(
        "arc",
        (MotionSegment(2.0, 0.30, 0.80),),
    )
    samples = [
        MotionSample(
            received_at=index * 0.05,
            stamp_s=index * 0.05,
            x=index * 0.015,
            y=0.0,
            yaw=index * 0.04,
            linear_speed=0.295,
            angular_speed=0.79,
            segment_index=0,
            segment_elapsed=index * 0.05,
            command_linear=0.30,
            command_angular=0.80,
        )
        for index in range(40)
    ]
    result = evaluate_motion_primitive(
        primitive,
        samples,
        False,
        _thresholds(),
        0.30,
    )
    assert result["passed"]
    assert result["maximum_radius_relative_error_percent"] < 5.0


def test_motion_evaluation_measures_turn_reversal_latency():
    primitive = MotionPrimitive(
        "slalom",
        (
            MotionSegment(1.0, 0.30, 1.0),
            MotionSegment(1.0, 0.30, -1.0),
        ),
    )
    samples: list[MotionSample] = []
    for segment_index, command in enumerate((1.0, -1.0)):
        for index in range(20):
            elapsed = index * 0.05
            actual_angular = command if segment_index == 0 or elapsed >= 0.15 else 0.2
            samples.append(
                MotionSample(
                    received_at=len(samples) * 0.05,
                    stamp_s=len(samples) * 0.05,
                    x=len(samples) * 0.015,
                    y=0.0,
                    yaw=0.0,
                    linear_speed=0.30,
                    angular_speed=actual_angular,
                    segment_index=segment_index,
                    segment_elapsed=elapsed,
                    command_linear=0.30,
                    command_angular=command,
                )
            )
    result = evaluate_motion_primitive(
        primitive,
        samples,
        False,
        _thresholds(),
        0.30,
    )
    assert result["passed"]
    assert result["maximum_turn_transition_latency_sec"] == pytest.approx(0.15)


def test_motion_evaluation_rejects_collision_and_wrong_reverse_direction():
    primitive = MotionPrimitive(
        "reverse",
        (MotionSegment(1.0, -0.30, 0.0),),
    )
    samples = [
        MotionSample(
            received_at=index * 0.05,
            stamp_s=index * 0.05,
            x=index * 0.01,
            y=0.0,
            yaw=0.0,
            linear_speed=0.20,
            angular_speed=0.0,
            segment_index=0,
            segment_elapsed=index * 0.05,
            command_linear=-0.30,
            command_angular=0.0,
        )
        for index in range(20)
    ]
    result = evaluate_motion_primitive(
        primitive,
        samples,
        True,
        _thresholds(),
        0.20,
    )
    assert not result["passed"]
    assert "collision_detected" in result["failure_reasons"]
    assert "wrong_direction" in result["failure_reasons"]
    assert math.isclose(result["translation_tracking_fraction"], 0.0)
