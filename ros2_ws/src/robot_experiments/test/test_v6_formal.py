from collections import deque
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import robot_experiments.v6_formal as v6_formal_module
import yaml

from robot_experiments.v6_formal import (
    DISPATCH_SUBSCRIPTION_TOPICS,
    DynamicActionLedger,
    DynamicScheduleEntry,
    ENGINEERING_PILOT,
    EpisodeGuard,
    MissionLeg,
    NOT_QUALIFIED,
    ReadinessFacts,
    V6ContractError,
    V6FormalNode,
    authorize_manifest,
    cli,
    load_manifest,
)


PACKAGE = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
CONFIG = PACKAGE / "config"
MANIFEST = CONFIG / "v6_r3_phase2_kujiale_baseline.yaml"
PHASE_B_MANIFEST = CONFIG / "v6_r5_phase_b_kujiale_exact_baseline.yaml"
LEGACY_MANIFESTS = tuple(
    CONFIG / f"v6_final_{world}_{category}.yaml"
    for world in ("kujiale", "rivermark")
    for category in ("static", "dynamic", "appearance")
)


def _raw() -> dict:
    return yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))


def _write_manifest(tmp_path: Path, raw: dict) -> Path:
    path = tmp_path / "manifest.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return path


def ready_facts() -> ReadinessFacts:
    return ReadinessFacts(
        **{name: True for name in ReadinessFacts.__dataclass_fields__}
    )


def ready_guard(*legs: str) -> EpisodeGuard:
    guard = EpisodeGuard(mission_leg_ids=legs)
    guard.arm_reset(ready_facts())
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_receipt_generation(1)
    guard.record_reset_event()
    guard.record_initialpose(100)
    guard.record_amcl(101)
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_gate_status(1, False)
    assert guard.goal_ready
    return guard


def test_r3_phase2_manifest_is_the_only_dispatch_candidate():
    manifest = load_manifest(MANIFEST)

    assert manifest.scene_id == "v6_kujiale_clearance_r2"
    assert [leg.goal_id for leg in manifest.mission_legs] == [
        "G2", "G3", "G4", "G5", "G1"
    ]
    assert manifest.dynamic_schedule == ()
    assert manifest.runtime["canonical_odom"] == {
        "topic": "/odom",
        "owner": "isaac_compute_odometry",
        "tf": "odom->base_link",
    }
    assert manifest.runtime["global_localization"] == {
        "pose_topic": "/amcl_pose",
        "owner": "amcl",
        "tf": "map->odom",
    }
    assert manifest.runtime["module1_odom"] == {
        "topic": "/bio_nav/module1/odom",
        "owner": "wheel_imu_ekf",
        "publish_tf": False,
    }
    assert manifest.runtime["recovery_enabled"] is False
    assert manifest.runtime["module2_navigation_write_enabled"] is False
    assert manifest.runtime["cognitive_place_graph_enabled"] is False
    assert manifest.runtime["route_backend"] == "gvg"
    assert manifest.runtime["low_obstacles_enabled"] is False
    assert manifest.runtime["dynamic_actors_enabled"] is False
    assert manifest.runtime["goal_checker"] == "position_xy"

    text = MANIFEST.read_text(encoding="utf-8")
    for forbidden in ("B5", "M3", "primary", "rf2o"):
        assert forbidden not in text


def test_r5_phase_b_manifest_binds_original_scene_and_shadow_baseline():
    manifest = load_manifest(PHASE_B_MANIFEST)

    assert manifest.scene_id == "kujiale_0026_A_to_B_door_open"
    assert manifest.runtime["cognitive_profile"] == "M0"
    assert manifest.runtime["module1_mode"] == "shadow"
    assert manifest.runtime["module2_navigation_write_enabled"] is False
    assert manifest.runtime["cognitive_place_graph_enabled"] is False
    assert manifest.runtime["route_backend"] == "gvg"
    assert manifest.runtime["low_obstacles_enabled"] is False
    assert manifest.runtime["dynamic_actors_enabled"] is False
    assert manifest.assets["scene_asset"].endswith(
        "/kujiale_0026/kujiale_0026_A_to_B_door_open.usd"
    )
    assert manifest.assets["occupancy_map"].endswith(
        "/data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml"
    )
    assert manifest.assets["spawn_manifest"].endswith(
        "/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
    )
    assert manifest.assets["route_graph"].endswith(
        "/v6_kujiale_isaacgen_v1_gvg_v1.geojson"
    )
    assert [leg.goal_id for leg in manifest.mission_legs] == [
        "G2", "G3", "G4", "G5", "G1"
    ]


def test_r5_phase_b_rejects_nonexact_scene_asset(tmp_path):
    raw = yaml.safe_load(PHASE_B_MANIFEST.read_text(encoding="utf-8"))
    raw["assets"]["scene_asset"] = "/tmp/modified_scene.usd"
    with pytest.raises(V6ContractError, match="accepted Phase B asset"):
        load_manifest(_write_manifest(tmp_path, raw))


@pytest.mark.parametrize("path", LEGACY_MANIFESTS)
def test_legacy_campaign_manifests_are_rejected_by_r3_dispatcher(path):
    with pytest.raises(V6ContractError, match="schema_version"):
        load_manifest(path)


def test_r3_phase2_is_pilot_only(capsys):
    manifest = load_manifest(MANIFEST)
    assert authorize_manifest(manifest, mode="pilot") == NOT_QUALIFIED
    with pytest.raises(V6ContractError, match="engineering pilot only"):
        authorize_manifest(manifest, mode="formal")

    assert cli(["--manifest", str(MANIFEST), "--pilot"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["qualification"] == ENGINEERING_PILOT
    assert payload["formal_qualification"] == NOT_QUALIFIED
    assert payload["dispatch"] is False

    assert cli(["--manifest", str(MANIFEST)]) == 2
    assert "engineering pilot only" in capsys.readouterr().err


def test_runtime_contract_rejects_nonbaseline_navigation_features(tmp_path):
    for key, value in (
        ("recovery_enabled", True),
        ("module2_navigation_write_enabled", True),
        ("cognitive_place_graph_enabled", True),
        ("route_backend", "primary"),
        ("low_obstacles_enabled", True),
        ("dynamic_actors_enabled", True),
    ):
        raw = _raw()
        raw["runtime"][key] = value
        with pytest.raises(V6ContractError, match=f"runtime.{key}"):
            load_manifest(_write_manifest(tmp_path, raw))


def test_mission_legs_are_xy_only_and_schedule_is_separate(tmp_path):
    manifest = load_manifest(MANIFEST)
    assert all(
        set(row) == {"id", "frame_id", "x", "y"}
        for row in manifest.raw["mission"]["legs"]
    )

    raw = _raw()
    raw["mission"]["legs"][0]["yaw_deg"] = 45.0
    with pytest.raises(V6ContractError, match="only id/frame_id/x/y"):
        load_manifest(_write_manifest(tmp_path, raw))


@pytest.mark.parametrize(
    "schedule, message",
    [
        ([{"leg_id": "missing", "group": "actor_a"}], "not a mission leg"),
        (
            [
                {"leg_id": "G2", "group": "actor_a"},
                {"leg_id": "G2", "group": "actor_b"},
            ],
            "must be unique",
        ),
        (
            [
                {"leg_id": "G2", "group": "actor_a"},
                {"leg_id": "G3", "group": "actor_a"},
            ],
            "must be unique",
        ),
    ],
)
def test_dynamic_schedule_validates_leg_and_uniqueness(tmp_path, schedule, message):
    raw = _raw()
    raw["dynamic_schedule"] = schedule
    with pytest.raises(V6ContractError, match=message):
        load_manifest(_write_manifest(tmp_path, raw))


def test_dynamic_schedule_parses_independently_from_xy_goals(tmp_path):
    raw = _raw()
    raw["dynamic_schedule"] = [{"leg_id": "G3", "group": "actor_a"}]
    manifest = load_manifest(_write_manifest(tmp_path, raw))
    assert manifest.dynamic_schedule == (DynamicScheduleEntry("G3", "actor_a"),)
    assert not hasattr(manifest.mission_legs[1], "dynamic_trigger_group")


def test_baseline_readiness_has_no_candidate_bridge_or_prior_dependency():
    fields = set(ReadinessFacts.__dataclass_fields__)
    assert fields == {
        "reset_service_ready",
        "reset_event_publisher_ready",
        "reset_subscriber_roster_ready",
        "route_goal_subscriber_ready",
        "clock_seen",
        "scan_seen",
        "map_seen",
        "navigation_graph_seen",
        "estimated_odom_seen",
    }
    assert not any("module2" in topic for topic in DISPATCH_SUBSCRIPTION_TOPICS)
    assert "/bio_nav/localization/candidates" not in DISPATCH_SUBSCRIPTION_TOPICS


def test_goal_requires_fresh_initialpose_then_amcl_nav_tf_and_gate_release():
    guard = EpisodeGuard(mission_leg_ids=("G2",))
    guard.arm_reset(ready_facts())
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_receipt_generation(7)
    guard.record_reset_event()
    guard.record_amcl(99)
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_gate_status(7, False)
    assert not guard.goal_ready

    guard.record_initialpose(100)
    guard.record_amcl(100)
    assert not guard.goal_ready
    guard.record_amcl(101)
    assert guard.localization_ready
    assert guard.goal_ready


def test_stale_reset_gate_release_does_not_authorize_goal():
    guard = EpisodeGuard(mission_leg_ids=("G2",))
    guard.arm_reset(ready_facts())
    guard.record_reset_call()
    guard.record_reset_response(True)
    guard.record_reset_event()
    guard.record_initialpose(100)
    guard.record_amcl(101)
    guard.record_navigation_ready(nav2_active=True, tf_active=True)
    guard.record_reset_receipt_generation(4)
    guard.record_reset_gate_status(3, False)
    assert not guard.goal_ready
    guard.record_reset_gate_status(4, False)
    assert guard.goal_ready


def _tf_message(parent: str, child: str):
    return SimpleNamespace(
        transforms=[
            SimpleNamespace(
                header=SimpleNamespace(frame_id=parent),
                child_frame_id=child,
            )
        ]
    )


def _stamped_pose(stamp_ns: int):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            )
        )
    )


def _tf_epoch_adapter() -> V6FormalNode:
    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter.guard = EpisodeGuard(mission_leg_ids=("G2",))
    adapter.map_odom_tf_seen = False
    adapter.odom_base_tf_seen = False
    adapter._capture = lambda *_args, **_kwargs: None
    return adapter


def _record_adapter_navigation_ready(adapter: V6FormalNode) -> None:
    adapter.guard.record_navigation_ready(
        nav2_active=True,
        tf_active=adapter.map_odom_tf_seen and adapter.odom_base_tf_seen,
    )


def test_reset_epoch_requires_both_tf_edges_to_be_observed_again():
    adapter = _tf_epoch_adapter()
    adapter._tf(_tf_message("map", "odom"))
    adapter._tf(_tf_message("odom", "base_link"))
    assert adapter.map_odom_tf_seen and adapter.odom_base_tf_seen

    adapter.guard.arm_reset(ready_facts())
    adapter.guard.record_reset_call()
    adapter.guard.record_reset_response(True)
    adapter._reset_gate_status(
        SimpleNamespace(data='{"generation":7,"held":false}')
    )
    adapter._reset_event(SimpleNamespace())
    adapter.guard.record_reset_receipt_generation(7)
    adapter._initialpose(_stamped_pose(100))
    adapter._amcl_pose(_stamped_pose(101))

    _record_adapter_navigation_ready(adapter)
    assert not adapter.guard.goal_ready
    assert not adapter.guard.tf_active

    adapter._tf(_tf_message("map", "odom"))
    _record_adapter_navigation_ready(adapter)
    assert not adapter.guard.goal_ready
    assert not adapter.guard.tf_active

    adapter._tf(_tf_message("odom", "base_footprint"))
    _record_adapter_navigation_ready(adapter)
    assert adapter.guard.tf_active
    assert adapter.guard.goal_ready


def test_invalid_reset_events_do_not_rewrite_tf_epoch_observations():
    out_of_order = _tf_epoch_adapter()
    out_of_order._tf(_tf_message("map", "odom"))
    out_of_order._reset_event(SimpleNamespace())
    assert out_of_order.guard.stop_reason == "reset_event_without_call"
    assert out_of_order.map_odom_tf_seen

    duplicate = _tf_epoch_adapter()
    duplicate.guard.arm_reset(ready_facts())
    duplicate.guard.record_reset_call()
    duplicate.guard.record_reset_response(True)
    duplicate._reset_event(SimpleNamespace())
    duplicate._tf(_tf_message("map", "odom"))
    duplicate._reset_event(SimpleNamespace())
    assert duplicate.guard.stop_reason == "second_reset_event"
    assert duplicate.map_odom_tf_seen
    duplicate._tf(_tf_message("odom", "base_link"))
    _record_adapter_navigation_ready(duplicate)
    assert not duplicate.guard.goal_ready


def test_reset_is_exactly_once():
    guard = EpisodeGuard()
    guard.arm_reset(ready_facts())
    guard.record_reset_call()
    guard.record_reset_response(None)
    assert guard.stop_reason == "reset_response_unknown"
    with pytest.raises(V6ContractError, match="reset_retry_forbidden"):
        guard.record_reset_call()


def test_multileg_order_and_xy_goal_message():
    guard = ready_guard("G2", "G3")
    guard.record_goal_publication("G2")
    guard.record_route_progress()
    guard.record_route_completion(True)
    with pytest.raises(V6ContractError, match="mission_leg_order"):
        guard.record_goal_publication("G4")

    class PoseStamped:
        def __init__(self):
            self.header = SimpleNamespace(frame_id="", stamp=None)
            self.pose = SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=0.0),
            )

    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter._types = {"PoseStamped": PoseStamped}
    adapter.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: "stamp")
        )
    )
    goal = adapter._goal_message(MissionLeg("G2", "map", 1.0, 2.0))
    assert (goal.pose.position.x, goal.pose.position.y) == (1.0, 2.0)
    assert goal.pose.orientation.z == 0.0
    assert goal.pose.orientation.w == 1.0


def _mission_leg_adapter(*, spin_result: bool, route_success: bool):
    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter.guard = ready_guard("G2")
    adapter.canonical_route_count = 0
    adapter.route_goal_results = []
    adapter._navigation_terminal_observed = False
    events = []
    adapter._call_dynamic_action = lambda group, action, timeout: (
        events.append((action, group)) or True
    )
    adapter._goal_message = lambda leg: f"goal:{leg.goal_id}"
    adapter.route_goal_publisher = SimpleNamespace(
        publish=lambda message: events.append(("publish", message))
    )
    adapter._write = lambda event, **payload: events.append((event, payload))

    def spin_until(_predicate, _timeout):
        if spin_result:
            adapter.guard.record_route_progress()
            adapter.guard.record_route_completion(route_success)
            adapter.canonical_route_count += 1
        return spin_result

    adapter._spin_until = spin_until
    return adapter, events


@pytest.mark.parametrize(
    "spin_result, route_success, expected_state",
    [(True, True, "SUCCEEDED"), (True, False, "FAILED"), (False, False, "STOP")],
)
def test_dynamic_action_triggers_before_goal_and_completes_after_leg_terminal(
    spin_result, route_success, expected_state
):
    adapter, events = _mission_leg_adapter(
        spin_result=spin_result, route_success=route_success
    )
    adapter._run_mission_leg(
        index=0,
        leg=MissionLeg("G2", "map", 1.0, 2.0),
        dynamic_group="actor_a",
        reset_timeout_sec=1.0,
        navigation_timeout_sec=2.0,
    )

    labels = [row[0] for row in events]
    assert labels.index("trigger") < labels.index("publish")
    assert labels.index("publish") < labels.index("complete")
    assert adapter.guard.state == expected_state


def test_dynamic_action_ledger_is_exactly_once():
    ledger = DynamicActionLedger()
    ledger.claim("actor_a", "trigger")
    ledger.claim("actor_a", "complete")
    with pytest.raises(V6ContractError, match="retry forbidden"):
        ledger.claim("actor_a", "complete")
    with pytest.raises(V6ContractError, match="completion before trigger"):
        DynamicActionLedger().claim("actor_b", "complete")


class _Twist:
    def __init__(self, *, nonzero: bool = False):
        self.linear = SimpleNamespace(x=0.2 if nonzero else 0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class _CancelGoal:
    class Request:
        pass


def _terminal_adapter(
    monkeypatch,
    *,
    state: str,
    downstream_events: tuple[tuple[float, bool], ...],
    cancel_done_after: float | None = 0.0,
    timeout_sec: float = 0.65,
):
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(v6_formal_module.time, "monotonic", lambda: clock.now)

    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter.guard = ready_guard("G2")
    adapter.guard.record_goal_publication("G2")
    adapter.guard.record_route_progress()
    if state == "SUCCEEDED":
        adapter.guard.record_route_completion(True)
    elif state == "FAILED":
        adapter.guard.record_route_completion(False)
    else:
        adapter.guard.stop("collision")
    adapter._terminal_cancel_requested = False
    adapter._terminal_cancel_future = None
    adapter._terminal_started_monotonic = None
    adapter._navigation_terminal_observed = state in {"SUCCEEDED", "FAILED"}
    adapter._terminal_zero_settled = False
    adapter._terminal_zero_confirmed = False
    adapter._terminal_zero_reason = "not_required"
    adapter._cmd_vel_sim_last_receive_monotonic = None
    adapter._cmd_vel_sim_last_nonzero_monotonic = None
    adapter._cmd_vel_sim_zero_stamps = deque()
    adapter._cmd_window = deque()
    adapter._types = {"CancelGoal": _CancelGoal, "Twist": _Twist}
    adapter.node = SimpleNamespace()
    adapter.TERMINAL_ZERO_TIMEOUT_SEC = timeout_sec
    adapter.TERMINAL_ZERO_PERIOD_SEC = 0.05
    adapter.TERMINAL_ZERO_QUIET_SEC = 0.15
    adapter.TERMINAL_ZERO_CADENCE_TOLERANCE_SEC = 0.10
    adapter._capture = lambda *_args, **_kwargs: None

    lifecycle = []
    adapter.navigate_cancel_client = SimpleNamespace(
        call_async=lambda request: (
            lifecycle.append((clock.now, "cancel"))
            or SimpleNamespace(
                done=lambda: cancel_done_after is not None
                and clock.now >= 10.0 + cancel_done_after
            )
        )
    )
    adapter.terminal_zero_publisher = SimpleNamespace(
        publish=lambda message: lifecycle.append((clock.now, "zero_publish"))
    )
    adapter._write = lambda event, **payload: lifecycle.append((clock.now, event))
    pending = list(downstream_events)

    def spin_once(_node, *, timeout_sec):
        clock.now += timeout_sec
        while pending and clock.now >= 10.0 + pending[0][0]:
            _offset, nonzero = pending.pop(0)
            adapter._track_command("/cmd_vel_sim", _Twist(nonzero=nonzero))

    adapter._rclpy = SimpleNamespace(ok=lambda: True, spin_once=spin_once)
    return adapter, clock, lifecycle


def test_success_terminal_settle_publishes_20hz_without_cancel(monkeypatch):
    adapter, _clock, lifecycle = _terminal_adapter(
        monkeypatch,
        state="SUCCEEDED",
        downstream_events=((0.05, False), (0.20, False)),
    )
    adapter._start_terminal_settle(cancel_navigation=False, reason="SUCCEEDED")

    assert adapter._settle_terminal_zero()
    labels = [label for _stamp, label in lifecycle]
    assert "cancel" not in labels
    publish_stamps = [stamp for stamp, label in lifecycle if label == "zero_publish"]
    assert len(publish_stamps) >= 4
    assert all(
        later - earlier == pytest.approx(0.05)
        for earlier, later in zip(publish_stamps, publish_stamps[1:])
    )


@pytest.mark.parametrize(
    "state, cancel_navigation",
    [("SUCCEEDED", False), ("FAILED", True), ("STOP", True)],
)
def test_result_routes_every_terminal_state_through_zero_settle(
    state, cancel_navigation
):
    adapter = V6FormalNode.__new__(V6FormalNode)
    adapter.guard = EpisodeGuard(state=state, stop_reason="failure" if state != "SUCCEEDED" else "")
    adapter.qualification = ENGINEERING_PILOT
    adapter.reset_receipt = None
    adapter.route_goal_results = []
    adapter.dynamic_actions = DynamicActionLedger()
    adapter.obstacle_state_messages = []
    adapter.collision = False
    adapter._terminal_zero_confirmed = True
    adapter._terminal_zero_reason = "terminal_zero_confirmed"
    calls = []
    adapter._start_terminal_settle = lambda **kwargs: calls.append(
        ("start", kwargs)
    )
    adapter._settle_terminal_zero = lambda: calls.append(("settle", {})) or True
    adapter._write = lambda *_args, **_kwargs: None

    result = adapter.result()

    assert result["state"] == state
    assert calls == [
        (
            "start",
            {
                "cancel_navigation": cancel_navigation,
                "reason": "failure" if state != "SUCCEEDED" else "SUCCEEDED",
            },
        ),
        ("settle", {}),
    ]


def test_success_terminal_settle_timeout_becomes_stop(monkeypatch):
    adapter, _clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="SUCCEEDED",
        downstream_events=((0.05, False),),
        timeout_sec=0.35,
    )
    adapter._start_terminal_settle(cancel_navigation=False, reason="SUCCEEDED")

    assert not adapter._settle_terminal_zero()
    assert adapter.guard.state == "STOP"
    assert adapter.guard.stop_reason == "terminal_zero_timeout_after_success"


def test_failed_terminal_cancels_before_zero_settle(monkeypatch):
    adapter, _clock, lifecycle = _terminal_adapter(
        monkeypatch,
        state="FAILED",
        downstream_events=((0.05, False), (0.20, False)),
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="route_failed")

    assert adapter._settle_terminal_zero()
    labels = [label for _stamp, label in lifecycle]
    assert labels.index("cancel") < labels.index("zero_publish")


def test_single_downstream_zero_plus_silence_never_passes(monkeypatch):
    adapter, _clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="STOP",
        downstream_events=((0.05, False),),
        timeout_sec=0.35,
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="collision")
    assert not adapter._settle_terminal_zero()
    assert not adapter._terminal_zero_confirmed


def test_repeated_downstream_zero_covers_quiet_window_and_passes(monkeypatch):
    adapter, clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="STOP",
        downstream_events=((0.05, False), (0.22, False)),
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="collision")
    assert adapter._settle_terminal_zero()
    assert clock.now <= 10.32


def test_repeated_zero_then_silence_fails_latest_cadence_check(monkeypatch):
    adapter, _clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="STOP",
        downstream_events=((0.05, False), (0.22, False)),
        cancel_done_after=0.45,
        timeout_sec=0.60,
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="collision")
    assert not adapter._settle_terminal_zero()


def test_downstream_nonzero_resets_zero_window(monkeypatch):
    adapter, clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="STOP",
        downstream_events=(
            (0.05, False),
            (0.15, False),
            (0.18, True),
            (0.30, False),
            (0.48, False),
        ),
    )
    adapter._start_terminal_settle(cancel_navigation=True, reason="collision")
    assert adapter._settle_terminal_zero()
    assert clock.now >= 10.48


def test_command_observation_and_terminal_publish_qos_are_depth_one():
    source = (PACKAGE / "robot_experiments" / "v6_formal.py").read_text()
    observation = source[
        source.index("command_observation_qos = QoSProfile("):
        source.index("terminal_zero_qos = QoSProfile(")
    ]
    assert "depth=1" in observation
    assert "ReliabilityPolicy.RELIABLE" in observation
    assert "DurabilityPolicy.VOLATILE" in observation
    for topic in ("/cmd_vel", "/cmd_vel_nav", "/cmd_vel_sim"):
        assert f'lambda m: self._track_command("{topic}", m), command_observation_qos' in source

    terminal = source[
        source.index("terminal_zero_qos = QoSProfile("):
        source.index("self.reset_client =")
    ]
    assert "depth=1" in terminal
    assert 'Twist, "/cmd_vel_nav", terminal_zero_qos' in terminal
    assert 'create_publisher(\n            Twist, "/cmd_vel_sim"' not in source


def test_reset_stop_gate_and_omnigraph_command_queues_remain_depth_one():
    gate = (REPO / "isaac_sim/src/bridge/reset_stop_gate.py").read_text()
    command_qos = gate[
        gate.index("command_qos = QoSProfile("):
        gate.index("status_qos = QoSProfile(")
    ]
    assert "HistoryPolicy.KEEP_LAST" in command_qos
    assert "depth=1" in command_qos

    graph = (REPO / "isaac_sim/graphs/control_graph.py").read_text()
    assert '("SubscribeTwist.inputs:queueSize", 1)' in graph


def test_dispatcher_ground_truth_firewall():
    assert DISPATCH_SUBSCRIPTION_TOPICS
    assert not [
        topic for topic in DISPATCH_SUBSCRIPTION_TOPICS
        if topic.startswith("/ground_truth/")
    ]
