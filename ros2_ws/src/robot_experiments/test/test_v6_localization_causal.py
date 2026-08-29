import json
import math
import os
from pathlib import Path
import shlex
import subprocess
from types import SimpleNamespace

import pytest
import yaml

import robot_experiments.v6_localization_causal as localization_causal
from robot_experiments.v6_localization_causal import (
    ALLOWED_EVENTS,
    ARMS,
    CONFIG_SCHEMA,
    EVENT_SCHEMA,
    PHASE_D_STARTUP_INITIALPOSE,
    PHASE_E_MANUAL_RECOVERY_EXPERIMENT,
    PHASE_E_MODE2_RECOVERY,
    RUN4_CANDIDATE_STATUS,
    SEED_CONFIRMATION_POSITION_THRESHOLD_M,
    SEED_CONFIRMATION_YAW_THRESHOLD_DEG,
    STARTUP_AMCL_POSES_REQUIRED,
    WHOLE_HOUSE_ONEBOX_VARIANT,
    WHOLE_HOUSE_USER_ARM_ALIASES,
    LocalizationCausalNode,
    LocalizationConfigError,
    _contains_ground_truth,
    _pose_disagreement,
    _propagate_module1_odom_delta,
    _require_sim_time,
    build_plan,
    cli,
    execute_route_actions,
    load_config,
    route_actions,
)


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config" / "v6_localization_causal.yaml"
REPO = Path(__file__).resolve().parents[4]
WRAPPER = REPO / "scripts" / "run_v6_localization_causal.sh"


def test_config_freezes_only_four_single_round_arms_and_held_constants():
    config = load_config(CONFIG)
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert raw["held_constants"]["use_sim_time"] is True
    assert tuple(config.seeds) == ARMS
    assert config.route_ids == ("G2", "G3", "G4", "G5", "G1")
    assert config.fault_id == "F2"
    assert config.fault_kind == "amcl_global_localization_particle_spread"
    assert config.fault_service == "/reinitialize_global_localization"
    assert config.seeds["S0"] == config.seeds["S1"]
    assert config.seeds["R0"] == config.seeds["R1"]
    assert config.phase_d_run4_candidate["status"] == RUN4_CANDIDATE_STATUS
    assert config.phase_d_run4_candidate["model_id"] == (
        "kujiale_0026_visual_heads_run4_v310"
    )
    assert config.phase_d_run4_candidate["checkpoint"].endswith(
        "/kujiale_0026_visual_heads_run4_v310.pt"
    )
    assert config.phase_d_run4_candidate["posterior_pregate_config"].endswith(
        "/posterior_region_pregate_config_v1.json"
    )
    assert config.phase_d_run4_candidate["startup_initialpose"] == (
        PHASE_D_STARTUP_INITIALPOSE
    )
    assert config.phase_e_run4_candidate["manual_recovery_experiment"] == (
        PHASE_E_MANUAL_RECOVERY_EXPERIMENT
    )


def test_plan_has_no_old_s3_r2_or_60_run_matrix():
    plan = build_plan(load_config(CONFIG))
    assert plan["schema_version"] == CONFIG_SCHEMA
    assert plan["event_schema"] == EVENT_SCHEMA
    assert plan["single_round_arm_count"] == 4
    assert [row["arm"] for row in plan["runs"]] == list(ARMS)
    assert len(plan["runs"]) == 4
    assert not {"S3", "R2"} & {row["arm"] for row in plan["runs"]}
    assert "core_run_count" not in plan
    assert plan["phase_e_run4_candidate_enabled"] is True
    by_arm = {row["arm"]: row for row in plan["runs"]}
    assert by_arm["S0"]["run4_candidate_enabled"] is True
    assert by_arm["S1"]["run4_candidate_enabled"] is True
    assert by_arm["R0"]["run4_candidate_enabled"] is True
    assert by_arm["R1"]["run4_candidate_enabled"] is True
    assert by_arm["R1"]["mode2_recovery"] == PHASE_E_MODE2_RECOVERY
    assert all(
        "mode2_recovery" not in by_arm[arm] for arm in ("S0", "S1", "R0")
    )
    assert by_arm["S0"]["expected_startup_initialpose"] == {
        "source": "runner",
        "seed_kind": "broad_initialpose",
        "expected_total_count": 1,
        "expected_supervisor_count": 0,
    }
    assert by_arm["S1"]["expected_startup_initialpose"] == {
        "source": "supervisor",
        "seed_kind": "cognitive_prior",
        "expected_total_count": 1,
        "expected_supervisor_count": 1,
    }
    for arm in ("R0", "R1"):
        assert by_arm[arm]["expected_startup_initialpose"] == {
            "source": "runner",
            "seed_kind": "broad_initialpose",
            "expected_total_count": 1,
            "expected_supervisor_count": 0,
        }


def test_whole_house_onebox_variant_freezes_identity_seed_asset_and_actions():
    config = load_config(CONFIG, variant=WHOLE_HOUSE_ONEBOX_VARIANT)
    variant = config.selected_variant
    assert variant is not None
    assert variant.user_arm_aliases == WHOLE_HOUSE_USER_ARM_ALIASES
    assert variant.seed == 8601
    assert variant.obstacle_asset == {
        "config": (
            "module3://isaac_sim/configs/experiments/"
            "v6_kujiale_low_obstacles_frozen.yaml"
        ),
        "id": "v6_low_box_solo",
        "mode": "stationary",
        "position_m": [-0.75, -0.35, 0.08],
        "size_m": [0.30, 0.30, 0.16],
    }
    assert variant.route == ("G2", "F2", "recover", "G3", "G4", "G5", "G1")
    assert variant.fault_pose == localization_causal.SeedPose(
        -2.20, -2.95, -42.0, 0.04, 0.030461742
    )
    assert variant.runtime_identity == {
        "low_obstacles_enabled": True,
        "module2_navigation_write_enabled": True,
        "module2_active_effect_scope": "obstacle_only",
        "cognitive_place_graph_enabled": False,
        "dynamic_actors_enabled": False,
        "route_backend": "gvg",
        "route_prior_enabled": False,
        "cognitive_profile": "M3",
    }

    asset = yaml.safe_load(
        (REPO / "isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen.yaml")
        .read_text(encoding="utf-8")
    )
    assert asset["seed"] == 8601
    assert asset["obstacles"] == [
        {
            "id": "v6_low_box_solo",
            "mode": "stationary",
            "trigger_group": None,
            "size": [0.30, 0.30, 0.16],
            "mass": 5.0,
            "start": [-0.75, -0.35, 0.08],
            "end": [-0.75, -0.35, 0.08],
            "speed": 0.0,
            "delay_sec": 0.0,
            "jitter_sec": 0.0,
            "post_motion": "hold",
        }
    ]

    plan = build_plan(config)
    assert plan["single_round_arm_count"] == 2
    assert [(row["user_arm"], row["arm"], row["seed"]) for row in plan["runs"]] == [
        ("W0", "R0", 8601),
        ("W1", "R1", 8601),
    ]
    assert plan["runs"][0]["actions"] == list(route_actions(config, "R0"))
    assert plan["runs"][1]["actions"] == list(route_actions(config, "R1"))
    assert plan["runs"][0]["actions"][2] == {
        "action": "recover",
        "method": "amcl_no_cognitive_write",
    }
    assert plan["runs"][1]["actions"][2] == {
        "action": "recover",
        "method": "supervisor_manual_rescue",
    }


def test_config_rejects_non_startup_run4_candidate_status(tmp_path):
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    raw["phase_d_run4_candidate"]["status"] = "SHADOW_ONLY"
    path = tmp_path / "not_startup_allowed.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(LocalizationConfigError, match="startup-only status"):
        load_config(path)


def _write_candidate_manifest(
    path: Path, *, status: str = RUN4_CANDIDATE_STATUS
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "status": status,
                "recovery_qualification": "NOT_ACTIVE_RECOVERY_QUALIFIED",
                "default_enabled": False,
                "allowed_supervisor_modes": ["shadow", "startup"],
                "manual_recovery_experiment": PHASE_E_MANUAL_RECOVERY_EXPERIMENT,
            }
        ),
        encoding="utf-8",
    )


def _run_wrapper(
    tmp_path: Path,
    arm: str,
    component: str,
    *,
    status: str = RUN4_CANDIDATE_STATUS,
) -> subprocess.CompletedProcess[str]:
    integration = tmp_path / "integration"
    module2 = tmp_path / "module2"
    manifest = integration / (
        "ros2_ws/src/bio_nav_ros_bridge/config/"
        "kujiale_0026_run4_read_only_shadow_candidate.json"
    )
    _write_candidate_manifest(manifest, status=status)
    env = os.environ.copy()
    env.update(
        {
            "BIO_NAV_INTEGRATION_ROOT": str(integration),
            "BIO_NAV_MODULE2_V310_ROOT": str(module2),
            "BIO_NAV_PHASE_DE_RUN4_CANDIDATE_MANIFEST": str(manifest),
            "BIO_NAV_PHASE_DE_SOCKET_PATH": str(tmp_path / "module2.sock"),
        }
    )
    return subprocess.run(
        [
            "bash",
            str(WRAPPER),
            "--dry-run",
            "--module2-asset-root",
            str(tmp_path / "module2-assets"),
            "--arm",
            arm,
            component,
        ],
        capture_output=True,
        text=True,
        env=env,
        check=False,
    )


def test_s0_s1_argv_share_run4_server_manifest_and_keep_seed_modes(tmp_path):
    outputs = {
        (arm, component): _run_wrapper(tmp_path, arm, component)
        for arm in ("S0", "S1")
        for component in ("module1", "bridge")
    }
    assert all(result.returncode == 0 for result in outputs.values())
    manifest = tmp_path / (
        "integration/ros2_ws/src/bio_nav_ros_bridge/config/"
        "kujiale_0026_run4_read_only_shadow_candidate.json"
    )
    for arm in ("S0", "S1"):
        server = outputs[(arm, "module1")].stdout
        bridge = outputs[(arm, "bridge")].stdout
        assert f"--candidate-manifest {manifest}" in server
        argv = shlex.split(server)
        assert argv.count("--module2-asset-root") == 1
        assert argv[argv.index("--module2-asset-root") + 1] == str(
            tmp_path / "module2-assets"
        )
        assert f"localization_candidate_manifest:={manifest}" in bridge
    assert "localization_supervisor_mode:=shadow" in outputs[("S0", "bridge")].stdout
    assert "localization_supervisor_mode:=startup" in outputs[("S1", "bridge")].stdout


def test_module1_requires_asset_root_but_bridge_does_not(tmp_path):
    missing = subprocess.run(
        ["bash", str(WRAPPER), "--dry-run", "--arm", "S0", "module1"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert missing.returncode == 2
    assert "--module2-asset-root is required for module1" in missing.stderr

    bridge = subprocess.run(
        ["bash", str(WRAPPER), "--dry-run", "--arm", "S0", "bridge"],
        capture_output=True,
        text=True,
        check=False,
    )
    assert "--module2-asset-root is required" not in bridge.stderr


def test_all_arms_ros_wait_for_their_later_seed(tmp_path):
    for arm in ARMS:
        result = _run_wrapper(tmp_path, arm, "ros")
        assert result.returncode == 0, result.stderr
        assert "initial_pose_source:=rviz" in result.stdout
        assert "activation_startup_policy:=wait_for_seed" in result.stdout


@pytest.mark.parametrize(
    ("arm", "expected_seed_count"),
    (("S0", 1), ("S1", 0), ("R0", 1), ("R1", 1)),
)
def test_runner_preserves_frozen_startup_seed_ownership(
    monkeypatch, arm, expected_seed_count
):
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter.config = load_config(CONFIG)
    adapter.arm = arm
    adapter.phase = "D"
    adapter.episode = SimpleNamespace(
        seed=1,
        dynamic_case_id="none",
        variant_id="baseline",
        reset_pose_name="long_route_start_g1",
    )
    stops = []
    adapter.guard = SimpleNamespace(
        state="READY",
        reset_events=1,
        localization_ready=True,
        nav2_active=True,
        tf_active=True,
        goal_ready=True,
        arm_reset=lambda _facts: None,
        record_reset_call=lambda: None,
        record_reset_response=lambda _success: None,
        record_reset_receipt_generation=lambda _generation: None,
    )
    adapter.guard.stop = lambda reason: stops.append(reason)
    adapter.facts = object()
    adapter._types = {"Trigger": SimpleNamespace(Request=lambda: object())}
    response = SimpleNamespace(success=True, message="unused by focused fake")
    future = SimpleNamespace(done=lambda: True, result=lambda: response)
    adapter.reset_client = SimpleNamespace(call_async=lambda _request: future)
    adapter._amcl_count = 0
    adapter._initialpose_count = 0
    adapter._assert_ground_truth_firewall = lambda: None
    adapter._pre_reset_ready = lambda: True
    adapter._readiness_blockers = lambda: ""
    adapter._set_episode_parameters = lambda _timeout: None
    adapter._emit_episode_start = lambda: None
    adapter._event = lambda *_args, **_kwargs: None
    adapter._check_post_reset_odom = lambda: None
    adapter._wait_nav2_and_tf_ready = lambda _timeout: None
    spin_calls = 0

    def spin_until(predicate, _timeout):
        nonlocal spin_calls
        spin_calls += 1
        if arm == "S1" and spin_calls == 2:
            adapter._initialpose_count = 1
            adapter._amcl_count = 1
        return bool(predicate())

    adapter._spin_until = spin_until
    adapter._amcl_recovered = lambda _baseline: True
    readiness_baselines = []

    def request_stationary_updates(baseline, _timeout):
        readiness_baselines.append(baseline)
        return True

    adapter._request_stationary_amcl_updates = request_stationary_updates
    published = []

    def publish_seed(pose, kind):
        published.append((pose, kind))
        adapter._initialpose_count += 1

    adapter._publish_seed = publish_seed
    monkeypatch.setattr(
        localization_causal,
        "parse_reset_receipt",
        lambda *_args, **_kwargs: {"generation": 1},
    )

    assert adapter._reset_and_localize(
        readiness_timeout_sec=1.0,
        reset_timeout_sec=1.0,
    )
    assert len(published) == expected_seed_count
    if arm != "S1":
        assert published == [(adapter.config.broad_seed, "broad_initialpose")]
    else:
        assert published == []
    assert readiness_baselines == [0]
    assert stops == []


def test_phase_d_stationary_readiness_requests_only_missing_amcl_poses():
    class EmptyService:
        class Request:
            pass

    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter._types = {"EmptyService": EmptyService}
    adapter.guard = SimpleNamespace(state="READY", stop=lambda _reason: None)
    adapter._amcl_count = 8  # the observed post-seed pose above baseline 7
    adapter._nomotion_request_count = 0
    calls = []

    class Client:
        def wait_for_service(self, timeout_sec):
            calls.append(("wait", timeout_sec))
            return True

        def call_async(self, _request):
            calls.append(("call", None))
            adapter._amcl_count += 1
            return SimpleNamespace(done=lambda: True, result=lambda: object())

    adapter.nomotion_update_client = Client()
    adapter._spin_until = lambda predicate, _timeout: bool(predicate())

    assert adapter._request_stationary_amcl_updates(7, 30.0)
    assert adapter._amcl_count == 7 + STARTUP_AMCL_POSES_REQUIRED
    assert [kind for kind, _ in calls].count("call") == 2


def test_phase_d_stationary_readiness_fails_closed_when_nomotion_unavailable():
    class EmptyService:
        class Request:
            pass

    stops = []
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter._types = {"EmptyService": EmptyService}
    adapter.guard = SimpleNamespace(state="READY", stop=stops.append)
    adapter._amcl_count = 1
    adapter._nomotion_request_count = 0
    adapter.nomotion_update_client = SimpleNamespace(
        wait_for_service=lambda **_kwargs: False
    )

    assert not adapter._request_stationary_amcl_updates(0, 30.0)
    assert stops == ["service_unavailable:/request_nomotion_update"]


@pytest.mark.parametrize("arm", ("S0", "S1"))
def test_phase_d_events_use_the_runner_sim_clock(monkeypatch, tmp_path, arm):
    captured = []
    ros_clock_type = object()
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter.node = SimpleNamespace(
        get_parameter=lambda _name: SimpleNamespace(value=True),
        get_clock=lambda: SimpleNamespace(
            clock_type=ros_clock_type,
            now=lambda: SimpleNamespace(nanoseconds=12_345_000_000)
        )
    )
    adapter._ros_clock_type = ros_clock_type
    adapter.output_jsonl = tmp_path / f"{arm}.jsonl"
    adapter.run_id = f"phase-d-{arm.lower()}"
    adapter.phase = "D"
    adapter.arm = arm
    adapter.episode = SimpleNamespace(seed=8810)
    adapter._event_stream_started = True
    monkeypatch.setattr(
        localization_causal,
        "append_evidence_jsonl",
        lambda path, event, **payload: captured.append((path, event, payload)),
    )

    adapter._event("estimated_pose", x=0.45, y=-5.35)

    assert captured[0][2]["stamp_s"] == pytest.approx(12.345)
    assert captured[0][2]["phase"] == "D"
    assert captured[0][2]["arm"] == arm


def test_localization_runner_forces_use_sim_time_true():
    ros_clock_type = object()

    class Parameter:
        def __init__(self, name, *, value):
            self.name = name
            self.value = value

    class Node:
        def __init__(self):
            self.value = False

        def set_parameters(self, parameters):
            assert [(item.name, item.value) for item in parameters] == [
                ("use_sim_time", True)
            ]
            self.value = True
            return [SimpleNamespace(successful=True)]

        def get_parameter(self, name):
            assert name == "use_sim_time"
            return SimpleNamespace(value=self.value)

        def get_clock(self):
            return SimpleNamespace(clock_type=ros_clock_type)

    node = Node()
    _require_sim_time(node, Parameter, ros_clock_type)
    assert node.value is True


def test_localization_event_fails_closed_if_sim_time_authority_is_lost(tmp_path):
    ros_clock_type = object()
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter.node = SimpleNamespace(
        get_parameter=lambda _name: SimpleNamespace(value=False),
        get_clock=lambda: SimpleNamespace(
            clock_type=ros_clock_type,
            now=lambda: SimpleNamespace(nanoseconds=12_345_000_000),
        ),
    )
    adapter._ros_clock_type = ros_clock_type
    adapter.output_jsonl = tmp_path / "lost-clock.jsonl"
    adapter.run_id = "phase-d-s0"
    adapter.phase = "D"
    adapter.arm = "S0"
    adapter.episode = SimpleNamespace(seed=8810)
    adapter._event_stream_started = True

    with pytest.raises(localization_causal.V6ContractError, match="lost use_sim_time"):
        adapter._event("estimated_pose", x=0.45, y=-5.35)


def test_r0_r1_share_run4_server_manifest_with_shadow_and_manual_active(tmp_path):
    outputs = {
        (arm, component): _run_wrapper(tmp_path, arm, component)
        for arm in ("R0", "R1")
        for component in ("module1", "bridge")
    }
    assert all(result.returncode == 0 for result in outputs.values())
    manifest = tmp_path / (
        "integration/ros2_ws/src/bio_nav_ros_bridge/config/"
        "kujiale_0026_run4_read_only_shadow_candidate.json"
    )
    for arm in ("R0", "R1"):
        assert f"--candidate-manifest {manifest}" in outputs[(arm, "module1")].stdout
        assert (
            f"localization_candidate_manifest:={manifest}"
            in outputs[(arm, "bridge")].stdout
        )
    assert "localization_supervisor_mode:=shadow" in outputs[("R0", "bridge")].stdout
    assert "localization_supervisor_mode:=active" in outputs[("R1", "bridge")].stdout


def test_w0_w1_wrapper_maps_to_r0_r1_with_onebox_m3_argv(tmp_path):
    outputs = {
        (arm, component): _run_wrapper(tmp_path, arm, component)
        for arm in ("W0", "W1")
        for component in ("isaac", "ros", "module1", "bridge", "plan")
    }
    assert all(result.returncode == 0 for result in outputs.values())
    manifest = tmp_path / (
        "integration/ros2_ws/src/bio_nav_ros_bridge/config/"
        "kujiale_0026_run4_read_only_shadow_candidate.json"
    )
    for arm in ("W0", "W1"):
        assert "run_v6_kujiale_low_obstacles.sh isaac" in outputs[(arm, "isaac")].stdout
        ros = outputs[(arm, "ros")].stdout
        assert "run_v6_kujiale_low_obstacles.sh ros M3" in ros
        assert "route_prior_enabled:=false" in ros
        assert "initial_pose_source:=rviz" in ros
        assert "activation_startup_policy:=wait_for_seed" in ros
        module1 = outputs[(arm, "module1")].stdout
        assert "run_v6_module2_causal_obstacle_server.sh" in module1
        assert "--startup-profile module2_causal_obstacle_active" in module1
        assert "--active-effect-scope obstacle_only" in module1
        assert f"--candidate-manifest {manifest}" in module1
        assert "--shadow-config" not in module1
        assert "startup_profile:=module2_causal_obstacle_active" in outputs[
            (arm, "bridge")
        ].stdout
        assert f"--variant {WHOLE_HOUSE_ONEBOX_VARIANT}" in outputs[
            (arm, "plan")
        ].stdout
    assert "localization_supervisor_mode:=shadow" in outputs[("W0", "bridge")].stdout
    assert "localization_supervisor_mode:=active" in outputs[("W1", "bridge")].stdout

    w0_module1 = shlex.split(outputs[("W0", "module1")].stdout)
    w1_module1 = shlex.split(outputs[("W1", "module1")].stdout)
    assert w0_module1 == w1_module1
    assert w0_module1[w0_module1.index("--startup-profile") + 1] == (
        "module2_causal_obstacle_active"
    )
    assert w0_module1[w0_module1.index("--active-effect-scope") + 1] == (
        "obstacle_only"
    )
    assert w0_module1[w0_module1.index("--module2-root") + 1] == str(
        tmp_path / "module2"
    )
    assert w0_module1[w0_module1.index("--candidate-manifest") + 1] == str(
        manifest
    )
    assert w0_module1[w0_module1.index("--socket") + 1] == str(
        tmp_path / "module2.sock"
    )
    assert w0_module1[w0_module1.index("--device") + 1] == "cuda"

    default_r0 = _run_wrapper(tmp_path, "R0", "ros")
    assert default_r0.returncode == 0
    assert "run_v6_r5_phase_b_kujiale.sh" in default_r0.stdout
    assert "run_v6_kujiale_low_obstacles.sh" not in default_r0.stdout


def test_wrapper_fails_closed_when_run4_manifest_is_not_startup_allowed(tmp_path):
    result = _run_wrapper(tmp_path, "S1", "module1", status="SHADOW_ONLY")
    assert result.returncode != 0
    assert "not startup-allowed" in result.stderr


def test_phase_d_actions_are_one_ordinary_full_route():
    config = load_config(CONFIG)
    for arm in ("S0", "S1"):
        assert route_actions(config, arm) == tuple(
            {"action": "goal", "leg_id": leg}
            for leg in ("G2", "G3", "G4", "G5", "G1")
        )


def test_phase_e_actions_put_only_f2_after_g2_then_continue_from_g3():
    config = load_config(CONFIG)
    for arm, method in (
        ("R0", "amcl_no_cognitive_write"),
        ("R1", "supervisor_manual_rescue"),
    ):
        actions = route_actions(config, arm)
        assert actions[0] == {"action": "goal", "leg_id": "G2"}
        assert actions[1] == {
            "action": "fault",
            "fault_id": "F2",
            "kind": "amcl_global_localization_particle_spread",
            "service": "/reinitialize_global_localization",
        }
        assert actions[2] == {"action": "recover", "method": method}
        assert [row["leg_id"] for row in actions[3:]] == ["G3", "G4", "G5", "G1"]
        assert sum(row["action"] == "fault" for row in actions) == 1
    variant_fault = route_actions(load_config(CONFIG, variant=WHOLE_HOUSE_ONEBOX_VARIANT), "R0")[1]
    assert (variant_fault["kind"], variant_fault["service"]) == ("deterministic_initialpose", None)


def test_whole_house_recovery_rejects_wrong_small_covariance_until_anchor():
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter.config = load_config(CONFIG, variant=WHOLE_HOUSE_ONEBOX_VARIANT)
    adapter._amcl_count = STARTUP_AMCL_POSES_REQUIRED + 3
    adapter._last_amcl_covariance = (0.01, 0.01, 0.01)
    adapter._fault_anchor_pose = (5.0, 0.0, 0.0)
    adapter._last_amcl_pose = (-2.20, -2.95, -42.0)
    assert not adapter._fault_recovered(3)
    adapter._last_amcl_pose = (5.0, 0.0, 42.0)
    assert adapter._fault_recovered(3)


class _FakeAdapter:
    def __init__(self):
        self.actions = []

    def perform_action(self, action):
        self.actions.append(dict(action))


def test_fake_adapter_executes_frozen_actions_in_order():
    config = load_config(CONFIG)
    fake = _FakeAdapter()
    expected = route_actions(config, "R1")
    execute_route_actions(fake, expected)
    assert fake.actions == list(expected)


def test_planning_prior_uses_dominant_mode_fields_directly():
    class Message:
        dominant_mode_root_state_id = 184
        dominant_mode_mass = 0.72
        dominant_mode_covariance_m2 = [0.1, 0.01, 0.01, 0.2]
        place_entropy_normalized = 0.3
        visual_reliability = 0.9
        visual_ood_probability = 0.05

    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    events = []
    adapter._event = lambda event, **payload: events.append((event, payload))
    adapter._planning_prior(Message())
    assert events == [
        (
            "module1_diagnostic",
            {
                "name": "planning_prior",
                "region_id": 184,
                "entropy": 0.3,
                "reliability": 0.9,
                "ood_probability": 0.05,
                "dominant_mass": 0.72,
                "dominant_covariance_m2": [0.1, 0.01, 0.01, 0.2],
                "values": {
                    "region_id": 184,
                    "entropy": 0.3,
                    "reliability": 0.9,
                    "ood_probability": 0.05,
                    "dominant_mass": 0.72,
                    "dominant_covariance_m2": [0.1, 0.01, 0.01, 0.2],
                },
            },
        )
    ]


@pytest.mark.parametrize("cloud_after_pose", (False, True))
def test_r0_89m_fault_is_discriminative_with_late_or_missing_best_effort_cloud(
    cloud_after_pose,
):
    class EmptyService:
        class Request:
            pass

    calls = []
    events = []
    stops = []
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter.config = load_config(CONFIG)
    adapter.arm = "R0"
    adapter._types = {"EmptyService": EmptyService}
    adapter.guard = SimpleNamespace(
        state="LEG_SUCCEEDED",
        goal_publications=1,
        completed_leg_ids=["G2"],
        stop=stops.append,
    )
    adapter._supervisor_initialpose_count = 0
    adapter._last_supervisor = {
        "state": "NORMAL",
        "reason": "amcl_healthy",
        "reset_attempts": "0",
    }
    adapter._last_cmd_zero = True
    adapter._cmd_vel_sim_zero_since = 0.0
    adapter._fault_service_request_count = 0
    adapter._amcl_count = 3
    adapter._particle_cloud_count = 4
    adapter._last_amcl_pose = (0.0, 0.0, 0.0)
    adapter._last_module1_odom_pose = (5.0, -2.0, 10.0)
    adapter._event = lambda event, **payload: events.append((event, payload))
    adapter.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=5_000_000_000)
        )
    )
    future = SimpleNamespace(done=lambda: True, result=lambda: object())

    class Client:
        def wait_for_service(self, timeout_sec):
            return timeout_sec == 10.0

        def call_async(self, _request):
            calls.append("call")
            return future

    adapter.reinitialize_global_localization_client = Client()
    spins = 0

    def spin_until(predicate, _timeout):
        nonlocal spins
        spins += 1
        if spins == 3:
            adapter._amcl_count += 1
            adapter._first_post_fault_amcl_covariance = (0.1, 0.1, 0.1)
            adapter._first_post_fault_amcl_pose = (8.928, 0.0, 101.22)
            adapter._last_module1_odom_pose = (5.0017, -2.0, 11.63)
        elif spins == 4 and cloud_after_pose:
            adapter._particle_cloud_count += 1
        return bool(predicate())

    adapter._spin_until = spin_until
    adapter._fault()

    assert calls == ["call"]
    assert stops == []
    fault = next(payload for event, payload in events if event == "fault_injected")
    assert fault["kind"] == "amcl_global_localization_particle_spread"
    assert fault["service"] == "/reinitialize_global_localization"
    assert fault["service_request_count"] == 1
    assert fault["fault_initialpose_count"] == 0
    assert fault["first_post_fault_particle_cloud_observed"] is cloud_after_pose
    assert fault["outcome"] == "FAULT_DISCRIMINATIVE"
    assert fault["amcl_jump_observed"] is True
    assert fault["supervisor_lost_observed"] is False
    assert fault["amcl_disagreement_position_m"] > 8.9
    assert fault["amcl_disagreement_yaw_deg"] == pytest.approx(99.59)
    assert fault["seed_confirmation_position_threshold_m"] == 0.75
    assert fault["seed_confirmation_yaw_threshold_deg"] == 20.0


def test_whole_house_fault_publishes_one_tagged_initialpose_and_no_service(
    monkeypatch,
):
    monkeypatch.setattr(
        localization_causal.V6FormalNode, "_initialpose", lambda *_args: None
    )
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    events, published, stops = [], [], []
    pose_message = lambda: SimpleNamespace(
        header=SimpleNamespace(frame_id="", stamp=None),
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
            covariance=[0.0] * 36,
        ),
    )
    now = SimpleNamespace(
        nanoseconds=5_000_000_000,
        to_msg=lambda: SimpleNamespace(sec=5, nanosec=0),
    )
    adapter.__dict__.update(
        config=load_config(CONFIG, variant=WHOLE_HOUSE_ONEBOX_VARIANT),
        arm="R0",
        _types={"PoseWithCovarianceStamped": pose_message},
        guard=SimpleNamespace(
            state="LEG_SUCCEEDED", goal_publications=1,
            completed_leg_ids=["G2"], stop=stops.append,
        ),
        _supervisor_initialpose_count=0,
        _last_supervisor={
            "state": "NORMAL", "reason": "amcl_healthy", "reset_attempts": "0"
        },
        _last_cmd_zero=True, _cmd_vel_sim_zero_since=0.0,
        _fault_service_request_count=0, _fault_initialpose_count=0,
        _initialpose_source_queue=localization_causal.deque(),
        _initialpose_count=0, _prior_write_count=0,
        _amcl_count=3, _particle_cloud_count=4,
        _last_amcl_pose=(5.0, 5.0, 0.0),
        _last_module1_odom_pose=(0.0, 0.0, 0.0),
        _event=lambda event, **payload: events.append((event, payload)),
        node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: now)),
    )
    adapter.initialpose_publisher = SimpleNamespace(
        publish=lambda message: (published.append(message), adapter._initialpose(message))
    )
    spins = 0

    def spin_until(predicate, _timeout):
        nonlocal spins
        spins += 1
        if spins == 2:
            adapter._amcl_count += 1
            adapter._first_post_fault_amcl_covariance = (0.04, 0.04, 0.030461742)
            adapter._first_post_fault_amcl_pose = (-2.20, -2.95, -42.0)
        elif spins == 3:
            adapter._particle_cloud_count += 1
        return bool(predicate())

    adapter._spin_until = spin_until
    adapter._fault()
    adapter._fault()

    assert len(published) == 1
    message = published[0]
    yaw_deg = math.degrees(2.0 * math.atan2(message.pose.pose.orientation.z, message.pose.pose.orientation.w))
    assert (
        message.pose.pose.position.x, message.pose.pose.position.y, yaw_deg,
        message.pose.covariance[0], message.pose.covariance[7], message.pose.covariance[35],
    ) == pytest.approx((-2.20, -2.95, -42.0, 0.04, 0.04, 0.030461742))
    assert (adapter._fault_initialpose_count, adapter._fault_service_request_count) == (1, 0)
    initialpose = next(payload for event, payload in events if event == "initialpose")
    assert (initialpose["source"], initialpose["seed_kind"]) == (
        "fault_injector",
        "deterministic_fault",
    )
    fault = next(payload for event, payload in events if event == "fault_injected")
    assert (fault["kind"], fault["service"], fault["service_request_count"], fault["fault_initialpose_count"]) == ("deterministic_initialpose", None, 0, 1)
    assert fault["injected_pose_distance_m"] == pytest.approx(0.0)
    assert fault["outcome"] == "FAULT_DISCRIMINATIVE"
    assert stops == ["F2_fault_service_retry_forbidden"]


def test_small_jump_is_invalid_and_yaw_disagreement_wraps():
    predicted, delta = _propagate_module1_odom_delta(
        (1.0, 2.0, 179.0),
        (4.0, 5.0, 179.0),
        (4.0, 5.0, -179.0),
    )
    position_m, yaw_deg = _pose_disagreement((1.1, 2.0, -177.0), predicted)

    assert delta[2] == pytest.approx(2.0)
    assert predicted[2] == pytest.approx(-179.0)
    assert position_m == pytest.approx(0.1)
    assert yaw_deg == pytest.approx(2.0)
    assert not (
        position_m > SEED_CONFIRMATION_POSITION_THRESHOLD_M
        or yaw_deg > SEED_CONFIRMATION_YAW_THRESHOLD_DEG
    )


def test_fault_without_strict_post_service_amcl_pose_stops():
    class EmptyService:
        class Request:
            pass

    stops = []
    events = []
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter.config = load_config(CONFIG)
    adapter.arm = "R0"
    adapter._types = {"EmptyService": EmptyService}
    adapter.guard = SimpleNamespace(
        state="LEG_SUCCEEDED",
        goal_publications=1,
        completed_leg_ids=["G2"],
        stop=stops.append,
    )
    adapter._supervisor_initialpose_count = 0
    adapter._last_supervisor = {
        "state": "NORMAL",
        "reason": "amcl_healthy",
        "reset_attempts": "0",
    }
    adapter._last_cmd_zero = True
    adapter._cmd_vel_sim_zero_since = 0.0
    adapter._fault_service_request_count = 0
    adapter._amcl_count = 3
    adapter._particle_cloud_count = 4
    adapter._last_amcl_pose = (0.0, 0.0, 0.0)
    adapter._last_module1_odom_pose = (0.0, 0.0, 0.0)
    adapter._event = lambda event, **payload: events.append((event, payload))
    adapter.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=5_000_000_000)
        )
    )
    future = SimpleNamespace(done=lambda: True, result=lambda: object())
    adapter.reinitialize_global_localization_client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: True,
        call_async=lambda _request: future,
    )
    spins = 0

    def spin_until(predicate, _timeout):
        nonlocal spins
        spins += 1
        return bool(predicate()) if spins < 3 else False

    adapter._spin_until = spin_until
    adapter._fault()

    assert stops == ["F2_first_post_fault_amcl_pose_timeout"]
    assert not [event for event, _ in events if event == "fault_injected"]


def test_r1_mode2_gate_requires_post_request_revalidation_and_advanced_floors():
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter._fault_stamp_ns = 5_000
    adapter._manual_request_stamp_ns = 6_000
    adapter._manual_request_diagnostic_floor = {
        "candidate_array_last_validation_stamp_ns": 4_000,
        "candidate_array_received_count": 10,
        "candidate_array_accepted_count": 8,
        "candidate_array_last_sequence": 20,
        "candidate_array_publish_count": 0,
    }
    adapter._last_supervisor = {
        "candidate_validation": "recovery_stationary_revalidated",
        "candidate_array_last_validation_stamp_ns": "7000",
        "candidate_array_received_count": "11",
        "candidate_array_accepted_count": "9",
        "candidate_array_last_sequence": "21",
        "candidate_array_last_candidate_count": "2",
        "candidate_array_last_structural_rejection": "",
        "candidate_array_last_state_machine_decision_reason": "manual_rescue",
        "candidate_array_last_event_reason": "manual_rescue",
        "candidate_array_publish_count": "1",
    }

    assert adapter._post_request_mode2_candidate()
    adapter._last_supervisor["candidate_array_last_sequence"] = "20"
    assert not adapter._post_request_mode2_candidate()


def test_r1_recovery_publishes_one_manual_topic_request_and_gets_one_supervisor_seed():
    class Empty:
        pass

    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter.config = load_config(CONFIG)
    adapter._types = {"Empty": Empty}
    adapter.guard = SimpleNamespace(state="LEG_SUCCEEDED", stop=pytest.fail)
    adapter._manual_rescue_count = 0
    adapter._supervisor_initialpose_count = 0
    adapter._amcl_count = 3
    adapter._last_amcl_covariance = (0.1, 0.1, 0.1)
    adapter._last_supervisor = {
        "state": "LOST",
        "reason": "amcl_covariance_lost",
        "recovery_result": "observed",
        "reset_attempts": "0",
        "candidate_array_publish_count": "0",
        "candidate_validation": "cached",
        "candidate_array_last_validation_stamp_ns": "4000",
        "candidate_array_received_count": "10",
        "candidate_array_accepted_count": "8",
        "candidate_array_last_sequence": "20",
    }
    adapter._fault_stamp_ns = 5_000
    adapter._fault_service_request_count = 1
    adapter._nomotion_request_count = 0
    adapter._fault_observation_active = True
    adapter.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=6_000)
        )
    )
    events = []
    adapter._event = lambda event, **payload: events.append((event, payload))

    def publish(_message):
        adapter._supervisor_initialpose_count += 1
        adapter._amcl_count += STARTUP_AMCL_POSES_REQUIRED
        adapter._last_supervisor.update(
            {
                "state": "RECOVERED",
                "reason": "manual_seed_confirmed",
                "recovery_result": "seed_confirmed",
                "candidate_validation": "recovery_stationary_revalidated",
                "candidate_array_last_validation_stamp_ns": "7000",
                "candidate_array_received_count": "11",
                "candidate_array_accepted_count": "9",
                "candidate_array_last_sequence": "21",
                "candidate_array_last_candidate_count": "2",
                "candidate_array_last_structural_rejection": "",
                "candidate_array_last_state_machine_decision_reason": (
                    "manual_rescue"
                ),
                "candidate_array_last_event_reason": "manual_rescue",
                "candidate_array_publish_count": "1",
            }
        )

    adapter.manual_rescue_publisher = SimpleNamespace(publish=publish)
    adapter._spin_until = lambda predicate, _timeout: bool(predicate())

    adapter._recover("supervisor_manual_rescue")

    assert adapter._manual_rescue_count == 1
    assert adapter._supervisor_initialpose_count == 1
    assert [event for event, _ in events] == [
        "manual_rescue_requested",
        "localization_recovered",
    ]
    request = events[0][1]
    assert request["request_stamp_ns"] == 6_000
    assert request["fault_stamp_ns"] == 5_000
    assert request["diagnostic_floors"]["candidate_array_last_sequence"] == 20
    assert request["diagnostic_floors"]["candidate_array_publish_count"] == 0


def test_r1_cached_pre_request_candidate_cannot_satisfy_mode2_and_times_out():
    class Empty:
        pass

    stops = []
    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter.config = load_config(CONFIG)
    adapter._types = {"Empty": Empty}
    adapter.guard = SimpleNamespace(state="LEG_SUCCEEDED", stop=stops.append)
    adapter._manual_rescue_count = 0
    adapter._supervisor_initialpose_count = 0
    adapter._amcl_count = 3
    adapter._last_amcl_covariance = (0.1, 0.1, 0.1)
    adapter._last_supervisor = {
        "state": "LOST",
        "reason": "amcl_covariance_lost",
        "recovery_result": "observed",
        "reset_attempts": "0",
        "candidate_array_publish_count": "0",
        "candidate_validation": "cached",
        "candidate_array_last_validation_stamp_ns": "4000",
        "candidate_array_received_count": "10",
        "candidate_array_accepted_count": "8",
        "candidate_array_last_sequence": "20",
        "candidate_array_last_candidate_count": "2",
        "candidate_array_last_structural_rejection": "",
        "candidate_array_last_state_machine_decision_reason": (
            "no_authorized_rescue_request"
        ),
        "candidate_array_last_event_reason": "no_authorized_rescue_request",
    }
    adapter._fault_stamp_ns = 5_000
    adapter._fault_service_request_count = 1
    adapter._nomotion_request_count = 0
    adapter._fault_observation_active = True
    adapter.node = SimpleNamespace(
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=6_000)
        )
    )
    adapter._event = lambda _event, **_payload: None

    def publish(_message):
        adapter._supervisor_initialpose_count += 1

    adapter.manual_rescue_publisher = SimpleNamespace(publish=publish)
    adapter._spin_until = lambda predicate, _timeout: bool(predicate())

    adapter._recover("supervisor_manual_rescue")

    assert adapter._manual_rescue_count == 1
    assert stops == ["post_request_mode2_candidate_timeout"]


def test_event_schema_is_small_gt_free_and_has_evaluator_fields():
    assert "ground_truth_pose" not in ALLOWED_EVENTS
    assert not [event for event in ALLOWED_EVENTS if "ground_truth" in event]
    assert {
        "episode_start",
        "initialpose",
        "fault_injected",
        "particle_cloud",
        "manual_rescue_requested",
        "pause_requested",
        "pause_confirmed",
        "prior_write",
        "localization_ready",
        "localization_recovered",
        "goal_dispatched",
        "goal_result",
        "supervisor_diagnostic",
        "estimated_pose",
        "odom_pose",
        "cmd_vel_sim",
        "collision",
        "module1_diagnostic",
        "episode_end",
    } == ALLOWED_EVENTS
    assert _contains_ground_truth({"nested": {"ground_truth_pose": {}}})
    assert _contains_ground_truth({"source": "/ground_truth/odom"})
    assert not _contains_ground_truth({"passive_evaluator_only": True})


def test_invalid_fault_service_or_kind_is_rejected(tmp_path):
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    raw["fault"]["service"] = "/global_localization"
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(LocalizationConfigError, match="reinitialize_global_localization"):
        load_config(path)

    raw["fault"]["service"] = "/reinitialize_global_localization"
    raw["fault"]["kind"] = "wrong_region_initialpose"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(LocalizationConfigError, match="particle spread"):
        load_config(path)


def test_paired_arms_must_keep_the_same_seed(tmp_path):
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    raw["seeds"]["S1"] += 1
    path = tmp_path / "unpaired.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(LocalizationConfigError, match="paired"):
        load_config(path)


def test_cli_config_and_plan_are_non_live(capsys):
    assert cli(["config", "--config", str(CONFIG)]) == 0
    config_output = json.loads(capsys.readouterr().out)
    assert config_output["arms"] == list(ARMS)
    assert config_output["formal_qualification"] == "NOT_QUALIFIED"

    assert cli(["plan", "--config", str(CONFIG)]) == 0
    plan_output = json.loads(capsys.readouterr().out)
    assert len(plan_output["runs"]) == 4
