import json
import os
from pathlib import Path
import subprocess

import pytest
import yaml

from robot_experiments.v6_localization_causal import (
    ALLOWED_EVENTS,
    ARMS,
    CONFIG_SCHEMA,
    EVENT_SCHEMA,
    PHASE_D_STARTUP_INITIALPOSE,
    RUN4_CANDIDATE_STATUS,
    LocalizationCausalNode,
    LocalizationConfigError,
    _contains_ground_truth,
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
    assert tuple(config.seeds) == ARMS
    assert config.route_ids == ("G2", "G3", "G4", "G5", "G1")
    assert config.fault_id == "F2"
    assert config.fault_leg_id == "G3"
    assert config.fault_min_arc_length_m == pytest.approx(1.0)
    assert (
        config.wrong_region_seed.x,
        config.wrong_region_seed.y,
        config.wrong_region_seed.yaw_deg,
    ) == pytest.approx((-2.20, -2.95, -42.0))
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


def test_plan_has_no_old_s3_r2_or_60_run_matrix():
    plan = build_plan(load_config(CONFIG))
    assert plan["schema_version"] == CONFIG_SCHEMA
    assert plan["event_schema"] == EVENT_SCHEMA
    assert plan["single_round_arm_count"] == 4
    assert [row["arm"] for row in plan["runs"]] == list(ARMS)
    assert len(plan["runs"]) == 4
    assert not {"S3", "R2"} & {row["arm"] for row in plan["runs"]}
    assert "core_run_count" not in plan
    assert plan["phase_e_run4_candidate_enabled"] is False
    by_arm = {row["arm"]: row for row in plan["runs"]}
    assert by_arm["S0"]["run4_candidate_enabled"] is True
    assert by_arm["S1"]["run4_candidate_enabled"] is True
    assert by_arm["R0"]["run4_candidate_enabled"] is False
    assert by_arm["R1"]["run4_candidate_enabled"] is False
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
    assert by_arm["R0"]["expected_startup_initialpose"] is None
    assert by_arm["R1"]["expected_startup_initialpose"] is None


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
        assert f"localization_candidate_manifest:={manifest}" in bridge
    assert "localization_supervisor_mode:=shadow" in outputs[("S0", "bridge")].stdout
    assert "localization_supervisor_mode:=startup" in outputs[("S1", "bridge")].stdout


def test_r0_r1_argv_keep_old_path_without_run4_candidate(tmp_path):
    r0 = _run_wrapper(tmp_path, "R0", "module1")
    r1 = _run_wrapper(tmp_path, "R1", "bridge")
    assert r0.returncode == 0
    assert r1.returncode == 0
    assert "module1-shadow" in r0.stdout
    assert "localization_supervisor_mode:=active" in r1.stdout
    assert "candidate-manifest" not in r0.stdout
    assert "localization_candidate_manifest" not in r1.stdout


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
        ("R0", "global_localization"),
        ("R1", "supervisor_manual_rescue"),
    ):
        actions = route_actions(config, arm)
        assert actions[0] == {"action": "goal", "leg_id": "G2"}
        assert actions[1] == {
            "action": "fault_leg",
            "leg_id": "G3",
            "fault_id": "F2",
            "min_arc_length_m": 1.0,
        }
        assert actions[2] == {"action": "recover", "method": method}
        assert [row["leg_id"] for row in actions[3:]] == ["G3", "G4", "G5", "G1"]
        assert sum(row["action"] == "fault_leg" for row in actions) == 1


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


def test_fault_preview_cancel_binds_zero_settle_to_completed_cancel_future():
    class Response:
        return_code = 0

    class Future:
        def done(self):
            return True

        def result(self):
            return Response()

    future = Future()

    class Client:
        def wait_for_service(self, timeout_sec):
            return timeout_sec == 2.0

        def call_async(self, request):
            return future

    class CancelGoal:
        class Request:
            pass

    adapter = LocalizationCausalNode.__new__(LocalizationCausalNode)
    adapter.config = load_config(CONFIG)
    adapter._fault_arc_length_m = 1.0
    adapter.navigate_cancel_client = Client()
    adapter._types = {"CancelGoal": CancelGoal}
    adapter._event = lambda *_args, **_kwargs: None
    adapter._start_terminal_settle = lambda **_kwargs: None
    adapter._settle_terminal_zero = lambda: True
    adapter._terminal_cancel_requested = False
    adapter._terminal_cancel_future = None
    adapter._fault_cancel_future = None
    assert adapter._cancel_fault_preview("F2") == (True, True)
    assert adapter._terminal_cancel_requested is True
    assert adapter._terminal_cancel_future is future


def test_event_schema_is_small_gt_free_and_has_evaluator_fields():
    assert "ground_truth_pose" not in ALLOWED_EVENTS
    assert not [event for event in ALLOWED_EVENTS if "ground_truth" in event]
    assert {
        "episode_start",
        "initialpose",
        "fault_injected",
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


def test_invalid_fault_pose_or_arc_is_rejected(tmp_path):
    raw = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    raw["fault"]["min_arc_length_m"] = 0.99
    path = tmp_path / "bad.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(LocalizationConfigError, match=">= 1.0"):
        load_config(path)

    raw["fault"]["min_arc_length_m"] = 1.0
    raw["fault"]["wrong_region_seed"]["x"] = -2.19
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(LocalizationConfigError, match="G5"):
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
