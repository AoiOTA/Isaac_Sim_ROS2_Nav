from collections import deque
from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest
import robot_experiments.v6_formal as v6_formal_module
import yaml
from robot_experiments.scenario import load_scenario

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
    evaluate_formal_campaign,
    execute_formal_campaign,
    formal_dispatch_plan,
    load_formal_campaign_manifest,
    load_manifest,
    validate_condition_stack_contract,
)


PACKAGE = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
CONFIG = PACKAGE / "config"
MANIFEST = CONFIG / "v6_r3_phase2_kujiale_baseline.yaml"
PHASE_B_MANIFEST = CONFIG / "v6_r5_phase_b_kujiale_exact_baseline.yaml"
LEGACY_MANIFESTS = tuple(
    CONFIG / (
        f"v6_final_{world}_{category}.yaml"
        if world == "kujiale"
        else f"final_{world}_{category}.yaml"
    )
    for world in ("kujiale", "rivermark")
    for category in ("static", "dynamic", "appearance")
)


def _raw() -> dict:
    return yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))


def _write_manifest(tmp_path: Path, raw: dict) -> Path:
    path = tmp_path / "manifest.yaml"
    path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    return path


def _formal_raw(tmp_path: Path, *, authorization: str = "NOT_AUTHORIZED") -> dict:
    rows = []
    for scene, world in (("indoor", "kujiale"), ("outdoor", "rivermark")):
        for category in ("static", "dynamic", "appearance"):
            scenario = (
                CONFIG / f"v6_final_{world}_{category}.yaml"
                if world == "kujiale"
                else CONFIG / f"final_{world}_{category}.yaml"
            )
            rows.append({
                "id": f"{scene}_{category}",
                "scene": scene,
                "category": category,
                "scenario_file": str(scenario),
                "output_directory": str(tmp_path / f"{scene}-{category}"),
                "runner_arguments": [
                    "nav2_profile:=v6_low_obstacle_isolation",
                    "navigation_execution_backend:=route_guided",
                    "require_module2_planning_ready:=true",
                ],
            })
    by_id = {row["id"]: row for row in rows}
    ordered = [by_id[name] for name in v6_formal_module.FORMAL_CONDITION_IDS]
    def file_entry(path: Path) -> dict[str, str]:
        path = path.resolve()
        return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}

    scenario_entries = {}
    scenario_configs = {}
    for row in ordered:
        scenario_path = Path(row["scenario_file"])
        scenario_entries[row["id"]] = file_entry(scenario_path)
        scenario = load_scenario(scenario_path)
        config_paths = {
            scenario.resolve_path(path)
            for path in (
                scenario.robot_config_file,
                scenario.nav2_config_file,
                scenario.dynamic_config_file,
                scenario.appearance_config_file,
                scenario.optimal_reference_file,
            )
            if path is not None
        }
        scenario_configs[row["id"]] = [
            file_entry(path) for path in sorted(config_paths)
        ]
    repository_paths = {
        "integration": Path(
            "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
            "bio_nav_integration"
        ),
        "module2": Path(
            "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
            "bio_nav_module2"
        ),
        "module3": Path(
            "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
            "bio_nav_module3"
        ),
    }
    repositories = {
        name: {
            "path": str(path.resolve()),
            "head": subprocess.run(
                ["git", "-C", str(path), "rev-parse", "HEAD"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip(),
        }
        for name, path in repository_paths.items()
    }
    runner_entrypoint = REPO / "scripts" / "run_experiment.sh"
    frozen_assets = {}
    for name in sorted(v6_formal_module.FORMAL_FROZEN_ASSET_KEYS):
        if name == "rivermark_catalog_constraints_tree":
            path = tmp_path / "frozen-assets" / name
            path.mkdir(parents=True, exist_ok=True)
            (path / "region_02.json").write_text("{}\n", encoding="utf-8")
            frozen_assets[name] = {
                "path": str(path.resolve()),
                "sha256": v6_formal_module._constraints_tree_sha256(path),
            }
            continue
        path = tmp_path / "frozen-assets" / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(f"{name}\n", encoding="utf-8")
        frozen_assets[name] = file_entry(path)
    return {
        "schema_version": "bio_nav_v6_formal_campaign_v1",
        "intended_use": "formal_qualification",
        "execution_authorization": authorization,
        "runs_per_condition": 20,
        "runner_entrypoint": str(runner_entrypoint),
        "freeze": {
            "repositories": repositories,
            "driver_version": v6_formal_module._current_driver_version(),
            "kernel_release": os.uname().release,
            "scenarios": scenario_entries,
            "scenario_configs": scenario_configs,
            "frozen_assets": frozen_assets,
            "runner_entrypoint": file_entry(runner_entrypoint),
            "experiment_runner": file_entry(
                PACKAGE / "robot_experiments" / "experiment_runner.py"
            ),
            "v6_formal": file_entry(PACKAGE / "robot_experiments" / "v6_formal.py"),
        },
        "conditions": ordered,
    }


def _write_formal_manifest(
    tmp_path: Path, *, authorization: str = "NOT_AUTHORIZED"
) -> Path:
    path = tmp_path / "formal.yaml"
    path.write_text(
        yaml.safe_dump(_formal_raw(tmp_path, authorization=authorization)),
        encoding="utf-8",
    )
    return path


def _write_formal_run(
    condition,
    run_index: int,
    *,
    strict_success: bool,
    formal_freeze_digest: str,
    valid: bool = True,
    stack_session_id: str = "a" * 64,
) -> Path:
    identity = condition.episode_identities[run_index - 1]
    seed = identity["seed"]
    root = (
        condition.output_directory
        / condition.scenario_id
        / f"run-{run_index:04d}-seed-{seed}"
    )
    root.mkdir(parents=True)
    summary = {
        "strict_success": strict_success,
        "checksums_verified": True,
        "episode_validity": {"valid": valid},
        "final_trial_metric_gate": {"passed": True},
        "condition_stack_id": condition.condition_id,
        "stack_session_id": stack_session_id,
        "formal_freeze_digest": formal_freeze_digest,
    }
    episode = {
        "scenario_id": condition.scenario_id,
        "run_index": run_index,
        "random_seed": seed,
        "condition_id": identity["condition_id"],
        "dynamic_selection": {
            "case_id": identity["dynamic_case_id"],
            "variant_id": identity["dynamic_variant_id"],
        },
        "appearance": {"profile_id": identity["appearance_profile_id"]},
        "condition_stack_id": condition.condition_id,
        "stack_session_id": stack_session_id,
        "formal_freeze_digest": formal_freeze_digest,
        "reset_receipt": {"generation": run_index},
    }
    telemetry = root / "telemetry"
    telemetry.mkdir()
    (telemetry / "metadata.yaml").write_text("metadata\n", encoding="utf-8")
    (telemetry / "telemetry_0.mcap").write_bytes(b"mcap")
    (root / "run_summary.json").write_text(json.dumps(summary), encoding="utf-8")
    (root / "run_manifest.json").write_text(json.dumps(episode), encoding="utf-8")
    _refresh_checksums(root)
    return root


def _refresh_checksums(root: Path) -> None:
    entries = [
        f"{hashlib.sha256(item.read_bytes()).hexdigest()}  {item.relative_to(root)}"
        for item in sorted(
            path
            for path in root.rglob("*")
            if path.is_file() and path.name != "checksums.sha256"
        )
    ]
    (root / "checksums.sha256").write_text(
        "\n".join(entries) + "\n", encoding="utf-8"
    )


def _live_stack_contract(
    tmp_path: Path,
    *,
    condition_id: str = "indoor_static",
    pid: int | None = None,
    **overrides,
) -> Path:
    pid = os.getpid() if pid is None else pid
    stat = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    scene, condition = condition_id.split("_", 1)
    payload = {
        "schema": "bio_nav.v6_stack_contract.v1",
        "condition_id": condition_id,
        "scene": scene,
        "condition": condition,
        "arm": "M3",
        "domain": 150,
        "startup_profile": "module2_causal_obstacle_active",
        "pid": pid,
        "pgid": int(stat[2]),
        "start_ticks": int(stat[19]),
        "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "integration_head": subprocess.run(
            [
                "git", "-C",
                "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
                "bio_nav_integration",
                "rev-parse", "HEAD",
            ],
            check=True, capture_output=True, text=True,
        ).stdout.strip(),
        "module2_head": subprocess.run(
            [
                "git", "-C",
                "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
                "bio_nav_module2",
                "rev-parse", "HEAD",
            ],
            check=True, capture_output=True, text=True,
        ).stdout.strip(),
        "module3_head": subprocess.run(
            [
                "git", "-C",
                "/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/"
                "bio_nav_module3",
                "rev-parse", "HEAD",
            ],
            check=True, capture_output=True, text=True,
        ).stdout.strip(),
        "driver_version": v6_formal_module._current_driver_version(),
        "kernel_release": os.uname().release,
    }
    payload.update(overrides)
    payload["stack_session_id"] = v6_formal_module._stack_session_id(payload)
    path = tmp_path / "stack.contract.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
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


def test_formal_manifest_dry_run_freezes_six_conditions_and_120_runs(
    tmp_path, capsys
):
    path = _write_formal_manifest(tmp_path)

    assert cli(["--formal-manifest", str(path)]) == 0
    payload = json.loads(capsys.readouterr().out)

    assert payload["execution_authorization"] == "NOT_AUTHORIZED"
    assert payload["dispatch"] is False
    assert payload["aggregate"]["expected_episodes"] == 120
    assert payload["aggregate"]["present_episodes"] == 0
    assert payload["aggregate"]["strict_successes"] == 0
    assert [
        row["id"] for row in payload["aggregate"]["conditions"]
    ] == list(v6_formal_module.FORMAL_CONDITION_IDS)
    assert set(payload["resume_points"].values()) == {1}
    assert len(payload["dispatch_plan"]) == 6
    assert all("run_indices:=1" in row["command"] for row in payload["dispatch_plan"])
    assert all(
        row["command"][0] == str(REPO / "scripts" / "run_experiment.sh")
        and row["condition_stack_contract_required"] is True
        for row in payload["dispatch_plan"]
    )
    assert all(
        row["stack_boundary"] == "cold"
        and row["requires_existing_condition_stack"] is True
        for row in payload["dispatch_plan"]
    )

    campaign = load_formal_campaign_manifest(path)
    dynamic = next(
        condition
        for condition in campaign.conditions
        if condition.condition_id == "indoor_dynamic"
    )
    assert len(dynamic.episode_identities) == 20
    assert len({row["seed"] for row in dynamic.episode_identities}) == 4
    assert len({
        (
            row["seed"],
            row["dynamic_case_id"],
            row["dynamic_variant_id"],
        )
        for row in dynamic.episode_identities
    }) == 20


def test_formal_manifest_requires_source_runner_and_route_prior_contract(tmp_path):
    raw = _formal_raw(tmp_path)
    raw["runner_entrypoint"] = str(tmp_path / "missing-runner")
    with pytest.raises(V6ContractError, match="runner_entrypoint"):
        load_formal_campaign_manifest(_write_manifest(tmp_path, raw))

    for argument, message in (
        ("navigation_execution_backend:=navigate_to_pose", "route_guided"),
        ("require_module2_planning_ready:=false", "planning readiness"),
    ):
        raw = _formal_raw(tmp_path)
        name = argument.split(":=", 1)[0]
        raw["conditions"][0]["runner_arguments"] = [
            argument if item.startswith(f"{name}:=") else item
            for item in raw["conditions"][0]["runner_arguments"]
        ]
        with pytest.raises(V6ContractError, match=message):
            load_formal_campaign_manifest(_write_manifest(tmp_path, raw))


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("repo_head", "repository head mismatch"),
        ("driver", "driver_version mismatch"),
        ("file_hash", "sha256 mismatch"),
        ("asset_keys", "freeze.frozen_assets keys"),
    ],
)
def test_formal_freeze_rejects_tuple_or_file_drift(tmp_path, mutation, message):
    raw = _formal_raw(tmp_path)
    if mutation == "repo_head":
        raw["freeze"]["repositories"]["module3"]["head"] = "0" * 40
    elif mutation == "driver":
        raw["freeze"]["driver_version"] = "stale-driver"
    elif mutation == "file_hash":
        raw["freeze"]["v6_formal"]["sha256"] = "0" * 64
    else:
        raw["freeze"]["frozen_assets"].pop("dino_checkpoint")

    with pytest.raises(V6ContractError, match=message):
        load_formal_campaign_manifest(_write_manifest(tmp_path, raw))


def test_formal_freeze_rejects_rivermark_constraint_tree_drift(tmp_path):
    raw = _formal_raw(tmp_path)
    tree = Path(
        raw["freeze"]["frozen_assets"]["rivermark_catalog_constraints_tree"]["path"]
    )
    (tree / "region_02.json").write_text('{"changed": true}\n', encoding="utf-8")

    with pytest.raises(V6ContractError, match="sha256 mismatch"):
        load_formal_campaign_manifest(_write_manifest(tmp_path, raw))


def test_formal_execution_requires_flag_and_authorized_manifest(tmp_path, capsys):
    path = _write_formal_manifest(tmp_path)

    assert cli(["--formal-manifest", str(path), "--execute-formal"]) == 2
    assert "requires --condition-stack-id" in capsys.readouterr().err

    assert cli([
        "--formal-manifest",
        str(path),
        "--execute-formal",
        "--condition-stack-id",
        "indoor_static",
        "--condition-stack-contract",
        "/missing/stack.contract.json",
    ]) == 2
    assert "manifest is NOT_AUTHORIZED" in capsys.readouterr().err

    assert cli(["--manifest", str(MANIFEST), "--execute-formal"]) == 2
    assert "require --formal-manifest" in capsys.readouterr().err


def test_formal_execution_rejects_wrong_stack_without_subprocess(
    tmp_path, monkeypatch
):
    campaign = load_formal_campaign_manifest(
        _write_formal_manifest(tmp_path, authorization="AUTHORIZED")
    )
    calls = []
    monkeypatch.setattr(
        v6_formal_module.subprocess,
        "run",
        lambda *args, **kwargs: calls.append((args, kwargs)),
    )

    with pytest.raises(V6ContractError, match="unknown formal condition stack"):
        execute_formal_campaign(
            campaign,
            condition_stack_id="wrong_stack",
            condition_stack_contract="/missing/stack.contract.json",
        )

    assert calls == []


@pytest.mark.parametrize(
    ("override", "message"),
    [
        ({"integration_head": "0" * 40}, "repository head mismatch"),
        ({"driver_version": "stale-driver"}, "system freeze mismatch"),
    ],
)
def test_formal_execution_rejects_stack_freeze_mismatch(
    tmp_path, monkeypatch, override, message
):
    campaign = load_formal_campaign_manifest(
        _write_formal_manifest(tmp_path, authorization="AUTHORIZED")
    )
    contract = _live_stack_contract(tmp_path, **override)
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    monkeypatch.setattr(
        v6_formal_module.subprocess,
        "run",
        lambda *args, **kwargs: pytest.fail("subprocess must not run"),
    )

    with pytest.raises(V6ContractError, match=message):
        execute_formal_campaign(
            campaign,
            condition_stack_id="indoor_static",
            condition_stack_contract=contract,
        )


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"boot_id": "stale-boot"}, "boot_id is stale"),
        ({"start_ticks": 1}, "start_ticks is stale"),
    ],
)
def test_stack_contract_rejects_stale_process_identity(
    tmp_path, overrides, message
):
    contract = _live_stack_contract(tmp_path, **overrides)

    with pytest.raises(V6ContractError, match=message):
        validate_condition_stack_contract(
            contract, expected_condition_id="indoor_static"
        )


def test_stack_contract_rejects_wrong_condition(tmp_path):
    contract = _live_stack_contract(tmp_path)

    with pytest.raises(V6ContractError, match="condition mismatch"):
        validate_condition_stack_contract(
            contract, expected_condition_id="outdoor_static"
        )


def test_stack_contract_rejects_dead_process(tmp_path):
    process = subprocess.Popen(["sleep", "30"])
    contract = _live_stack_contract(tmp_path, pid=process.pid)
    process.terminate()
    process.wait(timeout=5)

    with pytest.raises(V6ContractError, match="process is not live"):
        validate_condition_stack_contract(
            contract, expected_condition_id="indoor_static"
        )


def test_formal_execution_dispatches_one_episode_and_returns(
    tmp_path, monkeypatch
):
    campaign = load_formal_campaign_manifest(
        _write_formal_manifest(tmp_path, authorization="AUTHORIZED")
    )
    contract = _live_stack_contract(tmp_path)
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    contract_payload = json.loads(contract.read_text())
    calls = []

    def fake_run(command, *, check):
        calls.append((command, check))
        _write_formal_run(
            campaign.conditions[0],
            1,
            strict_success=True,
            formal_freeze_digest=campaign.freeze_digest,
            stack_session_id=contract_payload["stack_session_id"],
        )

    monkeypatch.setattr(v6_formal_module.subprocess, "run", fake_run)

    aggregate = execute_formal_campaign(
        campaign,
        condition_stack_id="indoor_static",
        condition_stack_contract=contract,
    )

    assert len(calls) == 1
    assert calls[0][1] is True
    assert "run_indices:=1" in calls[0][0]
    assert f"condition_stack_id:=indoor_static" in calls[0][0]
    assert any(
        argument == f"stack_session_id:={contract_payload['stack_session_id']}"
        for argument in calls[0][0]
    )
    assert f"formal_freeze_digest:={campaign.freeze_digest}" in calls[0][0]
    assert aggregate["present_episodes"] == 1


def test_formal_execution_requires_exactly_one_new_strict_target(
    tmp_path, monkeypatch
):
    campaign = load_formal_campaign_manifest(
        _write_formal_manifest(tmp_path, authorization="AUTHORIZED")
    )
    contract = _live_stack_contract(tmp_path)
    monkeypatch.setenv("ROS_DOMAIN_ID", "150")
    monkeypatch.setattr(v6_formal_module.subprocess, "run", lambda *args, **kwargs: None)

    with pytest.raises(V6ContractError, match="exactly one strict-success"):
        execute_formal_campaign(
            campaign,
            condition_stack_id="indoor_static",
            condition_stack_contract=contract,
        )


def test_formal_shell_requires_and_forwards_condition_stack_id():
    source = (REPO / "scripts" / "run_v6_formal_episode.sh").read_text()
    assert "formal execution requires stack ID and contract path" in source
    assert '--condition-stack-contract "$5"' in source


def test_formal_aggregate_resumes_after_valid_strict_episode(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    _write_formal_run(
        first, 1, strict_success=True, formal_freeze_digest=campaign.freeze_digest
    )

    aggregate = evaluate_formal_campaign(campaign)
    plans = formal_dispatch_plan(campaign, aggregate)

    first_result = aggregate["conditions"][0]
    assert first_result["strict_successes"] == 1
    assert first_result["valid_episodes"] == 1
    assert first_result["next_run_index"] == 2
    assert plans[0]["condition_id"] == first.condition_id
    assert plans[0]["run_index"] == 2
    assert plans[0]["stack_boundary"] == "hot_reset"
    assert plans[0]["stack_session"] == first.condition_id


@pytest.mark.parametrize(
    ("strict_success", "valid", "expected_status"),
    [
        (False, True, "product_failure"),
        (False, False, "invalid_evidence"),
    ],
)
def test_formal_aggregate_stops_at_product_or_evidence_failure(
    tmp_path, strict_success, valid, expected_status
):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    _write_formal_run(
        first,
        1,
        strict_success=strict_success,
        valid=valid,
        formal_freeze_digest=campaign.freeze_digest,
    )

    aggregate = evaluate_formal_campaign(campaign)
    first_result = aggregate["conditions"][0]

    assert first_result["runs"][0]["status"] == expected_status
    assert first_result["next_run_index"] is None
    assert aggregate["blockers"] == [
        f"{first.condition_id}:run-1:{expected_status}"
    ]


def test_formal_aggregate_rejects_changed_stack_session(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    _write_formal_run(
        first, 1, strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
        stack_session_id="a" * 64,
    )
    _write_formal_run(
        first, 2, strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
        stack_session_id="b" * 64,
    )

    aggregate = evaluate_formal_campaign(campaign)

    assert aggregate["conditions"][0]["stack_session_id"] is None
    assert "indoor_static:stack_session_mismatch" in aggregate["blockers"]


@pytest.mark.parametrize("generation", [None, 1, 4])
def test_formal_aggregate_requires_contiguous_reset_generation(
    tmp_path, generation
):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    _write_formal_run(
        first, 1, strict_success=True, formal_freeze_digest=campaign.freeze_digest
    )
    root = _write_formal_run(
        first, 2, strict_success=True, formal_freeze_digest=campaign.freeze_digest
    )
    manifest_path = root / "run_manifest.json"
    episode = json.loads(manifest_path.read_text())
    if generation is None:
        episode["reset_receipt"] = {}
    else:
        episode["reset_receipt"]["generation"] = generation
    manifest_path.write_text(json.dumps(episode), encoding="utf-8")
    _refresh_checksums(root)

    aggregate = evaluate_formal_campaign(campaign)

    expected = (
        "reset_generation_missing"
        if generation is None
        else "reset_generation_discontinuous"
    )
    assert f"indoor_static:{expected}" in aggregate["blockers"]


def test_formal_checksum_requires_core_and_mcap_coverage(tmp_path):
    root = tmp_path / "run"
    root.mkdir()
    (root / "unrelated.txt").write_text("ok\n", encoding="utf-8")
    _refresh_checksums(root)

    assert not v6_formal_module._checksums_verified(root)


def test_formal_checksum_rejects_unlisted_regular_file(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    root = _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest=campaign.freeze_digest,
    )
    (root / "unlisted.txt").write_text("not in checksum\n", encoding="utf-8")

    assert not v6_formal_module._checksums_verified(root)


def test_formal_aggregate_requires_final_metric_gate(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    first = campaign.conditions[0]
    root = _write_formal_run(
        first, 1, strict_success=True, formal_freeze_digest=campaign.freeze_digest
    )
    summary_path = root / "run_summary.json"
    summary = json.loads(summary_path.read_text())
    summary["final_trial_metric_gate"]["passed"] = False
    summary_path.write_text(json.dumps(summary), encoding="utf-8")
    _refresh_checksums(root)

    aggregate = evaluate_formal_campaign(campaign)

    assert aggregate["conditions"][0]["runs"][0]["status"] == "invalid_evidence"


def test_formal_aggregate_rejects_run_freeze_digest_mismatch(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    _write_formal_run(
        campaign.conditions[0],
        1,
        strict_success=True,
        formal_freeze_digest="0" * 64,
    )

    aggregate = evaluate_formal_campaign(campaign)

    assert aggregate["conditions"][0]["runs"][0]["status"] == "invalid_evidence"


def test_unauthorized_complete_campaign_never_reports_pass(tmp_path):
    campaign = load_formal_campaign_manifest(_write_formal_manifest(tmp_path))
    for condition in campaign.conditions:
        for run_index in range(1, 21):
            _write_formal_run(
                condition,
                run_index,
                strict_success=True,
                formal_freeze_digest=campaign.freeze_digest,
            )

    aggregate = evaluate_formal_campaign(campaign)

    assert aggregate["strict_successes"] == 120
    assert aggregate["execution_authorization"] == "NOT_AUTHORIZED"
    assert aggregate["formal_qualification"] == "INCOMPLETE"
    authorized = evaluate_formal_campaign(
        replace(campaign, authorization="AUTHORIZED")
    )
    assert authorized["formal_qualification"] == "PASS"


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
    adapter._terminal_topic_summary = {}
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


def test_terminal_capture_writer_stall_cannot_starve_zero_observation(monkeypatch):
    adapter, clock, _lifecycle = _terminal_adapter(
        monkeypatch,
        state="SUCCEEDED",
        downstream_events=(
            (0.05, False),
            (0.10, False),
            (0.12, True),
            (0.20, False),
            (0.38, False),
        ),
    )
    adapter._capture = V6FormalNode._capture.__get__(adapter, V6FormalNode)
    writes = []

    def stalled_write(event, **payload):
        writes.append((event, payload, adapter._terminal_zero_confirmed))
        clock.now += 1.0

    adapter._write = stalled_write
    adapter._start_terminal_settle(cancel_navigation=False, reason="SUCCEEDED")

    assert adapter._settle_terminal_zero()
    assert adapter._terminal_zero_confirmed
    summary_event, payload, confirmed_before_stall = writes[0]
    assert summary_event == "terminal_topic_summary"
    assert confirmed_before_stall
    assert payload["topics"]["/cmd_vel_sim"]["count"] == 5
    assert payload["topics"]["/cmd_vel_sim"]["last_message"]["linear"]["x"] == 0.0
    assert writes[-1][0] == "terminal_zero_confirmed"


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
