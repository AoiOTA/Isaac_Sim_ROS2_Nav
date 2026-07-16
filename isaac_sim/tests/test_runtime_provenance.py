from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from isaac_sim.src.runtime_provenance import (
    RuntimeProvenanceError,
    capture_runtime_provenance,
    capture_runtime_provenance_v6_legacy,
    file_sha256,
    git_metadata,
    runtime_provenance_parameters,
    stage_articulation_solver_iterations,
)
from isaac_sim.src.stage.ground_topology import (
    collider_paths_sha256,
    load_ground_topology_profile,
)
from isaac_sim.src.robot.kinematics_config import load_robot_config_contract
from isaac_sim.src.robot.mass_collision_config import (
    load_mass_collision_profile,
)


JACKAL_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "robots"
    / "jackal.yaml"
)
LEGACY_MASS_PROFILE = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "robot_mass_profiles"
    / "legacy_default_sensor_density_v1.yaml"
)
ROOT = Path(__file__).resolve().parents[2]
JACKAL_ASSET = (
    ROOT / "isaac_sim/assets/robots/jackal/jackal_nav.usda"
)
WHEEL_JOINTS = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)
HAS_PXR = importlib.util.find_spec("pxr") is not None


def _write_test_robot_config(path: Path) -> None:
    """Copy the schema-v3 robot contract with a fixture-stable profile path."""

    source = JACKAL_CONFIG.read_text(encoding="utf-8")
    relative_profile = (
        "../robot_mass_profiles/legacy_default_sensor_density_v1.yaml"
    )
    assert relative_profile in source
    path.write_text(
        source.replace(relative_profile, str(LEGACY_MASS_PROFILE)),
        encoding="utf-8",
    )


class _RootLayer:
    def ExportToString(self) -> str:
        return "#usda 1.0\ndef Xform \"World\" {}\n"


class _Stage:
    def __init__(self, solver_iterations: tuple[int, int] = (32, 4)):
        self._prim = _Prim(solver_iterations)

    def GetRootLayer(self) -> _RootLayer:
        return _RootLayer()

    def GetPrimAtPath(self, path: str) -> "_Prim":
        assert path == "/World/Robot"
        return self._prim


class _Attribute:
    def __init__(self, value: int):
        self._value = value

    def __bool__(self) -> bool:
        return True

    def IsValid(self) -> bool:
        return True

    def Get(self) -> int:
        return self._value


class _Prim:
    def __init__(self, solver_iterations: tuple[int, int]):
        self._attributes = {
            "physxArticulation:solverPositionIterationCount": _Attribute(
                solver_iterations[0]
            ),
            "physxArticulation:solverVelocityIterationCount": _Attribute(
                solver_iterations[1]
            ),
        }

    def __bool__(self) -> bool:
        return True

    def IsValid(self) -> bool:
        return True

    def GetAttribute(self, name: str) -> _Attribute:
        return self._attributes[name]


def _contact_snapshot(
    profile_path: Path,
    ground_colliders: list[str] | None = None,
) -> dict[str, object]:
    wheel_colliders = [
        f"/World/Robot/wheel_{index}/collider" for index in range(4)
    ]
    ground_colliders = ground_colliders or ["/World/Ground/Collision"]
    wheel_material_path = "/World/Looks/WheelPhysics"
    return {
        "profile_path": str(profile_path.resolve()),
        "profile_sha256": file_sha256(profile_path),
        "profile_id": "legacy-baseline",
        "profile_mode": "legacy_baseline",
        "overlay_identifier": "anon:0x123:contact_legacy-baseline.usda",
        "overlay_sha256": "1" * 64,
        "explicit_materials": False,
        "thresholds_authored": False,
        "scene": {
            "physics_scene_path": "/PhysicsScene",
            "friction_correlation_distance": 0.00025,
            "friction_offset_threshold": 0.0004,
            "friction_type": "patch",
        },
        "wheel_colliders": wheel_colliders,
        "ground_colliders": ground_colliders,
        "wheel_bindings": [
            {
                "collider_path": path,
                "direct_physics_material_path": wheel_material_path,
                "effective_physics_material_path": wheel_material_path,
            }
            for path in wheel_colliders
        ],
        "ground_bindings": [
            {
                "collider_path": path,
                "direct_physics_material_path": None,
                "effective_physics_material_path": None,
            }
            for path in ground_colliders
        ],
        "wheel_material": {
            "material_path": wheel_material_path,
            "static_friction": 0.2,
            "dynamic_friction": 0.2,
            "restitution": 0.0,
            "friction_combine_mode": None,
            "restitution_combine_mode": None,
            "friction_combine_mode_authored": False,
            "restitution_combine_mode_authored": False,
        },
        "ground_material": None,
        "stage_usd_readback_verified": True,
    }


def _write_ground_topology_profile(
    profile_path: Path,
    source_asset: Path,
    source_colliders: list[str],
    target_colliders: list[str],
    *,
    environment_id: str = "Warehouse",
) -> None:
    source_colliders = sorted(source_colliders)
    target_colliders = sorted(target_colliders)
    disabled_colliders = sorted(set(source_colliders) - set(target_colliders))
    profile_id = (
        f"test_{environment_id.lower()}_combined{len(source_colliders)}_v1"
        if source_colliders == target_colliders
        else f"test_{environment_id.lower()}_target{len(target_colliders)}_v1"
    )
    operation = (
        "preserve_source_colliders"
        if source_colliders == target_colliders
        else "disable_non_target_colliders"
    )
    profile_path.write_text(
        "\n".join(
            (
                "schema_version: 1",
                f"id: {profile_id}",
                f"environment_id: {environment_id}",
                f"operation: {operation}",
                "source:",
                f"  asset_sha256: {file_sha256(source_asset)}",
                "  required_prim_paths: "
                + json.dumps(source_colliders, separators=(",", ":")),
                "  semantic_classes: []",
                f"  collider_count: {len(source_colliders)}",
                "  collider_paths_sha256: "
                + collider_paths_sha256(source_colliders),
                "target:",
                "  required_prim_paths: "
                + json.dumps(target_colliders, separators=(",", ":")),
                "  semantic_classes: []",
                f"  collider_count: {len(target_colliders)}",
                "  collider_paths_sha256: "
                + collider_paths_sha256(target_colliders),
                "disabled:",
                f"  collider_count: {len(disabled_colliders)}",
                "  collider_paths_sha256: "
                + collider_paths_sha256(disabled_colliders),
                "",
            )
        ),
        encoding="utf-8",
    )


def _ground_topology_snapshot(
    profile_path: Path,
    source_asset: Path,
    source_colliders: list[str],
    target_colliders: list[str],
) -> dict[str, object]:
    profile = load_ground_topology_profile(profile_path)
    source_colliders = sorted(source_colliders)
    target_colliders = sorted(target_colliders)
    disabled_colliders = sorted(set(source_colliders) - set(target_colliders))
    return {
        "profile_path": str(profile_path.resolve()),
        "profile_sha256": file_sha256(profile_path),
        "profile_id": profile.identifier,
        "environment_id": profile.environment_id,
        "operation": profile.operation,
        "source_asset_path": str(source_asset.resolve()),
        "source_asset_sha256": file_sha256(source_asset),
        "overlay_identifier": "anon:0x456:ground_topology_test.usda",
        "overlay_sha256": "2" * 64,
        "source_colliders": source_colliders,
        "source_collider_count": len(source_colliders),
        "source_collider_paths_sha256": collider_paths_sha256(source_colliders),
        "target_colliders": target_colliders,
        "target_collider_count": len(target_colliders),
        "target_collider_paths_sha256": collider_paths_sha256(target_colliders),
        "disabled_colliders": disabled_colliders,
        "disabled_collider_count": len(disabled_colliders),
        "disabled_collider_paths_sha256": collider_paths_sha256(
            disabled_colliders
        ),
        "stage_usd_readback_verified": True,
    }


def _reset_strategy_snapshot(
    *,
    wheel_joints: tuple[str, ...] = ("fl", "fr", "rl", "rr"),
    wheel_link_paths: list[str] | None = None,
    ground_filter_paths: list[str] | None = None,
    identifier: str = "pose_restore_v1",
) -> dict[str, object]:
    wheel_link_paths = wheel_link_paths or [
        f"/World/Robot/wheel_{index}" for index in range(4)
    ]
    assert len(wheel_link_paths) == len(wheel_joints) == 4
    ground_filter_paths = sorted(
        ground_filter_paths or ["/World/Ground/Collision"]
    )
    if identifier == "pose_restore_v1":
        lift_distance_m = 0.0
        separation_step_count = 0
    else:
        assert identifier == "separate_recontact_0p20m_1step_v1"
        lift_distance_m = 0.2
        separation_step_count = 1
    return {
        "schema_version": 1,
        "id": identifier,
        "lift_distance_m": lift_distance_m,
        "separation_step_count": separation_step_count,
        "recontact_step_count": 1,
        "contact_probe": {
            "schema_version": 1,
            "enabled": True,
            "wheel_bindings": [
                {
                    "joint_name": joint_name,
                    "wheel_link_path": wheel_link_path,
                }
                for joint_name, wheel_link_path in zip(
                    wheel_joints, wheel_link_paths, strict=True
                )
            ],
            "wheel_count": 4,
            "ground_filter_paths": ground_filter_paths,
            "ground_filter_count": len(ground_filter_paths),
            "max_contact_count": 128,
            "report_threshold_n": 0.0,
            "stage_usd_readback_verified": True,
        },
    }


def _wheel_velocity_drive_snapshot(config) -> dict[str, object]:
    contract = load_robot_config_contract(config.files.robot)
    drive = contract.wheel_velocity_drive
    configured_si = {
        "drive_type": drive.drive_type,
        "stiffness_n_m_per_rad": drive.stiffness_n_m_per_rad,
        "damping_n_m_s_per_rad": drive.damping_n_m_s_per_rad,
        "max_effort_n_m": drive.max_effort_n_m,
        "max_joint_velocity_rad_s": drive.max_joint_velocity_rad_s,
    }
    authored_usd = {
        "drive_type": drive.drive_type,
        "stiffness_n_m_per_degree": (
            drive.stiffness_n_m_per_rad * 3.141592653589793 / 180.0
        ),
        "damping_n_m_s_per_degree": (
            drive.damping_n_m_s_per_rad * 3.141592653589793 / 180.0
        ),
        "max_force_n_m": drive.max_effort_n_m,
        "max_joint_velocity_deg_s": (
            drive.max_joint_velocity_rad_s * 180.0 / 3.141592653589793
        ),
    }
    return {
        "schema_version": 1,
        "profile_path": str(Path(config.files.robot).resolve()),
        "profile_sha256": file_sha256(config.files.robot),
        "profile_id": drive.profile_id,
        "configured_si": configured_si,
        "authored_usd": authored_usd,
        "joint_paths": [
            f"{config.robot.runtime_prim_path}/{name}"
            for name in contract.wheel_joints.ordered
        ],
        "overlay_identifier": "anon:0x123:wheel_velocity_drive.usda",
        "overlay_sha256": "3" * 64,
        "stage_usd_readback_verified": True,
    }


def _wheel_drive_tensor_snapshot(
    config,
    stage_snapshot: dict[str, object],
) -> dict[str, object]:
    contract = load_robot_config_contract(config.files.robot)
    drive = contract.wheel_velocity_drive
    return {
        "schema_version": 1,
        "profile_path": stage_snapshot["profile_path"],
        "profile_sha256": stage_snapshot["profile_sha256"],
        "profile_id": stage_snapshot["profile_id"],
        "stage_overlay_sha256": stage_snapshot["overlay_sha256"],
        "dof_names": list(contract.wheel_joints.ordered),
        "dof_indices": [0, 1, 2, 3],
        "drive_types": [drive.drive_type] * 4,
        "stiffnesses_n_m_per_rad": [
            drive.stiffness_n_m_per_rad
        ] * 4,
        "dampings_n_m_s_per_rad": [drive.damping_n_m_s_per_rad] * 4,
        "max_efforts_n_m": [drive.max_effort_n_m] * 4,
        "max_joint_velocities_rad_s": [
            drive.max_joint_velocity_rad_s
        ] * 4,
        "physics_tensor_readback_verified": True,
    }


def _mass_collision_snapshot(config) -> dict[str, object]:
    contract = load_robot_config_contract(config.files.robot)
    profile_path = contract.mass_collision_profile
    profile = load_mass_collision_profile(profile_path)
    articulation_root = config.robot.articulation_root

    def prim_path(suffix: str) -> str:
        return f"{articulation_root}{suffix}"

    base_inertial = None
    if profile.base_inertial is not None:
        base_inertial = {
            "prim_path": config.robot.base_link_prim,
            "mass_kg": profile.base_inertial.mass_kg,
            "center_of_mass_m": list(
                profile.base_inertial.center_of_mass_m
            ),
            "inertia_kg_m2": [
                list(row) for row in profile.base_inertial.inertia_kg_m2
            ],
        }
    return {
        "schema_version": 1,
        "profile": {
            "path": profile_path.relative_to(ROOT).as_posix(),
            "sha256": file_sha256(profile_path),
            "id": profile.profile_id,
            "mode": profile.mode,
        },
        "robot_asset_sha256": file_sha256(config.robot.asset_path),
        "sensor_shells": sorted(
            [
                {
                    "prim_path": prim_path(shell.prim_suffix),
                    "active": shell.active,
                    "collision_enabled": shell.collision_enabled,
                }
                for shell in profile.sensor_shells
            ],
            key=lambda shell: shell["prim_path"],
        ),
        "base_inertial": base_inertial,
        "expected_link_masses": sorted(
            [
                {
                    "prim_path": prim_path(expectation.prim_suffix),
                    "mass_kg": expectation.mass_kg,
                }
                for expectation in profile.expected_link_masses
            ],
            key=lambda expectation: expectation["prim_path"],
        ),
        "expected_total_mass_kg": profile.expected_total_mass_kg,
        "overlay": {
            "id": f"mass_collision_profile/{profile.profile_id}",
            "identifier": "anon:0x456:mass_collision.usda",
            "sha256": "4" * 64,
        },
        "stage_usd_readback_verified": True,
    }


def _mass_tensor_snapshot(
    stage_snapshot: dict[str, object],
) -> dict[str, object]:
    base = stage_snapshot["base_inertial"]
    links = []
    for expectation in stage_snapshot["expected_link_masses"]:
        path = expectation["prim_path"]
        is_base = base is not None and path == base["prim_path"]
        links.append(
            {
                "name": Path(path).name,
                "prim_path": path,
                "mass_kg": expectation["mass_kg"],
                "center_of_mass_m": (
                    base["center_of_mass_m"] if is_base else [0.0, 0.0, 0.0]
                ),
                "inertia_kg_m2": (
                    base["inertia_kg_m2"]
                    if is_base
                    else [
                        [0.01, 0.0, 0.0],
                        [0.0, 0.01, 0.0],
                        [0.0, 0.0, 0.01],
                    ]
                ),
            }
        )
    return {
        "schema_version": 1,
        "profile_id": stage_snapshot["profile"]["id"],
        "links": links,
        "total_mass_kg": stage_snapshot["expected_total_mass_kg"],
        "physics_tensor_readback_verified": True,
    }


def _control_graph_snapshot(
    config,
    *,
    wheel_command_application: str = "split_axle_v1",
) -> dict[str, object]:
    joints = list(config.robot.wheel_joints)
    if wheel_command_application == "split_axle_v1":
        writers = [
            {
                "node": "FrontController",
                "target_prim": config.robot.articulation_root,
                "joint_names": joints[:2],
            },
            {
                "node": "RearController",
                "target_prim": config.robot.articulation_root,
                "joint_names": joints[2:],
            },
        ]
    else:
        writers = [
            {
                "node": "WheelController",
                "target_prim": config.robot.articulation_root,
                "joint_names": joints,
            }
        ]
    nodes = sorted(
        [
            {"name": "OnPhysicsStep", "type_name": "OnPhysicsStep"},
            *[
                {"name": writer["node"], "type_name": "Controller"}
                for writer in writers
            ],
        ],
        key=lambda node: (node["name"], node["type_name"]),
    )
    connections = sorted(
        [
            {
                "source": "OnPhysicsStep.outputs:step",
                "target": f"{writer['node']}.inputs:execIn",
            }
            for writer in writers
        ],
        key=lambda connection: (connection["source"], connection["target"]),
    )
    topology = {
        "graph_path": "/World/Graphs/Control",
        "pipeline_stage": "on_demand",
        "nodes": nodes,
        "connections": connections,
        "command_writers": writers,
    }
    topology_json = json.dumps(
        topology,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )
    return {
        "schema_version": 1,
        "wheel_command_application": wheel_command_application,
        "topology": topology,
        "topology_sha256": hashlib.sha256(
            topology_json.encode("utf-8")
        ).hexdigest(),
        "materialized_readback_verified": True,
    }


def _runtime_v7_evidence(config) -> dict[str, dict[str, object]]:
    wheel_stage = _wheel_velocity_drive_snapshot(config)
    mass_stage = _mass_collision_snapshot(config)
    return {
        "wheel_velocity_drive_snapshot": wheel_stage,
        "wheel_drive_tensor_snapshot": _wheel_drive_tensor_snapshot(
            config, wheel_stage
        ),
        "mass_collision_snapshot": mass_stage,
        "mass_tensor_snapshot": _mass_tensor_snapshot(mass_stage),
        "control_graph_snapshot": _control_graph_snapshot(config),
    }


def _patch_v7_stage_readbacks(
    monkeypatch,
    evidence: dict[str, dict[str, object]],
    fresh_readbacks: list[str] | None = None,
) -> None:
    from isaac_sim.src.robot import (
        mass_collision_runtime,
        wheel_velocity_drive,
    )

    def drive_readback(stage, config):
        if fresh_readbacks is not None:
            fresh_readbacks.append("wheel_velocity_drive")
        return deepcopy(evidence["wheel_velocity_drive_snapshot"])

    def mass_readback(stage, config):
        if fresh_readbacks is not None:
            fresh_readbacks.append("mass_collision")
        return deepcopy(evidence["mass_collision_snapshot"])

    monkeypatch.setattr(
        wheel_velocity_drive,
        "capture_wheel_velocity_drive_snapshot",
        drive_readback,
    )
    monkeypatch.setattr(
        mass_collision_runtime,
        "capture_mass_collision_snapshot",
        mass_readback,
    )


def _git(repository: Path, *arguments: str) -> None:
    subprocess.run(
        ["git", "-C", str(repository), *arguments],
        check=True,
        capture_output=True,
        text=True,
    )


def _repository(tmp_path: Path) -> Path:
    repository = tmp_path / "repository"
    repository.mkdir()
    _git(repository, "init")
    _git(repository, "config", "user.email", "tests@example.com")
    _git(repository, "config", "user.name", "Runtime Provenance Tests")
    tracked = repository / "tracked.txt"
    tracked.write_text("frozen\n", encoding="utf-8")
    _git(repository, "add", "tracked.txt")
    _git(repository, "commit", "-m", "initial")
    return repository


def _runtime_inputs(
    tmp_path: Path,
    *,
    source_colliders: list[str],
    target_colliders: list[str],
) -> tuple[Path, SimpleNamespace, dict[str, object], dict[str, object]]:
    repository = _repository(tmp_path)
    robot_config = tmp_path / "jackal.yaml"
    robot_asset = tmp_path / "jackal_nav.usda"
    project_stage = tmp_path / "navigation_scene.usda"
    source_asset = tmp_path / "warehouse.usd"
    topology_profile = tmp_path / "ground_topology.yaml"
    contact_profile = tmp_path / "legacy_baseline.yaml"
    _write_test_robot_config(robot_config)
    robot_asset.write_bytes(JACKAL_ASSET.read_bytes())
    project_stage.write_bytes(b"#usda 1.0\n")
    source_asset.write_bytes(b"PXR-USDC")
    contact_profile.write_text(
        "schema_version: 1\nid: legacy-baseline\nmode: legacy_baseline\n",
        encoding="utf-8",
    )
    _write_ground_topology_profile(
        topology_profile,
        source_asset,
        source_colliders,
        target_colliders,
    )
    asset_root = tmp_path / "6.0"
    asset_root.mkdir()
    config = SimpleNamespace(
        files=SimpleNamespace(
            robot=robot_config,
            ground_topology_profile=topology_profile,
            contact_profile=contact_profile,
        ),
        robot=SimpleNamespace(
            asset_path=robot_asset,
            articulation_root="/World/Robot",
            runtime_prim_path="/World/Robot",
            base_link_prim="/World/Robot/base_link",
            wheel_joints=WHEEL_JOINTS,
            front_wheel_joints=WHEEL_JOINTS[:2],
            rear_wheel_joints=WHEEL_JOINTS[2:],
        ),
        environment=SimpleNamespace(
            identifier="Warehouse",
            project_stage=project_stage,
            source_asset=source_asset,
            ground_colliders=SimpleNamespace(
                required_prim_paths=tuple(sorted(source_colliders)),
                semantic_classes=(),
                expected_enabled_count=len(source_colliders),
            ),
        ),
        asset_root=asset_root,
        simulation=SimpleNamespace(
            navigation_mode="mapping",
            odometry_mode="ideal",
            physics_hz=60.0,
            expected_physics_scene="/PhysicsScene",
            reset_strategy=SimpleNamespace(
                schema_version=1,
                identifier="pose_restore_v1",
            ),
        ),
    )
    topology_snapshot = _ground_topology_snapshot(
        topology_profile,
        source_asset,
        source_colliders,
        target_colliders,
    )
    contact_snapshot = _contact_snapshot(
        contact_profile,
        sorted(target_colliders),
    )
    return repository, config, topology_snapshot, contact_snapshot


def test_file_sha256_streams_the_exact_required_file(tmp_path):
    source = tmp_path / "input.bin"
    source.write_bytes(b"runtime-input\x00")
    assert file_sha256(source) == hashlib.sha256(source.read_bytes()).hexdigest()


def test_git_metadata_captures_revision_branch_and_dirty_state(tmp_path):
    repository = _repository(tmp_path)
    clean = git_metadata(repository)
    assert len(clean["commit"]) in {40, 64}
    assert clean["branch"]
    assert clean["dirty"] is False

    (repository / "untracked.txt").write_text("dirty\n", encoding="utf-8")
    assert git_metadata(repository)["dirty"] is True


def test_schema_v7_binds_drive_mass_control_and_ros_parameters(
    tmp_path,
    monkeypatch,
):
    ground = ["/World/Ground/Collision"]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=ground,
        target_colliders=ground,
    )
    evidence = _runtime_v7_evidence(config)
    fresh_readbacks: list[str] = []
    _patch_v7_stage_readbacks(monkeypatch, evidence, fresh_readbacks)
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(contact_snapshot),
    )
    evidence = _runtime_v7_evidence(config)
    _patch_v7_stage_readbacks(monkeypatch, evidence)

    provenance = capture_runtime_provenance(
        config,
        _Stage(),
        articulation_usd_solver_iterations=(32, 4),
        repository_root=repository,
        reset_strategy_snapshot=_reset_strategy_snapshot(
            wheel_joints=tuple(config.robot.wheel_joints),
            ground_filter_paths=ground,
        ),
        ground_topology_snapshot=topology_snapshot,
        contact_snapshot=contact_snapshot,
        **evidence,
    )
    parameters = runtime_provenance_parameters(provenance)

    assert fresh_readbacks == ["wheel_velocity_drive", "mass_collision"]
    assert provenance["schema_version"] == 7
    assert set(provenance) == {
        "schema_version",
        "robot",
        "environment",
        "simulation",
        "ground_topology",
        "contact",
        "control_graph",
        "git",
    }
    assert provenance["robot"]["config"] == {
        "schema_version": 3,
        "path": str(config.files.robot),
        "sha256": file_sha256(config.files.robot),
    }
    drive = provenance["robot"]["wheel_velocity_drive"]
    assert set(drive) == {
        "schema_version",
        "profile_path",
        "profile_sha256",
        "profile_id",
        "configured_si",
        "authored_usd",
        "joint_paths",
        "overlay_identifier",
        "overlay_sha256",
        "stage_usd_readback_verified",
        "physics_tensor",
    }
    mass = provenance["robot"]["mass_collision"]
    assert set(mass) == {
        "schema_version",
        "profile_path",
        "profile_sha256",
        "profile_id",
        "profile_mode",
        "robot_asset_sha256",
        "sensor_shells",
        "base_inertial",
        "expected_link_masses",
        "expected_total_mass_kg",
        "overlay_id",
        "overlay_identifier",
        "overlay_sha256",
        "stage_usd_readback_verified",
        "physics_tensor",
    }
    assert provenance["control_graph"] == evidence["control_graph_snapshot"]
    assert parameters["runtime_provenance.schema_version"] == 7
    assert parameters[
        "runtime_provenance.robot.config.schema_version"
    ] == 3
    for key, value in (
        ("robot.wheel_velocity_drive", drive),
        ("robot.mass_collision", mass),
        ("control_graph", provenance["control_graph"]),
    ):
        parameter_prefix = f"runtime_provenance.{key}"
        encoded = json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        assert parameters[f"{parameter_prefix}.json"] == encoded
        assert parameters[f"{parameter_prefix}.sha256"] == hashlib.sha256(
            encoded.encode("utf-8")
        ).hexdigest()


@pytest.mark.parametrize(
    ("component", "field"),
    [
        ("wheel_velocity_drive_snapshot", "overlay_sha256"),
        ("mass_collision_snapshot", "overlay.sha256"),
    ],
)
def test_schema_v7_rejects_stale_supplied_stage_snapshots(
    tmp_path,
    monkeypatch,
    component,
    field,
):
    ground = ["/World/Ground/Collision"]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=ground,
        target_colliders=ground,
    )
    fresh = _runtime_v7_evidence(config)
    supplied = deepcopy(fresh)
    if field == "overlay.sha256":
        supplied[component]["overlay"]["sha256"] = "9" * 64
    else:
        supplied[component][field] = "9" * 64
    _patch_v7_stage_readbacks(monkeypatch, fresh)
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(contact_snapshot),
    )

    with pytest.raises(RuntimeProvenanceError, match="stale or differs"):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            reset_strategy_snapshot=_reset_strategy_snapshot(
                wheel_joints=tuple(config.robot.wheel_joints),
                ground_filter_paths=ground,
            ),
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=contact_snapshot,
            **supplied,
        )


@pytest.mark.parametrize("component", ["drive", "mass"])
def test_schema_v7_rejects_tampered_physics_tensor_evidence(
    tmp_path,
    monkeypatch,
    component,
):
    ground = ["/World/Ground/Collision"]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=ground,
        target_colliders=ground,
    )
    evidence = _runtime_v7_evidence(config)
    _patch_v7_stage_readbacks(monkeypatch, evidence)
    if component == "drive":
        evidence["wheel_drive_tensor_snapshot"]["max_efforts_n_m"][0] *= 0.9
        message = "max_efforts_n_m"
    else:
        evidence["mass_tensor_snapshot"]["links"][0]["mass_kg"] += 1.0
        message = "mass_kg"
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(contact_snapshot),
    )

    with pytest.raises(RuntimeProvenanceError, match=message):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            reset_strategy_snapshot=_reset_strategy_snapshot(
                wheel_joints=tuple(config.robot.wheel_joints),
                ground_filter_paths=ground,
            ),
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=contact_snapshot,
            **evidence,
        )


def test_schema_v7_rejects_control_topology_hash_tampering(
    tmp_path,
    monkeypatch,
):
    ground = ["/World/Ground/Collision"]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=ground,
        target_colliders=ground,
    )
    evidence = _runtime_v7_evidence(config)
    _patch_v7_stage_readbacks(monkeypatch, evidence)
    evidence["control_graph_snapshot"]["topology_sha256"] = "0" * 64
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(contact_snapshot),
    )

    with pytest.raises(RuntimeProvenanceError, match="topology SHA256"):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            reset_strategy_snapshot=_reset_strategy_snapshot(
                wheel_joints=tuple(config.robot.wheel_joints),
                ground_filter_paths=ground,
            ),
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=contact_snapshot,
            **evidence,
        )


def test_schema_v7_rejects_mass_profile_path_not_bound_to_robot_config(
    tmp_path,
    monkeypatch,
):
    ground = ["/World/Ground/Collision"]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=ground,
        target_colliders=ground,
    )
    evidence = _runtime_v7_evidence(config)
    evidence["mass_collision_snapshot"]["profile"]["path"] = (
        "isaac_sim/configs/robot_mass_profiles/not_selected.yaml"
    )
    _patch_v7_stage_readbacks(monkeypatch, evidence)
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(contact_snapshot),
    )

    with pytest.raises(RuntimeProvenanceError, match="profile path"):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            reset_strategy_snapshot=_reset_strategy_snapshot(
                wheel_joints=tuple(config.robot.wheel_joints),
                ground_filter_paths=ground,
            ),
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=contact_snapshot,
            **evidence,
        )


def test_capture_flattens_the_effective_robot_environment_and_stage(
    tmp_path, monkeypatch
):
    repository = _repository(tmp_path)
    robot_config = tmp_path / "jackal.yaml"
    robot_asset = tmp_path / "jackal_nav.usda"
    project_stage = tmp_path / "navigation_scene.usda"
    source_asset = tmp_path / "warehouse.usd"
    ground_topology_profile = tmp_path / "ground_topology.yaml"
    contact_profile = tmp_path / "legacy_baseline.yaml"
    for path, content in (
        (robot_asset, JACKAL_ASSET.read_bytes()),
        (project_stage, b"#usda 1.0\n"),
        (source_asset, b"PXR-USDC"),
        (
            contact_profile,
            b"schema_version: 1\nid: legacy-baseline\nmode: legacy_baseline\n",
        ),
    ):
        path.write_bytes(content)
    _write_test_robot_config(robot_config)
    ground_colliders = ["/World/Ground/Collision"]
    _write_ground_topology_profile(
        ground_topology_profile,
        source_asset,
        ground_colliders,
        ground_colliders,
    )
    asset_root = tmp_path / "6.0"
    asset_root.mkdir()
    config = SimpleNamespace(
        files=SimpleNamespace(
            robot=robot_config,
            ground_topology_profile=ground_topology_profile,
            contact_profile=contact_profile,
        ),
        robot=SimpleNamespace(
            asset_path=robot_asset,
            articulation_root="/World/Robot",
            runtime_prim_path="/World/Robot",
            base_link_prim="/World/Robot/base_link",
            wheel_joints=WHEEL_JOINTS,
            front_wheel_joints=WHEEL_JOINTS[:2],
            rear_wheel_joints=WHEEL_JOINTS[2:],
        ),
        environment=SimpleNamespace(
            identifier="Warehouse",
            project_stage=project_stage,
            source_asset=source_asset,
            ground_colliders=SimpleNamespace(
                required_prim_paths=("/World/Ground/Collision",),
                semantic_classes=(),
                expected_enabled_count=1,
            ),
        ),
        asset_root=asset_root,
        simulation=SimpleNamespace(
            navigation_mode="mapping",
            odometry_mode="ideal",
            physics_hz=60.0,
            expected_physics_scene="/PhysicsScene",
            reset_strategy=SimpleNamespace(
                schema_version=1,
                identifier="pose_restore_v1",
            ),
        ),
    )
    from isaac_sim.src.stage import contact_setup
    from isaac_sim.src.stage import ground_topology

    expected_contact_snapshot = _contact_snapshot(
        contact_profile,
        ground_colliders,
    )
    expected_ground_topology_snapshot = _ground_topology_snapshot(
        ground_topology_profile,
        source_asset,
        ground_colliders,
        ground_colliders,
    )
    fresh_readbacks = []
    evidence = _runtime_v7_evidence(config)
    _patch_v7_stage_readbacks(monkeypatch, evidence, fresh_readbacks)
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, config: (
            fresh_readbacks.append("contact")
            or deepcopy(expected_contact_snapshot)
        ),
    )
    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, config: (
            fresh_readbacks.append("ground_topology")
            or deepcopy(expected_ground_topology_snapshot)
        ),
    )
    provenance = capture_runtime_provenance(
        config,
        _Stage(),
        articulation_usd_solver_iterations=(32, 4),
        repository_root=repository,
        reset_strategy_snapshot=_reset_strategy_snapshot(
            wheel_joints=tuple(config.robot.wheel_joints),
        ),
        ground_topology_snapshot=expected_ground_topology_snapshot,
        contact_snapshot=expected_contact_snapshot,
        **evidence,
    )
    parameters = runtime_provenance_parameters(provenance)

    assert fresh_readbacks == [
        "ground_topology",
        "contact",
        "wheel_velocity_drive",
        "mass_collision",
    ]
    assert provenance["schema_version"] == 7
    assert parameters["runtime_provenance.schema_version"] == 7
    assert provenance["robot"]["config"]["sha256"] == file_sha256(
        robot_config
    )
    assert provenance["robot"]["solver"] == {
        "position_iterations": 32,
        "velocity_iterations": 4,
        "stage_articulation_usd_readback_verified": True,
    }
    assert provenance["robot"]["kinematics"] == {
        "profile_id": "jackal_legacy_geometric_v1",
        "lifecycle": "stable_baseline",
        "wheel_radius_m": 0.098,
        "wheel_width_m": 0.040,
        "geometric_track_width_m": 0.37559,
        "effective_track_width_m": 0.37559,
        "controller_contract_verified": True,
    }
    for name, value in provenance["robot"]["kinematics"].items():
        assert (
            parameters[f"runtime_provenance.robot.kinematics.{name}"]
            == value
        )
    assert provenance["environment"]["id"] == "Warehouse"
    assert (
        parameters["runtime_provenance.environment.id"] == "Warehouse"
    )
    assert (
        parameters[
            "runtime_provenance.robot.solver."
            "stage_articulation_usd_readback_verified"
        ]
        is True
    )
    assert provenance["environment"]["asset_version"] == "6.0"
    assert parameters["runtime_provenance.robot.asset.path"] == str(
        robot_asset
    )
    assert (
        parameters[
            "runtime_provenance.environment.composed_root_layer_sha256"
        ]
        == hashlib.sha256(_RootLayer().ExportToString().encode()).hexdigest()
    )
    assert parameters["runtime_provenance.git.dirty"] is False
    topology_json = parameters["runtime_provenance.ground_topology.json"]
    assert isinstance(topology_json, str)
    assert topology_json == json.dumps(
        provenance["ground_topology"],
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    assert parameters[
        "runtime_provenance.ground_topology.sha256"
    ] == hashlib.sha256(topology_json.encode("utf-8")).hexdigest()
    contact_json = parameters["runtime_provenance.contact.json"]
    assert isinstance(contact_json, str)
    assert contact_json == json.dumps(
        provenance["contact"],
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    assert parameters["runtime_provenance.contact.sha256"] == hashlib.sha256(
        contact_json.encode("utf-8")
    ).hexdigest()
    reset_strategy_json = parameters[
        "runtime_provenance.simulation.reset_strategy.json"
    ]
    assert isinstance(reset_strategy_json, str)
    assert reset_strategy_json == json.dumps(
        provenance["simulation"]["reset_strategy"],
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    assert parameters[
        "runtime_provenance.simulation.reset_strategy.sha256"
    ] == hashlib.sha256(reset_strategy_json.encode("utf-8")).hexdigest()
    assert provenance["contact"]["collider_contract"] == {
        "wheel_joint_names": list(WHEEL_JOINTS),
        "wheel_expected_count": 4,
        "ground_required_prim_paths": ["/World/Ground/Collision"],
        "ground_semantic_classes": [],
        "ground_expected_enabled_count": 1,
    }


@pytest.mark.parametrize("target_count", [32, 1])
def test_schema_v7_locks_reset_strategy_and_ground_topologies(
    tmp_path,
    monkeypatch,
    target_count,
):
    plane = "/Root/GroundPlane/CollisionPlane"
    source_colliders = [plane] + [
        f"/Root/Decals/Floor_{index:02d}/Collision"
        for index in range(31)
    ]
    target_colliders = source_colliders if target_count == 32 else [plane]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=source_colliders,
        target_colliders=target_colliders,
    )
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(contact_snapshot),
    )
    evidence = _runtime_v7_evidence(config)
    _patch_v7_stage_readbacks(monkeypatch, evidence)

    provenance = capture_runtime_provenance(
        config,
        _Stage(),
        articulation_usd_solver_iterations=(32, 4),
        repository_root=repository,
        reset_strategy_snapshot=_reset_strategy_snapshot(
            wheel_joints=tuple(config.robot.wheel_joints),
            ground_filter_paths=sorted(target_colliders),
        ),
        ground_topology_snapshot=topology_snapshot,
        contact_snapshot=contact_snapshot,
        **evidence,
    )
    parameters = runtime_provenance_parameters(provenance)

    assert provenance["schema_version"] == 7
    assert set(provenance) == {
        "schema_version",
        "robot",
        "environment",
        "simulation",
        "ground_topology",
        "contact",
        "control_graph",
        "git",
    }
    assert provenance["ground_topology"]["source_collider_count"] == 32
    assert provenance["ground_topology"]["target_collider_count"] == target_count
    assert provenance["contact"]["ground_colliders"] == sorted(
        target_colliders
    )
    assert [
        binding["collider_path"]
        for binding in provenance["contact"]["ground_bindings"]
    ] == sorted(target_colliders)
    collider_contract = provenance["contact"]["collider_contract"]
    assert collider_contract["ground_required_prim_paths"] == sorted(
        target_colliders
    )
    assert collider_contract["ground_semantic_classes"] == []
    assert collider_contract["ground_expected_enabled_count"] == target_count
    assert {
        key
        for key in parameters
        if key.startswith("runtime_provenance.ground_topology.")
    } == {
        "runtime_provenance.ground_topology.json",
        "runtime_provenance.ground_topology.sha256",
    }


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda snapshot: snapshot.update(extra=True),
            "keys must be exactly",
        ),
        (
            lambda snapshot: snapshot.update(schema_version=2),
            "schema_version must be integer 1",
        ),
        (
            lambda snapshot: snapshot.update(
                id="separate_recontact_0p20m_1step_v1"
            ),
            "config identity",
        ),
        (
            lambda snapshot: snapshot.update(lift_distance_m=0.2),
            "strategy semantics",
        ),
        (
            lambda snapshot: snapshot["contact_probe"][
                "wheel_bindings"
            ].reverse(),
            "wheel joint order",
        ),
        (
            lambda snapshot: snapshot["contact_probe"]["wheel_bindings"][
                0
            ].update(wheel_link_path="/World/Robot/unbound_wheel"),
            "bound contact wheel collider or its ancestor",
        ),
        (
            lambda snapshot: snapshot["contact_probe"].update(
                ground_filter_paths=["/World/Ground/Other"]
            ),
            "ground topology target",
        ),
        (
            lambda snapshot: snapshot["contact_probe"].update(
                max_contact_count=127
            ),
            "max_contact_count must be 128",
        ),
        (
            lambda snapshot: snapshot["contact_probe"].update(
                stage_usd_readback_verified=False
            ),
            "readback",
        ),
    ],
)
def test_capture_rejects_invalid_reset_strategy_snapshot(
    tmp_path,
    monkeypatch,
    mutation,
    message,
):
    target_colliders = ["/World/Ground/Collision"]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=target_colliders,
        target_colliders=target_colliders,
    )
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(contact_snapshot),
    )
    reset_strategy_snapshot = _reset_strategy_snapshot(
        wheel_joints=tuple(config.robot.wheel_joints),
        ground_filter_paths=target_colliders,
    )
    mutation(reset_strategy_snapshot)
    evidence = _runtime_v7_evidence(config)
    _patch_v7_stage_readbacks(monkeypatch, evidence)

    with pytest.raises(RuntimeProvenanceError, match=message):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            reset_strategy_snapshot=reset_strategy_snapshot,
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=contact_snapshot,
            **evidence,
        )


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
@pytest.mark.parametrize(
    ("profile_name", "target_count"),
    [
        ("warehouse_combined32_v1.yaml", 32),
        ("warehouse_plane_only1_v1.yaml", 1),
    ],
)
def test_schema_v7_captures_scene_composer_stage_snapshots(
    profile_name,
    target_count,
):
    from isaac_sim.src.config import load_project_config
    from isaac_sim.src.stage.scene_composer import SceneComposer

    config = load_project_config(
        ROOT / "isaac_sim/configs/project.yaml",
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": (
                "/home/lyb/isaacsim_assets/Assets/Isaac/6.0"
            ),
        },
    )
    config = replace(
        config,
        files=replace(
            config.files,
            ground_topology_profile=(
                ROOT
                / "isaac_sim/configs/ground_topologies"
                / profile_name
            ),
        ),
    )
    composer = SceneComposer(config)
    stage = composer.compose(save=False)
    assert composer.ground_topology_snapshot is not None
    assert composer.contact_snapshot is not None
    assert composer.wheel_velocity_drive_snapshot is not None
    assert composer.mass_collision_snapshot is not None
    solver_iterations = stage_articulation_solver_iterations(
        stage,
        config.robot.articulation_root,
    )

    wheel_stage = composer.wheel_velocity_drive_snapshot.to_dict()
    mass_stage = composer.mass_collision_snapshot.to_dict()
    evidence = {
        "wheel_velocity_drive_snapshot": wheel_stage,
        "wheel_drive_tensor_snapshot": _wheel_drive_tensor_snapshot(
            config, wheel_stage
        ),
        "mass_collision_snapshot": mass_stage,
        "mass_tensor_snapshot": _mass_tensor_snapshot(mass_stage),
        "control_graph_snapshot": _control_graph_snapshot(config),
    }
    provenance = capture_runtime_provenance(
        config,
        stage,
        articulation_usd_solver_iterations=solver_iterations,
        repository_root=ROOT,
        reset_strategy_snapshot=_reset_strategy_snapshot(
            wheel_joints=tuple(config.robot.wheel_joints),
            wheel_link_paths=[
                path.rsplit("/", 1)[0]
                for path in composer.contact_snapshot.wheel_colliders
            ],
            ground_filter_paths=list(
                composer.ground_topology_snapshot.target_colliders
            ),
        ),
        ground_topology_snapshot=composer.ground_topology_snapshot,
        contact_snapshot=composer.contact_snapshot,
        **evidence,
    )

    assert provenance["schema_version"] == 7
    assert provenance["ground_topology"]["source_collider_count"] == 32
    assert provenance["ground_topology"]["target_collider_count"] == target_count
    assert provenance["contact"]["ground_colliders"] == provenance[
        "ground_topology"
    ]["target_colliders"]
    assert provenance["contact"]["collider_contract"][
        "ground_expected_enabled_count"
    ] == target_count


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda snapshot: snapshot.update(profile_path="/wrong/profile.yaml"),
            "profile path",
        ),
        (
            lambda snapshot: snapshot.update(environment_id="WrongEnvironment"),
            "profile identity",
        ),
        (
            lambda snapshot: snapshot.update(source_asset_sha256="0" * 64),
            "source asset SHA256",
        ),
        (
            lambda snapshot: snapshot.update(
                source_collider_paths_sha256="0" * 64
            ),
            "source collider set",
        ),
        (
            lambda snapshot: snapshot.update(stage_usd_readback_verified=False),
            "readback",
        ),
    ],
)
def test_capture_rejects_injected_ground_topology_disagreement(
    tmp_path,
    monkeypatch,
    mutation,
    message,
):
    ground = ["/World/Ground/Collision"]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=ground,
        target_colliders=ground,
    )
    mutation(topology_snapshot)
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(contact_snapshot),
    )

    with pytest.raises(RuntimeProvenanceError, match=message):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            reset_strategy_snapshot=_reset_strategy_snapshot(
                wheel_joints=tuple(config.robot.wheel_joints),
                ground_filter_paths=list(
                    topology_snapshot["target_colliders"]
                ),
            ),
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=contact_snapshot,
            **evidence,
        )


@pytest.mark.parametrize("stale_component", ["ground_topology", "contact"])
def test_capture_rejects_stale_scene_composer_snapshots(
    tmp_path,
    monkeypatch,
    stale_component,
):
    ground = ["/World/Ground/Collision"]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=ground,
        target_colliders=ground,
    )
    fresh_topology_snapshot = deepcopy(topology_snapshot)
    fresh_contact_snapshot = deepcopy(contact_snapshot)
    if stale_component == "ground_topology":
        topology_snapshot["overlay_sha256"] = "7" * 64
    else:
        contact_snapshot["overlay_sha256"] = "8" * 64

    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(fresh_topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(fresh_contact_snapshot),
    )
    evidence = _runtime_v7_evidence(config)
    _patch_v7_stage_readbacks(monkeypatch, evidence)

    with pytest.raises(RuntimeProvenanceError, match="stale or differs"):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            reset_strategy_snapshot=_reset_strategy_snapshot(
                wheel_joints=tuple(config.robot.wheel_joints),
                ground_filter_paths=list(
                    topology_snapshot["target_colliders"]
                ),
            ),
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=contact_snapshot,
            **evidence,
        )


def test_schema_v7_requires_physical_and_control_evidence_arguments():
    parameters = inspect.signature(capture_runtime_provenance).parameters
    required = {
        "wheel_velocity_drive_snapshot",
        "wheel_drive_tensor_snapshot",
        "mass_collision_snapshot",
        "mass_tensor_snapshot",
        "control_graph_snapshot",
    }
    assert required <= set(parameters)
    assert all(
        parameters[name].default is inspect.Parameter.empty
        for name in required
    )


def test_explicit_legacy_diagnostic_producer_remains_schema_v6(
    tmp_path,
    monkeypatch,
):
    ground = ["/World/Ground/Collision"]
    repository, config, topology_snapshot, contact_snapshot = _runtime_inputs(
        tmp_path,
        source_colliders=ground,
        target_colliders=ground,
    )
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(contact_snapshot),
    )

    provenance = capture_runtime_provenance_v6_legacy(
        config,
        _Stage(),
        articulation_usd_solver_iterations=(32, 4),
        repository_root=repository,
        reset_strategy_snapshot=_reset_strategy_snapshot(
            wheel_joints=tuple(config.robot.wheel_joints),
            ground_filter_paths=ground,
        ),
        ground_topology_snapshot=topology_snapshot,
        contact_snapshot=contact_snapshot,
    )

    assert provenance["schema_version"] == 6
    assert "control_graph" not in provenance
    assert "wheel_velocity_drive" not in provenance["robot"]
    assert runtime_provenance_parameters(provenance)[
        "runtime_provenance.schema_version"
    ] == 6


def test_capture_rejects_stage_and_articulation_solver_disagreement(tmp_path):
    repository = _repository(tmp_path)
    required = []
    for name in ("robot.yaml", "robot.usda", "project.usda", "source.usd"):
        path = tmp_path / name
        path.write_bytes(b"input")
        required.append(path)
    config = SimpleNamespace(
        files=SimpleNamespace(robot=required[0]),
        robot=SimpleNamespace(
            asset_path=required[1],
            articulation_root="/World/Robot",
        ),
        environment=SimpleNamespace(
            identifier="Warehouse",
            project_stage=required[2],
            source_asset=required[3],
        ),
        asset_root=tmp_path,
        simulation=SimpleNamespace(
            navigation_mode="mapping",
            odometry_mode="ideal",
            physics_hz=60.0,
        ),
    )

    with pytest.raises(RuntimeProvenanceError, match="solver readback disagree"):
        capture_runtime_provenance(
            config,
            _Stage((32, 4)),
            articulation_usd_solver_iterations=(32, 16),
            repository_root=repository,
            reset_strategy_snapshot=_reset_strategy_snapshot(),
            wheel_velocity_drive_snapshot={},
            wheel_drive_tensor_snapshot={},
            mass_collision_snapshot={},
            mass_tensor_snapshot={},
            control_graph_snapshot={},
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda snapshot: snapshot.update(profile_path="/wrong/profile.yaml"),
            "profile path",
        ),
        (
            lambda snapshot: snapshot.update(profile_sha256="0" * 64),
            "profile SHA256",
        ),
        (
            lambda snapshot: snapshot["scene"].update(
                physics_scene_path="/WrongScene"
            ),
            "PhysicsScene",
        ),
        (
            lambda snapshot: snapshot.update(
                ground_colliders=["/World/Ground/Other"]
            ),
            "ground topology target",
        ),
        (
            lambda snapshot: snapshot.update(
                stage_usd_readback_verified=False
            ),
            "readback",
        ),
        (
            lambda snapshot: snapshot.update(explicit_materials=0),
            "flags must be boolean",
        ),
    ],
)
def test_capture_rejects_contact_snapshot_that_disagrees_with_config(
    tmp_path, monkeypatch, mutation, message
):
    repository = _repository(tmp_path)
    robot_config = tmp_path / "robot.yaml"
    robot_asset = tmp_path / "robot.usda"
    project_stage = tmp_path / "project.usda"
    source_asset = tmp_path / "source.usd"
    ground_topology_profile = tmp_path / "ground_topology.yaml"
    contact_profile = tmp_path / "legacy_baseline.yaml"
    _write_test_robot_config(robot_config)
    robot_asset.write_bytes(JACKAL_ASSET.read_bytes())
    for path in (project_stage, source_asset):
        path.write_bytes(b"input")
    contact_profile.write_text(
        "schema_version: 1\nid: legacy-baseline\nmode: legacy_baseline\n",
        encoding="utf-8",
    )
    ground_colliders = ["/World/Ground/Collision"]
    _write_ground_topology_profile(
        ground_topology_profile,
        source_asset,
        ground_colliders,
        ground_colliders,
    )
    config = SimpleNamespace(
        files=SimpleNamespace(
            robot=robot_config,
            ground_topology_profile=ground_topology_profile,
            contact_profile=contact_profile,
        ),
        robot=SimpleNamespace(
            asset_path=robot_asset,
            articulation_root="/World/Robot",
            runtime_prim_path="/World/Robot",
            base_link_prim="/World/Robot/base_link",
            wheel_joints=WHEEL_JOINTS,
            front_wheel_joints=WHEEL_JOINTS[:2],
            rear_wheel_joints=WHEEL_JOINTS[2:],
        ),
        environment=SimpleNamespace(
            identifier="Warehouse",
            project_stage=project_stage,
            source_asset=source_asset,
            ground_colliders=SimpleNamespace(
                required_prim_paths=("/World/Ground/Collision",),
                semantic_classes=(),
                expected_enabled_count=1,
            ),
        ),
        asset_root=tmp_path,
        simulation=SimpleNamespace(
            navigation_mode="mapping",
            odometry_mode="ideal",
            physics_hz=60.0,
            expected_physics_scene="/PhysicsScene",
            reset_strategy=SimpleNamespace(
                schema_version=1,
                identifier="pose_restore_v1",
            ),
        ),
    )
    snapshot = _contact_snapshot(contact_profile)
    topology_snapshot = _ground_topology_snapshot(
        ground_topology_profile,
        source_asset,
        ground_colliders,
        ground_colliders,
    )
    mutation(snapshot)
    from isaac_sim.src.stage import contact_setup, ground_topology

    monkeypatch.setattr(
        ground_topology,
        "capture_ground_topology_snapshot",
        lambda stage, runtime_config: deepcopy(topology_snapshot),
    )
    monkeypatch.setattr(
        contact_setup,
        "capture_contact_profile_snapshot",
        lambda stage, runtime_config: deepcopy(snapshot),
    )
    evidence = _runtime_v7_evidence(config)
    _patch_v7_stage_readbacks(monkeypatch, evidence)

    with pytest.raises(RuntimeProvenanceError, match=message):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            reset_strategy_snapshot=_reset_strategy_snapshot(
                wheel_joints=tuple(config.robot.wheel_joints),
                ground_filter_paths=list(
                    topology_snapshot["target_colliders"]
                ),
            ),
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=snapshot,
            **evidence,
        )
