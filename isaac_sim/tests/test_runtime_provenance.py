from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from isaac_sim.src.runtime_provenance import (
    RuntimeProvenanceError,
    capture_runtime_provenance,
    file_sha256,
    git_metadata,
    runtime_provenance_parameters,
    stage_articulation_solver_iterations,
)
from isaac_sim.src.stage.ground_topology import (
    collider_paths_sha256,
    load_ground_topology_profile,
)


JACKAL_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "configs"
    / "robots"
    / "jackal.yaml"
)
ROOT = Path(__file__).resolve().parents[2]
HAS_PXR = importlib.util.find_spec("pxr") is not None


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
    robot_config.write_bytes(JACKAL_CONFIG.read_bytes())
    robot_asset.write_bytes(b"#usda 1.0\n")
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
            wheel_joints=("fl", "fr", "rl", "rr"),
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
        (robot_config, JACKAL_CONFIG.read_bytes()),
        (robot_asset, b"#usda 1.0\n"),
        (project_stage, b"#usda 1.0\n"),
        (source_asset, b"PXR-USDC"),
        (
            contact_profile,
            b"schema_version: 1\nid: legacy-baseline\nmode: legacy_baseline\n",
        ),
    ):
        path.write_bytes(content)
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
            wheel_joints=("fl", "fr", "rl", "rr"),
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
        ground_topology_snapshot=expected_ground_topology_snapshot,
        contact_snapshot=expected_contact_snapshot,
    )
    parameters = runtime_provenance_parameters(provenance)

    assert fresh_readbacks == ["ground_topology", "contact"]
    assert provenance["schema_version"] == 5
    assert parameters["runtime_provenance.schema_version"] == 5
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
    assert provenance["contact"]["collider_contract"] == {
        "wheel_joint_names": ["fl", "fr", "rl", "rr"],
        "wheel_expected_count": 4,
        "ground_required_prim_paths": ["/World/Ground/Collision"],
        "ground_semantic_classes": [],
        "ground_expected_enabled_count": 1,
    }


@pytest.mark.parametrize("target_count", [32, 1])
def test_schema_v5_locks_combined_and_plane_only_ground_topologies(
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

    provenance = capture_runtime_provenance(
        config,
        _Stage(),
        articulation_usd_solver_iterations=(32, 4),
        repository_root=repository,
        ground_topology_snapshot=topology_snapshot,
        contact_snapshot=contact_snapshot,
    )
    parameters = runtime_provenance_parameters(provenance)

    assert provenance["schema_version"] == 5
    assert set(provenance) == {
        "schema_version",
        "robot",
        "environment",
        "simulation",
        "ground_topology",
        "contact",
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


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
@pytest.mark.parametrize(
    ("profile_name", "target_count"),
    [
        ("warehouse_combined32_v1.yaml", 32),
        ("warehouse_plane_only1_v1.yaml", 1),
    ],
)
def test_schema_v5_captures_scene_composer_topology_snapshots(
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
    solver_iterations = stage_articulation_solver_iterations(
        stage,
        config.robot.articulation_root,
    )

    provenance = capture_runtime_provenance(
        config,
        stage,
        articulation_usd_solver_iterations=solver_iterations,
        repository_root=ROOT,
        ground_topology_snapshot=composer.ground_topology_snapshot,
        contact_snapshot=composer.contact_snapshot,
    )

    assert provenance["schema_version"] == 5
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
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=contact_snapshot,
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

    with pytest.raises(RuntimeProvenanceError, match="stale or differs"):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=contact_snapshot,
        )


def test_navigation_runtime_injects_scene_composer_snapshots():
    source = (ROOT / "isaac_sim/apps/navigation_sim.py").read_text(
        encoding="utf-8"
    )
    runtime = source.split("def run(", 1)[1].split("def main(", 1)[0]
    assert "composer = SceneComposer(config)" in runtime
    assert "stage = composer.compose(save=False)" in runtime
    capture = runtime.split("runtime_provenance = capture_runtime_provenance(", 1)[
        1
    ].split("\n        )", 1)[0]
    assert (
        "ground_topology_snapshot=composer.ground_topology_snapshot" in capture
    )
    assert "contact_snapshot=composer.contact_snapshot" in capture


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
    robot_config.write_bytes(JACKAL_CONFIG.read_bytes())
    for path in (robot_asset, project_stage, source_asset):
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
            wheel_joints=("fl", "fr", "rl", "rr"),
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

    with pytest.raises(RuntimeProvenanceError, match=message):
        capture_runtime_provenance(
            config,
            _Stage(),
            articulation_usd_solver_iterations=(32, 4),
            repository_root=repository,
            ground_topology_snapshot=topology_snapshot,
            contact_snapshot=snapshot,
        )
