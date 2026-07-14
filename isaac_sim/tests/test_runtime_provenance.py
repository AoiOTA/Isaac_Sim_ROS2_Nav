from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess
from types import SimpleNamespace

from isaac_sim.src.runtime_provenance import (
    capture_runtime_provenance,
    file_sha256,
    git_metadata,
    runtime_provenance_parameters,
)


class _RootLayer:
    def ExportToString(self) -> str:
        return "#usda 1.0\ndef Xform \"World\" {}\n"


class _Stage:
    def GetRootLayer(self) -> _RootLayer:
        return _RootLayer()


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


def test_capture_flattens_the_effective_robot_environment_and_stage(tmp_path):
    repository = _repository(tmp_path)
    robot_config = tmp_path / "jackal.yaml"
    robot_asset = tmp_path / "jackal_nav.usda"
    project_stage = tmp_path / "navigation_scene.usda"
    source_asset = tmp_path / "warehouse.usd"
    for path, content in (
        (robot_config, b"physics: {}\n"),
        (robot_asset, b"#usda 1.0\n"),
        (project_stage, b"#usda 1.0\n"),
        (source_asset, b"PXR-USDC"),
    ):
        path.write_bytes(content)
    asset_root = tmp_path / "6.0"
    asset_root.mkdir()
    config = SimpleNamespace(
        files=SimpleNamespace(robot=robot_config),
        robot=SimpleNamespace(asset_path=robot_asset),
        environment=SimpleNamespace(
            project_stage=project_stage,
            source_asset=source_asset,
        ),
        asset_root=asset_root,
        simulation=SimpleNamespace(
            navigation_mode="mapping",
            odometry_mode="ideal",
            physics_hz=60.0,
        ),
    )
    settings = SimpleNamespace(
        solver_position_iterations=32,
        solver_velocity_iterations=4,
    )

    provenance = capture_runtime_provenance(
        config,
        settings,
        _Stage(),
        repository_root=repository,
    )
    parameters = runtime_provenance_parameters(provenance)

    assert provenance["robot"]["config"]["sha256"] == file_sha256(
        robot_config
    )
    assert provenance["robot"]["solver"] == {
        "position_iterations": 32,
        "velocity_iterations": 4,
    }
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
