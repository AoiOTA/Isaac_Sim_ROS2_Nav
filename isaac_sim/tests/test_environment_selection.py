from __future__ import annotations

import os
from pathlib import Path

import pytest

from isaac_sim.src.environment_selection import (
    EnvironmentSelectionError,
    resolve_environment_usd,
    resolve_spawn_poses_file,
    runtime_project_stage,
)
from isaac_sim.apps.navigation_sim import _apply_cli_overrides, _parser


def _usd(path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("#usda 1.0\n", encoding="utf-8")
    return path


def test_environment_resolves_absolute_relative_and_unique_filename(tmp_path: Path):
    asset = _usd(tmp_path / "room_a" / "living_room.usd")

    assert resolve_environment_usd(asset, tmp_path) == asset.resolve()
    assert resolve_environment_usd("room_a/living_room.usd", tmp_path) == asset.resolve()
    assert resolve_environment_usd("living_room.usd", tmp_path) == asset.resolve()


def test_environment_filename_must_be_unambiguous(tmp_path: Path):
    _usd(tmp_path / "room_a" / "scene.usd")
    _usd(tmp_path / "room_b" / "scene.usd")

    with pytest.raises(EnvironmentSelectionError, match="ambiguous"):
        resolve_environment_usd("scene.usd", tmp_path)


def test_environment_rejects_missing_and_non_usd_files(tmp_path: Path):
    text_file = tmp_path / "scene.txt"
    text_file.write_text("not usd", encoding="utf-8")

    with pytest.raises(EnvironmentSelectionError, match="not found"):
        resolve_environment_usd("missing.usd", tmp_path)
    with pytest.raises(EnvironmentSelectionError, match="must be a USD file"):
        resolve_environment_usd(text_file, tmp_path)


def test_spawn_profile_precedence_is_explicit_sidecar_then_repository(tmp_path: Path):
    asset = _usd(tmp_path / "assets" / "scene.usd")
    repository = tmp_path / "profiles"
    repository.mkdir()
    repository_profile = repository / "scene.spawn.yaml"
    repository_profile.write_text("repo", encoding="utf-8")

    assert resolve_spawn_poses_file(
        asset,
        explicit=None,
        repository_profiles=repository,
    ) == repository_profile.resolve()

    sidecar = asset.with_name("scene.spawn.yaml")
    sidecar.write_text("sidecar", encoding="utf-8")
    assert resolve_spawn_poses_file(
        asset,
        explicit=None,
        repository_profiles=repository,
    ) == sidecar.resolve()

    explicit = tmp_path / "explicit.yaml"
    explicit.write_text("explicit", encoding="utf-8")
    assert resolve_spawn_poses_file(
        asset,
        explicit=explicit,
        repository_profiles=repository,
    ) == explicit.resolve()


def test_missing_spawn_profile_fails_with_creation_locations(tmp_path: Path):
    asset = _usd(tmp_path / "assets" / "scene.usd")

    with pytest.raises(EnvironmentSelectionError, match="no spawn-pose profile"):
        resolve_spawn_poses_file(
            asset,
            explicit=None,
            repository_profiles=tmp_path / "profiles",
        )


def test_runtime_project_stage_is_stable_and_asset_specific(tmp_path: Path):
    asset_a = _usd(tmp_path / "assets" / "scene_a.usd")
    asset_b = _usd(tmp_path / "assets" / "scene_b.usd")
    runtime = tmp_path / "runtime"

    first = runtime_project_stage(asset_a, runtime)
    assert first == runtime_project_stage(asset_a, runtime)
    assert first.parent == runtime / "stages"
    assert first.suffix == ".usda"
    assert first != runtime_project_stage(asset_b, runtime)


def test_custom_environment_keeps_default_follow_camera_enabled(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    asset = _usd(tmp_path / "room" / "scene.usd")
    asset.with_name("scene.spawn.yaml").write_text(
        "schema_version: 1\nposes: {}\n",
        encoding="utf-8",
    )
    monkeypatch.delenv(
        "ISAAC_NAV__THIRD_PERSON_CAMERA__ENABLED",
        raising=False,
    )
    args = _parser().parse_args([
        "--environment-usd",
        str(asset),
        "--environment-root",
        str(tmp_path),
    ])

    _apply_cli_overrides(args)

    assert "ISAAC_NAV__THIRD_PERSON_CAMERA__ENABLED" not in os.environ
