"""Capture immutable evidence for the inputs loaded by one Isaac process."""

from __future__ import annotations

import hashlib
from pathlib import Path
import subprocess
from typing import Any, Mapping


class RuntimeProvenanceError(RuntimeError):
    """Raised when a runtime input cannot be bound to reproducible evidence."""


def file_sha256(path: str | Path) -> str:
    """Return the SHA256 digest of one required regular file."""

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise RuntimeProvenanceError(
            f"runtime provenance input is not a file: {source}"
        )
    digest = hashlib.sha256()
    with source.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_metadata(root: str | Path) -> dict[str, object]:
    """Snapshot the source revision and dirty state at Isaac startup."""

    repository = Path(root).expanduser().resolve()

    def git(*arguments: str) -> str:
        try:
            result = subprocess.run(
                ["git", "-C", str(repository), *arguments],
                text=True,
                capture_output=True,
                check=False,
                timeout=3.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeProvenanceError(
                f"failed to inspect Git repository {repository}: {exc}"
            ) from exc
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeProvenanceError(
                f"Git metadata command failed in {repository}: {detail}"
            )
        return result.stdout.strip()

    commit = git("rev-parse", "HEAD")
    branch = git("branch", "--show-current") or "detached"
    status = git("status", "--porcelain", "--untracked-files=normal")
    return {
        "commit": commit,
        "branch": branch,
        "dirty": bool(status),
    }


def capture_runtime_provenance(
    config: Any,
    articulation_settings: Any,
    stage: Any,
    *,
    repository_root: str | Path,
) -> dict[str, object]:
    """Capture the effective files and in-memory Stage loaded by Isaac."""

    root_layer_source = stage.GetRootLayer().ExportToString()
    if not isinstance(root_layer_source, str) or not root_layer_source:
        raise RuntimeProvenanceError(
            "composed Stage root layer could not be exported"
        )
    return {
        "schema_version": 1,
        "robot": {
            "config": {
                "path": str(config.files.robot),
                "sha256": file_sha256(config.files.robot),
            },
            "asset": {
                "path": str(config.robot.asset_path),
                "sha256": file_sha256(config.robot.asset_path),
            },
            "solver": {
                "position_iterations": (
                    articulation_settings.solver_position_iterations
                ),
                "velocity_iterations": (
                    articulation_settings.solver_velocity_iterations
                ),
            },
        },
        "environment": {
            "project_stage": {
                "path": str(config.environment.project_stage),
                "sha256": file_sha256(config.environment.project_stage),
            },
            "source_asset": {
                "path": str(config.environment.source_asset),
                "sha256": file_sha256(config.environment.source_asset),
            },
            "asset_root": str(config.asset_root),
            "asset_version": config.asset_root.name,
            "composed_root_layer_sha256": hashlib.sha256(
                root_layer_source.encode("utf-8")
            ).hexdigest(),
        },
        "simulation": {
            "navigation_mode": config.simulation.navigation_mode,
            "odometry_mode": config.simulation.odometry_mode,
            "physics_hz": config.simulation.physics_hz,
        },
        "git": git_metadata(repository_root),
    }


def runtime_provenance_parameters(
    provenance: Mapping[str, Any],
) -> dict[str, str | bool | int | float]:
    """Flatten a captured snapshot into read-only ROS parameter values."""

    robot = provenance["robot"]
    environment = provenance["environment"]
    simulation = provenance["simulation"]
    git = provenance["git"]
    return {
        "runtime_provenance.schema_version": provenance["schema_version"],
        "runtime_provenance.robot.config.path": robot["config"]["path"],
        "runtime_provenance.robot.config.sha256": robot["config"]["sha256"],
        "runtime_provenance.robot.asset.path": robot["asset"]["path"],
        "runtime_provenance.robot.asset.sha256": robot["asset"]["sha256"],
        "runtime_provenance.robot.solver.position_iterations": robot["solver"][
            "position_iterations"
        ],
        "runtime_provenance.robot.solver.velocity_iterations": robot["solver"][
            "velocity_iterations"
        ],
        "runtime_provenance.environment.project_stage.path": environment[
            "project_stage"
        ]["path"],
        "runtime_provenance.environment.project_stage.sha256": environment[
            "project_stage"
        ]["sha256"],
        "runtime_provenance.environment.source_asset.path": environment[
            "source_asset"
        ]["path"],
        "runtime_provenance.environment.source_asset.sha256": environment[
            "source_asset"
        ]["sha256"],
        "runtime_provenance.environment.asset_root": environment["asset_root"],
        "runtime_provenance.environment.asset_version": environment[
            "asset_version"
        ],
        "runtime_provenance.environment.composed_root_layer_sha256": environment[
            "composed_root_layer_sha256"
        ],
        "runtime_provenance.simulation.navigation_mode": simulation[
            "navigation_mode"
        ],
        "runtime_provenance.simulation.odometry_mode": simulation["odometry_mode"],
        "runtime_provenance.simulation.physics_hz": simulation["physics_hz"],
        "runtime_provenance.git.commit": git["commit"],
        "runtime_provenance.git.branch": git["branch"],
        "runtime_provenance.git.dirty": git["dirty"],
    }
