"""Resolve user-selected USD environments and their spawn-pose profiles."""

from __future__ import annotations

import hashlib
from pathlib import Path


USD_SUFFIXES = frozenset({".usd", ".usda", ".usdc", ".usdz"})
DEFAULT_ENVIRONMENT_ROOT = Path.home() / "kujiale_usd_rooms_20260717"


class EnvironmentSelectionError(ValueError):
    """Raised when a requested environment cannot be selected unambiguously."""


def _require_usd_file(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    if resolved.suffix.lower() not in USD_SUFFIXES:
        raise EnvironmentSelectionError(
            f"environment asset must be a USD file ({', '.join(sorted(USD_SUFFIXES))}): "
            f"{resolved}"
        )
    if not resolved.is_file():
        raise EnvironmentSelectionError(f"environment USD file not found: {resolved}")
    return resolved


def resolve_environment_usd(request: str | Path, root: str | Path) -> Path:
    """Resolve an absolute path, a root-relative path, or a unique basename."""

    requested = Path(request).expanduser()
    environment_root = Path(root).expanduser().resolve()
    if requested.is_absolute():
        return _require_usd_file(requested)
    if not environment_root.is_dir():
        raise EnvironmentSelectionError(
            f"environment root directory not found: {environment_root}"
        )

    direct = environment_root / requested
    if direct.is_file():
        return _require_usd_file(direct)
    if len(requested.parts) > 1:
        raise EnvironmentSelectionError(
            f"environment USD file not found below {environment_root}: {requested}"
        )

    matches = sorted(
        path.resolve()
        for path in environment_root.rglob(requested.name)
        if path.is_file() and path.suffix.lower() in USD_SUFFIXES
    )
    if not matches:
        raise EnvironmentSelectionError(
            f"environment USD named {requested.name!r} was not found below "
            f"{environment_root}"
        )
    if len(matches) > 1:
        choices = "\n".join(f"- {path}" for path in matches)
        raise EnvironmentSelectionError(
            f"environment USD name {requested.name!r} is ambiguous; use a relative "
            f"or absolute path:\n{choices}"
        )
    return matches[0]


def resolve_spawn_poses_file(
    environment_usd: str | Path,
    *,
    explicit: str | Path | None,
    repository_profiles: str | Path,
) -> Path:
    """Select an explicit, sidecar, or repository-owned spawn profile."""

    asset = Path(environment_usd).expanduser().resolve()
    if explicit is not None:
        profile = Path(explicit).expanduser().resolve()
        if not profile.is_file():
            raise EnvironmentSelectionError(f"spawn poses file not found: {profile}")
        return profile

    filename = f"{asset.stem}.spawn.yaml"
    candidates = (
        asset.with_name(filename),
        Path(repository_profiles).expanduser().resolve() / filename,
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    locations = "\n".join(f"- {candidate}" for candidate in candidates)
    raise EnvironmentSelectionError(
        f"no spawn-pose profile exists for {asset.name}; create one at either:\n"
        f"{locations}\nor pass --spawn-poses-file PATH"
    )


def runtime_project_stage(environment_usd: str | Path, runtime_dir: str | Path) -> Path:
    """Return a stable per-environment writable Stage path outside the source tree."""

    asset = Path(environment_usd).expanduser().resolve()
    digest = hashlib.sha256(str(asset).encode("utf-8")).hexdigest()[:12]
    return Path(runtime_dir).expanduser().resolve() / "stages" / f"{asset.stem}-{digest}.usda"
