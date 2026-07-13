"""Strict, ROS-independent map-bundle integrity and calibration contract."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import math
import os
from pathlib import Path
import re
import tempfile
from typing import Any, Mapping, Sequence

import yaml


SCHEMA_VERSION = 1
_MAP_VERSION = re.compile(r"^[A-Za-z0-9._-]+$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_LFS_POINTER = b"version https://git-lfs.github.com/spec/v1"
_ARTIFACT_LAYOUT = (
    ("occupancy_grid", "yaml", "data/maps/occupancy/{version}.yaml"),
    ("occupancy_grid", "image", "data/maps/occupancy/{version}.pgm"),
    ("pose_graph", "posegraph", "data/maps/posegraphs/{version}.posegraph"),
    ("pose_graph", "data", "data/maps/posegraphs/{version}.data"),
)
_ROOT_FIELDS = {
    "schema_version",
    "map_version",
    "created_at",
    "availability",
    "artifact_policy",
    "bundle_sha256",
    "source",
    "occupancy_grid",
    "pose_graph",
    "calibration",
    "notes",
}
_OCCUPANCY_FIELDS = {
    "resolution_m", "width_cells", "height_cells", "origin", "files"
}
_POSE_GRAPH_FIELDS = {"prefix", "files"}
_CALIBRATION_FIELDS = {
    "calibrated",
    "spawn_pose_profile",
    "bundle_sha256",
    "calibrated_at",
    "calibration_method",
    "usd_base_position",
    "usd_base_yaw_deg",
    "map_base_position",
    "map_base_yaw_deg",
    "position_stddev_m",
    "yaw_stddev_deg",
}


def _validate_map_version(value: Any) -> str:
    if (
        not isinstance(value, str)
        or len(value) > 64
        or not _MAP_VERSION.fullmatch(value)
        or not any(character != "." for character in value)
    ):
        raise MapManifestError("map_version contains unsafe characters")
    return value


class MapManifestError(ValueError):
    """Raised when a map manifest or its artifact bundle is unsafe."""


@dataclass(frozen=True)
class MapArtifact:
    role: str
    path: str
    bytes: int
    sha256: str
    resolved_path: Path


@dataclass(frozen=True)
class MapCalibration:
    calibrated: bool
    spawn_pose_profile: str | None
    bundle_sha256: str | None
    usd_base_position: tuple[float, float, float] | None
    usd_base_yaw_deg: float | None
    map_base_position: tuple[float, float] | None
    map_base_yaw_deg: float | None
    position_stddev_m: float | None
    yaw_stddev_deg: float | None


@dataclass(frozen=True)
class MapManifest:
    source: Path
    project_root: Path
    map_version: str
    bundle_sha256: str
    artifacts: tuple[MapArtifact, ...]
    calibration: MapCalibration

    @property
    def occupancy_yaml(self) -> Path:
        return self.artifacts[0].resolved_path

    @property
    def occupancy_image(self) -> Path:
        return self.artifacts[1].resolved_path

    @property
    def posegraph_prefix(self) -> Path:
        return self.artifacts[2].resolved_path.with_suffix("")


def _mapping(value: Any, location: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MapManifestError(f"{location} must be a mapping")
    return value


def _reject_unknown(
    value: Mapping[str, Any], allowed: set[str], location: str
) -> None:
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise MapManifestError(f"unknown {location} fields: {unknown}")


def _finite_number(value: Any, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MapManifestError(f"{location} must be a finite number")
    parsed = float(value)
    if not math.isfinite(parsed):
        raise MapManifestError(f"{location} must be a finite number")
    return parsed


def _finite_vector(value: Any, length: int, location: str) -> tuple[float, ...]:
    if not isinstance(value, list) or len(value) != length:
        raise MapManifestError(
            f"{location} must contain exactly {length} finite numbers"
        )
    return tuple(
        _finite_number(component, f"{location}[{index}]")
        for index, component in enumerate(value)
    )


def _load_yaml(path: Path, label: str) -> Mapping[str, Any]:
    if not path.is_file():
        raise MapManifestError(f"{label} does not exist: {path}")
    if path.is_symlink():
        raise MapManifestError(f"{label} must not be a symlink: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise MapManifestError(f"cannot read {label} {path}: {exc}") from exc
    if payload.startswith(_LFS_POINTER):
        raise MapManifestError(
            f"{label} is an unhydrated Git LFS pointer: {path}; run git lfs pull"
        )
    try:
        document = yaml.safe_load(payload)
    except yaml.YAMLError as exc:
        raise MapManifestError(f"invalid YAML in {label} {path}: {exc}") from exc
    return _mapping(document, label)


def _infer_project_root(manifest_path: Path) -> Path:
    absolute = Path(os.path.abspath(manifest_path))
    if len(absolute.parents) < 4:
        raise MapManifestError(
            f"cannot infer project root from map manifest path: {manifest_path}"
        )
    return absolute.parents[3].resolve()


def _safe_project_path(project_root: Path, relative: str, location: str) -> Path:
    if not isinstance(relative, str) or not relative:
        raise MapManifestError(
            f"{location} must be a non-empty project-relative path"
        )
    path = Path(relative)
    if path.is_absolute():
        raise MapManifestError(f"{location} must be project-relative: {relative}")
    root = project_root.resolve()
    unresolved = root / path
    current = root
    for component in path.parts:
        current /= component
        if current.is_symlink():
            raise MapManifestError(
                f"{location} traverses a symlink: {relative}"
            )
    candidate = unresolved.resolve()
    try:
        candidate.relative_to(root)
    except ValueError as exc:
        raise MapManifestError(
            f"{location} escapes the project root: {relative}"
        ) from exc
    return candidate


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def compute_bundle_sha256(entries: Sequence[tuple[str, str, int, str]]) -> str:
    """Hash ordered role/path/size/content hashes into one bundle identity."""
    digest = hashlib.sha256()
    for role, path, byte_count, sha256 in entries:
        digest.update(
            f"{role}\0{path}\0{byte_count}\0{sha256}\n".encode("utf-8")
        )
    return digest.hexdigest()


def _validate_artifact(
    project_root: Path,
    section_name: str,
    role: str,
    expected_path: str,
    raw_entry: Any,
    *,
    verify_files: bool,
) -> MapArtifact:
    entry = _mapping(raw_entry, f"{section_name}.files[{role}]")
    if set(entry) != {"role", "path", "bytes", "sha256"}:
        raise MapManifestError(
            f"{section_name}.files[{role}] must contain exactly "
            "role, path, bytes, and sha256"
        )
    if entry["role"] != role:
        raise MapManifestError(
            f"{section_name}.files role mismatch: expected {role!r}, "
            f"got {entry['role']!r}"
        )
    if entry["path"] != expected_path:
        raise MapManifestError(
            f"{section_name}.{role} path must be {expected_path!r}, "
            f"got {entry['path']!r}"
        )
    byte_count = entry["bytes"]
    if (
        isinstance(byte_count, bool)
        or not isinstance(byte_count, int)
        or byte_count <= 0
    ):
        raise MapManifestError(
            f"{section_name}.{role}.bytes must be a positive integer"
        )
    content_hash = entry["sha256"]
    if not isinstance(content_hash, str) or not _SHA256.fullmatch(content_hash):
        raise MapManifestError(
            f"{section_name}.{role}.sha256 must be a lowercase SHA256 digest"
        )
    resolved = _safe_project_path(
        project_root, expected_path, f"{section_name}.{role}.path"
    )
    if verify_files:
        if not resolved.is_file():
            raise MapManifestError(f"map artifact does not exist: {resolved}")
        if resolved.is_symlink():
            raise MapManifestError(f"map artifact must not be a symlink: {resolved}")
        with resolved.open("rb") as stream:
            if stream.read(len(_LFS_POINTER)).startswith(_LFS_POINTER):
                raise MapManifestError(
                    f"map artifact is an unhydrated Git LFS pointer: {resolved}; "
                    "run git lfs pull"
                )
        actual_size = resolved.stat().st_size
        if actual_size != byte_count:
            raise MapManifestError(
                f"map artifact size mismatch for {resolved}: "
                f"manifest={byte_count}, actual={actual_size}"
            )
        actual_hash = _sha256(resolved)
        if actual_hash != content_hash:
            raise MapManifestError(
                f"map artifact SHA256 mismatch for {resolved}: "
                f"manifest={content_hash}, actual={actual_hash}"
            )
    return MapArtifact(
        role=role,
        path=expected_path,
        bytes=byte_count,
        sha256=content_hash,
        resolved_path=resolved,
    )


def _validate_occupancy_image_binding(
    yaml_path: Path, image_path: Path
) -> Mapping[str, Any]:
    document = _load_yaml(yaml_path, "occupancy map YAML")
    image = document.get("image")
    if not isinstance(image, str) or not image.strip():
        raise MapManifestError("occupancy map YAML image must be a non-empty path")
    declared = Path(image)
    if declared.is_absolute():
        raise MapManifestError("occupancy map YAML image must be relative")
    if declared != Path(image_path.name):
        raise MapManifestError(
            "occupancy map YAML image must be the manifested PGM basename: "
            f"expected {image_path.name!r}, got {image!r}"
        )
    resolved = (yaml_path.parent / declared).resolve()
    if resolved != image_path.resolve():
        raise MapManifestError(
            "occupancy map YAML image does not bind to the manifested PGM: "
            f"{image!r} resolves to {resolved}, expected {image_path.resolve()}"
        )
    return document


def _pgm_dimensions(path: Path) -> tuple[int, int]:
    tokens: list[bytes] = []
    try:
        with path.open("rb") as stream:
            while len(tokens) < 4:
                line = stream.readline()
                if not line:
                    break
                tokens.extend(line.split(b"#", 1)[0].split())
    except OSError as exc:
        raise MapManifestError(f"cannot read PGM header {path}: {exc}") from exc
    if len(tokens) < 4 or tokens[0] not in {b"P2", b"P5"}:
        raise MapManifestError(f"occupancy image has an invalid PGM header: {path}")
    try:
        width, height, maximum = map(int, tokens[1:4])
    except ValueError as exc:
        raise MapManifestError(
            f"occupancy image has non-integer PGM dimensions: {path}"
        ) from exc
    if width <= 0 or height <= 0 or not 0 < maximum <= 65535:
        raise MapManifestError(f"occupancy image has invalid PGM dimensions: {path}")
    return width, height


def _validate_occupancy_metadata(
    section: Mapping[str, Any],
    occupancy_yaml: Mapping[str, Any],
    image_path: Path,
) -> tuple[float, int, int, list[float]]:
    yaml_resolution = _finite_number(
        occupancy_yaml.get("resolution"), "occupancy map YAML resolution"
    )
    if yaml_resolution <= 0.0:
        raise MapManifestError(
            "occupancy map YAML resolution must be finite and positive"
        )
    yaml_origin_raw = occupancy_yaml.get("origin")
    if not isinstance(yaml_origin_raw, list) or len(yaml_origin_raw) != 3:
        raise MapManifestError(
            "occupancy map YAML origin must contain exactly three numbers"
        )
    yaml_origin = [
        _finite_number(value, f"occupancy map YAML origin[{index}]")
        for index, value in enumerate(yaml_origin_raw)
    ]
    width, height = _pgm_dimensions(image_path)
    if "resolution_m" in section:
        declared = _finite_number(
            section["resolution_m"], "occupancy_grid.resolution_m"
        )
        if not math.isclose(declared, yaml_resolution, rel_tol=0.0, abs_tol=1e-12):
            raise MapManifestError(
                "occupancy_grid.resolution_m does not match occupancy YAML: "
                f"manifest={declared}, yaml={yaml_resolution}"
            )
    for key, actual in (("width_cells", width), ("height_cells", height)):
        if key in section:
            declared = section[key]
            if isinstance(declared, bool) or not isinstance(declared, int):
                raise MapManifestError(f"occupancy_grid.{key} must be an integer")
            if declared != actual:
                raise MapManifestError(
                    f"occupancy_grid.{key} does not match PGM header: "
                    f"manifest={declared}, pgm={actual}"
                )
    if "origin" in section:
        raw_origin = section["origin"]
        if not isinstance(raw_origin, list) or len(raw_origin) != 3:
            raise MapManifestError(
                "occupancy_grid.origin must contain exactly three numbers"
            )
        declared_origin = [
            _finite_number(value, f"occupancy_grid.origin[{index}]")
            for index, value in enumerate(raw_origin)
        ]
        if any(
            not math.isclose(declared, actual, rel_tol=0.0, abs_tol=1e-12)
            for declared, actual in zip(declared_origin, yaml_origin)
        ):
            raise MapManifestError(
                "occupancy_grid.origin does not match occupancy YAML: "
                f"manifest={declared_origin}, yaml={yaml_origin}"
            )
    return yaml_resolution, width, height, yaml_origin


def load_map_manifest(
    manifest_path: str | Path,
    *,
    project_root: str | Path | None = None,
    verify_files: bool = True,
    enforce_manifest_path: bool = True,
) -> MapManifest:
    """Load and verify one indivisible four-artifact map bundle."""
    requested_source = Path(manifest_path).expanduser().absolute()
    root = (
        Path(project_root).expanduser().resolve()
        if project_root is not None and str(project_root).strip()
        else _infer_project_root(requested_source)
    )
    try:
        source_relative = requested_source.relative_to(root)
    except ValueError as exc:
        raise MapManifestError(
            f"map manifest escapes the project root: {requested_source}"
        ) from exc
    source = _safe_project_path(
        root, source_relative.as_posix(), "map manifest path"
    )
    document = _load_yaml(source, "map manifest")
    _reject_unknown(document, _ROOT_FIELDS, "map manifest root")
    if document.get("schema_version") != SCHEMA_VERSION:
        raise MapManifestError(
            f"map manifest schema_version must be {SCHEMA_VERSION}"
        )
    map_version = _validate_map_version(document.get("map_version"))
    expected_manifest = _safe_project_path(
        root,
        f"data/maps/manifests/{map_version}.yaml",
        "map manifest path",
    )
    if enforce_manifest_path and source != expected_manifest:
        raise MapManifestError(
            f"map manifest path must be {expected_manifest}, got {source}"
        )

    artifacts: list[MapArtifact] = []
    occupancy_section: Mapping[str, Any] | None = None
    for section_name in ("occupancy_grid", "pose_graph"):
        section = _mapping(document.get(section_name), section_name)
        _reject_unknown(
            section,
            _OCCUPANCY_FIELDS
            if section_name == "occupancy_grid"
            else _POSE_GRAPH_FIELDS,
            section_name,
        )
        if section_name == "occupancy_grid":
            occupancy_section = section
        if section_name == "pose_graph":
            expected_prefix = f"data/maps/posegraphs/{map_version}"
            if section.get("prefix") != expected_prefix:
                raise MapManifestError(
                    f"pose_graph.prefix must be {expected_prefix!r}"
                )
        files = section.get("files")
        if isinstance(files, (str, bytes)) or not isinstance(files, Sequence):
            raise MapManifestError(f"{section_name}.files must be a sequence")
        expected = [item for item in _ARTIFACT_LAYOUT if item[0] == section_name]
        if len(files) != len(expected):
            raise MapManifestError(
                f"{section_name}.files must contain exactly {len(expected)} artifacts"
            )
        for raw_entry, (_, role, path_template) in zip(files, expected):
            artifacts.append(
                _validate_artifact(
                    root,
                    section_name,
                    role,
                    path_template.format(version=map_version),
                    raw_entry,
                    verify_files=verify_files,
                )
            )

    declared_bundle = document.get("bundle_sha256")
    if not isinstance(declared_bundle, str) or not _SHA256.fullmatch(declared_bundle):
        raise MapManifestError("bundle_sha256 must be a lowercase SHA256 digest")
    actual_bundle = compute_bundle_sha256(
        [(item.role, item.path, item.bytes, item.sha256) for item in artifacts]
    )
    if declared_bundle != actual_bundle:
        raise MapManifestError(
            "map bundle SHA256 mismatch: "
            f"manifest={declared_bundle}, computed={actual_bundle}"
        )

    calibration_doc = _mapping(document.get("calibration"), "calibration")
    _reject_unknown(calibration_doc, _CALIBRATION_FIELDS, "calibration")
    calibrated = calibration_doc.get("calibrated")
    if not isinstance(calibrated, bool):
        raise MapManifestError("calibration.calibrated must be boolean")
    profile = calibration_doc.get("spawn_pose_profile")
    calibration_bundle = calibration_doc.get("bundle_sha256")
    calibrated_at = calibration_doc.get("calibrated_at")
    calibration_method = calibration_doc.get("calibration_method")
    usd_base_position: tuple[float, float, float] | None = None
    usd_base_yaw_deg: float | None = None
    map_base_position: tuple[float, float] | None = None
    map_base_yaw_deg: float | None = None
    position_stddev_m: float | None = None
    yaw_stddev_deg: float | None = None
    if calibrated:
        if not isinstance(profile, str) or not profile.strip():
            raise MapManifestError(
                "calibrated map requires calibration.spawn_pose_profile"
            )
        if calibration_bundle != declared_bundle:
            raise MapManifestError(
                "calibration.bundle_sha256 must equal the map bundle SHA256"
            )
        if not isinstance(calibrated_at, str) or not calibrated_at.strip():
            raise MapManifestError(
                "calibrated map requires calibration.calibrated_at"
            )
        if not isinstance(calibration_method, str) \
                or not calibration_method.strip():
            raise MapManifestError(
                "calibrated map requires calibration.calibration_method"
            )
        usd_position_values = _finite_vector(
            calibration_doc.get("usd_base_position"),
            3,
            "calibration.usd_base_position",
        )
        usd_base_position = (
            usd_position_values[0],
            usd_position_values[1],
            usd_position_values[2],
        )
        usd_base_yaw_deg = _finite_number(
            calibration_doc.get("usd_base_yaw_deg"),
            "calibration.usd_base_yaw_deg",
        )
        map_position_values = _finite_vector(
            calibration_doc.get("map_base_position"),
            2,
            "calibration.map_base_position",
        )
        map_base_position = (map_position_values[0], map_position_values[1])
        map_base_yaw_deg = _finite_number(
            calibration_doc.get("map_base_yaw_deg"),
            "calibration.map_base_yaw_deg",
        )
        position_stddev_m = _finite_number(
            calibration_doc.get("position_stddev_m"),
            "calibration.position_stddev_m",
        )
        yaw_stddev_deg = _finite_number(
            calibration_doc.get("yaw_stddev_deg"),
            "calibration.yaw_stddev_deg",
        )
        if position_stddev_m < 0.0 or yaw_stddev_deg < 0.0:
            raise MapManifestError(
                "calibration standard deviations must be non-negative"
            )
    elif profile is not None or calibration_bundle is not None:
        raise MapManifestError(
            "uncalibrated map must use null spawn_pose_profile and bundle_sha256"
        )
    elif calibrated_at is not None or calibration_method is not None:
        raise MapManifestError(
            "uncalibrated map must use null calibrated_at and calibration_method"
        )
    elif any(
        calibration_doc.get(field) is not None
        for field in (
            "usd_base_position",
            "usd_base_yaw_deg",
            "map_base_position",
            "map_base_yaw_deg",
            "position_stddev_m",
            "yaw_stddev_deg",
        )
    ):
        raise MapManifestError(
            "uncalibrated map must not declare calibrated pose values"
        )

    if verify_files:
        occupancy_yaml = _validate_occupancy_image_binding(
            artifacts[0].resolved_path, artifacts[1].resolved_path
        )
        assert occupancy_section is not None
        _validate_occupancy_metadata(
            occupancy_section, occupancy_yaml, artifacts[1].resolved_path
        )
    return MapManifest(
        source=source,
        project_root=root,
        map_version=map_version,
        bundle_sha256=declared_bundle,
        artifacts=tuple(artifacts),
        calibration=MapCalibration(
            calibrated=calibrated,
            spawn_pose_profile=profile,
            bundle_sha256=calibration_bundle,
            usd_base_position=usd_base_position,
            usd_base_yaw_deg=usd_base_yaw_deg,
            map_base_position=map_base_position,
            map_base_yaw_deg=map_base_yaw_deg,
            position_stddev_m=position_stddev_m,
            yaw_stddev_deg=yaw_stddev_deg,
        ),
    )


def validate_initial_pose_contract(
    manifest: MapManifest,
    *,
    initial_pose_source: str,
    spawn_poses_file: str | Path,
    spawn_pose_name: str,
) -> None:
    """Require auto initial pose to be calibrated for this exact bundle."""
    source = initial_pose_source.strip().lower()
    if source == "rviz":
        return
    if source != "auto":
        raise MapManifestError("initial_pose_source must be auto or rviz")
    if not manifest.calibration.calibrated:
        raise MapManifestError(
            f"map {manifest.map_version!r} is uncalibrated; "
            "use initial_pose_source=rviz and calibrate before enabling auto"
        )
    pose_name = spawn_pose_name.strip()
    if manifest.calibration.spawn_pose_profile != pose_name:
        raise MapManifestError(
            f"map {manifest.map_version!r} is calibrated for spawn pose "
            f"{manifest.calibration.spawn_pose_profile!r}, not {pose_name!r}"
        )
    requested_spawn_path = Path(spawn_poses_file).expanduser().absolute()
    if requested_spawn_path.is_symlink():
        raise MapManifestError(
            f"spawn pose configuration must not be a symlink: "
            f"{requested_spawn_path}"
        )
    spawn_path = requested_spawn_path.resolve()
    poses_document = _load_yaml(spawn_path, "spawn pose configuration")
    if poses_document.get("schema_version") != 1:
        raise MapManifestError("spawn pose schema_version must be 1")
    poses = _mapping(poses_document.get("spawn_poses"), "spawn_poses")
    pose = _mapping(poses.get(pose_name), f"spawn_poses.{pose_name}")
    usd_pose = _mapping(pose.get("usd"), f"spawn_poses.{pose_name}.usd")
    map_pose = _mapping(pose.get("map"), f"spawn_poses.{pose_name}.map")
    if map_pose.get("calibrated") is not True:
        raise MapManifestError(f"spawn pose {pose_name!r} is not calibrated")
    if map_pose.get("map_version") != manifest.map_version:
        raise MapManifestError(
            f"spawn pose {pose_name!r} map_version does not match "
            f"{manifest.map_version!r}"
        )
    if map_pose.get("map_bundle_sha256") != manifest.bundle_sha256:
        raise MapManifestError(
            f"spawn pose {pose_name!r} map_bundle_sha256 does not match "
            f"map {manifest.map_version!r}"
        )

    usd_position_values = _finite_vector(
        usd_pose.get("position"), 3, f"spawn_poses.{pose_name}.usd.position"
    )
    map_position_values = _finite_vector(
        map_pose.get("position"), 2, f"spawn_poses.{pose_name}.map.position"
    )
    scalar_values = {
        "usd.yaw_deg": _finite_number(
            usd_pose.get("yaw_deg"), f"spawn_poses.{pose_name}.usd.yaw_deg"
        ),
        "map.yaw_deg": _finite_number(
            map_pose.get("yaw_deg"), f"spawn_poses.{pose_name}.map.yaw_deg"
        ),
        "map.position_stddev_m": _finite_number(
            map_pose.get("position_stddev_m"),
            f"spawn_poses.{pose_name}.map.position_stddev_m",
        ),
        "map.yaw_stddev_deg": _finite_number(
            map_pose.get("yaw_stddev_deg"),
            f"spawn_poses.{pose_name}.map.yaw_stddev_deg",
        ),
    }
    expected_vectors = {
        "usd.position": manifest.calibration.usd_base_position,
        "map.position": manifest.calibration.map_base_position,
    }
    actual_vectors = {
        "usd.position": usd_position_values,
        "map.position": map_position_values,
    }
    for field, expected in expected_vectors.items():
        if expected is None or any(
            not math.isclose(actual, declared, rel_tol=0.0, abs_tol=1e-12)
            for actual, declared in zip(actual_vectors[field], expected)
        ):
            raise MapManifestError(
                f"spawn pose {pose_name!r} {field} does not match "
                "the map calibration"
            )
    expected_scalars = {
        "usd.yaw_deg": manifest.calibration.usd_base_yaw_deg,
        "map.yaw_deg": manifest.calibration.map_base_yaw_deg,
        "map.position_stddev_m": manifest.calibration.position_stddev_m,
        "map.yaw_stddev_deg": manifest.calibration.yaw_stddev_deg,
    }
    for field, expected in expected_scalars.items():
        if expected is None or not math.isclose(
            scalar_values[field], expected, rel_tol=0.0, abs_tol=1e-12
        ):
            raise MapManifestError(
                f"spawn pose {pose_name!r} {field} does not match "
                "the map calibration"
            )


def _source_artifact(
    role: str, logical_path: str, source: Path
) -> tuple[dict[str, object], tuple[str, str, int, str]]:
    if not source.is_file() or source.is_symlink():
        raise MapManifestError(
            f"staged {role} artifact is not a regular file: {source}"
        )
    with source.open("rb") as stream:
        if stream.read(len(_LFS_POINTER)).startswith(_LFS_POINTER):
            raise MapManifestError(f"staged {role} artifact is a Git LFS pointer")
    byte_count = source.stat().st_size
    if byte_count <= 0:
        raise MapManifestError(f"staged {role} artifact is empty: {source}")
    content_hash = _sha256(source)
    entry = {
        "role": role,
        "path": logical_path,
        "bytes": byte_count,
        "sha256": content_hash,
    }
    return entry, (role, logical_path, byte_count, content_hash)


def create_uncalibrated_manifest(
    *,
    project_root: str | Path,
    map_version: str,
    occupancy_yaml: str | Path,
    occupancy_image: str | Path,
    posegraph: str | Path,
    posegraph_data: str | Path,
    output: str | Path,
) -> Path:
    """Create a manifest for staged files; calibration is always false."""
    map_version = _validate_map_version(map_version)
    root = Path(project_root).expanduser().resolve()
    if not root.is_dir():
        raise MapManifestError(f"project root does not exist: {root}")
    source_inputs = (
        Path(occupancy_yaml).expanduser(),
        Path(occupancy_image).expanduser(),
        Path(posegraph).expanduser(),
        Path(posegraph_data).expanduser(),
    )
    sources_list = []
    for unresolved in source_inputs:
        requested = unresolved.absolute()
        try:
            relative = requested.relative_to(root)
        except ValueError as exc:
            raise MapManifestError(
                f"staged map artifact escapes project root: {requested}"
            ) from exc
        sources_list.append(
            _safe_project_path(
                root, relative.as_posix(), "staged map artifact path"
            )
        )
    sources = tuple(sources_list)
    occupancy_yaml_document = _validate_occupancy_image_binding(
        sources[0], sources[1]
    )
    resolution, width, height, origin = _validate_occupancy_metadata(
        {}, occupancy_yaml_document, sources[1]
    )
    entries: list[dict[str, object]] = []
    bundle_entries: list[tuple[str, str, int, str]] = []
    for source, (_, role, template) in zip(sources, _ARTIFACT_LAYOUT):
        entry, bundle_entry = _source_artifact(
            role, template.format(version=map_version), source
        )
        entries.append(entry)
        bundle_entries.append(bundle_entry)
    bundle = compute_bundle_sha256(bundle_entries)
    document = {
        "schema_version": SCHEMA_VERSION,
        "map_version": map_version,
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "availability": "local_generated",
        "artifact_policy": "manifest_commits_four_artifact_bundle",
        "bundle_sha256": bundle,
        "occupancy_grid": {
            "resolution_m": resolution,
            "width_cells": width,
            "height_cells": height,
            "origin": origin,
            "files": entries[:2],
        },
        "pose_graph": {
            "prefix": f"data/maps/posegraphs/{map_version}",
            "files": entries[2:],
        },
        "calibration": {
            "calibrated": False,
            "spawn_pose_profile": None,
            "bundle_sha256": None,
            "calibrated_at": None,
            "calibration_method": None,
        },
        "notes": [
            "The four generated artifacts are one indivisible map bundle.",
            "Auto initial pose is disabled until this exact bundle is calibrated.",
        ],
    }
    requested_destination = Path(output).expanduser().absolute()
    try:
        destination_relative = requested_destination.relative_to(root)
    except ValueError as exc:
        raise MapManifestError(
            f"manifest output escapes project root: {requested_destination}"
        ) from exc
    destination = _safe_project_path(
        root, destination_relative.as_posix(), "manifest output path"
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        raise MapManifestError(f"refusing to overwrite manifest: {destination}")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", dir=destination.parent
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, destination)
    except BaseException:
        Path(temporary_name).unlink(missing_ok=True)
        raise
    return destination


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    verify = subparsers.add_parser("verify")
    verify.add_argument("--project-root", required=True)
    verify.add_argument("--manifest", required=True)
    verify.add_argument("--allow-staged-manifest", action="store_true")
    create = subparsers.add_parser("create")
    create.add_argument("--project-root", required=True)
    create.add_argument("--map-version", required=True)
    create.add_argument("--occupancy-yaml", required=True)
    create.add_argument("--occupancy-image", required=True)
    create.add_argument("--posegraph", required=True)
    create.add_argument("--posegraph-data", required=True)
    create.add_argument("--output", required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    try:
        if arguments.command == "verify":
            manifest = load_map_manifest(
                arguments.manifest,
                project_root=arguments.project_root,
                enforce_manifest_path=not arguments.allow_staged_manifest,
            )
            print(
                f"map manifest verified: {manifest.map_version} "
                f"bundle={manifest.bundle_sha256}"
            )
        else:
            destination = create_uncalibrated_manifest(
                project_root=arguments.project_root,
                map_version=arguments.map_version,
                occupancy_yaml=arguments.occupancy_yaml,
                occupancy_image=arguments.occupancy_image,
                posegraph=arguments.posegraph,
                posegraph_data=arguments.posegraph_data,
                output=arguments.output,
            )
            print(f"created uncalibrated map manifest: {destination}")
    except MapManifestError as exc:
        raise SystemExit(f"map manifest error: {exc}") from exc
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
