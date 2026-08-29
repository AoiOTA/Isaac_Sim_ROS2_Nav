from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
import yaml

from robot_bringup.map_manifest import MapManifestError
from robot_bringup.map_manifest import compute_bundle_sha256
from robot_bringup.map_manifest import create_uncalibrated_manifest
from robot_bringup.map_manifest import load_map_manifest
from robot_bringup.map_manifest import validate_initial_pose_contract
from robot_bringup.mode_contract import validate_mode


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]


def _write_bundle(root: Path, version: str = "warehouse_v2") -> Path:
    occupancy = root / "data/maps/occupancy"
    posegraphs = root / "data/maps/posegraphs"
    manifests = root / "data/maps/manifests"
    occupancy.mkdir(parents=True)
    posegraphs.mkdir(parents=True)
    manifests.mkdir(parents=True)
    (occupancy / f"{version}.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (occupancy / f"{version}.yaml").write_text(
        f"image: {version}.pgm\n"
        "mode: trinary\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8",
    )
    (posegraphs / f"{version}.posegraph").write_bytes(b"posegraph")
    (posegraphs / f"{version}.data").write_bytes(b"data")
    return create_uncalibrated_manifest(
        project_root=root,
        map_version=version,
        occupancy_yaml=occupancy / f"{version}.yaml",
        occupancy_image=occupancy / f"{version}.pgm",
        posegraph=posegraphs / f"{version}.posegraph",
        posegraph_data=posegraphs / f"{version}.data",
        output=manifests / f"{version}.yaml",
    )


def _rewrite_manifest(path: Path, mutate) -> None:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    mutate(document)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _rehash_manifest(path: Path, root: Path) -> None:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    entries = [
        *document["occupancy_grid"]["files"],
        *document["pose_graph"]["files"],
    ]
    bundle_entries = []
    for entry in entries:
        artifact = root / entry["path"]
        entry["bytes"] = artifact.stat().st_size
        entry["sha256"] = hashlib.sha256(artifact.read_bytes()).hexdigest()
        bundle_entries.append(
            (entry["role"], entry["path"], entry["bytes"], entry["sha256"])
        )
    document["bundle_sha256"] = compute_bundle_sha256(bundle_entries)
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def _write_spawn_pose(
    path: Path,
    *,
    version: str,
    bundle: str,
    pose_name: str = "mapping_start",
) -> None:
    path.write_text(
        yaml.safe_dump(
            {
                "schema_version": 1,
                "spawn_poses": {
                    pose_name: {
                        "usd": {"position": [4.0, 0.0, 0.0635], "yaw_deg": 0.0},
                        "map": {
                            "calibrated": True,
                            "map_version": version,
                            "map_bundle_sha256": bundle,
                            "position": [0.0, 0.0],
                            "yaw_deg": 0.0,
                            "position_stddev_m": 0.05,
                            "yaw_stddev_deg": 5.0,
                        },
                    }
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )


def _calibrate_manifest(path: Path, pose_name: str = "mapping_start") -> str:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    bundle = document["bundle_sha256"]
    document["calibration"].update(
        {
            "calibrated": True,
            "spawn_pose_profile": pose_name,
            "bundle_sha256": bundle,
            "calibrated_at": "2026-07-13T00:00:00+00:00",
            "calibration_method": "test_fixture",
            "usd_base_position": [4.0, 0.0, 0.0635],
            "usd_base_yaw_deg": 0.0,
            "map_base_position": [0.0, 0.0],
            "map_base_yaw_deg": 0.0,
            "position_stddev_m": 0.05,
            "yaw_stddev_deg": 5.0,
        }
    )
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return bundle


def test_repository_warehouse_new_bundle_and_auto_pose_are_exactly_bound():
    manifest_path = REPOSITORY_ROOT / "data/maps/manifests/warehouse_new.yaml"
    manifest = load_map_manifest(manifest_path, project_root=REPOSITORY_ROOT)
    assert manifest.map_version == "warehouse_new"
    assert len(manifest.artifacts) == 4
    assert manifest.calibration.calibrated is True
    for source in ("auto", "isaac"):
        validate_initial_pose_contract(
            manifest,
            initial_pose_source=source,
            spawn_poses_file=(
                REPOSITORY_ROOT
                / "isaac_sim/configs/environments/"
                "kujiale_0026_A_to_B_door_open.spawn.yaml"
            ),
            spawn_pose_name="mapping_start",
        )


def test_repository_v6_isaacgen_bundle_and_auto_pose_are_exactly_bound():
    manifest_path = (
        REPOSITORY_ROOT / "data/maps/manifests/v6_kujiale_isaacgen_v1.yaml"
    )
    manifest = load_map_manifest(manifest_path, project_root=REPOSITORY_ROOT)
    assert manifest.map_version == "v6_kujiale_isaacgen_v1"
    assert len(manifest.artifacts) == 4
    assert manifest.calibration.calibrated is True
    spawn_poses = (
        REPOSITORY_ROOT
        / "isaac_sim/configs/environments/"
        "kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml"
    )
    for pose_name in (
        "mapping_start",
        "long_route_start_g1",
        "long_route_start_g2",
        "long_route_start_g5",
    ):
        validate_initial_pose_contract(
            manifest,
            initial_pose_source="auto",
            spawn_poses_file=spawn_poses,
            spawn_pose_name=pose_name,
        )
    # The pose graph is a byte-identical reuse of warehouse_new: in navigation
    # mode it is only a manifest/calibration binding, never deserialized.
    new_posegraph = (
        REPOSITORY_ROOT / "data/maps/posegraphs/v6_kujiale_isaacgen_v1.posegraph"
    ).read_bytes()
    old_posegraph = (
        REPOSITORY_ROOT / "data/maps/posegraphs/warehouse_new.posegraph"
    ).read_bytes()
    assert hashlib.sha256(new_posegraph).hexdigest() == hashlib.sha256(
        old_posegraph
    ).hexdigest()


def test_warehouse_v2_uncalibrated_auto_fails_fast_but_rviz_passes(tmp_path):
    manifest_path = _write_bundle(tmp_path)
    prefix = tmp_path / "data/maps/posegraphs/warehouse_v2"
    occupancy_yaml = tmp_path / "data/maps/occupancy/warehouse_v2.yaml"

    selection = validate_mode(
        "navigation",
        "ideal",
        "isaac",
        str(prefix),
        str(occupancy_yaml),
        map_manifest_file=str(manifest_path),
        project_root=str(tmp_path),
        initial_pose_source="rviz",
    )
    assert selection.map_version == "warehouse_v2"

    for source in ("auto", "isaac"):
        with pytest.raises(ValueError, match="warehouse_v2.*uncalibrated"):
            validate_mode(
                "navigation",
                "ideal",
                "isaac",
                str(prefix),
                str(occupancy_yaml),
                map_manifest_file=str(manifest_path),
                project_root=str(tmp_path),
                initial_pose_source=source,
                spawn_poses_file=str(tmp_path / "does-not-need-to-exist.yaml"),
            )


def test_calibrated_auto_requires_same_version_bundle_and_profile(tmp_path):
    manifest_path = _write_bundle(tmp_path)
    bundle = _calibrate_manifest(manifest_path)
    spawn_path = tmp_path / "spawn_poses.yaml"
    _write_spawn_pose(spawn_path, version="warehouse_v2", bundle=bundle)
    manifest = load_map_manifest(manifest_path, project_root=tmp_path)
    validate_initial_pose_contract(
        manifest,
        initial_pose_source="auto",
        spawn_poses_file=spawn_path,
        spawn_pose_name="mapping_start",
    )

    _write_spawn_pose(spawn_path, version="warehouse_v1", bundle=bundle)
    with pytest.raises(MapManifestError, match="map_version does not match"):
        validate_initial_pose_contract(
            manifest,
            initial_pose_source="auto",
            spawn_poses_file=spawn_path,
            spawn_pose_name="mapping_start",
        )
    _write_spawn_pose(spawn_path, version="warehouse_v2", bundle="0" * 64)
    with pytest.raises(MapManifestError, match="map_bundle_sha256 does not match"):
        validate_initial_pose_contract(
            manifest,
            initial_pose_source="auto",
            spawn_poses_file=spawn_path,
            spawn_pose_name="mapping_start",
        )
    with pytest.raises(MapManifestError, match="not 'different'"):
        validate_initial_pose_contract(
            manifest,
            initial_pose_source="auto",
            spawn_poses_file=spawn_path,
            spawn_pose_name="different",
        )


@pytest.mark.parametrize(
    ("section", "field", "value", "message"),
    [
        ("usd", "position", [4.1, 0.0, 0.0635], "usd.position"),
        ("usd", "yaw_deg", 1.0, "usd.yaw_deg"),
        ("map", "position", [0.1, 0.0], "map.position"),
        ("map", "yaw_deg", 1.0, "map.yaw_deg"),
        ("map", "position_stddev_m", 0.1, "map.position_stddev_m"),
        ("map", "yaw_stddev_deg", 10.0, "map.yaw_stddev_deg"),
    ],
)
def test_calibrated_auto_requires_exact_pose_and_uncertainty_values(
    tmp_path, section, field, value, message
):
    manifest_path = _write_bundle(tmp_path)
    bundle = _calibrate_manifest(manifest_path)
    spawn_path = tmp_path / "spawn_poses.yaml"
    _write_spawn_pose(spawn_path, version="warehouse_v2", bundle=bundle)
    document = yaml.safe_load(spawn_path.read_text(encoding="utf-8"))
    document["spawn_poses"]["mapping_start"][section][field] = value
    spawn_path.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )

    manifest = load_map_manifest(manifest_path, project_root=tmp_path)
    with pytest.raises(MapManifestError, match=message):
        validate_initial_pose_contract(
            manifest,
            initial_pose_source="auto",
            spawn_poses_file=spawn_path,
            spawn_pose_name="mapping_start",
        )


def test_calibrated_manifest_requires_complete_pose_values(tmp_path):
    manifest_path = _write_bundle(tmp_path)
    _calibrate_manifest(manifest_path)
    _rewrite_manifest(
        manifest_path,
        lambda document: document["calibration"].pop("map_base_position"),
    )
    with pytest.raises(MapManifestError, match="map_base_position"):
        load_map_manifest(manifest_path, project_root=tmp_path)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda document: document["occupancy_grid"]["files"][0].__setitem__(
                "bytes", 999
            ),
            "size mismatch",
        ),
        (
            lambda document: document["pose_graph"]["files"][1].__setitem__(
                "sha256", "0" * 64
            ),
            "SHA256 mismatch",
        ),
        (
            lambda document: document.__setitem__("bundle_sha256", "0" * 64),
            "bundle SHA256 mismatch",
        ),
        (
            lambda document: document["occupancy_grid"]["files"][0].__setitem__(
                "path", "../../outside.yaml"
            ),
            "path must be",
        ),
    ],
)
def test_manifest_rejects_size_sha_bundle_and_out_of_contract_paths(
    tmp_path, mutation, message
):
    manifest_path = _write_bundle(tmp_path)
    _rewrite_manifest(manifest_path, mutation)
    with pytest.raises(MapManifestError, match=message):
        load_map_manifest(manifest_path, project_root=tmp_path)


@pytest.mark.parametrize(
    ("mutation", "location"),
    [
        (
            lambda document: document.__setitem__("unknown", 1),
            "map manifest root",
        ),
        (
            lambda document: document["occupancy_grid"].__setitem__(
                "unknown", 1
            ),
            "occupancy_grid",
        ),
        (
            lambda document: document["pose_graph"].__setitem__("unknown", 1),
            "pose_graph",
        ),
        (
            lambda document: document["calibration"].__setitem__("unknown", 1),
            "calibration",
        ),
    ],
)
def test_manifest_rejects_unknown_fields_at_every_contract_level(
    tmp_path, mutation, location
):
    manifest_path = _write_bundle(tmp_path)
    _rewrite_manifest(manifest_path, mutation)
    with pytest.raises(MapManifestError, match=f"unknown {location}.*fields"):
        load_map_manifest(manifest_path, project_root=tmp_path)


@pytest.mark.parametrize("version", ["v" * 65, ".", "..", "..."])
def test_map_version_rejects_oversized_and_dot_only_names(tmp_path, version):
    with pytest.raises(MapManifestError, match="unsafe characters"):
        _write_bundle(tmp_path, version)


def test_manifest_rejects_unhydrated_lfs_artifact_before_size_fallback(tmp_path):
    manifest_path = _write_bundle(tmp_path)
    artifact = tmp_path / "data/maps/posegraphs/warehouse_v2.posegraph"
    artifact.write_bytes(
        b"version https://git-lfs.github.com/spec/v1\n"
        b"oid sha256:0000000000000000000000000000000000000000000000000000000000000000\n"
        b"size 9\n"
    )
    with pytest.raises(MapManifestError, match="unhydrated Git LFS pointer"):
        load_map_manifest(manifest_path, project_root=tmp_path)


def test_occupancy_yaml_must_bind_exact_manifested_pgm(tmp_path):
    manifest_path = _write_bundle(tmp_path)
    occupancy_yaml = tmp_path / "data/maps/occupancy/warehouse_v2.yaml"
    occupancy_yaml.write_text("image: wrong.pgm\n", encoding="utf-8")
    _rehash_manifest(manifest_path, tmp_path)
    with pytest.raises(MapManifestError, match="manifested PGM"):
        load_map_manifest(manifest_path, project_root=tmp_path)


@pytest.mark.parametrize(
    ("mutate_artifact", "message"),
    [
        (
            lambda root: (
                root / "data/maps/occupancy/warehouse_v2.yaml"
            ).write_text(
                "image: warehouse_v2.pgm\n"
                "resolution: 0.10\n"
                "origin: [0.0, 0.0, 0.0]\n",
                encoding="utf-8",
            ),
            "resolution_m does not match",
        ),
        (
            lambda root: (
                root / "data/maps/occupancy/warehouse_v2.yaml"
            ).write_text(
                "image: warehouse_v2.pgm\n"
                "resolution: 0.05\n"
                "origin: [1.0, 0.0, 0.0]\n",
                encoding="utf-8",
            ),
            "origin does not match",
        ),
        (
            lambda root: (
                root / "data/maps/occupancy/warehouse_v2.pgm"
            ).write_bytes(b"P5\n2 1\n255\n\x00\x00"),
            "width_cells does not match",
        ),
    ],
)
def test_declared_occupancy_metadata_matches_yaml_and_pgm_header(
    tmp_path, mutate_artifact, message
):
    manifest_path = _write_bundle(tmp_path)
    mutate_artifact(tmp_path)
    _rehash_manifest(manifest_path, tmp_path)
    with pytest.raises(MapManifestError, match=message):
        load_map_manifest(manifest_path, project_root=tmp_path)


@pytest.mark.parametrize("resolution", [0.0, -0.05])
def test_occupancy_resolution_must_be_positive(tmp_path, resolution):
    manifest_path = _write_bundle(tmp_path)
    occupancy_yaml = tmp_path / "data/maps/occupancy/warehouse_v2.yaml"
    document = yaml.safe_load(occupancy_yaml.read_text(encoding="utf-8"))
    document["resolution"] = resolution
    occupancy_yaml.write_text(
        yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
    )
    _rehash_manifest(manifest_path, tmp_path)
    with pytest.raises(MapManifestError, match="resolution.*positive"):
        load_map_manifest(manifest_path, project_root=tmp_path)


def test_manifest_location_and_input_pair_cannot_escape_or_cross_versions(tmp_path):
    manifest_path = _write_bundle(tmp_path)
    moved = tmp_path / "warehouse_v2.yaml"
    manifest_path.rename(moved)
    with pytest.raises(MapManifestError, match="map manifest path must be"):
        load_map_manifest(moved, project_root=tmp_path)

    moved.rename(manifest_path)
    linked = tmp_path / "linked-manifest.yaml"
    linked.symlink_to(manifest_path)
    with pytest.raises(MapManifestError, match="symlink"):
        load_map_manifest(linked, project_root=tmp_path)

    prefix = tmp_path / "data/maps/posegraphs/warehouse_v2"
    wrong_map = tmp_path / "data/maps/occupancy/other.yaml"
    wrong_map.write_text("image: other.pgm\n", encoding="utf-8")
    with pytest.raises(ValueError, match="map_file does not match"):
        validate_mode(
            "navigation",
            "ideal",
            "isaac",
            str(prefix),
            str(wrong_map),
            map_manifest_file=str(manifest_path),
            project_root=str(tmp_path),
            initial_pose_source="rviz",
        )


def test_manifest_parent_directory_symlink_is_rejected(tmp_path):
    project = tmp_path / "project"
    manifest_path = _write_bundle(project)
    manifests = manifest_path.parent
    external = tmp_path / "external_manifests"
    manifests.rename(external)
    manifests.symlink_to(external, target_is_directory=True)

    with pytest.raises(MapManifestError, match="traverses a symlink"):
        load_map_manifest(manifest_path, project_root=project)


def test_mode_rejects_user_alias_and_returns_manifest_canonical_paths(tmp_path):
    manifest_path = _write_bundle(tmp_path)
    canonical_prefix = tmp_path / "data/maps/posegraphs/warehouse_v2"
    canonical_map = tmp_path / "data/maps/occupancy/warehouse_v2.yaml"
    selection = validate_mode(
        "navigation",
        "ideal",
        "isaac",
        str(canonical_prefix),
        str(canonical_map),
        map_manifest_file=str(manifest_path),
        project_root=str(tmp_path),
        initial_pose_source="rviz",
    )
    assert selection.posegraph_prefix == str(canonical_prefix)
    assert selection.occupancy_map_file == str(canonical_map)

    alias = tmp_path / "posegraph_alias"
    alias.symlink_to(canonical_prefix.parent, target_is_directory=True)
    with pytest.raises(ValueError, match="posegraph_file does not match"):
        validate_mode(
            "navigation",
            "ideal",
            "isaac",
            str(alias / "warehouse_v2"),
            str(canonical_map),
            map_manifest_file=str(manifest_path),
            project_root=str(tmp_path),
            initial_pose_source="rviz",
        )


def test_create_manifest_is_uncalibrated_and_refuses_overwrite(tmp_path):
    manifest_path = _write_bundle(tmp_path)
    document = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    assert document["calibration"] == {
        "calibrated": False,
        "spawn_pose_profile": None,
        "bundle_sha256": None,
        "calibrated_at": None,
        "calibration_method": None,
    }
    assert document["occupancy_grid"]["resolution_m"] == 0.05
    assert document["occupancy_grid"]["width_cells"] == 1
    assert document["occupancy_grid"]["height_cells"] == 1
    assert document["occupancy_grid"]["origin"] == [0.0, 0.0, 0.0]
    with pytest.raises(MapManifestError, match="refusing to overwrite"):
        create_uncalibrated_manifest(
            project_root=tmp_path,
            map_version="warehouse_v2",
            occupancy_yaml=tmp_path / "data/maps/occupancy/warehouse_v2.yaml",
            occupancy_image=tmp_path / "data/maps/occupancy/warehouse_v2.pgm",
            posegraph=tmp_path / "data/maps/posegraphs/warehouse_v2.posegraph",
            posegraph_data=tmp_path / "data/maps/posegraphs/warehouse_v2.data",
            output=manifest_path,
        )
