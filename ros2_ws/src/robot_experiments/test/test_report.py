import csv
from copy import deepcopy
import hashlib
import json
import math

import pytest

from robot_experiments.report import (
    REPRODUCIBILITY_FIELDS,
    ReportValidationError,
    configuration_sha256,
    decode_hashed_contact_snapshot,
    validate_manifest,
    validate_runtime_provenance,
    write_run_report,
    write_strict_json_report,
)


def valid_runtime_provenance():
    wheel_colliders = [
        f"/World/Robot/wheel_{index}/collider" for index in range(4)
    ]
    ground_colliders = ["/Root/GroundPlane/CollisionPlane"]
    wheel_material_path = "/World/Looks/WheelPhysics"
    return {
        "verified": True,
        "schema_version": 3,
        "robot": {
            "config": {"path": "/repo/jackal.yaml", "sha256": "a" * 64},
            "asset": {"path": "/repo/jackal_nav.usda", "sha256": "b" * 64},
            "solver": {
                "position_iterations": 32,
                "velocity_iterations": 4,
                "stage_articulation_usd_readback_verified": True,
            },
        },
        "environment": {
            "id": "Warehouse",
            "project_stage": {
                "path": "/repo/navigation_scene.usda",
                "sha256": "c" * 64,
            },
            "source_asset": {
                "path": "/assets/warehouse.usd",
                "sha256": "d" * 64,
            },
            "asset_root": "/assets/Isaac/6.0",
            "asset_version": "6.0",
            "composed_root_layer_sha256": "e" * 64,
        },
        "simulation": {
            "navigation_mode": "mapping",
            "odometry_mode": "ideal",
            "physics_hz": 60.0,
        },
        "contact": {
            "profile_path": "/repo/isaac_sim/configs/physics/legacy_baseline.yaml",
            "profile_sha256": "1" * 64,
            "profile_id": "legacy-baseline",
            "profile_mode": "legacy_baseline",
            "overlay_identifier": "anon:0x123:contact_legacy-baseline.usda",
            "overlay_sha256": "2" * 64,
            "explicit_materials": False,
            "thresholds_authored": False,
            "scene": {
                "physics_scene_path": "/PhysicsScene",
                "friction_correlation_distance": 0.00025,
                "friction_offset_threshold": 0.0004,
                "friction_type": "patch",
            },
            "collider_contract": {
                "wheel_joint_names": ["fl", "fr", "rl", "rr"],
                "wheel_expected_count": 4,
                "ground_required_prim_paths": ground_colliders,
                "ground_semantic_classes": [],
                "ground_expected_enabled_count": 1,
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
                    "collider_path": ground_colliders[0],
                    "direct_physics_material_path": None,
                    "effective_physics_material_path": None,
                }
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
        },
        "git": {
            "commit": "f" * 40,
            "branch": "codex/navigation-quality-fidelity",
            "dirty": True,
        },
    }


def valid_runtime_provenance_v4():
    provenance = valid_runtime_provenance()
    provenance["schema_version"] = 4
    provenance["robot"]["kinematics"] = {
        "profile_id": "jackal_legacy_geometric_v1",
        "lifecycle": "stable_baseline",
        "wheel_radius_m": 0.098,
        "wheel_width_m": 0.040,
        "geometric_track_width_m": 0.37559,
        "effective_track_width_m": 0.37559,
        "controller_contract_verified": True,
    }
    return provenance


def canonical_path_sha256(paths):
    payload = json.dumps(
        sorted(paths),
        separators=(",", ":"),
        allow_nan=False,
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def valid_runtime_provenance_v5():
    provenance = valid_runtime_provenance_v4()
    provenance["schema_version"] = 5
    source_colliders = [
        "/Root/GroundPlane/CollisionPlane",
        "/Root/SM_floor_decal_01/SM_floor_decal_01",
        "/Root/SM_floor_decal_02/SM_floor_decal_02",
    ]
    target_colliders = [source_colliders[0]]
    disabled_colliders = source_colliders[1:]
    provenance["ground_topology"] = {
        "profile_path": (
            "/repo/isaac_sim/configs/ground_topologies/"
            "test_warehouse_target1_v1.yaml"
        ),
        "profile_sha256": "3" * 64,
        "profile_id": "test_warehouse_target1_v1",
        "environment_id": "Warehouse",
        "operation": "disable_non_target_colliders",
        "source_asset_path": "/assets/warehouse.usd",
        "source_asset_sha256": "d" * 64,
        "overlay_identifier": (
            "anon:0x456:ground_topology_test_warehouse_target1_v1.usda"
        ),
        "overlay_sha256": "4" * 64,
        "source_colliders": source_colliders,
        "source_collider_count": len(source_colliders),
        "source_collider_paths_sha256": canonical_path_sha256(
            source_colliders
        ),
        "target_colliders": target_colliders,
        "target_collider_count": len(target_colliders),
        "target_collider_paths_sha256": canonical_path_sha256(
            target_colliders
        ),
        "disabled_colliders": disabled_colliders,
        "disabled_collider_count": len(disabled_colliders),
        "disabled_collider_paths_sha256": canonical_path_sha256(
            disabled_colliders
        ),
        "stage_usd_readback_verified": True,
    }
    return provenance


def replace_topology_paths(topology, name, paths):
    topology[f"{name}_colliders"] = paths
    topology[f"{name}_collider_count"] = len(paths)
    topology[f"{name}_collider_paths_sha256"] = canonical_path_sha256(paths)


def valid_preserve_runtime_provenance_v5():
    provenance = valid_runtime_provenance_v5()
    topology = provenance["ground_topology"]
    target_colliders = topology["target_colliders"]
    topology["profile_path"] = (
        "/repo/isaac_sim/configs/ground_topologies/"
        "test_warehouse_combined1_v1.yaml"
    )
    topology["profile_sha256"] = "5" * 64
    topology["profile_id"] = "test_warehouse_combined1_v1"
    topology["operation"] = "preserve_source_colliders"
    topology["overlay_identifier"] = (
        "anon:0x457:ground_topology_test_warehouse_combined1_v1.usda"
    )
    topology["overlay_sha256"] = "6" * 64
    replace_topology_paths(topology, "source", target_colliders)
    replace_topology_paths(topology, "disabled", [])
    return provenance


def valid_manifest():
    return {
        "scenario_id": "static_a",
        "random_seed": 42,
        "map_version": "map-v1",
        "posegraph_version": "graph-v1",
        "robot_config_hash": "a" * 64,
        "nav2_config_hash": "b" * 64,
        "dynamic_runtime_contract": {
            "verified": True,
            "enabled": False,
            "config_sha256": "c" * 64,
            "obstacle_ids": ["crossing_box", "cart_proxy_b"],
        },
        "spawn_pose_name": "mapping_start",
        "usd_start_pose": {"position": [1.0, 2.0, 0.1], "yaw_deg": 0.0},
        "map_start_pose": {"position": [0.0, 0.0], "yaw_deg": 0.0},
        "goal_pose": {"position": [4.0, 1.0], "yaw_deg": 0.0},
        "obstacle_trajectories": [],
        "physics_dt": 1.0 / 60.0,
        "rtf": 1.0,
        "result": "success",
        "failure_reason": "",
        "metrics": {"path_length_m": 4.2},
    }


def test_configuration_sha256_streams_exact_file(tmp_path):
    config = tmp_path / "config.yaml"
    config.write_bytes(b"alpha: 1\n")
    assert configuration_sha256(config) == hashlib.sha256(b"alpha: 1\n").hexdigest()


def test_validate_manifest_requires_every_reproducibility_field():
    manifest = valid_manifest()
    del manifest[REPRODUCIBILITY_FIELDS[0]]
    with pytest.raises(ReportValidationError, match="missing reproducibility"):
        validate_manifest(manifest)


@pytest.mark.parametrize("bad_hash", ["short", "g" * 64, 123])
def test_validate_manifest_rejects_invalid_hashes(bad_hash):
    manifest = valid_manifest()
    manifest["robot_config_hash"] = bad_hash
    with pytest.raises(ReportValidationError, match="SHA256"):
        validate_manifest(manifest)


def test_validate_manifest_requires_verified_dynamic_runtime_contract():
    manifest = valid_manifest()
    manifest["dynamic_runtime_contract"]["verified"] = False
    with pytest.raises(ReportValidationError, match="runtime-verified"):
        validate_manifest(manifest)


def test_validate_runtime_provenance_accepts_a_complete_startup_snapshot():
    validate_runtime_provenance(valid_runtime_provenance())
    validate_runtime_provenance(valid_runtime_provenance_v4())
    validate_runtime_provenance(valid_runtime_provenance_v5())


@pytest.mark.parametrize("bad_version", [True, 3.0, "5", None, 2, 6])
def test_validate_runtime_provenance_requires_exact_integer_schema_version(
    bad_version,
):
    provenance = valid_runtime_provenance()
    provenance["schema_version"] = bad_version
    with pytest.raises(ReportValidationError, match="integer 3, 4, or 5"):
        validate_runtime_provenance(provenance)


@pytest.mark.parametrize(
    ("field", "bad_value", "message"),
    [
        ("profile_id", "bad/id", "path-safe"),
        ("lifecycle", "promoted", "unsupported"),
        ("wheel_radius_m", True, "finite positive"),
        ("wheel_width_m", math.nan, "NaN or infinity"),
        ("geometric_track_width_m", 0.0, "finite positive"),
        ("effective_track_width_m", -1.0, "finite positive"),
        ("controller_contract_verified", False, "must be true"),
    ],
)
def test_validate_runtime_provenance_v4_rejects_bad_kinematics(
    field, bad_value, message
):
    provenance = valid_runtime_provenance_v4()
    provenance["robot"]["kinematics"][field] = bad_value
    with pytest.raises(ReportValidationError, match=message):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_v5_accepts_preserved_topology():
    validate_runtime_provenance(valid_preserve_runtime_provenance_v5())


@pytest.mark.parametrize(
    ("version", "mutation"),
    [
        (3, "kinematics"),
        (4, "ground_topology"),
        (5, "missing_kinematics"),
        (5, "missing_ground_topology"),
    ],
)
def test_validate_runtime_provenance_rejects_version_field_confusion(
    version, mutation
):
    if version == 3:
        provenance = valid_runtime_provenance()
        provenance["robot"]["kinematics"] = deepcopy(
            valid_runtime_provenance_v4()["robot"]["kinematics"]
        )
    elif version == 4:
        provenance = valid_runtime_provenance_v4()
        provenance["ground_topology"] = deepcopy(
            valid_runtime_provenance_v5()["ground_topology"]
        )
    else:
        provenance = valid_runtime_provenance_v5()
        if mutation == "missing_kinematics":
            del provenance["robot"]["kinematics"]
        else:
            del provenance["ground_topology"]

    with pytest.raises(ReportValidationError, match="keys must be exactly"):
        validate_runtime_provenance(provenance)


@pytest.mark.parametrize(
    ("field", "bad_value", "message"),
    [
        ("profile_path", "relative.yaml", "absolute path"),
        ("profile_sha256", "A" * 64, "lowercase SHA256"),
        ("profile_id", "bad/id", "path-safe"),
        ("environment_id", "SimplePlane", "match runtime environment"),
        ("operation", [], "non-empty string"),
        ("operation", "hide_prims", "operation must be one of"),
        ("source_asset_path", "/assets/other.usd", "match runtime environment"),
        ("source_asset_sha256", "e" * 64, "match runtime environment"),
        ("overlay_identifier", "saved.usda", "anonymous layer"),
        ("overlay_sha256", "B" * 64, "lowercase SHA256"),
        ("stage_usd_readback_verified", False, "must be true"),
    ],
)
def test_validate_runtime_provenance_v5_rejects_bad_topology_identity(
    field, bad_value, message
):
    provenance = valid_runtime_provenance_v5()
    provenance["ground_topology"][field] = bad_value
    with pytest.raises(ReportValidationError, match=message):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_v5_requires_exact_topology_keys():
    provenance = valid_runtime_provenance_v5()
    provenance["ground_topology"]["unversioned_note"] = "not allowed"
    with pytest.raises(ReportValidationError, match="keys must be exactly"):
        validate_runtime_provenance(provenance)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        ("tuple", "JSON list"),
        ("unsorted", "must be sorted"),
        ("duplicate", "unique paths"),
        ("relative", "valid absolute USD prim paths"),
        ("count", "count must match"),
        ("bool_count", "positive integer"),
        ("hash", "canonical sorted path list"),
    ],
)
def test_validate_runtime_provenance_v5_rejects_bad_topology_path_evidence(
    mutation, message
):
    provenance = valid_runtime_provenance_v5()
    topology = provenance["ground_topology"]
    if mutation == "tuple":
        topology["source_colliders"] = tuple(topology["source_colliders"])
    elif mutation == "unsorted":
        topology["source_colliders"] = list(
            reversed(topology["source_colliders"])
        )
    elif mutation == "duplicate":
        topology["source_colliders"] = [
            topology["source_colliders"][0],
            topology["source_colliders"][0],
        ]
    elif mutation == "relative":
        topology["source_colliders"] = ["Root/not_absolute"]
    elif mutation == "count":
        topology["source_collider_count"] += 1
    elif mutation == "bool_count":
        topology["source_collider_count"] = True
    else:
        topology["source_collider_paths_sha256"] = "0" * 64

    with pytest.raises(ReportValidationError, match=message):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_v5_rejects_overlapping_topology_sets():
    provenance = valid_runtime_provenance_v5()
    topology = provenance["ground_topology"]
    overlap = sorted(
        topology["disabled_colliders"] + topology["target_colliders"]
    )
    replace_topology_paths(topology, "disabled", overlap)
    with pytest.raises(ReportValidationError, match="must be disjoint"):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_v5_requires_topology_source_partition():
    provenance = valid_runtime_provenance_v5()
    topology = provenance["ground_topology"]
    replace_topology_paths(
        topology,
        "disabled",
        topology["disabled_colliders"][:-1],
    )
    with pytest.raises(ReportValidationError, match="must partition source"):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_v5_enforces_topology_operation_contract():
    provenance = valid_runtime_provenance_v5()
    provenance["ground_topology"]["operation"] = "preserve_source_colliders"
    with pytest.raises(ReportValidationError, match="target equal source"):
        validate_runtime_provenance(provenance)

    provenance = valid_preserve_runtime_provenance_v5()
    provenance["ground_topology"]["operation"] = (
        "disable_non_target_colliders"
    )
    with pytest.raises(ReportValidationError, match="true source subset"):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_v5_locks_topology_to_contact_target():
    provenance = valid_runtime_provenance_v5()
    topology = provenance["ground_topology"]
    replacement_target = [topology["disabled_colliders"][0]]
    replacement_disabled = sorted(
        set(topology["source_colliders"]) - set(replacement_target)
    )
    replace_topology_paths(topology, "target", replacement_target)
    replace_topology_paths(topology, "disabled", replacement_disabled)
    with pytest.raises(ReportValidationError, match="must equal.*ground_colliders"):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_v5_requires_selectors_to_cover_target():
    provenance = valid_runtime_provenance_v5()
    provenance["contact"]["collider_contract"][
        "ground_required_prim_paths"
    ] = ["/Root/NotInTopology"]
    with pytest.raises(ReportValidationError, match="missing required paths"):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_v5_rejects_incomplete_path_only_selector():
    provenance = valid_runtime_provenance_v5()
    topology = provenance["ground_topology"]
    source = topology["source_colliders"]
    topology["operation"] = "preserve_source_colliders"
    replace_topology_paths(topology, "target", source)
    replace_topology_paths(topology, "disabled", [])

    contact = provenance["contact"]
    contact["ground_colliders"] = source
    contact["collider_contract"]["ground_expected_enabled_count"] = len(source)
    contact["collider_contract"]["ground_semantic_classes"] = []
    contact["ground_bindings"] = [
        {
            "collider_path": path,
            "direct_physics_material_path": None,
            "effective_physics_material_path": None,
        }
        for path in source
    ]
    with pytest.raises(ReportValidationError, match="complete target set"):
        validate_runtime_provenance(provenance)


def test_decode_hashed_contact_snapshot_requires_canonical_verified_json():
    contact = valid_runtime_provenance()["contact"]
    payload = json.dumps(
        contact,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    digest = hashlib.sha256(payload.encode("utf-8")).hexdigest()

    assert decode_hashed_contact_snapshot(payload, digest) == contact

    with pytest.raises(ReportValidationError, match="SHA256 mismatch"):
        decode_hashed_contact_snapshot(payload + " ", digest)
    with pytest.raises(ReportValidationError, match="canonical strict JSON"):
        pretty = json.dumps(contact, indent=2, sort_keys=True)
        decode_hashed_contact_snapshot(
            pretty,
            hashlib.sha256(pretty.encode("utf-8")).hexdigest(),
        )
    with pytest.raises(ReportValidationError, match="valid JSON"):
        invalid = "{not-json}"
        decode_hashed_contact_snapshot(
            invalid,
            hashlib.sha256(invalid.encode("utf-8")).hexdigest(),
        )


@pytest.mark.parametrize(
    ("path", "bad_value", "message"),
    [
        (("schema_version",), 2, "integer 3, 4, or 5"),
        (("robot", "config", "sha256"), "g" * 64, "SHA256"),
        (("robot", "solver", "velocity_iterations"), True, "integer"),
        (("robot", "solver", "velocity_iterations"), 0, "integer"),
        (("robot", "solver", "velocity_iterations"), 256, "integer"),
        (
            (
                "robot",
                "solver",
                "stage_articulation_usd_readback_verified",
            ),
            False,
            "must be true",
        ),
        (("environment", "id"), "bad/id", "path-safe"),
        (("environment", "source_asset", "path"), "", "non-empty"),
        (("simulation", "physics_hz"), 0.0, "positive"),
        (("contact", "profile_path"), "relative.yaml", "absolute path"),
        (("contact", "profile_sha256"), "g" * 64, "SHA256"),
        (("contact", "overlay_identifier"), "saved.usda", "anonymous"),
        (
            ("contact", "stage_usd_readback_verified"),
            False,
            "must be true",
        ),
        (
            ("contact", "scene", "physics_scene_path"),
            "PhysicsScene",
            "absolute prim path",
        ),
        (
            ("contact", "collider_contract", "wheel_expected_count"),
            3,
            "must be 4",
        ),
        (("git", "commit"), "z" * 40, "Git object id"),
        (("git", "dirty"), "false", "boolean"),
    ],
)
def test_validate_runtime_provenance_rejects_bad_evidence(
    path, bad_value, message
):
    provenance = deepcopy(valid_runtime_provenance())
    cursor = provenance
    for key in path[:-1]:
        cursor = cursor[key]
    cursor[path[-1]] = bad_value
    with pytest.raises(ReportValidationError, match=message):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_requires_runtime_verification():
    provenance = valid_runtime_provenance()
    provenance["verified"] = False
    with pytest.raises(ReportValidationError, match="runtime-verified"):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_rejects_tampered_collider_and_binding_sets():
    provenance = valid_runtime_provenance()
    provenance["contact"]["ground_colliders"] = [
        "/Root/GroundPlane/CollisionPlane",
        "/Root/GroundPlane/CollisionPlane",
    ]
    with pytest.raises(ReportValidationError, match="unique"):
        validate_runtime_provenance(provenance)

    provenance = valid_runtime_provenance()
    provenance["contact"]["wheel_bindings"][0]["collider_path"] = (
        "/World/Robot/not_a_wheel/collider"
    )
    with pytest.raises(ReportValidationError, match="one-to-one"):
        validate_runtime_provenance(provenance)


@pytest.mark.parametrize(
    ("mode", "explicit", "thresholds", "message"),
    [
        ("legacy_baseline", True, False, "legacy_baseline"),
        ("threshold_only", False, False, "threshold_only"),
        ("explicit_material", False, True, "explicit_material"),
    ],
)
def test_validate_runtime_provenance_rejects_mode_flag_disagreement(
    mode, explicit, thresholds, message
):
    provenance = valid_runtime_provenance()
    contact = provenance["contact"]
    contact["profile_mode"] = mode
    contact["explicit_materials"] = explicit
    contact["thresholds_authored"] = thresholds
    with pytest.raises(ReportValidationError, match=message):
        validate_runtime_provenance(provenance)


def test_validate_runtime_provenance_requires_explicit_binding_material_evidence():
    provenance = valid_runtime_provenance()
    contact = provenance["contact"]
    contact["profile_mode"] = "explicit_material"
    contact["explicit_materials"] = True
    contact["thresholds_authored"] = True
    contact["ground_material"] = deepcopy(contact["wheel_material"])
    contact["ground_material"]["material_path"] = "/World/Looks/GroundPhysics"
    contact["ground_material"]["static_friction"] = 0.5
    contact["ground_material"]["dynamic_friction"] = 0.5
    for material in (contact["wheel_material"], contact["ground_material"]):
        material["friction_combine_mode"] = "average"
        material["restitution_combine_mode"] = "average"
        material["friction_combine_mode_authored"] = True
        material["restitution_combine_mode_authored"] = True
    contact["ground_bindings"][0]["direct_physics_material_path"] = (
        "/World/Looks/WrongGroundPhysics"
    )
    contact["ground_bindings"][0]["effective_physics_material_path"] = (
        "/World/Looks/GroundPhysics"
    )

    with pytest.raises(ReportValidationError, match="direct binding"):
        validate_runtime_provenance(provenance)


@pytest.mark.parametrize("bad_value", [math.nan, math.inf, -math.inf])
def test_nan_and_infinity_are_rejected_recursively(bad_value):
    manifest = valid_manifest()
    manifest["metrics"] = {"nested": [0.0, bad_value]}
    with pytest.raises(ReportValidationError, match="NaN or infinity"):
        validate_manifest(manifest)


def test_json_and_csv_are_written_with_complete_fields_and_no_temp_files(tmp_path):
    manifest = valid_manifest()
    json_path, csv_path = write_run_report(manifest, tmp_path, "run-0001")

    assert json.loads(json_path.read_text()) == manifest
    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert set(REPRODUCIBILITY_FIELDS) <= set(rows[0])
    assert json.loads(rows[0]["metrics"]) == manifest["metrics"]
    assert not list(tmp_path.glob("*.tmp"))


def test_report_atomically_replaces_existing_files(tmp_path):
    first = valid_manifest()
    write_run_report(first, tmp_path, "same-run")
    second = valid_manifest()
    second["result"] = "failure"
    second["failure_reason"] = "timed_out"
    write_run_report(second, tmp_path, "same-run")
    assert json.loads((tmp_path / "same-run.json").read_text())["result"] == "failure"


def test_generic_strict_json_report_is_atomic_and_rejects_nonfinite_values(tmp_path):
    destination = tmp_path / "motion.json"
    assert write_strict_json_report({"result": "success"}, destination) == destination
    assert json.loads(destination.read_text()) == {"result": "success"}
    with pytest.raises(ReportValidationError, match="NaN or infinity"):
        write_strict_json_report({"metric": math.nan}, destination)
    assert json.loads(destination.read_text()) == {"result": "success"}
    assert not list(tmp_path.glob("*.tmp"))


@pytest.mark.parametrize("stem", ["", ".", "..", "../escape", "nested/run"])
def test_report_stem_cannot_escape_output_directory(tmp_path, stem):
    with pytest.raises(ValueError, match="path-safe"):
        write_run_report(valid_manifest(), tmp_path, stem)
