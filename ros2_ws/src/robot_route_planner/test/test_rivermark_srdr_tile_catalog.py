from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "scripts/build_rivermark_srdr_tile_catalog.py"
SPEC = importlib.util.spec_from_file_location("rivermark_srdr_tile_catalog", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
GENERATOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GENERATOR)

DATA = ROOT / "data/rivermark_demo"
MAP_VERSION = "ba4f3fafb54d3b6e886a4dc9f933c94899ce83b176877f3726b73f2aed9ed4dc"
COUNTS = {
    "02": (87, 234), "03": (21, 52), "04": (27, 78),
    "06": (223, 740), "07": (192, 604), "08": (64, 182),
    "09": (92, 266), "10": (160, 472), "11": (87, 278),
    "13": (203, 646), "14": (239, 852), "15": (216, 722),
    "16": (227, 764), "17": (236, 832), "21": (223, 754),
    "22": (204, 728), "23": (205, 738), "24": (216, 722),
    "27": (85, 266), "28": (226, 744), "29": (231, 782),
    "30": (223, 776), "31": (236, 878), "34": (82, 236),
    "35": (202, 608), "36": (197, 626), "37": (192, 538),
    "42": (62, 182), "43": (97, 308), "44": (34, 92),
}
NEIGHBORS = {
    "02": ("07",), "03": ("04", "10"), "04": ("03", "11"),
    "06": ("07", "13"), "07": ("02", "06", "08", "14"),
    "08": ("07", "09", "15"), "09": ("08", "10", "16"),
    "10": ("03", "09", "11", "17"), "11": ("04", "10"),
    "13": ("06", "14", "21"), "14": ("07", "13", "15", "22"),
    "15": ("08", "14", "16", "23"), "16": ("09", "15", "17", "24"),
    "17": ("10", "16"), "21": ("13", "22", "29"),
    "22": ("14", "21", "23", "30"), "23": ("15", "22", "24", "31"),
    "24": ("16", "23"), "27": ("28", "34"),
    "28": ("27", "29", "35"), "29": ("21", "28", "30", "36"),
    "30": ("22", "29", "31", "37"), "31": ("23", "30"),
    "34": ("27", "35", "42"), "35": ("28", "34", "36", "43"),
    "36": ("29", "35", "37", "44"), "37": ("30", "36"),
    "42": ("34", "43"), "43": ("35", "42", "44"),
    "44": ("36", "43"),
}
CONSTRAINT_FIELDS = {
    "grid_width", "grid_height", "resolution_m", "map_version",
    "cognitive_tile_id", "tile_revision", "graph_revision", "model_id",
    "valid_state_count", "verified_directed_transition_count",
    "T_map_canvas", "valid_state_mask", "verified_transitions",
}


def _arguments(output_root: Path, selection: Path | None = None) -> dict:
    return {
        "map_yaml": DATA / "rivermark_selected.yaml",
        "graph_geojson": DATA / "rivermark_selected.geojson",
        "regions_yaml": DATA / "rivermark_regions.yaml",
        "selection_yaml": selection or DATA / "rivermark_srdr_tile_selection_v1.yaml",
        "output_root": output_root,
    }


@pytest.fixture(scope="module")
def generated(tmp_path_factory: pytest.TempPathFactory) -> tuple[Path, Path, dict]:
    temporary = tmp_path_factory.mktemp("rivermark_catalog")
    first = temporary / "first"
    second = temporary / "second"
    catalog = GENERATOR.build_catalog(**_arguments(first))
    GENERATOR.build_catalog(**_arguments(second))
    return first, second, catalog


def test_catalog_freezes_selection_geometry_counts_and_route_coverage(generated) -> None:
    first, _, catalog = generated
    assert catalog["schema"] == "bio_nav.v310.srdr_tile_catalog.v1"
    assert catalog["map_version"] == MAP_VERSION
    assert catalog["graph_id"] == "rivermark_selected:gvg_v1"
    assert catalog["region_count"] == 30
    assert len(catalog["route_region_ids"]) == 16
    assert len(catalog["ring_region_ids"]) == 14
    assert catalog["route_coverage"]["outside_catalog_region_ids"] == []
    assert catalog["route_coverage"]["outside_catalog_m"] == 0.0
    assert catalog["route_coverage"]["waypoint_region_ids"] == {
        "start": "rivermark_a:region_03", "G1": "rivermark_a:region_09",
        "G2": "rivermark_a:region_14", "G3": "rivermark_a:region_30",
        "G4": "rivermark_a:region_35", "G5": "rivermark_a:region_43",
    }
    assert catalog["route_coverage"]["static_obstacle_region_id"].endswith("09")
    assert catalog["route_coverage"]["dynamic_region_id"].endswith("22")

    entries = {row["region_id"].rsplit("_", 1)[-1]: row for row in catalog["entries"]}
    assert set(entries) == set(COUNTS)
    for suffix, (valid, directed) in COUNTS.items():
        entry = entries[suffix]
        assert entry["region_id"] == entry["cognitive_tile_id"]
        assert (entry["valid_state_count"], entry["verified_directed_transition_count"]) == (valid, directed)
        assert tuple(value.rsplit("_", 1)[-1] for value in entry["neighbor_tile_ids"]) == NEIGHBORS[suffix]
        assert not Path(entry["constraints_relpath"]).is_absolute()
        assert not Path(entry["snapshot_relpath"]).is_absolute()
        assert entry["snapshot_relpath"] == f"snapshots/region_{suffix}"
        assert (first / entry["constraints_relpath"]).is_file()
    assert entries["22"]["valid_state_count"] == 204
    assert entries["22"]["verified_directed_transition_count"] == 728


def test_constraints_are_strict_cardinal_in_mask_and_table_consistent(generated) -> None:
    first, _, catalog = generated
    for entry in catalog["entries"]:
        payload = json.loads((first / entry["constraints_relpath"]).read_text())
        assert set(payload) == CONSTRAINT_FIELDS
        assert (payload["grid_width"], payload["grid_height"], payload["resolution_m"]) == (16, 16, 1.0)
        assert payload["map_version"] == MAP_VERSION
        assert payload["cognitive_tile_id"] == entry["region_id"]
        assert payload["tile_revision"] == payload["graph_revision"] == 1
        assert payload["model_id"] == "module2_srdr_v310_seed20260822"
        mask = payload["valid_state_mask"]
        assert len(mask) == 256 and all(type(value) is bool for value in mask)
        assert sum(mask) == payload["valid_state_count"] == entry["valid_state_count"]
        transitions = payload["verified_transitions"]
        assert len(transitions) == payload["verified_directed_transition_count"] == entry["verified_directed_transition_count"]
        assert len({tuple(edge) for edge in transitions}) == len(transitions)
        for source, target in transitions:
            assert mask[source] and mask[target]
            sr, sc = divmod(source, 16)
            tr, tc = divmod(target, 16)
            assert abs(sr - tr) + abs(sc - tc) == 1


def test_generation_is_byte_deterministic(generated) -> None:
    first, second, _ = generated
    first_files = sorted(path.relative_to(first) for path in first.rglob("*") if path.is_file())
    second_files = sorted(path.relative_to(second) for path in second.rglob("*") if path.is_file())
    assert first_files == second_files
    assert all((first / path).read_bytes() == (second / path).read_bytes() for path in first_files)


def test_generator_rejects_nonfresh_root_and_stale_map_identity(generated, tmp_path: Path) -> None:
    first, _, _ = generated
    with pytest.raises(FileExistsError, match="already exists"):
        GENERATOR.build_catalog(**_arguments(first))

    selection = yaml.safe_load((DATA / "rivermark_srdr_tile_selection_v1.yaml").read_text())
    selection["expected_map_version"] = "0" * 64
    stale = tmp_path / "stale.yaml"
    stale.write_text(yaml.safe_dump(selection, sort_keys=False), encoding="utf-8")
    # Keep relative references valid after moving the test selection file.
    for key, value in selection["references"].items():
        selection["references"][key] = str((DATA / value).resolve())
    stale.write_text(yaml.safe_dump(selection, sort_keys=False), encoding="utf-8")
    rejected_output = tmp_path / "rejected"
    with pytest.raises(ValueError, match="expected_map_version"):
        GENERATOR.build_catalog(**_arguments(rejected_output, stale))
    assert not rejected_output.exists()
