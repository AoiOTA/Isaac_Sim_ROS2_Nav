from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ASSET_DIRECTORY = ROOT / "isaac_sim/assets/robots/jackal"


def test_import_manifest_is_reproducible_and_local():
    manifest = json.loads((ASSET_DIRECTORY / "asset_manifest.json").read_text())
    assert manifest["schema_version"] == 1
    assert manifest["files"]
    destinations = []
    for entry in manifest["files"]:
        destination = Path(entry["destination"])
        assert not destination.is_absolute()
        assert ".." not in destination.parts
        assert len(entry["sha256"]) == 64
        destinations.append(entry["destination"])
        imported = ASSET_DIRECTORY / destination
        if imported.exists():
            digest = hashlib.sha256(imported.read_bytes()).hexdigest()
            assert digest == entry["sha256"]
    assert len(destinations) == len(set(destinations))


def test_project_overlay_references_only_local_imports():
    overlay = (ASSET_DIRECTORY / "jackal_nav.usda").read_text()
    assert "defaultPrim = \"jackal\"" in overlay
    assert "@./source/jackal_original.usd@" in overlay
    assert "/home/" not in overlay
