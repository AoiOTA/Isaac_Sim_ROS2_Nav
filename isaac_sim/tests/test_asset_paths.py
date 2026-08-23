from __future__ import annotations

import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
ASSET_DIRECTORY = ROOT / "isaac_sim/assets/robots/jackal"
IMPORTER = ROOT / "isaac_sim/tools/import_assets.py"


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


def _fake_archived_importer(tmp_path: Path):
    module3 = tmp_path / "snapshot" / "m3_src"
    tools = module3 / "isaac_sim" / "tools"
    jackal = module3 / "isaac_sim" / "assets" / "robots" / "jackal"
    tools.mkdir(parents=True)
    jackal.mkdir(parents=True)
    shutil.copy2(IMPORTER, tools / "import_assets.py")

    asset_root = tmp_path / "authorized_assets"
    sources = {
        "Isaac/Robots/Clearpath/Jackal/jackal.usd": b"fake jackal layer\n",
        "Isaac/Robots/Clearpath/Jackal/configuration/schema.usd":
            b"fake schema layer\n",
    }
    for relative, content in sources.items():
        source = asset_root / relative
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_bytes(content)

    files = [
        ("Isaac/Robots/Clearpath/Jackal/jackal.usd",
         "source/jackal_original.usd"),
        ("Isaac/Robots/Clearpath/Jackal/configuration/schema.usd",
         "source/configuration/jackal_robot_schema.usd"),
        ("Isaac/Robots/Clearpath/Jackal/configuration/schema.usd",
         "configuration/jackal_robot_schema.usd"),
    ]
    manifest = {
        "schema_version": 1,
        "source_root_hint": str(asset_root),
        "files": [
            {
                "source": source,
                "destination": destination,
                "sha256": hashlib.sha256(sources[source]).hexdigest(),
            }
            for source, destination in files
        ],
    }
    (jackal / "asset_manifest.json").write_text(
        json.dumps(manifest), encoding="utf-8")
    return module3, asset_root, jackal, sources, files


def _run_importer(module3: Path, asset_root: Path, *arguments: str):
    return subprocess.run(
        [
            sys.executable,
            str(module3 / "isaac_sim" / "tools" / "import_assets.py"),
            "--asset-root",
            str(asset_root),
            *arguments,
        ],
        cwd=module3,
        text=True,
        capture_output=True,
        check=False,
    )


def test_archived_importer_materializes_full_closure_only_in_snapshot_and_is_idempotent(
        tmp_path):
    module3, asset_root, jackal, sources, files = _fake_archived_importer(
        tmp_path)
    live_before = {
        entry["destination"]: (
            (ASSET_DIRECTORY / entry["destination"]).exists(),
            ((ASSET_DIRECTORY / entry["destination"]).stat().st_ino
             if (ASSET_DIRECTORY / entry["destination"]).exists() else None),
        )
        for entry in json.loads(
            (ASSET_DIRECTORY / "asset_manifest.json").read_text())["files"]
    }

    imported = _run_importer(module3, asset_root)
    assert imported.returncode == 0, imported.stderr
    checked = _run_importer(module3, asset_root, "--check")
    assert checked.returncode == 0, checked.stderr
    first_inodes = {}
    for source, destination in files:
        output = jackal / destination
        assert output.read_bytes() == sources[source]
        first_inodes[destination] = output.stat().st_ino

    repeated = _run_importer(module3, asset_root)
    assert repeated.returncode == 0, repeated.stderr
    assert {
        destination: (jackal / destination).stat().st_ino
        for _, destination in files
    } == first_inodes
    live_after = {
        destination: (
            (ASSET_DIRECTORY / destination).exists(),
            ((ASSET_DIRECTORY / destination).stat().st_ino
             if (ASSET_DIRECTORY / destination).exists() else None),
        )
        for destination in live_before
    }
    assert live_after == live_before


def test_archived_importer_rejects_missing_root_or_manifest_source(tmp_path):
    module3, asset_root, jackal, _, _ = _fake_archived_importer(tmp_path)

    missing_root = _run_importer(module3, tmp_path / "missing_assets")
    assert missing_root.returncode == 2
    assert "missing asset" in missing_root.stderr
    assert not (jackal / "source" / "jackal_original.usd").exists()

    (asset_root / "Isaac/Robots/Clearpath/Jackal/jackal.usd").unlink()
    missing_source = _run_importer(module3, asset_root)
    assert missing_source.returncode == 2
    assert "missing asset" in missing_source.stderr
    assert not (jackal / "source" / "jackal_original.usd").exists()
