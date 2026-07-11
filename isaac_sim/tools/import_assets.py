#!/usr/bin/env python3
"""Import checksum-pinned NVIDIA runtime assets without adding them to Git."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
from typing import Any


HERE = Path(__file__).resolve().parent
JACKAL_DIR = HERE.parent / "assets/robots/jackal"
MANIFEST = JACKAL_DIR / "asset_manifest.json"


class AssetImportError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_manifest(path: Path = MANIFEST) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or not isinstance(data.get("files"), list):
        raise AssetImportError(f"invalid asset manifest: {path}")
    for index, entry in enumerate(data["files"]):
        if set(entry) != {"source", "destination", "sha256"}:
            raise AssetImportError(f"invalid manifest file entry #{index}")
    return data


def verify_file(path: Path, expected: str) -> None:
    if not path.is_file():
        raise AssetImportError(f"missing asset: {path}")
    actual = sha256(path)
    if actual != expected:
        raise AssetImportError(f"checksum mismatch for {path}: expected {expected}, got {actual}")


def import_assets(asset_root: Path, *, check_only: bool = False, force: bool = False) -> list[Path]:
    manifest = load_manifest()
    outputs: list[Path] = []
    for entry in manifest["files"]:
        source = (asset_root / entry["source"]).resolve()
        destination = (JACKAL_DIR / entry["destination"]).resolve()
        try:
            destination.relative_to(JACKAL_DIR.resolve())
        except ValueError as exc:
            raise AssetImportError(f"destination escapes Jackal directory: {destination}") from exc
        if check_only:
            verify_file(destination, entry["sha256"])
            outputs.append(destination)
            continue
        verify_file(source, entry["sha256"])
        if destination.exists() and not force:
            verify_file(destination, entry["sha256"])
            outputs.append(destination)
            continue
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        shutil.copyfile(source, temporary)
        verify_file(temporary, entry["sha256"])
        temporary.replace(destination)
        outputs.append(destination)
    return outputs


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    manifest = load_manifest()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--asset-root",
        type=Path,
        default=Path(manifest["source_root_hint"]),
        help="Isaac asset pack root containing the Isaac/ directory",
    )
    parser.add_argument("--check", action="store_true", help="verify imported destinations without copying")
    parser.add_argument("--force", action="store_true", help="replace existing imported files after source verification")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        outputs = import_assets(args.asset_root, check_only=args.check, force=args.force)
    except AssetImportError as exc:
        print(f"asset import failed: {exc}", file=sys.stderr)
        return 2
    verb = "verified" if args.check else "imported"
    for output in outputs:
        print(f"{verb}: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
