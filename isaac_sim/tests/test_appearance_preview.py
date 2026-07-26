from datetime import datetime, timezone
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "isaac_sim/apps/appearance_preview.py"
SPEC = importlib.util.spec_from_file_location("appearance_preview", SOURCE)
assert SPEC is not None and SPEC.loader is not None
preview = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preview)


def test_preview_defaults_cover_every_campaign_profile_in_contract_order():
    assert preview.selected_profiles(None) == (
        "baseline",
        "dim_warm",
        "dim_cool",
        "bright_warm",
        "bright_cool",
    )
    assert preview.selected_profiles(["bright_cool", "baseline", "baseline"]) == (
        "baseline",
        "bright_cool",
    )


def test_preview_output_refuses_to_replace_existing_content(tmp_path, monkeypatch):
    monkeypatch.setattr(preview, "DEFAULT_OUTPUT_ROOT", tmp_path / "generated")
    target = tmp_path / "preview"
    resolved = preview.resolve_output_dir(
        None,
        now=datetime(2026, 7, 26, tzinfo=timezone.utc),
    )
    assert resolved.name == "kujiale_appearance_20260726T000000Z"

    target.mkdir()
    (target / "existing.png").write_bytes(b"not an image")
    with pytest.raises(ValueError, match="new or empty"):
        preview.resolve_output_dir(target)


def test_preview_gallery_is_clickable_and_identifies_living_room_view():
    page = preview.preview_html(("baseline", "bright_warm"), 1920, 1080)
    assert 'href="baseline.png"' in page
    assert 'src="baseline.png"' in page
    assert 'target="_blank"' in page
    assert "不是小车前向 RGB-D 图" in page
    assert "固定客厅观察位" in page
    assert "1920×1080" in page


def test_kit_only_arguments_are_not_part_of_preview_argparse_contract(monkeypatch):
    captured = {}

    def fake_run(args):
        captured["output_dir"] = args.output_dir
        return 0

    monkeypatch.setattr(preview, "run", fake_run)
    assert (
        preview.main(
            (
                "--output-dir",
                "/tmp/preview",
                "--/crashreporter/skipOldDumpUpload=1",
                "--/app/skipOldDumpUpload=true",
            )
        )
        == 0
    )
    assert captured["output_dir"] == Path("/tmp/preview")
