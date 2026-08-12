import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[4] / "scripts" / "render_attempt30_a21_pilot.py"
SPEC = importlib.util.spec_from_file_location("attempt30_a21_pilot_renderer", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
RENDERER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RENDERER)


def test_dynamic_success_reports_deviation_as_not_applicable() -> None:
    text = RENDERER.deviation_annotation({
        "kind": "dynamic",
        "strict_success": True,
        "path_deviation_percent": None,
        "planned_path_deviation_percent": None,
    })
    assert text == "deviation not applicable to dynamic runs"


def test_unfinished_static_run_keeps_failure_explanation() -> None:
    text = RENDERER.deviation_annotation({
        "kind": "static",
        "strict_success": False,
        "path_deviation_percent": None,
        "planned_path_deviation_percent": None,
    })
    assert text == "deviation unavailable (run did not finish)"
