import importlib.util
import json
from pathlib import Path


SCRIPT = Path(__file__).parents[4] / "scripts" / "audit_attempt30_a21_smac_geometry.py"
SPEC = importlib.util.spec_from_file_location("smac_geometry_audit", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_detects_duplicate_and_reversal_geometry():
    record = MODULE.diagnose([
        [0.0, 0.0], [1.0, 0.0], [1.0, 0.0], [0.8, 0.0], [1.2, 0.0]
    ])
    assert record["duplicate_steps"] == 1
    assert record["reversal_turns"] == 1
    assert record["maximum_turn_deg"] == 180.0


def test_accepts_monotonic_right_angle_path(tmp_path):
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    (run_dir / "run_manifest.json").write_text(json.dumps({
        "smac_plans": [{"points": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]}]
    }))
    report = MODULE.audit(run_dir)
    assert report["plan_count"] == 1
    assert report["defective_plan_count"] == 0
    assert report["maximum_turn_deg"] == 90.0
