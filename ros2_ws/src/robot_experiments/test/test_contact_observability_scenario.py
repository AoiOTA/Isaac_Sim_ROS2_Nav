from __future__ import annotations

from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def scenario(name: str) -> dict:
    return yaml.safe_load((ROOT / "config" / name).read_text(encoding="utf-8"))["scenario"]


def test_contact_observability_has_one_fresh_non_confirmation_run() -> None:
    diagnostic = scenario("kujiale_contact_observability_dynamic.yaml")
    matrix = diagnostic["runs"]["matrix"]

    assert diagnostic["id"] == "kujiale_contact_observability_dynamic"
    assert matrix == [
        {
            "seed": 10403,
            "case_id": "full_route_three_stage",
            "variant_id": "v2",
            "condition_id": "dynamic_baseline",
            "appearance_profile_id": "baseline",
        }
    ]
