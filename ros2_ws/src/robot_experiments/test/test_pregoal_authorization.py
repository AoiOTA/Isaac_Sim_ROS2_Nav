from __future__ import annotations

import json

import pytest

from robot_experiments.configuration import ConfigurationError
from robot_experiments.experiment_runner import (
    PREGOAL_AUTHORIZATION_RECEIPT,
    _pregoal_identity,
    validate_pregoal_authorization,
)
from robot_experiments.scenario import RunSelection


def _selection() -> RunSelection:
    return RunSelection(
        seed=8201,
        case_id="static",
        variant_id="v1",
        condition_id="static_baseline",
        appearance_profile_id="baseline",
    )


def _receipt(selection: RunSelection) -> dict[str, object]:
    return {
        "pass": True,
        "receipt": PREGOAL_AUTHORIZATION_RECEIPT,
        "completed_wall_ns": 123,
        "identity": _pregoal_identity("kujiale_stage2_2_r2c4_r2_static", 1, selection),
    }


def test_pregoal_authorization_requires_passing_matching_identity(tmp_path):
    selection = _selection()
    path = tmp_path / "authorization.json"
    path.write_text(json.dumps(_receipt(selection)), encoding="utf-8")
    assert validate_pregoal_authorization(
        path,
        scenario_id="kujiale_stage2_2_r2c4_r2_static",
        run_index=1,
        selection=selection,
    )["pass"] is True

    value = _receipt(selection)
    value["identity"]["seed"] = 9999  # type: ignore[index]
    path.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="identity mismatch"):
        validate_pregoal_authorization(
            path,
            scenario_id="kujiale_stage2_2_r2c4_r2_static",
            run_index=1,
            selection=selection,
        )


def test_pregoal_authorization_rejects_missing_or_nonpassing_receipts(tmp_path):
    selection = _selection()
    with pytest.raises(ConfigurationError, match="missing"):
        validate_pregoal_authorization(
            tmp_path / "missing.json",
            scenario_id="kujiale_stage2_2_r2c4_r2_static",
            run_index=1,
            selection=selection,
        )


def test_pregoal_authorization_binds_r3_contract_fields(tmp_path):
    selection = _selection()
    path = tmp_path / "authorization.json"
    value = _receipt(selection)
    value.update({
        "receipt": "R2C4_R3_PREGOAL_AUTHORIZED",
        "schema": "bio_nav_stage2_2_r2c4_r3_toolchain_fenced_audit_v1",
        "campaign": "campaign-r3-r1",
        "prereg_sha256": "a" * 64,
    })
    path.write_text(json.dumps(value), encoding="utf-8")
    assert validate_pregoal_authorization(
        path, scenario_id="kujiale_stage2_2_r2c4_r2_static", run_index=1,
        selection=selection, expected_receipt="R2C4_R3_PREGOAL_AUTHORIZED",
        expected_schema="bio_nav_stage2_2_r2c4_r3_toolchain_fenced_audit_v1",
        expected_campaign="campaign-r3-r1", expected_prereg_sha256="a" * 64,
    )["pass"] is True
    value["campaign"] = "other"
    path.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="campaign mismatch"):
        validate_pregoal_authorization(
            path, scenario_id="kujiale_stage2_2_r2c4_r2_static", run_index=1,
            selection=selection, expected_receipt="R2C4_R3_PREGOAL_AUTHORIZED",
            expected_campaign="campaign-r3-r1",
        )
    path = tmp_path / "authorization.json"
    value = _receipt(selection)
    value["pass"] = False
    path.write_text(json.dumps(value), encoding="utf-8")
    with pytest.raises(ConfigurationError, match="not passing"):
        validate_pregoal_authorization(
            path,
            scenario_id="kujiale_stage2_2_r2c4_r2_static",
            run_index=1,
            selection=selection,
        )
