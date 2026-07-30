from __future__ import annotations

import math

from isaac_sim.src.experiment.collision_monitor import CONTACT_DIAGNOSTIC_SCHEMA, contact_diagnostic


def test_contact_diagnostic_preserves_raw_contact_identity_without_decoding() -> None:
    result = contact_diagnostic(
        {
            "in_contact": True,
            "time": 12.5,
            "physics_step": 42,
            "force": 9.0,
            "number_of_contacts": 1,
            "contacts": [
                {
                    "body0": 101,
                    "body1": 202,
                    "position": [1.0, 2.0, 3.0],
                    "normal": [0.0, 1.0, 0.0],
                    "impulse": [4.0, 5.0, 6.0],
                    "time": 12.5,
                    "dt": 1.0 / 60.0,
                }
            ],
        },
        12.5,
        raw_data_enabled=True,
        raw_data_error="",
    )

    assert result["schema"] == CONTACT_DIAGNOSTIC_SCHEMA
    assert result["raw_contact_data_enabled"] is True
    assert result["contacts"] == [
        {
            "body0": 101,
            "body1": 202,
            "position": [1.0, 2.0, 3.0],
            "normal": [0.0, 1.0, 0.0],
            "impulse": [4.0, 5.0, 6.0],
            "time": 12.5,
            "dt": 1.0 / 60.0,
        }
    ]


def test_contact_diagnostic_fails_closed_for_unserializable_or_missing_raw_data() -> None:
    result = contact_diagnostic(
        {"in_contact": True, "force": math.nan, "contacts": "not-a-list"},
        4.0,
        raw_data_enabled=False,
        raw_data_error="sensor_has_no_raw_contact_frame_api",
    )

    assert result["force"] is None
    assert result["contacts"] == []
    assert result["raw_contact_data_enabled"] is False
    assert result["raw_contact_data_error"] == "sensor_has_no_raw_contact_frame_api"
