from __future__ import annotations

import pytest

from robot_experiments.reset_receipt import (
    ResetReceiptError,
    parse_reset_receipt,
)


def _message(seed: int, generation: int, case: str = "", variant: str = "") -> str:
    return (
        "simulation reset transaction complete; "
        'reset_receipt={"case_id":"' + case
        + '","generation":' + str(generation)
        + ',"odometry":"realistic","pose":"mapping_start","seed":'
        + str(seed) + ',"variant_id":"' + variant + '"}; reset_event emitted'
    )


def test_seed_can_change_between_reset_generations():
    first = parse_reset_receipt(_message(8601, 1), requested_seed=8601)
    second = parse_reset_receipt(_message(8602, 2), requested_seed=8602)
    assert (first["actual_seed"], first["generation"]) == (8601, 1)
    assert (second["actual_seed"], second["generation"]) == (8602, 2)
    assert first["full_response"].startswith("simulation reset")


def test_seed_mismatch_is_a_fail_closed_stop():
    with pytest.raises(ResetReceiptError, match="receipt mismatch"):
        parse_reset_receipt(
            _message(8601, 7, "crossing", "medium"),
            requested_seed=8602,
            requested_case_id="crossing",
            requested_variant_id="medium",
        )


def test_case_variant_and_full_response_are_preserved():
    message = _message(8601, 3, "crossing", "medium")
    receipt = parse_reset_receipt(
        message,
        requested_seed=8601,
        requested_case_id="crossing",
        requested_variant_id="medium",
    )
    assert receipt["case_id"] == "crossing"
    assert receipt["variant_id"] == "medium"
    assert receipt["full_response"] == message


def test_case_and_variant_may_contain_delimiter_and_escaped_quotes():
    case = 'valid};case "east"'
    variant = 'variant};with "quotes"'
    message = (
        "complete; reset_receipt="
        '{"case_id":"valid};case \\"east\\"","generation":4,'
        '"odometry":"realistic","pose":"mapping_start","seed":8601,'
        '"variant_id":"variant};with \\"quotes\\""}; suffix'
    )
    receipt = parse_reset_receipt(
        message,
        requested_seed=8601,
        requested_case_id=case,
        requested_variant_id=variant,
    )
    assert receipt["case_id"] == case
    assert receipt["variant_id"] == variant


@pytest.mark.parametrize(
    "message, error",
    [
        ("no receipt here", "no reset_receipt"),
        (
            'reset_receipt={"seed":1}junk',
            "trailing junk",
        ),
        (
            'reset_receipt={"case_id":7,"generation":1,'
            '"odometry":"realistic","pose":"mapping_start","seed":1,'
            '"variant_id":""}',
            "string fields",
        ),
        (
            'reset_receipt={"case_id":"","generation":true,'
            '"odometry":"realistic","pose":"mapping_start","seed":1,'
            '"variant_id":""}',
            "seed/generation",
        ),
    ],
)
def test_malformed_or_wrong_typed_receipts_are_rejected(message, error):
    with pytest.raises(ResetReceiptError, match=error):
        parse_reset_receipt(message, requested_seed=1)
