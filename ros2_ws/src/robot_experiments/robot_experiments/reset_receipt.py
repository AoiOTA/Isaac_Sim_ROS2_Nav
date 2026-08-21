"""Parse and validate reset receipts returned by the Isaac Trigger service."""

from __future__ import annotations

import json
from typing import Any


class ResetReceiptError(RuntimeError):
    pass


_RECEIPT_MARKER = "reset_receipt="


def parse_reset_receipt(
    response_message: str,
    *,
    requested_seed: int,
    requested_case_id: str = "",
    requested_variant_id: str = "",
) -> dict[str, Any]:
    """Return complete provenance or fail closed on any request mismatch."""

    if not isinstance(response_message, str):
        raise ResetReceiptError("simulation reset response must be a string")
    message = response_message
    marker_at = message.find(_RECEIPT_MARKER)
    if marker_at < 0:
        raise ResetReceiptError("simulation reset response has no reset_receipt")
    encoded = message[marker_at + len(_RECEIPT_MARKER):].lstrip()
    try:
        payload, end = json.JSONDecoder().raw_decode(encoded)
    except json.JSONDecodeError as exc:
        raise ResetReceiptError(f"invalid simulation reset receipt: {exc}") from exc
    remainder = encoded[end:].lstrip()
    if remainder and not remainder.startswith(";"):
        raise ResetReceiptError(
            "invalid simulation reset receipt: trailing junk after JSON"
        )
    required = {"seed", "generation", "pose", "odometry", "case_id", "variant_id"}
    if not isinstance(payload, dict) or not required.issubset(payload):
        raise ResetReceiptError("simulation reset receipt is incomplete")
    actual_seed = payload["seed"]
    generation = payload["generation"]
    if (
        isinstance(actual_seed, bool)
        or not isinstance(actual_seed, int)
        or isinstance(generation, bool)
        or not isinstance(generation, int)
        or actual_seed < 0
        or generation < 1
    ):
        raise ResetReceiptError("simulation reset receipt seed/generation is invalid")
    string_fields = ("pose", "odometry", "case_id", "variant_id")
    if any(not isinstance(payload[field], str) for field in string_fields):
        raise ResetReceiptError("simulation reset receipt string fields are invalid")
    if not payload["pose"] or not payload["odometry"]:
        raise ResetReceiptError("simulation reset receipt pose/odometry is empty")
    if (
        isinstance(requested_seed, bool)
        or not isinstance(requested_seed, int)
        or requested_seed < 0
        or not isinstance(requested_case_id, str)
        or not isinstance(requested_variant_id, str)
    ):
        raise ResetReceiptError("requested reset provenance has invalid types")
    expected = (
        requested_seed, requested_case_id, requested_variant_id
    )
    actual = (
        actual_seed, payload["case_id"], payload["variant_id"]
    )
    if actual != expected:
        raise ResetReceiptError(
            "simulation reset receipt mismatch: "
            f"requested={expected}, actual={actual}, generation={generation}"
        )
    return {
        "requested_seed": requested_seed,
        "actual_seed": actual_seed,
        "generation": generation,
        "pose": payload["pose"],
        "odometry": payload["odometry"],
        "case_id": actual[1],
        "variant_id": actual[2],
        "full_response": message,
    }
