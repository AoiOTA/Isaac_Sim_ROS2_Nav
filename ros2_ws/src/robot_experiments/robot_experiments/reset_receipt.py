"""Parse and validate reset receipts returned by the Isaac Trigger service."""

from __future__ import annotations

import json
import re
from typing import Any


class ResetReceiptError(RuntimeError):
    pass


_RECEIPT_PATTERN = re.compile(r"reset_receipt=(\{.*?\})(?:;|$)")


def parse_reset_receipt(
    response_message: str,
    *,
    requested_seed: int,
    requested_case_id: str = "",
    requested_variant_id: str = "",
) -> dict[str, Any]:
    """Return complete provenance or fail closed on any request mismatch."""

    message = str(response_message)
    match = _RECEIPT_PATTERN.search(message)
    if match is None:
        raise ResetReceiptError("simulation reset response has no reset_receipt")
    try:
        payload = json.loads(match.group(1))
    except json.JSONDecodeError as exc:
        raise ResetReceiptError(f"invalid simulation reset receipt: {exc}") from exc
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
        or generation < 1
    ):
        raise ResetReceiptError("simulation reset receipt seed/generation is invalid")
    expected = (
        int(requested_seed), str(requested_case_id), str(requested_variant_id)
    )
    actual = (
        actual_seed, str(payload["case_id"]), str(payload["variant_id"])
    )
    if actual != expected:
        raise ResetReceiptError(
            "simulation reset receipt mismatch: "
            f"requested={expected}, actual={actual}, generation={generation}"
        )
    return {
        "requested_seed": int(requested_seed),
        "actual_seed": actual_seed,
        "generation": generation,
        "pose": str(payload["pose"]),
        "odometry": str(payload["odometry"]),
        "case_id": actual[1],
        "variant_id": actual[2],
        "full_response": message,
    }
