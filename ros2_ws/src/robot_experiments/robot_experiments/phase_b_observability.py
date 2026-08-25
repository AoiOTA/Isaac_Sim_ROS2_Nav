"""Minimal recorder/plot inputs for the R5 Kujiale Phase B baseline."""

from __future__ import annotations

import argparse
from typing import Any


PHASE_B_RECORDER_TOPICS = (
    "/clock",
    "/camera/front/image_raw",
    "/wheel/odom",
    "/imu/data_raw",
    "/imu/data",
    "/ground_truth/odom",
    "/odom",
    "/amcl_pose",
    "/bio_nav/module1/odom",
    "/bio_nav/cognitive_map/constraints",
    "/bio_nav/module2/planning_prior",
    "/bio_nav/navigation_graph",
    "/bio_nav/canonical_route",
    "/bio_nav/route_progress",
    "/bio_nav/route_goal_complete",
    "/bio_nav/route_goal_result",
    "/cmd_vel",
    "/cmd_vel_nav",
    "/cmd_vel_sim",
    "/simulation/reset_event",
    "/simulation/reset_stop_gate/status",
    "/simulation/collision",
    "/simulation/collision_diagnostics",
    "/scan",
    "/tf",
    "/tf_static",
)


def _vector(message: Any, name: str, size: int) -> list[float]:
    values = [float(value) for value in getattr(message, name)]
    if len(values) != size:
        raise ValueError(f"PlanningPrior.{name} must contain {size} values")
    return values


def _bool_vector(message: Any, name: str, size: int) -> list[bool]:
    values = [bool(value) for value in getattr(message, name)]
    if len(values) != size:
        raise ValueError(f"PlanningPrior.{name} must contain {size} values")
    return values


def planning_prior_localization_sample(message: Any) -> dict[str, Any]:
    """Extract all fields needed to plot the fifth localization trajectory."""

    stamp = getattr(message, "stamp")
    return {
        "stamp_ns": int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
        "sequence": int(message.sequence),
        "map_version": str(message.map_version),
        "cognitive_tile_id": str(message.cognitive_tile_id),
        "t_map_canvas": _vector(message, "t_map_canvas", 9),
        "valid_state_mask": _bool_vector(message, "valid_state_mask", 256),
        "place_belief": _vector(message, "place_belief", 256),
        "place_mean_canvas_m": _vector(message, "place_mean_canvas_m", 2),
        "heading_belief": _vector(message, "heading_belief", 12),
        "metric_state_canvas_m": _vector(message, "metric_state_canvas_m", 3),
        "place_entropy_normalized": float(message.place_entropy_normalized),
        "visual_reliability": float(message.visual_reliability),
        "visual_ood_probability": float(message.visual_ood_probability),
        "module2_healthy": bool(message.module2_healthy),
        "observation_valid": bool(message.observation_valid),
        "trusted_write": bool(message.trusted_write),
        "input_healthy": bool(message.input_healthy),
        "health_reasons": [str(value) for value in message.health_reasons],
    }


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-recorder-topics", action="store_true")
    args = parser.parse_args(argv)
    if args.print_recorder_topics:
        print("\n".join(PHASE_B_RECORDER_TOPICS))


if __name__ == "__main__":
    main()


__all__ = [
    "PHASE_B_RECORDER_TOPICS",
    "planning_prior_localization_sample",
]
