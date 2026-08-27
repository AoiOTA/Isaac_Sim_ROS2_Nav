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
    "/bio_nav/module2/goal_planning_prior",
    "/bio_nav/module2/edge_priors",
    "/bio_nav/module2/srdr_edge_diagnostics",
    "/bio_nav/navigation_graph",
    "/bio_nav/canonical_route",
    "/bio_nav/route_edge_costs",
    "/bio_nav/route_progress",
    "/bio_nav/route_goal_complete",
    "/bio_nav/route_goal_result",
    "/bio_nav/module2/cognitive_obstacles",
    "/bio_nav/cognitive_obstacle_layer/status",
    "/bio_nav/local_risk_layer/status",
    "/bio_nav/cognitive_risk_critic/status",
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


def _mode_candidate(candidate: Any) -> dict[str, Any]:
    return {
        "state_id": int(candidate.state_id),
        "probability": float(candidate.probability),
        "canvas_xy_m": _vector(candidate, "canvas_xy_m", 2),
        "mode_root_state_id": int(candidate.mode_root_state_id),
        "mode_state_count": int(candidate.mode_state_count),
        "mode_mass": float(candidate.mode_mass),
        "mode_expected_xy_m": _vector(candidate, "mode_expected_xy_m", 2),
        "mode_covariance_m2": _vector(candidate, "mode_covariance_m2", 4),
        "mode_ellipse_1sigma": {
            "semi_major_axis_m": float(
                candidate.mode_ellipse_1sigma_semi_major_axis_m
            ),
            "semi_minor_axis_m": float(
                candidate.mode_ellipse_1sigma_semi_minor_axis_m
            ),
            "yaw_rad": float(candidate.mode_ellipse_1sigma_yaw_rad),
        },
    }


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
        "full_posterior_mean_canvas_m_diagnostic": _vector(
            message, "place_mean_canvas_m", 2
        ),
        "dominant_mode_root_state_id": int(message.dominant_mode_root_state_id),
        "dominant_mode_mass": float(message.dominant_mode_mass),
        "dominant_mode_expected_xy_m": _vector(
            message, "dominant_mode_expected_xy_m", 2
        ),
        "dominant_mode_covariance_m2": _vector(
            message, "dominant_mode_covariance_m2", 4
        ),
        "dominant_mode_ellipse_1sigma": {
            "semi_major_axis_m": float(
                message.dominant_mode_ellipse_1sigma_semi_major_axis_m
            ),
            "semi_minor_axis_m": float(
                message.dominant_mode_ellipse_1sigma_semi_minor_axis_m
            ),
            "yaw_rad": float(message.dominant_mode_ellipse_1sigma_yaw_rad),
        },
        "full_posterior_covariance_m2_diagnostic": _vector(
            message, "full_posterior_covariance_m2_diagnostic", 4
        ),
        "place_top_k": [
            _mode_candidate(candidate) for candidate in message.place_top_k
        ],
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
