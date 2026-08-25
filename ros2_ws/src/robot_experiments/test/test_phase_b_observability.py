from types import SimpleNamespace

from robot_experiments.phase_b_observability import (
    PHASE_B_RECORDER_TOPICS,
    planning_prior_localization_sample,
)


def test_phase_b_recorder_has_localization_shadow_and_terminal_topics():
    required = {
        "/ground_truth/odom",
        "/odom",
        "/amcl_pose",
        "/bio_nav/module1/odom",
        "/bio_nav/cognitive_map/constraints",
        "/bio_nav/module2/planning_prior",
        "/cmd_vel",
        "/cmd_vel_sim",
        "/simulation/reset_event",
        "/simulation/collision",
        "/tf",
        "/tf_static",
        "/scan",
        "/clock",
    }
    assert required <= set(PHASE_B_RECORDER_TOPICS)
    assert len(PHASE_B_RECORDER_TOPICS) == len(set(PHASE_B_RECORDER_TOPICS))


def test_planning_prior_exposes_fifth_trajectory_without_new_pose_topic():
    message = SimpleNamespace(
        stamp=SimpleNamespace(sec=12, nanosec=34),
        sequence=7,
        map_version="map-v1",
        cognitive_tile_id="tile-1",
        t_map_canvas=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        valid_state_mask=[True] * 256,
        place_belief=[1.0 / 256.0] * 256,
        place_mean_canvas_m=[1.25, -2.5],
        heading_belief=[1.0 / 12.0] * 12,
        metric_state_canvas_m=[1.0, -2.0, 0.4],
        place_entropy_normalized=0.25,
        visual_reliability=0.8,
        visual_ood_probability=0.1,
        module2_healthy=True,
        observation_valid=True,
        trusted_write=False,
        input_healthy=True,
        health_reasons=[],
    )

    sample = planning_prior_localization_sample(message)

    assert sample["stamp_ns"] == 12_000_000_034
    assert len(sample["place_belief"]) == 256
    assert sample["place_mean_canvas_m"] == [1.25, -2.5]
    assert sample["metric_state_canvas_m"] == [1.0, -2.0, 0.4]
    assert sample["visual_reliability"] == 0.8
    assert sample["visual_ood_probability"] == 0.1
    assert sample["trusted_write"] is False
