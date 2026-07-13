import math

import pytest

from robot_experiments.scan_fault import (
    ScanFaultCommandError,
    ScanFaultController,
)


def _process(controller, stamp_ns, now_s):
    return controller.process(stamp_ns=stamp_ns, now_s=now_s)


def test_normal_mode_forwards_scans_and_reports_exact_counters():
    controller = ScanFaultController()
    first = _process(controller, 1_000_000_000, 10.0)
    second = _process(controller, 1_100_000_000, 10.1)

    assert first.forward and first.reason == "normal"
    assert second.forward and second.reason == "normal"
    status = controller.status(now_s=10.1)
    assert status["mode"] == "normal"
    assert status["total"] == {"received": 2, "forwarded": 2, "dropped": 0}
    assert status["current_epoch"] == status["total"]


def test_drop_next_one_drops_exactly_one_frame_then_resumes():
    controller = ScanFaultController()
    controller.apply_command(
        {"command": "drop_next", "count": 1, "epoch": 0}, now_s=1.0
    )

    dropped = _process(controller, 1_000_000_000, 1.1)
    resumed = _process(controller, 1_100_000_000, 1.2)

    assert not dropped.forward and dropped.reason == "drop_next"
    assert dropped.state_changed
    assert resumed.forward and resumed.reason == "normal"
    assert controller.status(now_s=1.2)["mode"] == "normal"


def test_drop_next_multiple_has_no_off_by_one_and_resume_cancels_it():
    controller = ScanFaultController()
    controller.apply_command(
        {"command": "drop_next", "count": 3, "epoch": 0}, now_s=2.0
    )
    decisions = [
        _process(controller, 2_000_000_000 + index * 100_000_000, 2.1 + index * 0.1)
        for index in range(4)
    ]
    assert [decision.forward for decision in decisions] == [False, False, False, True]
    assert controller.status(now_s=2.5)["total"]["dropped"] == 3

    controller.apply_command(
        {"command": "drop_next", "count": 5, "epoch": 0}, now_s=2.6
    )
    controller.apply_command({"command": "resume", "epoch": 0}, now_s=2.7)
    assert _process(controller, 2_500_000_000, 2.8).forward


def test_pause_for_uses_monotonic_deadline_and_recovers_at_boundary():
    controller = ScanFaultController()
    controller.apply_command(
        {"command": "pause_for", "seconds": 0.25, "epoch": 0}, now_s=10.0
    )

    assert not _process(controller, 10_000_000_000, 10.0).forward
    assert not _process(controller, 10_100_000_000, 10.249).forward
    boundary = _process(controller, 10_200_000_000, 10.25)
    assert boundary.forward and boundary.reason == "pause_complete"
    assert boundary.state_changed
    assert controller.status(now_s=10.25)["pause_remaining_s"] == 0.0


def test_drop_all_represents_stream_loss_until_explicit_resume():
    controller = ScanFaultController()
    controller.apply_command({"command": "drop_all", "epoch": 0}, now_s=3.0)
    for index in range(20):
        assert not _process(
            controller, 3_000_000_000 + index * 100_000_000, 3.1 + index * 0.1
        ).forward
    controller.apply_command({"command": "resume", "epoch": 0}, now_s=5.2)
    assert _process(controller, 5_000_000_000, 5.3).forward


def test_replace_frame_id_forwards_payload_for_tf_missing_test_until_resume():
    controller = ScanFaultController()
    controller.apply_command(
        {
            "command": "replace_frame_id",
            "frame_id": "fault_missing_lidar",
            "epoch": 0,
        },
        now_s=6.0,
    )
    replaced = _process(controller, 6_000_000_000, 6.1)
    assert replaced.forward
    assert replaced.reason == "replace_frame_id"
    assert replaced.frame_id_override == "fault_missing_lidar"

    controller.apply_command({"command": "normal", "epoch": 0}, now_s=6.2)
    normal = _process(controller, 6_100_000_000, 6.3)
    assert normal.forward and normal.frame_id_override is None


def test_explicit_reset_opens_epoch_clears_fault_and_rejects_stale_commands():
    controller = ScanFaultController()
    controller.apply_command(
        {"command": "drop_next", "count": 5, "epoch": 0}, now_s=7.0
    )
    assert not _process(controller, 7_000_000_000, 7.1).forward

    assert controller.begin_new_epoch("reset_event") == 1
    status = controller.status(now_s=7.2)
    assert status["mode"] == "normal"
    assert status["remaining"] == 0
    assert status["last_stamp_ns"] is None
    assert status["current_epoch"] == {"received": 0, "forwarded": 0, "dropped": 0}
    assert _process(controller, 100_000_000, 7.3).forward

    with pytest.raises(ScanFaultCommandError, match="stale epoch 0"):
        controller.apply_command(
            {"command": "drop_all", "epoch": 0}, now_s=7.4
        )
    controller.apply_command({"command": "drop_all", "epoch": 1}, now_s=7.5)
    assert not _process(controller, 200_000_000, 7.6).forward


def test_timestamp_rollback_is_reset_fallback_and_does_not_double_count_scan():
    controller = ScanFaultController(rollback_tolerance_ns=1_000)
    _process(controller, 8_000_000_000, 8.0)
    controller.apply_command({"command": "drop_all", "epoch": 0}, now_s=8.1)

    reset_scan = _process(controller, 50_000_000, 8.2)
    assert reset_scan.epoch_changed
    assert reset_scan.epoch == 1
    assert reset_scan.forward and reset_scan.reason == "normal"
    status = controller.status(now_s=8.2)
    assert status["last_epoch_reason"] == "scan_stamp_rollback"
    assert status["current_epoch"] == {"received": 1, "forwarded": 1, "dropped": 0}
    assert status["total"] == {"received": 2, "forwarded": 2, "dropped": 0}


def test_small_out_of_order_stamp_within_tolerance_is_not_a_reset():
    controller = ScanFaultController(rollback_tolerance_ns=1_000)
    _process(controller, 1_000_000, 1.0)
    decision = _process(controller, 999_001, 1.1)
    assert not decision.epoch_changed
    assert decision.epoch == 0


@pytest.mark.parametrize(
    "command, message",
    [
        ({}, "command field"),
        ({"command": "unknown", "epoch": 0}, "unsupported"),
        ({"command": "drop_all"}, "epoch is required"),
        ({"command": "drop_next", "count": 0, "epoch": 0}, "positive integer"),
        ({"command": "drop_next", "count": True, "epoch": 0}, "positive integer"),
        ({"command": "pause_for", "seconds": True, "epoch": 0}, "finite and positive"),
        ({"command": "pause_for", "seconds": math.inf, "epoch": 0}, "finite and positive"),
        ({"command": "replace_frame_id", "frame_id": "/bad", "epoch": 0}, "relative TF frame"),
        ({"command": "drop_all", "extra": 1, "epoch": 0}, "unknown command field"),
    ],
)
def test_invalid_commands_are_rejected_without_changing_state(command, message):
    controller = ScanFaultController()
    with pytest.raises(ScanFaultCommandError, match=message):
        controller.apply_command(command, now_s=1.0)
    assert controller.status(now_s=1.0)["mode"] == "normal"
    assert controller.status(now_s=1.0)["command_sequence"] == 0
