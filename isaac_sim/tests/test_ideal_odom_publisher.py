from __future__ import annotations

import pytest

from isaac_sim.graphs.odometry_graph import (
    IdealOdomPublishError,
    IdealOdomPublisher,
)


class _Attribute:
    def __init__(self) -> None:
        self.values: list[bool] = []

    def set(self, value: bool) -> None:
        self.values.append(value)


def test_ideal_odom_trigger_evaluates_once_per_loop_and_records_epoch():
    attribute = _Attribute()
    evaluated: list[object] = []
    graph = object()
    publisher = IdealOdomPublisher(
        graph=graph,
        impulse_attribute=attribute,
        evaluate_sync=lambda value: evaluated.append(value),
        epoch=7,
    )

    receipt = publisher.trigger(11)

    assert attribute.values == [True]
    assert evaluated == [graph]
    assert receipt == {
        "graph_epoch": 7,
        "loop_sequence": 11,
        "trigger_status": True,
        "evaluate_status": True,
        "loop_publish_count": 1,
    }
    with pytest.raises(IdealOdomPublishError, match="already triggered"):
        publisher.trigger(11)


def test_ideal_odom_trace_payload_comes_from_graph_compute_and_publisher_inputs():
    source = {
        "position": [1.0, 2.0, 0.0], "yaw_rad": 0.25,
        "linear_xyz": [0.5, 0.0, 0.0], "angular_xyz": [0.0, 0.0, 0.25],
    }
    publisher = IdealOdomPublisher(
        graph=object(),
        impulse_attribute=_Attribute(),
        evaluate_sync=lambda _graph: None,
        epoch=3,
        payload_reader=lambda: {
            "source_payload": source,
            "publisher_payload": dict(source),
        },
    )

    receipt = publisher.trigger(5)

    assert receipt["source_payload"] == source
    assert receipt["publisher_payload"] == source


def test_ideal_odom_trigger_fails_closed_for_stale_or_failed_graph():
    publisher = IdealOdomPublisher(
        graph=object(),
        impulse_attribute=_Attribute(),
        evaluate_sync=lambda _graph: False,
        epoch=1,
    )
    with pytest.raises(IdealOdomPublishError, match="evaluate returned failure"):
        publisher.trigger(0)

    publisher = IdealOdomPublisher(
        graph=object(),
        impulse_attribute=_Attribute(),
        evaluate_sync=lambda _graph: None,
        epoch=2,
    )
    publisher.retire()
    with pytest.raises(IdealOdomPublishError, match="stale"):
        publisher.trigger(0)


def test_mixed_navigation_rebuilds_per_epoch_and_triggers_once_per_loop():
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1] / "apps" / "navigation_sim.py"
    ).read_text(encoding="utf-8")
    assert 'if mode in {"ideal", "mixed"}:' in source
    assert 'config.simulation.odometry_mode in {"ideal", "mixed"}' in source
    assert "previous.retire()" in source
    assert "odom_graph_epoch += 1" in source
    assert "ideal_odom.trigger(frame)" in source
