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
