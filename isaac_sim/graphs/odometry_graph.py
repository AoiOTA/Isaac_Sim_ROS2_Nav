"""Ideal Isaac odometry graph; intentionally absent in realistic mode.

The ideal publisher is deliberately on-demand.  ``navigation_sim`` invokes it
once, after the loop's motion-assist write, so the published body twist is the
one that causally applies to the following simulation interval.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from isaac_sim.graphs.spec import GraphSpec, TargetPaths, materialize_graph
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.src.config import ProjectConfig


class IdealOdomPublishError(RuntimeError):
    """Raised when an ideal-odometry loop cannot publish exactly once."""


@dataclass
class IdealOdomPublisher:
    """A single-epoch, single-trigger handle for the ideal odometry graph.

    The small wrapper keeps the stateful trigger contract testable without
    importing Kit.  Runtime construction is isolated in :meth:`create`.
    """

    graph: Any
    impulse_attribute: Any
    evaluate_sync: Callable[[Any], Any]
    epoch: int
    _retired: bool = False
    _last_loop_sequence: int | None = None

    @classmethod
    def create(cls, config: ProjectConfig, *, epoch: int = 0) -> "IdealOdomPublisher":
        graph, nodes = materialize_graph(ideal_odometry_graph_spec(config))
        import omni.graph.core as og

        try:
            impulse_node = nodes["OnImpulseEvent"]
            attribute = impulse_node.get_attribute("state:enableImpulse")
        except (KeyError, TypeError, AttributeError) as exc:
            raise IdealOdomPublishError(
                "ideal odometry impulse node is unavailable"
            ) from exc
        if attribute is None:
            raise IdealOdomPublishError("ideal odometry impulse attribute is unavailable")
        return cls(
            graph=graph,
            impulse_attribute=attribute,
            evaluate_sync=og.Controller.evaluate_sync,
            epoch=int(epoch),
        )

    def retire(self) -> None:
        """Reject future triggers from a graph replaced by reset/reload."""

        self._retired = True

    def trigger(self, loop_sequence: int) -> dict[str, object]:
        """Synchronously emit one ideal odometry/TF publication for a loop."""

        if self._retired:
            raise IdealOdomPublishError("refusing to trigger stale ideal odometry graph")
        if isinstance(loop_sequence, bool) or not isinstance(loop_sequence, int):
            raise IdealOdomPublishError("ideal odometry loop sequence must be an integer")
        if self._last_loop_sequence == loop_sequence:
            raise IdealOdomPublishError(
                f"ideal odometry already triggered for loop {loop_sequence}"
            )
        try:
            self.impulse_attribute.set(True)
            result = self.evaluate_sync(self.graph)
        except Exception as exc:
            raise IdealOdomPublishError(
                f"ideal odometry trigger/evaluate failed for loop {loop_sequence}"
            ) from exc
        if result is False:
            raise IdealOdomPublishError(
                f"ideal odometry evaluate returned failure for loop {loop_sequence}"
            )
        self._last_loop_sequence = loop_sequence
        return {
            "graph_epoch": self.epoch,
            "loop_sequence": loop_sequence,
            "trigger_status": True,
            "evaluate_status": True,
            # The graph has one odometry and one odom->base publisher, both
            # downstream of this sole impulse.  The trace/evaluator verifies
            # received ROS payload cardinality independently.
            "loop_publish_count": 1,
        }


def ideal_odometry_graph_spec(config: ProjectConfig) -> GraphSpec:
    if config.simulation.odometry_mode != "ideal":
        raise ValueError("Isaac ideal odometry graph is forbidden in realistic mode")
    topics = load_topics(config.files.topics)
    qos = load_qos_profiles(config.files.qos)
    nodes = (
        ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
        ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
        ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
        ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
    )
    connections = (
        ("OnImpulseEvent.outputs:execOut", "ComputeOdometry.inputs:execIn"),
        ("OnImpulseEvent.outputs:execOut", "PublishOdometry.inputs:execIn"),
        ("OnImpulseEvent.outputs:execOut", "PublishOdomTF.inputs:execIn"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
        ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
        ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
        ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
        ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
        ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
        ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
        ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
    )
    values = (
        ("ComputeOdometry.inputs:chassisPrim", TargetPaths((config.robot.base_link_prim,))),
        ("PublishOdometry.inputs:topicName", topics["odom"]),
        ("PublishOdometry.inputs:odomFrameId", topics["frames"]["odom"]),
        ("PublishOdometry.inputs:chassisFrameId", topics["frames"]["base"]),
        ("PublishOdometry.inputs:nodeNamespace", config.ros2.namespace),
        ("PublishOdometry.inputs:queueSize", 10),
        ("PublishOdometry.inputs:qosProfile", qos["state"]),
        ("PublishOdomTF.inputs:parentFrameId", topics["frames"]["odom"]),
        ("PublishOdomTF.inputs:childFrameId", topics["frames"]["base"]),
        ("PublishOdomTF.inputs:topicName", topics["tf"]),
        ("PublishOdomTF.inputs:nodeNamespace", config.ros2.namespace),
        ("PublishOdomTF.inputs:qosProfile", qos["tf"]),
    )
    return GraphSpec(
        "/World/Graphs/IdealOdometry",
        nodes,
        connections,
        values,
        on_demand=True,
    )


def build_odometry_graph(config: ProjectConfig, *, epoch: int = 0):
    if config.simulation.odometry_mode == "realistic":
        return None
    return IdealOdomPublisher.create(config, epoch=epoch)
