"""Pure OmniGraph specification model plus delayed runtime materialization."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


class GraphSpecError(ValueError):
    pass


@dataclass(frozen=True)
class TargetPaths:
    paths: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.paths or any(not path.startswith("/") for path in self.paths):
            raise GraphSpecError("target paths must contain absolute USD prim paths")


@dataclass(frozen=True)
class GraphSpec:
    path: str
    nodes: tuple[tuple[str, str], ...]
    connections: tuple[tuple[str, str], ...]
    values: tuple[tuple[str, Any], ...] = field(default_factory=tuple)

    def validate(self) -> None:
        if not self.path.startswith("/World/Graphs/"):
            raise GraphSpecError(f"graph path must be under /World/Graphs: {self.path}")
        names = [name for name, _ in self.nodes]
        if len(names) != len(set(names)):
            raise GraphSpecError(f"duplicate node names in {self.path}")
        known = set(names)
        for source, target in self.connections:
            if source.split(".", 1)[0] not in known or target.split(".", 1)[0] not in known:
                raise GraphSpecError(f"connection references unknown node: {(source, target)}")
        for attribute, _ in self.values:
            if attribute.split(".", 1)[0] not in known:
                raise GraphSpecError(f"value references unknown node: {attribute}")
        all_text = repr((self.connections, self.values)).lower()
        if "ground_truth" in all_text:
            raise GraphSpecError("ground truth is not allowed in control/sensor/odometry/TF graphs")
        if '"world"' in all_text or "'world'" in all_text:
            raise GraphSpecError("ROS frame 'world' is forbidden")


def graph_pipeline_kind(spec: GraphSpec) -> str:
    """Return the Isaac evaluator contract required by an event graph."""

    if any(
        node_type == "isaacsim.core.nodes.OnPhysicsStep"
        for _, node_type in spec.nodes
    ):
        return "on_demand"
    return "execution"


def materialize_graph(spec: GraphSpec):
    """Create a graph with Isaac Sim imports delayed until this call."""

    spec.validate()
    import omni.graph.core as og
    import omni.usd
    import usdrt

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(spec.path).IsValid():
        stage.RemovePrim(spec.path)

    def runtime_value(value: Any) -> Any:
        if isinstance(value, TargetPaths):
            return [usdrt.Sdf.Path(path) for path in value.paths]
        return value

    keys = og.Controller.Keys
    graph_config: dict[str, object] = {"graph_path": spec.path}
    if graph_pipeline_kind(spec) == "on_demand":
        # OnPhysicsStep is an event source and Isaac Sim 6.0.1 rejects it in
        # the default execution evaluator. The vendor's own ROS Clock test
        # uses the on-demand pipeline for this exact topology.
        graph_config["pipeline_stage"] = (
            og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND
        )
    else:
        graph_config["evaluator_name"] = "execution"
    graph, nodes, _, _ = og.Controller.edit(
        graph_config,
        {
            keys.CREATE_NODES: list(spec.nodes),
            keys.CONNECT: list(spec.connections),
            keys.SET_VALUES: [(attribute, runtime_value(value)) for attribute, value in spec.values],
        },
    )
    return graph, nodes
