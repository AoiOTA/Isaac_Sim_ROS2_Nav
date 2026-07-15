"""Pure OmniGraph specification model plus delayed runtime materialization."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any


class GraphSpecError(ValueError):
    pass


class MaterializedGraphReadbackError(RuntimeError):
    """Raised when an OmniGraph no longer matches the spec that created it."""


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


@dataclass(frozen=True)
class MaterializedGraphReadback:
    """Canonical facts read back from a materialized OmniGraph."""

    graph_path: str
    pipeline_stage: str
    nodes: tuple[tuple[str, str], ...]
    connections: tuple[tuple[str, str], ...]
    values: tuple[tuple[str, Any], ...]


def _normalized_runtime_value(value: Any) -> Any:
    """Convert OmniGraph/NumPy/USD containers to comparable Python values."""

    if isinstance(value, TargetPaths):
        return [_normalized_runtime_value(path) for path in value.paths]
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Mapping):
        return {
            str(key): _normalized_runtime_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_normalized_runtime_value(item) for item in value]

    # NumPy arrays/scalars and several OmniGraph array wrappers expose one
    # of these methods. Keep the imports delayed so pure-Python contract tests
    # do not need the Isaac Sim environment.
    tolist = getattr(value, "tolist", None)
    if callable(tolist):
        return _normalized_runtime_value(tolist())
    item = getattr(value, "item", None)
    if callable(item):
        try:
            return _normalized_runtime_value(item())
        except (TypeError, ValueError):
            pass

    text = str(value)
    if type(value).__name__ == "Path" or text.startswith("/"):
        return text
    if isinstance(value, Iterable):
        return [_normalized_runtime_value(item) for item in value]
    return value


def _runtime_values_equal(expected: Any, actual: Any) -> bool:
    if isinstance(expected, bool) or isinstance(actual, bool):
        return type(expected) is type(actual) and expected == actual
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        if isinstance(expected, float) or isinstance(actual, float):
            return math.isclose(
                float(expected),
                float(actual),
                rel_tol=1.0e-6,
                abs_tol=1.0e-9,
            )
        return expected == actual
    if isinstance(expected, list) and isinstance(actual, list):
        return len(expected) == len(actual) and all(
            _runtime_values_equal(expected_item, actual_item)
            for expected_item, actual_item in zip(expected, actual, strict=True)
        )
    if isinstance(expected, Mapping) and isinstance(actual, Mapping):
        return expected.keys() == actual.keys() and all(
            _runtime_values_equal(expected[key], actual[key])
            for key in expected
        )
    return type(expected) is type(actual) and expected == actual


def _pipeline_stage_name(raw_stage: Any) -> str:
    name = getattr(raw_stage, "name", None)
    token = str(name if name is not None else raw_stage).lower()
    token = "".join(character for character in token if character.isalnum())
    if "ondemand" in token:
        return "on_demand"
    if "simulation" in token or token == "execution":
        return "execution"
    raise MaterializedGraphReadbackError(
        f"unsupported materialized graph pipeline stage: {raw_stage!r}"
    )


def _relative_node_name(graph_path: str, node_path: Any) -> str:
    prefix = f"{graph_path}/"
    path = str(node_path)
    if not path.startswith(prefix):
        raise MaterializedGraphReadbackError(
            f"node path is outside materialized graph: {path!r}"
        )
    name = path[len(prefix) :]
    if not name or "/" in name:
        raise MaterializedGraphReadbackError(
            f"materialized graph contains a nested or unnamed node: {path!r}"
        )
    return name


def _relative_attribute_path(graph_path: str, attribute: Any) -> str:
    prefix = f"{graph_path}/"
    path = str(attribute.get_path())
    if not path.startswith(prefix):
        raise MaterializedGraphReadbackError(
            f"attribute path is outside materialized graph: {path!r}"
        )
    relative = path[len(prefix) :]
    if "." not in relative or "/" in relative.split(".", 1)[0]:
        raise MaterializedGraphReadbackError(
            f"invalid materialized graph attribute path: {path!r}"
        )
    return relative


def read_materialized_graph(
    spec: GraphSpec,
    materialized_graph: Any,
) -> MaterializedGraphReadback:
    """Read and exactly verify the graph, nodes, edges, and configured values."""

    spec.validate()
    graph = (
        materialized_graph[0]
        if isinstance(materialized_graph, tuple)
        else materialized_graph
    )
    if graph is None:
        raise MaterializedGraphReadbackError("materialized graph handle is missing")

    try:
        actual_graph_path = str(graph.get_path_to_graph())
        raw_nodes = tuple(graph.get_nodes())
        actual_pipeline_stage = _pipeline_stage_name(graph.get_pipeline_stage())
    except (AttributeError, TypeError) as exc:
        raise MaterializedGraphReadbackError(
            "materialized graph does not expose the OmniGraph readback API"
        ) from exc

    if actual_graph_path != spec.path:
        raise MaterializedGraphReadbackError(
            "materialized graph path mismatch: "
            f"expected={spec.path!r}, actual={actual_graph_path!r}"
        )
    expected_pipeline_stage = graph_pipeline_kind(spec)
    if actual_pipeline_stage != expected_pipeline_stage:
        raise MaterializedGraphReadbackError(
            "materialized graph pipeline mismatch: "
            f"expected={expected_pipeline_stage!r}, "
            f"actual={actual_pipeline_stage!r}"
        )

    node_by_name: dict[str, Any] = {}
    actual_nodes: list[tuple[str, str]] = []
    for node in raw_nodes:
        try:
            name = _relative_node_name(spec.path, node.get_prim_path())
            type_name = str(node.get_type_name())
        except AttributeError as exc:
            raise MaterializedGraphReadbackError(
                "materialized graph contains a node without readback metadata"
            ) from exc
        if name in node_by_name:
            raise MaterializedGraphReadbackError(
                f"materialized graph contains duplicate node {name!r}"
            )
        node_by_name[name] = node
        actual_nodes.append((name, type_name))

    expected_nodes = tuple(sorted(spec.nodes))
    canonical_nodes = tuple(sorted(actual_nodes))
    if canonical_nodes != expected_nodes:
        raise MaterializedGraphReadbackError(
            "materialized graph node/type mismatch: "
            f"expected={expected_nodes!r}, actual={canonical_nodes!r}"
        )

    actual_connections: list[tuple[str, str]] = []
    for node in raw_nodes:
        try:
            attributes = tuple(node.get_attributes())
        except (AttributeError, TypeError) as exc:
            raise MaterializedGraphReadbackError(
                "materialized graph node does not expose its attributes"
            ) from exc
        for source_attribute in attributes:
            try:
                downstream = tuple(
                    source_attribute.get_downstream_connections()
                )
            except (AttributeError, TypeError) as exc:
                raise MaterializedGraphReadbackError(
                    "materialized graph attribute does not expose connections"
                ) from exc
            if not downstream:
                continue
            source = _relative_attribute_path(spec.path, source_attribute)
            actual_connections.extend(
                (source, _relative_attribute_path(spec.path, target_attribute))
                for target_attribute in downstream
            )

    expected_connections = tuple(sorted(spec.connections))
    canonical_connections = tuple(sorted(actual_connections))
    if canonical_connections != expected_connections:
        raise MaterializedGraphReadbackError(
            "materialized graph connection mismatch: "
            f"expected={expected_connections!r}, "
            f"actual={canonical_connections!r}"
        )

    actual_values: list[tuple[str, Any]] = []
    for attribute_path, expected_raw_value in spec.values:
        node_name, attribute_name = attribute_path.split(".", 1)
        node = node_by_name[node_name]
        try:
            attribute = node.get_attribute(attribute_name)
            if attribute is None or not attribute:
                raise AttributeError(attribute_name)
            actual_value = _normalized_runtime_value(attribute.get())
        except (AttributeError, TypeError, ValueError) as exc:
            raise MaterializedGraphReadbackError(
                "materialized graph configured attribute is unreadable: "
                f"{attribute_path!r}"
            ) from exc
        expected_value = _normalized_runtime_value(expected_raw_value)
        if not _runtime_values_equal(expected_value, actual_value):
            raise MaterializedGraphReadbackError(
                "materialized graph configured value mismatch: "
                f"attribute={attribute_path!r}, expected={expected_value!r}, "
                f"actual={actual_value!r}"
            )
        actual_values.append((attribute_path, actual_value))

    return MaterializedGraphReadback(
        graph_path=actual_graph_path,
        pipeline_stage=actual_pipeline_stage,
        nodes=canonical_nodes,
        connections=canonical_connections,
        values=tuple(sorted(actual_values)),
    )


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
