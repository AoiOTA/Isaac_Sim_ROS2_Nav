from __future__ import annotations

from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import textwrap

import pytest

from isaac_sim.apps.navigation_sim import _parser, _rebuild_control_graph
from isaac_sim.graphs.control_graph import (
    capture_materialized_control_graph_snapshot,
    control_graph_spec,
)
from isaac_sim.graphs.odometry_graph import ideal_odometry_graph_spec
from isaac_sim.graphs.ros_contract import load_qos_profiles, load_topics
from isaac_sim.graphs.sensor_graph import core_sensor_graph_spec, lidar_graph_spec
from isaac_sim.graphs.spec import (
    GraphSpec,
    TargetPaths,
    graph_pipeline_kind,
)
from isaac_sim.graphs.tf_graph import structure_tf_graph_spec
from isaac_sim.src.bridge import ros_graph_builder as ros_graph_builder_module
from isaac_sim.src.bridge.ros_graph_builder import RosGraphBuilder
from isaac_sim.src.bridge.tf_ownership import (
    TfOwnershipError,
    expected_tf_owners,
    validate_tf_publishers,
)
from isaac_sim.src.config import load_project_config
from isaac_sim.src.yaml_utils import YamlConfigError


ROOT = Path(__file__).resolve().parents[2]
SPLIT_AXLE_V1 = "split_axle_v1"
SINGLE_FOUR_WHEEL_WRITE_V1 = "single_four_wheel_write_v1"


class _FakeAttribute:
    def __init__(self, path: str, value=None):
        self._path = path
        self._value = value
        self._downstream = []

    def get_path(self):
        return self._path

    def get(self):
        return self._value

    def get_downstream_connections(self):
        return list(self._downstream)


class _FakeNode:
    def __init__(self, path: str, type_name: str):
        self._path = path
        self._type_name = type_name
        self._attributes = {}

    def get_prim_path(self):
        return self._path

    def get_type_name(self):
        return self._type_name

    def get_attributes(self):
        return list(self._attributes.values())

    def get_attribute(self, name: str):
        return self._attributes.get(name)

    def attribute(self, name: str, value=None):
        attribute = self._attributes.get(name)
        if attribute is None:
            attribute = _FakeAttribute(f"{self._path}.{name}", value)
            self._attributes[name] = attribute
        elif value is not None:
            attribute._value = value
        return attribute


class _FakeGraph:
    def __init__(self, path: str, nodes, pipeline_stage="on_demand"):
        self._path = path
        self._nodes = list(nodes)
        self._pipeline_stage = pipeline_stage

    def get_path_to_graph(self):
        return self._path

    def get_nodes(self):
        return list(self._nodes)

    def get_pipeline_stage(self):
        return self._pipeline_stage


def _fake_materialized_graph(spec: GraphSpec):
    nodes = {
        name: _FakeNode(f"{spec.path}/{name}", type_name)
        for name, type_name in spec.nodes
    }
    for attribute_path, value in spec.values:
        node_name, attribute_name = attribute_path.split(".", 1)
        if isinstance(value, TargetPaths):
            value = list(value.paths)
        nodes[node_name].attribute(attribute_name, value)
    for source_path, target_path in spec.connections:
        source_node, source_name = source_path.split(".", 1)
        target_node, target_name = target_path.split(".", 1)
        source = nodes[source_node].attribute(source_name)
        target = nodes[target_node].attribute(target_name)
        source._downstream.append(target)
    return _FakeGraph(spec.path, reversed(tuple(nodes.values()))), nodes


def _snapshot_spec(mode: str) -> GraphSpec:
    common_nodes = (
        ("DifferentialController", "example.DifferentialController"),
        ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
    )
    if mode == SPLIT_AXLE_V1:
        nodes = common_nodes + (
            ("RearController", "example.ArticulationController"),
            ("FrontController", "example.ArticulationController"),
        )
        connections = (
            (
                "DifferentialController.outputs:velocityCommand",
                "RearController.inputs:velocityCommand",
            ),
            (
                "DifferentialController.outputs:velocityCommand",
                "FrontController.inputs:velocityCommand",
            ),
        )
        values = (
            ("DifferentialController.inputs:wheelRadius", 0.098),
            (
                "RearController.inputs:targetPrim",
                TargetPaths(("/World/Robots/Jackal",)),
            ),
            (
                "RearController.inputs:jointNames",
                ["rear_left_wheel_joint", "rear_right_wheel_joint"],
            ),
            (
                "FrontController.inputs:targetPrim",
                TargetPaths(("/World/Robots/Jackal",)),
            ),
            (
                "FrontController.inputs:jointNames",
                ["front_left_wheel_joint", "front_right_wheel_joint"],
            ),
        )
    else:
        nodes = common_nodes + (
            ("WheelController", "example.ArticulationController"),
            ("AppendWheelCommands", "example.AppendArray"),
        )
        connections = (
            (
                "DifferentialController.outputs:velocityCommand",
                "AppendWheelCommands.inputs:input1",
            ),
            (
                "DifferentialController.outputs:velocityCommand",
                "AppendWheelCommands.inputs:input0",
            ),
            (
                "AppendWheelCommands.outputs:array",
                "WheelController.inputs:velocityCommand",
            ),
        )
        values = (
            ("DifferentialController.inputs:wheelRadius", 0.098),
            (
                "WheelController.inputs:targetPrim",
                TargetPaths(("/World/Robots/Jackal",)),
            ),
            (
                "WheelController.inputs:jointNames",
                [
                    "front_left_wheel_joint",
                    "front_right_wheel_joint",
                    "rear_left_wheel_joint",
                    "rear_right_wheel_joint",
                ],
            ),
        )
    return GraphSpec(
        "/World/Graphs/Control",
        nodes,
        connections,
        values,
    )


@pytest.mark.parametrize(
    ("mode", "writer_names"),
    (
        (SPLIT_AXLE_V1, ["FrontController", "RearController"]),
        (SINGLE_FOUR_WHEEL_WRITE_V1, ["WheelController"]),
    ),
)
def test_materialized_control_snapshot_is_canonical_and_hash_stable(
    mode, writer_names
):
    spec = _snapshot_spec(mode)
    first_graph, _ = _fake_materialized_graph(spec)
    second_graph, _ = _fake_materialized_graph(spec)

    first = capture_materialized_control_graph_snapshot(
        spec,
        first_graph,
        mode,
    )
    second = capture_materialized_control_graph_snapshot(
        spec,
        second_graph,
        mode,
    )

    assert first == second
    assert first["schema_version"] == 1
    assert first["wheel_command_application"] == mode
    assert first["materialized_readback_verified"] is True
    assert first["topology"]["graph_path"] == spec.path
    assert first["topology"]["pipeline_stage"] == "on_demand"
    assert [node["name"] for node in first["topology"]["nodes"]] == sorted(
        dict(spec.nodes)
    )
    assert first["topology"]["connections"] == sorted(
        (
            {"source": source, "target": target}
            for source, target in spec.connections
        ),
        key=lambda connection: (connection["source"], connection["target"]),
    )
    assert [
        writer["node"] for writer in first["topology"]["command_writers"]
    ] == writer_names
    assert len(first["topology_sha256"]) == 64


@pytest.mark.parametrize(
    "tamper",
    (
        "extra_node",
        "missing_node",
        "node_type",
        "extra_connection",
        "missing_connection",
        "configured_value",
        "writer_joint_names",
        "writer_target_prim",
        "pipeline_stage",
    ),
)
def test_materialized_control_snapshot_rejects_graph_tampering(tamper):
    spec = _snapshot_spec(SINGLE_FOUR_WHEEL_WRITE_V1)
    graph, nodes = _fake_materialized_graph(spec)
    if tamper == "extra_node":
        graph._nodes.append(_FakeNode(f"{spec.path}/Extra", "example.Extra"))
    elif tamper == "missing_node":
        graph._nodes = [
            node
            for node in graph._nodes
            if not str(node.get_prim_path()).endswith("/OnPhysicsStep")
        ]
    elif tamper == "node_type":
        nodes["WheelController"]._type_name = "example.WrongController"
    elif tamper == "extra_connection":
        source = nodes["OnPhysicsStep"].attribute("outputs:step")
        target = nodes["WheelController"].attribute("inputs:execIn")
        source._downstream.append(target)
    elif tamper == "missing_connection":
        nodes["AppendWheelCommands"].attribute(
            "outputs:array"
        )._downstream.clear()
    elif tamper == "configured_value":
        nodes["DifferentialController"].attribute(
            "inputs:wheelRadius"
        )._value = 0.5
    elif tamper == "writer_joint_names":
        nodes["WheelController"].attribute("inputs:jointNames")._value = [
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
            "front_left_wheel_joint",
            "front_right_wheel_joint",
        ]
    elif tamper == "writer_target_prim":
        nodes["WheelController"].attribute("inputs:targetPrim")._value = [
            "/World/Robots/Other"
        ]
    elif tamper == "pipeline_stage":
        graph._pipeline_stage = "execution"

    with pytest.raises(RuntimeError, match="materialized control graph"):
        capture_materialized_control_graph_snapshot(
            spec,
            graph,
            SINGLE_FOUR_WHEEL_WRITE_V1,
        )


def _config(mode: str = "ideal"):
    return load_project_config(
        ROOT / "isaac_sim/configs/project.yaml",
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": "/home/lyb/isaacsim_assets/Assets/Isaac/6.0",
            "ISAAC_NAV__SIMULATION__ODOMETRY_MODE": mode,
        },
    )


def test_control_sensor_and_ideal_odometry_specs_validate():
    config = _config()
    specs = (
        control_graph_spec(config),
        core_sensor_graph_spec(config, "/World/Robots/Jackal/base_link/imu_link/imu_sensor"),
        lidar_graph_spec(config, "/Render/Product"),
        ideal_odometry_graph_spec(config),
    )
    for spec in specs:
        spec.validate()
        assert "ground_truth" not in repr(spec).lower()
    for spec in (specs[0], specs[1], specs[3]):
        assert "omni.graph.action.OnPlaybackTick" not in dict(spec.nodes).values()
        assert "isaacsim.core.nodes.OnPhysicsStep" in dict(spec.nodes).values()
        assert graph_pipeline_kind(spec) == "on_demand"
    assert graph_pipeline_kind(specs[2]) == "execution"
    dt_values = [value for name, value in specs[0].values if name.endswith("inputs:dt")]
    assert dt_values == [pytest.approx(1.0 / 60.0)]
    sensor_nodes = dict(specs[1].nodes)
    sensor_values = dict(specs[1].values)
    lidar_values = dict(specs[2].values)
    assert sensor_nodes["ReadJointState"].endswith("IsaacReadJointState")
    assert "PublishJointState.inputs:targetPrim" not in sensor_values
    assert (
        lidar_values[
            "PointCloudPublisher.inputs:resetSimulationTimeOnStop"
        ]
        is True
    )
    assert (
        "ReadJointState.outputs:jointNames",
        "PublishJointState.inputs:jointNames",
    ) in specs[1].connections


def test_control_graph_uses_top_level_radius_and_effective_track_width():
    spec = control_graph_spec(_config())
    nodes = dict(spec.nodes)
    values = dict(spec.values)

    assert "AppendWheelCommands" not in nodes
    assert nodes["FrontController"].endswith("IsaacArticulationController")
    assert nodes["RearController"].endswith("IsaacArticulationController")
    assert values["DifferentialController.inputs:wheelRadius"] == 0.098
    assert values["DifferentialController.inputs:wheelDistance"] == 0.37559
    assert values["FrontController.inputs:jointNames"] == [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
    ]
    assert values["RearController.inputs:jointNames"] == [
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]


def test_control_graph_single_write_duplicates_command_for_ordered_four_wheels():
    spec = control_graph_spec(
        _config(),
        wheel_command_application=SINGLE_FOUR_WHEEL_WRITE_V1,
    )
    spec.validate()
    nodes = dict(spec.nodes)
    values = dict(spec.values)

    assert "FrontController" not in nodes
    assert "RearController" not in nodes
    assert nodes["AppendWheelCommands"] == "omni.graph.nodes.AppendArray"
    assert nodes["WheelController"].endswith("IsaacArticulationController")
    assert (
        "DifferentialController.outputs:velocityCommand",
        "AppendWheelCommands.inputs:input0",
    ) in spec.connections
    assert (
        "DifferentialController.outputs:velocityCommand",
        "AppendWheelCommands.inputs:input1",
    ) in spec.connections
    assert (
        "AppendWheelCommands.outputs:array",
        "WheelController.inputs:velocityCommand",
    ) in spec.connections
    assert values["WheelController.inputs:jointNames"] == [
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    ]


def test_control_graph_rejects_unknown_wheel_command_application():
    with pytest.raises(ValueError, match="wheel command application"):
        control_graph_spec(
            _config(),
            wheel_command_application="last_writer_wins_v1",
        )


def test_ros_graph_builder_keeps_wheel_command_application_immutable_for_rebuild(
    monkeypatch,
):
    calls = []

    def fake_build_control_graph(config, wheel_command_application):
        calls.append((config, wheel_command_application))
        return "control-graph"

    monkeypatch.setattr(
        ros_graph_builder_module,
        "build_control_graph",
        fake_build_control_graph,
    )
    config = _config()
    builder = RosGraphBuilder(
        config,
        object(),
        wheel_command_application=SINGLE_FOUR_WHEEL_WRITE_V1,
    )

    assert builder.build_control() == "control-graph"
    assert calls == [(config, SINGLE_FOUR_WHEEL_WRITE_V1)]
    with pytest.raises(AttributeError):
        builder.wheel_command_application = SPLIT_AXLE_V1


def test_ros_graph_builder_exposes_verified_build_and_reset_recapture(
    monkeypatch,
):
    config = _config()
    materialized = object()
    initial_snapshot = {"topology_sha256": "a" * 64}
    rebuilt_snapshot = {"topology_sha256": "a" * 64}
    spec = object()
    calls = []

    def fake_build_with_snapshot(selected_config, selected_mode):
        calls.append(("build", selected_config, selected_mode))
        return materialized, initial_snapshot

    def fake_control_spec(selected_config, selected_mode):
        calls.append(("spec", selected_config, selected_mode))
        return spec

    def fake_capture(selected_spec, selected_graph, selected_mode):
        calls.append(("capture", selected_spec, selected_graph, selected_mode))
        return rebuilt_snapshot

    monkeypatch.setattr(
        ros_graph_builder_module,
        "build_control_graph_with_snapshot",
        fake_build_with_snapshot,
    )
    monkeypatch.setattr(
        ros_graph_builder_module,
        "control_graph_spec",
        fake_control_spec,
    )
    monkeypatch.setattr(
        ros_graph_builder_module,
        "capture_materialized_control_graph_snapshot",
        fake_capture,
    )
    builder = RosGraphBuilder(
        config,
        object(),
        wheel_command_application=SINGLE_FOUR_WHEEL_WRITE_V1,
    )

    assert builder.build_control_with_snapshot() == (
        materialized,
        initial_snapshot,
    )
    assert builder.capture_control_snapshot(materialized) is rebuilt_snapshot
    assert calls == [
        ("build", config, SINGLE_FOUR_WHEEL_WRITE_V1),
        ("spec", config, SINGLE_FOUR_WHEEL_WRITE_V1),
        (
            "capture",
            spec,
            materialized,
            SINGLE_FOUR_WHEEL_WRITE_V1,
        ),
    ]


@pytest.mark.isaac
@pytest.mark.skipif(
    os.environ.get("ISAAC_NAV_RUN_REAL_GRAPH_TEST") != "1",
    reason="set ISAAC_NAV_RUN_REAL_GRAPH_TEST=1 for the isolated Kit test",
)
def test_real_control_graph_materializes_and_recaptures_both_modes():
    isaac_python = Path(
        os.environ.get(
            "ISAAC_PYTHON",
            "/home/lyb/miniconda3/envs/isaacsim/bin/python",
        )
    )
    if not isaac_python.is_file():
        pytest.skip(f"Isaac Sim Python runtime is unavailable: {isaac_python}")

    # SimulationApp's supported fast shutdown terminates its process. Run it
    # in an isolated child and emit a flushed machine-readable result first,
    # so the parent pytest process can still verify the exit and both hashes.
    script = textwrap.dedent(
        f"""
        import json
        import os
        from pathlib import Path
        import sys
        import traceback

        project_root = Path({str(ROOT)!r})
        sys.path.insert(0, str(project_root))
        sys.argv = [sys.argv[0]]

        try:
            from isaacsim import SimulationApp

            app = SimulationApp({{
                "headless": True,
                "disable_viewport_updates": True,
                "fast_shutdown": True,
            }})

            import omni.kit.app
            import omni.usd

            from isaac_sim.graphs.control_graph import (
                SPLIT_AXLE_V1,
                SINGLE_FOUR_WHEEL_WRITE_V1,
                capture_materialized_control_graph_snapshot,
                control_graph_spec,
            )
            from isaac_sim.graphs.spec import materialize_graph
            from isaac_sim.src.config import load_project_config

            config = load_project_config(
                project_root / "isaac_sim/configs/project.yaml",
                {{
                    "PROJECT_ROOT": str(project_root),
                    "ISAAC_ASSET_ROOT": (
                        "/home/lyb/isaacsim_assets/Assets/Isaac/6.0"
                    ),
                }},
            )
            manager = omni.kit.app.get_app().get_extension_manager()
            for extension_id in (
                "isaacsim.core.nodes",
                "isaacsim.robot.wheeled_robots.nodes",
                "isaacsim.ros2.bridge",
            ):
                manager.set_extension_enabled_immediate(extension_id, True)
            app.update()

            results = {{}}
            for mode in (SPLIT_AXLE_V1, SINGLE_FOUR_WHEEL_WRITE_V1):
                omni.usd.get_context().new_stage()
                app.update()
                spec = control_graph_spec(config, mode)
                first = capture_materialized_control_graph_snapshot(
                    spec,
                    materialize_graph(spec),
                    mode,
                )
                rebuilt = capture_materialized_control_graph_snapshot(
                    spec,
                    materialize_graph(spec),
                    mode,
                )
                if first["topology_sha256"] != rebuilt["topology_sha256"]:
                    raise AssertionError(
                        f"unstable topology SHA for {{mode}}: "
                        f"{{first['topology_sha256']}} != "
                        f"{{rebuilt['topology_sha256']}}"
                    )
                results[mode] = {{
                    "first": first["topology_sha256"],
                    "rebuilt": rebuilt["topology_sha256"],
                    "verified": rebuilt["materialized_readback_verified"],
                }}

            print(
                "CONTROL_GRAPH_RESULT="
                + json.dumps(results, sort_keys=True),
                flush=True,
            )
            app.close()
            os._exit(0)
        except BaseException:
            traceback.print_exc()
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(1)
        """
    )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        filter(None, (str(ROOT), environment.get("PYTHONPATH", "")))
    )
    completed = subprocess.run(
        (str(isaac_python), "-c", script),
        cwd=ROOT,
        env=environment,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=180,
        check=False,
    )
    if completed.returncode != 0:
        pytest.fail(completed.stdout[-12000:])

    marker = "CONTROL_GRAPH_RESULT="
    result_lines = [
        line for line in completed.stdout.splitlines() if line.startswith(marker)
    ]
    assert len(result_lines) == 1, completed.stdout[-12000:]
    result = json.loads(result_lines[0].removeprefix(marker))
    print(result_lines[0])
    assert set(result) == {SPLIT_AXLE_V1, SINGLE_FOUR_WHEEL_WRITE_V1}
    for mode_result in result.values():
        assert mode_result["verified"] is True
        assert mode_result["first"] == mode_result["rebuilt"]


def test_navigation_cli_selects_only_versioned_wheel_command_applications():
    parser = _parser()

    assert parser.parse_args([]).wheel_command_application == SPLIT_AXLE_V1
    assert (
        parser.parse_args(
            ["--wheel-command-application", SINGLE_FOUR_WHEEL_WRITE_V1]
        ).wheel_command_application
        == SINGLE_FOUR_WHEEL_WRITE_V1
    )
    with pytest.raises(SystemExit):
        parser.parse_args(
            ["--wheel-command-application", "last_writer_wins_v1"]
        )


def test_navigation_reset_executes_control_graph_rebuild_through_selected_builder():
    events = []

    class IdleBrake:
        def reset(self):
            events.append("idle_brake.reset")

    class GraphBuilder:
        wheel_command_application = SINGLE_FOUR_WHEEL_WRITE_V1

        def build_control(self):
            events.append(
                f"build:{self.wheel_command_application}"
            )
            return {"mode": self.wheel_command_application}

    references = {"control": {"mode": SPLIT_AXLE_V1}}

    rebuilt = _rebuild_control_graph(
        IdleBrake(),
        GraphBuilder(),
        references,
    )

    assert events == [
        "idle_brake.reset",
        f"build:{SINGLE_FOUR_WHEEL_WRITE_V1}",
    ]
    assert rebuilt == {"mode": SINGLE_FOUR_WHEEL_WRITE_V1}
    assert references["control"] is rebuilt


def test_control_graph_rejects_project_joint_targets_that_diverge_from_robot_yaml():
    config = _config()
    project_robot = replace(
        config.robot,
        wheel_joints=("project_fl", "project_fr", "project_rl", "project_rr"),
        front_wheel_joints=("project_fl", "project_fr"),
        rear_wheel_joints=("project_rl", "project_rr"),
    )

    with pytest.raises(YamlConfigError, match="ProjectConfig robot joint targets"):
        control_graph_spec(replace(config, robot=project_robot))


def test_static_sensor_frames_use_raw_tf_and_no_world_frame():
    spec = structure_tf_graph_spec(_config())
    spec.validate()
    node_types = dict(spec.nodes)
    values = dict(spec.values)
    assert node_types["ComputeWheelTF"].endswith("IsaacComputeTransformTree")
    assert node_types["WheelTF"].endswith("ROS2PublishTransformTree")
    for node, node_type in node_types.items():
        if node.startswith("StaticTF"):
            assert node_type.endswith("ROS2PublishRawTransformTree")
    frame_values = [
        value
        for attribute, value in spec.values
        if attribute.endswith(("parentFrameId", "childFrameId"))
    ]
    assert "world" not in frame_values
    assert "WheelTF.inputs:targetPrims" not in values
    assert (
        "ComputeWheelTF.outputs:parentFrames",
        "WheelTF.inputs:parentFrames",
    ) in spec.connections
    assert graph_pipeline_kind(spec) == "on_demand"


def test_tf_ownership_requires_exactly_one_publisher():
    expected = expected_tf_owners("realistic")
    validate_tf_publishers("realistic", {key: [owner] for key, owner in expected.items()})
    duplicated = {key: [owner] for key, owner in expected.items()}
    duplicated["odom->base_link"].append("isaac")
    with pytest.raises(TfOwnershipError, match="sole owner"):
        validate_tf_publishers("realistic", duplicated)

    rsp_expected = expected_tf_owners("realistic", "rsp")
    assert rsp_expected["base_link->sensor_links"] == "rsp"
    validate_tf_publishers(
        "realistic",
        {key: [owner] for key, owner in rsp_expected.items()},
        "rsp",
    )
    with pytest.raises(TfOwnershipError, match="ideal odometry requires"):
        expected_tf_owners("ideal", "rsp")


def test_topic_and_qos_contracts_are_absolute_and_encoded():
    topics = load_topics(ROOT / "isaac_sim/configs/ros2_bridge/topics.yaml")
    qos = load_qos_profiles(ROOT / "isaac_sim/configs/ros2_bridge/qos.yaml")
    assert topics["pointcloud"] == "/lidar/points_raw"
    assert topics["frames"]["base"] == "base_link"
    assert topics["camera_front_image"] == "/camera/front/image_raw"
    assert topics["camera_front_info"] == "/camera/front/camera_info"
    assert topics["frames"]["camera_front_optical"] == (
        "camera_front_optical_frame"
    )
    assert '"reliability":"bestEffort"' in qos["sensor_data"]
    assert '"depth":2' in qos["camera_sensor_data"]
    assert '"durability":"transientLocal"' in qos["static_tf"]
