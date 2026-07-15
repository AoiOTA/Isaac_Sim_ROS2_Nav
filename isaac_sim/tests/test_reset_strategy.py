from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest

from isaac_sim.src.config import (
    ResetStrategyConfig,
    load_project_config,
)
from isaac_sim.src.robot.reset_strategy import (
    ResetStrategyError,
    WheelBinding,
    WheelGroundContactProbe,
    author_wheel_ground_contact_probe,
)


try:
    from pxr import PhysxSchema

    HAS_PXR = True
except ImportError:
    HAS_PXR = False


ROOT = Path(__file__).resolve().parents[2]


WHEEL_BINDINGS = tuple(
    WheelBinding(
        joint_name=name,
        wheel_link_path=f"/World/Robot/{name.removesuffix('_joint')}",
    )
    for name in (
        "front_left_wheel_joint",
        "front_right_wheel_joint",
        "rear_left_wheel_joint",
        "rear_right_wheel_joint",
    )
)
GROUND_FILTERS = (
    "/World/Ground/CollisionA",
    "/World/Ground/CollisionB",
)


class FakeTensor:
    def __init__(self, values):
        self._values = np.asarray(values)

    def numpy(self):
        return self._values


@dataclass
class FakeContactView:
    counts: tuple[tuple[int, int], ...]
    valid: bool = True
    num_contact_filters: int = 2
    wheel_z_m: float = 0.098

    def is_physics_tensor_entity_valid(self):
        return self.valid

    def get_contact_force_data(self, *, dt):
        assert dt == pytest.approx(1.0 / 60.0)
        return None, None, None, None, FakeTensor(self.counts), None

    def get_world_poses(self):
        return (
            FakeTensor(((0.0, 0.0, self.wheel_z_m),) * 4),
            FakeTensor(((1.0, 0.0, 0.0, 0.0),) * 4),
        )


class FakeApp:
    def __init__(self):
        self.updates = 0

    def update(self):
        self.updates += 1


def initialized_probe(counts=((0, 0),) * 4):
    view = FakeContactView(counts=counts)
    factory_calls = []
    probe = WheelGroundContactProbe(
        wheel_bindings=WHEEL_BINDINGS,
        ground_filter_paths=GROUND_FILTERS,
        stage_usd_readback_verified=True,
    )
    app = FakeApp()

    def factory(paths, *, contact_filter_paths, max_contact_count):
        factory_calls.append(
            (tuple(paths), tuple(contact_filter_paths), max_contact_count)
        )
        return view

    probe.initialize(app, view_factory=factory)
    assert app.updates == 1
    assert factory_calls == [
        (
            tuple(binding.wheel_link_path for binding in WHEEL_BINDINGS),
            GROUND_FILTERS,
            128,
        )
    ]
    return probe


@pytest.mark.parametrize(
    ("identifier", "lift_distance_m", "separation_step_count"),
    [
        ("pose_restore_v1", 0.0, 0),
        ("separate_recontact_0p20m_1step_v1", 0.2, 1),
    ],
)
def test_reset_strategy_snapshot_is_strict_and_complete(
    identifier, lift_distance_m, separation_step_count
):
    probe = initialized_probe()

    snapshot = probe.provenance_snapshot(
        ResetStrategyConfig(schema_version=1, identifier=identifier)
    )

    assert snapshot == {
        "schema_version": 1,
        "id": identifier,
        "lift_distance_m": lift_distance_m,
        "separation_step_count": separation_step_count,
        "recontact_step_count": 1,
        "contact_probe": {
            "schema_version": 1,
            "enabled": True,
            "wheel_bindings": [
                {
                    "joint_name": binding.joint_name,
                    "wheel_link_path": binding.wheel_link_path,
                }
                for binding in WHEEL_BINDINGS
            ],
            "wheel_count": 4,
            "ground_filter_paths": list(GROUND_FILTERS),
            "ground_filter_count": 2,
            "max_contact_count": 128,
            "report_threshold_n": 0.0,
            "stage_usd_readback_verified": True,
        },
    }


def test_contact_probe_accepts_only_zero_counts_for_every_wheel_and_filter():
    initialized_probe().assert_all_wheels_separated(physics_dt_s=1.0 / 60.0)

    probe = initialized_probe(
        counts=((0, 0), (0, 1), (0, 0), (0, 0))
    )
    with pytest.raises(
        ResetStrategyError,
        match=r"front_right_wheel_joint=1@z=0\.098000m",
    ):
        probe.assert_all_wheels_separated(physics_dt_s=1.0 / 60.0)


def test_contact_probe_rejects_invalid_runtime_view_or_count_shape():
    probe = WheelGroundContactProbe(
        wheel_bindings=WHEEL_BINDINGS,
        ground_filter_paths=GROUND_FILTERS,
        stage_usd_readback_verified=True,
    )
    with pytest.raises(ResetStrategyError, match="not initialized"):
        probe.assert_all_wheels_separated(physics_dt_s=1.0 / 60.0)

    bad_view = FakeContactView(
        counts=((0, 0),) * 3,
        num_contact_filters=2,
    )
    probe.initialize(
        FakeApp(),
        view_factory=lambda *args, **kwargs: bad_view,
    )
    with pytest.raises(ResetStrategyError, match="contact count shape"):
        probe.assert_all_wheels_separated(physics_dt_s=1.0 / 60.0)


def test_contact_probe_exposes_only_an_initialized_runtime_view():
    probe = WheelGroundContactProbe(
        wheel_bindings=WHEEL_BINDINGS,
        ground_filter_paths=GROUND_FILTERS,
        stage_usd_readback_verified=True,
    )
    with pytest.raises(ResetStrategyError, match="not initialized"):
        _ = probe.view

    view = FakeContactView(counts=((0, 0),) * 4)
    probe.initialize(
        FakeApp(),
        view_factory=lambda *args, **kwargs: view,
    )
    assert probe.view is view


@pytest.mark.isaac
@pytest.mark.skipif(not HAS_PXR, reason="Isaac/USD pxr bindings are unavailable")
def test_contact_reports_are_authored_on_all_wheels_before_physics():
    config = load_project_config(
        ROOT / "isaac_sim/configs/simple_plane.project.yaml",
        {
            "PROJECT_ROOT": str(ROOT),
            "ISAAC_ASSET_ROOT": (
                "/home/lyb/isaacsim_assets/Assets/Isaac/6.0"
            ),
        },
    )
    from isaac_sim.src.stage.scene_composer import SceneComposer

    composer = SceneComposer(config)
    stage = composer.compose(save=False)
    assert composer.ground_topology_snapshot is not None

    probe = author_wheel_ground_contact_probe(
        stage,
        config,
        composer.ground_topology_snapshot.target_colliders,
    )

    assert tuple(binding.joint_name for binding in probe.wheel_bindings) == (
        config.robot.wheel_joints
    )
    for binding in probe.wheel_bindings:
        prim = stage.GetPrimAtPath(binding.wheel_link_path)
        assert prim.HasAPI(PhysxSchema.PhysxContactReportAPI)
        assert (
            PhysxSchema.PhysxContactReportAPI(prim).GetThresholdAttr().Get()
            == pytest.approx(0.0)
        )
