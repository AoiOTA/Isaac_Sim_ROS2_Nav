"""Versioned reset strategies and wheel-ground contact instrumentation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable, Iterable, Sequence

from isaac_sim.src.config import ResetStrategyConfig


class ResetStrategyError(RuntimeError):
    pass


POSE_RESTORE_V1 = "pose_restore_v1"
SEPARATE_RECONTACT_0P20M_1STEP_V1 = (
    "separate_recontact_0p20m_1step_v1"
)
CONTACT_PROBE_SCHEMA_VERSION = 1
RESET_STRATEGY_SNAPSHOT_SCHEMA_VERSION = 1
MAX_CONTACT_COUNT = 128
REPORT_THRESHOLD_N = 0.0


@dataclass(frozen=True)
class ResetStrategySpec:
    identifier: str
    lift_distance_m: float
    separation_step_count: int
    recontact_step_count: int = 1


_RESET_STRATEGY_SPECS = {
    POSE_RESTORE_V1: ResetStrategySpec(
        identifier=POSE_RESTORE_V1,
        lift_distance_m=0.0,
        separation_step_count=0,
    ),
    SEPARATE_RECONTACT_0P20M_1STEP_V1: ResetStrategySpec(
        identifier=SEPARATE_RECONTACT_0P20M_1STEP_V1,
        lift_distance_m=0.2,
        separation_step_count=1,
    ),
}


def reset_strategy_spec(
    strategy: ResetStrategyConfig | str,
) -> ResetStrategySpec:
    if isinstance(strategy, ResetStrategyConfig):
        if strategy.schema_version != 1:
            raise ResetStrategyError("reset strategy schema_version must be 1")
        identifier = strategy.identifier
    elif isinstance(strategy, str):
        identifier = strategy
    else:
        raise ResetStrategyError("reset strategy must be configured explicitly")
    try:
        return _RESET_STRATEGY_SPECS[identifier]
    except KeyError as exc:
        raise ResetStrategyError(
            f"unknown reset strategy {identifier!r}; "
            f"available={sorted(_RESET_STRATEGY_SPECS)}"
        ) from exc


@dataclass(frozen=True)
class WheelBinding:
    joint_name: str
    wheel_link_path: str

    def __post_init__(self) -> None:
        if not self.joint_name:
            raise ResetStrategyError("wheel binding joint_name must be non-empty")
        if not self.wheel_link_path.startswith("/"):
            raise ResetStrategyError(
                "wheel binding path must be an absolute USD prim path"
            )


def _normalized_ground_filter_paths(paths: Iterable[str]) -> tuple[str, ...]:
    values = tuple(paths)
    if not values or not all(
        isinstance(path, str) and path.startswith("/") for path in values
    ):
        raise ResetStrategyError(
            "ground contact filters must be non-empty absolute USD prim paths"
        )
    if len(set(values)) != len(values):
        raise ResetStrategyError("ground contact filters must not contain duplicates")
    return tuple(sorted(values))


class WheelGroundContactProbe:
    """One filtered four-wheel contact view shared by every reset strategy."""

    def __init__(
        self,
        *,
        wheel_bindings: Sequence[WheelBinding],
        ground_filter_paths: Iterable[str],
        stage_usd_readback_verified: bool,
    ) -> None:
        bindings = tuple(wheel_bindings)
        if len(bindings) != 4:
            raise ResetStrategyError(
                "wheel-ground contact probe requires exactly four wheel bindings"
            )
        if len({binding.joint_name for binding in bindings}) != 4:
            raise ResetStrategyError("wheel binding joint names must be unique")
        if len({binding.wheel_link_path for binding in bindings}) != 4:
            raise ResetStrategyError("wheel rigid-body paths must be unique")
        if stage_usd_readback_verified is not True:
            raise ResetStrategyError(
                "contact-report Stage USD readback must be verified"
            )
        self.wheel_bindings = bindings
        self.ground_filter_paths = _normalized_ground_filter_paths(
            ground_filter_paths
        )
        self.stage_usd_readback_verified = True
        self._view: Any | None = None

    @property
    def view(self) -> Any:
        if self._view is None:
            raise ResetStrategyError("wheel-ground contact probe is not initialized")
        return self._view

    def initialize(
        self,
        app: Any,
        *,
        view_factory: Callable[..., Any] | None = None,
    ) -> None:
        if self._view is not None:
            raise ResetStrategyError("wheel-ground contact probe is already initialized")
        if view_factory is None:
            from isaacsim.core.experimental.prims import RigidPrim

            view_factory = RigidPrim
        view = view_factory(
            [binding.wheel_link_path for binding in self.wheel_bindings],
            contact_filter_paths=list(self.ground_filter_paths),
            max_contact_count=MAX_CONTACT_COUNT,
        )
        app.update()
        if not view.is_physics_tensor_entity_valid():
            raise ResetStrategyError(
                "wheel-ground contact tensor view is invalid after physics warmup"
            )
        if int(view.num_contact_filters) != len(self.ground_filter_paths):
            raise ResetStrategyError(
                "wheel-ground contact filter count mismatch: "
                f"expected={len(self.ground_filter_paths)}, "
                f"actual={view.num_contact_filters}"
            )
        self._view = view

    def assert_all_wheels_separated(self, *, physics_dt_s: float) -> None:
        if self._view is None:
            raise ResetStrategyError("wheel-ground contact probe is not initialized")
        if (
            isinstance(physics_dt_s, bool)
            or not isinstance(physics_dt_s, (int, float))
            or not math.isfinite(physics_dt_s)
            or physics_dt_s <= 0.0
        ):
            raise ResetStrategyError("physics_dt_s must be finite and positive")
        _, _, _, _, counts, _ = self._view.get_contact_force_data(
            dt=float(physics_dt_s)
        )
        values = counts.numpy()
        expected_shape = (4, len(self.ground_filter_paths))
        if tuple(values.shape) != expected_shape:
            raise ResetStrategyError(
                "wheel-ground contact count shape mismatch: "
                f"expected={expected_shape}, actual={tuple(values.shape)}"
            )
        active: list[str] = []
        for wheel_index, binding in enumerate(self.wheel_bindings):
            total = 0
            for raw_value in values[wheel_index]:
                numeric = float(raw_value)
                if (
                    not math.isfinite(numeric)
                    or numeric < 0.0
                    or not numeric.is_integer()
                ):
                    raise ResetStrategyError(
                        "wheel-ground contact counts must be finite "
                        "non-negative integers"
                    )
                total += int(numeric)
            if total:
                active.append(f"{binding.joint_name}={total}")
        if active:
            raise ResetStrategyError(
                "wheel-ground contact remains after separation step: "
                + ", ".join(active)
            )

    def provenance_snapshot(
        self,
        strategy: ResetStrategyConfig | str,
    ) -> dict[str, object]:
        if self._view is None:
            raise ResetStrategyError(
                "wheel-ground contact probe must be initialized before provenance capture"
            )
        spec = reset_strategy_spec(strategy)
        return {
            "schema_version": RESET_STRATEGY_SNAPSHOT_SCHEMA_VERSION,
            "id": spec.identifier,
            "lift_distance_m": spec.lift_distance_m,
            "separation_step_count": spec.separation_step_count,
            "recontact_step_count": spec.recontact_step_count,
            "contact_probe": {
                "schema_version": CONTACT_PROBE_SCHEMA_VERSION,
                "enabled": True,
                "wheel_bindings": [
                    {
                        "joint_name": binding.joint_name,
                        "wheel_link_path": binding.wheel_link_path,
                    }
                    for binding in self.wheel_bindings
                ],
                "wheel_count": len(self.wheel_bindings),
                "ground_filter_paths": list(self.ground_filter_paths),
                "ground_filter_count": len(self.ground_filter_paths),
                "max_contact_count": MAX_CONTACT_COUNT,
                "report_threshold_n": REPORT_THRESHOLD_N,
                "stage_usd_readback_verified": (
                    self.stage_usd_readback_verified
                ),
            },
        }


def _resolve_wheel_bindings(stage: Any, config: Any) -> tuple[WheelBinding, ...]:
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(config.robot.articulation_root)
    if not root or not root.IsValid() or not root.IsActive():
        raise ResetStrategyError(
            "reset contact probe articulation root is invalid or inactive"
        )
    bindings: list[WheelBinding] = []
    for joint_name in config.robot.wheel_joints:
        matches = [
            prim
            for prim in Usd.PrimRange(root)
            if prim.IsActive()
            and prim.GetName() == joint_name
            and prim.IsA(UsdPhysics.RevoluteJoint)
        ]
        if len(matches) != 1:
            raise ResetStrategyError(
                f"wheel joint {joint_name!r} resolved to {len(matches)} prims"
            )
        joint = UsdPhysics.Joint(matches[0])
        body0 = tuple(joint.GetBody0Rel().GetTargets())
        body1 = tuple(joint.GetBody1Rel().GetTargets())
        if len(body0) != 1 or len(body1) != 1:
            raise ResetStrategyError(
                f"wheel joint {joint_name!r} must have one Body0 and Body1"
            )
        wheel_paths = [
            path
            for path in body0 + body1
            if str(path) != config.robot.base_link_prim
        ]
        if len(wheel_paths) != 1:
            raise ResetStrategyError(
                f"wheel joint {joint_name!r} must connect one wheel rigid body"
            )
        wheel_path = str(wheel_paths[0])
        wheel_prim = stage.GetPrimAtPath(wheel_path)
        if (
            not wheel_prim
            or not wheel_prim.IsValid()
            or not wheel_prim.IsActive()
            or not wheel_prim.HasAPI(UsdPhysics.RigidBodyAPI)
        ):
            raise ResetStrategyError(
                f"wheel link is not an active rigid body: {wheel_path}"
            )
        bindings.append(WheelBinding(joint_name, wheel_path))
    return tuple(bindings)


def author_wheel_ground_contact_probe(
    stage: Any,
    config: Any,
    ground_filter_paths: Iterable[str],
) -> WheelGroundContactProbe:
    """Author reports before PhysX parses the Stage and return an unbound probe."""

    from pxr import PhysxSchema

    filters = _normalized_ground_filter_paths(ground_filter_paths)
    bindings = _resolve_wheel_bindings(stage, config)
    original_target = stage.GetEditTarget()
    try:
        stage.SetEditTarget(stage.GetSessionLayer())
        for binding in bindings:
            prim = stage.GetPrimAtPath(binding.wheel_link_path)
            report_api = PhysxSchema.PhysxContactReportAPI.Apply(prim)
            if not report_api:
                raise ResetStrategyError(
                    "failed to apply PhysxContactReportAPI: "
                    f"{binding.wheel_link_path}"
                )
            threshold = report_api.CreateThresholdAttr()
            if not threshold.Set(REPORT_THRESHOLD_N):
                raise ResetStrategyError(
                    "failed to author zero contact-report threshold: "
                    f"{binding.wheel_link_path}"
                )
            if (
                not prim.HasAPI(PhysxSchema.PhysxContactReportAPI)
                or float(threshold.Get()) != REPORT_THRESHOLD_N
            ):
                raise ResetStrategyError(
                    "contact-report Stage USD readback mismatch: "
                    f"{binding.wheel_link_path}"
                )
    finally:
        stage.SetEditTarget(original_target)
    return WheelGroundContactProbe(
        wheel_bindings=bindings,
        ground_filter_paths=filters,
        stage_usd_readback_verified=True,
    )


__all__ = [
    "POSE_RESTORE_V1",
    "SEPARATE_RECONTACT_0P20M_1STEP_V1",
    "ResetStrategyError",
    "ResetStrategySpec",
    "WheelBinding",
    "WheelGroundContactProbe",
    "author_wheel_ground_contact_probe",
    "reset_strategy_spec",
]
