"""ROS-independent transform and result-gating logic for V6-GRID."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Tuple


@dataclass(frozen=True)
class RigidTransform:
    """Translation and quaternion for a parent-to-child transform."""

    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class GateDecision:
    accepted: bool
    reason: str
    generation: int
    trigger_stamp_ns: int
    result_stamp_ns: int


def _quaternion(transform: RigidTransform) -> Tuple[float, float, float, float]:
    return (transform.qx, transform.qy, transform.qz, transform.qw)


def _normalized_quaternion(
        quaternion: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    norm = math.sqrt(sum(component * component for component in quaternion))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError('quaternion must be finite and non-zero')
    return tuple(component / norm for component in quaternion)


def _multiply_quaternions(
        left: Tuple[float, float, float, float],
        right: Tuple[float, float, float, float]
) -> Tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def _rotate_vector(
        quaternion: Tuple[float, float, float, float],
        vector: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    qx, qy, qz, qw = quaternion
    vx, vy, vz = vector
    # Unit-quaternion rotation without constructing a matrix.
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def compose(left: RigidTransform, right: RigidTransform) -> RigidTransform:
    """Return ``T_a_c = T_a_b * T_b_c``."""
    left_q = _normalized_quaternion(_quaternion(left))
    right_q = _normalized_quaternion(_quaternion(right))
    rotated = _rotate_vector(left_q, (right.x, right.y, right.z))
    result_q = _normalized_quaternion(
        _multiply_quaternions(left_q, right_q))
    return RigidTransform(
        left.x + rotated[0],
        left.y + rotated[1],
        left.z + rotated[2],
        *result_q,
    )


def inverse(transform: RigidTransform) -> RigidTransform:
    quaternion = _normalized_quaternion(_quaternion(transform))
    inverse_q = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    translation = _rotate_vector(
        inverse_q, (-transform.x, -transform.y, -transform.z))
    return RigidTransform(*translation, *inverse_q)


def map_to_odom(
        map_to_base: RigidTransform,
        odom_to_base: RigidTransform
) -> RigidTransform:
    """Compute ``T_map_odom = T_map_base * inverse(T_odom_base)``."""
    return compose(map_to_base, inverse(odom_to_base))


def yaw(transform: RigidTransform) -> float:
    qx, qy, qz, qw = _normalized_quaternion(_quaternion(transform))
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


class LocalizationGate:
    """One outstanding localization result per accepted trigger."""

    def __init__(self) -> None:
        self.generation = 0
        self.pending_generation: Optional[int] = None
        self.trigger_stamp_ns = 0
        self.last_accepted_result_stamp_ns = 0

    def begin_trigger(self, trigger_stamp_ns: int) -> GateDecision:
        if self.pending_generation is not None:
            return GateDecision(
                False, 'request_already_pending', self.generation,
                self.trigger_stamp_ns, 0)
        self.generation += 1
        self.pending_generation = self.generation
        self.trigger_stamp_ns = trigger_stamp_ns
        return GateDecision(
            True, 'trigger_accepted', self.generation,
            self.trigger_stamp_ns, 0)

    def reject_pending(self, reason: str, result_stamp_ns: int = 0) -> GateDecision:
        decision = GateDecision(
            False, reason, self.generation, self.trigger_stamp_ns,
            result_stamp_ns)
        self.pending_generation = None
        return decision

    def expire_pending(
            self,
            now_ns: int,
            timeout_ns: int
    ) -> Optional[GateDecision]:
        """Reject a pending generation once its trigger timeout has elapsed."""
        if self.pending_generation is None:
            return None
        if now_ns - self.trigger_stamp_ns < timeout_ns:
            return None
        return self.reject_pending('localization_timeout')

    def classify_result(
            self,
            result_stamp_ns: int,
            finite: bool,
            has_same_stamp_tf: bool
    ) -> GateDecision:
        if self.pending_generation is None:
            return GateDecision(
                False, 'no_pending_generation', self.generation,
                self.trigger_stamp_ns, result_stamp_ns)
        if result_stamp_ns <= 0:
            return self.reject_pending('invalid_result_stamp', result_stamp_ns)
        if result_stamp_ns < self.trigger_stamp_ns:
            return self.reject_pending(
                'result_before_current_trigger', result_stamp_ns)
        if result_stamp_ns <= self.last_accepted_result_stamp_ns:
            return self.reject_pending('stale_result', result_stamp_ns)
        if not finite:
            return self.reject_pending('non_finite_result', result_stamp_ns)
        if not has_same_stamp_tf:
            return self.reject_pending(
                'missing_same_stamp_odom_to_base', result_stamp_ns)

        decision = GateDecision(
            True, 'accepted', self.generation, self.trigger_stamp_ns,
            result_stamp_ns)
        self.last_accepted_result_stamp_ns = result_stamp_ns
        self.pending_generation = None
        return decision


STATUS_KEYS = (
    'generation',
    'state',
    'accepted',
    'reason',
    'trigger_stamp_ns',
    'result_stamp_ns',
    'correction_x_m',
    'correction_y_m',
    'correction_yaw_rad',
    'latency_s',
)


def status_values(
        decision: GateDecision,
        state: str,
        correction: Optional[RigidTransform],
        latency_s: float
) -> Tuple[Tuple[str, str], ...]:
    correction_x = correction.x if correction is not None else 0.0
    correction_y = correction.y if correction is not None else 0.0
    correction_yaw = yaw(correction) if correction is not None else 0.0
    values = (
        str(decision.generation),
        state,
        'true' if decision.accepted and state == 'ACCEPTED' else 'false',
        decision.reason,
        str(decision.trigger_stamp_ns),
        str(decision.result_stamp_ns),
        f'{correction_x:.9f}',
        f'{correction_y:.9f}',
        f'{correction_yaw:.9f}',
        f'{max(0.0, latency_s):.9f}',
    )
    return tuple(zip(STATUS_KEYS, values))
