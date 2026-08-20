"""Pure SE(2) metrics for timestamped estimated-state trajectories."""

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class PoseSample:
    """One planar pose sample and its optional flattened covariance."""

    stamp_ns: int
    x: float
    y: float
    yaw: float
    covariance: Optional[Tuple[float, ...]] = None


@dataclass(frozen=True)
class EvaluationResult:
    """JSON-ready summary and CSV-ready per-match rows."""

    summary: dict
    rows: Tuple[dict, ...]


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def stream_diagnostics(samples: Sequence[PoseSample]) -> dict:
    """Summarize arrival-order timing and covariance health."""
    duplicate = 0
    backward = 0
    positive_deltas_ns = []
    for previous, current in zip(samples, samples[1:]):
        delta_ns = current.stamp_ns - previous.stamp_ns
        if delta_ns == 0:
            duplicate += 1
        elif delta_ns < 0:
            backward += 1
        else:
            positive_deltas_ns.append(delta_ns)

    frequency_hz = None
    if positive_deltas_ns:
        frequency_hz = (
            1.0e9 * len(positive_deltas_ns) / sum(positive_deltas_ns)
        )

    present = [sample.covariance for sample in samples
               if sample.covariance is not None]
    finite = [values for values in present
              if all(math.isfinite(value) for value in values)]
    nonnegative_diagonal = 0
    for values in finite:
        indices = _planar_covariance_indices(len(values))
        if indices and all(values[index] >= 0.0 for index in indices):
            nonnegative_diagonal += 1

    sample_count = len(samples)
    present_count = len(present)
    finite_count = len(finite)
    return {
        'received': sample_count,
        'duplicate_stamps': duplicate,
        'backward_stamps': backward,
        'positive_delta_count': len(positive_deltas_ns),
        'frequency_hz': frequency_hz,
        'covariance': {
            'present_count': present_count,
            'coverage_fraction': (
                present_count / sample_count if sample_count else None
            ),
            'finite_count': finite_count,
            'finite_fraction': (
                finite_count / present_count if present_count else None
            ),
            'nonnegative_planar_diagonal_count': nonnegative_diagonal,
            'nonnegative_planar_diagonal_fraction': (
                nonnegative_diagonal / present_count
                if present_count else None
            ),
        },
    }


def evaluate_trajectory(
    estimate: Sequence[PoseSample],
    ground_truth: Sequence[PoseSample],
    max_time_delta_ns: int,
) -> EvaluationResult:
    """Associate, first-frame-align, and score one estimated trajectory."""
    if max_time_delta_ns < 0:
        raise ValueError('max_time_delta_ns must be non-negative')

    clean_estimate = _finite_unique_sorted(estimate)
    clean_ground_truth = _finite_unique_sorted(ground_truth)
    pairs = _associate_nearest(
        clean_estimate, clean_ground_truth, max_time_delta_ns)

    summary = {
        'input': stream_diagnostics(estimate),
        'valid_unique_samples': len(clean_estimate),
        'association': {
            'matched_count': len(pairs),
            'max_time_delta_ns': max_time_delta_ns,
            'mean_abs_time_delta_ms': (
                sum(abs(est.stamp_ns - gt.stamp_ns) for est, gt in pairs)
                / len(pairs) / 1.0e6
                if pairs else None
            ),
            'max_abs_time_delta_ms': (
                max(abs(est.stamp_ns - gt.stamp_ns) for est, gt in pairs)
                / 1.0e6
                if pairs else None
            ),
        },
        'alignment': None,
        'relative_ate': {
            'xy_m': _metric_summary([]),
            'yaw_rad': _metric_summary([]),
        },
        'rpe': {
            'xy_m': _metric_summary([]),
            'yaw_rad': _metric_summary([]),
        },
    }
    if not pairs:
        return EvaluationResult(summary=summary, rows=())

    first_estimate, first_ground_truth = pairs[0]
    rotation = wrap_angle(first_ground_truth.yaw - first_estimate.yaw)
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    translation_x = (
        first_ground_truth.x
        - (cosine * first_estimate.x - sine * first_estimate.y)
    )
    translation_y = (
        first_ground_truth.y
        - (sine * first_estimate.x + cosine * first_estimate.y)
    )
    summary['alignment'] = {
        'translation_x_m': translation_x,
        'translation_y_m': translation_y,
        'rotation_rad': rotation,
    }

    aligned_pairs = []
    ate_xy = []
    ate_yaw = []
    rows = []
    for estimate_sample, truth_sample in pairs:
        aligned = PoseSample(
            stamp_ns=estimate_sample.stamp_ns,
            x=(cosine * estimate_sample.x - sine * estimate_sample.y
               + translation_x),
            y=(sine * estimate_sample.x + cosine * estimate_sample.y
               + translation_y),
            yaw=wrap_angle(estimate_sample.yaw + rotation),
            covariance=estimate_sample.covariance,
        )
        aligned_pairs.append((aligned, truth_sample))
        xy_error = math.hypot(
            aligned.x - truth_sample.x, aligned.y - truth_sample.y)
        yaw_error = wrap_angle(aligned.yaw - truth_sample.yaw)
        ate_xy.append(xy_error)
        ate_yaw.append(yaw_error)
        rows.append({
            'estimate_stamp_ns': estimate_sample.stamp_ns,
            'ground_truth_stamp_ns': truth_sample.stamp_ns,
            'time_delta_ms': (
                estimate_sample.stamp_ns - truth_sample.stamp_ns
            ) / 1.0e6,
            'aligned_x_m': aligned.x,
            'aligned_y_m': aligned.y,
            'aligned_yaw_rad': aligned.yaw,
            'ground_truth_x_m': truth_sample.x,
            'ground_truth_y_m': truth_sample.y,
            'ground_truth_yaw_rad': truth_sample.yaw,
            'ate_xy_m': xy_error,
            'ate_yaw_rad': yaw_error,
        })

    rpe_xy = []
    rpe_yaw = []
    for (previous_estimate, previous_truth), (current_estimate, current_truth) in zip(
        aligned_pairs, aligned_pairs[1:]
    ):
        estimate_delta = _relative_pose(previous_estimate, current_estimate)
        truth_delta = _relative_pose(previous_truth, current_truth)
        rpe_xy.append(math.hypot(
            estimate_delta[0] - truth_delta[0],
            estimate_delta[1] - truth_delta[1],
        ))
        rpe_yaw.append(wrap_angle(estimate_delta[2] - truth_delta[2]))

    summary['relative_ate'] = {
        'xy_m': _metric_summary(ate_xy),
        'yaw_rad': _metric_summary(ate_yaw),
    }
    summary['rpe'] = {
        'xy_m': _metric_summary(rpe_xy),
        'yaw_rad': _metric_summary(rpe_yaw),
    }
    return EvaluationResult(summary=summary, rows=tuple(rows))


def _finite_unique_sorted(samples):
    unique = {}
    for sample in samples:
        if (
            sample.stamp_ns >= 0
            and all(math.isfinite(value) for value in (
                sample.x, sample.y, sample.yaw))
        ):
            unique.setdefault(sample.stamp_ns, sample)
    return [unique[stamp] for stamp in sorted(unique)]


def _associate_nearest(estimate, ground_truth, max_time_delta_ns):
    if not estimate or not ground_truth:
        return []
    truth_stamps = [sample.stamp_ns for sample in ground_truth]
    candidates = []
    for estimate_index, estimate_sample in enumerate(estimate):
        start = bisect_left(
            truth_stamps, estimate_sample.stamp_ns - max_time_delta_ns)
        stop = bisect_right(
            truth_stamps, estimate_sample.stamp_ns + max_time_delta_ns)
        for truth_index in range(start, stop):
            candidates.append((
                abs(estimate_sample.stamp_ns - truth_stamps[truth_index]),
                estimate_index,
                truth_index,
            ))

    used_estimate = set()
    used_truth = set()
    selected = []
    for _, estimate_index, truth_index in sorted(candidates):
        if estimate_index in used_estimate or truth_index in used_truth:
            continue
        used_estimate.add(estimate_index)
        used_truth.add(truth_index)
        selected.append((estimate[estimate_index], ground_truth[truth_index]))
    return sorted(selected, key=lambda pair: pair[0].stamp_ns)


def _relative_pose(start, end):
    delta_x = end.x - start.x
    delta_y = end.y - start.y
    cosine = math.cos(start.yaw)
    sine = math.sin(start.yaw)
    return (
        cosine * delta_x + sine * delta_y,
        -sine * delta_x + cosine * delta_y,
        wrap_angle(end.yaw - start.yaw),
    )


def _metric_summary(values):
    if not values:
        return {
            'count': 0,
            'rmse': None,
            'mean_abs': None,
            'max_abs': None,
        }
    absolute = [abs(value) for value in values]
    return {
        'count': len(values),
        'rmse': math.sqrt(sum(value * value for value in values) / len(values)),
        'mean_abs': sum(absolute) / len(absolute),
        'max_abs': max(absolute),
    }


def _planar_covariance_indices(length):
    if length >= 36:
        return 0, 7, 35
    if length >= 9:
        return 0, 4, 8
    return ()
