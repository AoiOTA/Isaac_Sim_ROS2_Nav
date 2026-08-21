"""Pure SE(2) metrics for timestamped estimated-state trajectories."""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


MINIMUM_LINEAR_SCALE_DENOMINATOR_M = 0.5
MINIMUM_YAW_SCALE_DENOMINATOR_RAD = 0.5


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


def stream_diagnostics(
    samples: Sequence[PoseSample],
    *,
    maximum_pose_step_m: float = 0.5,
    maximum_yaw_step_rad: float = math.radians(30.0),
) -> dict:
    """Summarize arrival-order timing, jumps, and planar covariance health."""
    duplicate = 0
    backward = 0
    pose_jumps = 0
    yaw_jumps = 0
    positive_deltas_ns = []
    for previous, current in zip(samples, samples[1:]):
        delta_ns = current.stamp_ns - previous.stamp_ns
        if delta_ns == 0:
            duplicate += 1
        elif delta_ns < 0:
            backward += 1
        else:
            positive_deltas_ns.append(delta_ns)
            if math.hypot(current.x - previous.x, current.y - previous.y) > maximum_pose_step_m:
                pose_jumps += 1
            if abs(wrap_angle(current.yaw - previous.yaw)) > maximum_yaw_step_rad:
                yaw_jumps += 1

    frequency_hz = None
    if positive_deltas_ns:
        frequency_hz = 1.0e9 * len(positive_deltas_ns) / sum(positive_deltas_ns)

    present = [sample.covariance for sample in samples if sample.covariance is not None]
    finite = [values for values in present if all(math.isfinite(value) for value in values)]
    symmetric_count = 0
    psd_count = 0
    for values in finite:
        matrix = _planar_covariance(values)
        if matrix is not None and _symmetric(matrix):
            symmetric_count += 1
            if _positive_semidefinite(matrix):
                psd_count += 1

    sample_count = len(samples)
    present_count = len(present)
    finite_count = len(finite)
    return {
        'received': sample_count,
        'duplicate_stamps': duplicate,
        'backward_stamps': backward,
        'positive_delta_count': len(positive_deltas_ns),
        'frequency_hz': frequency_hz,
        'pose_jump_count': pose_jumps,
        'yaw_jump_count': yaw_jumps,
        'jump_thresholds': {
            'position_m': maximum_pose_step_m,
            'yaw_rad': maximum_yaw_step_rad,
        },
        'covariance': {
            'present_count': present_count,
            'coverage_fraction': present_count / sample_count if sample_count else None,
            'finite_count': finite_count,
            'finite_fraction': finite_count / present_count if present_count else None,
            # Retained for old report consumers.
            'nonnegative_planar_diagonal_count': psd_count,
            'nonnegative_planar_diagonal_fraction': psd_count / present_count if present_count else None,
            'symmetric_count': symmetric_count,
            'symmetric_fraction': symmetric_count / present_count if present_count else None,
            'positive_semidefinite_count': psd_count,
            'positive_semidefinite_fraction': psd_count / present_count if present_count else None,
        },
    }


def evaluate_trajectory(
    estimate: Sequence[PoseSample],
    ground_truth: Sequence[PoseSample],
    max_time_delta_ns: int,
    *,
    max_time_offset_ns: int = 0,
    time_offset_step_ns: int = 10_000_000,
    rpe_time_ns: int = 1_000_000_000,
    rpe_time_tolerance_ns: int = 100_000_000,
    rpe_distance_m: float = 1.0,
) -> EvaluationResult:
    """Associate and score absolute/aligned ATE, fixed RPE, scale and covariance."""
    if max_time_delta_ns < 0 or max_time_offset_ns < 0:
        raise ValueError('time bounds must be non-negative')
    if time_offset_step_ns <= 0 or rpe_time_ns <= 0 or rpe_time_tolerance_ns < 0:
        raise ValueError('time steps must be positive and tolerances non-negative')
    if not math.isfinite(rpe_distance_m) or rpe_distance_m <= 0.0:
        raise ValueError('rpe_distance_m must be finite and positive')

    clean_estimate = _finite_unique_sorted(estimate)
    clean_ground_truth = _finite_unique_sorted(ground_truth)
    selected_offset_ns, pairs = _select_time_offset(
        clean_estimate,
        clean_ground_truth,
        max_time_delta_ns,
        max_time_offset_ns,
        time_offset_step_ns,
    )

    empty_pose_metric = lambda: {'xy_m': _metric_summary([]), 'yaw_rad': _metric_summary([])}
    summary = {
        'input': stream_diagnostics(estimate),
        'valid_unique_samples': len(clean_estimate),
        'association': {
            'matched_count': len(pairs),
            'max_time_delta_ns': max_time_delta_ns,
            'best_estimate_time_offset_ns': selected_offset_ns,
            'best_estimate_time_offset_ms': selected_offset_ns / 1.0e6,
            'search_bound_ns': max_time_offset_ns,
            'mean_abs_time_delta_ms': None,
            'max_abs_time_delta_ms': None,
        },
        'alignment': None,
        'absolute_ate': empty_pose_metric(),
        'aligned_ate': empty_pose_metric(),
        'relative_ate': empty_pose_metric(),
        'rpe': empty_pose_metric(),
        'rpe_fixed_1s': empty_pose_metric(),
        'rpe_fixed_1m': empty_pose_metric(),
        'endpoint': None,
        'scale': {
            'linear': None,
            'yaw': None,
            'yaw_change_bias_rad': None,
            'minimum_linear_denominator_m': MINIMUM_LINEAR_SCALE_DENOMINATOR_M,
            'minimum_yaw_denominator_rad': MINIMUM_YAW_SCALE_DENOMINATOR_RAD,
        },
        'covariance_consistency': _empty_covariance_consistency(),
        'planar_nees': {'status': 'NOT_AVAILABLE', 'count': 0, 'summary': _metric_summary([])},
        'nis': {
            'status': 'NOT_AVAILABLE',
            'reason': 'innovation and innovation covariance are not recorded',
        },
    }
    if not pairs:
        return EvaluationResult(summary=summary, rows=())

    residuals = [abs(est.stamp_ns + selected_offset_ns - gt.stamp_ns) for est, gt in pairs]
    summary['association']['mean_abs_time_delta_ms'] = sum(residuals) / len(residuals) / 1.0e6
    summary['association']['max_abs_time_delta_ms'] = max(residuals) / 1.0e6

    first_estimate, first_ground_truth = pairs[0]
    rotation = wrap_angle(first_ground_truth.yaw - first_estimate.yaw)
    cosine = math.cos(rotation)
    sine = math.sin(rotation)
    translation_x = first_ground_truth.x - (cosine * first_estimate.x - sine * first_estimate.y)
    translation_y = first_ground_truth.y - (sine * first_estimate.x + cosine * first_estimate.y)
    summary['alignment'] = {
        'method': 'first_frame_se2',
        'translation_x_m': translation_x,
        'translation_y_m': translation_y,
        'rotation_rad': rotation,
    }

    aligned_pairs = []
    absolute_xy = []
    absolute_yaw = []
    aligned_xy = []
    aligned_yaw = []
    nees_values = []
    coverage = []
    rows = []
    for estimate_sample, truth_sample in pairs:
        aligned = PoseSample(
            stamp_ns=estimate_sample.stamp_ns,
            x=cosine * estimate_sample.x - sine * estimate_sample.y + translation_x,
            y=sine * estimate_sample.x + cosine * estimate_sample.y + translation_y,
            yaw=wrap_angle(estimate_sample.yaw + rotation),
            covariance=estimate_sample.covariance,
        )
        aligned_pairs.append((aligned, truth_sample))
        abs_error = (
            estimate_sample.x - truth_sample.x,
            estimate_sample.y - truth_sample.y,
            wrap_angle(estimate_sample.yaw - truth_sample.yaw),
        )
        aligned_error = (
            aligned.x - truth_sample.x,
            aligned.y - truth_sample.y,
            wrap_angle(aligned.yaw - truth_sample.yaw),
        )
        absolute_xy.append(math.hypot(abs_error[0], abs_error[1]))
        absolute_yaw.append(abs_error[2])
        aligned_xy.append(math.hypot(aligned_error[0], aligned_error[1]))
        aligned_yaw.append(aligned_error[2])

        covariance = _planar_covariance(estimate_sample.covariance)
        nees = None
        covered = (None, None, None)
        if covariance is not None and _symmetric(covariance) and _positive_semidefinite(covariance):
            covered = tuple(
                abs(error) <= 2.0 * math.sqrt(max(covariance[index][index], 0.0))
                for index, error in enumerate(abs_error)
            )
            coverage.append(covered)
            inverse = _inverse_3x3(covariance)
            if inverse is not None:
                nees = sum(
                    abs_error[row] * inverse[row][column] * abs_error[column]
                    for row in range(3)
                    for column in range(3)
                )
                if math.isfinite(nees) and nees >= -1.0e-9:
                    nees_values.append(max(nees, 0.0))
                else:
                    nees = None
        rows.append({
            'estimate_stamp_ns': estimate_sample.stamp_ns,
            'ground_truth_stamp_ns': truth_sample.stamp_ns,
            'time_delta_ms': (estimate_sample.stamp_ns + selected_offset_ns - truth_sample.stamp_ns) / 1.0e6,
            'estimate_x_m': estimate_sample.x,
            'estimate_y_m': estimate_sample.y,
            'estimate_yaw_rad': estimate_sample.yaw,
            'aligned_x_m': aligned.x,
            'aligned_y_m': aligned.y,
            'aligned_yaw_rad': aligned.yaw,
            'ground_truth_x_m': truth_sample.x,
            'ground_truth_y_m': truth_sample.y,
            'ground_truth_yaw_rad': truth_sample.yaw,
            'absolute_ate_xy_m': absolute_xy[-1],
            'absolute_ate_yaw_rad': absolute_yaw[-1],
            'aligned_ate_xy_m': aligned_xy[-1],
            'aligned_ate_yaw_rad': aligned_yaw[-1],
            'ate_xy_m': aligned_xy[-1],
            'ate_yaw_rad': aligned_yaw[-1],
            'covariance_2sigma_x_covered': covered[0],
            'covariance_2sigma_y_covered': covered[1],
            'covariance_2sigma_yaw_covered': covered[2],
            'planar_nees': nees,
        })

    adjacent_rpe = _rpe_for_index_pairs(aligned_pairs, [(i, i + 1) for i in range(len(aligned_pairs) - 1)])
    fixed_time_rpe = _rpe_for_index_pairs(aligned_pairs, _fixed_time_pairs(aligned_pairs, rpe_time_ns, rpe_time_tolerance_ns))
    fixed_distance_rpe = _rpe_for_index_pairs(aligned_pairs, _fixed_distance_pairs(aligned_pairs, rpe_distance_m))
    summary['absolute_ate'] = {'xy_m': _metric_summary(absolute_xy), 'yaw_rad': _metric_summary(absolute_yaw)}
    summary['aligned_ate'] = {'xy_m': _metric_summary(aligned_xy), 'yaw_rad': _metric_summary(aligned_yaw)}
    summary['relative_ate'] = summary['aligned_ate']
    summary['rpe'] = adjacent_rpe
    summary['rpe_fixed_1s'] = {**fixed_time_rpe, 'target_interval_sec': rpe_time_ns / 1.0e9}
    summary['rpe_fixed_1m'] = {**fixed_distance_rpe, 'target_distance_m': rpe_distance_m}
    summary['endpoint'] = _endpoint_summary(pairs, aligned_pairs)
    summary['scale'] = _scale_summary(pairs)
    summary['covariance_consistency'] = {
        'matched_covariance_count': len(coverage),
        'two_sigma_coverage': {
            'x': _fraction_true([value[0] for value in coverage]),
            'y': _fraction_true([value[1] for value in coverage]),
            'yaw': _fraction_true([value[2] for value in coverage]),
            'all_axes': _fraction_true([all(value) for value in coverage]),
            'diagnostic_only': True,
        },
    }
    summary['planar_nees'] = {
        'status': 'AVAILABLE' if nees_values else 'NOT_AVAILABLE',
        'count': len(nees_values),
        'summary': _metric_summary(nees_values),
        'definition': 'ground-truth planar pose error transpose times inverse reported pose covariance times error',
        'not_nis': True,
    }
    return EvaluationResult(summary=summary, rows=tuple(rows))


def _finite_unique_sorted(samples):
    unique = {}
    for sample in samples:
        if sample.stamp_ns >= 0 and all(math.isfinite(value) for value in (sample.x, sample.y, sample.yaw)):
            unique.setdefault(sample.stamp_ns, sample)
    return [unique[stamp] for stamp in sorted(unique)]


def _select_time_offset(estimate, truth, max_delta, bound, step):
    offsets = {0, -bound, bound}
    value = -bound
    while value <= bound:
        offsets.add(value)
        value += step
    best = (0, [])
    best_key = None
    for offset in sorted(offsets):
        pairs = _associate_nearest(estimate, truth, max_delta, offset)
        residual = sum(abs(est.stamp_ns + offset - gt.stamp_ns) for est, gt in pairs)
        key = (-len(pairs), residual / len(pairs) if pairs else math.inf, abs(offset), offset)
        if best_key is None or key < best_key:
            best_key = key
            best = (offset, pairs)
    return best


def _associate_nearest(estimate, ground_truth, max_time_delta_ns, offset_ns=0):
    if not estimate or not ground_truth:
        return []
    truth_stamps = [sample.stamp_ns for sample in ground_truth]
    candidates = []
    for estimate_index, estimate_sample in enumerate(estimate):
        shifted_stamp = estimate_sample.stamp_ns + offset_ns
        start = bisect_left(truth_stamps, shifted_stamp - max_time_delta_ns)
        stop = bisect_right(truth_stamps, shifted_stamp + max_time_delta_ns)
        for truth_index in range(start, stop):
            candidates.append((abs(shifted_stamp - truth_stamps[truth_index]), estimate_index, truth_index))
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


def _rpe_for_index_pairs(pairs, indices):
    xy = []
    yaw = []
    for start_index, end_index in indices:
        start_estimate, start_truth = pairs[start_index]
        end_estimate, end_truth = pairs[end_index]
        estimate_delta = _relative_pose(start_estimate, end_estimate)
        truth_delta = _relative_pose(start_truth, end_truth)
        xy.append(math.hypot(estimate_delta[0] - truth_delta[0], estimate_delta[1] - truth_delta[1]))
        yaw.append(wrap_angle(estimate_delta[2] - truth_delta[2]))
    return {'xy_m': _metric_summary(xy), 'yaw_rad': _metric_summary(yaw)}


def _fixed_time_pairs(pairs, interval_ns, tolerance_ns):
    stamps = [truth.stamp_ns for _, truth in pairs]
    result = []
    for index, stamp in enumerate(stamps):
        target = stamp + interval_ns
        location = bisect_left(stamps, target, index + 1)
        candidates = [candidate for candidate in (location - 1, location) if index < candidate < len(stamps)]
        if candidates:
            selected = min(candidates, key=lambda candidate: abs(stamps[candidate] - target))
            if abs(stamps[selected] - target) <= tolerance_ns:
                result.append((index, selected))
    return result


def _fixed_distance_pairs(pairs, distance_m):
    result = []
    for start in range(len(pairs) - 1):
        travelled = 0.0
        for end in range(start + 1, len(pairs)):
            previous_truth = pairs[end - 1][1]
            current_truth = pairs[end][1]
            travelled += math.hypot(current_truth.x - previous_truth.x, current_truth.y - previous_truth.y)
            if travelled >= distance_m:
                result.append((start, end))
                break
    return result


def _endpoint_summary(raw_pairs, aligned_pairs):
    first_truth = raw_pairs[0][1]
    result = {}
    for name, selected_pairs in (('absolute', raw_pairs), ('aligned', aligned_pairs)):
        end_estimate, end_truth = selected_pairs[-1]
        dx = end_estimate.x - end_truth.x
        dy = end_estimate.y - end_truth.y
        cosine = math.cos(first_truth.yaw)
        sine = math.sin(first_truth.yaw)
        result[name] = {
            'longitudinal_error_m': cosine * dx + sine * dy,
            'lateral_error_m': -sine * dx + cosine * dy,
            'position_error_m': math.hypot(dx, dy),
            'yaw_error_rad': wrap_angle(end_estimate.yaw - end_truth.yaw),
        }
    return result


def _scale_summary(pairs):
    estimate_path = sum(math.hypot(current[0].x - previous[0].x, current[0].y - previous[0].y) for previous, current in zip(pairs, pairs[1:]))
    truth_path = sum(math.hypot(current[1].x - previous[1].x, current[1].y - previous[1].y) for previous, current in zip(pairs, pairs[1:]))
    estimate_yaw = sum(wrap_angle(current[0].yaw - previous[0].yaw) for previous, current in zip(pairs, pairs[1:]))
    truth_yaw = sum(wrap_angle(current[1].yaw - previous[1].yaw) for previous, current in zip(pairs, pairs[1:]))
    duration_sec = (pairs[-1][1].stamp_ns - pairs[0][1].stamp_ns) / 1.0e9
    yaw_bias = estimate_yaw - truth_yaw
    return {
        'linear': (
            estimate_path / truth_path
            if truth_path >= MINIMUM_LINEAR_SCALE_DENOMINATOR_M else None
        ),
        'yaw': (
            estimate_yaw / truth_yaw
            if abs(truth_yaw) >= MINIMUM_YAW_SCALE_DENOMINATOR_RAD else None
        ),
        'minimum_linear_denominator_m': MINIMUM_LINEAR_SCALE_DENOMINATOR_M,
        'minimum_yaw_denominator_rad': MINIMUM_YAW_SCALE_DENOMINATOR_RAD,
        'linear_denominator_valid': truth_path >= MINIMUM_LINEAR_SCALE_DENOMINATOR_M,
        'yaw_denominator_valid': abs(truth_yaw) >= MINIMUM_YAW_SCALE_DENOMINATOR_RAD,
        'estimate_path_length_m': estimate_path,
        'ground_truth_path_length_m': truth_path,
        'estimate_yaw_change_rad': estimate_yaw,
        'ground_truth_yaw_change_rad': truth_yaw,
        'yaw_change_bias_rad': yaw_bias,
        'yaw_change_bias_rad_per_sec': yaw_bias / duration_sec if duration_sec > 0.0 else None,
        'ground_truth_duration_sec': duration_sec,
    }


def _metric_summary(values):
    if not values:
        return {'count': 0, 'rmse': None, 'mean_abs': None, 'median_abs': None, 'p95_abs': None, 'max_abs': None}
    absolute = sorted(abs(value) for value in values)
    return {
        'count': len(values),
        'rmse': math.sqrt(sum(value * value for value in values) / len(values)),
        'mean_abs': sum(absolute) / len(absolute),
        'median_abs': _percentile(absolute, 0.5),
        'p95_abs': _percentile(absolute, 0.95),
        'max_abs': absolute[-1],
    }


def _percentile(sorted_values, fraction):
    if len(sorted_values) == 1:
        return sorted_values[0]
    position = fraction * (len(sorted_values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    weight = position - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


def _planar_covariance(values):
    if values is None:
        return None
    if len(values) >= 36:
        indices = ((0, 1, 5), (6, 7, 11), (30, 31, 35))
    elif len(values) >= 9:
        indices = ((0, 1, 2), (3, 4, 5), (6, 7, 8))
    else:
        return None
    matrix = tuple(tuple(float(values[index]) for index in row) for row in indices)
    return matrix if all(math.isfinite(value) for row in matrix for value in row) else None


def _symmetric(matrix, tolerance=1.0e-9):
    return all(abs(matrix[row][column] - matrix[column][row]) <= tolerance for row in range(3) for column in range(3))


def _positive_semidefinite(matrix, tolerance=1.0e-12):
    if any(matrix[index][index] < -tolerance for index in range(3)):
        return False
    for first, second in ((0, 1), (0, 2), (1, 2)):
        if matrix[first][first] * matrix[second][second] - matrix[first][second] * matrix[second][first] < -tolerance:
            return False
    return _determinant_3x3(matrix) >= -tolerance


def _determinant_3x3(matrix):
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def _inverse_3x3(matrix):
    determinant = _determinant_3x3(matrix)
    if abs(determinant) <= 1.0e-15:
        return None
    result = []
    for row in range(3):
        result_row = []
        for column in range(3):
            minor_rows = [index for index in range(3) if index != column]
            minor_columns = [index for index in range(3) if index != row]
            minor = matrix[minor_rows[0]][minor_columns[0]] * matrix[minor_rows[1]][minor_columns[1]] - matrix[minor_rows[0]][minor_columns[1]] * matrix[minor_rows[1]][minor_columns[0]]
            result_row.append(((-1.0) ** (row + column)) * minor / determinant)
        result.append(tuple(result_row))
    return tuple(result)


def _fraction_true(values):
    return sum(bool(value) for value in values) / len(values) if values else None


def _empty_covariance_consistency():
    return {
        'matched_covariance_count': 0,
        'two_sigma_coverage': {'x': None, 'y': None, 'yaw': None, 'all_axes': None, 'diagnostic_only': True},
    }
