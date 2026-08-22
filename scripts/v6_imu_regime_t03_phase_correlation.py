#!/usr/bin/env python3
"""T0.3 offline phase-trace correlation for the motion-assist sampling hypothesis.

Reads one Attempt4-style ``imu_regime_phase.jsonl`` and checks whether the
per-regime k* elevation is consistent with the loop order
``app.update -> motion_assist.update -> ground_truth.update``:

- per generation: raw-IMU integral ``A`` vs ground-truth yaw change ``G``,
  the implied ratio, and the post-sampling assist velocity injection
  ``S = sum(post_assist_w - pre_assist_w) * dt``;
- per loop: which body rate the IMU sample actually tracks
  (``pre_app`` vs ``pre_assist`` vs previous-loop ``post_assist``).

Diagnostic only; never controls a robot.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path


def _corr(xs, ys):
    if len(xs) < 3:
        return None
    mx = statistics.fmean(xs)
    my = statistics.fmean(ys)
    cov = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    vx = sum((x - mx) ** 2 for x in xs)
    vy = sum((y - my) ** 2 for y in ys)
    if vx <= 0.0 or vy <= 0.0:
        return None
    return cov / math.sqrt(vx * vy)


def _slope(xs, ys):
    if len(xs) < 3:
        return None
    mx = statistics.fmean(xs)
    my = statistics.fmean(ys)
    vx = sum((x - mx) ** 2 for x in xs)
    if vx <= 0.0:
        return None
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / vx


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase-jsonl", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    generations: dict[int, list[dict]] = {}
    with args.phase_jsonl.expanduser().open(encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            if row.get("kind") != "loop":
                continue
            generations.setdefault(int(row["reset_generation"]), []).append(row)

    report = {"phase_jsonl": str(args.phase_jsonl), "generations": {}}
    pooled = {"imu": [], "pre_app": [], "pre_assist": [], "prev_post_assist": []}
    for generation in sorted(generations):
        loops = generations[generation]
        a_imu = 0.0
        g_gt = 0.0
        s_inject = 0.0
        d_phys = 0.0
        pre_app_integral = 0.0
        applied = 0
        target_w_abs = []
        imu_series = []
        pre_app_series = []
        pre_assist_series = []
        prev_post_series = []
        prev_loop = None
        prev_gt = None
        gt_gap_steps = 0
        for row in loops:
            t = row.get("simulation_time_after_app_s")
            imu_graph = row.get("imu_graph_after_app") or {}
            read_imu = (imu_graph.get("read_imu_ang_vel") or {}).get("value")
            pre_app = row.get("pre_app_body") or {}
            pre_assist = row.get("pre_assist_body") or {}
            post_assist = row.get("post_assist_body") or {}
            body_ok = (
                t is not None
                and isinstance(read_imu, list)
                and pre_app.get("yaw_rate_radps") is not None
                and pre_assist.get("yaw_rate_radps") is not None
                and post_assist.get("yaw_rate_radps") is not None
            )
            if body_ok:
                t = float(t)
                imu_w = float(read_imu[2])
                pre_app_w = float(pre_app["yaw_rate_radps"])
                pre_assist_w = float(pre_assist["yaw_rate_radps"])
                post_assist_w = float(post_assist["yaw_rate_radps"])
                if prev_loop is not None:
                    dt = t - prev_loop["t"]
                    if dt > 0.0:
                        a_imu += imu_w * dt
                        s_inject += (post_assist_w - pre_assist_w) * dt
                        d_phys += (pre_assist_w - pre_app_w) * dt
                        pre_app_integral += pre_app_w * dt
                        imu_series.append(imu_w)
                        pre_app_series.append(pre_app_w)
                        pre_assist_series.append(pre_assist_w)
                        prev_post_series.append(prev_loop["post_assist_w"])
                prev_loop = {"t": t, "post_assist_w": post_assist_w}
            receipt = row.get("ground_truth_receipt")
            if isinstance(receipt, dict) and receipt.get("yaw_rad") is not None:
                gt_yaw = float(receipt["yaw_rad"])
                gt_t = float(receipt.get("simulation_time_s") or 0.0)
                if prev_gt is not None and gt_t > prev_gt["t"]:
                    delta = gt_yaw - prev_gt["yaw"]
                    # GT yaw is atan2-wrapped; unwrap the per-sample delta.
                    g_gt += (delta + math.pi) % (2.0 * math.pi) - math.pi
                elif prev_gt is not None:
                    gt_gap_steps += 1
                prev_gt = {"t": gt_t, "yaw": gt_yaw}
            assist = row.get("assist") or {}
            if assist.get("applied") is True:
                applied += 1
                target = assist.get("target")
                if isinstance(target, list) and len(target) == 2:
                    target_w_abs.append(abs(float(target[1])))
        pooled["imu"].extend(imu_series)
        pooled["pre_app"].extend(pre_app_series)
        pooled["pre_assist"].extend(pre_assist_series)
        pooled["prev_post_assist"].extend(prev_post_series)
        entry = {
            "loop_count": len(loops),
            "assist_applied_fraction": applied / len(loops) if loops else None,
            "assist_target_abs_yaw_rate_mean": (
                statistics.fmean(target_w_abs) if target_w_abs else None
            ),
            "raw_imu_integral_rad": a_imu,
            "ground_truth_delta_rad": g_gt,
            "ground_truth_nonmonotonic_steps": gt_gap_steps,
            "implied_gt_over_imu": (g_gt / a_imu) if abs(a_imu) > 1.0e-9 else None,
            "residual_gt_minus_imu_rad": g_gt - a_imu,
            "assist_post_sampling_injection_rad": s_inject,
            "physics_in_step_delta_rad": d_phys,
            "pre_app_integral_rad": pre_app_integral,
            "corr_imu_vs_pre_app": _corr(imu_series, pre_app_series),
            "corr_imu_vs_pre_assist": _corr(imu_series, pre_assist_series),
            "corr_imu_vs_prev_post_assist": _corr(imu_series, prev_post_series),
            "slope_imu_vs_pre_app": _slope(pre_app_series, imu_series),
            "slope_imu_vs_pre_assist": _slope(pre_assist_series, imu_series),
        }
        report["generations"][str(generation)] = entry

    report["pooled"] = {
        "corr_imu_vs_pre_app": _corr(pooled["imu"], pooled["pre_app"]),
        "corr_imu_vs_pre_assist": _corr(pooled["imu"], pooled["pre_assist"]),
        "corr_imu_vs_prev_post_assist": _corr(pooled["imu"], pooled["prev_post_assist"]),
        "slope_imu_vs_pre_app": _slope(pooled["pre_app"], pooled["imu"]),
        "slope_imu_vs_pre_assist": _slope(pooled["pre_assist"], pooled["imu"]),
        "loop_count": len(pooled["imu"]),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "generations": sorted(report["generations"], key=int)}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
