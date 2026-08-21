# V6 IMU yaw regime-dependence amendment

Date: 2026-08-21

## Decision

- Branch/worktree/start: `cognitive-navigation`; permitted Module3 worktree;
  `f63fd23ab9dfe9b19d9d5f2e7f44c7d8eff90ca0`.
- Verdict: **ENGINEERING FAIL for the Kujiale mixed-motion route; the flat20
  pure-rotation `yaw_scale=0.9294` baseline remains valid**.
- One global constant cannot satisfy both available regimes. The proposed
  global `0.9814` change is rejected and was not committed. The calibrated
  profile remains `0.9294`; the explicit identity profile remains `1.0`.
- A subscriber-side piecewise scale model is also rejected at this stage.
  Candidate values are offline diagnostics only until the phase mechanism is
  reproduced and measured.
- No motion-assist, IMU implementation, EKF, TF, routing, control, or safety
  model was changed by this amendment.

## Evidence A: flat20 pure rotations and S-route

Authoritative read-only results:
`/tmp/v6_imu_calibration_live.VAf50R/results`.

Three valid CW and three valid CCW pure rotations established the historical
rotation result for `0.9294`:

- raw IMU/GT yaw scale: `1.075078--1.077350`;
- corrected IMU/GT yaw scale: `0.999178--1.001289`;
- EKF/GT yaw scale: `0.998662--1.001724`;
- maximum corrected-IMU closure: `0.470 deg`;
- maximum EKF closure: `0.629 deg`.

All six rotation rows passed. The same bounded run also produced two S-route
reference passes out of three (`r01` and `r03` passed; `r02` failed). These are
engineering, not formal-qualification, results.

## Evidence B: Kujiale mixed-motion route

Authoritative NAS evidence:
`/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_estimated_dynamic_smoke_20260821T150407Z`.
The exact runtime snapshot was Integration
`2dd3aa937ae470d497cd97722302281efcc2e3f0`, Module3
`3dc2830c1da5b5f441191217220bc120058bd4b2`, and Module2
`2925f806c88b1551d1c48ca89d1c1c5adf2ba748`.

Navigation succeeded in `51.93 s`, with final goal error `0.195 m`, no physical
collision, and no actual-trajectory footprint collision. Raw and corrected IMU
samples were exactly stamp-paired and the measured nonzero ratio was `0.9294`.
Nevertheless, evaluator-only GT comparison failed non-degradation:

- full-window aligned yaw RMSE: raw `0.08527 rad`, corrected `0.12342 rad`;
- full-window endpoint absolute yaw error: raw `0.08447 rad`, corrected
  `0.21168 rad`;
- goal-window endpoint absolute yaw error: raw `0.13643 rad`, corrected
  `0.18516 rad`.

The overall result is therefore **ENGINEERING FAIL**, despite successful
navigation. Formal qualification was not run. Offline zero-bias fits identify
`0.981373` for the full endpoint and `0.970049` for the goal endpoint; their
rounded forms `0.9814` and `0.9700` are not live results or accepted settings.

Two separate runtime debts must remain separate from the scale conclusion:
the requested dynamic seed was `8601` but the service reset receipt reported
seed `0`, and strict `/cmd_vel` unique-publisher evidence failed because the
collision monitor and Isaac reset-zero publisher coexisted.

## Exact constant tradeoff

Applying the `<=5 deg` closure bound to the six valid rotation episodes permits
only `k=[0.917435, 0.940927]`. Requiring the mixed-route full-window result to
be no worse than identity permits `k=[0.962746, 1.0]`. The intervals do not
intersect. Consequently neither `0.9294`, `0.9814`, nor any other single global
multiplier can satisfy both evidence sets under the current measurement model.

## Leading phase-mechanism hypothesis

The old and new Module3 motion-assist, main-loop, and metrics blobs are
identical, so source drift in those blobs is not the leading explanation. The
current ordering calls the sensor `app.update()` before motion assist, while GT
is sampled after assist; assist itself depends on linear speed and turn
direction. That phase/order difference could make raw-IMU-to-GT scale depend on
motion regime. This is the leading mechanism hypothesis, not a proven cause.

## Next diagnostic sequence

Use the committed `0.9294` baseline and keep all alternative scales offline:

1. stationary capture;
2. one CW pure rotation, then one CCW pure rotation;
3. low-speed arcs at `0.05` and `0.10`, both directions;
4. normal-speed arcs at `0.25`, both directions;
5. S-route;
6. Kujiale mixed route.

At each step compare raw IMU, corrected IMU, EKF, and evaluator-only GT with
explicit phase/timestamp accounting. If the regime dependence reproduces, run
motion assist on/off A/B before proposing any runtime model. Fix or explicitly
control the reset-seed and `/cmd_vel` ownership debts so they cannot confound
the comparison.

Final freeze requires a fresh three-CW plus three-CCW set, an S-route set, and
a Kujiale mixed-route run to support the same model. Do not promote an offline
candidate or requalify affected manifests before that combined evidence exists.
