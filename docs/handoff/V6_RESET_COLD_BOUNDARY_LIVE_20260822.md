# V6 cold-episode-boundary R5 live validation — attempts 1–7 (2026-08-22)

## Status

R5 was stopped by user decision after attempt 7 completed. The cold-episode-boundary
reset machinery (work package R1–R4, commits `6484e73`/`a639270`/`c95996b`) was
exercised live across 7 same-stack sessions. The reset/bootstrap gauntlet now passes;
the remaining failure is navigation-level (collision), which belongs to Phase 2, not
to the reset machinery.

## User decision recorded (2026-08-22)

- IMU regime investigation is **closed**. `yaw_scale=0.9294` stays frozen as the
  pure-spin calibration value. Schema-2 evidence (Attempt4, run dir
  `v6_imu_regime_session_a_attempt4_20260822T065252Z`) proved no single global IMU
  scale covers all motion regimes; the project stops searching for one.
- The estimated-substrate non-degradation promotion gate is lifted by user decision.
  The project returns to the estimated-odom + localization + navigation main line
  with 0.9294 in place.
- References: `docs/handoff/V6_IMU_REGIME_ATTEMPT4_20260822.md`, commit `e4ad26d`
  (c-class goal-binding contract fix), and the IMU regime ledger entries.

## Attempt arc (all sessions: Kujiale scene, estimated policy wheel_imu + 0.9294,
## RF2O off, one persistent Isaac+ROS stack, domain 173, manifest seed 7201)

| # | run dir (suffix) | stop_reason | distinct defect exposed | fix commit |
|---|---|---|---|---|
| 1 | 095144Z | `readiness_timeout:facts:constraints_seen` | `/map` binding attempted during HOLD window | `90219af`, `e49b74c` (defer + replay at barrier open) |
| 2 | 103433Z | same | same (fix in progress) | — |
| 3 | 105300Z | `second_initialpose` | enrollment reseed burst inside one reset generation rejected | `6757daa` |
| 4 | 110847Z | `amcl_not_strictly_newer_than_initialpose` | scan-stamped AMCL stragglers broke seed ordering | `67ee113` |
| 5 | 111740Z | `active_prior_not_trusted` | untrusted intermediate planning prior armed goal readiness | `d5ac812` |
| 6 | 112910Z | `post_reset_readiness_timeout` | B5 gate did not witness enrollment-seeded composition | `7e9417b` |
| 7 | 115942Z | **`collision`** | none in reset machinery — episode reached navigation and collided | — |

Each attempt produced new discriminating information (distinct failure class per
attempt); iteration was justified under the no-rabbit-hole rule and has now been
stopped by user decision after attempt 7.

## Attempt 7 terminal facts (run dir `v6_reset_cold_boundary_r5_20260822T115942Z`)

- Boundary invariants verified at the episode boundary:
  - ① receipt correct: `actual_seed=7201` == requested, `generation=2`,
    pose `long_route_start_g1`, odometry `realistic`;
  - ⑤ GT firewall: `/ground_truth/*` max 1 subscriber (evaluator/recorder class) — PASS;
  - ⑥ publisher ownership: `/amcl_pose`, `/cmd_vel`, `/cmd_vel_sim`, `/odom` each
    exactly 1 publisher — PASS;
  - exactly-once: `reset_calls=1`, `reset_events=1`; `goal_publications=1`.
- Episode outcome: `state=STOP`, `stop_reason=collision`, `completed_leg_ids=[]`,
  `route_goal_results=[]` — the robot collided before completing any mission leg.
- Evidence: `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_reset_cold_boundary_r5_20260822T115942Z/`
  (`episodes/episode_seed7201.jsonl`, `boundary_seed7201.json`, logs, rosbag, provenance).

## Code landed during R5 (Module3 `cognitive-navigation`)

`c1d4cde` session driver + boundary checker; `a6b2cae` per-episode timeout bound;
`90219af`/`e49b74c` HOLD-window map-binding defer+replay; `7f39ddf` flake8 parity;
`6757daa` enrollment reseed burst; `67ee113` AMCL straggler ordering; `d5ac812`
untrusted-prior gating; `7e9417b` B5 gate generation witness.

## Assessment

- Cold boundary + minimal invariants + Gate + Hot Reset implementation principle held
  up: no fence/forensic machinery had to be reintroduced; every live defect was a
  real (a)-class functional gap in the new bootstrap path, fixed minimally.
- The simplified reset path is **engineering-usable**: reset receipt, generation
  re-arm, epoch rollover, publisher ownership, and GT firewall all behaved correctly
  once the bootstrap-ordering gaps were closed.
- **Current blocker is navigation, not reset**: seed-7201 episode collides with zero
  completed legs (`route_goal_results` empty). This is the Phase 2 question —
  estimated-state navigation in the Kujiale low-obstacle scene — not a boundary defect.

## Next steps (when the goal resumes)

1. Phase 2 / T2.1-style investigation of the seed-7201 collision: pull the attempt-7
   bag, localize where the executed trajectory hits (costmap/local planner/localization
   jump), using visual evidence (map+trajectory overlay, costmap, scan overlay).
   Note: obstacle layout/position/size may be adjusted freely per user direction —
   the goal is a fast closed loop under estimated localization, not ideal-localization
   era layout assumptions.
2. Then resume the R2 live queue: moving cognitive chain → PRIMARY retry → B5 live →
   causal evidence → pilot/freeze → 120-trial campaign.

## Warnings / tech debt

- Multi-episode consecutive-run validation (2–3 episodes in one stack) was **not**
  reached: every session stopped inside episode 7201. Hot re-arm for episode N+1
  remains unproven live.
- The session driver STOPs the whole session on first episode failure by design;
  a per-episode continue-on-fail mode would suit future multi-episode validation.
- R5 agent was stopped before writing its own handoff; this document was written by
  master from the NAS evidence and commit history.
