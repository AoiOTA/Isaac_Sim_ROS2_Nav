# V6 IMU regime Attempt4 — first live schema-2 capture — 2026-08-22

## Scope and provenance

- Branch/worktree: `cognitive-navigation` in the permitted Module3 worktree.
- Baselines rechecked before starting and matched the project boundary:
  Integration `f23a7eccc542e602ec641daf7a20b14c2371dca9`, Module3
  `22d66470c4b903349b2467dc876490bbebfc0083`, Module2
  `c8297a590ba61bcf712ad4a339437fb2c44a027e`.
- Runtime snapshot: `git archive` of the cognitive-navigation HEAD into
  `/tmp/v6_imu_session_a_attempt4.IPZuzv` (m3_src / i_src / m2_src), asset
  import + check, isolated colcon builds (`build/install/log_attempt4`),
  Integration 2 packages + Module3 14 packages all PASS. All live processes
  ran from this snapshot; ROS_DOMAIN_ID 171; RMW rmw_fastrtps_cpp;
  use_rviz false; capture scale frozen at `yaw_scale=0.9294`, RF2O off, M0.
- Final session HEAD at capture time: `5f0e0880398ed4c730697148d8ba58a679d42210`
  (flat20) and `2958fb4fe4d3fafdb41e7adf6222d394f879bd26` (goal; driver-only
  delta on top).

## Code changes made for the capture (all committed)

1. `scripts/v6_imu_regime_attempt4_session.sh` + `..._monitor.py` +
   `..._goal_metadata.py` + `..._contract_audit.py`: the locked session
   driver (flat20 and Kujiale goal subcommands), a passive safety monitor, a
   strict goal-metadata extractor, and an offline contract audit tool.
2. `motion_benchmark.py` HOLD zeros: the post-reset dispatch-barrier wait now
   publishes zero intent at the command rate. Without it the HOLD window
   `[reset_event, first_schedule_start)` had zero `/cmd_vel_nav` samples and
   the schema-2 HOLD contract could never pass (first capture, run 051750Z).
3. `motion_benchmark.py` fresh clock tick before each schedule start: a HOLD
   zero could otherwise share the exact `start_sim_s` stamp and break the
   exact `intent_publish_count` binding (observed live: 231 vs 230).
4. `motion_benchmark.py` settle drain (`SETTLE_DRAIN_SEC=0.5`): zero intent is
   published for 0.5 s before the receipted 0.8 s settle window opens, so the
   smoother/CollisionMonitor/gate deceleration tail (~0.15 s) no longer lands
   inside the receipt window (`command_zero_leak` in run 063016Z).
5. `motion_benchmark.py` stationary duration epsilon:
   `measured + 0.05 + 1e-9 < requested`; the live measured span was
   `10.0 - 0.05 - 5e-15`, which the 0.05 s tolerance is meant to accept but
   float rounding failed (run 055725Z aborted on this coin-flip).
6. Diagnostic-session-only overlay: the driver's standalone collision monitor
   is launched with `-p stop_pub_timeout:=30.0` (default 1.0 s stops
   republishing zeros during sustained zero input, which starved stationary
   and settle zero coverage). Production navigation configs were not changed.
7. Focused tests added/updated: HOLD-zero publication, stationary duration
   boundary, fresh-tick schedule start. `test_motion_benchmark.py` +
   `test_imu_regime_analysis.py`: **126 passed**; full snapshot
   `robot_experiments` suite: **567 passed, 1 pre-existing path-sensitive
   `test_rivermark_reference` failure** (known, unrelated).

No reset-gate, route, planner, critic, IMU calibration, or Module2/Integration
product logic was changed. `robot_odometry` was not touched.

## Capture evidence (NAS)

- flat20 (schema-2, contract-PASS):
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_attempt4_20260822T065252Z`
  - `rosbag/flat20_motion` (MCAP), `imu_regime_phase.jsonl` (13714 loops),
    `analysis/motion_report.json` (schema 2), `analysis/safety_monitor_summary.json`,
    `analysis/imu_regime_analysis_flat20_only.json`,
    `analysis/imu_regime_analysis_full_contract.json`, `summary.json`,
    `provenance/` (preflight, authority, lifecycle, bag info, stages).
- Kujiale Estimated goal (route completed, binding fences failed):
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_imu_regime_session_a_attempt4_goal_20260822T071314Z`
  - `rosbag/kujiale_goal` (MCAP incl. `/rosout`), `analysis/goal_metadata.json`,
    `probe/closed_loop.json/.png`, `evaluator/`, `summary.json`.
- Intermediate diagnostic runs (kept, each documented in the flat20
  `summary.json`): `..._20260822T051750Z`, `..._055725Z`, `..._063016Z`,
  goal `..._070050Z`, `..._070735Z`.

## Results

### flat20 session (seeds 8609 + 8610..8618, 10 consecutive generations)

- LiDAR preflight PASS; command authority PASS; safety monitor PASS (0
  collision, 0 sim-time backward, 10 reset events); bag finalized; all owned
  processes cleaned; domain 171 empty afterwards.
- **capture_contract_status = PASS** — first live schema-2 capture: HOLD
  coverage, exact schedule publish-count binding, 0.8 s downstream zero
  receipts, and final-stop checks all pass on all four command stages for all
  ten epochs. All 12 analysis windows OK; phase trace OK.
- **performance_status = FAIL**: `cw_360`, `ccw_360`, `arc_v005_cw`,
  `arc_v025_ccw` pass; `arc_v005_ccw` (translation fraction, velocity
  overshoot), `arc_v010_cw/ccw` (angular + curvature tracking), `arc_v025_cw`
  (curvature), `s_route` (curvature + turn-reversal latency) fail. This is
  consistent with Attempt3: the diagnostic chain under-tracks the arc/S
  primitives against the frozen thresholds.
- Segment k* (flat20-only official analysis): spins 0.929171 (CW) / 0.930892
  (CCW); v=0.05 arcs 0.923058 / 0.949645; v=0.10 arcs **0.983599 / 0.974894**;
  v=0.25 arcs 0.922902 / 0.918223; S 0.913967 / 0.920863 / 0.913894.
- 12-window <=5 deg intersection: **[0.9207, 0.9420]** — contains frozen
  0.9294. The v=0.10 bins pulling toward 0.97-0.98 while v=0.25 and S sit at
  0.91-0.92 is the regime signature inside the flat20 primitives themselves.

### Kujiale goal session (seed 8619, G1→G2)

- Route completed: 44.92 s, final goal error 0.202 m, no physical collision,
  no footprint collision; reset receipt seed **8619 == requested** (the
  Evidence-B requested/actual seed debt is fixed at this HEAD).
- The goal MCAP nevertheless fails two single-attempt binding fences, both
  deterministic behaviors of the production probe/coordinator chain:
  1. `goal_request_count` — `probe_closed_loop` republishes the identical
     goal at 1 Hz until the route ack lands (here 4 recorded
     `/bio_nav/route_goal` messages; the contract allows exactly 1 when the
     topic is recorded). Not recording the topic is a legal contract path
     (`reset_terminal_single_command_attempt`), but see 2.
  2. `goal_command_after_terminal` — one nonzero `/cmd_vel` 45 ms after the
     terminal (smoother decel tail outlives the NavToPose-result-driven
     terminal). The historical Evidence B bag shows the same +58 ms tail, so
     this fence has never been satisfiable live.
- Full-contract analyzer output (flat20 + goal): verdict **FAIL** at
  `goal_request_count` (recorded in the flat20 run's
  `analysis/imu_regime_analysis_full_contract.json`).

## Verdict

**FAIL / NOT FORMAL — with the primary unblock delivered.**

- The schema-2 capture pipeline is now live-capable: capture_contract_status
  PASS with complete 12-window evidence and valid phase trace.
- `PASS_CANDIDATE` / `CONFIRMED_NO_GLOBAL_CONSTANT` remain unreachable this
  round because (a) benchmark performance fails on 5/9 primitives and (b) the
  goal single-attempt binding fences conflict with production probe/coordinator
  behavior. Neither is promotable per the evidence contract.
- The segment evidence says the flat20 regimes still admit a global constant:
  intersection [0.9207, 0.9420] contains 0.9294; the goal-side non-degradation
  interval is unmeasured this round (goal binding failed), so the
  regime-dependence question stays formally open.

## Decisions needed from master (not made here, out of scope)

1. Goal binding: either amend the contract (accept identical 1 Hz dispatch
   republishes as one request; allow a bounded post-terminal decel drain) or
   change `probe_closed_loop`/coordinator terminal timing (robot_route_planner
   is the parallel agent's scope).
2. Diagnostic-chain tracking performance on arc/S primitives (5/9 fail): tune
   the chain or the thresholds, or accept `performance_status=FAIL` and read
   regime evidence from `capture_contract_status=PASS` captures only.

## Suggested next action

- master's call on the two decisions above; once the goal binding is resolved,
  rerun only the goal session (driver `goal` subcommand) and re-run
  `imu_regime_analysis` with `--goal-mcap/--goal-evaluator` for the full
  verdict. Frozen `yaw_scale=0.9294` and RF2O-off remain unchanged throughout.
