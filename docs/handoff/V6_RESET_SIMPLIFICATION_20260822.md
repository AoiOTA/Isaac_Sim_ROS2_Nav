# V6 reset simplification — cold episode boundary + minimal generation isolation

Date: 2026-08-22

## Scope and verdict

- Worktree: permitted Module3 worktree
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`.
- Branch: `cognitive-navigation`; code commits `6484e73` (R3 coordinator
  simplification) and `a639270` (R1/R2/R4 runner contract, probe retirement);
  started at `96549133f45e8f0abd80f48f1e5bdee012b81a5c`.
- Fixed main ancestry: `22d66470c4b903349b2467dc876490bbebfc0083`.
- Work package: R (reset simplification), code part R1–R4.  Model: **Cold
  Episode Boundary + minimal generation isolation + Gate + Hot Reset
  (Option A: the stack is not restarted across episodes; each episode re-arms
  in place after a cold boundary reset)**.
- Verdict: **PASS (code/build/unit only)**.  No ROS graph, Isaac, Nav2,
  navigation, or live reset was run; live verification is R5.

## Protocol definition (R1): cold episode boundary

Reset may only happen when the stack is provably idle and still; a violation
fails the episode stop (STOP) with an explicit reason before any reset call:

1. **No active goal/route.**  The runner arms reset before its first goal
   (`goal_publications == 0`, enforced in `EpisodeGuard.arm_reset`, violation
   `reset_with_active_goal_forbidden`).  Coordinator-side quiescence is
   observed through a negative window: any pre-reset message on
   `/bio_nav/route_progress`, `/bio_nav/route_goal_complete`, or
   `/bio_nav/route_goal_result` blocks readiness forever
   (`pre_reset_negative_window:route`).  An active route from a previous
   episode is a protocol violation and stops the episode instead of being
   reset underneath.
2. **Confirmed stillness.**  During the same 1.0 s quiet window
   (`PRE_RESET_NEGATIVE_WINDOW_S`), every observed `/cmd_vel` and
   `/cmd_vel_sim` sample must be zero (`|linear.x|, |linear.y|, |angular.z|
   ≤ 1e-3`) and the `/odom` XY span must be ≤ 0.10 m with at least one odom
   sample in-window.  Blocker label `pre_reset_not_still`.
3. **Quiet localization channels.**  Zero pre-reset planning priors,
   localization candidates, and `/initialpose` messages (unchanged
   semantics), plus the `route` channel above.  AMCL was removed from the
   negative window: in a warm stack AMCL republishes continuously, and in a
   cold stack `candidate == 0 ∧ prior == 0 ∧ initialpose == 0 ∧ B5 waiting`
   already implies AMCL silence.  Post-reset AMCL is still guarded by the
   stamp-ordering check (`amcl_not_strictly_newer_than_initialpose`).
4. **Readiness still requires** the full endpoint roster, the bridge
   epoch/session baseline, the B5 reset-ready fact, and sole-publisher
   ownership (invariant ⑥).  All blockers are reported in one
   `readiness_timeout:<blocker list>` stop reason.

## The six minimal invariants (R2), now owned by the v6_formal runner

Missing any of these makes an episode untrustworthy; all are PASS/FAIL
(STOP) level.  Forensic items from the retired probe (timing budgets, GID
topology, cross-writer classification) are not PASS/FAIL; at most they remain
record-only evidence rows.

1. **Receipt match.**  `parse_reset_receipt` fails closed on seed/case/
   variant mismatch (unchanged) and now also on pose mismatch
   (`requested_pose`, back-compatible optional parameter).  The receipt
   `generation` must be an int ≥ 1; `odometry` mode is recorded provenance
   (the sim rejects a wrong running mode itself).
2. **Odometry landing and span.**  After the B5 bootstrap completes and
   before the first goal, post-reset `/odom` samples (the first, possibly
   boundary-straddling, sample is skipped) must land within 0.10 m of the
   re-zeroed odom origin and keep an XY span ≤ 0.10 m.  Stops:
   `post_reset_odom_missing`, `post_reset_odom_landing:<m>`,
   `post_reset_odom_span:<m>`.  The Ground Truth half of this invariant
   stays with the evaluator/recorder (`estimated_state_evaluator` records
   `/ground_truth/odom`; the dispatcher never touches GT).
3. **No stale drive replay.**  From the reset call until the runner's first
   goal, any nonzero `/cmd_vel_sim` or `/cmd_vel` sample stops the episode:
   `post_reset_command_nonzero:<topic>`.
4. **No stale route.**  With no active route there must be zero route
   terminals; after the reset boundary only the runner's own new goal may
   resume motion.  Any route progress/complete/result message after the
   reset call and before the first goal stops the episode:
   `stale_<kind>_after_reset`.  (A coordinator abort terminal for a previous
   episode's route therefore fails the episode — the cold boundary was
   violated upstream.)
5. **GT firewall.**  Import-time check on `DISPATCH_SUBSCRIPTION_TOPICS`
   (unchanged) plus a runtime assertion over the live subscription list at
   `run()` start (`_assert_ground_truth_firewall`).  GT is subscribed only
   by the evaluator/recorder.
6. **Sole-publisher ownership.**  At readiness, exactly one publisher each
   on `/odom`, `/cmd_vel` (CollisionMonitor), `/cmd_vel_sim` (ResetStopGate),
   and `/amcl_pose` (the AMCL proxy for map→odom authority; per-transform TF
   ownership is not graph-observable and remains enforced bridge-side by the
   `tf_ownership` contract).  Blocker label
   `publisher_ownership:<topic>=<count>`.

## Coordinator (robot_route_planner) simplification (R3)

Retained semantics, unchanged behavior:

- **Input generation filtering**: external inputs and async outputs capture
  `RouteInputGeneration(reset_generation, request_id, graph_generation,
  graph_id, graph_revision)` at admission and are discarded when it no longer
  matches.  Because `_begin_simulation_reset_locked` advances
  `reset_generation`, `request_id`, and `graph_generation` at HOLD, stale work
  can never match after a reset; the per-generation fence fields
  (`reset_intent_generation`/`reset_event_completed_generation`) inside the
  input identity were redundant and were removed.
- **Exactly-once retire (A1)**: a route interrupted by reset gets exactly one
  abort terminal pair (`route_goal_complete=false` +
  `route_goal_result{status:aborted, reason:simulation_reset}`) and one
  navigation-handle cancellation; a reset with no active route publishes no
  fake terminal; the epoch advances exactly once per reset.
- **ResetStopGate status parser** and the fail-closed posture (any malformed,
  backward, or out-of-sequence status keeps `reset_hold_barrier` held and
  logs an error).
- **Completion-owned GVG reassert authorization during HOLD**, reduced to a
  truthy token + `graph_reassert_required` + GVG identity (stale dispatches
  remain fenced by the route-input identity).

Removed forensic surface:

- The pristine startup binding machine (`startup_reset_event_pending`,
  deadline, legacy/invalidated flags, bind/can-bind/safety-check methods and
  the 0.1 s `_startup_reset_event_tick` timer; ~190 lines).  Under the cold
  boundary the coordinator has route state only after a release, so strict
  status authority is always established before route-era state exists; the
  volatile-Empty vs transient-local-status races the machine arbitrated
  resolve safely in the minimal machine (see below).
- The 12-condition READY re-verification (`_publish_reset_gvg_ready_if_
  reconciled_under_output_lock`, `reset_gvg_ready_generation`,
  `reset_release_seen_generation`; ~48 lines) and the retain/re-arm of reset
  retry contexts across commits.  Replaced by one flag:
  `reset_ready_pending` is set when the completion-owned reassert commits
  while HOLD still fences outputs, and the release of that generation
  publishes the single `READY "reset GVG reconciled"` — only while the graph
  is still coherent; otherwise the eventual commit announces READY itself.
- Probe-coupled timing/ordering contracts (0.5 s bind grace, linearization
  proofs).

The minimal machine (~175 lines for the status handler, ~60 for the event
handler, docstrings included):

- New gate generation must begin with `hold` → retire + epoch bump +
  `intent=G`, barrier held.  `reset_complete` (Empty event or strict status,
  whichever first) runs completion work once (`completed=G`, empty runtime
  snapshot + GVG reassert).  `released:*` opens the barrier only when
  `completed == G`.  Duplicates are idempotent; anything else holds + error.
- First-seen statuses (late join): `released:*` is the open startup baseline;
  `initialized`/`closed` establish authority while held; first-seen
  `hold`/`reset_complete` synchronize the generation and retire once (A1 —
  an active route gets its single abort terminal; previously this path only
  held without retiring).
- `_on_reset_event` (Empty, no generation): with a pending intent it only
  completes it; a duplicate after completion is dropped.  With no gate
  authority ever seen (gate-less deployments) it retains the legacy path:
  retire + complete + open immediately.  With authority but no pending
  intent (a lost HOLD) it retires once and stays held — fail closed — until
  the status stream re-synchronizes.
- Graph switch retries without reset-completion authority are no longer
  *consumed* during HOLD: the reconciliation tick leaves them due and they
  dispatch after the release (closes a liveness gap the simplification would
  otherwise have introduced: a late stale success during HOLD could strand
  the desired graph incoherent).

## Multi-episode re-arm (R4, Option A)

- `EpisodeGuard.arm_reset` no longer requires bridge epoch 0.  The runner
  discovers the current bridge epoch/session as its baseline and requires
  the post-reset epochs to roll as baseline+1 (`physical`, reset_event) and
  baseline+2 (`bootstrap`, initialpose), with new sessions at each step —
  the pre-existing `record_bridge` logic needed no change.
- B5 readiness (`b5_reset_ready`) accepts the cold waiting values
  (`not_received`/`waiting_after_physical_reset` …) or, for warm stacks, a
  seeded candidate generation string whose `epoch`/`session` equals the
  discovered baseline.
- Per-generation isolation semantics are unchanged: one runner process still
  drives exactly one reset and one episode (`reset_retry_forbidden`,
  `second_reset_event`, `second_initialpose`, generation-bound planning
  prior), and B5/bridge-side `reset_generation()` per-generation clearing is
  untouched.

## Deleted / kept summary

- Deleted: `active_reset_probe.py` (2364 lines), `test_active_reset_probe.py`
  (1369 lines), the `active_reset_probe` console entry, the pristine binding
  machine, the 12-condition READY recheck, the release/GVG-ready ledgers,
  the startup tick timer, and the reset-fence fields of
  `RouteInputGeneration`.
- Kept: ResetStopGate + reset_service + InitialPoseRepublisher (untouched),
  `reset_receipt.py` (+pose check), generation re-arm and input generation
  filtering, A1 exactly-once abort terminal, epoch exactly-once advance,
  fail-closed posture everywhere.
- Superseded: `V6_ACTIVE_RESET_PROBE_20260822.md` (the probe is retired;
  its PASS/FAIL-worthy checks live in the runner per the six invariants).

## Metrics

- Probe retirement: **-3733 lines** (2364 product + 1369 tests) + 1 entry.
- `ros_node.py`: 4969 → 4634 lines.  Reset surface per block (old → new):
  gate status handler 230 → 174; reset event handler 80 → 59; completion +
  READY recheck 86 → 38; pristine binding machine 190 → 0; startup tick
  35 → 0; fence dataclass fields and init ledgers removed.  Commit stats:
  `6484e73` 3 files +259/-887; `a639270` 7 files +365/-3774.
- Test counts: robot_route_planner 190 → 173 collected (14 startup-machine
  or probe-coupled tests deleted, 7 rewritten to retained semantics, 1
  added); robot_experiments 676 → 568 collected (the probe's 108 focused
  tests removed, 13 runner-invariant/baseline tests added).

## Validation (code/build/unit only)

- Source-first at `5f0e088`: robot_route_planner **172 passed, 1 skipped**
  (optional `pxr`); robot_bringup **228 passed**; robot_experiments
  **567 passed, 1 pre-existing failure**
  (`test_rivermark_reference_sums_all_five_route_legs_and_converges` compares
  a frozen JSON with absolute paths baked in another worktree; fails
  identically without these changes).
- Fresh-installed from the isolated base: identical results.
- Isolated colcon build PASS (`build_reset_simpl_QYUBBF`,
  `install_reset_simpl_QYUBBF`, `log_reset_simpl_QYUBBF`; robot_bringup built
  against the worktree underlay).  Installed `v6_formal_episode` entry point
  present; no `active_reset_probe` entry remains.
- Changed-file flake8 (`--max-line-length=99 --extend-ignore=A003,Q000`):
  no new findings vs the pre-change baseline (ros_node 51→46, v6_formal
  56→55, reset_receipt 2→2).  `py_compile` PASS; `git diff --check` PASS.

## R5 readiness and residual risks

- Warm-stack readiness (R4) is unit-tested only.  R5 must drive two episodes
  in one stack and watch: (a) residual pre-reset prior/candidate traffic
  from the previous generation would block readiness forever (cumulative
  negative window) — if observed, scope the window by the generation tags
  both messages carry; (b) the B5 baseline-generation string match
  (`epoch=N,session=...`) must equal the bridge diagnostic pair;
  (c) post-reset odom landing threshold 0.10 m assumes the re-zeroed odom
  origin behavior the probe measured (`reset_odom_landing_xy=(0,0)`).
- A stale AMCL sample landing between the bootstrap initialpose and the
  first post-reset AMCL sample could satisfy the stamp-ordering check early;
  B5 `seed_confirmation` independently gates the seed, so the impact is a
  slightly earlier `goal_ready`, not a false one.
- READY for the reconciled GVG may now be published at reassert commit
  time when the release already happened (previously always at release);
  consumers keying on the detail string see
  `cognitive graph fallback applied: ...` instead of `reset GVG reconciled`.
- An Empty reset event arriving before its HOLD at an established
  coordinator (cross-topic race) is dropped as a duplicate of the previous
  completed intent; the strict status stream drives the cycle to completion
  regardless.  With no pending intent it logs an error and holds — both are
  safe, but R5 should confirm the error never appears in practice (HOLD
  precedes the Empty by seconds in `reset_service`).
