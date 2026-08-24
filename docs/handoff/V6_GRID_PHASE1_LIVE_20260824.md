# V6-GRID Phase 1 live handoff (2026-08-24)

## Verdict

**FAIL at canonical Isaac startup.** Phase 1B did not run, so Phase 1C was not
authorized. No reset or route goal was sent. This is a real live startup
failure, not a code-test pass, engineering navigation success, or formal
qualification.

## Run

- Module3: `42a222bb088b3184d2a99399979bd1a6e3678db7`
- Integration: `2578366c350fee741ca2e97cd846d5741b48eb68`
- Module2 metadata only: `c18bd9ea7c69b4cc44e4226a7e37d6e1b803de30`
- Snapshot: `/tmp/v6_phase1_combined_repreflight.sOYIbk`
- ROS domain: `209`, empty before launch and after cleanup
- Requested canonical episode: index `0`, seed `7201` only
- NAS evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_20260823T163229Z/`
- Unrelated process preserved: `odom_static` on ROS domain `141`

## First error layer

The session started its owned Isaac process group, which exited before Isaac
Sim Kit with:

```text
[isaac-nav] error: required file not found: /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_intergration/ros2_ws/install/bio_nav_interfaces/include/bio_nav_interfaces/bio_nav_interfaces/msg/detail/cognitive_obstacle_array__struct.hpp
```

The session declares an immutable snapshot and sources `/opt/ros/jazzy`, then
snapshot Integration, then snapshot Module3
(`scripts/v6_reset_cold_boundary_r5_session.sh:14-16,196-216`). It then starts
the snapshot Isaac entrypoint (`:293-300`). However, the canonical wrapper
enables the V6 Integration requirement
(`scripts/run_v6_kujiale_low_obstacles.sh:14-16`), while
`scripts/lib/common.sh:11,69-96` defaults the Integration setup and allowed
root to the live Integration worktree and checks required headers there.

The required header exists in snapshot `i_src/ros2_ws/install_r5`, and the
prescribed source order resolves `bio_nav_interfaces` from that snapshot. The
live-worktree header is absent. Building or sourcing that live install would
violate this run's immutable-snapshot boundary, so the reviewer did not work
around the failure.

## Evidence and boundary

- Decisive log: `logs/isaac.log`
- Run contract: `provenance/pins_and_run_contract.txt`
- Startup chronology: `provenance/stages.jsonl`
- Terminal review: `conclusion.md` and `STOP.md`
- No rosbag or localization/navigation metrics exist because failure preceded
  the recorder and all runtime interfaces.
- No actual `/joint_states`, `/imu`, `/scan`, `/odom`, FlatScan, Grid result,
  localization manager acceptance, `map→odom`, full TF, Nav2 lifecycle,
  collision/safety, GT drift, or path evidence was produced.

## Next action

A fresh coder should make the canonical session explicitly pass the supplied
snapshot Integration install/root into the Isaac entrypoint while keeping the
underlay validation fail closed. Then rebuild a fresh combined snapshot and
rerun Phase 1B; only a passing interface smoke may dispatch the one-loop
Phase 1C episode.

## Retry after snapshot-underlay repair

**FAIL at Isaac scene composition.** The underlay repair itself passed: fresh
snapshot Integration and Module3 installs resolved exclusively from the
snapshot, and Isaac reached `Simulation App Startup Complete`. The next actual
first error layer was:

```text
RuntimeError: robot asset has unresolved dependencies: ('/tmp/v6_phase1_combined_retry.5gqVsI/m3_src/isaac_sim/assets/robots/jackal/source/jackal_original.usd',)
```

- Module3: `a1232a3cc9f25e9a7ece5dcf64a3a4aa9456fcda`
- Integration: `2578366c350fee741ca2e97cd846d5741b48eb68`
- Module2 metadata only: `c18bd9ea7c69b4cc44e4226a7e37d6e1b803de30`
- Fresh strict snapshot: `/tmp/v6_phase1_combined_retry.5gqVsI`
- Snapshot build: Integration **2 packages finished**; Module3 explicit
  stable/M0 closure **13 packages finished**, excluding `bio_nav_fusion` and
  `rf2o_laser_odometry`.
- Pre-Kit validation: required Integration setup/IDL headers and both
  Integration package prefixes were inside snapshot `i_src/install_r5`;
  `robot_experiments` was inside snapshot `m3_src/install_r5`; overlay paths
  were `/opt -> snapshot Integration -> snapshot Module3` with no live-worktree
  prefix.
- Runtime: domain `210`, episode index `0`, seed `7201`; evidence at
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_20260823T165127Z/`.

The strict archive contains the tracked `source/.gitignore` but not the
required USD. In the allowed live worktree the 1,179,416-byte USD exists but is
untracked and ignored by `source/.gitignore:2:*.usd`. It was not copied into the
strict snapshot. This is now the minimal reproducibility blocker for the
committed-HEAD live path.

Phase 1B did not start: no real sensor, EKF, FlatScan, Grid result/status,
`map->odom`, TF, Nav2 lifecycle, frequency/stamp/frame/QoS, Grid
latency/correction, or initial GT-error evidence exists. Phase 1C was not
authorized; no reset or route goal was sent, so no rosbag, motion metric, or
visual exists. Only the owned driver/Isaac groups were stopped; domain 210 was
empty after cleanup and the unrelated domain-141 `odom_static` remained alive.

Next: a fresh coder should make the required Jackal runtime dependency
reproducibly available from an authorized committed/snapshot source, then build
another strict snapshot and rerun Phase 1B before any goal. This retry is an
engineering live startup failure, not code-test success, navigation success, or
formal qualification.

## Asset-materialization repair status (2026-08-24)

The canonical session now requires an operator-selected absolute
`ISAAC_ASSET_ROOT` and invokes the existing manifest importer plus `--check`
inside archived `m3_src` before starting Isaac. It neither copies from the live
worktree nor versions the NVIDIA binaries. The selected root and verified
materialization status are inputs to `run.yaml`, the materialization log, STOP,
and the run contract.

Focused tests reported **43 passed** and shell syntax passed. In a bounded
strict archive at `/tmp/v6_asset_prekit.LU45Nm`, the local authorized Isaac Sim
Assets 6.0 root materialized both omitted Jackal source layers plus the runtime
configuration destination; the existing check passed and
`dependency_report.unresolved=[]` for `jackal_nav.usda`.

This is code/test/pre-Kit evidence only. The live verdict above remains FAIL;
no Kit, Phase 1B, Phase 1C, reset, or route goal was run by this amendment. A
fresh combined snapshot must be built and the canonical one-episode command in
`V6_GRID_PHASE1_CANONICAL_RUNNER_20260823.md` rerun with
`ISAAC_ASSET_ROOT=/home/lyb/isaacsim_assets/Assets/Isaac/6.0`.

## Live retry after asset materialization (2026-08-24)

**FAIL in Phase 1B before the first reset.** The asset repair itself passed in
the canonical session: all three manifest destinations were imported and
checked inside fresh strict snapshot
`/tmp/v6_phase1_combined_asset_live.w0TLP3`. Kit reached
`Simulation App Startup Complete`; the actual `/clock`, `/joint_states`, raw
and corrected IMU, `/scan`, `/flatscan`, wheel odometry, EKF `/odom`, and
`odom->base_link` streams were recorded.

- Module3: `0df8f131b6226c622f8acbea2f214bfd4a2e75e3`
- Integration: `2578366c350fee741ca2e97cd846d5741b48eb68`
- Module2 metadata only: `c18bd9ea7c69b4cc44e4226a7e37d6e1b803de30`
- Build: Integration 2 packages; Module3 stable/M0 13 packages, excluding
  fusion and RF2O; strict overlay/IDL source-order check passed
- Runtime: confirmed-empty domain `211`, episode index `0`, seed `7201`
- Evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_20260823T172000Z/`

The first error layer is a circular pre-reset startup dependency. The Grid TF
manager creates the transient-local status publisher but emits its first
status only after `/bio_nav/relocalize`. The episode dispatcher requires a
status message in pre-reset readiness before it calls the physical reset; that
reset is what drives Integration's relocalize call. No Grid request therefore
started. The Nav2 activation gate, already running with the canonical
120-second fail-closed policy, then exited with `latest=0 state=none` while
waiting for an accepted Grid generation and `map->odom`.

The 256.461-second MCAP contains 112,748 messages. It records 4,353
`odom->base_link` transforms but zero Grid status, zero NVIDIA localization
results, zero `map->odom`, zero reset events, and zero route goals. The episode
result is `STOP`, with `reset_calls=0`, `localization_generation=null`, and
`goal_publications=0`. Phase 1C was not authorized. No spatial/path/costmap
failure visual exists because global localization and navigation never began;
the decisive evidence is `logs/navigation.log`, the MCAP, and
`review/phase1b_bag_metrics.json`.

Only run-owned process groups were stopped. Domain 211 was empty after cleanup;
the unrelated domain-141 `odom_static` remained alive. This is an engineering
live interface failure, not code-test success, navigation success, or formal
qualification.

Next: a fresh coder should minimally break the pre-reset status/reset cycle
while preserving post-reset WAITING/ACCEPTED, full-TF, Nav2-active, and reset
gate requirements before G2. Then rebuild a fresh strict snapshot and rerun
Phase 1B; no Phase 1C goal is authorized until it passes.

## Startup-order repair status (2026-08-24)

The reproduced cycle has a minimal code/test repair on the Module3
`cognitive-navigation` branch. Pre-reset readiness no longer waits for a Grid
status sample or route/Nav2 readiness; it does require the public
`/bio_nav/relocalize` service endpoint without calling it. Exactly one reset
then opens the post-reset epoch. G2 still requires a newer WAITING followed by
same-generation ACCEPTED with matching finite correction, fresh post-accept
full TF, matching ResetStopGate release, active Nav2, route endpoint, and sole
publisher ownership. Stale ACCEPTED or TF state cannot release the goal gate.

Focused V6 tests reported **33 passed, 28 deselected**. An isolated
`robot_experiments` build, installed CLI dry probe, and installed-package
single-reset/five-leg synthetic sequence passed under
`/tmp/v6_reset_order_isolated.rwsJIm`. These are code/test/build/synthetic
results only; the live FAIL above remains the latest runtime verdict.

Next live boundary: build a fresh strict combined snapshot containing the
repair, confirm domain 212 is empty, and run only episode index 0 / seed 7201
with the existing canonical command and explicit Isaac asset root. Phase 1B
passes only if the one reset now occurs and the complete new-generation
Grid/correction/TF/ActivationGate/Nav2 chain is observed before G2. Otherwise
STOP; no retry, fallback, or timeout increase is authorized.

## R2 live collision and terminal-stop repair (2026-08-24)

The next canonical R2 episode reached Phase 1C but **FAIL/STOP collided** with
the fixed sofa on G1→G2. It is a single-episode engineering failure, not
qualification.

- Module3: `08f3337d7cf0901b5670ec22cfe8477c81af23f8`; domain `231`; index `0`;
  seed `7201`; evidence:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_grid_phase1_clearance_r2_20260824T003034Z/`.
- Phase 1B passed with one reset, Grid generation 1, WAITING_FOR_SCAN ->
  WAITING_FOR_RESULT -> ACCEPTED, 0.133 s latency, and initial Grid-vs-GT
  error 0.071 m / 1.75 degrees.
- The collision occurred 36.434 s after G2 dispatch. Collision Monitor had
  entered StopZone 2.214 s earlier, then cleared. After collision, the bag
  retained 134 nonzero `/cmd_vel_sim` samples and a route success 17.887 s
  later. The collision still owns the episode verdict and no leg completed.
- The canonical script ran its 20-second boundary probe before checking the
  nonzero episode exit and calling `_stop`; this delayed owned navigation
  cleanup. The bag also omitted both raw costmaps, so collision-time costmap
  state could not be reconstructed.

The minimal code/test repair now makes the dispatcher request Nav2 action
cancel exactly once on an active-goal collision/terminal failure, then makes
the session stop its registered navigation PGID before the delayed read-only
probe. Late success cannot overwrite the collision guard. The explicit
recorder adds both raw costmaps. No estimator, controller, Collision Monitor,
Nav2 threshold/config, R2 scene, map, or graph changed.

Direct formal/runtime tests **66 passed** and shell syntax passed. No live run
was performed for this amendment. The next R2 live safety assertion is exact:
one terminal cancel, owned navigation stop before boundary probe, zero nonzero
`/cmd_vel_sim` after that stop boundary, both raw costmaps present, collision
retained over any late success, and domain 141/unowned processes preserved.
