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
