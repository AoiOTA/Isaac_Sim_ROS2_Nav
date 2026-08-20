# V6 M1 shadow engineering handoff — 2026-08-21

## Scope and checkout

- Goal: record the live M1 telemetry/control-isolation observation from
  `/tmp/v6_live_m1_shadow.UDx2Bz`; this amendment does not change source,
  configuration, scripts, or installed artifacts.
- Module3 worktree/branch: `worktrees/cognitive-navigation/bio_nav_module3` /
  `cognitive-navigation`.
- Fixed Module3 `refs/heads/main`:
  `22d66470c4b903349b2467dc876490bbebfc0083`.
- Starting Module3 HEAD: `faddfbd3b840450dfa49c1e6b87771a3613cd907`.
- Live stack revisions: Integration
  `430e977884202d9800235b73eab320dd68e5f325`, Module2
  `163dbdc4a469c37aced5a1a7c673b84b2765efe4`, and Module3
  `faddfbd3b840450dfa49c1e6b87771a3613cd907`.

## Observed result

- **M1 telemetry and control isolation: PASS (engineering observation).** The
  30.01 s joint probe received 14 obstacle messages and all 14 were nonempty;
  every message contained 18 obstacles. `observation_valid`, `input_healthy`,
  and `module2_healthy` were true, while `trusted_write` remained false.
- The maximum advertised TTL was 0.5 s. Identity stayed stable
  (`identity_variants=1`) while source sequence advanced `3022 -> 3118`.
- Both cognitive consumers remained shadow-only. Layer and critic status
  reported `applied=false` and `raised_cell_count=0`; the critic reported
  `fallback_reason=shadow`.
- No command or goal stimulus was observed in the window:
  `/cmd_vel` messages/nonzero messages were `0/0`, `/initialpose=0`, and
  `/goal_pose=0`.
- AMCL and the Nav2 lifecycle nodes were active before the Integration start.
  Runtime modes were `shadow` for both cognitive obstacle layers and the
  cognitive critic.
- Isaac reported `camera=rgbd_navigation`. The live local/global Costmap plugin
  lists used the direct obstacle layer plus the cognitive layer and inflation
  (and the global static layer); neither voxel nor STVL was enabled. The classic
  obstacle layers subscribed to `/scan`.

## Why the overall verdict is PARTIAL

- **Overall M1 acceptance: PARTIAL.** All three layer captures rejected the
  offered sample as `fallback_reason=stale`, with message age approximately
  1.7 s. Therefore this run did not establish a fresh cognitive-layer offer,
  projection, or merge path even though shadow isolation held.
- Scan invisibility was not verified: `scan_invisibility_probe.json` reports
  `success=false` and `result=null`; the one-shot obstacle/scan comparison had
  no candidates and a 0.6 s stamp delta.
- After Module2 was stopped, the captured core lifecycle states still showed
  AMCL and Nav2 controller/planner/route active. However, no fresh cognitive
  layer status was obtained after that stop, so fail-open continuity of the
  consumer status path remains unverified.
- This is **not** an M2 active-mode result, causal navigation evidence, or a
  formal qualification PASS. `trusted_write=false`, `applied=false`, zero
  raised cells, and absence of a navigation goal preclude those claims.

## Evidence, cleanup, and next step

- Primary artifacts:
  `m1_30s_joint_probe.json`, `obs_sync_once.yaml`, the three
  `cognitive_layer_status_once_*.yaml` captures, the three
  `cognitive_critic_status_once_*.yaml` captures,
  `runtime_params_shadow.log`, `lifecycle_pre_int.log`,
  `failopen_after_m2_stop.log`, `scan_invisibility_probe.json`, and
  `scan_once_obstacle_comparison.json`, all under
  `/tmp/v6_live_m1_shadow.UDx2Bz`.
- Full owned launch/monitor logs and PID records remain in the same directory;
  all processes owned by this live run were cleaned up. The exact live launch
  command transcript was not preserved as a standalone artifact, so it is not
  reconstructed here.
- Next live run should capture a fresh layer-status sample within TTL, align the
  scan/obstacle timestamps for the invisibility check, and capture a new layer
  status after stopping Module2. Those are engineering follow-ups, not grounds
  to upgrade this record to causal or formal qualification.
