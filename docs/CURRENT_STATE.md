# Module3 current runtime handoff

Date: 2026-08-31

The authoritative cross-repository state and stop boundary are in the
Integration
[`V6_INDOOR_TWO_PHASE_READINESS_STOP_HANDOFF_20260831.md`](/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration/docs/handoff/V6_INDOOR_TWO_PHASE_READINESS_STOP_HANDOFF_20260831.md).
This file records the Module3 boundary only.

## Stop and exact source state

The current state is **SOURCE/TEST/BUILD CLOSED, LIVE NOT VERIFIED, STOP**.
Do not start ROS, Nav2, Module2, rosbag, Isaac/GPU, seed `8601`, Pilot,
qualification, outdoor startup A/B, or Formal execution without a new user
instruction.

- canonical worktree:
  `/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module3`
- branch: `v6-compute-amcl-dual-odom`
- final runtime source commit:
  `7ca46e639cb836a4b126d3a46c6d57d8f282a7c9`
- final runtime source tree:
  `b533b622814f88c97b52c64eddb9f56743046e38`
- final source tuple: Integration `50b4174133b78350fcff2c9f25dc882ef3148ed4`,
  Module2 `2f584b00fe0ff89cca5063afbcdb544432a233ed`, Module3
  `7ca46e639cb836a4b126d3a46c6d57d8f282a7c9`

At documentation time the branch was local-ahead and not pushed. Historical
untracked `ros2_ws/build*`, `install*`, and `log*` trees remain preserved and
are not source evidence. Before any future run, require live upstream/remote
checks, tracked-clean source, and package/module provenance from the exact
source tuple.

## Implemented Module3 closure

The current source contains:

- fail-fast navigation governance and owner-visible callback/runner failures;
- the static odometry endpoint timing correction;
- fail-closed cognitive episode evidence and MCAP rereading;
- required ResetStopGate, planning-chain identity, generation/session,
  map/content namespace, and immutable episode-receipt bindings;
- a two-phase cognitive readiness contract: current Module2 periodic health
  plus global/local layer admission are required before dispatch, while the
  MPPI critic is proven after dispatch and before the first non-zero command;
- current-episode post-dispatch critic evidence is reread from the production
  MCAP rather than inferred from a pre-dispatch score that cannot yet exist;
- live `/proc` viewport producer argv attestation, viewport A/B arm support,
  critical-child manifests, and owned process-group cleanup;
- split indoor/outdoor half-Pilot and half-qualification aggregation/freezing,
  with later six-condition combination and no double counting.

ContactSensor `/simulation/collision` remains the only physical-contact truth.
SAT/AABB/geometric overlap is diagnostic only. Collision Monitor stop,
navigation failure, invalid infrastructure/evidence, and physical contact keep
separate result semantics.

## Evidence boundary

Final source validation reported `476 passed` plus a separate `59 passed`, an
isolated affected build PASS, and `72` gtest cases across `2` CTest targets
PASS. Earlier readiness review
findings drove the recovery, but no fresh independent review of final
`7ca46e6` was performed. These are code/test/build claims, not live, Pilot, or
Formal qualification.

The only current-cycle live attempt used the older tuple Integration `d9bd139`,
Module2 `2f584b0`, Module3 `e1d9e30`, seed `8601`. It stopped invalid before
route dispatch; ContactSensor remained false and no Xid was observed. Its root
is
`/mnt/nas_home/Bio_Nav_Data/experiments/engineering/v6_indoor_seed8601_matched_control_d9bd139_e1d9e30_20260831T1328Z`.
It is not a product result and is not evidence for final `7ca46e6`.

Current counts are: indoor Pilot `0/9`, indoor qualification `0/60`, current
outdoor viewport startup A/B `0/6`, outdoor Pilot `0/9`, outdoor qualification
`0/60`, combined qualification `0/120`. Historical indoor `9/9` and historical
descriptor-set startup results cannot be back-labelled to this source tuple.

## Next eligible command order

Only after a new user instruction: verify the exact tuple and source/install
provenance, run preflight/build checks, then rerun one seed-`8601` indoor
engineering attempt. Only a valid PASS may unlock indoor `9/9`, then indoor
`3x20`; subsequently run current outdoor startup A/B, outdoor `9/9`, outdoor
`3x20`, and combine the two qualified halves. Full formal execution remains
`NOT_AUTHORIZED` until its separate explicit authorization contract is met.
The detailed commands are retained in [RUNBOOK.md](RUNBOOK.md); their presence
does not authorize execution.
