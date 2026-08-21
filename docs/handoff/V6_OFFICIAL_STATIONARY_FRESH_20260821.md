# V6 official stationary fresh runtime handoff — Module3

Date: 2026-08-21

## Scope and provenance

- Evidence root:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_stationary_static_authority_fresh_20260821T140713Z`.
- Reviewed sources: `summary.json`, `provenance.json`, and
  `analysis/topology_summary.txt`. The evidence tree was not modified.
- Fresh archived targets were Integration `af913fee42de167741094d5794b0163e4f5aace1`,
  Module3 `1d977d7c822ef81d6139d082cdade373769bdb35`, and Module2
  `2925f806c88b1551d1c48ca89d1c1c5adf2ba748`. Fixed main pins and ancestry
  were verified by the runtime provenance.
- This handoff was authored from the permitted Module3 `cognitive-navigation`
  worktree at pre-commit HEAD `25b97a12ff8c77f0e882cdce259209a7eaeb7374`,
  with fixed main `22d66470c4b903349b2467dc876490bbebfc0083` as
  ancestor and tracked files clean.
- The runtime Module3 snapshot is therefore older than the authoring HEAD. The
  run can support only the archived `1d977d7c...` Bridge/Module2/Costmap-layer,
  TF, and stationary-safety claims below. It does **not** validate the critic at
  current HEAD `25b97a12...`.

## Narrow engineering result

- Verdict: **ENGINEERING PASS (stationary only)**;
  **NOT FORMAL QUALIFICATION**.
- Fresh build/assets: all 14 Module3 packages passed (`27.3 s`), and the Isaac
  asset import/check passed. The post-reset observation window was
  `165.502708967 s`.
- Reset/session discipline: exactly one physical reset call and one reset event
  were recorded. Reset epochs and depth generations had zero regressions; the
  rollover discarded one stale inference result rather than applying it.
- Depth conservation: 2,504 slots accepted, 2,501 consumed, 3 replaced, 0
  pending, and 0 retired at the final counter snapshot; there were zero
  conservation, counter, or generation-regression violations.
- Module2/obstacle handoff: all 518 observed cognitive-obstacle messages were
  healthy, input-healthy, observation-valid, and trusted writes, each carrying
  18 statically confirmed obstacles.
- Costmap application: the global layer applied 284 statuses and the local
  layer 1,431. Global raised/masked cell sums were 771,595/136,353 (maxima
  2,722/485); local sums were 1,231,175/95,362 (maxima 866/73). Raised and
  masked status counts matched the corresponding applied counts.
- Continuity review: the obstacle stream's maximum wall gap was `3.461708187 s`.
  Transient `odom_time`/`validation_stale` rejections produced 48 global and 51
  local stale clears, after which layer application recovered within the
  stationary window. This observation is not moving-authority evidence.
- Stationary safety: nonzero `/cmd_vel`-family count 0, ground-truth
  displacement `0.0 m`, collision-true count 0, and maximum odometry
  displacement `3.924656304988303e-9 m`.
- TF/GT firewall: `map -> odom` produced 1,170 samples at 10 Hz and
  `odom -> base_link` 5,848 samples at 60 Hz, both with zero stamp regressions.
  `/ground_truth/odom` had only the Isaac publisher, and no Bridge, Nav2,
  route, localization, or controller subscriber was present.
- Teardown limitation: after the observation window, two Integration helper
  children emitted `ExternalShutdownException` and exited 1 during commanded
  shutdown. The recorded runtime window had no fatal Bridge or Module2 error.

## Claim limits and next action

- No goal or runner was active. `FollowPath.CognitiveRiskCritic` reported 579
  offered statuses and 0 applied statuses; this is zero-goal observability, not
  a critic PASS or FAIL.
- Module3 used `cognitive_graph_mode=gvg`; graph rows are producer-only evidence.
  No graph application or `PRIMARY` claim is made.
- This run proves neither navigation effectiveness nor moving authority, and it
  cannot be promoted to current-HEAD critic evidence or formal qualification.
- Next: run one fresh, current-HEAD, at-most-180-second, low-speed single-goal
  active-M3 pilot. Any moving authority gap measured in seconds is
  **FAIL / STOP**.
