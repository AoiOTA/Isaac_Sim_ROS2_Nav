# V6 critic static-revalidation and obstacle-independence repair

## 2026-08-21 real-timestamp obstacle-independence amendment (current)

### Scope and result

- Starting HEAD: `fea521b5b849c381d29191659ea97b291e7a0aeb` on the permitted
  Module3 `cognitive-navigation` worktree. The fixed Module3 main
  `22d66470c4b903349b2467dc876490bbebfc0083` remained its ancestor.
- Goal: remove the false assumption that Integration V3.10 republishes a
  same-sequence fresh planning prior when it refreshes a cached static
  obstacle. `PlanningPrior.stamp` is the inference source stamp; static depth
  revalidation refreshes only the obstacle validation timeline.
- Verdict: **PASS (code/build/unit only)**. Result commit: the commit containing
  this handoff.

### Change

- Obstacle admission and scoring no longer depend on a planning prior. The
  callback uses the same trusted-write, health, identity, dual-timeline,
  finite/OOD, ordering, and static-confirmation validator as
  `CognitiveObstacleLayer`, records its own accepted cursor, and preserves the
  last accepted snapshot on rejection. `score()` rechecks the accepted
  obstacle and uses `validation_stamp` for TF.
- A missing, stale, untrusted, OOD, identity/session-incompatible, or
  different-sequence prior suppresses context, novelty, uncertainty, and local
  direction only. It cannot suppress an otherwise accepted obstacle. A prior
  must retain its real positive inference stamp and pass the basic health,
  trust, OOD, identity, and matching-inference-sequence gates before any prior
  component can contribute. Context and local direction retain their own
  component gates.
- Same-instance reset rebind follows only an accepted obstacle. It requires a
  strictly higher reset epoch, a new recurrent session, and unchanged
  map/tile/tile-revision/graph/model identity. The source/validation cursor is
  reset only after that candidate passes the obstacle validator. Old epochs,
  replay, untrusted candidates, and arbitrary identity changes cannot rebind;
  no accepted or fresh planning prior is required.
- Status uses `obstacle_applied=true` for the accepted obstacle and explicitly
  reports each prior component as applied or suppressed with its reason. A
  basically valid prior is described as `prior_accepted`, not as an overall
  applied control influence. Obstacle rejection remains explicit as
  `obstacle_rejected=<reason>`.
- The test helper for static revalidation changes obstacle source and
  validation stamps only. It never changes `PlanningPrior.stamp`. The
  validation-time moving/rotating TF test now scores the static obstacle with
  no prior instead of manufacturing a validation-time prior.
- No Integration or Module2 source was changed, no prior was replayed or
  synthesized on the production path, and no `GoalPlanningPrior` subscription
  was added. The critic remains additive/nonnegative; Module3 TF, Costmap,
  collision handling, Nav2, and `/cmd_vel` authority are unchanged.

### Validation

- Fresh isolated build root:
  `/tmp/bio_nav_module3_critic_independent.pbOLuY`.
- Allowed Integration `bio_nav_interfaces` plus Module3 `bio_nav_fusion`:
  PASS, 2/2 packages.
- Focused `test_equal_cost_search`: PASS, 35/35 gtest cases. Production-time
  cases cover an old source/sequence obstacle with a fresh validation and the
  original matching but stale prior; a fresh different-sequence prior; no
  prior; a legal fresh original pair; stale/untrusted/OOD/nonfinite obstacle
  rejection; validation-time TF; ordinary LIVE behavior; callback
  interleaving; and trusted higher-epoch/no-prior rebind with old-epoch replay
  rejection.
- `test_plugin_loader_isolation`: PASS, 1/1.
- `git diff --check`: PASS before commit.

Commands:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/bio_nav_module3_critic_independent.pbOLuY/log_build build \
  --base-paths \
    /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_intergration/ros2_ws/src/bio_nav_interfaces \
    /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3/ros2_ws/src/bio_nav_fusion \
  --packages-up-to bio_nav_fusion \
  --build-base /tmp/bio_nav_module3_critic_independent.pbOLuY/build \
  --install-base /tmp/bio_nav_module3_critic_independent.pbOLuY/install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo

source /tmp/bio_nav_module3_critic_independent.pbOLuY/install/setup.bash
colcon --log-base /tmp/bio_nav_module3_critic_independent.pbOLuY/log_test_equal test \
  --packages-select bio_nav_fusion \
  --build-base /tmp/bio_nav_module3_critic_independent.pbOLuY/build \
  --install-base /tmp/bio_nav_module3_critic_independent.pbOLuY/install \
  --ctest-args -R '^test_equal_cost_search$' --output-on-failure

ctest --test-dir \
  /tmp/bio_nav_module3_critic_independent.pbOLuY/build/bio_nav_fusion \
  -R '^test_plugin_loader_isolation$' --output-on-failure
```

### Correction and remaining risk

- This amendment supersedes the earlier handoff statements that a fresh basic
  obstacle/prior pair exists for every static refresh or that such a pair is
  required for reset rebind. Those statements were based on a test helper that
  incorrectly copied `validation_stamp` into `PlanningPrior.stamp`. Earlier
  sections remain below only as historical provenance.
- No ROS graph, active MPPI, Nav2, Isaac, navigation, visual evidence,
  engineering campaign, or formal qualification was run. Live callback/status
  observation and nonzero active-MPPI obstacle influence remain unverified.

## 2026-08-21 component-trust amendment (superseded)

### Scope and result

- Starting HEAD: `1d977d7c822ef81d6139d082cdade373769bdb35` on the permitted
  Module3 `cognitive-navigation` worktree. The fixed Module3 main
  `22d66470c4b903349b2467dc876490bbebfc0083` remained its ancestor.
- Goal: make the critic consume the trusted obstacle component of the real
  Integration V3.10 `PlanningPrior` contract without promoting diagnostic
  context or periodic local direction to trusted control input.
- Verdict: **PASS (code/build/unit only)**.

### Change

- `validateInputs()` is now the basic obstacle/prior pair gate. It still
  requires matching sequence and complete generation identity, permitted
  schemas, fresh stamps, healthy/trusted top-level inputs, zero rejection
  masks, bounded OOD, and finite values. `context_trusted=false` no longer
  rejects the complete pair.
- A basic accepted pair is recorded by either callback ordering and is
  sufficient for the existing strictly-higher-epoch/new-session reset rebind.
  Existing replay, same/older epoch, map/tile/graph/model, route/schema, and
  identity-change rejection remains in force.
- Obstacle scoring remains nonnegative and uses the existing validation-time
  TF and trusted obstacle samples. When `context_trusted=false`, novelty and
  uncertainty inputs are exactly zero. Periodic local direction is used only
  when `validateDirectionPrior()` passes schema, frame, sequence, health,
  trusted-write, normalization, and TTL gates; the real V3.10
  `module2_canvas` / `local_direction_trusted_write=false` envelope therefore
  contributes exactly zero direction cost.
- Applied status now states `obstacle_applied` and explicitly lists suppressed
  novelty, uncertainty, and/or direction components. Thus `applied=true`
  describes the obstacle component, not an assertion that direction was used.
- No `GoalPlanningPrior` subscription was added. Goal-conditioned direction is
  still a separate Integration service/topic and is **not connected to this
  critic** by this amendment.

### Validation

- Fresh isolated build root:
  `/tmp/bio_nav_module3_critic_component.gIA7MD`.
- Allowed Integration `bio_nav_interfaces` plus Module3 `bio_nav_fusion`:
  PASS, 2/2 packages.
- Focused `test_equal_cost_search`: PASS, 35/35 gtest cases. Production-shaped
  V3.10 cases cover trusted static-revalidated obstacle cost with
  context/direction suppressed, synthetic trusted context enabling only
  novelty/uncertainty, stale/untrusted/identity-mismatched zero cost,
  prior-first and obstacle-first interleaving, and context-false reset/replay.
- `test_plugin_loader_isolation`: PASS, 1/1. Total focused gtest cases: 36.
- `git diff --check`: PASS before commit.
- Correction to the preceding handoff: its focused result was **33**
  `test_equal_cost_search` cases plus **1** loader case, total **34**, not 35.

### Remaining risk

- No ROS graph, active MPPI, Nav2, Isaac, navigation, visual evidence,
  engineering campaign, or formal qualification was run. Live V3.10 callback
  scheduling, status observation, and nonzero active-MPPI obstacle cost remain
  unverified.
- M0/M1/M2/M3 configuration helpers were not changed. The focused C++ tests
  exercise active scoring and fail-open gates; mode-profile behavior was not
  rerun in this amendment.
- Goal-conditioned direction remains unconnected and must not be claimed as
  implemented by the critic.

## Scope

- Goal: close the reviewer blockers in `CognitiveRiskCritic` callback ordering
  and same-instance reset recovery while retaining the earlier
  static-revalidation TF/freshness behavior.
- Branch/worktree: `cognitive-navigation` in the permitted Module3 worktree.
- Repair starting HEAD: `1e77edce4fec62ad0387f32c59e9925b0a898096`.
- Result commit: the commit containing this handoff.

## Change

- The obstacle callback now runs the same
  `CognitiveObstacleLayer::validateMessage()` gate as the Costmap layer and
  immediately advances the persistent source/validation cursor. It stores only
  accepted snapshots. Duplicate, backward, source-regressed, malformed,
  untrusted, OOD, or identity-changing callbacks leave the latest accepted
  snapshot intact.
- `score()` reuses that accepted snapshot without a second ordering admission.
  It still rechecks obstacle/prior identity and sequence pairing, prior TTL,
  obstacle validation TTL, trust, health, schema, OOD, and finite data on every
  cycle. Missing or mismatched pairs therefore add zero cognitive cost.
- Nav2 Jazzy's MPPI `CriticFunction` API exposes no reset or lifecycle hook to
  an individual critic; controller reset does not call a critic reset method.
  Same-instance rebind therefore uses the two canonical input streams as the
  reset authority. Under the input mutex, rebind is allowed only when:
  - the candidate `reset_epoch` is strictly greater than the bound epoch;
  - the recurrent session changes;
  - obstacle and current planning prior have the same complete identity and
    sequence and both pass the existing freshness/health/trust/schema gates;
  - map version, cognitive tile, tile revision, graph revision, model ID, prior
    schema, local-direction graph, physical graph ID/revision, and topology
    revision remain equal to the last accepted pair.
- On that proof, the old obstacle/cursor/identity state is cleared and the
  already validated new obstacle/prior pair is bound atomically. Same-epoch
  identity changes, epoch rollback/replay, changed map/tile/graph/model, and
  untrusted reset pairs cannot rebind.
- Initialization now clears both input pointers as well as cursor and identity.
  All callback, rebind, and snapshot state stays under one mutex; scoring copies
  immutable shared pointers and identity under that mutex before validation, so
  it does not hold the lock across TF or trajectory work.
- Validation-time TF, moving/rotating trajectory score behavior, ordinary LIVE
  behavior, direction-prior handling, scoring weights, Costmap ownership,
  physical safety ownership, and MPPI control authority are unchanged.

## Validation

- Isolated build root:
  `/tmp/bio_nav_module3_critic_admission.Bzf9rD`.
- Fresh allowed Integration `bio_nav_interfaces` plus Module3
  `bio_nav_fusion` build: PASS, 2/2 packages.
- Focused `test_equal_cost_search`: PASS, 33/33 gtest cases. Together with the
  1/1 loader case, the preceding handoff covered 34 focused cases (corrected
  from the earlier erroneous value of 35).
- `test_plugin_loader_isolation`: PASS, 1/1.
- `git diff --check`: PASS before commit.
- The first build-shell invocation stopped before colcon because `set -u`
  conflicts with `/opt/ros/jazzy/setup.bash`; rerunning without nounset produced
  the successful isolated results above.

The focused score/callback tests retain validation-time TF with translated and
rotated frames, ordinary LIVE placement, repeated scoring, and missing,
expired, malformed, OOD, and untrusted zero-cost behavior. New tests drive the
real private callback entry through the test peer and cover first binding,
duplicate/backward/source-regression/identity rejection, newer obstacle before
prior pairing, lower-sequence rejection matching the layer, same-instance
  trusted epoch rebind, old-epoch replay rejection, changed-map/route rejection,
  and untrusted reset rejection.

Commands:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/bio_nav_module3_critic_admission.Bzf9rD/log_build build \
  --base-paths \
    /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_intergration/ros2_ws/src/bio_nav_interfaces \
    /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3/ros2_ws/src/bio_nav_fusion \
  --packages-up-to bio_nav_fusion \
  --build-base /tmp/bio_nav_module3_critic_admission.Bzf9rD/build \
  --install-base /tmp/bio_nav_module3_critic_admission.Bzf9rD/install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo

source /tmp/bio_nav_module3_critic_admission.Bzf9rD/install/setup.bash
colcon --log-base /tmp/bio_nav_module3_critic_admission.Bzf9rD/log_test_final test \
  --packages-select bio_nav_fusion \
  --build-base /tmp/bio_nav_module3_critic_admission.Bzf9rD/build \
  --install-base /tmp/bio_nav_module3_critic_admission.Bzf9rD/install \
  --ctest-args -R '^test_equal_cost_search$' --output-on-failure

ctest --test-dir /tmp/bio_nav_module3_critic_admission.Bzf9rD/build/bio_nav_fusion \
  -R '^test_plugin_loader_isolation$' --output-on-failure
```

## Result and remaining risk

- Verdict: **PASS (implementation/build/unit only)**.
- No active MPPI, ROS graph, Nav2, Isaac, navigation, engineering evidence, or
  formal qualification was run. Live callback scheduling and the producer
  ordering of the new prior/obstacle pair remain unverified. If the obstacle
  arrives before the matching new-epoch prior, it is safely rejected and a
  later obstacle publication is required after the prior arrives.
- Next authorized runtime check: keep one MPPI controller instance alive across
  an Integration reset, observe a strictly higher epoch/new session on both
  streams, confirm critic status returns to applied, then replay the old epoch
  and confirm it cannot displace the new accepted snapshot.

## 2026-08-21 route-context rebind and applied-truth amendment

### Scope and result

- Starting HEAD: `3dc2830c1da5b5f441191217220bc120058bd4b2` on the permitted
  Module3 `cognitive-navigation` worktree; fixed main ancestry and tracked-clean
  preflight passed.
- Goal: close the two follow-up reviewer blockers without changing Integration,
  Module2, `GoalPlanningPrior`, Costmap ownership, safety ownership, or the
  stationary-runtime handoff.
- Verdict: **PASS (implementation/build/unit only)**. This amendment supersedes
  the earlier claim that a higher-epoch obstacle alone can rebind the critic.

### Change

- A higher-epoch/new-session obstacle can no longer rebind from obstacle
  identity alone. The critic first binds a basic-compatible `PlanningPrior`
  route context: planning/local-direction schemas, route graph ID, physical
  graph ID/revision, topology revision, and the complete
  map/tile/graph/model/session/reset identity. Core input/module health,
  observation validity, trusted-write, and rejection-mask gates must pass.
- Rebind accepts only an unchanged route context relative to the old stable
  context and the exact pending new identity. Prior source age and sequence are
  intentionally not scoring gates for this identity proof, so a stale or
  different-sequence prior may authorize the context; it still cannot add
  context or direction cost. Obstacle-first delivery reports
  `reset_route_context_missing`; the next compatible obstacle after the prior
  performs rebind. Changed route/physical revision/topology/schema, stable
  identity spoofing, and old-epoch replay do not rebind.
- `RiskLayerStatus.applied` and component `*_applied` text now reflect actual,
  finite positive changes to `data.costs` above `1e-6`, not merely successful
  input admission. Empty obstacle arrays, zero weights, and trajectories with
  zero effective contribution report `zero_cost_delta`, `applied=false`, and
  `obstacle_applied=false`. Context/direction may independently set overall
  applied while obstacle remains false. Nonfinite, overflowing, or negative
  component scores fail open and never mutate costs.
- Every distinct rejected obstacle callback publishes an independent rejected
  offer status and is retained under the critic mutex. Later scoring of the old
  accepted snapshot names both `accepted_source_sequence` and the latest
  rejected offer identity/reason, so it cannot imply the rejected offer was
  applied. Exact duplicate rejections are suppressed to avoid callback spam.
- The low-obstacle causal evaluator now recognizes `online_applied` only when
  status contains `cost_delta_applied=true`; legacy or overall-zero-delta
  `applied=true` evidence cannot claim online participation. Individual zero
  components may still coexist with a positive overall delta.

### Validation

- Fresh isolated allowed Integration `bio_nav_interfaces` + Module3
  `bio_nav_fusion` build: PASS, 2 packages. Build/install roots:
  `ros2_ws/build_critic_fix_kOEv7h` and
  `ros2_ws/install_critic_fix_8raZSf`.
- Focused CTest: `test_equal_cost_search` PASS and
  `test_plugin_loader_isolation` PASS (2/2 executables).
- Source-first Python focused tests: low-obstacle causal, localization causal,
  and bringup mode contract: **52 passed**.
- `git diff --check`: PASS after documentation update.
- Test coverage retains the real V3.10 old-source static refresh with stale,
  different-sequence, and missing priors; validation-time TF; callback cursor;
  off/shadow/mode and safety contracts. New cases cover stale-prior route
  authorization, reset pending/rebind/replay, route/revision/topology/schema and
  spoof rejection, empty/zero/far/positive component deltas, context-only and
  direction-only deltas, nonfinite/negative fail-open scoring, rejected-offer
  observability, and causal online-applied truth.

### Remaining risk

- No ROS graph, active MPPI, Nav2, Isaac, navigation, visual evidence,
  engineering campaign, or formal qualification was run. Live DDS callback
  ordering, status observation, and active-MPPI influence remain unverified.
- `GoalPlanningPrior` remains unconnected. Goal-conditioned direction and the
  earlier stationary run are not validated or changed by this code/build/unit
  amendment.
