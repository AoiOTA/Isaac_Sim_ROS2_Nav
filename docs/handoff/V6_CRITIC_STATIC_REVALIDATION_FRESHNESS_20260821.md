# V6 critic callback admission and reset-epoch rebind repair

## 2026-08-21 component-trust amendment (current)

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
