# V6 critic callback admission and reset-epoch rebind repair

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
- Focused `test_equal_cost_search`: PASS, package summary 35 tests / 0 errors /
  0 failures / 0 skipped.
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
