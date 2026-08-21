# V6 critic static-revalidation TF and acceptance-state rework

## Scope

- Goal: align `CognitiveRiskCritic` obstacle admission and TF time with
  `CognitiveObstacleLayer` for `VALIDATION_STATIC_DEPTH_REVALIDATED`.
- Branch/worktree: `cognitive-navigation` in the permitted Module3 worktree.
- Rework starting HEAD: `6a64e4a67cd866759e07ca3db2dfed77efa8153b`.
- Result commit: the commit containing this handoff.

## Change

- `CognitiveRiskCritic::validateInputs()` delegates obstacle validation to
  `CognitiveObstacleLayer::validateMessage()` using the same compound
  acceptance cursor and bound identity contract as the Costmap layer.
- Only a newly received obstacle snapshot goes through the cursor ordering
  gate. An already accepted snapshot is rechecked without the ordering gate so
  MPPI can score it on every cycle until freshness or trust validation fails.
- The cursor, bound identity, and accepted snapshot are protected by the
  critic's input mutex and reset during critic initialization. A changed
  identity rejects like the bound Costmap layer; a fresh critic initialization
  can bind the new identity.
- Accepted obstacle points and robot yaw now use `validation_stamp` for TF.
  The validator requires it to equal the source stamp for ordinary `FRESH`
  input, while static revalidation uses the fresh validation-time pose that
  Integration already compensated into `base_link`.
- A static source up to five seconds old is therefore eligible while its depth
  validation is fresh and all dual-timeline, static-confirmation, identity,
  trust, OOD, and obstacle-data checks pass.
- Expired validation, expired ordinary/FRESH input, identity drift, untrusted
  writes, OOD, and malformed obstacle data still reject before scoring, so the
  critic adds zero cognitive cost.
- Planning-prior TTL/freshness, direction-prior handling, scoring weights,
  control, physical safety ownership, Costmap ownership, and MPPI authority are
  unchanged.

## Validation

- Final isolated build root:
  `/tmp/bio_nav_module3_critic_rework_final.TIm4by`.
- Fresh build of allowed Integration `bio_nav_interfaces` and Module3
  `bio_nav_fusion`: PASS.
- `test_equal_cost_search`: PASS, package summary 32 tests / 0 errors / 0
  failures / 0 skipped. New score-level coverage injects moving `map <-
  base_link` TF samples and calls `score()` to check static validation-time
  placement and yaw, ordinary LIVE source/effective-time placement, repeat
  scoring of an accepted snapshot, expired/missing/bad/OOD/untrusted zero cost,
  cursor duplicate/regression rejection, bound identity rejection, and
  reset/rebind.
- `test_plugin_loader_isolation`: PASS, 1/1.
- `git diff --check`: PASS before commit.

Commands:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/bio_nav_module3_critic_rework_final.TIm4by/log_build build \
  --base-paths \
    /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_intergration/ros2_ws/src/bio_nav_interfaces \
    /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3/ros2_ws/src/bio_nav_fusion \
  --packages-up-to bio_nav_fusion \
  --build-base /tmp/bio_nav_module3_critic_rework_final.TIm4by/build \
  --install-base /tmp/bio_nav_module3_critic_rework_final.TIm4by/install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo

source /tmp/bio_nav_module3_critic_rework_final.TIm4by/install/setup.bash
colcon --log-base /tmp/bio_nav_module3_critic_rework_final.TIm4by/log_test_equal test \
  --packages-select bio_nav_fusion \
  --build-base /tmp/bio_nav_module3_critic_rework_final.TIm4by/build \
  --install-base /tmp/bio_nav_module3_critic_rework_final.TIm4by/install \
  --ctest-args -R '^test_equal_cost_search$' --output-on-failure

ctest --test-dir /tmp/bio_nav_module3_critic_rework_final.TIm4by/build/bio_nav_fusion \
  -R '^test_plugin_loader_isolation$' --output-on-failure
```

## Result and remaining risk

- Verdict: **PASS (implementation/build/unit only)**.
- No active MPPI, ROS graph, Nav2, Isaac, navigation, engineering evidence, or
  formal qualification was run. Runtime callback scheduling and live TF-buffer
  behavior remain unverified.
- Next authorized runtime check: move the robot between source and validation
  time, observe Costmap and critic status/placement on the same fresh static
  revalidation, then expire `validation_ttl` and confirm zero cognitive cost.
