# V6 critic static-revalidation freshness alignment

## Scope

- Goal: align `CognitiveRiskCritic` obstacle admission with
  `CognitiveObstacleLayer` for `VALIDATION_STATIC_DEPTH_REVALIDATED`.
- Branch/worktree: `cognitive-navigation` in the permitted Module3 worktree.
- Starting HEAD: `e55ccc13fbd01479cd1ad20aefca800f9e218d73`.
- Result commit: the commit containing this handoff.

## Change

- `CognitiveRiskCritic::validateInputs()` delegates obstacle validation to
  `CognitiveObstacleLayer::validateMessage()` using the planning prior as the
  expected identity.
- A static source up to five seconds old is therefore eligible while its depth
  validation is fresh and all dual-timeline, static-confirmation, identity,
  trust, OOD, and obstacle-data checks pass.
- Expired validation, expired ordinary/FRESH input, identity drift, untrusted
  writes, OOD, and malformed obstacle data still reject before scoring, so the
  critic adds zero cognitive cost.
- Planning-prior TTL/freshness, direction-prior handling, scoring weights, TF,
  control, Costmap ownership, and MPPI authority are unchanged.

## Validation

- Isolated build root: `/tmp/bio_nav_module3_critic.JLGsAi`.
- Fresh build of allowed Integration `bio_nav_interfaces` and Module3
  `bio_nav_fusion`: PASS.
- `test_equal_cost_search`: PASS, 28/28 tests. New coverage checks fresh static
  revalidation with source age above 0.5 seconds, expired validation, expired
  ordinary/FRESH input, identity mismatch, OOD, untrusted input, and matching
  layer/critic verdicts for the same message and time.
- `test_plugin_loader_isolation`: PASS, 1/1.
- `git diff --check`: PASS before commit.
- An initial isolated build at `/tmp/bio_nav_module3_critic.mUTBid` failed
  before this change compiled because the pre-existing Integration overlay had
  CMake metadata but no generated `CognitiveObstacleArray` header. The passing
  run rebuilt the interface from its allowed `cognitive-navigation` source.

Commands:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/bio_nav_module3_critic.JLGsAi/log_build build \
  --base-paths \
    /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_intergration/ros2_ws/src/bio_nav_interfaces \
    /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3/ros2_ws/src/bio_nav_fusion \
  --packages-up-to bio_nav_fusion \
  --build-base /tmp/bio_nav_module3_critic.JLGsAi/build \
  --install-base /tmp/bio_nav_module3_critic.JLGsAi/install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo

source /tmp/bio_nav_module3_critic.JLGsAi/install/setup.bash
colcon --log-base /tmp/bio_nav_module3_critic.JLGsAi/log_test test \
  --packages-select bio_nav_fusion \
  --build-base /tmp/bio_nav_module3_critic.JLGsAi/build \
  --install-base /tmp/bio_nav_module3_critic.JLGsAi/install \
  --ctest-args -R '^test_equal_cost_search$' --output-on-failure

ctest --test-dir /tmp/bio_nav_module3_critic.JLGsAi/build/bio_nav_fusion \
  -R '^test_plugin_loader_isolation$' --output-on-failure
```

## Result and remaining risk

- Verdict: **PASS (implementation/build/unit only)**.
- No active MPPI, ROS graph, Nav2, Isaac, navigation, engineering evidence, or
  formal qualification was run.
- Next authorized runtime check: observe both Costmap and critic status on the
  same fresh static-depth-revalidated message whose source age exceeds 0.5
  seconds, then expire `validation_ttl` and confirm the critic adds zero cost.
