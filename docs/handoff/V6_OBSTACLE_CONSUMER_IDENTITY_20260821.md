# V6 CognitiveObstacleLayer consumer identity amendment

## Scope

- Goal: make the shared absolute obstacle-layer status topic distinguish the
  global and local Costmap consumers without changing obstacle validation or
  Costmap writes.
- Branch/worktree: `cognitive-navigation` in the permitted Module3 worktree.
- Starting Module3 HEAD: `dff45d0434638b9282bf6ea61811f09f638bb0cf`.
- Result commit: the commit containing this handoff.

## Change

- `CognitiveObstacleLayer` declares an optional plugin-scoped `consumer_id`
  parameter.
- A non-empty override is used verbatim. Otherwise the stable identity is
  `<costmap node fully-qualified name>:<layer/plugin name>`.
- Empty node/layer inputs have deterministic fallbacks
  `/unknown_costmap:cognitive_obstacle_layer`.
- Every `RiskLayerStatus` publication now uses the resolved consumer identity;
  topic, validation, freshness, TF, costs, merge behavior and fail-open behavior
  are unchanged.

Expected default live identities are:

- `/global_costmap/global_costmap:cognitive_obstacle_layer`
- `/local_costmap/local_costmap:cognitive_obstacle_layer`

## Validation

- Isolated `bio_nav_fusion` build: PASS.
- Package tests: PASS, 26 tests / 0 errors / 0 failures / 0 skipped. The two new
  unit cases cover distinct/stable global and local fake Costmap namespaces,
  explicit override, empty override and fully empty deterministic fallback.
- `git diff --check`: PASS before commit.
- Build/test root: `/tmp/v6_obstacle_consumer_identity.mdIXgn`.

The successful build used the complete Integration interface underlay at
`ros2_ws/install_m1_reval_current_eCAtsc`. An initial attempt against the
default Integration install stopped at compile time because that install did
not contain the generated CognitiveObstacleArray header; it was not a product
test result.

## Result and remaining risk

- Verdict: **PASS (implementation/build/unit only)**.
- No ROS, Nav2, Isaac, navigation, evidence or qualification campaign was
  launched. A later authorized live run must confirm the actual Nav2 lifecycle
  node fully-qualified names and observe one status stream from each consumer.

