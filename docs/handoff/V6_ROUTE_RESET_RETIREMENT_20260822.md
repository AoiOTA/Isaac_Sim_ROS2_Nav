# V6 RouteCoordinator reset retirement handoff

Date: 2026-08-22

## Scope and verdict

- Worktree: permitted Module3 worktree
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`.
- Branch/start: `cognitive-navigation` at
  `baa05f96061bb8084c71d258216fe0aa105568cc` for this review-blocker
  amendment (the original retirement implementation started at
  `0a0fdf8adccaf95bfdf4d933993fa926ba6f3be4`).
- Fixed main ancestry: `22d66470c4b903349b2467dc876490bbebfc0083`.
- Goal: retire every RouteCoordinator-owned old-epoch route/action intent when
  `/simulation/reset_event` arrives, and prevent normal goal preemption from
  exposing an old tracker under a new request ID; close the multithreaded
  callback TOCTOU, Route Server graph-authority, stale rebuild, and transient
  runtime-state review blockers.
- Verdict: **PASS (code/build/unit only)**. No ROS graph, Isaac, Nav2,
  navigation, visual evidence, engineering campaign, or formal qualification
  was run.

## Implementation

- Added a synchronous reset retirement path. It captures the old request,
  goal, and accepted action handle; advances request and graph callback fences;
  then clears the active route, pending goal/prior, tracker, navigation flags,
  reroute state, transient runtime edges, old pose/costmap/TF/region cache, and
  pending structural candidate without triggering a structural rebuild.
- Cancellation is best effort and occurs only after coordinator state is
  cleared. A pending `NavigateToPose` future that accepts after its generation
  becomes stale is still consumed and its returned handle is immediately
  cancelled rather than retained.
- An active route reset publishes exactly one
  `/bio_nav/route_goal_complete=false` and one String JSON terminal on
  `/bio_nav/route_goal_result` with `request_id`, `status=aborted`,
  `reason=simulation_reset`, and the new `reset_epoch`. A reset with no active
  route publishes no fake terminal.
- New goals advance the request fence and synchronously clear the previous
  tracker/navigation state before the new pending goal or route context is
  visible. Existing preemption semantics remain non-terminal.
- Generation checks continue to discard late route and navigation results, so
  they cannot restore state or publish a second terminal after reset.
- A shared `RLock` now makes each route/action callback generation check and
  local state commit atomic with reset, preemption, and the progress timer.
  Completed futures are consumed outside the lock; publish, cancel, action,
  and service operations are also outside the state lock. Stale accepted
  `ComputeRoute` and `NavigateToPose` handles are cancelled.
- `desired_graph` is now the local authority for one serialized
  `SetRouteGraph` transaction. Reset and preemption force desired GVG even if
  the local graph is already GVG or an older cognitive/rebuild request is in
  flight. Every stale response is read; a stale success never commits local
  graph state and triggers a generation-fenced GVG compensation. A failed or
  incoherent graph blocks fresh route preparation and publishes a
  `LAST_KNOWN_GOOD` status.
- Structural rebuild requests capture request/reset/structural/desired tokens.
  A reset or fresh active goal makes a late rebuild stale, so its graph, map,
  support, and GVG state cannot overwrite the fresh epoch; a stale success is
  reconciled to the current desired graph.
- Reset publishes an explicit empty transient-local
  `/bio_nav/runtime_edge_states` snapshot using the desired GVG identity after
  clearing the internal runtime-edge map. Publication failure is visible in
  logs and structural status.

## Validation actually run

- Source-first/no-cache focused route/reset tests:
  `test_route_core.py test_cognitive_graph_adapter.py`: **73 passed**.
- Source-first/no-cache complete `robot_route_planner/test`: **97 passed,
  1 skipped**. The skip is the existing Isaac benchmark import because `pxr`
  is unavailable; Isaac was outside this task.
- Source-first/no-cache pure ActivationGate and mode-contract tests:
  **44 passed**. No ROS launch/integration test was run.
- Changed-file `python3 -m py_compile`: PASS.
- Fresh isolated package build: PASS at
  `/tmp/bionav_route_concurrency.j12692`.
- Fresh isolated `colcon test`: **98 tests, 0 errors, 0 failures, 1 skipped**
  (`97 passed`).
- `git diff --check`: PASS.

Tests cover active reset state retirement and one cancellation, single Bool and
JSON terminals, pending-action late acceptance cancellation, ignored late
results, no old-request timer/context/progress/lookahead/goal-update/canonical
publication, no fake no-active terminal, transient edge/cache cleanup, normal
preemption tracker retirement, and a fresh post-reset goal generation. New
deterministic coverage uses a real `threading.Barrier` to interleave reset with
an accepted-handle callback, and covers stale cognitive success compensation,
rebuild -> reset -> fresh goal -> late success, explicit empty runtime
snapshot identity, and consumption of stale futures without old-request
publications.

## Remaining live risk and next step

The cited review blockers are **closed in code/build/unit evidence only**. This
amendment does not prove executor/action-server timing, DDS publication
ordering, ActivationGate cancel-all interaction, or command behavior across a
real active reset. StopGate live closure remains **pending authorized live
review** with ROS/Isaac/Nav2 authority. That review must verify an
active goal reset produces one abort terminal, old request IDs never reappear,
late accepted handles are cancelled, no stale NavigateToPose or nonzero command
crosses StopGate release, and a fresh post-reset goal is the only route that can
resume motion.
