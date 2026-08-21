# V6 RouteCoordinator reset retirement handoff

Date: 2026-08-22

## Scope and verdict

- Worktree: permitted Module3 worktree
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`.
- Branch/start: `cognitive-navigation` at
  `0a0fdf8adccaf95bfdf4d933993fa926ba6f3be4`.
- Fixed main ancestry: `22d66470c4b903349b2467dc876490bbebfc0083`.
- Goal: retire every RouteCoordinator-owned old-epoch route/action intent when
  `/simulation/reset_event` arrives, and prevent normal goal preemption from
  exposing an old tracker under a new request ID.
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

## Validation actually run

- Source-first/no-cache focused route/reset tests:
  `test_route_core.py test_cognitive_graph_adapter.py`: **69 passed**.
- Source-first/no-cache complete `robot_route_planner/test`: **93 passed,
  1 skipped**. The skip is the existing Isaac benchmark import because `pxr`
  is unavailable; Isaac was outside this task.
- Changed-file `python3 -m py_compile`: PASS.
- Fresh isolated package build: PASS at
  `/tmp/bionav_route_reset.0m4Bm7`.
- Fresh isolated `colcon test`: **94 tests, 0 errors, 0 failures, 1 skipped**
  (`93 passed`).
- `git diff --check`: PASS.

Tests cover active reset state retirement and one cancellation, single Bool and
JSON terminals, pending-action late acceptance cancellation, ignored late
results, no old-request timer/context/progress/lookahead/goal-update/canonical
publication, no fake no-active terminal, transient edge/cache cleanup, normal
preemption tracker retirement, and a fresh post-reset goal generation.

## Remaining live risk and next step

This amendment does not prove executor/action-server timing, DDS publication
ordering, ActivationGate cancel-all interaction, or command behavior across a
real active reset. StopGate live closure remains **BLOCKED pending the combined
read-only reviewer** with ROS/Isaac/Nav2 authority. That review must verify an
active goal reset produces one abort terminal, old request IDs never reappear,
late accepted handles are cancelled, no stale NavigateToPose or nonzero command
crosses StopGate release, and a fresh post-reset goal is the only route that can
resume motion.
