# V6 D3 cognitive route repair handoff

- Scope: Module3 `robot_route_planner` only. Base HEAD was
  `c05d86537f7a8a584e796b4d6346d26ed45829ec` on `cognitive-navigation`;
  result commit is the commit containing this handoff.
- Worktree: `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`.
- Goal: repair the D3 physical adapter, async route-generation guards, primary
  whole-GVG fail-open, and EdgePrior consumption-time TTL/identity checks.
- Changed files:
  - `ros2_ws/src/robot_route_planner/robot_route_planner/cognitive_graph_adapter.py`
  - `ros2_ws/src/robot_route_planner/robot_route_planner/ros_node.py`
  - `ros2_ws/src/robot_route_planner/test/test_cognitive_graph_adapter.py`
  - `ros2_ws/src/robot_route_planner/test/test_route_core.py`
- Candidate validation now binds the first session/tile/model identity only
  after freshness, identity, health, numeric, rigid-transform, endpoint,
  physical-feasibility and strong directed-connectivity checks all pass.
  Reset invalidates callback generations and clears session/tile/model/prior
  state before requesting GVG restoration.
- Edge polylines must connect their referenced source and target nodes within
  the map/sweep tolerance. Bidirectional geometry is reversed explicitly;
  directed candidates must be strongly connected. Hybrid connector insertion
  recomputes every node's degree and type.
- SetRouteGraph, DynamicEdges, ComputeRoute and NavigateToPose callbacks carry
  request/graph generation snapshots. Stale callbacks cannot consume a newer
  support mapping or install route/navigation state. Graph commits replace the
  graph and support mapping together, invalidate in-flight route callbacks and
  reroute the active goal.
- In `primary`, candidate/SetRouteGraph/DynamicEdges/ComputeRoute/NavigateToPose
  rejection or failure requests at most one whole-GVG fallback. A fallback
  reroutes the same goal and locks that goal out of later cognitive switching.
- EdgePrior TTL plus request/graph/model identity is rechecked immediately
  before DynamicEdges cost construction. Stale, OOD, untrusted, rejected or
  identity-mismatched input contributes no learned cost; geometry/runtime
  routing continues.

## Validation

- `python3 -m py_compile robot_route_planner/*.py test/*.py`: PASS.
- Focused pytest (`test_cognitive_graph_adapter.py test_route_core.py`):
  `40 passed` in 0.25 s.
- Full package pytest: `64 passed, 1 skipped` in 7.65 s. The skip is the
  existing optional `pxr` import.
- `colcon build --packages-select robot_route_planner --symlink-install
  --event-handlers console_direct+`: PASS, one package finished.
- No Isaac, ROS graph, Route Server, Nav2 navigation, visual evidence or formal
  qualification run was performed.

## Verdict and remaining risk

- Verdict: **PASS for static implementation, focused unit tests and minimal
  package build only**.
- Remaining primary risk: live Route Server service ordering/cancellation and
  the exact Nav2 action timing around a mid-goal fallback remain unverified.
  The next useful check is one bounded primary-mode ROS smoke that forces each
  failure class and confirms the same goal reroutes on the GVG exactly once.
