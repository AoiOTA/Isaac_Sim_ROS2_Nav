# V6 Stage 1 prior and legacy Ideal wrapper fix

- Scope: reject stale Module2 responses after a timeout or from an older
  refresh generation, and keep the two legacy navigation wrappers on the
  explicit Ideal localization backend.
- Branch/worktree: `cognitive-navigation` in
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`.
- Modified code: `robot_route_planner/ros_node.py`, the Rivermark and
  multiroute legacy launch wrappers, and focused route/launch tests.
- Route result: a response is accepted only while its bounded pending window
  is live and its request, graph, refresh timestamp, and established model
  identity match. Timeout consumes the window and clears learned costs; the
  next refresh creates a new bounded window.
- Launch result: both wrappers pass `localization_backend=ideal`; the deleted
  `use_posegraph_localization` argument is absent. Offline expansion selects
  `ideal_localization_tf`, not AMCL or its default Kujiale parameter file.
- Validation: focused source pytest `72 passed, 1 skipped` (optional `pxr`
  unavailable); pure Jazzy plus allowed Integration `local_setup.bash`
  dependency build `13 packages finished`; focused colcon tests
  `27 passed` for route and `21 passed` for bringup; new launch-test flake8 and
  `git diff --check` passed.
- Verdict: **PASS (implementation/build/unit and offline launch expansion)**.
- Not run: ROS/Isaac/Nav2 runtime, the requested 8-second no-Isaac launch
  smoke, spatial/visual evidence, and formal qualification.
- Known unrelated issue: the full suite has a pre-existing checkout-sensitive
  frozen-JSON absolute-path assertion. It was not run, edited, or used as
  evidence for this focused fix.
