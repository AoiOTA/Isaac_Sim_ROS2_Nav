# Final Rivermark outdoor navigation qualification

These rules apply only to this worktree.

- Build a runnable Rivermark outdoor research demo by minimally extending the
  A21 V3.10 Route Server/Nav2 path.
- Module3 uniquely owns the global occupancy map, footprint-feasible GVD/GVG,
  graph revisions, runtime edge state, Route Server, route tracking/lookahead,
  Nav2, collision safety, and `/cmd_vel`.
- Keep one continuous global `map` frame. Cognitive-region changes only alter
  `T_map_canvas` and Module2 context; they never reset TF, localization, Nav2,
  the global graph, or the active navigation action.
- Use only real collision-derived occupancy and real feasible GVG edges.
  Module2 priors are additive and fail-open; lethal/BLOCKED state wins.
- Reuse existing A21 launch, graph, runtime-edge, Smac, MPPI, Collision Monitor,
  Isaac robot, sensors, and dynamic-actor code. Avoid unrelated refactors and
  production/qualification/governance machinery.
- Develop only on `codex/final-outdoor-navigation` in this worktree. Treat
  every Attempt30/31/32 worktree and its raw, STOP, diagnostic, or accepted
  evidence as read-only.
- The paired Integration worktree is
  `/home/lyb/Workspace/Bio_Nav/worktrees/integration/final-indoor-outdoor-navigation`.
- The user authorized a fresh clean-revision Rivermark campaign with enhanced
  static obstacles, stronger dynamic interactions, and static/dynamic/
  appearance 3 x 20.  New evidence must use a new campaign identity and must
  never be mixed with Attempt31 rows.
- The user also authorized the remaining V4 query campaigns after the 3 x 20
  closeout.  Derive the runnable set from the frozen V4 matrix and preserve
  every earlier STOP or completed campaign unchanged.
- Fail-stop on collision, incomplete evidence, invalid threat/interactions,
  stale runtime ownership, or cleanup failure.  Do not relax acceptance gates
  during a running campaign.
- `/home/lyb/Rivermark/rivermark.usd` is the source asset. Derived maps,
  overlays, and demo configs belong in this worktree or its run directory.
- Do not start a second Isaac Sim while an unrelated A21 run is active.
- Keep changes small, add focused tests, and stage explicit paths only.
