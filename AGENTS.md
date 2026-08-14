# Attempt31 / Rivermark outdoor research demo

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
- Develop only on `codex/attempt31-outdoor-nav-navigation` in this worktree.
  Do not edit, clean, stage, or launch from Attempt30 or historical worktrees.
- The paired Integration worktree is
  `/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt31-outdoor-nav`.
- `/home/lyb/Rivermark/rivermark.usd` is the source asset. Derived maps,
  overlays, and demo configs belong in this worktree or its run directory.
- Do not start a second Isaac Sim while an unrelated A21 run is active.
- Keep changes small, add focused tests, and stage explicit paths only.
