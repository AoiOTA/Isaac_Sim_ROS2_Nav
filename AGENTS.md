# Attempt30 / A21 engineering rules

These rules apply to this worktree. The objective is a runnable end-to-end A21
navigation stack with direct engineering feedback.

- Module3 uniquely owns the occupancy map, GVD/GVG graph, stable graph IDs,
  footprint feasibility, runtime edge state, Route Server integration, route
  tracking/lookahead, Smac planning, MPPI, costmaps, collision safety, and
  `/cmd_vel`.
- A21 numeric behavior is loaded from the Integration worktree's canonical
  `ros2_ws/src/bio_nav_ros_bridge/config/engineering_defaults.yaml`. Every value
  is an adjustable initial engineering default, not an API contract, frozen
  threshold, qualification gate, or safety certification. Do not scatter new
  A21 constants through source or Nav2 overlays.
- Use official Nav2 Route Server for graph search. Because its edges are straight
  endpoint segments, split canonical curved polylines into support segments and
  track the canonical polyline for lookahead.
- Keep one direction-aware MPPI controller. Smac 2D is the accepted primary
  planner; Smac Lattice remains a diagnostic alternative. The original mission
  goal checker is the only navigation-success authority.
- Temporary obstacles change runtime edge state; only persistent structural-map
  updates rebuild the graph. Live costmap observations must not rewrite the
  structural graph.
- Do not add A19 portal/tube/crossing ownership, hard corridors, preregistration,
  hash receipts, formal seed gates, or defensive amendment layers.
- Preserve Attempt25--29 worktrees/data and stage explicit source/config/test
  paths only.
- Every spatial/geometry/topology phase must export context-rich PNG overlays and
  be visually inspected together with automated checks. Cover occupancy/free
  space, clearance, raw/thinned/pruned GVD, GVG junctions/endpoints/edges,
  footprint feasibility, graph alignment, Route Server route, projection,
  lookahead, Smac path, blocked-edge reroute, and structural rebuild as relevant.
  Store new shared evidence in Integration
  `docs/evidence/attempt30_a21_v310/`; the old `attempt30_a21` evidence is
  immutable history.
- Do not mark a phase complete when tests pass but the visual has wall hugging,
  obstacle intrusion, breaks/spurs, bad junctions/endpoints, isolated branches,
  shortcuts, route jumps, wrong-corridor Smac, or direction errors. Diagnose and
  fix the owning GVD/GVG/graph/Route/guidance/Smac/MPPI layer; do not add special
  cases, recovery ladders, fallback chains, or portal-style hacks. Visual review
  never replaces automated free-space, connectivity, clearance, footprint,
  route-edge, lethal-cost, or planner-success validation.

## V3.10 execution scope

- This worktree is `codex/attempt30-a21-v310-srdr-rviz-navigation`, created
  from exact Module3 commit `4caae47fabf8f09ed4274756e8a82ee48accb747`.
- The paired Integration worktree is
  `/home/lyb/Workspace/Bio_Nav/worktrees/integration/attempt30-a21-v310-srdr-rviz`
  at base commit `b375d9049d2c99ac989fd966fc7bfbd3733663bf`.
- Module3 publishes footprint-swept reachable cognitive states and verified
  directed transitions. It never performs Module2 cognitive learning or
  materializes `M_SR`/`M_DR`.
- Module2 V3.10 is additive and fail-open. A missing, stale, nonfinite,
  timed-out, or revision-mismatched prior contributes zero; physical
  infeasibility and BLOCKED edges remain authoritative.
- A guidance experiment may only rank edges already present and feasible in
  the current NavigationGraph. Preserve the real 3 -> 48 GVG cycle and never
  synthesize a shortcut or fake alternative.
- Use ROS domain 151 and lock
  `/tmp/isaac_sim_ros2_nav_1000_a21_v310_q151`. Preserve all other worktrees,
  branches, and run data.
