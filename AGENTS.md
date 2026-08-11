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
- Keep one direction-aware MPPI controller. Smac Lattice is the primary planner;
  Smac 2D is the baseline/fallback profile. The original mission goal checker is
  the only navigation-success authority.
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
  Store shared evidence in Integration `docs/evidence/attempt30_a21/`.
- Do not mark a phase complete when tests pass but the visual has wall hugging,
  obstacle intrusion, breaks/spurs, bad junctions/endpoints, isolated branches,
  shortcuts, route jumps, wrong-corridor Smac, or direction errors. Diagnose and
  fix the owning GVD/GVG/graph/Route/guidance/Smac/MPPI layer; do not add special
  cases, recovery ladders, fallback chains, or portal-style hacks. Visual review
  never replaces automated free-space, connectivity, clearance, footprint,
  route-edge, lethal-cost, or planner-success validation.
