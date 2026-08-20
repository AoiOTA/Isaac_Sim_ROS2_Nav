# V6 C2/C3/D3 Module3 consumer handoff

- Scope: Module3-only consumption of the Integration `e16929d` typed V6
  interfaces. Module3 remains the physical graph, Costmap, MPPI and Route
  Server authority; no second planner or `/cmd_vel` publisher was added.
- C2: `CognitiveObstacleLayer` is a real Nav2 Costmap plugin. `off` creates no
  subscription, `shadow` never merges its private map, and `active` clears its
  private FREE_SPACE layer each update, transforms `base_link` obstacles, and
  max-combines costs. Input gates cover schema/frame/stamp/TTL/sequence,
  reset/session/map/tile/graph/model identity, health/trust/rejection mask,
  reliability/OOD and all obstacle numeric fields. Soft costs are 1--80;
  lethal promotion requires the specified confidence/reliability/OOD/count,
  standard-deviation and low collision-height thresholds.
- C3: `CognitiveRiskCritic` is a registered Jazzy MPPI `CriticFunction` plugin.
  It gates paired obstacle and PlanningPrior generations and adds only
  nonnegative obstacle distance/overlap, direction-deviation, novelty and
  uncertainty scores. Missing/stale/OOD/unhealthy input and off/shadow modes
  score zero. It does not bypass MPPI, smoother, Collision Monitor or publish
  velocity commands.
- D3: RouteCoordinator consumes transient-local
  `CognitivePlaceGraphCandidate`, validates a rigid transform and inversely
  maps canvas geometry, rejects occupied nodes and all edges not proven
  `FEASIBLE`, and filters malformed/disconnected graphs. `shadow` only reports;
  `hybrid` retains GVG plus validated cognitive edges and Module3-feasible
  connectors; `primary` uses the validated cognitive graph. Every switch is
  exported by the existing support exporter and committed atomically through
  the single Route Server `SetRouteGraph`. Primary candidate/switch/route
  failure requests one whole-graph GVG fallback. The committed active graph is
  republished through the existing typed NavigationGraph with real edge IDs.
- Profiles: C4 disables direct depth/STVL writes in both Costmaps, loads C2 in
  both Costmaps and C3 in MPPI, and documents M0--M3 off/shadow/hybrid/primary
  combinations. Bringup passes `cognitive_graph_mode` to RouteCoordinator.
- Interface build: exact Integration source `e16929d2369c0d5fce7cbaa5e07dbc0465a901f0`
  was archived into `/tmp/bionav_v6_interfaces_e16929d.duLV99` and built there;
  the source repository checkout was not used.
- Validation: temporary interface build PASS; pure Jazzy plus temporary
  interface 14-package dependency build PASS; `bio_nav_fusion` 15 tests PASS
  including both plugin loaders; Route/profile/mode pytest `77 passed, 1
  skipped` (`pxr` unavailable); Python compilation and `git diff --check`
  PASS.
- Verdict: **PASS (implementation/build/unit and fixture validation only)**.
- Not run: Isaac, ROS graph runtime, Costmap byte comparison in a live Nav2
  process, MPPI runtime ranking, Route Server live switching/fallback,
  navigation closure, metrics, visual evidence or formal qualification.
