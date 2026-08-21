# V6 Module3 cognitive graph feedback

## Scope

- Worktree: `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`
- Start revision: `e328e27b4c4dcedc4748b59d39ec36bf38535152`
- Interface underlay: permitted Integration revision
  `c1e411b6c579e4f07f72b9e768760ff1f13c2bb0`
- Result: implementation/build/unit **PASS**; no ROS, Isaac, Nav2, navigation,
  evidence, or qualification campaign was run.

## Implemented contract

- Publishes `CognitiveGraphValidationAck` on
  `/bio_nav/module3/cognitive_graph_validation_ack` and
  `CognitiveEdgeOutcome` on
  `/bio_nav/module3/cognitive_edge_outcome`.
- Preserves Module2 candidate generation identity and maps every cognitive
  `external_edge_id` onto its concrete Module3 edge ID, including hybrid graph
  renumbering and bidirectional edges.
- Physical rejection never binds the candidate identity. Shadow validation
  reports `physically_validated_shadow_not_selected` and never creates an
  execution outcome. Primary/hybrid validation is accepted only after a
  successful `SetRouteGraph` response.
- RouteTracker edge transitions publish one success for the crossed concrete
  edge. Intermediate lookahead action success publishes no outcome. Final
  success additionally requires the existing final-goal distance gate.
- Navigate rejection or terminal failure publishes one current-edge failure.
  DynamicEdges/ComputeRoute failures before a canonical edge exists do not
  fabricate per-edge evidence.
- A primary fallback first reports `reroute_applied=false`; only successful
  whole-GVG `SetRouteGraph` completion emits a later event with a strictly
  increasing `reroute_revision` and `reroute_applied=true`.
- Reset clears active/pending provenance, event sequences, terminal dedupe, and
  reroute state. Old async graph-switch generations remain ignored.

## Validation

- `python3 -m py_compile` on both changed Python modules: PASS.
- Owned focused pytest with ROS Jazzy and Integration interface underlay:
  `46 passed`.
- Independent temporary build root:
  `/tmp/v6_module3_graph_feedback.H2jtx5`.
- `colcon build --packages-select robot_route_planner`: PASS.
- `colcon test --packages-select robot_route_planner`: `70 passed, 1 skipped`;
  the sole skip is the existing optional `pxr` import.
- `colcon test-result --test-result-base .../build --verbose`:
  `71 tests, 0 errors, 0 failures, 1 skipped`.
- `git diff --check`: PASS.

## Remaining risk and next step

This is code-level evidence only. A fresh integration run must verify that
Integration accepts the typed events for the live recurrent session/reset
epoch, that selected graph IDs/revisions match the live navigation graph, and
that real route traversal produces causal validation/outcome updates. Do not
treat this handoff as navigation or qualification evidence.
