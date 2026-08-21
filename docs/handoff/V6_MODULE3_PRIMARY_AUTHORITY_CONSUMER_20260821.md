# V6 Module3 PRIMARY authority consumer amendment — 2026-08-21

## Scope and identity

- Worktree/branch: permitted Module3 `cognitive-navigation` worktree.
- Parent: `9c515892aeb76a2eef0363e91f6d870e99ecdd10`.
- Integration interface dependency: permitted Integration worktree at
  `9373d9d9155198f8c84ac2f888e2c76b7b9ebc03`.
- No ROS, Isaac Sim, Nav2, evidence, or qualification campaign was launched.

## Changes

- `cognitive_graph_adapter.py` accepts the legacy direct-fresh contract and
  separately validates `identity_revalidated_static_graph` provenance. The
  latter preserves the source timestamp/age (maximum 5 s), bounds validation
  and arrival freshness by their typed TTLs (maximum 0.5 s), requires a valid
  observation, and then applies the existing exact generation/physical graph,
  health, numeric topology, rigid-transform, and occupancy checks.
- PRIMARY/hybrid candidates with fewer than two nodes, no edges, or no
  external edge identity are classified as
  `cognitive_graph_immature_gvg_bootstrap`. Module3 retains GVG, does not call
  `SetRouteGraph`, emits no per-edge terminal acknowledgement, and continues
  accepting later candidates. Shadow validation remains observational and no
  longer reports an execution acceptance.
- PRIMARY/hybrid goal-prior waiting is explicitly 4.0 s (valid range 3.5--4.0
  s). Exact request/graph identity remains mandatory; an identity-valid empty
  prior falls back immediately, timeout falls back after the window, and late
  replies remain ignored. GVG/shadow retain the existing configured timeout.

## Validation

- Python compile: PASS for changed source and tests.
- Focused tests: `64 passed`.
- Full `robot_route_planner` source tests: `88 passed, 1 skipped`; the optional
  `pxr` benchmark test was skipped because USD Python bindings are absent from
  the unit-test environment.
- Clean isolated build against Integration `9373d9d...` interfaces: PASS at
  `/tmp/v6_m3_primary_cleanbuild.BErizZ` (`bio_nav_interfaces` and
  `robot_route_planner`).
- Isolated colcon test: `89 tests, 0 errors, 0 failures, 1 skipped`.
- `git diff --check`: PASS.

## Result and remaining work

**PASS (implementation/build/unit only).** Live PRIMARY graph switching and
the 2.8--4.0 s UDS/ROS timing behavior still require the planned engineering
run. The optional `pxr` benchmark remains unexecuted in this environment.
