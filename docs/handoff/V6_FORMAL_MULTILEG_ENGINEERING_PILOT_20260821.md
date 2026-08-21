# V6 formal multi-leg engineering-pilot amendment (2026-08-21)

## Scope

- Worktree/branch: permitted Module3 `cognitive-navigation` worktree.
- Start HEAD: `5b6e7b8854151c385790801cd96b645ad8eaad68`.
- No ROS graph, Isaac, Nav2, pilot, evidence campaign, or qualification run.
- Source changes are limited to `robot_experiments/v6_formal.py`, its focused
  test and six candidate manifests, the existing wrapper, this handoff, and the
  experiment ledger.

## Amendment

- Replaced the final-goal-only draft contract with a five-leg mission. Kujiale
  dispatch order is `G2 -> G3 -> G4 -> G5 -> G1`; Rivermark is
  `G1 -> G2 -> G3 -> G4 -> G5`. Goals are the existing calibrated values and
  zero-distance legs are rejected at manifest load.
- Filled all candidate runtime asset bindings. Rivermark's unsupported
  posegraph remains the sole permitted null and is explicitly paired with
  `posegraph_required: false`. Every scene remains
  `scene_contract_frozen: false`, so formal dispatch remains fail-closed.
- Added explicit `--pilot --dispatch-pilot`. `--pilot` alone remains a lint;
  dispatch without `--pilot` is rejected. Engineering pilot output is labelled
  `ENGINEERING_PILOT` / `NOT_QUALIFIED` and is never a formal ledger.
- The ROS adapter retains exactly-one reset and GT-free subscriptions, then
  publishes each mission leg in order and waits for route progress/completion.
  Route result messages, graph/ack/outcome, layer/critic, collision, TF-related
  diagnostics, obstacle state, and appearance state are captured to JSONL.
- Dynamic missions call each declared
  `/experiment/obstacles/<G>/trigger` before its leg and `<G>/complete` after
  the attempt. Each `(group, action)` can be claimed once; unavailable,
  rejected, unknown, or timed-out responses STOP without retry. Periodic actor
  state snapshots preserve observed armed/moving/retired/parked states.
- Appearance pilots only request the preconfigured profile parameter and
  capture the existing status profile, light/material overrides, and applied
  counts. The runner performs no geometry mutation. Static and appearance
  missions declare no actor trigger groups.

## Candidate bindings

- Kujiale uses the requested `kujiale_0026_A_to_B_door_open.usd`,
  `warehouse_new.yaml`/posegraph/spawn, current GVG/Nav2 configuration, and the
  V6 frozen low-obstacle manifest. Dynamic case is
  `full_route_three_stage`; appearance keeps physical low obstacles enabled.
- Rivermark uses `/home/lyb/Rivermark/rivermark.usd`,
  `rivermark_selected.yaml`, `rivermark.spawn.yaml`, selected route graph and
  goals, plus the final static/dynamic configs. Dynamic case is
  `full_route_four_stage`; the audited appearance launch keeps physical
  obstacles disabled.

## Validation

- Python compile and wrapper shell syntax: PASS.
- Focused formal tests: `42 passed`.
- Formal + V6 causal + package regression: `98 passed`.
- Isolated `robot_experiments` build: PASS, `1 package finished`, at
  `/tmp/v6_formal_multileg_build.a5pQGC`.
- `git diff --check`: PASS.

Verdict: **PASS (implementation, unit/regression, and isolated build only)**.

## Remaining risk

- No live engineering pilot has run. Endpoint types, RouteCoordinator timing,
  actor service/state behavior, appearance application, and JSONL content still
  require a live pilot review.
- Candidate assets are deliberately not frozen. A successful engineering pilot
  and review are prerequisites to freezing any scene and starting the 120-run
  formal matrix.
