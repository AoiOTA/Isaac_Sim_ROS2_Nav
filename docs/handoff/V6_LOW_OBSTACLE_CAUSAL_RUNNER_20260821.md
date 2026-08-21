# V6 low-obstacle M0--M3 causal runner handoff (2026-08-21)

## Scope and baseline

- Role: Module3 coder; no ROS, Isaac, Nav2, evidence, or qualification run.
- Worktree/branch: permitted Module3 `cognitive-navigation` worktree.
- Start HEAD: `734330df30b7d0838ecf6b9b6b761892976fb706`.
- Read-only baselines: Integration
  `730bd933d472f6e740826b053ad56e150ca0930c`; Module2
  `5039a68e322bbe3d1c937beca44d0c7d324dba1e`.
- Source ownership was limited to the new `robot_experiments` causal module,
  its YAML/test/setup entry, one wrapper, this handoff, and the experiment
  ledger. The formal runner and live runtime packages were not changed.

## Frozen engineering matrix

- One frozen low-obstacle identity for every row: scene
  `v6_kujiale_low_obstacles_static`, layout
  `kujiale_v6_low_obstacles_frozen_r1_20260820`, seed `8601`, G1 to G2,
  PRIMARY route, GVG graph, direct RGB-D Costmap disabled, and 180-second
  navigation timeout.
- Three counterbalanced repeats, twelve unique IDs, in exact order:
  `M0 M1 M2 M3`; `M3 M2 M1 M0`; `M1 M3 M0 M2`.
- M0 keeps Integration and the same `estimated_autonomy`/B5 localization
  contract but explicitly disables the Module2 UDS connection and bridge
  socket. M1 is UDS + shadow; M2 is active obstacle layer with critic off; M3
  is active layer + active critic.
- Every external-adapter plan reuses the V6 formal exactly-once sequence: wait
  for readiness, set seed, one reset call/event, bridge epoch +1,
  localization-seeded, and one RouteCoordinator G2 goal.
- Dispatcher topics have no `/ground_truth/*`. Passive GT odometry/collision
  evidence remains a separate evaluator input.

## Offline evaluator

- Requires one JSON evidence file per frozen run ID and rejects missing or
  mismatched identity, reset, arm, Costmap, path, trajectory, odometry,
  command, passive GT, or typed validation evidence.
- Computes synchronized scan/RGB-D invisibility counts, typed obstacle spatial
  matches, discrete path Hausdorff/length, passive minimum clearance,
  collision/success, and reroute direction.
- Checks M1/M0 isolation (`<=0.15 m`, `<=5%`), M2/M3 median clearance gain
  (`>=0.20 m`), three active repetitions without new collision, and consistent
  reroute direction.
- M3 may report offline-reconstructed critic scores, but without M3/M2
  trajectory separation it returns `AMBIGUOUS`; topic/status presence alone
  cannot produce causal PASS.
- Typed obstacle TTL violation is never accepted as causal evidence. A properly
  stopped, zero-layer-write, critic-not-applied row is retained as
  `STOP_FAIL_OPEN`; otherwise it is `INVALID`.
- Summary JSON includes explicit representative visualization inputs for
  map/Costmap/RGB-D/scan/typed-obstacle/path overlays. It does not draw or
  claim a result without recorded evidence.

## CLI and current result

- Entry point: `v6_low_obstacle_causal` with `manifest`, `plan`, `evaluate`,
  and `run` subcommands.
- `manifest` and `plan` are read-only. `evaluate` is pure offline evaluation.
  `run` returns `NOT_RUN` without external scene/reset/live adapters and does
  not implement or pretend to execute them.
- Current result: **PASS for implementation/build/unit contract only**;
  **ENGINEERING_CAUSAL_NOT_RUN** for the 12 live rows. No causal, navigation,
  or formal PASS is claimed.

## Validation

- `python3 -m py_compile` for the new module and test: PASS.
- `bash -n scripts/run_v6_low_obstacle_causal.sh`: PASS.
- Focused causal + existing V6 formal tests: `38 passed`.
- Isolated `robot_experiments` build: PASS, `1 package finished`, under
  `/tmp/v6_low_obstacle_causal_build.S2ysce`.
- Installed entry-point manifest inspection: `ENGINEERING_CAUSAL_NOT_RUN 12`.
- `git diff --check`: PASS before the handoff-only amendment.

## Remaining risk / next step

- An external live adapter still must bind the abstract arm switches to the
  current Integration and Module3 launch parameters, create the separate
  recorder streams, and populate all twelve evidence JSON files.
- No live on/off/on carryover, layer-cell behavior, critic effect, clearance,
  collision, or route outcome has been measured. The evaluator's synthetic
  unit fixtures test formulas and fail-closed semantics only.
- Run the 12-row engineering study only after the current PRIMARY smoke owns
  no frozen-copy processes, then evaluate the retained evidence and inspect
  the generated visualization-input manifest before any causal claim.
