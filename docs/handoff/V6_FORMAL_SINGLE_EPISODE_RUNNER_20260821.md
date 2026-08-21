# V6 formal single-episode runner handoff (2026-08-21)

## Scope and baseline

- Role: Module3 coder; no ROS, Isaac, Nav2, evidence, or qualification run.
- Worktree/branch: permitted Module3 `cognitive-navigation` worktree.
- Start HEAD: `5d40626dbf3c8c29dfa577a7fb0b5c31ba43b61f`.
- Interface read baseline: permitted Integration
  `c5fe274949cc616a8eae4391f42098967476ba19`.
- Source changes are limited to `robot_experiments`, one wrapper, this handoff,
  and the experiment ledger.

## Implemented contract

- Added an independent `v6_formal_episode` entry point. It does not reuse the
  legacy Ground-Truth-consuming experiment runner.
- Dispatcher subscriptions contain no `/ground_truth/*`; GT evaluation remains
  owned by the passive `estimated_state_evaluator`/external recorder.
- Navigation dispatch publishes exactly one `PoseStamped` to
  `/bio_nav/route_goal` and waits for fresh canonical route, route progress,
  and `/bio_nav/route_goal_complete`. It has no NavigateToPose client.
- Reset is armed only after reset/localization/bridge/RouteCoordinator endpoints
  and estimated odom, AMCL, map, constraints, and navigation graph facts exist.
  Isaac seed/reset-pose/dynamic/appearance parameters are set before one and
  only one `/simulation/reset` Trigger call.
- Missing/unknown reset response is STOP with no retry. A second reset event is
  STOP. Goal dispatch additionally requires exactly one reset event, bridge
  epoch `baseline + 1`, a post-reset goal prior in that epoch, and a B5
  localization-seeded event.
- Capture schema includes cognitive graph candidate/validation, navigation
  graph, canonical route, progress, goal prior, layer/critic statuses, edge
  outcome, command, collision, and collision diagnostics. Module3 writes only
  the explicit single-episode JSONL; campaign/NAS layout remains Integration's
  responsibility.
- Formal dispatch is disabled by default and needs both a frozen manifest and
  explicit opt-in. Pilot mode never creates the requested output and reports
  `NOT_QUALIFIED`.

## Draft manifests

All six manifests have `scene_contract_frozen: false` and required null
placeholders for scene/reset/route plus dynamic or appearance assets where
applicable. Formal mode therefore rejects them before ROS initialization.

- Kujiale static: seeds `7201..7220`.
- Kujiale dynamic: seeds `7301..7320`, `v1..v5` four each.
- Kujiale appearance: seeds `7201..7220`; `dim_warm`, `dim_cool`,
  `bright_warm`, `bright_cool` five each.
- Rivermark static: seeds `19301..19320`.
- Rivermark dynamic: seeds `19401..19420`, `v1..v5` four each.
- Rivermark appearance: seeds `19501..19520`, the same four profiles five each.

Every manifest fixes `estimated_autonomy`, `active`, `L3`, `M3`, `primary`,
`structure_tf_source: isaac`, GT evaluator-only, and direct RGB-D Costmap off.

## Validation

- `python3 -m py_compile robot_experiments/v6_formal.py test/test_v6_formal.py`:
  PASS.
- `PYTHONPATH="$PWD" pytest -q -p no:cacheprovider test/test_v6_formal.py`:
  `26 passed`.
- `bash -n scripts/run_v6_formal_episode.sh`: PASS.
- `git diff --check`: PASS.
- Isolated `robot_experiments` colcon build: PASS at
  `/tmp/v6_formal_build.D1EGxV` (`1 package finished`).
- Installed pilot entry point, with the isolated site-packages first, reported
  `NOT_QUALIFIED`, `dispatch: false`, and the three expected missing Kujiale
  static placeholders.

Verdict: **PASS (implementation, unit contract, and isolated build only)**.

## Remaining risk / next step

- No manifest is frozen and no runtime scene/route/dynamic asset value is
  claimed. These values must be supplied and reviewed before formal dispatch.
- ROS endpoint discovery, exactly-once reset behavior on the live graph,
  RouteCoordinator completion, JSONL capture, and all 120 runs remain untested.
- When running an isolated install, put that install's Python site-packages
  before stale underlays; otherwise an older same-version `robot_experiments`
  distribution can shadow the new console entry point.
