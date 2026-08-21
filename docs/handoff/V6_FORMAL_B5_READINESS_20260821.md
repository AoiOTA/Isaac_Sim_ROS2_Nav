# V6 formal B5 cognitive bootstrap readiness amendment

## Scope

- Worktree/branch: permitted Module3 `cognitive-navigation` worktree.
- Parent: `6071afc9727aa5a397bd918e66a442e05ffb2ee9`.
- Goal: align the V6 single-episode runner with active B5 localization without
  changing calibration, route, bringup, Isaac, Integration, or Module2 code.

## Implemented contract

- All six final manifests explicitly select
  `runtime.localization_seed_source: b5_cognitive`.
- Pre-reset readiness requires the reset service/listener roster, route-goal
  consumer, Bridge/B4/B5 publishers and diagnostics, and fresh clock, scan,
  estimated odom, map, constraints, and navigation graph.
- The epoch-zero quiet window requires zero PlanningPrior, localization
  candidate, `/initialpose`, and AMCL messages. It does not require a prior or
  AMCL pose before reset.
- The runner calls reset exactly once, then requires one reset event, Bridge
  physical epoch 1 with active bootstrap, B5 startup consensus, exactly one
  `/initialpose`, a strictly newer AMCL pose, and B5
  `normal/succeeded/succeeded` recovery diagnostics.
- Goal authorization additionally requires Bridge epoch 2 with a new recurrent
  session, a same-generation healthy/trusted active PlanningPrior, active Nav2,
  and both `map->odom` and `odom->base_*` TF observations.
- The active B5 path no longer subscribes to or depends on Isaac
  `/simulation/localization_seeded`. Second reset/initialpose, bad epochs or
  sessions, untrusted priors, seed-confirmation failures, and timeouts stop the
  episode before a goal.

## Validation

- `python3 -m py_compile .../v6_formal.py`: PASS.
- Focused formal tests: `53 passed`.
- Formal + localization causal + obstacle causal + calibration regression:
  `90 passed`.
- Isolated `robot_experiments` build: PASS at
  `/tmp/v6_formal_b5_build.5JYXDP`.
- `git diff --check`: PASS.

## Verdict and remaining risk

**PASS (implementation/build/unit only).** No ROS, Isaac, Nav2, evidence, or
qualification campaign was launched. DDS callback ordering and the real
Bridge epoch 0 -> 1 -> 2/B5 diagnostic timing still require the next authorized
engineering pilot before any formal campaign is dispatched.
