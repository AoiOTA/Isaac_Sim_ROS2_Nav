# V6 Isaac cold stage-readiness timeout repair

## Scope

- Goal: tolerate the first-run RTX shader/PSO/cache warm-up that can block one
  `app.update()` for several minutes, without weakening the bounded failure.
- Branch/worktree: `cognitive-navigation` at the permitted Module3 worktree.
- Starting HEAD: `ff7ab2724aba4095a0f334f7c50ae79690aeaee8`.
- Result commit: the commit containing this handoff.

## Implementation

- `simulation.stage_readiness_timeout_s` is a required positive configuration
  value, defaults to 420 seconds, supports the existing
  `ISAAC_NAV__SIMULATION__...` environment override, and is exposed as
  `--stage-readiness-timeout-s`.
- A matching, fully loaded context stage returns immediately on a warm start.
  After every blocking `app.update()` return, readiness is checked before the
  deadline, so a cold update that crosses the nominal deadline but returns a
  ready stage succeeds. A late update returning no ready stage fails bounded.
- Progress is logged at phase changes or 30-second intervals with a cold
  shader/cache explanation. Timeout errors report elapsed time, configured
  deadline, update count, and last observed state.
- The real `run()` path now reuses `_simulation_app_config(config)`, retaining
  the RTX motion/multi-tick settings while enforcing `multi_gpu=False`.

## Validation

Commands:

```bash
PYTHONPYCACHEPREFIX=/tmp/codex_v6_stage_timeout_pycache python3 -m py_compile \
  isaac_sim/src/config.py isaac_sim/src/stage/stage_loader.py \
  isaac_sim/src/stage/scene_composer.py isaac_sim/apps/navigation_sim.py \
  isaac_sim/tests/test_stage_loader_readiness.py \
  isaac_sim/tests/test_config.py isaac_sim/tests/test_camera_contracts.py

PYTHONDONTWRITEBYTECODE=1 pytest -q -p no:cacheprovider \
  isaac_sim/tests/test_stage_loader_readiness.py \
  isaac_sim/tests/test_config.py isaac_sim/tests/test_camera_contracts.py
```

- `py_compile`: PASS.
- Focused pytest: PASS, 29 tests.
- Fake-clock coverage includes instant warm readiness, one 135-second blocking
  update returning ready after a shorter deadline, readiness within 420
  seconds, bounded no-stage failure, config/env/CLI bounds, and the actual
  single-GPU SimulationApp construction path.
- `git diff --check`: PASS.

## Result and remaining risk

- Verdict: **PASS (implementation and code-level tests only)**.
- No Isaac, ROS, Nav2, navigation, evidence, or qualification run was started.
  The 420-second value and cold-start behavior still require the separately
  authorized next Isaac retry for runtime confirmation.
