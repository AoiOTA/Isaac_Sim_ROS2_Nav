# V6 core sensor on-demand execution amendment

## Scope

- Worktree/branch: permitted Module3 `cognitive-navigation` worktree.
- Parent: `c96f434811b5c698b1d5157d91e5af1a23500eed`.
- Goal: make the physics-step-driven core Sensors graph legal at Isaac graph
  materialization without changing its publication contract.

## Implemented contract

- `/World/Graphs/Sensors` now sets `GraphSpec.on_demand=True`, matching the
  existing physics-step-driven Control graph contract.
- Materialization selects
  `GRAPH_PIPELINE_STAGE_ONDEMAND` and does not set the execution evaluator.
- The single `OnPhysicsStep` node and all Clock, JointState, IMU nodes, edges,
  topics, frames, QoS, target prims, and timestamp sources are unchanged.
- The independent RTX LiDAR playback/render graph is unchanged.

## Validation

- Changed Python compile: PASS.
- Focused graph contracts: `7 passed`.
- All static Isaac tests: `186 passed, 11 skipped`; the skips are the existing
  stage-composition cases requiring unavailable `pxr` bindings.
- `git diff --check`: PASS.

## Verdict and remaining risk

**PASS (implementation/static-test only).** The spec and a mocked
materialization test prove the on-demand pipeline selection. No Isaac, ROS,
Nav2, evidence, or qualification campaign was launched. A fresh live retry is
still required to confirm that the original core Sensors graph materialization
fatal is gone and that Clock/JointState/IMU publish once per physics step.
