# V6 core sensor physics-step publication amendment

## Scope

- Worktree/branch: permitted Module3 `cognitive-navigation` worktree.
- Parent: `d5cd8d9f89b06465e4080b5dee3378d25b29daf1`.
- Goal: remove the duplicate same-stamp Clock/JointState/IMU execution path
  without changing RGB-D, RTX LiDAR, navigation, covariance, Integration, or
  Module2 behavior.

## Implemented contract

- The core sensor graph has exactly one
  `isaacsim.core.nodes.OnPhysicsStep` trigger and no `OnPlaybackTick` node.
- Each physics step directly triggers Clock, JointState, and IMU read once.
  IMU publication is ordered after that read through
  `ReadIMU.outputs:execOut -> PublishIMU.inputs:execIn`.
- Every core exec target has exactly one incoming execution edge and the graph
  contains no duplicate connection.
- Clock, JointState, and IMU topics, frames, QoS, timestamps, target prims, and
  publisher counts are unchanged. The RTX LiDAR graph retains its independent
  playback/render publication path.

## Validation

- Changed Python compile: PASS.
- Focused graph contracts: `6 passed`.
- All static Isaac tests: `184 passed, 11 skipped`; the skips are the existing
  stage-composition cases requiring unavailable `pxr` bindings.
- `git diff --check`: PASS.

## Verdict and remaining risk

**PASS (implementation/static-test only).** No Isaac, ROS, Nav2, evidence, or
qualification campaign was launched. A live engineering retry still must
confirm one Clock/JointState/IMU message per simulation stamp. IMU covariance
remains zero/unspecified and must be measured during Estimated State
calibration; this amendment intentionally does not guess covariance values.
