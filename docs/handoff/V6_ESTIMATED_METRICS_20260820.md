# V6 estimated-state metrics and wheel stamp diagnostics

## Scope and hypothesis

- Worktree: `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`
- Branch: `cognitive-navigation`
- Starting HEAD: `783347f82977747fb9e03c8b08fa78bb02339253`
- Fixed local `refs/heads/main`: `22d66470c4b903349b2467dc876490bbebfc0083`
- Hypothesis: duplicate/backward `JointState` stamps can remain strictly rejected
  without warning floods, and estimated-state error can be measured offline from
  evaluator-only ground truth without changing control, TF, or estimation.

## Changes

- `robot_odometry/wheel_odometry_node.py`
  - preserves the existing stamp rejection and integration/output-stamp path;
  - counts accepted, duplicate, and backward samples;
  - logs the first duplicate/backward rejection and then every 1000 occurrences;
  - logs accepted counters every 1000 accepted samples;
  - does not clear rejection history after each accepted sample.
- `robot_experiments/estimated_state_metrics.py`
  - pure-Python timestamp diagnostics and covariance finite/coverage summary;
  - one-to-one nearest timestamp association with an explicit upper bound;
  - first-match SE(2) alignment and relative ATE/RPE in xy/yaw.
- `robot_experiments/estimated_state_evaluator.py`
  - subscribes to estimated `/odom`, estimated `/amcl_pose`, and evaluator-only
    `/ground_truth/odom`;
  - has no publisher, TF broadcaster, control output, or estimator input;
  - requires explicit `output_dir`, then periodically and at shutdown writes
    `estimated_state_metrics.json` and `estimated_state_matches.csv`.
- `robot_experiments/setup.py` installs the
  `estimated_state_evaluator` console entrypoint.
- Focused tests cover straight motion, turning, timestamp mismatch/upper bound,
  NaN/covariance reporting, duplicate warning throttling, and backward rejection.

## Operator entrypoint

After building and sourcing this worktree overlay:

```bash
ros2 run robot_experiments estimated_state_evaluator --ros-args \
  -p output_dir:=/absolute/path/to/estimated_metrics \
  -p max_time_delta_sec:=0.1 \
  -p report_period_sec:=5.0
```

The ground-truth topic is consumed only by this evaluator. The output is a
measurement artifact, not a control input or a qualification result.

## Validation

- `python3 -m py_compile ...`: PASS for both changed nodes/modules and focused
  tests.
- Focused pytest with this worktree first in `PYTHONPATH` and cache disabled:
  `19 passed in 0.30s`.
- `source /opt/ros/jazzy/setup.bash && colcon build --packages-select
  robot_odometry robot_experiments --symlink-install`: PASS, 2 packages built.
  Colcon reported the expected warning that both Python packages also exist in
  an underlay; the local overlay build itself completed successfully.

Result: **PASS (code-level and synthetic only)**.

## Remaining live work

- No ROS/DDS, Isaac Sim, 3 m straight, 360-degree turn, S-route, or Rivermark
  run was performed in this task.
- Covariance values were not tuned or calibrated. Live runs must establish
  sampling rates, matching tolerance, covariance behavior, and usable metric
  thresholds before these summaries support experimental conclusions.
