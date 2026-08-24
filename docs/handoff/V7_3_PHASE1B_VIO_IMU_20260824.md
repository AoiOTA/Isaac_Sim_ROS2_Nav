# V7.3 Phase 1B VIO IMU handoff (2026-08-24)

## Result and boundary

- Worktree: `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`
  on `cognitive-navigation`, parent `bba3d3822d2cb443c9481606ba080f36743d64fc`.
- Explicit `stereo_vio` now selects 120 Hz physics and 60 Hz rendering. All
  other Camera profiles retain 60/60 Hz. SimulationApp `minFrameRate` remains
  60, motion assist advances once per render update with `dt=1/60`, and the
  ready line reports both rates.
- The VIO graph has one physical IMU reader at the 120 Hz physics cadence.
  `/imu/vio_raw` publishes directly from that reader. Step-2 gates preserve
  `/imu/data_raw` and `/joint_states` at 60 Hz; `/clock` stays ungated at
  120 Hz. Both IMU publishers share linAcc, angVel, orientation, sensorTime,
  frame `imu_link`, and SensorData QoS.
- `ImuCalibrationNode` now parameterizes only its input/output topics while
  preserving calibration, stamp rejection/reset, copying, and covariance
  behavior. The default remains `/imu/data_raw -> /imu/data`.
  `imu_vio_calibration.yaml` reuses the same yaw parameters for
  `/imu/vio_raw -> /imu/vio`.
- `ros_stack.launch.py` adds only a default-false `vio_imu_enabled` argument.
  In realistic/estimated mode, true adds `imu_vio_calibrator`; false leaves the
  single legacy calibrator. EKF still consumes only `/imu/data`.
- No Isaac, ROS launch, cuVSLAM, Nav2, recorder, or live experiment ran. This
  is code/static/build evidence, not proof of actual rates, stamps, gravity,
  yaw behavior, RTF, or formal qualification.

## Validation

Source-first focused tests with the ROS Jazzy base sourced and pytest cache
disabled:

```bash
PYTHONPATH="$PWD:$PWD/ros2_ws/src/robot_odometry:$PWD/ros2_ws/src/robot_bringup:$PWD/ros2_ws/src/robot_localization_config:$PYTHONPATH" \
  python3 -m pytest -p no:cacheprovider \
  isaac_sim/tests/test_camera_contracts.py \
  isaac_sim/tests/test_graph_contracts.py \
  isaac_sim/tests/test_skid_steer_motion_assist.py \
  ros2_ws/src/robot_odometry/test/test_imu_calibration.py \
  ros2_ws/src/robot_bringup/test/test_mode_contract.py -q
```

Result: `87 passed`; Python compilation and `git diff --check` also passed.

Isolated target build/test root:
`/tmp/v73_phase1b_targets.7FLxny`. `robot_odometry` and `robot_bringup` built
from this worktree, with only their three direct local test dependencies added
to the isolated prefix. The installed `robot_odometry` share contains the new
VIO YAML. Results:

- `robot_odometry`: `83 passed`;
- `robot_bringup`: `257 passed, 10 skipped` (retired Attempt21 profiles);
- total: `350 tests, 0 errors, 0 failures, 10 skipped`.

An earlier broad `--packages-up-to` attempt at
`/tmp/v73_phase1b_colcon.3y9ewe` failed in unrelated `bio_nav_fusion` because
its external `bio_nav_interfaces` header was unavailable. Target packages had
not been processed in that attempt; no source change was made for it.

## Next bounded camera + IMU smoke

Use a freshly checked empty domain no higher than 232 (for example 231 if it
is empty), keep the committed 4 MiB Fast DDS profile active for producer and
subscriber, and start only:

1. Isaac with explicit `--camera-profile stereo_vio`;
2. the estimated ROS core with `vio_imu_enabled:=true`;
3. a bounded camera/IMU observer, without cuVSLAM or Nav2 first.

Measure actual rates for `/clock`, `/imu/vio_raw`, `/imu/vio`,
`/imu/data_raw`, `/imu/data`, `/joint_states`, and all five stereo topics.
Check unique monotonic stamps, raw/calibrated one-to-one stamp preservation,
VIO/legacy shared-sample alignment, gravity/frame/field invariants, yaw-scale
behavior, camera pairing/drop gaps, RTF, GPU, and run-scoped UDP counters.
Expected behavior is 120 Hz clock/VIO IMU, 60 Hz legacy IMU/joints, and 20 Hz
stereo, but none of those rates is claimed until this smoke passes.
