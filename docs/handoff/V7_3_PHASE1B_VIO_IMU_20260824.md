# V7.3 Phase 1B VIO IMU handoff (2026-08-24)

## Result and boundary

- Worktree: `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`
  on `cognitive-navigation`; this convergence amendment started from
  `4232d43f1fed14aca1ebeda8c6a633763011c665`.
- Explicit `stereo_vio` now selects 120 Hz physics and 60 Hz rendering. All
  other Camera profiles retain 60/60 Hz. SimulationApp `minFrameRate` remains
  60, motion assist advances once per render update with `dt=1/60`, and the
  ready line reports both rates.
- The VIO graph has one physical IMU reader at the 120 Hz physics cadence.
  Its `execOut` drives both `/imu/vio_raw` and `/imu/data_raw`, while the same
  physics step drives `/clock`, `/joint_states`, and the reader. Both IMU
  publishers share linAcc, angVel, orientation, sensorTime, frame `imu_link`,
  and SensorData QoS. The observed wheel input path therefore remains 120 Hz;
  EKF `/odom` remains 50 Hz.
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

## Next bounded motion smoke

Use a freshly checked empty domain, keep the committed 4 MiB Fast DDS profile
active for producer and subscriber, and run the shortest motion interval that
actually overlaps the observer. Check that:

1. both raw and calibrated IMU streams remain finite, monotonic, and aligned;
2. nonzero angular velocity and wheel/joint motion are captured;
3. `/odom` remains finite at 50 Hz and reacts consistently to the motion;
4. the robot stops after the bounded command.

The accepted stationary cadence is 120 Hz for clock, both IMU paths, joints,
and wheel input; 20 Hz for stereo; and 50 Hz for EKF `/odom`. Phase 1C should
use actual cuVSLAM tracker/gap behavior to decide whether the roughly 0.64 RTF
is a blocking load problem. Do not reintroduce a cadence counter first.

## Cadence first-error amendment

- Live run
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1b_camera_imu_20260824T111210Z/run_summary.json`
  was an **ENGINEERING FAIL**: clock/VIO were 120 Hz, stereo was 20 Hz, and
  odometry was 50 Hz, but legacy raw/calibrated IMU, joints, and wheel odometry
  were all 120 Hz instead of 60 Hz. RTF was `0.631315`. Fields, gravity,
  stereo, and ownership passed; the motion interval did not overlap the probe.
- RCA: source inspection found no direct execution bypass and graph
  materialization preserved the requested values/connections. In an Isaac
  6.0.1 headless OGN probe, 22 physics events produced 22 downstream events
  for `IsaacSimulationGate` step values 0, 1, 2, and 3. That gate therefore did
  not decimate this direct `OnPhysicsStep` ON_DEMAND path.
- Amendment: the VIO graph removes both simulation gates and their step values.
  One native `omni.graph.action.OnPlaybackTick` is now the sole execution
  source for legacy IMU and joint publication at the configured 60 Hz render
  cadence. `/clock`, IMU reading, and VIO IMU remain on the 120 Hz physics
  path; both IMU publishers still share the same reader data and timestamps.
  The non-VIO 60/60 graph is unchanged.
- Static validation passed: source-first/no-cache graph and camera tests
  reported `27 passed`; Python compilation passed. No live run was performed
  for this amendment. Actual 60/120 rates, strict legacy/VIO stamp-subset
  behavior, and RTF remain for the next same-stack cadence + RTF live run.

## Cadence convergence decision (supersedes the 60 Hz target)

- Simulation-gate run:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1b_camera_imu_20260824T111210Z/run_summary.json`
  on `f90ff0ad` was an **ENGINEERING FAIL against the former 60 Hz
  contract**. Clock, both IMU paths, joints, and wheel odometry were 120 Hz;
  stereo was 20 Hz and EKF `/odom` was 50 Hz. RTF was `0.631315`.
- Playback-tick run:
  `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v73_phase1b_cadence_fix_20260824T115711Z/run_summary.json`
  on `4232d43f1fed14aca1ebeda8c6a633763011c665` failed the same former
  contract. Clock was 120 Hz; raw/calibrated VIO and legacy IMU were
  `119.99998` Hz; joints and wheel odometry were 120 Hz; stereo was 20 Hz;
  EKF `/odom` was `50.0033` Hz. Overall RTF was `0.640761`.
- Both runs retained the useful interface results: camera pairing, IMU shared
  fields/calibration, stationary gravity, publisher ownership, and run-scoped
  UDP receive-buffer counters passed. Both attempted native gate sources still
  executed legacy publishers at physics cadence, so neither demonstrated a
  real 60 Hz legacy path.
- After the user asked why legacy inputs must be reduced to 60 Hz, master
  explicitly accepted 120 Hz for clock, VIO IMU, legacy IMU, joint states,
  and wheel input under `stereo_vio`. There is no observed consumer failure
  that justifies a third custom counter. The graph therefore removes the
  unused 60 Hz parameters, cadence validation, and playback-tick split. Other
  profiles remain naturally 60 Hz because their physics/render timing stays
  60/60.
- `RTF >= 0.8` was a plan recommendation, not a hard boundary. The observed
  roughly `0.64` RTF is retained as a warning; Phase 1C actual cuVSLAM
  tracker/gap behavior determines whether load is blocking.
- Convergence static validation passed: source-first/no-cache graph and camera
  tests reported `24 passed`; changed Python compilation and diff check passed.
- Neither live observer overlapped effective motion, so nonzero-motion IMU,
  wheel, and odometry behavior is still unverified. The next action is the
  short motion smoke above. This convergence is not a Phase 1B full PASS and
  is not formal qualification. No live run was performed for this code
  amendment.
