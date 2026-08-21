# V6 flat Estimated State calibration amendment

Date: 2026-08-21

## Scope and result

- Worktree/branch/start: permitted Module3 `cognitive-navigation` worktree at
  `df0b985a0a13fa4aa48dd396f8f5d37f417c4ae1`.
- Result: **PASS (implementation/build/unit only)**.
- Runtime state: **CALIBRATION_NOT_RUN**. No Isaac, ROS, Nav2, tuning run,
  RF2O promotion, PRIMARY replay, or qualification was performed.

## Flat calibration environment

- Reuses the official local Grid asset at
  `/home/lyb/isaacsim_assets/Assets/Isaac/6.0/Isaac/Environments/Grid/default_environment.usd`.
  Read-only USD inspection found a 100 x 100 m flat grid and
  `/World/GroundPlane/CollisionPlane`. No duplicate ground or overlay USD is
  tracked.
- The existing schema-v2 stationary-obstacle runtime authors four 1.0 m-high,
  0.2 m-thick collision walls with inner faces at `+/-9.9 m`, plus three
  asymmetric full-LiDAR-height localization features near `(-6,-4)`, `(6,5)`,
  and `(-3,7)`. There are no moving/low obstacles or complex materials.
- `mapping_start` is USD `[0,0,0.0635], 0 deg` and map `[0,0], 0 deg`, bound to
  map version `flat20_v1`.
- The deterministic generator produces a `404 x 404`, `0.05 m` occupancy grid
  at origin `[-10.1,-10.1,0]` from the exact wall/feature rectangles. Static
  tests compare the tracked PGM byte-for-byte with regenerated output and keep
  the 3 m/S envelopes more than 1.5 m from all collision rectangles.
- Supplemental revision
  `v6_estimated_calibration_flat20_supplement_r1` explicitly excludes prior
  narrow-indoor calibration evidence, leaves PRIMARY evidence unclassified,
  and keeps Rivermark rows separate. Existing evidence files were not changed.

## Evaluator and fail-closed contracts

- Passive evaluator streams are independent: fused `/odom`, raw
  `/wheel/odom`, RF2O `/lidar/odom`, raw `/imu/data`, `/amcl_pose`, and
  evaluator-only `/ground_truth/odom`. IMU `angular_velocity.z` is integrated
  trapezoidally and labelled yaw-only; wheel and RF2O retain full trajectory
  metrics. NIS remains `NOT_AVAILABLE`; NEES/covariance remain diagnostic.
- Frequency gates are stream-specific: `/odom >=30 Hz`, RF2O `>=15 Hz`, and
  AMCL `>=1 Hz`, so normal 2 Hz AMCL does not fail. RF2O promotion never uses
  fused `/odom` as a proxy.
- Linear/yaw scale is reported only with at least `0.5 m`/`0.5 rad` of matched
  GT motion. Promotion checks are primitive-specific: straight scale plus
  longitudinal/lateral endpoint, rotation yaw scale plus yaw/position closure,
  S-route ATE/yaw, and separate Rivermark absolute pose checks.
- Every evaluated episode requires a matching `dispatcher_result.json` with
  `status=SUCCEEDED` and `collision_detected=false`. Missing dispatcher results
  are `NOT_RUN`; collisions or malformed/failed results are `INVALID`; neither
  can increment threshold PASS.
- The future Rivermark adapter must observe both
  `/bio_nav/canonical_route` and `/bio_nav/route_progress` for the current
  request before starting its route timeout. Acceptance is bounded to 15 s,
  then fails without resend. The current `run` command still records
  `NOT_RUN/runtime_adapter_not_implemented`.

## Validation

- Changed Python compilation: PASS.
- Focused calibration/metrics/motion/package regression: `67 passed`.
- Relevant flat-map/dynamic-obstacle/spawn/environment static tests:
  `52 passed`.
- Shell syntax and `git diff --check`: PASS.
- Isolated `robot_experiments` build: PASS at
  `/tmp/v6_flat20_robot_experiments.YT6lXC` (one existing underlay-override
  warning; build completed).
- Manifest smoke: 45 episodes, exact supplemental revision, external Grid
  asset, and all six passive topics PASS at
  `/tmp/v6_flat20_manifest.xuLCJH`.

## Remaining runtime work

Connect the real episode adapter, run off/shadow groups in this flat arena,
inspect wheel/IMU/RF2O scale, timing, yaw bias and covariance, tune only from
measured failures, then rerun shadow evaluation. Fused remains blocked until
all 15 shadow rows and the explicit promotion flag pass. Rivermark and PRIMARY
must be rerun after accepted localization parameters; no prior live result is
automatically promoted by this code-level amendment.
