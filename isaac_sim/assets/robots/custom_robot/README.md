# Custom robot migration contract

This directory is the Stage-13 hand-off point for a real four-wheel skid-steer
asset. The repository intentionally does not contain a fabricated custom USD or
invented calibration values. `isaac_sim/configs/robots/custom_robot.yaml` is a
live-schema template: its `null` values and empty footprint make startup fail
until the measurements below are supplied.

## Files that may change

Migration must be expressible by replacing robot assets and parameter files,
without renaming ROS interfaces or editing SLAM/Nav2 launch topology:

1. Put the project-owned USD and all local dependencies in this directory.
2. Fill `isaac_sim/configs/robots/custom_robot.yaml` from measurements.
3. Copy the LiDAR, IMU, and camera YAML files and change only their custom USD
   prim paths and measured sensor settings. Keep `/lidar/points_raw`,
   `/imu/data`, `lidar_link`, and `imu_link` unchanged.
4. Supply a custom URDF/Xacro with the stable link/joint names listed below.
5. Supply custom Wheel Odom YAML (wheel radius, track width and joint lists)
   and a custom Nav2 YAML (footprints, Collision Monitor polygons, velocity and
   acceleration limits). The Nav2 node/plugin structure must stay unchanged.
6. Re-measure every USD spawn pose and Map pose; rebuild or revalidate the map.

The Isaac project template requires the real inputs explicitly, so it cannot
silently fall back to Jackal paths:

```bash
export ISAAC_NAV_PROJECT_CONFIG="$PWD/isaac_sim/configs/custom_robot.project.yaml"
export CUSTOM_ROBOT_USD="$PWD/isaac_sim/assets/robots/custom_robot/custom_robot.usda"
export CUSTOM_ROBOT_DEFAULT_PRIM="custom_robot"
export CUSTOM_ROBOT_LIDAR_CONFIG="$PWD/path/to/custom_lidar.yaml"
export CUSTOM_ROBOT_IMU_CONFIG="$PWD/path/to/custom_imu.yaml"
export CUSTOM_ROBOT_CAMERA_CONFIG="$PWD/path/to/custom_camera.yaml"
./scripts/run_isaac.sh --validate-only
```

The ROS-side files are selected at the stable bringup entrypoint and are
checked for existence before any node starts:

```bash
./scripts/run_ros.sh mapping \
  odometry_mode:=realistic \
  structure_tf_source:=rsp \
  robot_description_file:="$PWD/path/to/custom_robot.urdf.xacro" \
  wheel_odometry_params_file:="$PWD/path/to/custom_wheel_odometry.yaml" \
  nav2_params_file:="$PWD/path/to/custom_nav2.yaml"
```

Use the same three ROS file arguments for localization and navigation. Ideal
mode still uses the selected robot YAML's measured `static_transforms`; Realistic
`rsp` mode uses the custom Xacro. Both trees must agree numerically.

## Stable interface and USD requirements

The custom asset and Xacro must preserve these names:

- `base_link`, `lidar_link`, `imu_link`, `camera_link`;
- `camera_left_link`, `camera_right_link`, and both optical frames;
- `front_left_wheel_joint`, `front_right_wheel_joint`,
  `rear_left_wheel_joint`, `rear_right_wheel_joint`;
- `/cmd_vel`, `/joint_states`, `/imu/data`, `/lidar/points_raw`, `/scan`,
  `/wheel/odom`, `/odom`, and the existing Ground Truth/Reset interfaces;
- the navigation TF tree `map -> odom -> base_link`.

`--validate-only` checks the configurable `defaultPrim`, unresolved USD
dependencies, meter scale, Z-up axis, the expected single PhysicsScene, exactly
one Articulation Root in the robot subtree, four named Revolute Joints with
valid Body0/Body1 links, rigid wheel bodies with collision geometry, and fixed
sensor frames that are not independent rigid bodies. Runtime startup also
checks that the articulation exposes all four configured DOFs. Robot YAML
validation checks finite positive controller limits, valid physics settings,
the exact seven-edge static TF topology, and normalized quaternions.

## Evidence that cannot be supplied without the real robot

The following Stage-13 acceptance items remain blocked until the custom USD and
measured calibration are available:

- mass, center of mass, inertia, tire material, finite drive gains, collision
  fidelity, and one-wheel-at-a-time positive-direction trials;
- wheel radius, effective track width, wheelbase, chassis footprint, sensor
  extrinsics, self-filter bounds, and safe velocity/acceleration limits;
- Xacro-to-USD transform agreement and Realistic Wheel Odom/EKF tuning;
- calibrated fixed USD/Map spawn poses and maps generated for the new sensor
  geometry;
- end-to-end Ideal, Realistic, Ground Truth, Reset/fixed-spawn, Localization,
  and Navigation runs, including TF/topic uniqueness evidence.

Do not set placeholder values merely to make validation pass. Record the asset
revision, measurement method, map version, and runtime evidence with the commit
that enables the custom profile.
