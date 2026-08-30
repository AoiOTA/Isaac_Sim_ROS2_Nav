# V6 Module3 runbook

This runbook contains the current component commands. It does not authorize a
live run. Read `docs/CURRENT_STATE.md` and the authoritative Integration
`docs/CURRENT_STATE.md` before using it.

## 1. Clean shell and one underlay

Use the same values in every terminal. Source ROS/setup scripts before enabling
shell nounset.

```bash
export V6_DOMAIN=<fresh-domain-0-to-232>
export PATH="/usr/bin:/bin:${PATH}"
hash -r
test "$(readlink -f "$(command -v python3)")" = /usr/bin/python3

export BIO_NAV_INTEGRATION_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration
export BIO_NAV_MODULE3_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module3
export BIO_NAV_MODULE2_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module2
export BIO_NAV_MODULE2_CONDA_EXE=/home/lyb/miniconda3/bin/conda

export BIO_NAV_MODULE2_ASSET_ROOT=/mnt/nas_home/Bio_Nav_Data/experiments/assets/module2_v310_runtime_assets_v6_cleanup_d176498b
export BIO_NAV_ROUTE_PRIOR_SNAPSHOT=/mnt/nas_home/Bio_Nav_Data/experiments/assets/v6_kujiale_isaacgen_v1_sr_snapshot_d7db461171893953
export BIO_NAV_ROUTE_PRIOR_CATALOG=/mnt/nas_home/Bio_Nav_Data/experiments/assets/rivermark_a_srdr_tile_catalog_v1
export RIVERMARK_USD=/mnt/nas_home/Bio_Nav_Data/experiments/assets/rivermark_plaza_v6_final_20260829/rivermark.usd

source "${BIO_NAV_INTEGRATION_ROOT}/env/v6_pilot_setup.sh" "${V6_DOMAIN}"
cd "${BIO_NAV_MODULE3_ROOT}"
```

Before a data-producing attempt, select an absent NAS root and a short absent
socket path:

```bash
export BIO_NAV_RUN_ROOT=/mnt/nas_home/Bio_Nav_Data/experiments/pilots/<new-run-id>
export BIO_NAV_PHASE_F_SOCKET=/tmp/bio_nav_v6_${UID}_${V6_DOMAIN}.sock
test ! -e "${BIO_NAV_RUN_ROOT}"
test ! -e "${BIO_NAV_PHASE_F_SOCKET}"
```

Stop if NAS is unavailable. Do not redirect bags into the repository or `/tmp`.

## 2. Provenance and idle checks

Require all three repositories to be on `v6-compute-amcl-dual-odom`, tracked
clean, and `0/0` relative to upstream. Check live `ls-remote`, not only the
cached remote-tracking ref.

Source the environment above and verify:

```bash
ros2 pkg prefix bio_nav_interfaces
ros2 pkg prefix bio_nav_ros_bridge
ros2 pkg prefix robot_experiments
python3 - <<'PY'
import robot_experiments.experiment_runner as runner
print(runner.__file__)
PY
```

The first two paths must resolve under the canonical Integration worktree; the
last two must resolve under canonical Module3. No product owner or GPU compute
process may already be running.

## 3. Startup ordering

T1 must start before T2. Before T2, wait only for
`/wheel_odometry/reset`; do not wait for EKF `/set_pose`. The mixed EKF
advertises `/set_pose` only after Isaac bootstraps `/clock`, and Isaac's
bounded fail-closed startup reset discovers the required reset services.
T2-first therefore fails by design. T3 always starts last, after observable
readiness.

Rivermark uses a 240 s fail-closed activation timeout because cold USD/RTX
startup has exceeded 120 s. This is an upper bound, not a fixed sleep, and does
not relax any readiness predicate.

## 4. Indoor M3 stack

Terminal T1:

```bash
./scripts/run_v6_low_obstacle_phase_f_stack.sh M3 \
  --domain "${V6_DOMAIN}" \
  --run-dir "${BIO_NAV_RUN_ROOT}/runtime" \
  --socket "${BIO_NAV_PHASE_F_SOCKET}" \
  --module2-root "${BIO_NAV_MODULE2_ROOT}" \
  --module2-asset-root "${BIO_NAV_MODULE2_ASSET_ROOT}" \
  --enable-route-prior \
  --route-prior-snapshot "${BIO_NAV_ROUTE_PRIOR_SNAPSHOT}"
```

Terminal T2, after the ROS reset services exist:

```bash
./scripts/run_v6_kujiale_low_obstacles.sh --condition static isaac
```

Indoor keeps mixed Compute Odometry plus AMCL.

## 5. Outdoor Rivermark startup discriminator

At the current handoff, run only this startup discriminator. Do not start T3.

Terminal T1:

```bash
./scripts/run_v6_low_obstacle_phase_f_stack.sh M3 \
  --domain "${V6_DOMAIN}" \
  --run-dir "${BIO_NAV_RUN_ROOT}/runtime" \
  --socket "${BIO_NAV_PHASE_F_SOCKET}" \
  --scene rivermark \
  --condition static \
  --module2-root "${BIO_NAV_MODULE2_ROOT}" \
  --module2-asset-root "${BIO_NAV_MODULE2_ASSET_ROOT}" \
  --route-prior-catalog-root "${BIO_NAV_ROUTE_PRIOR_CATALOG}"
```

Terminal T2, after only `/wheel_odometry/reset` exists (do not wait for EKF
`/set_pose` before Isaac starts):

```bash
/usr/bin/python3 "${BIO_NAV_MODULE3_ROOT}/scripts/wait_for_empty_service.py" \
  --service /wheel_odometry/reset \
  --timeout 120
./scripts/run_v6_rivermark.sh isaac static --headless
```

The wrapper fixes the original map/route/regions, mixed Compute Odometry,
calibrated fixed `map -> odom`, Module2/GVG/RoutePrior M3 chain, RGB-D profile,
and Rivermark-only DLSS disable. Caller overrides are rejected.

The startup discriminator passes only when Isaac and Nav2 are READY, no DLSS
internal-upscale warning or GPU page fault/device-lost occurs, and live RGB,
depth, CameraInfo, scan, odom, GT, fixed TF, Module2, and catalog identities are
current. Hold a bounded stability window, then stop without an episode.

## 6. Outdoor static runner after startup PASS

Only after Section 5 passes on a separate fresh root may Terminal T3 run:

`run_experiment.sh` otherwise falls back to the Kujiale spawn manifest, which
does not define `rivermark_start`.

```bash
export ISAAC_NAV_SPAWN_POSES="${BIO_NAV_MODULE3_ROOT}/data/rivermark_demo/rivermark.spawn.yaml"
test -f "${ISAAC_NAV_SPAWN_POSES}"

./scripts/run_experiment.sh \
  ros2_ws/src/robot_experiments/config/final_rivermark_static.yaml \
  "${BIO_NAV_RUN_ROOT}/rep1" \
  nav2_profile:=v6_low_obstacle_isolation \
  nav2_config_file:="${BIO_NAV_MODULE3_ROOT}/ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml" \
  navigation_execution_backend:=route_guided \
  require_module2_planning_ready:=true \
  record_evidence:=true \
  record_bag:=true \
  clear_slam_localization_buffer:=false \
  reset_map_base_translation_tolerance_m:=0.15 \
  run_indices:=1 \
  resume:=false
```

Omit `experiment_arm`; the Phase-F stack already owns the M3 arm. For the
sufficient static Pilot, repeat index 1 in fresh `rep1`, `rep2`, and `rep3`
outputs on one stack, stopping after the first valid product failure or invalid
episode.

## 7. Readiness essentials

- exactly one `/odom` publisher and one `odom -> base_link` owner;
- indoor: AMCL alone owns `map -> odom`;
- outdoor: AMCL absent and `ideal_localization_tf` alone owns `map -> odom`;
- `/bio_nav/module1/odom` fresh with no Module1 TF;
- all required Nav2 lifecycle nodes active;
- ResetStopGate released for the current generation before dispatch;
- Module2 socket/READY/current 0.10 m depth config;
- current RoutePrior snapshot or Rivermark tile catalog;
- cognitive obstacle layer and critic active; raw depth voxel plugin absent;
- exactly the selected condition contract: low obstacle for static/appearance,
  or the LiDAR-visible G2 crossing obstacle for dynamic. Dynamic results do not
  validate sub-LiDAR obstacle perception.

Topic names alone are insufficient; inspect fresh messages, QoS, timestamps,
TF values, and actual process/package provenance.

## 8. Owned shutdown

Stop in reverse dispatch order: T3, T2, then T1. Signal only process groups
created by the current run. Stop the current domain daemon if owned, remove only
the current socket/runtime directory, and verify product processes, GPU compute
apps, and owned locks are gone. Never use global `pkill`.

Preserve NAS logs and bags for any failure or invalid attempt. Record invalid
operator/startup runs separately and do not count them as Pilot episodes.
