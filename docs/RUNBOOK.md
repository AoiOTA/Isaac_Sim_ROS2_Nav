# V6 Module3 runbook

This runbook records the current component topology. It does not authorize a
live run by itself; use the current experiment plan and a new NAS run root.

## 1. Clean shell and one setup

Open fresh terminals. In every terminal, ensure `python3` resolves to the system
interpreter, select the explicit Module2 Conda executable, then source only the
paired Integration V6 setup. Do not activate Conda, stack old workspace
installs, or manually edit ROS path variables. Set the same values in every
terminal:

```bash
export V6_DOMAIN=150
export PATH="/usr/bin:/bin:${PATH}"
hash -r
test "$(readlink -f "$(command -v python3)")" = /usr/bin/python3
export BIO_NAV_MODULE2_CONDA_EXE=/home/lyb/miniconda3/bin/conda
export BIO_NAV_INTEGRATION_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/cleanup-v6-integration-convergence/bio_nav_integration
export BIO_NAV_MODULE3_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/cleanup-v6-module3-convergence/bio_nav_module3
export BIO_NAV_MODULE2_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/cleanup-v6-module2-convergence/bio_nav_module2
export BIO_NAV_MODULE2_ASSET_ROOT=/mnt/nas_home/Bio_Nav_Data/experiments/assets/module2_v310_runtime_assets_v6_cleanup_d176498b
export BIO_NAV_ROUTE_PRIOR_SNAPSHOT=/mnt/nas_home/Bio_Nav_Data/experiments/assets/v6_kujiale_isaacgen_v1_sr_snapshot_d7db461171893953
source "${BIO_NAV_INTEGRATION_ROOT}/env/v6_pilot_setup.sh" "${V6_DOMAIN}"
cd "${BIO_NAV_MODULE3_ROOT}"
```

The compatible checkout must pair the intended Integration, Module3, and
Module2 runtime. The setup exports the same value as `ROS_DOMAIN_ID`,
`ISAAC_NAV_EXPECTED_DOMAIN_ID`, `ISAAC_NAV__ROS2__DOMAIN_ID`,
`BIO_NAV_PHASE_B_DOMAIN_ID`, and `BIO_NAV_PHASE_F_DOMAIN_ID`; do not override
any of those variables per terminal.

Before a data-producing run, choose one unique `V6_DOMAIN` and one new NAS root:

```bash
export BIO_NAV_RUN_ROOT=/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_<run_id>
export BIO_NAV_PHASE_F_SOCKET=/tmp/bio_nav_phase_f_${UID}_${V6_DOMAIN}.sock
```

`BIO_NAV_RUN_ROOT` must not already contain another run. If NAS is unavailable,
stop; do not redirect bags or experiment data into the repository or `/tmp`.

## 2. Phase B exact-scene baseline

The wrapper fixes the original USD, `v6_kujiale_isaacgen_v1` occupancy/spawn/GVG,
mixed odometry, AMCL ownership, `stable` Nav2, and M0/GVG routing. Validate the
manifest without launching a runtime:

```bash
./scripts/run_v6_r5_phase_b_kujiale.sh \
  --run-root "${BIO_NAV_RUN_ROOT}" --domain "${V6_DOMAIN}" manifest
```

Start components in the order printed by the wrapper:

```bash
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${V6_DOMAIN}" ros
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${V6_DOMAIN}" isaac
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${V6_DOMAIN}" module1-shadow
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${V6_DOMAIN}" bridge
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${V6_DOMAIN}" record
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${V6_DOMAIN}" runner
```

Each command owns its terminal. The ROS component starts before Isaac because
Isaac waits for the required ROS reset service. Start the runner last, after
the observable readiness conditions are satisfied.

## 3. Current three-terminal Pilot composition

The current Integration semantics are fixed: T1 owns the stack, T2 owns Isaac,
and T3 owns the runner. This is the present adapter, not a reason to add another
runner.

The example below uses the static M3 arm with the frozen RoutePrior snapshot.
Select another authorized condition or arm only through the current experiment
plan.

Terminal T1 owns Module3 ROS, the Module2 server, and the Integration bridge.
Start from a clean shell and run the Section 1 setup with the shared
`V6_DOMAIN` before this command:

```bash
./scripts/run_v6_low_obstacle_phase_f_stack.sh M3 \
  --domain "${V6_DOMAIN}" \
  --run-dir "${BIO_NAV_RUN_ROOT}/stack/m3" \
  --socket "${BIO_NAV_PHASE_F_SOCKET}" \
  --module2-root "${BIO_NAV_MODULE2_ROOT}" \
  --module2-asset-root "${BIO_NAV_MODULE2_ASSET_ROOT}" \
  --enable-route-prior \
  --route-prior-snapshot "${BIO_NAV_ROUTE_PRIOR_SNAPSHOT}"
```

Terminal T2 owns Isaac. Start it from another clean shell after the same setup
and after T1 reports the required pre-Isaac reset service ready:

```bash
./scripts/run_v6_kujiale_low_obstacles.sh --condition static isaac
```

Terminal T3 owns `ExperimentRunner` and its evidence output. Start from a third
clean shell, run the same Section 1 setup, wait for T1 and T2 readiness, and use
the selected static or dynamic config. The current Pilot invocation includes
the observed hot-reset tolerance explicitly:

```bash
./scripts/run_experiment.sh \
  ros2_ws/src/robot_experiments/config/v6_pilot_kujiale_static_hotreset.yaml \
  "${BIO_NAV_RUN_ROOT}/runner" \
  nav2_profile:=v6_low_obstacle_isolation \
  nav2_config_file:="${BIO_NAV_MODULE3_ROOT}/ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml" \
  navigation_execution_backend:=route_guided \
  record_evidence:=true record_bag:=true \
  clear_slam_localization_buffer:=false \
  reset_map_base_translation_tolerance_m:=0.15 \
  run_indices:=1 resume:=false
```

The selected wrapper fixes `nav2_profile:=v6_low_obstacle_isolation`. Do not
override the Phase B map, spawn, GVG, localization owner, or mixed-odometry
substrate from these commands.

## 4. Independent current RViz view

From another clean shell, use the same setup and `V6_DOMAIN` as the active
stack, then start the existing navigation view independently:

```bash
./scripts/run_rviz.sh navigation
```

This display is optional and may be closed without stopping the run. It is not
a readiness condition and must not be used to publish a goal or initial pose.

## 5. Rivermark candidate bringup boundary

The existing `scripts/run_v6_rivermark.sh` exposes six component/condition
pairings: `isaac|ros` for each of `static`, `dynamic`, and `appearance`. They
are implementation-only selectors in this cleanup, have not been validated on
the current cleanup heads, and are not a README front door.

For the first separately authorized live use, select one matching condition,
bring its Isaac half to cold-ready, then start the ROS half and confirm AMCL,
EKF, Route, and PRIMARY readiness before sending a goal. The candidate external
asset/configuration must be explicitly frozen first; do not infer an asset from
a historical host-local path.

The appearance runner selects a profile and records its state; it does not
change scene geometry. Historical candidates used low-obstacle geometry for
Kujiale appearance and no low obstacle for Rivermark appearance, but neither
layout is frozen for a current appearance run.

## 6. Reset and episode boundary

Isaac `ResetServiceBridge` owns the `/simulation/reset` service and its reset
transaction. Phase B has one orchestrating episode caller, `v6_formal_episode`,
invoked through `run_v6_formal_episode.sh`; Phase F has one orchestrating caller,
`ExperimentRunner`. A run must use exactly one of those callers. The Isaac
transaction holds motion, restores the scene/spawn, runs required ROS reset
hooks, and emits the reset event. Do not call component reset services
independently during an owned episode and do not treat a process restart as a
reset receipt.

## 7. Owned shutdown

For the Phase F three-terminal composition, stop in reverse dispatch order:

1. T3: stop `ExperimentRunner` and allow its current output to close.
2. T2: stop Isaac after the runner has closed.
3. T1: stop the stack owner; it stops only its registered Module3,
   Integration, and Module2 process groups and removes its exact socket.

For the Phase B component layout, stop `runner`, `record`, `bridge`,
`module1-shadow`, `isaac`, then `ros`. Allow the recorder to close its bag before
the run root is evaluated.

Do not use global `pkill`, delete a socket with a live listener, or clean another
run's PID/runtime directory. If a wrapper reports that an owned child remains,
preserve the logs and diagnose that exact process identity.

## 8. Evidence boundary

Keep logs, bags, JSONL, images, and evaluator results inside the new NAS run
root. A successful startup, a focused test, or a historical campaign is not a
current live result. Record the three commit pins, domain, scene/condition,
arm, and output path with every engineering run.
