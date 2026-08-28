# V6 Module3 runbook

This runbook records the current component topology. It does not authorize a
live run by itself; use the current experiment plan and a new NAS run root.

## 1. Clean shell and one setup

Open fresh terminals. In every terminal, source only the paired Integration V6
setup; do not stack old workspace installs or manually edit ROS path variables.

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module3
source ../bio_nav_integration/env/v6_pilot_setup.sh
```

Before a data-producing run, choose one unique ROS domain and one new NAS root:

```bash
export ROS_DOMAIN_ID=150
export BIO_NAV_RUN_ROOT=/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_<run_id>
export BIO_NAV_PHASE_F_SOCKET=/tmp/bio_nav_phase_f_${UID}_${ROS_DOMAIN_ID}.sock
```

`BIO_NAV_RUN_ROOT` must not already contain another run. If NAS is unavailable,
stop; do not redirect bags or experiment data into the repository or `/tmp`.

## 2. Phase B exact-scene baseline

The wrapper fixes the original USD, `v6_kujiale_isaacgen_v1` occupancy/spawn/GVG,
mixed odometry, AMCL ownership, `stable` Nav2, and M0/GVG routing. Validate the
manifest without launching a runtime:

```bash
./scripts/run_v6_r5_phase_b_kujiale.sh \
  --run-root "${BIO_NAV_RUN_ROOT}" --domain "${ROS_DOMAIN_ID}" manifest
```

Start components in the order printed by the wrapper:

```bash
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${ROS_DOMAIN_ID}" ros
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${ROS_DOMAIN_ID}" isaac
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${ROS_DOMAIN_ID}" module1-shadow
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${ROS_DOMAIN_ID}" bridge
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${ROS_DOMAIN_ID}" record
./scripts/run_v6_r5_phase_b_kujiale.sh --run-root "${BIO_NAV_RUN_ROOT}" --domain "${ROS_DOMAIN_ID}" runner
```

Each command owns its terminal. The ROS component starts before Isaac because
Isaac waits for the required ROS reset service. Start the runner last, after
the observable readiness conditions are satisfied.

## 3. Phase F current three-terminal composition

Phase F currently uses three operator terminals. This is the present adapter,
not the desired final shape; later convergence should reduce it to one canonical
owner without changing runtime contracts.

The example below uses the static M0 arm. Select another authorized condition
or arm only through the current experiment plan.

Terminal T1 owns Isaac:

```bash
./scripts/run_v6_kujiale_low_obstacles.sh --condition static isaac
```

Terminal T2 owns Module3 ROS and the Phase-F stack. For M1-M3 it also owns the
Module2 server and Integration bridge:

```bash
./scripts/run_v6_low_obstacle_phase_f_stack.sh M0 \
  --domain "${ROS_DOMAIN_ID}" \
  --run-dir "${BIO_NAV_RUN_ROOT}/stack/m0" \
  --socket "${BIO_NAV_PHASE_F_SOCKET}"
```

Terminal T3 owns `ExperimentRunner` and its evidence output:

```bash
./scripts/run_v6_kujiale_low_obstacles.sh --condition static runner \
  "${BIO_NAV_RUN_ROOT}/episodes/static_m0"
```

The selected wrapper fixes `nav2_profile:=v6_low_obstacle_isolation`. Do not
override the Phase B map, spawn, GVG, localization owner, or mixed-odometry
substrate from these commands.

## 4. Reset and episode boundary

`ExperimentRunner` is the reset owner. It calls `/simulation/reset`; Isaac
holds motion, restores the scene/spawn, runs required ROS reset hooks, and emits
the reset event. Do not call component reset services independently during an
owned episode and do not treat a process restart as a reset receipt.

## 5. Owned shutdown

For the Phase F three-terminal composition, stop in reverse dispatch order:

1. T3: stop `ExperimentRunner` and allow its current output to close.
2. T2: stop the Phase-F stack owner; it stops only its registered Module3,
   Integration, and Module2 process groups and removes its exact socket.
3. T1: stop Isaac after the control/consumer stack is down.

For the Phase B component layout, stop `runner`, `record`, `bridge`,
`module1-shadow`, `isaac`, then `ros`. Allow the recorder to close its bag before
the run root is evaluated.

Do not use global `pkill`, delete a socket with a live listener, or clean another
run's PID/runtime directory. If a wrapper reports that an owned child remains,
preserve the logs and diagnose that exact process identity.

## 6. Evidence boundary

Keep logs, bags, JSONL, images, and evaluator results inside the new NAS run
root. A successful startup, a focused test, or a historical campaign is not a
current live result. Record the three commit pins, domain, scene/condition,
arm, and output path with every engineering run.
