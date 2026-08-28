# Bio_Nav Module3

Module3 owns the V6 physical navigation chain: Isaac Sim scene and sensors,
occupancy map, TF, GVG Route Server, Nav2, collision monitoring, and the final
velocity path to the simulated robot. The current line is the Kujiale V6
compute-odometry + AMCL dual-state stack. Rivermark and earlier campaigns remain
historical material; they are not the current runtime baseline.

## Start here

- [Current state](docs/CURRENT_STATE.md): pinned repository combination,
  evidence boundary, and open P2 work.
- [Runbook](docs/RUNBOOK.md): clean-shell setup, current Phase B and Phase F
  commands, NAS output, and owned shutdown order.
- [Runtime interfaces](docs/interfaces.md): current topic, TF, reset, and control
  ownership.
- [Repository index](docs/repository_index.md): current wrappers, assets,
  configs, and implementation locations.

## Current baseline

| Repository | Commit |
| --- | --- |
| Module3 | `4e9030f3413214c8a4cc0cf0f5e1a16b3785ee91` |
| Integration | `14594f38` |
| Module2 | `7f4fbae` |

Use the canonical V6 worktrees and one environment source:

```bash
cd /home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module3
source ../bio_nav_integration/env/v6_pilot_setup.sh
```

The Phase B exact-scene baseline is selected through the wrapper; do not bypass
it with hand-written launch arguments:

```bash
./scripts/run_v6_r5_phase_b_kujiale.sh manifest
./scripts/run_v6_r5_phase_b_kujiale.sh --domain "${ROS_DOMAIN_ID}" ros
./scripts/run_v6_r5_phase_b_kujiale.sh --domain "${ROS_DOMAIN_ID}" isaac
```

Phase F keeps the same scene/localization substrate and selects the frozen
low-obstacle/Nav2 isolation entrypoints:

```bash
./scripts/run_v6_kujiale_low_obstacles.sh --condition static isaac
./scripts/run_v6_low_obstacle_phase_f_stack.sh M0 --domain 150 \
  --run-dir /mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_example/stack/m0 \
  --socket /tmp/bio_nav_phase_f_example.sock --dry-run
```

These are component commands, not a one-terminal campaign. Follow
[the runbook](docs/RUNBOOK.md) for terminal ownership and cleanup. Store live
outputs under `/mnt/nas_home/Bio_Nav_Data/experiments/runs/`; do not silently
substitute a local data root when NAS is unavailable.
