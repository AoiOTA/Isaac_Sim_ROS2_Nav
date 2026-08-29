# Bio_Nav Module3

Module3 owns the V6 physical navigation chain: Isaac Sim scenes and sensors,
map/TF ownership, GVG routing, Nav2, collision monitoring, and the final
velocity path to the simulated robot.

## Start here

- [Current state](docs/CURRENT_STATE.md): current runtime boundary, evidence
  level, GPU blocker, and exact resume point.
- [Runbook](docs/RUNBOOK.md): clean-shell environment, indoor/outdoor component
  commands, startup ordering, and owned cleanup.
- [Runtime interfaces](docs/interfaces.md): topic, TF, reset, and control
  ownership.
- [Repository index](docs/repository_index.md): implementation and asset index.

The authoritative cross-repository handoff is Integration
`docs/CURRENT_STATE.md` on branch `v6-compute-amcl-dual-odom`.

## Canonical worktrees

```bash
export BIO_NAV_INTEGRATION_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration
export BIO_NAV_MODULE3_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module3
export BIO_NAV_MODULE2_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module2
```

Do not substitute similarly named cleanup, Attempt, or historical worktrees.
Use a fresh ROS domain, short socket, and absent NAS output root for every live
attempt. Generated data belongs on NAS, not in Git.

## Current scene split

- Indoor: mixed Compute Odometry plus AMCL localization.
- Outdoor: mixed Compute Odometry plus calibrated fixed `map -> odom`; AMCL is
  not started and the original Rivermark map remains active.
- Both: Module1 wheel+IMU odometry without TF, Module2 obstacle output, GVG,
  SR/DR RoutePrior, cognitive obstacle layers, and risk critic in the M3 arm.

At the current stop boundary, the Rivermark-only DLSS-disabled startup is an
unlive candidate. Read [Current state](docs/CURRENT_STATE.md) before launching
anything.
