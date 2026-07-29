# G2 dynamic-safety repair plan

## Trigger and scope

The frozen integration campaign `20260729-g2-motion-contract-gate-05` stopped
on dynamic record 16 (seed `10106`, `dynamic_appearance`, `bright_warm`,
variant `v3`).  This is a Module3 physical-safety repair, not a Module2
domain-adaptation or planning-prior change.

The preserved telemetry shows the relevant sequence:

- at simulation time `28.233 s`, `local_bypass_actor` entered
  `safety_yield` at `0.1195 m` calculated guard clearance;
- while the actor remained stopped, its calculated clearance reached `0.0 m`
  at `28.417 s` (`actor=(-1.102,-0.200)`,
  `robot=(-0.794,-0.705)` in map coordinates);
- the navigation command did not become zero until after that penetration;
- `/simulation/collision` first reported true at `52.2 s`, so the whole
  route is invalid even though it subsequently reached its final pose.

The repair is intentionally limited to the dynamic Nav2 avoidance profile:
increase the local, pre-contact avoidance margin so MPPI turns or slows before
the actor-side guard has to hold a kinematic cube in the robot's swept path.
It must not alter the actor's physical geometry, actor lifecycle, static
profile, map, localization, Module2, planner, collision-monitor source
ownership, or `/cmd_vel` chain.

## Fixed implementation hypothesis

The candidate changes only
`ros2_ws/src/robot_navigation/config/nav2_dynamic_avoidance.yaml`:

- extend the dynamic local-costmap inflation envelope from `0.60 m` to
  `0.75 m`;
- raise the dynamic MPPI `CostCritic` soft avoidance weight from `2.50` to
  `4.00` while retaining the existing hard footprint collision cost;
- retain the 15 Hz controller, 10 Hz local costmap, STVL lifecycle, LiDAR
  obstacle layer, and Collision Monitor configuration.

This is a pre-contact soft-cost repair.  It neither treats collision as a
warning nor disables the actor after a guard event.

## Validation sequence

1. Unit/configuration tests must prove the change is dynamic-profile-only and
   preserve the hard collision and safety-chain contract.
2. Run one fresh Module3-only, full-route `v3` smoke in the failing family
   (`dynamic_appearance`, `bright_warm`) after a cold Isaac restart.  This is
   a development smoke, not a G2 record and not reusable Gate evidence.
3. The smoke passes only if all of the following are true: navigation succeeds,
   `/simulation/collision` is never true, every actor is triggered and retired,
   `local_bypass_actor` minimum clearance is at least `0.10 m`, no guard
   abort occurs, and the dynamic runtime hash/actor IDs match the fresh
   configuration.
4. If the smoke fails, preserve its evidence and stop; do not tune a second
   parameter or run a batch.  If it passes, create a fresh G2 preregistration
   and campaign.  The old 15 sealed receipts and failing record are never
   reused.

## Boundaries

This stage does not authorize `BioNavGridBased`, a Goal-Prior Bridge, frozen
SR construction, online Module2 memory updates, Module2 control, or a change
to G2/Confirmation thresholds.  Those remain blocked until a fresh G2 and
Confirmation qualification pass.
