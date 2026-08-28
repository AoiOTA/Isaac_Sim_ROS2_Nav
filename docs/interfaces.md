# V6 runtime interfaces and ownership

This is the current Module3 runtime contract for the canonical
`v6-compute-amcl-dual-odom` stack. Older branch-specific contracts are
historical and do not override this document.

## Physical and localization ownership

| Interface | Current owner | Contract |
| --- | --- | --- |
| `/clock`, sensors, ground truth, scene physics | Isaac Sim | Simulation-time and physical-world source. |
| `/odom` | Isaac Compute Odometry | Canonical odometry consumed by Nav2 and route execution. |
| `odom -> base_link` | Isaac Compute Odometry | The only publisher of this dynamic TF in mixed mode. |
| `/amcl_pose` | AMCL | Global estimated pose observation. |
| `map -> odom` | AMCL | The only publisher of global-localization TF. |
| `/bio_nav/module1/odom` | wheel + corrected-IMU EKF | Module1 observation topic; `publish_tf=false`. It does not replace `/odom`. |
| robot structure TF | Isaac Sim | Mixed mode requires `structure_tf_source:=isaac`. |

The active wrappers fix `odometry_mode:=mixed`,
`localization_map_contract:=occupancy_only`, and `localization_owner:=amcl`.
LiDAR odometry and LiDAR EKF fusion are off in this substrate. A second
`map -> odom` or `odom -> base_link` publisher is a contract violation.

## Scene, map, and route identity

The current exact-scene combination is:

- original USD:
  `/home/lyb/kujiale_usd_rooms_20260717/kujiale_0026/kujiale_0026_A_to_B_door_open.usd`;
- occupancy map: `data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml`;
- spawn calibration:
  `isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml`;
- GVG:
  `ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson`.

Module3 owns physical traversability, graph legality, the selected route,
local avoidance, and final control. Module2 may provide bounded additive
priors and cognitive-obstacle observations. Missing, stale, invalid, or
unhealthy Module2 input must not grant traversability, publish TF, or command
the robot.

## Current data-plane topics

| Topic | Purpose / owner |
| --- | --- |
| `/bio_nav/navigation_graph` | Module3 graph identity and state. |
| `/bio_nav/canonical_route` | Module3 selected macro route. |
| `/bio_nav/route_edge_costs` | Module3 route cost snapshot, including admitted bounded additions. |
| `/bio_nav/route_progress` | Module3 route execution progress. |
| `/bio_nav/route_goal_complete`, `/bio_nav/route_goal_result` | Module3 terminal route outcome. |
| `/bio_nav/module2/planning_prior` | Module2 planning/localization observation consumed through Integration. |
| `/bio_nav/module2/edge_priors` | Optional bounded route-edge prior input. |
| `/bio_nav/module2/cognitive_obstacles` | Optional cognitive-obstacle input. |
| `/bio_nav/cognitive_obstacle_layer/status` | Module3 costmap consumer status. |
| `/bio_nav/cognitive_risk_critic/status` | Module3 MPPI consumer status. |
| `/scan`, `/scan_safety` | Global/localization scan and self-filtered local safety scan respectively. |

Topic presence alone is not readiness. Current consumers also require valid
identity, freshness, TF, and the wrapper-selected profile.

## Nav2 and control ownership

The Phase B navigation baseline uses `nav2_profile:=stable`. Phase F uses
`nav2_profile:=v6_low_obstacle_isolation` with its explicit config overlay.
Both preserve the same physical authority chain:

```text
Nav2 controller -> /cmd_vel_nav
velocity smoother -> /cmd_vel_smoothed
Collision Monitor -> /cmd_vel
Isaac ResetStopGate -> /cmd_vel_sim
robot articulation
```

`/cmd_vel` is the final public safety output. Isaac consumes only its private
`/cmd_vel_sim` relay. Module1, Module2, and Integration do not own either final
topic.

## Reset contract

There is one episode reset owner:

```text
ExperimentRunner -> /simulation/reset -> Isaac reset transaction
```

The Isaac transaction holds the ResetStopGate, restores the selected spawn and
simulation state, invokes required ROS odometry reset hooks, then publishes
`/simulation/reset_event`. Consumers clear or re-seed their episode state from
that event; they do not initiate independent resets. The gate status is exposed
on `/simulation/reset_stop_gate/status`, and motion is released only after the
current generation is ready.

Operator shutdown is not an episode reset. Stop the owning terminal/process
group as documented in [RUNBOOK.md](RUNBOOK.md); do not use global `pkill` or
manually unlink a live socket.
