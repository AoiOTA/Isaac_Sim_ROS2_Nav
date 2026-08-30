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
| `/amcl_pose` | AMCL, indoor only | Indoor global estimated pose observation; absent from the outdoor runtime. |
| `map -> odom` | AMCL indoors; `ideal_localization_tf` outdoors | Exactly one scene-selected global-localization TF owner. |
| `/bio_nav/module1/odom` | wheel + corrected-IMU EKF | Module1 observation topic; `publish_tf=false`. It does not replace `/odom`. |
| robot structure TF | Isaac Sim | Mixed mode requires `structure_tf_source:=isaac`. |

The active wrappers fix `odometry_mode:=mixed` and
`localization_map_contract:=occupancy_only`. Kujiale fixes
`localization_owner:=amcl`; Rivermark fixes `localization_owner:=ideal`, loads
the calibrated `rivermark_start`, and rejects a mismatched map bundle. LiDAR
odometry and LiDAR EKF fusion are off in both substrates. A second
`map -> odom` or `odom -> base_link` publisher is a contract violation.

## Scene, map, and route identity

The current indoor exact-scene combination is:

- original USD:
  `/home/lyb/kujiale_usd_rooms_20260717/kujiale_0026/kujiale_0026_A_to_B_door_open.usd`;
- occupancy map: `data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml`;
- spawn calibration:
  `isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml`;
- GVG:
  `ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson`.

The current outdoor combination retains:

- frozen USD:
  `/mnt/nas_home/Bio_Nav_Data/experiments/assets/rivermark_plaza_v6_final_20260829/rivermark.usd`;
- occupancy map: `data/rivermark_demo/rivermark_selected.yaml`;
- spawn calibration: `data/rivermark_demo/rivermark.spawn.yaml`;
- route graph: `data/rivermark_demo/rivermark_selected.geojson`;
- regions: `data/rivermark_demo/rivermark_regions.yaml`;
- 30-tile SR/DR catalog:
  `/mnt/nas_home/Bio_Nav_Data/experiments/assets/rivermark_a_srdr_tile_catalog_v1`.

Module3 owns physical traversability, graph legality, the selected route,
local avoidance, and final control. Module2 may provide bounded additive
priors and cognitive-obstacle observations. Missing, stale, invalid, or
unhealthy Module2 input must not grant traversability, publish TF, or command
the robot.

Appearance selection does not move collision geometry: static and appearance
retain the selected low-obstacle physical layout. The dynamic condition instead
uses the LiDAR-visible G2 crossing obstacle and does not validate sub-LiDAR
dynamic-obstacle perception.

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

Fresh live terminal acceptance requires the first `/cmd_vel_sim` zero no later
than 0.25 s after the terminal outcome, no later nonzero actuator command, and
post-terminal ground-truth XY/yaw within the preregistered bounds. This cleanup
does not validate that acceptance. Historical Attempt9 measurements remain in
the [experiment ledger](handoff/EXPERIMENT_LEDGER.md) and are not restated here.

## Reset contract

Isaac `ResetServiceBridge` is the sole owner of the `/simulation/reset` service
and reset transaction. Each current Pilot run has one orchestrating caller:

```text
Phase F: ExperimentRunner -> ResetServiceBridge (/simulation/reset)
```

Historical Phase B has its own alternative caller; it is never concurrent with
ExperimentRunner in one run. The Isaac transaction holds the ResetStopGate, restores the selected
spawn and simulation state, invokes required ROS odometry reset hooks, then
publishes `/simulation/reset_event`. Consumers clear or re-seed their episode
state from that event; they do not initiate independent resets. The gate status
is exposed on `/simulation/reset_stop_gate/status`, and motion is released only
after the current generation is ready.

Operator shutdown is not an episode reset. Stop the owning terminal/process
group as documented in [RUNBOOK.md](RUNBOOK.md); do not use global `pkill` or
manually unlink a live socket.
