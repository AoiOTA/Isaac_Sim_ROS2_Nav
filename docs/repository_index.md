# Current V6 repository index

This index covers the canonical indoor Kujiale and outdoor Rivermark V6
runtimes. Dated reports and the historical ledger are evidence records, not
current execution entrypoints.

## Start and operate

| Path | Current responsibility |
| --- | --- |
| `README.md` | Landing page and shortest commands. |
| `docs/CURRENT_STATE.md` | Pinned baseline, evidence layer, and open P2 work. |
| `docs/RUNBOOK.md` | Current multi-terminal run and owned cleanup procedure. |
| `docs/interfaces.md` | Runtime ownership and interface contract. |
| `scripts/run_v6_r5_phase_b_kujiale.sh` | Exact-scene Phase B component selector; delegates ROS and Isaac to the canonical launchers. |
| `scripts/run_isaac.sh` | Single Isaac owner and `navigation_sim.py` launcher. |
| `scripts/run_ros.sh` | Single ROS/Nav2 bringup owner and installed-space gate. |
| `scripts/run_v6_kujiale_low_obstacles.sh` | Phase F condition/profile selector on the Phase B substrate. |
| `scripts/run_v6_low_obstacle_phase_f_stack.sh` | Phase F owner for Module3 ROS and, for M1-M3, the Integration bridge and Module2 server. |
| `scripts/run_v6_rivermark.sh` | Rivermark selector for `isaac|ros` by `static|dynamic|appearance`; requires frozen `RIVERMARK_USD`, fixes mixed Compute Odometry plus calibrated fixed localization, enables the M3 cognitive chain, and disables DLSS only outdoors. |

The current environment entry is
`/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration/env/v6_pilot_setup.sh`.

## Scene and navigation identity

| Path | Current responsibility |
| --- | --- |
| `/home/lyb/kujiale_usd_rooms_20260717/kujiale_0026/kujiale_0026_A_to_B_door_open.usd` | Original, read-only Kujiale scene used by the active wrappers. |
| `data/maps/occupancy/v6_kujiale_isaacgen_v1.yaml` | Current occupancy map. |
| `isaac_sim/configs/environments/kujiale_0026_A_to_B_door_open.v6_isaacgen_v1.spawn.yaml` | Current spawn/map calibration. |
| `ros2_ws/src/robot_route_planner/config/v6_kujiale_isaacgen_v1_gvg_v1.geojson` | Current GVG route graph. |
| `ros2_ws/src/robot_navigation/config/nav2_stable.yaml` | Stable Phase B Nav2 overlay. |
| `ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml` | Phase F low-obstacle isolation overlay. |
| `isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen.yaml` | Frozen static low-obstacle condition. |
| `isaac_sim/configs/experiments/v6_single_dynamic_low_obstacle.yaml` | Single dynamic low-obstacle condition. |
| `isaac_sim/configs/experiments/kujiale_appearance_profiles.yaml` | Appearance profiles only; the runner selects and records a profile without changing the selected one-low-obstacle physical layout. |
| `data/rivermark_demo/rivermark_selected.yaml` | Original retained Rivermark occupancy map. |
| `data/rivermark_demo/rivermark.spawn.yaml` | Calibrated outdoor fixed `map -> odom` source. |
| `data/rivermark_demo/rivermark_selected.geojson` | Outdoor route graph. |
| `data/rivermark_demo/rivermark_regions.yaml` | Outdoor cognitive tile regions. |

Rivermark has no repository-local USD. Pass the frozen NAS scene through
`RIVERMARK_USD`; the wrapper requires it to be readable and does not infer a
historical host-local path. Its SR/DR catalog is also a required external NAS
asset; see `docs/CURRENT_STATE.md` for the exact paths.

## Core implementation

| Path | Current responsibility |
| --- | --- |
| `isaac_sim/apps/navigation_sim.py` | Scene, simulation loop, sensors, and robot runtime. |
| `isaac_sim/graphs/odometry_graph.py` | Isaac Compute Odometry publisher for mixed mode. |
| `isaac_sim/src/bridge/reset_service.py` | `/simulation/reset` transaction and reset event. |
| `isaac_sim/src/bridge/reset_stop_gate.py` | Final `/cmd_vel` to `/cmd_vel_sim` reset/terminal gate. |
| `isaac_sim/src/bridge/tf_ownership.py` | Supported TF publisher ownership. |
| `ros2_ws/src/robot_bringup/launch/ros_stack.launch.py` | Unified localization/odometry/profile selection. |
| `ros2_ws/src/robot_bringup/config/modes.yaml` | Dual-state mode and TF ownership declaration. |
| `ros2_ws/src/robot_mapping/launch/localization.launch.py` | Scene-selected AMCL or calibrated ideal `map -> odom` owner. |
| `ros2_ws/src/robot_route_planner/robot_route_planner/ros_node.py` | GVG routing, runtime edge state, and route publications. |
| `ros2_ws/src/robot_navigation/launch/navigation.launch.py` | Nav2, smoothing, Collision Monitor, and cognitive consumers. |
| `ros2_ws/src/robot_experiments/robot_experiments/experiment_runner.py` | Episode owner and `/simulation/reset` client. |

## Evidence and outputs

- Live bags, logs, images, and run results belong under the selected fresh
  `/mnt/nas_home/Bio_Nav_Data/experiments/{pilots,runs}/` root.
- `docs/handoff/EXPERIMENT_LEDGER.md` retains historical engineering context
  without changing its evidence classification; no V6 micro-handoff files
  remain.
- Generated `docs/reports/` from source HEAD
  `09c3ae80a5766ccf37fd244421e4c5f50afe7e91` is stored at
  `/mnt/nas_home/Bio_Nav_Data/experiments/visualizations/module3_repo_generated_09c3ae80a5766ccf37fd244421e4c5f50afe7e91/docs/reports/`:
  9 files, 512198 bytes, with per-file `cmp` PASS. This storage move does not
  promote the reports' evidence classification.
- `docs/report_assets/`, `docs/videos/`, and `docs/figures/` remain tracked
  because repository callers still reference them.
- Do not commit `build/`, `install/`, `log/`, local runtime roots, or new live
  output into the repository.
