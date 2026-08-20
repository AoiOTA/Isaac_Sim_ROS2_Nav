# V6 C/profile repair handoff — 2026-08-20

## Scope and hypothesis

- Worktree: `worktrees/cognitive-navigation/bio_nav_module3`
- Branch: `cognitive-navigation`
- Starting HEAD: `b6b623ae7931ef09aef1295fa23d9e8cf751f6c2`
- Goal: make C2/C3 and M0--M3 executable in the final Nav2 parameter chain, preserve fail-open behavior, and make the V6 ROS wrapper reject stale/foreign Integration underlays.
- No `robot_route_planner/**` files were read or modified for implementation.

## Implemented contracts

- A final exact-node cognitive overlay is appended after the A21 overlay. Its controller critic list retains `VelocityDeadbandCritic` and ends with `CognitiveRiskCritic`.
- `M0/M1/M2/M3` resolve respectively to `off/off/gvg`, `shadow/shadow/shadow`, `active/off/hybrid`, and `active/active/primary`. The V6 wrapper accepts the mode as its optional ROS argument and defaults to M3.
- Cognitive direction scoring treats N/E/S/W/stay as `base_link` semantics, rotates the preference by robot yaw into the global trajectory frame, and applies no direction penalty to zero/stay/cancelling vectors. Direction schema, frame, source sequence, health and trusted-write fields are gated independently; an invalid direction cannot bias scoring.
- The cognitive obstacle layer now gates maximum OOD, health/trust, finite obstacle fields and identity before first generation binding. Callback receipt never reports `applied=true`; only a successful TF projection and a cost that actually raises the master costmap may do so. Stale/future/TF failures clear the private layer and leave the classic layers running.
- V6 sourcing clears ambient ROS overlay paths, sources the explicitly allowed Integration underlay first, validates package prefixes, `engineering_defaults.yaml`, and current generated interface fields, then sources only Module3 `local_setup.bash`. The legacy source path remains unchanged.

## Validation

- `bash -n` on `scripts/lib/common.sh`, `scripts/build_ros2.sh`, and `scripts/run_v6_kujiale_low_obstacles.sh`: PASS.
- `python3 -m py_compile` on the modified Python launch/contract files: PASS.
- Focused owned pytest set (`mode_contract`, runtime scripts, V6 profile, Nav2 config, low-obstacle layout): `73 passed`.
- Clean Integration source build at HEAD `370324f6ae589baa5d6ad3829e9cfd2d63763e57`, with `/tmp` build/log and allowed-worktree temporary install: `bio_nav_interfaces` and `bio_nav_ros_bridge` PASS.
- Clean `bio_nav_fusion` build against that current interface underlay: PASS.
- Focused CTest: `test_equal_cost_search` and `test_plugin_loader_isolation`, `2/2` PASS.

## Result and remaining live risk

- Result: **PASS for code/build/focused contract tests**.
- No ROS/DDS/Nav2/Isaac runtime or navigation campaign was launched in this task. TF timing, lifecycle activation, real costmap merge telemetry, and M0--M3 live topic/status behavior remain runtime risks for the next authorized smoke run.
- The repository's default Integration install was stale during this task and is now rejected. Until it is rebuilt from the current allowed Integration worktree, use `BIO_NAV_INTEGRATION_SETUP` to select the current allowed-worktree temporary install or rebuild the default install using the exact command emitted by the wrapper.
