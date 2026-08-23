# V6-GRID localization core handoff (2026-08-23)

## Result

- Verdict: **PASS for code, focused tests, and isolated build/install** after
  replacing the service-trigger-time gate with FlatScan source-stamp
  correlation.
- Live ROS graph / Isaac Sim / NITROS / Nav2: **not run and not verified**.
- Branch/worktree: `cognitive-navigation` in the permitted Module3 worktree.
- Amendment base HEAD: `48ed78d7982d6bde61e7f3c8afb69f2918ff5071`.
- Result commit: the single commit containing this handoff (use
  `git rev-parse HEAD` after checkout).

## Why the trigger gate changed

The installed Isaac ROS Occupancy Grid Localizer 4.5.0 implementation caches
messages from `flatscan`. Its `std_srvs/Empty` callback localizes the cached
scan immediately when one exists. Separately, a message received on
`flatscan_localization` is localized directly. The published
`PoseWithCovarianceStamped` retains the selected `FlatScan.header`.

Therefore a result source stamp is legitimately earlier than the service call
that requested localization. The former `result_stamp >= trigger_clock_stamp`
rule rejected valid results and did not identify which scan produced a result.

## Source-stamp correlation

The production manager no longer calls
`/trigger_grid_search_localization`. The vendor service may still exist, but is
not part of this manager's production path.

1. `/bio_nav/relocalize` admits one generation and publishes
   `WAITING_FOR_SCAN`.
2. The manager continuously observes `/flatscan`. At trigger admission it
   records the greatest valid source stamp already observed.
3. The first subsequently received valid FlatScan whose source stamp is newer
   than that baseline is selected. A buffered/repeated pre-trigger stamp is not
   selected.
4. The original message is published exactly once, without changing its
   header or payload, to `/flatscan_localization`. Its source stamp becomes the
   generation's `expected_result_stamp_ns`, and status becomes
   `WAITING_FOR_RESULT`.
5. Only `/localization_result` with that exact source stamp can reach pose/TF
   validation. Different or old stamps are ignored without publishing a
   current-generation status and without consuming the pending generation.
6. The exact result is still required to be finite, map-frame, and to have a
   finite exact-stamp `odom->base_link` lookup. On acceptance it publishes the
   public pose/status and updates dynamic `map->odom`.

Only one scan can be selected per generation. A duplicate public Trigger
returns failure without changing the active generation or its latched state.
The existing `pending_timeout_s` default of 10 seconds clears either
`scan_timeout` or `result_timeout`, allowing a fresh trigger.

## ROS interfaces

| Name | Type | QoS / semantics |
|---|---|---|
| `/flatscan` | `isaac_ros_pointcloud_interfaces/msg/FlatScan` | Input observed by manager; reliable/volatile keep-last 10, matching the installed vendor `DEFAULT` QoS. |
| `/flatscan_localization` | `isaac_ros_pointcloud_interfaces/msg/FlatScan` | Exactly one selected original message per generation; same vendor-compatible QoS. |
| `/localization_result` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Vendor result input; exact selected source stamp required. |
| `/bio_nav/localization_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Accepted results only; reliable + transient-local, keep-last 1. |
| `/bio_nav/localization/status` | `diagnostic_msgs/msg/DiagnosticArray` | Latest manager state; reliable + transient-local, keep-last 1. |
| `/bio_nav/relocalize` | `std_srvs/srv/Trigger` | Public request. Success means a generation is waiting for its next scan. |

The three public interfaces remain unchanged. The ordered status keys are now:

```text
generation,state,accepted,reason,trigger_stamp_ns,
expected_result_stamp_ns,result_stamp_ns,correction_x_m,correction_y_m,
correction_yaw_rad,latency_s
```

`expected_result_stamp_ns` is additive and records the exact selected FlatScan
stamp. It is zero while waiting for a scan. Unrelated result stamps do not
replace the latched `WAITING_FOR_SCAN` or `WAITING_FOR_RESULT` status.

## TF behavior retained

- The accepted dynamic correction is sent at the result stamp and receipt
  stamp, then refreshed from current ROS time at 20 Hz by default.
- The manager still uses a two-thread executor so the TF listener can populate
  its buffer while the result callback performs an exact-stamp lookup.
- It remains the sole grid `map->odom` publisher and never publishes
  `odom->base_link`.
- The formula remains:

```text
T_map_odom(t) = T_map_base_grid(t) * inverse(T_odom_base_ekf(t))
```

## Validation

- Source-focused suite including unchanged mapping-mode checks: **24 passed**
  (20 package tests + 4 mapping checks).
- Coverage includes exact selected-stamp acceptance; unexpected stamp then
  exact acceptance; one forward per generation; pre-trigger stamp exclusion;
  no-scan and no-result timeout/retry; duplicate preservation; and the prior
  `t+1 ms` dynamic-TF lookup regression.
- Explicit `ament_flake8`, `ament_pep257`, and `ament_xmllint`: **PASS**.
- Clean `/opt/ros/jazzy` isolated build/install/import:
  `/tmp/v6_grid_stamp_repair.HMLzaL`; installed imports resolve from that
  prefix and include the local `FlatScan` Python type.
- Installed-package colcon tests: **20 passed, 0 failures**.
- No complete ROS graph, vendor component, Isaac Sim, Nav2, or live TF run was
  started.

## Downstream use and remaining risk

Call `/bio_nav/relocalize`, read `generation=N`, then wait for the same
generation to progress from `WAITING_FOR_SCAN` to `WAITING_FOR_RESULT` and
finally `ACCEPTED`. A timeout is terminal and permits a new trigger. No
downstream node may publish `map->odom` in grid mode.

The local Python message type and vendor v4.5 `DEFAULT` QoS contract were
verified statically. Actual NITROS negotiation, FlatScan delivery ordering,
vendor result echo, exact-stamp TF availability, component loading, and live TF
ownership remain **unverified** until the authorized grid smoke.

## Actual-launch blocker repair amendment (2026-08-23)

- `localization.launch.py` now follows the installed NVIDIA 4.5 launch
  contract: the resolved occupancy-map YAML is passed as a parameter-file
  source before the minimal `use_sim_time`, `loc_result_frame`, and
  `map_yaml_path` overrides. No map fields are copied in project code.
- `LaserScantoFlatScanNode` now receives its installed official
  `input_qos=SENSOR_DATA` setting. Its output remains the vendor `DEFAULT`
  profile, compatible with both Grid consumers.
- Isolated installed launch on `ROS_DOMAIN_ID=186/187` loaded the actual
  `v6_kujiale_isaacgen_v1` map without the constructor image failure. Runtime
  parameters reported image `v6_kujiale_isaacgen_v1.pgm`, resolution `0.05`,
  origin `[-5.14, -6.52, 0.0]`, and the exact map YAML path.
- The actual `/scan` publisher/subscription endpoints were both
  best-effort/volatile. A synthetic LaserScan produced `/flatscan` with the
  same stamp, frame, five angles, and five ranges; `/flatscan` publisher and
  both Grid subscribers remained reliable/volatile.
- Validation: mapping source tests **4 passed**; isolated build/install at
  `/tmp/v6_map_qos_repair_final.p23nr1`; installed `robot_mapping` tests and
  linters **15 passed, 0 failures**. Smoke logs are under
  `/tmp/v6_map_qos_smoke.wpWho2`.
- Scope remains launch smoke only: Isaac Sim, Nav2, localization-result/TF
  acceptance, five-leg motion, evidence, and qualification were not run. The
  controlled SIGINT exposed the manager's pre-existing double-shutdown
  traceback after the probes completed; it was not one of these reproduced
  launch blockers and was not changed.
