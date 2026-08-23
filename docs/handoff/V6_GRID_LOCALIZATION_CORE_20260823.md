# V6-GRID localization core handoff (2026-08-23)

## Result

- Verdict: **PASS for code, focused tests, and isolated build**, including the
  post-review Grid core blocker repair.
- Live ROS / Isaac Sim / Nav2: **not run and not verified**.
- Branch/worktree: `cognitive-navigation` at the permitted Module3 worktree.
- Original core base HEAD: `ccbd54d1f800fcf6db073f22b52377a24b67a900`.
- Blocker-repair base HEAD: `1097f2ca0b15ae17d80d625b8c67d321ae69759b`.
- Result commit: the single commit containing this handoff (use
  `git rev-parse HEAD` after checkout).

## Implementation

- `robot_mapping/launch/localization.launch.py` now defaults to `grid` and
  composes the installed Isaac ROS 4.5.0 components directly:
  `LaserScantoFlatScanNode` and `OccupancyGridLocalizerNode`.
- The same required `map_file` YAML is passed to Nav2 map server and the Grid
  Localizer (`map_yaml_path`). `/localization_result` is preserved; the NVIDIA
  Nav2 helper and its `/initialpose` remap are not used.
- The production grid backend starts exactly one
  `grid_localization_tf_manager`. AMCL and odom_static are not selectable or
  started by this launch. The explicit `ideal` evaluator backend remains.
- New package `robot_grid_localization` owns generation gating, exact-stamp TF
  lookup, accepted pose/status publication, and the sole grid `map->odom` TF.
  It never publishes `odom->base_link` and does not consume Ground Truth.
- The accepted correction is sent immediately at both the result stamp and the
  receipt stamp, then the same dynamic correction is refreshed from the current
  ROS clock at `tf_broadcast_rate_hz` (default 20 Hz). It is not a static TF.
- The manager runs a two-thread executor. The TF listener's reentrant callback
  group can therefore populate the buffer while the result callback waits for
  the exact-stamp `odom->base_link` lookup.

## Frozen ROS interface

| Name | Type | QoS / semantics |
|---|---|---|
| `/localization_result` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Input, reliable/volatile keep-last 10. Installed Isaac ROS 4.5.0 result is the map-frame base pose. |
| `/bio_nav/localization_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Accepted results only; reliable + transient-local, keep-last 1. Original result stamp/covariance retained. |
| `/bio_nav/localization/status` | `diagnostic_msgs/msg/DiagnosticArray` | Latest trigger/result event; reliable + transient-local, keep-last 1. One status named `grid_localization`. |
| `/bio_nav/relocalize` | `std_srvs/srv/Trigger` | Standard service QoS. A successful response means the request was proxied; response text contains `generation=N`. |
| `/trigger_grid_search_localization` | `std_srvs/srv/Empty` | Isaac ROS 4.5.0 service proxied by the manager. |

Status keys are fixed and ordered as:

```text
generation,state,accepted,reason,trigger_stamp_ns,result_stamp_ns,
correction_x_m,correction_y_m,correction_yaw_rad,latency_s
```

`WAITING` is the only pending state; `ACCEPTED` and `REJECTED` are terminal.
`accepted=true` only describes an accepted localization result, not a
successfully queued trigger.
`correction_*` is the planar value of the published `T_map_odom`, and
`latency_s` is result/status receipt time minus trigger time on the node clock.

Each trigger admitted after the vendor service is ready increments
`generation`; a synchronous or asynchronous proxy failure terminally rejects
that generation. A second request returns service failure while one generation
is pending, without publishing a new status or overwriting that generation's
latched `WAITING`. Every pending generation retains its trigger stamp. A result
must have
`result_stamp_ns >= trigger_stamp_ns` for the current generation, so a late
result from a proxy-failed/timed-out generation cannot be accepted by a later
trigger when its source stamp predates that later trigger. This is the minimum
correlation available from the vendor's untagged standard result message.

Results received without a pending generation, with invalid/non-monotonic or
pre-trigger stamps, non-finite pose/covariance, a non-map frame, or without a
finite exact-result-stamp `odom->base_link` lookup are rejected and do not
publish pose or TF. A rejected pending result consumes that generation so the
caller can retrigger. A pending request also becomes terminal
`REJECTED/localization_timeout` after `pending_timeout_s` (default 10 s), clears
pending, and permits a new trigger. Acceptance computes:

```text
T_map_odom(t) = T_map_base_grid(t) * inverse(T_odom_base_ekf(t))
```

and publishes the accepted pose/status plus the continuously refreshed dynamic
`map->odom` TF described above.

## Validation

- Focused source tests: `19 passed` (the original 14 plus current-trigger stamp
  correlation, late-result rejection after proxy failure, timeout/retry,
  duplicate preservation, and an actual tf2 `t+1 ms` lookup regression).
- The four explicit review probes all passed: future TF lookup, late generation
  rejection, timeout then retry, and duplicate trigger preserving active state.
- Clean `/opt/ros/jazzy` isolated build, with new `/tmp` build/install/log
  directories: `robot_grid_localization` finished successfully and installed
  import succeeded at `/tmp/v6_grid_core_repair_final.jpybT7`.
- `robot_grid_localization`: colcon pytest `15 passed`; explicit
  `ament_flake8`, `ament_pep257`, and `ament_xmllint` all passed.
- No ROS graph, vendor component, Isaac Sim, Nav2, or live TF run was started.

## Downstream Integration / bringup changes required

1. Pass `localization_backend:=grid` (or rely on this launch default) and the
   validated `v6_kujiale_isaacgen_v1.yaml` path via `map_file`; remove AMCL,
   odom_static, `/initialpose`, and `/amcl_pose` readiness paths.
2. Start EKF and confirm timestamped `odom->base_link` exists before calling
   `/bio_nav/relocalize`.
3. Treat Trigger `success=true` as queued, extract the returned generation, and
   wait for `/bio_nav/localization/status` with the same generation and
   `state=ACCEPTED`; use `/bio_nav/localization_pose` as the accepted pose.
4. On `REJECTED`, retry with a new trigger according to the existing reset/run
   flow. No downstream node may publish `map->odom` in grid mode.

Remaining risk: component loading, NITROS FlatScan flow, service/result timing,
exact-stamp TF availability, and live TF ownership have not been exercised.
