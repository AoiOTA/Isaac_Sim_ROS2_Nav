# V6-GRID localization core handoff (2026-08-23)

## Result

- Verdict: **PASS for code, focused tests, and isolated build**.
- Live ROS / Isaac Sim / Nav2: **not run and not verified**.
- Branch/worktree: `cognitive-navigation` at the permitted Module3 worktree.
- Base HEAD: `ccbd54d1f800fcf6db073f22b52377a24b67a900`.
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

`state` is `WAITING`, `ACCEPTED`, or `REJECTED`. `accepted=true` only describes
an accepted localization result, not a successfully queued trigger.
`correction_*` is the planar value of the published `T_map_odom`, and
`latency_s` is result/status receipt time minus trigger time on the node clock.

Each successfully proxied trigger increments `generation`; a second request is
rejected while one generation is pending. Results received without a pending
generation, with invalid/non-monotonic stamps, non-finite pose/covariance, a
non-map frame, or without a finite exact-result-stamp `odom->base_link` lookup
are rejected and do not publish pose or TF. A rejected pending result consumes
that generation so the caller can retrigger. Acceptance computes:

```text
T_map_odom(t) = T_map_base_grid(t) * inverse(T_odom_base_ekf(t))
```

and publishes the accepted pose, status, and dynamic `map->odom` TF using the
localization result timestamp.

## Validation

- Focused source tests: `14 passed` (transform formula; generation/pending;
  invalid/stale/no-TF rejection; accepted status; launch/static ownership;
  mapping modes).
- Clean `/opt/ros/jazzy` isolated build, with new `/tmp` build/install/log
  directories and `robot_slam_solver` ignored rather than reading/building that
  out-of-scope package: `robot_grid_localization` and `robot_mapping` both
  finished successfully.
- `robot_grid_localization`: colcon pytest `10 passed`; explicit
  `ament_flake8`, `ament_pep257`, and `ament_xmllint` all passed.
- `robot_mapping`: direct CTest from the clean configured build tree `5/5`
  passed, including focused pytest and all existing lints.
- Note: the generic `colcon test --packages-select ...` wrapper for
  `robot_mapping` stops before CTest because its pre-existing runtime dependency
  hook expects unselected workspace package `robot_slam_solver`. This is not a
  compile/test failure in the changed packages; direct CTest avoids expanding
  validation scope.

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
