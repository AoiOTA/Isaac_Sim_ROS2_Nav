# V6 C4 Kujiale low-obstacle layout handoff

Date: 2026-08-20

## Result

- Branch/worktree: `cognitive-navigation` at `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`.
- Starting HEAD: `f81c690f2064a5711ecfe8ea68edcabc3e41915e`.
- Commit: the commit containing this handoff.
- Verdict: **PASS for frozen configuration and static contract tests only**.
- Isaac, ROS, Nav2, camera visibility, collision, and navigation runtime were not started or claimed.

## Frozen layout

> **Revision 2026-08-22 (user decision): layout reduced 6 → 1.** Only
> `v6_low_box_solo` (0.30×0.30×0.16 m, 5 kg, stationary) remains, at
> map (-1.150, -0.350, 0.08) / usd (4.050, 0.150, 0.08) in the central open
> hall. New contract: edge clearance to every route segment ≥ 1.0 m
> (measured ≥ 1.60 m), best open side ≥ 1.2 m (measured 3.41 m); the old
> ≤1.5 m route-proximity and pairwise-clearance clauses no longer apply.
> `layout_id`/`revision`/`frozen_date` are unchanged so the causal identity
> chain (`v6_low_obstacle_causal`) is untouched. Overlay evidence:
> `/mnt/nas_home/Bio_Nav_Data/experiments/runs/v6_layout_single_obstacle_20260822T144924Z/`.
> The r1 six-obstacle description below is kept for history.

- Layout ID: `kujiale_v6_low_obstacles_frozen_r1_20260820`.
- Files: `isaac_sim/configs/experiments/v6_kujiale_low_obstacles_frozen.yaml` and `v6_kujiale_low_obstacles_frozen_manifest.yaml`.
- The source Kujiale USD and `warehouse_new` map were not edited. The new schema-v2 obstacle file remains `enabled: false`; only the explicit V6 wrapper passes `--dynamic-obstacles` through the existing Isaac launcher.
- Five original centers are retained. `v6_low_bar_east.x` moves minimally from `1.286899` to `1.300000` m so the nearest pairwise rectangle clearance increases from about 0.577 m to about 0.588 m.
- Required passage is 0.585 m: Jackal maximum footprint dimension 0.485 m plus 0.05 m on each side.
- All six 0.16 m-high obstacles sample as free in `warehouse_new`; every obstacle has at least one cardinal open side of 1.9 m or more and lies within 1.5 m of the whole-home route polyline.
- Vertical extent is `[0.0, 0.16]` m: below the 0.333 m LiDAR plane and 0.218 m RGB-D origin, while overlapping the wheel vertical extent `[0.0, 0.196]` m.
- The manifest records the audited transform `usd_x=2.9-map_x`, `usd_y=-0.2-map_y`, yaw offset 180 degrees.

## Explicit profile wiring

- `scripts/run_v6_kujiale_low_obstacles.sh isaac` selects only the additive `v6-low-obstacles` Isaac runner mode and its frozen V6 obstacle YAML.
- `... ros` selects `odometry_mode:=estimated`, Kujiale localization, and `nav2_profile:=v6_low_obstacle_isolation`.
- `... runner` selects only `v6_kujiale_low_obstacles_static.yaml`.
- Existing static/dynamic campaign defaults still point to their original obstacle files.
- `rgbd_navigation` continues publishing RGB, depth image, and depth points. The V6 isolation Nav2 overlay removes both depth voxel layers from plugin lists, so `/camera/front/depth/points` does not write a Costmap directly. A later cognitive obstacle path may consume the retained RGB-D stream.

## Validation

```bash
bash -n scripts/run_v6_kujiale_low_obstacles.sh
PYTHONPATH=.:ros2_ws/src/robot_experiments:ros2_ws/src/robot_bringup \
  /usr/bin/python3 -m pytest -q \
  isaac_sim/tests/test_v6_low_obstacle_layout.py \
  isaac_sim/tests/test_dynamic_obstacles.py \
  ros2_ws/src/robot_experiments/test/test_configuration.py \
  ros2_ws/src/robot_bringup/test/test_mode_contract.py
git diff --check
```

Result: `103 passed`; shell syntax and diff checks passed.

## Remaining runtime risks

- Static occupancy and clearance sampling do not prove physical placement, camera visibility, contact, Costmap behavior, or successful bypass in Isaac.
- The AMCL/estimated stack and future cognitive obstacle consumer must be exercised separately before using this layout for causal or qualification claims.
- No top-down design image was generated; the frozen YAML/manifest plus deterministic geometry tests are the design evidence.
