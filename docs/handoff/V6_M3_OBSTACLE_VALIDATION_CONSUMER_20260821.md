# V6 Module3 cognitive-obstacle validation consumer

## Scope and revisions

- Goal: accept only current, trusted fresh obstacles or depth-revalidated static
  obstacles, and otherwise leave the authoritative Nav2 Costmap unchanged.
- Branch/worktree: `cognitive-navigation` at
  `/home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_module3`.
- Starting Module3 HEAD: `b71fdf31644bd8a89dea70a91929e3bc537f8657`.
- Integration IDL underlay:
  `4f294be1b9f2f0eff3fc27199082cb22b9ab9cdb`.
- Result commit: the commit containing this handoff.

## Implementation

- `validateMessage()` now treats `validation_stamp` plus `validation_ttl` as
  the current-freshness clock (TTL at most 0.5 s, 50 ms future tolerance).
- Source age must exactly equal both source-to-validation timestamp deltas and
  be at most 2.0 s. Source/validation odometry stamps must be valid and
  monotonic.
- `VALIDATION_FRESH` requires identical source/validation and odometry stamps.
- `VALIDATION_STATIC_DEPTH_REVALIDATED` requires the depth sensor bit and every
  item to be `unknown_low_obstacle`, `MOTION_STATIC`, and `static_confirmed`.
- TF lookup and point transformation use `validation_stamp`. Rejection or TF
  failure clears the private layer and publishes `applied=false`; active merge
  remains max-only and shadow/off cannot raise a cell.
- Existing `RiskLayerStatus` IDL has no dedicated validation fields. Therefore
  `message_age_ms` carries validation age and `fallback_reason` carries
  `validation_mode`, `source_age_ms`, exact rejection reason, and confirmed
  count.

## Validation

- Fresh Integration interface underlay build: PASS (one package).
- Fresh Module3 `bio_nav_fusion` clean build/install: PASS.
- Colcon gtests: PASS, 21 tests / 0 errors / 0 failures / 0 skipped
  (18 equal-cost/fusion tests plus plugin-loader test reporting).
- Focused profile pytest: PASS, 7 tests.
- `git diff --check`: PASS.
- Final temporary build root: `/tmp/bionav_m3_obstacle_final.3stwVY`.

Commands:

```bash
env PYTHONDONTWRITEBYTECODE=1 pytest -p no:cacheprovider -q \
  ros2_ws/src/robot_bringup/test/test_bio_nav_fusion_profiles.py \
  ros2_ws/src/robot_navigation/test/test_v6_cognitive_profile.py

source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/bionav_m3_obstacle_final.3stwVY/int_log build \
  --base-paths /home/lyb/Workspace/Bio_Nav/worktrees/cognitive-navigation/bio_nav_intergration/ros2_ws/src/bio_nav_interfaces \
  --build-base /tmp/bionav_m3_obstacle_final.3stwVY/int_build \
  --install-base /tmp/bionav_m3_obstacle_final.3stwVY/int_install \
  --packages-select bio_nav_interfaces
source /tmp/bionav_m3_obstacle_final.3stwVY/int_install/setup.bash
colcon --log-base /tmp/bionav_m3_obstacle_final.3stwVY/m3_log build \
  --base-paths ros2_ws/src/bio_nav_fusion \
  --build-base /tmp/bionav_m3_obstacle_final.3stwVY/m3_build \
  --install-base /tmp/bionav_m3_obstacle_final.3stwVY/m3_install \
  --packages-select bio_nav_fusion
colcon test --build-base /tmp/bionav_m3_obstacle_final.3stwVY/m3_build \
  --install-base /tmp/bionav_m3_obstacle_final.3stwVY/m3_install \
  --packages-select bio_nav_fusion
```

## Result and remaining risk

- Verdict: **PASS (code-level contract, clean build, gtest, and focused pytest)**.
- No Isaac, ROS graph, Nav2 lifecycle, live TF, navigation, evidence campaign,
  or formal qualification was run. The TF-failure branch and max-only behavior
  have focused contract coverage, but live Costmap clearing still requires the
  later authorized runtime stage.
