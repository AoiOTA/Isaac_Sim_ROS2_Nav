# V6 obstacle static source-age contract alignment

## Scope

- Goal: align Module3 obstacle admission with the Integration producer's
  five-second static-observation retention contract.
- Branch/worktree: `cognitive-navigation` in the permitted Module3 worktree.
- Starting Module3 HEAD: `b1e37922a1191c3634881138e5d304620ab3abc6`.
- Result commit: the commit containing this handoff.

## Change

- `VALIDATION_STATIC_DEPTH_REVALIDATED` accepts an exact, nonnegative source
  age through `5.0 s` inclusive and reports `source_age` above that bound.
- `VALIDATION_FRESH` still requires an exact zero source age and reports
  `fresh_mismatch` for a nonzero age.
- Unknown validation modes still report `validation_mode`.
- Validation TTL/future-time, dual odometry timeline, depth mask, identity,
  sequence, obstacle freshness and TF behavior are unchanged.

## Validation

- Isolated `bio_nav_fusion` build: PASS.
- Package tests: PASS, 26 tests / 0 errors / 0 failures / 0 skipped.
- Boundary coverage: static ages `1.99 s`, `2.2 s`, and `4.9 s` accepted;
  `5.01 s` rejected as `source_age`; FRESH `2.2 s` rejected as
  `fresh_mismatch`.
- `git diff --check`: PASS before commit.
- Build/test root: `/tmp/v6_obstacle_static_source_age.RmMSTu`.

## Result and remaining risk

- Verdict: **PASS (implementation/build/unit only)**.
- No ROS, Nav2, Isaac, navigation, evidence or qualification campaign was
  launched. The next authorized live obstacle run must confirm that both
  Costmap consumers admit refreshed static observations older than two seconds
  while preserving the 0.5-second validation freshness requirement.
