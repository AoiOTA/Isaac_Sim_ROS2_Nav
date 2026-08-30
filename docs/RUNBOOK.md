# V6 Module3 runbook

This runbook contains the current component commands. It does not authorize a
live run. Read [Module3 current state](CURRENT_STATE.md) and the authoritative
[Integration current state](/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration/docs/CURRENT_STATE.md)
before using it.

## Current user stop boundary

Publishing current Module3 is the present stop boundary. Do not start the
outdoor static engineering repetitions, sufficient-Pilot aggregation, formal
freezer, formal dry-run, ROS, or Isaac without renewed authorization. The
commands below document the next production path only. Current counts remain
Pilot `0/18` and formal `0/120`; formal is `NOT_AUTHORIZED`.

## 1. Clean shell and one underlay

Use the same values in every terminal. Source ROS/setup scripts before enabling
shell nounset.

```bash
export V6_DOMAIN=<fresh-domain-0-to-232>
export PATH="/usr/bin:/bin:${PATH}"
hash -r
test "$(readlink -f "$(command -v python3)")" = /usr/bin/python3

export BIO_NAV_INTEGRATION_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_integration
export BIO_NAV_MODULE3_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module3
export BIO_NAV_MODULE2_ROOT=/home/lyb/Workspace/Bio_Nav/worktrees/v6-compute-amcl-dual-odom/bio_nav_module2
export BIO_NAV_MODULE2_CONDA_EXE=/home/lyb/miniconda3/bin/conda

export BIO_NAV_MODULE2_ASSET_ROOT=/mnt/nas_home/Bio_Nav_Data/experiments/assets/module2_v310_runtime_assets_v6_cleanup_d176498b
export BIO_NAV_ROUTE_PRIOR_SNAPSHOT=/mnt/nas_home/Bio_Nav_Data/experiments/assets/v6_kujiale_isaacgen_v1_sr_snapshot_d7db461171893953
export BIO_NAV_ROUTE_PRIOR_CATALOG=/mnt/nas_home/Bio_Nav_Data/experiments/assets/rivermark_a_srdr_tile_catalog_v1
export RIVERMARK_USD=/mnt/nas_home/Bio_Nav_Data/experiments/assets/rivermark_plaza_v6_final_20260829/rivermark.usd

source "${BIO_NAV_INTEGRATION_ROOT}/env/v6_pilot_setup.sh" "${V6_DOMAIN}"
cd "${BIO_NAV_MODULE3_ROOT}"
```

Before a data-producing attempt, select an absent NAS root and a short absent
socket path:

```bash
export BIO_NAV_RUN_ROOT=/mnt/nas_home/Bio_Nav_Data/experiments/pilots/<new-run-id>
export BIO_NAV_PHASE_F_SOCKET=/tmp/bio_nav_v6_${UID}_${V6_DOMAIN}.sock
test ! -e "${BIO_NAV_RUN_ROOT}"
test ! -e "${BIO_NAV_PHASE_F_SOCKET}"
```

Stop if NAS is unavailable. Do not redirect bags into the repository or `/tmp`.

## 2. Provenance and idle checks

Require all three repositories to be on `v6-compute-amcl-dual-odom`, tracked
clean, and `0/0` relative to upstream. Check live `ls-remote`, not only the
cached remote-tracking ref.

Source the environment above and verify:

```bash
ros2 pkg prefix bio_nav_interfaces
ros2 pkg prefix bio_nav_ros_bridge
ros2 pkg prefix robot_experiments
python3 - <<'PY'
import robot_experiments.experiment_runner as runner
print(runner.__file__)
PY
```

The first two paths must resolve under the canonical Integration worktree; the
last two must resolve under canonical Module3. No product owner or GPU compute
process may already be running.

## 3. Startup ordering

T1 must start before T2. Before T2, wait only for
`/wheel_odometry/reset`; do not wait for EKF `/set_pose`. The mixed EKF
advertises `/set_pose` only after Isaac bootstraps `/clock`, and Isaac's
bounded fail-closed startup reset discovers the required reset services.
T2-first therefore fails by design. T3 always starts last, after observable
readiness.

Rivermark uses a 240 s fail-closed activation timeout because cold USD/RTX
startup has exceeded 120 s. This is an upper bound, not a fixed sleep, and does
not relax any readiness predicate.

## 4. Indoor engineering reference — not current Pilot

The commands in this section predate the current condition-stack attestation
contract. They remain an engineering reference only and must not be used for a
counted sufficient-Pilot episode. The unified six-condition Pilot procedure
will be documented separately after its entrypoint is ready.

Terminal T1:

```bash
./scripts/run_v6_low_obstacle_phase_f_stack.sh M3 \
  --domain "${V6_DOMAIN}" \
  --run-dir "${BIO_NAV_RUN_ROOT}/runtime" \
  --socket "${BIO_NAV_PHASE_F_SOCKET}" \
  --module2-root "${BIO_NAV_MODULE2_ROOT}" \
  --module2-asset-root "${BIO_NAV_MODULE2_ASSET_ROOT}" \
  --enable-route-prior \
  --route-prior-snapshot "${BIO_NAV_ROUTE_PRIOR_SNAPSHOT}"
```

Terminal T2, after the ROS reset services exist:

```bash
./scripts/run_v6_kujiale_low_obstacles.sh --condition static isaac
```

Indoor keeps mixed Compute Odometry plus AMCL.

Terminal T3, after T1 and T2 are READY, is the only indoor episode command:

```bash
export BIO_NAV_REP=rep1  # then rep2 and rep3, without restarting T1 or T2
case "${BIO_NAV_REP}" in
  rep1) BIO_NAV_RUN_INDEX=1 ;;  # seed 8601, cold
  rep2) BIO_NAV_RUN_INDEX=2 ;;  # seed 8602, hot reset
  rep3) BIO_NAV_RUN_INDEX=3 ;;  # seed 8603, hot reset
  *) echo "BIO_NAV_REP must be rep1, rep2, or rep3" >&2; exit 2 ;;
esac
if [[ -e "${BIO_NAV_RUN_ROOT}/${BIO_NAV_REP}" ]]; then
  echo "refusing to reuse episode output: ${BIO_NAV_RUN_ROOT}/${BIO_NAV_REP}" >&2
  exit 2
fi

./scripts/run_experiment.sh \
  ros2_ws/src/robot_experiments/config/v6_final_kujiale_static.yaml \
  "${BIO_NAV_RUN_ROOT}/${BIO_NAV_REP}" \
  nav2_profile:=v6_low_obstacle_isolation \
  nav2_config_file:="${BIO_NAV_MODULE3_ROOT}/ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml" \
  navigation_execution_backend:=route_guided \
  require_module2_planning_ready:=true \
  module2_planning_ready_timeout_sec:=120.0 \
  record_evidence:=true \
  record_bag:=true \
  clear_slam_localization_buffer:=false \
  reset_map_base_translation_tolerance_m:=0.1 \
  run_indices:="${BIO_NAV_RUN_INDEX}" \
  resume:=false
```

For engineering use, inspect `rep1` `episode_validity`, topic coverage,
RoutePrior requested/applied counts, ContactSensor result, terminal-zero, and
checksums before continuing. `rep2` and `rep3` are hot resets on that same T1/T2
stack; changing the stack turns them into new cold runs and invalidates the
three-repetition sequence. None of these commands count toward current Pilot.

## 5. Outdoor Rivermark startup — PASSED FOR STATIC ENGINEERING

The descriptor-set value `20000` is the sole current Rivermark setting. The
authoritative campaign root is
`/mnt/nas_home/Bio_Nav_Data/experiments/pilots/v6_rivermark_descriptor_ab_33136fa2_20260830T105641Z`:
A1/A2/A3 produced same-location pre-READY page faults and Xid 109, so A passed
`0/3` at `33136fa2`; B1/B2/B3 ran `d62f482`, reported
`requested=20000 applied=20000`, reached complete READY with fixed TF and no
AMCL, held `603/604/603 s`, and passed `3/3` with zero kernel faults. The B tree
is identical to integrated `7ba6816`, which current `bb4e78d` contains. This
permits outdoor static engineering only and is not a driver or lower-level
root-cause claim.

Terminal T1:

```bash
./scripts/run_v6_low_obstacle_phase_f_stack.sh M3 \
  --domain "${V6_DOMAIN}" \
  --run-dir "${BIO_NAV_RUN_ROOT}/runtime" \
  --socket "${BIO_NAV_PHASE_F_SOCKET}" \
  --scene rivermark \
  --condition static \
  --module2-root "${BIO_NAV_MODULE2_ROOT}" \
  --module2-asset-root "${BIO_NAV_MODULE2_ASSET_ROOT}" \
  --route-prior-catalog-root "${BIO_NAV_ROUTE_PRIOR_CATALOG}"
```

Terminal T2, after only `/wheel_odometry/reset` exists (do not wait for EKF
`/set_pose` before Isaac starts):

```bash
/usr/bin/python3 "${BIO_NAV_MODULE3_ROOT}/scripts/wait_for_empty_service.py" \
  --service /wheel_odometry/reset \
  --timeout 120
./scripts/run_v6_rivermark.sh isaac static --headless
```

The wrapper fixes the original map/route/regions, mixed Compute Odometry,
calibrated fixed `map -> odom`, Module2/GVG/RoutePrior M3 chain, RGB-D profile,
Rivermark-only DLSS disable, and `--rtx-descriptor-sets 20000`. Caller overrides
are rejected. The startup log must report
`RTX_DESCRIPTOR_SETS requested=20000 applied=20000`.

The startup discriminator passes only when Isaac and Nav2 are READY, no DLSS
internal-upscale warning or GPU page fault/device-lost occurs, and live RGB,
depth, CameraInfo, scan, odom, GT, fixed TF, Module2, and catalog identities are
current. A startup-only discriminator holds a bounded stability window and then
stops without an episode. After separate renewed authorization for outdoor
static engineering, continue to Section 6 on that same attested stack instead
of restarting it.

## 6. Outdoor static engineering runner — FUTURE / CURRENTLY STOPPED

After renewed authorization, Terminal T3 may run outdoor static engineering
`3/3` only after T1/T2 meet the complete current startup contract above. This
is not sufficient Pilot or formal execution; those counts remain `0/18` and
`0/120`, and formal remains `NOT_AUTHORIZED`.

`run_experiment.sh` otherwise falls back to the Kujiale spawn manifest, which
does not define `rivermark_start`.

```bash
export ISAAC_NAV_SPAWN_POSES="${BIO_NAV_MODULE3_ROOT}/data/rivermark_demo/rivermark.spawn.yaml"
test -f "${ISAAC_NAV_SPAWN_POSES}"
export BIO_NAV_REP=rep1  # then rep2 and rep3 without restarting T1 or T2
case "${BIO_NAV_REP}" in
  rep1) BIO_NAV_RUN_INDEX=1 ;;
  rep2) BIO_NAV_RUN_INDEX=2 ;;
  rep3) BIO_NAV_RUN_INDEX=3 ;;
  *) echo "BIO_NAV_REP must be rep1, rep2, or rep3" >&2; exit 2 ;;
esac
test ! -e "${BIO_NAV_RUN_ROOT}/${BIO_NAV_REP}"

export BIO_NAV_STACK_CONTRACT="${BIO_NAV_RUN_ROOT}/runtime/stack.contract.json"
test -r "${BIO_NAV_STACK_CONTRACT}"
export BIO_NAV_STACK_SESSION_ID="$(
  /usr/bin/python3 - "${BIO_NAV_STACK_CONTRACT}" <<'PY'
import json
from pathlib import Path
import re
import sys

path = Path(sys.argv[1]).resolve()
payload = json.loads(path.read_text(encoding="utf-8"))
if payload.get("condition_id") != "outdoor_static":
    raise SystemExit("stack contract condition_id must be outdoor_static")
session = payload.get("stack_session_id")
if not isinstance(session, str) or re.fullmatch(r"[0-9a-f]{64}", session) is None:
    raise SystemExit("stack contract stack_session_id must be a SHA-256 digest")
print(session)
PY
)"

./scripts/run_experiment.sh \
  ros2_ws/src/robot_experiments/config/final_rivermark_static.yaml \
  "${BIO_NAV_RUN_ROOT}/${BIO_NAV_REP}" \
  nav2_profile:=v6_low_obstacle_isolation \
  nav2_config_file:="${BIO_NAV_MODULE3_ROOT}/ros2_ws/src/robot_navigation/config/nav2_v6_low_obstacle_isolation.yaml" \
  navigation_execution_backend:=route_guided \
  require_module2_planning_ready:=true \
  record_evidence:=true \
  record_bag:=true \
  clear_slam_localization_buffer:=false \
  reset_map_base_translation_tolerance_m:=0.15 \
  condition_stack_id:=outdoor_static \
  stack_session_id:="${BIO_NAV_STACK_SESSION_ID}" \
  condition_stack_contract_path:="${BIO_NAV_STACK_CONTRACT}" \
  run_indices:="${BIO_NAV_RUN_INDEX}" \
  resume:=false
```

Omit `experiment_arm`; the Phase-F stack already owns the M3 arm. Run index 1
in fresh `rep1`, index 2 in `rep2`, and index 3 in `rep3` on one attested stack.
The stack contract starts with sequence `0` and startup reset generation
baseline `1`; the three episode receipts must therefore bind cold
sequence/generation `1/2`, then hot `2/3` and `3/4`, without restarting T1 or
T2. Leave `formal_freeze_digest` empty: engineering attests the stack identity
but has no formal freeze. At episode start, the runner copies the live contract
into that episode's run root as `stack_contract.json`; this per-episode snapshot
is the authoritative aggregate/freezer input and survives later owned cleanup
of `runtime/stack.contract.json`. Stop after the first valid product failure or
invalid episode. These are three engineering repetitions and do not pre-count
toward sufficient Pilot.

## 7. Readiness essentials

- exactly one `/odom` publisher and one `odom -> base_link` owner;
- indoor: AMCL alone owns `map -> odom`;
- outdoor: AMCL absent and `ideal_localization_tf` alone owns `map -> odom`;
- `/bio_nav/module1/odom` fresh with no Module1 TF;
- all required Nav2 lifecycle nodes active;
- ResetStopGate released for the current generation before dispatch;
- Module2 socket/READY/current 0.10 m depth config;
- current RoutePrior snapshot or Rivermark tile catalog;
- cognitive obstacle layer and critic active; raw depth voxel plugin absent;
- exactly the selected condition contract: low obstacle for static/appearance,
  or the LiDAR-visible G2 crossing obstacle for dynamic. Dynamic results do not
  validate sub-LiDAR obstacle perception.

Topic names alone are insufficient; inspect fresh messages, QoS, timestamps,
TF values, and actual process/package provenance.

## 8. Owned shutdown

Stop in reverse dispatch order: T3, T2, then T1. Signal only process groups
created by the current run. Stop the current domain daemon if owned, remove only
the current socket/runtime directory, and verify product processes, GPU compute
apps, and owned locks are gone. Never use global `pkill`.

Preserve NAS logs and bags for any failure or invalid attempt. Record invalid
operator/startup runs separately and do not count them as Pilot episodes.

## 9. Future Pilot aggregate, freezer, and formal dry-run — STOPPED

Do not run this section at the current stop boundary. Only after all six future
condition roots contain cold `rep1` plus hot `rep2`/`rep3`, with 18 strict
successes and the required per-episode `stack_contract.json` snapshots, use the
existing wrapper in this exact order. All paths must be absolute, under the NAS
formal root, and the output paths must not already exist.

First build the sufficient-Pilot manifest and aggregate pair:

```bash
./scripts/run_v6_formal_episode.sh --aggregate-pilot \
  /absolute/path/to/PILOT_ROOT \
  /absolute/path/to/OUT_PILOT_MANIFEST.json \
  /absolute/path/to/OUT_PILOT_AGGREGATE.json
```

Only a successful `18/18` aggregate may be frozen into a still-unauthorized
formal manifest:

```bash
./scripts/run_v6_formal_episode.sh --freeze-pilot \
  /absolute/path/to/OUT_PILOT_MANIFEST.json \
  /absolute/path/to/OUT_PILOT_AGGREGATE.json \
  /absolute/path/to/OUT_FORMAL_MANIFEST.json \
  /absolute/path/to/NEW_FORMAL_OUTPUT_ROOT
```

Then, and still without dispatching ROS or Isaac, validate the frozen manifest
and report its digest and `0/120` progress:

```bash
./scripts/run_v6_formal_episode.sh --formal \
  /absolute/path/to/OUT_FORMAL_MANIFEST.json
```

The formal manifest contains exactly six ordered conditions and 20 run
identities per condition. Repeated seeds across dynamic or appearance variants
are valid; identity is the full run index, seed, case/variant, and appearance
profile tuple. It also freezes the readable source-direct
`scripts/run_experiment.sh` entrypoint, three clean repository HEADs,
driver/kernel, six scenario/config sets, active checkpoints, RoutePrior arrays,
Rivermark assets, maps, and formal evaluator sources. Dry-run validates that
freeze and reports its digest, immutable-evidence
aggregate state and the next resume point without requiring a live stack or
starting ROS or Isaac. The freezer keeps execution `NOT_AUTHORIZED`; neither a
successful aggregate nor dry-run grants permission to execute formal episodes.

Each dispatch-plan row states `cold` for rep1 and `hot_reset` for reps 2-20,
and requires the matching condition stack to be owned outside the episode
runner. Do not restart that stack between hot reps. Actual dispatch requires
both an `AUTHORIZED` manifest and the explicit command below:

```bash
./scripts/run_v6_formal_episode.sh --formal --execute-formal \
  --condition-stack-id indoor_static \
  --condition-stack-contract "${BIO_NAV_RUN_ROOT}/runtime/stack.contract.json" \
  /absolute/path/to/OUT_FORMAL_MANIFEST.json
```

The stack id must name the condition actually running in T1/T2. One invocation
dispatches at most that condition's next single episode and then exits; switch
the external stack explicitly before selecting a different condition. Current
authorization is `NOT_AUTHORIZED`; do not run the execution form.
