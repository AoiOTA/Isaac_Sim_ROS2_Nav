#!/usr/bin/env bash
# Run exactly one isolated G2 dynamic-safety development smoke.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

smoke_id="${1:-$(date +%Y%m%d-%H%M%S)}"
[[ "${smoke_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || die "invalid smoke id: ${smoke_id}"

scenario="${PROJECT_ROOT}/ros2_ws/src/robot_experiments/config/kujiale_g2_dynamic_safety_smoke.yaml"
profile="${PROJECT_ROOT}/ros2_ws/src/robot_navigation/config/nav2_dynamic_avoidance.yaml"
output_root="${PROJECT_ROOT}/data/experiment_runs/g2_dynamic_safety_smoke_${smoke_id}"
control_root="${output_root}/orchestrator"
isaac_pid=""
ros_pid=""

[[ ! -e "${output_root}" ]] || die "refusing to overwrite smoke evidence: ${output_root}"
mkdir -p "${control_root}"

stop_stack() {
  local status=0
  local child_pid child_pgid
  # Both launch wrappers create dedicated sessions.  Interrupting only their
  # supervisor PID leaves the actual Isaac/ros2 child in its own process group
  # and can strand a failed smoke stack.  Signal the known direct child group
  # first, then let the wrapper reap it normally.
  for child_pid in $(pgrep -P "${ros_pid}" 2>/dev/null || true); do
    child_pgid="$(ps -o pgid= -p "${child_pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ "${child_pgid}" =~ ^[1-9][0-9]*$ ]] \
      && kill -INT -- "-${child_pgid}" 2>/dev/null || true
  done
  for child_pid in $(pgrep -P "${isaac_pid}" 2>/dev/null || true); do
    child_pgid="$(ps -o pgid= -p "${child_pid}" 2>/dev/null | tr -d '[:space:]')"
    [[ "${child_pgid}" =~ ^[1-9][0-9]*$ ]] \
      && kill -INT -- "-${child_pgid}" 2>/dev/null || true
  done
  if [[ -n "${ros_pid}" ]] && kill -0 "${ros_pid}" 2>/dev/null; then
    kill -INT "${ros_pid}" 2>/dev/null || true
    wait "${ros_pid}" || status=$?
  fi
  if [[ -n "${isaac_pid}" ]] && kill -0 "${isaac_pid}" 2>/dev/null; then
    kill -INT "${isaac_pid}" 2>/dev/null || true
    wait "${isaac_pid}" || status=$?
  fi
  return 0
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  stop_stack
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

"${SCRIPT_DIR}/run_kujiale_4x20_isaac.sh" dynamic --headless \
  >"${control_root}/isaac.log" 2>&1 &
isaac_pid=$!
"${SCRIPT_DIR}/run_ros.sh" navigation odometry_mode:=ideal \
  spawn_pose_name:=long_route_start_g1 nav2_profile:=dynamic_avoidance \
  "nav2_profile_params_file:=${profile}" interactive:=false use_rviz:=false \
  >"${control_root}/nav2.log" 2>&1 &
ros_pid=$!

deadline=$((SECONDS + 900))
while (( SECONDS < deadline )); do
  kill -0 "${isaac_pid}" 2>/dev/null || die "Isaac exited; inspect ${control_root}/isaac.log"
  kill -0 "${ros_pid}" 2>/dev/null || die "Nav2 exited; inspect ${control_root}/nav2.log"
  if "${SCRIPT_DIR}/run_kujiale_4x20.sh" preflight dynamic >"${control_root}/preflight.log" 2>&1 \
      && ros2 param get /controller_server FollowPath.CostCritic.cost_weight 2>/dev/null | grep -Eq '4(\.0+)?' \
      && ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius 2>/dev/null | grep -Eq '0\.75'; then
    break
  fi
  sleep 5
done
(( SECONDS < deadline )) || die "smoke stack did not reach the repaired dynamic contract; inspect ${control_root}/preflight.log"

"${SCRIPT_DIR}/run_experiment.sh" "${scenario}" "${output_root}/evidence" \
  nav2_profile:=dynamic_avoidance >"${control_root}/runner.log" 2>&1

python3 - "${output_root}/evidence" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
manifests = list(root.rglob("run_manifest.json"))
if len(manifests) != 1:
    raise SystemExit(f"expected exactly one smoke manifest, found {len(manifests)}")
manifest = json.loads(manifests[0].read_text(encoding="utf-8"))
interaction = manifest.get("dynamic_interaction", {})
clearances = interaction.get("minimum_clearance_m_by_actor", {})
required = {"local_bypass_actor", "g2_g3_exit_actor", "g5_g1_crossing_actor"}
problems = []
if manifest.get("result") != "success":
    problems.append(f"result={manifest.get('result')!r}: {manifest.get('failure_reason')!r}")
if interaction.get("complete") is not True:
    problems.append("dynamic interaction is incomplete")
if set(interaction.get("triggered_ids", [])) != required:
    problems.append("unexpected triggered actor IDs")
if set(interaction.get("retired_ids", [])) != required:
    problems.append("unexpected retired actor IDs")
if interaction.get("guard_aborted") is not False:
    problems.append("actor guard aborted")
clearance = clearances.get("local_bypass_actor")
if not isinstance(clearance, (float, int)) or clearance < 0.10:
    problems.append(f"local_bypass_actor clearance={clearance!r} is below 0.10 m")
if problems:
    raise SystemExit("G2 dynamic-safety smoke failed: " + "; ".join(problems))
print(json.dumps({"result": "pass", "manifest": str(manifests[0]),
                  "local_bypass_clearance_m": clearance}, sort_keys=True))
PY
