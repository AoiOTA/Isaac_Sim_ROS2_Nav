#!/usr/bin/env bash
# Run exactly one fresh dynamic route for contact telemetry observability.
set -Eeuo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd "${script_dir}/.." && pwd)"
campaign_id="${1:-20260730-contact-observability-10401}"
branch="codex/module3-contact-observability"
scenario="${root}/ros2_ws/src/robot_experiments/config/kujiale_contact_observability_dynamic.yaml"
output="${root}/data/metrics/contact_observability_${campaign_id}"

[[ "${campaign_id}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]] || { echo "invalid campaign id" >&2; exit 2; }
[[ "$(git -C "${root}" branch --show-current)" == "${branch}" ]] || { echo "Module3 branch mismatch" >&2; exit 2; }
[[ -z "$(git -C "${root}" status --porcelain)" ]] || { echo "Module3 worktree must be completely clean" >&2; exit 2; }
[[ -f "${scenario}" && ! -e "${output}" ]] || { echo "scenario missing or output already exists" >&2; exit 2; }

set +u
source /opt/ros/jazzy/setup.bash
source "${root}/ros2_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

isaac_pid=""
nav2_pid=""
stop_tree() {
  local root_pid="$1" group
  [[ -z "${root_pid}" ]] && return 0
  local -a groups=()
  mapfile -t groups < <(ps -eo pid=,ppid=,pgid= | awk -v root="${root_pid}" '
    { parent[$1]=$2; pgid[$1]=$3 }
    function descendant(pid, current) {
      current=pid
      while (current in parent) {
        if (current == root || parent[current] == root) return 1
        current=parent[current]
      }
      return 0
    }
    END { for (pid in parent) if (pid == root || descendant(pid)) print pgid[pid] }
  ' | sort -u)
  for group in "${groups[@]}"; do kill -INT -- "-${group}" 2>/dev/null || true; done
  sleep 3
  for group in "${groups[@]}"; do kill -0 -- "-${group}" 2>/dev/null && kill -TERM -- "-${group}" 2>/dev/null || true; done
  sleep 2
  for group in "${groups[@]}"; do kill -0 -- "-${group}" 2>/dev/null && kill -KILL -- "-${group}" 2>/dev/null || true; done
  wait "${root_pid}" 2>/dev/null || true
}
cleanup() {
  stop_tree "${nav2_pid}"
  stop_tree "${isaac_pid}"
}
trap cleanup EXIT INT TERM HUP

require_empty_graph() {
  [[ -z "$(ros2 node list 2>/dev/null || true)" ]]
}
wait_for() {
  local label="$1"; shift
  local deadline=$((SECONDS + 180))
  while (( SECONDS < deadline )); do "$@" && return 0; sleep 1; done
  echo "timed out waiting for ${label}" >&2
  return 1
}

require_empty_graph || { echo "ROS graph must be empty" >&2; exit 2; }
mkdir -p "${output}"
setsid "${root}/scripts/run_kujiale_4x20_isaac.sh" dynamic --headless >"${output}/isaac.log" 2>&1 &
isaac_pid=$!
setsid "${root}/scripts/run_ros.sh" navigation odometry_mode:=ideal spawn_pose_name:=long_route_start_g1 nav2_profile:=dynamic_avoidance interactive:=false use_rviz:=false >"${output}/nav2.log" 2>&1 &
nav2_pid=$!
wait_for dynamic_preflight "${root}/scripts/run_kujiale_4x20.sh" preflight dynamic
"${root}/scripts/run_experiment.sh" "${scenario}" "${output}/evidence" run_indices:=1 nav2_profile:=dynamic_avoidance

run_root="${output}/evidence/kujiale_contact_observability_dynamic/run-0001-seed-10401"
[[ -f "${run_root}/run_manifest.json" && -f "${run_root}/run_summary.json" ]] || { echo "single-route evidence is incomplete" >&2; exit 1; }
telemetry="$(find "${run_root}/telemetry" -maxdepth 1 -name '*.mcap' -print -quit)"
[[ -n "${telemetry}" ]] || { echo "telemetry MCAP missing" >&2; exit 1; }
rg -q 'name: /simulation/collision_diagnostics' "${run_root}/telemetry/metadata.yaml" || { echo "contact diagnostic topic missing from MCAP" >&2; exit 1; }
python3 - "${run_root}" "${telemetry}" "${output}/diagnostic_receipt.json" <<'PY'
import hashlib, json, sys
from pathlib import Path

root, telemetry, output = map(Path, sys.argv[1:])
manifest = json.loads((root / "run_manifest.json").read_text(encoding="utf-8"))
summary = json.loads((root / "run_summary.json").read_text(encoding="utf-8"))
value = {
    "schema": "bio_nav_contact_observability_receipt_v1",
    "pass": bool(summary["data_complete"] and summary["checksums_verified"]),
    "run_index": manifest["run_index"],
    "seed": manifest["random_seed"],
    "physical_collision_free": summary["physical_collision_free"],
    "telemetry_mcap_sha256": hashlib.sha256(telemetry.read_bytes()).hexdigest(),
    "run_manifest_sha256": hashlib.sha256((root / "run_manifest.json").read_bytes()).hexdigest(),
    "run_summary_sha256": hashlib.sha256((root / "run_summary.json").read_bytes()).hexdigest(),
}
if value["run_index"] != 1 or value["seed"] != 10401:
    raise SystemExit("unexpected diagnostic route identity")
output.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
if not value["pass"]:
    raise SystemExit("diagnostic evidence failed integrity checks")
PY
echo "contact observability diagnostic completed: ${output}"
