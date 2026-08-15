#!/usr/bin/env bash
set -Eeuo pipefail

module3_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
integration_root="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/final-indoor-outdoor-navigation}"
scenario_revision="${RIVERMARK_SCENARIO_REVISION:-attempt31_rivermark}"
fail_stop="${RIVERMARK_FAIL_STOP:-0}"
usage() {
  echo "usage: $0 static|dynamic|appearance off|sr_medium|dr_medium|medium [--run-indices 1,2] [--output DIR] [--no-bag]"
}
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if (( $# < 2 )); then
  usage >&2
  exit 2
fi
condition="$1"
arm="$2"
shift 2

run_indices=""
output_directory=""
record_bag="true"
startup_timeout_sec=180
controller_max_linear_velocity_mps="0.75"
controller_linear_velocity_std_mps="0.35"
rendering_hz="30"
while (($#)); do
  case "$1" in
    --run-indices) shift; run_indices="${1:-}" ;;
    --output) shift; output_directory="${1:-}" ;;
    --no-bag) record_bag="false" ;;
    --startup-timeout-sec) shift; startup_timeout_sec="${1:-}" ;;
    -h|--help)
      usage
      exit 0
      ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

case "${condition}" in static|dynamic|appearance) ;; *) echo "condition must be static, dynamic or appearance" >&2; exit 2 ;; esac
case "${arm}" in off|sr_medium|dr_medium|medium) ;; *) echo "arm must be off, sr_medium, dr_medium or medium" >&2; exit 2 ;; esac
[[ "${startup_timeout_sec}" =~ ^[1-9][0-9]*$ ]] || { echo "startup timeout must be positive" >&2; exit 2; }
[[ "${fail_stop}" == "0" || "${fail_stop}" == "1" ]] || { echo "RIVERMARK_FAIL_STOP must be 0 or 1" >&2; exit 2; }

scenario_file="${module3_root}/ros2_ws/src/robot_experiments/config/${scenario_revision}_${condition}.yaml"
spawn_file="${module3_root}/data/rivermark_demo/rivermark.spawn.yaml"
[[ -f "${scenario_file}" ]] || { echo "missing ${scenario_file}" >&2; exit 2; }
[[ -f "${spawn_file}" ]] || { echo "missing ${spawn_file}" >&2; exit 2; }
if [[ -z "${output_directory}" ]]; then
  output_directory="${module3_root}/data/experiment_runs/${scenario_revision}/${condition}/${arm}"
fi
mkdir -p "${output_directory}/orchestrator"

filter_runtime_log() {
  awk '
    /PopulatePointInstancerBucket invalid protoIndex=/ {suppressed += 1; next}
    {print; fflush()}
    END {
      if (suppressed > 0) {
        print "[orchestrator] suppressed " suppressed \
          " repeated Hydra point-instancer warnings"
      }
    }
  '
}

domain_id="${ROS_DOMAIN_ID:-232}"
if [[ ! "${domain_id}" =~ ^[0-9]+$ ]] || (( domain_id > 232 )); then
  echo "ROS_DOMAIN_ID must be an integer in [0, 232], got ${domain_id}" >&2
  exit 2
fi
demo_mode="off"
guidance_profile=""
if [[ "${arm}" != "off" ]]; then
  demo_mode="module2"
  guidance_profile="${arm}"
fi
isaac_condition="static"
[[ "${condition}" == "dynamic" ]] && isaac_condition="dynamic"
obstacle_config="${module3_root}/data/rivermark_demo/rivermark_dynamic.yaml"
physical_obstacles="0"
if [[ "${scenario_revision}" == "final_rivermark" ]]; then
  if [[ "${condition}" == "static" ]]; then
    obstacle_config="${module3_root}/data/rivermark_demo/final_rivermark_static_obstacles.yaml"
    physical_obstacles="1"
  elif [[ "${condition}" == "dynamic" ]]; then
    obstacle_config="${module3_root}/data/rivermark_demo/final_rivermark_dynamic.yaml"
    physical_obstacles="1"
  fi
fi

demo_pid=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ -n "${demo_pid}" ]] && kill -0 "${demo_pid}" 2>/dev/null; then
    kill -TERM "${demo_pid}" 2>/dev/null || true
    wait "${demo_pid}" || true
  fi
  exit "${status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

ROS_DOMAIN_ID="${domain_id}" \
RIVERMARK_HEADLESS=1 \
RIVERMARK_AUTO_GOAL=0 \
RIVERMARK_GROUND_TRUTH=1 \
RIVERMARK_GUIDANCE_PROFILE="${guidance_profile}" \
RIVERMARK_MAX_LINEAR_SPEED_MPS="${controller_max_linear_velocity_mps}" \
RIVERMARK_LINEAR_SPEED_STD_MPS="${controller_linear_velocity_std_mps}" \
RIVERMARK_RENDERING_HZ="${rendering_hz}" \
RIVERMARK_OBSTACLE_CONFIG="${obstacle_config}" \
RIVERMARK_PHYSICAL_OBSTACLES="${physical_obstacles}" \
BIO_NAV_INTEGRATION_ROOT="${integration_root}" \
"${module3_root}/scripts/run_rivermark_demo.sh" "${demo_mode}" "${isaac_condition}" \
  > >(filter_runtime_log >>"${output_directory}/orchestrator/runtime.log") 2>&1 &
demo_pid=$!

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH ROS_PACKAGE_PATH
set +u
source /opt/ros/jazzy/setup.bash
source "${integration_root}/ros2_ws/install/local_setup.bash"
source "${module3_root}/ros2_ws/install/local_setup.bash"
set -u
export ROS_DOMAIN_ID="${domain_id}"

read_grid_dimensions() {
  local topic="$1"
  local field="$2"
  local width_key="$3"
  local height_key="$4"
  local document=""
  document="$(
    timeout 5 ros2 topic echo "${topic}" --once --field "${field}" \
      2>/dev/null || true
  )"
  local width height
  width="$(awk -v key="${width_key}:" '$1 == key {print $2; exit}' <<<"${document}")"
  height="$(awk -v key="${height_key}:" '$1 == key {print $2; exit}' <<<"${document}")"
  printf '%s %s\n' "${width:-0}" "${height:-0}"
}

runtime_geometry_ready() {
  read -r map_width map_height < <(
    read_grid_dimensions /map info width height
  )
  read -r global_width global_height < <(
    read_grid_dimensions /global_costmap/costmap_raw metadata size_x size_y
  )
  [[ "${map_width}" == "1600" && "${map_height}" == "1600" \
      && "${global_width}" == "1600" && "${global_height}" == "1600" ]]
}

deadline=$((SECONDS + startup_timeout_sec))
while (( SECONDS < deadline )); do
  kill -0 "${demo_pid}" 2>/dev/null || {
    tail -n 100 "${output_directory}/orchestrator/runtime.log" >&2 || true
    echo "Rivermark runtime exited during startup" >&2
    exit 4
  }
  reset_type="$(timeout 5 ros2 service type /simulation/reset 2>/dev/null || true)"
  nav_state="$(timeout 5 ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
  controller_state="$(timeout 5 ros2 lifecycle get /controller_server 2>/dev/null || true)"
  planner_state="$(timeout 5 ros2 lifecycle get /planner_server 2>/dev/null || true)"
  collision_state="$(timeout 5 ros2 lifecycle get /collision_monitor 2>/dev/null || true)"
  route_subscriptions="$(timeout 5 ros2 topic info /bio_nav/route_goal 2>/dev/null | awk '/Subscription count:/ {print $3}' || true)"
  gt_type="$(timeout 5 ros2 topic type /ground_truth/odom 2>/dev/null || true)"
  if [[ "${reset_type}" == "std_srvs/srv/Trigger" ]] \
      && [[ "${nav_state}" == active* ]] \
      && [[ "${controller_state}" == active* ]] \
      && [[ "${planner_state}" == active* ]] \
      && [[ "${collision_state}" == active* ]] \
      && [[ "${route_subscriptions:-0}" =~ ^[1-9][0-9]*$ ]] \
      && [[ "${gt_type}" == "nav_msgs/msg/Odometry" ]] \
      && runtime_geometry_ready; then
    break
  fi
  sleep 1
done
if (( SECONDS >= deadline )); then
  tail -n 100 "${output_directory}/orchestrator/runtime.log" >&2 || true
  echo "Rivermark runtime preflight timed out" >&2
  exit 4
fi

# Require another full geometry sample after a wall-clock dwell.  A 2.56 MB
# full global costmap can occasionally miss one CLI read while the first
# cognitive tile is being rasterized, so retry the read-only sample within a
# bounded window.  An actual 240x240 lifecycle regression never satisfies it.
sleep 2
geometry_stable="false"
geometry_deadline=$((SECONDS + 20))
while (( SECONDS < geometry_deadline )); do
  if runtime_geometry_ready; then
    geometry_stable="true"
    break
  fi
  sleep 1
done
if [[ "${geometry_stable}" != "true" ]]; then
  tail -n 120 "${output_directory}/orchestrator/runtime.log" >&2 || true
  echo "Rivermark map/global-costmap geometry did not remain 1600x1600" >&2
  exit 4
fi

read_double_parameter() {
  local node="$1"
  local parameter="$2"
  timeout 5 ros2 param get "${node}" "${parameter}" 2>/dev/null \
    | awk -F': ' '/Double value is:/ {print $2}'
}
observed_vx_max="$(read_double_parameter /controller_server FollowPath.vx_max)"
observed_vx_std="$(read_double_parameter /controller_server FollowPath.vx_std)"
observed_physics_hz="$(read_double_parameter /isaac_navigation_sim physics_hz)"
observed_rendering_hz="$(read_double_parameter /isaac_navigation_sim rendering_hz)"
python3 - \
  "${controller_max_linear_velocity_mps}" \
  "${controller_linear_velocity_std_mps}" \
  "${rendering_hz}" \
  "${observed_vx_max}" "${observed_vx_std}" \
  "${observed_physics_hz}" "${observed_rendering_hz}" \
  "${output_directory}/orchestrator/runtime_controller_contract.json" <<'PY'
import json
import math
from pathlib import Path
import sys

(
    expected_max,
    expected_std,
    expected_rendering_hz,
    observed_max,
    observed_std,
    observed_physics_hz,
    observed_rendering_hz,
) = map(float, sys.argv[1:8])
if not math.isclose(observed_max, expected_max, rel_tol=0.0, abs_tol=1.0e-9):
    raise SystemExit(
        f"controller vx_max mismatch: expected {expected_max}, got {observed_max}"
    )
if not math.isclose(observed_std, expected_std, rel_tol=0.0, abs_tol=1.0e-9):
    raise SystemExit(
        f"controller vx_std mismatch: expected {expected_std}, got {observed_std}"
    )
if not math.isclose(observed_physics_hz, 60.0, rel_tol=0.0, abs_tol=1.0e-9):
    raise SystemExit(
        f"Isaac physics_hz mismatch: expected 60.0, got {observed_physics_hz}"
    )
if not math.isclose(
    observed_rendering_hz,
    expected_rendering_hz,
    rel_tol=0.0,
    abs_tol=1.0e-9,
):
    raise SystemExit(
        "Isaac rendering_hz mismatch: "
        f"expected {expected_rendering_hz}, got {observed_rendering_hz}"
    )
target = Path(sys.argv[8])
target.write_text(
    json.dumps(
        {
            "contract": "attempt31_rivermark_controller_v1",
            "verified": True,
            "FollowPath.vx_max": observed_max,
            "FollowPath.vx_std": observed_std,
            "physics_hz": observed_physics_hz,
            "rendering_hz": observed_rendering_hz,
            "map_size": [1600, 1600],
            "global_costmap_size": [1600, 1600],
        },
        indent=2,
        sort_keys=True,
    )
    + "\n",
    encoding="utf-8",
)
print(
    f"verified Rivermark controller envelope: "
    f"vx_max={observed_max}, vx_std={observed_std}, "
    f"physics_hz={observed_physics_hz}, rendering_hz={observed_rendering_hz}"
)
PY

launch_args=(
  scenario_file:="${scenario_file}"
  spawn_poses_file:="${spawn_file}"
  output_directory:="${output_directory}/runs"
  record_evidence:=true
  record_bag:="${record_bag}"
  resume:=true
  require_successful_resume:="${fail_stop}"
  fail_stop:="${fail_stop}"
  navigation_execution_backend:=route_guided
  nav2_profile:=bio_nav_planning_only
  experiment_arm:="${arm}"
)
if [[ "${scenario_revision}" == "final_rivermark" ]]; then
  launch_args+=(
    fail_stop_metric_contract:="${module3_root}/data/rivermark_demo/final_rivermark_metric_contract.yaml"
  )
fi
if [[ -n "${run_indices}" ]]; then
  launch_args+=(run_indices:="${run_indices}")
fi
expected_run_count="$(python3 - "${scenario_file}" "${run_indices}" <<'PY'
from pathlib import Path
import sys
import yaml

selected = [value for value in sys.argv[2].split(",") if value]
if selected:
    print(len(selected))
else:
    document = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
    runs = document["scenario"]["runs"]
    print(len(runs.get("matrix", runs.get("seeds", []))))
PY
)"
max_runner_attempts=4
for ((runner_attempt = 1; runner_attempt <= max_runner_attempts; runner_attempt++)); do
  printf '[orchestrator] runner attempt %d/%d started at %s\n' \
    "${runner_attempt}" "${max_runner_attempts}" "$(date --iso-8601=seconds)" \
    | tee -a "${output_directory}/orchestrator/runner.log"
  set +e
  ros2 launch robot_experiments experiment.launch.py "${launch_args[@]}" \
    2>&1 | tee -a "${output_directory}/orchestrator/runner.log"
  runner_status="${PIPESTATUS[0]}"
  set -e
  observed_run_count="$(find "${output_directory}/runs" -name run_summary.json -type f 2>/dev/null | wc -l)"
  dispatched_trial_status=0
  python3 - "${output_directory}/runs" <<'PY' || dispatched_trial_status=$?
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
for receipt in sorted(root.rglob("TRIAL_DISPATCHED.json")):
    summary_path = receipt.with_name("run_summary.json")
    if not summary_path.is_file():
        print(
            f"post-dispatch evidence is incomplete; fail-stop: {receipt.parent}",
            file=sys.stderr,
        )
        raise SystemExit(42)
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if not all(
        summary.get(field) is True
        for field in (
            "strict_success",
            "physical_collision_free",
            "data_complete",
            "checksums_verified",
        )
    ):
        print(
            f"post-dispatch trial failed; fail-stop: {summary_path}",
            file=sys.stderr,
        )
        raise SystemExit(42)
    metric_gate = summary.get("final_trial_metric_gate", {})
    if metric_gate.get("applicable") is True and metric_gate.get("passed") is not True:
        print(
            f"post-dispatch Final trial metric gate failed; fail-stop: {summary_path}",
            file=sys.stderr,
        )
        raise SystemExit(42)
PY
  if [[ "${fail_stop}" == "1" && "${dispatched_trial_status}" == "42" ]]; then
    echo "Rivermark formal campaign stopped on immutable post-dispatch evidence" >&2
    exit 6
  fi
  if [[ "${dispatched_trial_status}" != "0" ]]; then
    echo "Rivermark dispatched-trial audit failed unexpectedly: status=${dispatched_trial_status}" >&2
    exit 5
  fi
  if [[ "${observed_run_count}" == "${expected_run_count}" ]]; then
    break
  fi
  if ! kill -0 "${demo_pid}" 2>/dev/null; then
    echo "Rivermark runtime exited before collection completed" >&2
    exit 4
  fi
  if (( runner_attempt == max_runner_attempts )); then
    echo "Rivermark runner exhausted ${max_runner_attempts} bounded resume attempts: expected ${expected_run_count}, got ${observed_run_count}, last_status=${runner_status}" >&2
    exit 5
  fi
  echo "Rivermark runner stopped before collection completed; preserving Isaac/Nav2 and resuming completed=${observed_run_count}/${expected_run_count}, last_status=${runner_status}" \
    | tee -a "${output_directory}/orchestrator/runner.log"
  sleep 2
done

read_integer_parameter() {
  local node="$1"
  local parameter="$2"
  ros2 param get "${node}" "${parameter}" 2>/dev/null \
    | awk -F': ' '/Integer value is:/ {print $2}'
}
cache_entries="$(read_integer_parameter /bio_nav_route_coordinator cognitive_tile_cache_entries)"
cache_hits="$(read_integer_parameter /bio_nav_route_coordinator cognitive_tile_cache_hits)"
cache_misses="$(read_integer_parameter /bio_nav_route_coordinator cognitive_tile_cache_misses)"
python3 - \
  "${cache_entries}" "${cache_hits}" "${cache_misses}" \
  "${output_directory}/orchestrator/runtime_tile_cache_contract.json" <<'PY'
import json
from pathlib import Path
import sys

entries, hits, misses = map(int, sys.argv[1:4])
if min(entries, hits, misses) < 0 or misses < entries:
    raise SystemExit(
        "invalid cognitive tile cache counters: "
        f"entries={entries}, hits={hits}, misses={misses}"
    )
target = Path(sys.argv[4])
target.write_text(
    json.dumps(
        {
            "contract": "attempt31_rivermark_tile_cache_v1",
            "verified": True,
            "entries": entries,
            "hits": hits,
            "misses": misses,
        },
        indent=2,
        sort_keys=True,
    )
    + "\n",
    encoding="utf-8",
)
print(
    "verified Rivermark cognitive tile cache: "
    f"entries={entries}, hits={hits}, misses={misses}"
)
PY

# ros2 launch may itself return zero after a required child exits nonzero.
# This wrapper validates collection completeness and arm binding.  Navigation
# and collision rates are evaluated later by the 20-run fail-closed qualifier.
python3 - "${scenario_file}" "${output_directory}/runs" "${run_indices}" "${arm}" <<'PY'
import json
from pathlib import Path
import sys
import yaml

scenario_file = Path(sys.argv[1])
run_root = Path(sys.argv[2])
selected = [value for value in sys.argv[3].split(",") if value]
arm = sys.argv[4]
document = yaml.safe_load(scenario_file.read_text(encoding="utf-8"))
runs = document["scenario"]["runs"]
configured = runs.get("matrix", runs.get("seeds", []))
expected = len(selected) if selected else len(configured)
paths = sorted(run_root.rglob("run_summary.json"))
if len(paths) != expected:
    raise SystemExit(
        f"Rivermark evidence count mismatch: expected {expected}, got {len(paths)}"
    )
for path in paths:
    summary = json.loads(path.read_text(encoding="utf-8"))
    if not (
        summary.get("data_complete") is True
        and summary.get("checksums_verified") is True
        and summary.get("experiment_arm") == arm
        and [leg.get("id") for leg in summary.get("legs", [])]
        == ["G1", "G2", "G3", "G4", "G5"]
    ):
        raise SystemExit(f"Rivermark run failed collection contract: {path}")
print(json.dumps({
    "collection_complete": True,
    "run_count": len(paths),
    "strict_successes": sum(
        json.loads(path.read_text(encoding="utf-8")).get("strict_success") is True
        for path in paths
    ),
    "collision_free_runs": sum(
        json.loads(path.read_text(encoding="utf-8")).get("physical_collision_free") is True
        for path in paths
    ),
}))
PY
