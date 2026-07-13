#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
source_ros

query_timeout="${ISAAC_NAV_DIAGNOSE_TIMEOUT:-3}"
[[ "${query_timeout}" =~ ^[0-9]+([.][0-9]+)?$ ]] \
  || die "ISAAC_NAV_DIAGNOSE_TIMEOUT must be numeric"
for command in timeout rg find python3 ros2 ps awk; do
  require_command "${command}"
done

section() {
  printf '\n== %s ==\n' "$1"
}

run_query() {
  local label="$1"
  shift
  printf '\n-- %s --\n' "${label}"
  timeout "${query_timeout}" "$@" 2>&1 || {
    local status=$?
    if [[ ${status} -eq 124 ]]; then
      printf '[timeout after %ss]\n' "${query_timeout}"
    else
      printf '[unavailable, exit=%s]\n' "${status}"
    fi
  }
}

summary_passes=0
summary_warnings=0
summary_failures=0

summary_result() {
  local level="$1" label="$2" message="$3"
  printf '%-4s %-34s %s\n' "${level}" "${label}" "${message}"
  case "${level}" in
    PASS) ((summary_passes += 1)) ;;
    WARN) ((summary_warnings += 1)) ;;
    FAIL) ((summary_failures += 1)) ;;
  esac
}

registered_pid() {
  local component="$1" pid_file pid expected_start actual_start
  local expected_boot actual_boot recorded_root state
  pid_file="$(runtime_pid_file "${component}")"
  [[ -r "${pid_file}" ]] || return 1
  pid="$(sed -n 's/^pid=//p' "${pid_file}" | head -n 1)"
  expected_start="$(sed -n 's/^leader_start_ticks=//p' "${pid_file}" | head -n 1)"
  expected_boot="$(sed -n 's/^boot_id=//p' "${pid_file}" | head -n 1)"
  recorded_root="$(sed -n 's/^project_root=//p' "${pid_file}" | head -n 1)"
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/stat" ]] || return 1
  state="$(awk '{print $3}' "/proc/${pid}/stat" 2>/dev/null || true)"
  actual_start="$(awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null || true)"
  actual_boot="$(< /proc/sys/kernel/random/boot_id)"
  [[ "${state}" != Z && "${recorded_root}" == "${PROJECT_ROOT}" ]] || return 1
  [[ -z "${expected_start}" || "${expected_start}" == "${actual_start}" ]] \
    || return 1
  [[ -z "${expected_boot}" || "${expected_boot}" == "${actual_boot}" ]] \
    || return 1
  printf '%s\n' "${pid}"
}

registered_command() {
  local component="$1" pid
  pid="$(registered_pid "${component}" || true)"
  [[ "${pid}" =~ ^[0-9]+$ && -r "/proc/${pid}/cmdline" ]] || return 1
  tr '\0' ' ' <"/proc/${pid}/cmdline"
}

process_environment_value() {
  local pid="$1" key="$2"
  [[ -r "/proc/${pid}/environ" ]] || return 1
  tr '\0' '\n' <"/proc/${pid}/environ" \
    | sed -n "s/^${key}=//p" \
    | head -n 1
}

publisher_count() {
  topic_endpoint_count "$1" Publisher
}

subscription_count() {
  topic_endpoint_count "$1" Subscription
}

topic_endpoint_count() {
  local topic="$1" kind="$2" output count
  if ! output="$(timeout "${query_timeout}" ros2 topic info "${topic}" 2>/dev/null)"; then
    printf 'unavailable\n'
    return 0
  fi
  count="$(sed -n "s/^${kind} count: //p" <<<"${output}" | head -n 1)"
  if [[ "${count}" =~ ^[0-9]+$ ]]; then
    printf '%s\n' "${count}"
  else
    printf 'unavailable\n'
  fi
}

registered_group_members() {
  local component="$1" pid_file pid process_group
  pid="$(registered_pid "${component}" || true)"
  [[ "${pid}" =~ ^[0-9]+$ ]] || return 0
  pid_file="$(runtime_pid_file "${component}")"
  process_group="$(sed -n 's/^process_group=//p' "${pid_file}" | head -n 1)"
  [[ "${process_group}" =~ ^[0-9]+$ && "${process_group}" == "${pid}" ]] \
    || return 0
  ps -eo pid=,pgid=,stat= | awk -v wanted="${process_group}" \
    '$2 == wanted && $3 !~ /^Z/ { print $1 }'
}

section "Environment"
printf 'PROJECT_ROOT=%s\n' "${PROJECT_ROOT}"
printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO:-unset}"
printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID}"
printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION}"
printf 'ROS_SETUP=%s\n' "${ROS_SETUP}"
printf 'WORKSPACE_SETUP=%s\n' "${PROJECT_ROOT}/ros2_ws/install/setup.bash"

section "Registered runtime"
prepare_runtime_directory
for component in isaac ros rviz teleop; do
  pid_file="$(runtime_pid_file "${component}")"
  if [[ -r "${pid_file}" ]]; then
    printf '%s:\n' "${component}"
    sed 's/^/  /' "${pid_file}"
    pid="$(sed -n 's/^pid=//p' "${pid_file}" | head -n 1)"
    if [[ "${pid}" =~ ^[0-9]+$ ]] && kill -0 "${pid}" 2>/dev/null; then
      printf '  state=alive\n'
      if [[ -r "/proc/${pid}/cmdline" ]]; then
        printf '  command='
        tr '\0' ' ' <"/proc/${pid}/cmdline"
        printf '\n'
      fi
    else
      printf '  state=stale\n'
    fi
  else
    printf '%s: not registered\n' "${component}"
  fi
done

section "Project-related processes"
ps -eo pid=,ppid=,user=,stat=,args= | awk -v root="${PROJECT_ROOT}" -v self="$$" '
  $1 != self && $0 !~ /diagnose[.]sh/ && $0 !~ /awk -v root=/ {
    if (index($0, root "/isaac_sim/apps/navigation_sim.py") ||
        $0 ~ /ros2 launch robot_bringup/ ||
        $0 ~ /robot_teleop.*keyboard_teleop/ ||
        $0 ~ /(^|[[:space:]])rviz2([[:space:]]|$)/) print
  }
' || true

section "ROS graph"
nodes="$(timeout "${query_timeout}" ros2 node list --no-daemon --spin-time 0.5 2>/dev/null || true)"
topics="$(timeout "${query_timeout}" ros2 topic list --no-daemon --spin-time 0.5 2>/dev/null || true)"
services="$(timeout "${query_timeout}" ros2 service list --no-daemon --spin-time 0.5 2>/dev/null || true)"
actions="$(timeout "${query_timeout}" ros2 action list 2>/dev/null || true)"
printf '%s\n' "${nodes:-[no nodes]}"
printf '\nDuplicate node names:\n'
duplicates="$(printf '%s\n' "${nodes}" | sed '/^$/d' | sort | uniq -d)"
printf '%s\n' "${duplicates:-none}"
printf '\nKey topics:\n'
printf '%s\n' "${topics}" | grep -E '^/(clock|map|slam_toolbox/map|scan|lidar/points_raw|odom|tf|tf_static|cmd_vel|cmd_vel_nav|cmd_vel_smoothed|initialpose|goal_pose|global_costmap|local_costmap|collision_monitor)' || true
printf '\nKey services:\n'
printf '%s\n' "${services}" | grep -E '(lifecycle|clear|simulation/reset)' || true
printf '\nKey actions:\n'
printf '%s\n' "${actions}" | grep -E '(navigate_to_pose|navigate_through_poses)' || true

section "Lifecycle"
for node in \
  map_server slam_toolbox controller_server planner_server behavior_server \
  bt_navigator velocity_smoother collision_monitor; do
  if printf '%s\n' "${nodes}" | grep -qx "/${node}"; then
    run_query "/${node}" ros2 lifecycle get "/${node}"
  else
    printf '/%s: not present\n' "${node}"
  fi
done

section "QoS"
for topic in /map /scan /lidar/points_raw /odom /tf /tf_static \
  /global_costmap/costmap /local_costmap/costmap; do
  if printf '%s\n' "${topics}" | grep -qx "${topic}"; then
    run_query "${topic}" ros2 topic info "${topic}" --verbose
  else
    printf '%s: not present\n' "${topic}"
  fi
done

section "Interaction interfaces"
for topic in /initialpose /goal_pose; do
  if printf '%s\n' "${topics}" | grep -qx "${topic}"; then
    run_query "${topic}" ros2 topic info "${topic}" --verbose
  else
    printf '%s: not present\n' "${topic}"
  fi
done
if printf '%s\n' "${actions}" | grep -qx '/navigate_to_pose'; then
  run_query "/navigate_to_pose" ros2 action info /navigate_to_pose
else
  printf '/navigate_to_pose: not present\n'
fi

section "TF"
for pair in 'map odom' 'odom base_link' 'map base_link'; do
  # Jazzy tf2_echo has no --once option, so every query is bounded whether a
  # sample is printed or the transform is missing.
  read -r target source <<<"${pair}"
  run_query "${target} -> ${source}" \
    ros2 run tf2_ros tf2_echo "${target}" "${source}"
done

section "Simulation time"
run_query "/clock sample" ros2 topic echo /clock --once
run_query "/scan header" ros2 topic echo /scan --once --field header
run_query "/odom header" ros2 topic echo /odom --once --field header

section "Fast DDS shared memory"
shm_root="${ISAAC_NAV_SHM_ROOT:-/dev/shm}"
if [[ -d "${shm_root}" ]]; then
  shopt -s nullglob
  shm_files=(
    "${shm_root}"/fastrtps_*
    "${shm_root}"/fastdds_*
    "${shm_root}"/sem.fastrtps_*
    "${shm_root}"/sem.fastdds_*
  )
  shopt -u nullglob
  if ((${#shm_files[@]})); then
    for path in "${shm_files[@]}"; do
      stat -c '%A uid=%u gid=%g size=%s %n' "${path}" 2>/dev/null || true
    done
  else
    printf 'none\n'
  fi
else
  printf 'SHM root unavailable: %s\n' "${shm_root}"
fi

section "CPU governor"
governors=(/sys/devices/system/cpu/cpu*/cpufreq/scaling_governor)
if [[ -e "${governors[0]}" ]]; then
  for governor in "${governors[@]}"; do
    printf '%s=%s\n' "${governor}" "$(<"${governor}")"
  done | sort -u
else
  printf 'unavailable\n'
fi

section "Recent runtime ROS warnings and errors"
current_ros_logs=()
if [[ -d "${HOME}/.ros/log" ]]; then
  for component in isaac ros rviz teleop; do
    while IFS= read -r member; do
      [[ "${member}" =~ ^[0-9]+$ ]] || continue
      while IFS= read -r -d '' path; do
        current_ros_logs+=("${path}")
      done < <(
        find "${HOME}/.ros/log" -maxdepth 1 -type f \
          -name "*_${member}_*.log" -print0 2>/dev/null
      )
    done < <(registered_group_members "${component}")
    leader="$(registered_pid "${component}" || true)"
    [[ "${leader}" =~ ^[0-9]+$ ]] || continue
    while IFS= read -r -d '' path; do
      current_ros_logs+=("${path}")
    done < <(
      find "${HOME}/.ros/log" -mindepth 2 -maxdepth 2 -type f \
        -path "*-${leader}/*" -name '*.log' -print0 2>/dev/null
    )
  done
fi
if ((${#current_ros_logs[@]})); then
  rg -n -i --glob '*.log' --glob '*.txt' '(error|warn|failed|missed.*rate)' \
    "${current_ros_logs[@]}" 2>/dev/null | tail -n 100 || printf 'none\n'
else
  printf 'no logs for the registered ROS runtime\n'
fi

section "Diagnostic summary"
summary_result PASS "shell ROS environment" \
  "domain=${ROS_DOMAIN_ID}, rmw=${RMW_IMPLEMENTATION}"
for component in isaac ros rviz teleop; do
  pid="$(registered_pid "${component}" || true)"
  if [[ ! "${pid}" =~ ^[0-9]+$ ]] || ! kill -0 "${pid}" 2>/dev/null; then
    summary_result WARN "${component} environment" "component is not running"
    continue
  fi
  component_domain="$(process_environment_value "${pid}" ROS_DOMAIN_ID || true)"
  component_rmw="$(process_environment_value "${pid}" RMW_IMPLEMENTATION || true)"
  if [[ "${component_domain}" == "${ROS_DOMAIN_ID}" \
        && "${component_rmw}" == "${RMW_IMPLEMENTATION}" ]]; then
    summary_result PASS "${component} environment" \
      "domain=${component_domain}, rmw=${component_rmw}"
  else
    summary_result FAIL "${component} environment" \
      "domain=${component_domain:-unset}, rmw=${component_rmw:-unset}"
  fi
done

isaac_command="$(registered_command isaac || true)"
ros_command="$(registered_command ros || true)"
rviz_command="$(registered_command rviz || true)"
operation=""
for candidate in mapping incremental_mapping localization navigation; do
  if [[ "${ros_command}" == *"${candidate}_bringup.launch.py"* ]]; then
    operation="${candidate}"
    break
  fi
done
odometry_mode="ideal"
[[ "${isaac_command}" != *"--mode realistic"* ]] || odometry_mode="realistic"
structure_tf_source="isaac"
[[ "${isaac_command}" != *"--structure-tf-source rsp"* ]] \
  || structure_tf_source="rsp"
pacing_mode="realtime"
[[ "${isaac_command}" != *"--pacing-mode unbounded"* ]] \
  || pacing_mode="unbounded"
runtime_mode="gui"
[[ "${isaac_command}" != *"--headless"* ]] || runtime_mode="headless"
camera_profile=""
if [[ "${isaac_command}" =~ --camera-profile[[:space:]]+([^[:space:]]+) ]]; then
  camera_profile="${BASH_REMATCH[1]}"
elif [[ "${runtime_mode}" == headless ]]; then
  camera_profile="off"
else
  camera_profile="monitoring"
fi
if [[ -n "${isaac_command}" ]]; then
  summary_result PASS "runtime modes" \
    "operation=${operation:-none}, odom=${odometry_mode}, tf=${structure_tf_source}, pacing=${pacing_mode}, ${runtime_mode}, camera=${camera_profile}"
else
  summary_result WARN "runtime modes" "Isaac is not registered"
fi
if [[ -n "${rviz_command}" ]]; then
  summary_result PASS "RViz camera display" \
    "RViz is running; integrated configs contain the front Camera dock"
else
  summary_result WARN "RViz camera display" "RViz is not running"
fi

declare -A expected_presence=(
  [/odom]=1
  [/lidar/points_raw]=1
)
for topic in /map /odom /cmd_vel /wheel/odom /transformed_global_plan \
  /camera/front/image_raw /camera/front/camera_info; do
  count="$(publisher_count "${topic}")"
  if [[ ! "${count}" =~ ^[0-9]+$ ]]; then
    summary_result WARN "owner ${topic}" "publisher count unavailable"
  elif ((count > 1)); then
    summary_result FAIL "owner ${topic}" "duplicate publishers=${count}"
  elif [[ "${expected_presence[${topic}]:-0}" == 1 && ${count} -ne 1 ]]; then
    summary_result FAIL "owner ${topic}" "expected one publisher, got ${count}"
  else
    summary_result PASS "owner ${topic}" "publishers=${count}"
  fi
done

image_publishers="$(publisher_count /camera/front/image_raw)"
info_publishers="$(publisher_count /camera/front/camera_info)"
image_subscribers="$(subscription_count /camera/front/image_raw)"
if [[ ! "${image_publishers}" =~ ^[0-9]+$ \
      || ! "${info_publishers}" =~ ^[0-9]+$ \
      || ! "${image_subscribers}" =~ ^[0-9]+$ ]]; then
  summary_result WARN "Camera runtime" "topic endpoint query unavailable"
elif [[ "${camera_profile}" == off ]]; then
  if [[ "${image_publishers}" == 0 && "${info_publishers}" == 0 ]]; then
    summary_result PASS "Camera runtime" "disabled with zero publishers"
  else
    summary_result FAIL "Camera runtime" \
      "profile=off but Image/CameraInfo publishers=${image_publishers}/${info_publishers}"
  fi
elif [[ "${image_publishers}" == 1 && "${info_publishers}" == 1 ]]; then
  if [[ "${image_subscribers}" == 0 ]]; then
    summary_result WARN "Camera runtime" \
      "publishing profile=${camera_profile}, but no Image subscriber"
  else
    summary_result PASS "Camera runtime" \
      "publishing profile=${camera_profile}, Image subscribers=${image_subscribers}"
  fi
else
  summary_result FAIL "Camera runtime" \
    "profile=${camera_profile}, Image/CameraInfo publishers=${image_publishers}/${info_publishers}"
fi

latest_kit_log="$(find \
  "$(dirname "${ISAAC_PYTHON}")/../lib/python3.12/site-packages/isaacsim/kit/logs/Kit/Isaac-Sim Python/6.0" \
  -maxdepth 1 -type f -name 'kit_*.log' -printf '%T@ %p\n' 2>/dev/null \
  | sort -nr | head -n 1 | cut -d' ' -f2- || true)"
log_sources=()
log_sources+=("${current_ros_logs[@]}")
isaac_pid_file="$(runtime_pid_file isaac)"
if [[ -n "$(registered_pid isaac || true)" \
      && -f "${latest_kit_log}" \
      && "${latest_kit_log}" -nt "${isaac_pid_file}" ]]; then
  log_sources+=("${latest_kit_log}")
fi
declare -a log_severities=(
  WARN WARN WARN WARN WARN WARN WARN FAIL FAIL FAIL
)
declare -a log_labels=(
  'control loop missed'
  'failed progress'
  'optimizer reset'
  'collision invalid source'
  'future extrapolation'
  'queue full'
  'old timestamp'
  'pending coroutine'
  'abnormal RViz exit'
  'Camera resource cleanup'
)
declare -a log_patterns=(
  'control loop missed|missed its desired rate'
  'failed to make progress'
  'optimizer reset|resetting optimizer'
  'ignoring the source|invalid source|source timeout'
  'extrapolation into the future'
  'queue is full|message filter dropping'
  'timestamp.*earlier|tf_old_data'
  'coroutine.*never awaited|guard condition'
  'exit code -6|std::terminate'
  'writer attach request|render product.*(invalid|failed|error)'
)
if ((${#log_severities[@]} != ${#log_labels[@]} \
      || ${#log_labels[@]} != ${#log_patterns[@]})); then
  die "diagnostic log rule arrays have inconsistent lengths"
fi
for index in "${!log_patterns[@]}"; do
  severity="${log_severities[${index}]}"
  label="${log_labels[${index}]}"
  pattern="${log_patterns[${index}]}"
  if ((${#log_sources[@]} == 0)); then
    summary_result WARN "log ${label}" "no current runtime logs"
    continue
  fi
  set +e
  match_output="$(rg -i -c "${pattern}" "${log_sources[@]}" 2>&1)"
  match_status=$?
  set -e
  if ((match_status >= 2)); then
    summary_result WARN "log ${label}" "search failed: ${match_output}"
    continue
  fi
  matches="$(awk -F: '{ total += $NF } END { print total + 0 }' \
    <<<"${match_output}")"
  if [[ "${matches}" == 0 ]]; then
    summary_result PASS "log ${label}" "count=0"
  else
    summary_result "${severity}" "log ${label}" "count=${matches}"
  fi
done

latest_profile="$(find "${PROJECT_ROOT}/data/reports" -type f -name '*.json' \
  -printf '%T@ %p\n' 2>/dev/null | sort -nr | head -n 1 \
  | cut -d' ' -f2- || true)"
if [[ -f "${latest_profile}" ]] && profile_summary="$(python3 - "${latest_profile}" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
rtf = report.get("rtf", {}).get("measured")
topics = report.get("topics", {})
scan = topics.get("scan", {}).get("age_s", {}).get("p99")
image = topics.get("camera_image", {}).get("age_s", {}).get("p99")
tf_entry = report.get("tf", {}).get("map->base_link", {})
tf = tf_entry.get("lag_s", tf_entry.get("age_s", {})).get("p99")
print(f"rtf={rtf}, scan_p99_age={scan}, image_p99_age={image}, map_base_p99_lag={tf}")
PY
)"; then
  summary_result PASS "latest runtime profile" \
    "$(basename "${latest_profile}"): ${profile_summary}"
elif [[ -f "${latest_profile}" ]]; then
  summary_result WARN "latest runtime profile" \
    "cannot parse $(basename "${latest_profile}")"
else
  summary_result WARN "latest runtime profile" \
    "run scripts/profile_runtime.sh to create timing evidence"
fi

printf '\nSummary counts: PASS=%s WARN=%s FAIL=%s\n' \
  "${summary_passes}" "${summary_warnings}" "${summary_failures}"
