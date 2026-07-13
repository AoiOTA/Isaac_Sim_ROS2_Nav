#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

usage() {
  cat <<'EOF'
usage: performance_mode.sh status|enable|restore

status   Show power profile, CPU governor/EPP, temperature, and GPU power.
enable   Save the original state, then select the performance CPU policy.
restore  Restore the exact state saved by enable.

Run enable/restore with sudo on the real /sys tree. This script never invokes
sudo itself and benchmark launchers never change host power policy implicitly.
EOF
}

action="${1:-}"
[[ $# -eq 1 && "${action}" =~ ^(status|enable|restore)$ ]] || {
  usage >&2
  exit 2
}

target_uid="${SUDO_UID:-${UID}}"
cpu_root="${ISAAC_NAV_CPU_SYSFS_ROOT:-/sys/devices/system/cpu}"
state_dir="${ISAAC_NAV_PERFORMANCE_STATE_DIR:-/tmp/isaac_sim_ros2_nav_${target_uid}}"
state_file="${state_dir}/performance_mode.state"
backend="${ISAAC_NAV_PERFORMANCE_BACKEND:-auto}"
[[ "${backend}" =~ ^(auto|powerprofilesctl|cpupower|sysfs)$ ]] \
  || die "ISAAC_NAV_PERFORMANCE_BACKEND must be auto, powerprofilesctl, cpupower, or sysfs"

if [[ "${action}" != status && "${cpu_root}" == /sys/* && ${EUID} -ne 0 ]]; then
  die "${action} modifies host CPU policy; rerun with sudo"
fi
[[ ! -L "${state_dir}" ]] || die "performance state directory must not be a symlink"
mkdir -p "${state_dir}"

shopt -s nullglob
governor_paths=("${cpu_root}"/cpu[0-9]*/cpufreq/scaling_governor)
epp_paths=("${cpu_root}"/cpu[0-9]*/cpufreq/energy_performance_preference)
driver_paths=("${cpu_root}"/cpu[0-9]*/cpufreq/scaling_driver)
shopt -u nullglob

power_profile() {
  if [[ "${cpu_root}" == /sys/* ]] && command -v powerprofilesctl >/dev/null 2>&1; then
    powerprofilesctl get 2>/dev/null || printf unavailable
  else
    printf unavailable
  fi
}

print_unique_values() {
  local label="$1"
  shift
  if (($# == 0)); then
    printf '%s=unavailable\n' "${label}"
    return
  fi
  local values=() path
  for path in "$@"; do
    [[ -r "${path}" ]] || continue
    values+=("$(<"${path}")")
  done
  if ((${#values[@]} == 0)); then
    printf '%s=unavailable\n' "${label}"
  else
    printf '%s\n' "${values[@]}" | sort -u | sed "s/^/${label}=/"
  fi
}

print_thermal_power_reminder() {
  local maximum="" value path
  shopt -s nullglob
  for path in /sys/class/hwmon/hwmon*/temp*_input; do
    [[ -r "${path}" ]] || continue
    value="$(<"${path}")"
    [[ "${value}" =~ ^-?[0-9]+$ ]] || continue
    if [[ -z "${maximum}" || ${value} -gt ${maximum} ]]; then
      maximum="${value}"
    fi
  done
  shopt -u nullglob
  if [[ -n "${maximum}" ]]; then
    printf 'maximum_sensor_temperature_millicelsius=%s\n' "${maximum}"
  else
    printf 'maximum_sensor_temperature_millicelsius=unavailable\n'
  fi
  if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi \
      --query-gpu=name,power.draw,power.limit,temperature.gpu \
      --format=csv,noheader 2>/dev/null \
      | sed 's/^/gpu=/' || true
  fi
  printf 'reminder=performance mode raises power and temperature; monitor cooling during benchmarks\n'
}

show_status() {
  printf 'power_profile=%s\n' "$(power_profile)"
  print_unique_values governor "${governor_paths[@]}"
  print_unique_values energy_performance_preference "${epp_paths[@]}"
  print_unique_values scaling_driver "${driver_paths[@]}"
  if [[ -f "${state_file}" ]]; then
    printf 'saved_state=%s\n' "${state_file}"
  else
    printf 'saved_state=none\n'
  fi
  print_thermal_power_reminder
}

require_safe_cpu_path() {
  local path resolved root_resolved
  path="$1"
  resolved="$(realpath -m "${path}")"
  root_resolved="$(realpath -m "${cpu_root}")"
  [[ "${resolved}" == "${root_resolved}"/cpu[0-9]*/cpufreq/* ]] \
    || die "refusing CPU policy path outside ${cpu_root}: ${path}"
}

write_cpu_value() {
  local path="$1" value="$2"
  require_safe_cpu_path "${path}"
  [[ -w "${path}" ]] || die "CPU policy file is not writable: ${path}"
  printf '%s' "${value}" >"${path}"
  [[ "$(<"${path}")" == "${value}" ]] \
    || die "CPU policy verification failed: ${path}"
}

save_state() {
  [[ ! -e "${state_file}" ]] \
    || die "performance state already exists; run restore before enable: ${state_file}"
  local temporary path
  temporary="$(mktemp "${state_dir}/performance_mode.state.XXXXXX")"
  chmod 600 "${temporary}"
  {
    printf 'schema_version|1\n'
    printf 'target_uid|%s\n' "${target_uid}"
    printf 'power_profile|%s\n' "$(power_profile)"
    for path in "${governor_paths[@]}"; do
      printf 'governor|%s|%s\n' "${path}" "$(<"${path}")"
    done
    for path in "${epp_paths[@]}"; do
      printf 'epp|%s|%s\n' "${path}" "$(<"${path}")"
    done
  } >"${temporary}"
  mv "${temporary}" "${state_file}"
  if [[ ${EUID} -eq 0 && "${target_uid}" != 0 ]]; then
    chown "${target_uid}:${target_uid}" "${state_file}"
  fi
}

set_power_profile_performance() {
  [[ "${cpu_root}" == /sys/* ]] || return 0
  command -v powerprofilesctl >/dev/null 2>&1 || return 0
  powerprofilesctl list 2>/dev/null | grep -q 'performance:' || return 0
  powerprofilesctl set performance
  [[ "$(powerprofilesctl get)" == performance ]] \
    || die "powerprofilesctl did not select performance"
}

enable_performance() {
  ((${#governor_paths[@]} > 0)) \
    || [[ "${backend}" == powerprofilesctl ]] \
    || die "no CPU cpufreq governor files found under ${cpu_root}"
  save_state
  if [[ "${backend}" == auto || "${backend}" == powerprofilesctl ]]; then
    set_power_profile_performance
  fi
  case "${backend}" in
    cpupower)
      require_command cpupower
      cpupower frequency-set -g performance
      ;;
    auto|sysfs)
      local path
      for path in "${governor_paths[@]}"; do
        write_cpu_value "${path}" performance
      done
      ;;
    powerprofilesctl) ;;
  esac
  local path
  for path in "${epp_paths[@]}"; do
    write_cpu_value "${path}" performance
  done
  for path in "${governor_paths[@]}"; do
    [[ "$(<"${path}")" == performance ]] \
      || die "governor is not performance after enable: ${path}"
  done
  log_info "CPU performance policy enabled; original state saved at ${state_file}"
  show_status
}

restore_state() {
  [[ -f "${state_file}" && ! -L "${state_file}" ]] \
    || die "no saved performance state to restore: ${state_file}"
  local saved_profile="" kind first second
  while IFS='|' read -r kind first second; do
    case "${kind}" in
      schema_version)
        [[ "${first}" == 1 ]] || die "unsupported performance state schema"
        ;;
      target_uid)
        [[ "${first}" == "${target_uid}" ]] \
          || die "performance state belongs to uid ${first}, expected ${target_uid}"
        ;;
      power_profile) saved_profile="${first}" ;;
      governor|epp)
        [[ -n "${first}" && -n "${second}" ]] \
          || die "invalid CPU entry in ${state_file}"
        ;;
      *) die "unknown performance state entry: ${kind}" ;;
    esac
  done <"${state_file}"

  if [[ "${saved_profile}" != unavailable && "${cpu_root}" == /sys/* ]] \
      && command -v powerprofilesctl >/dev/null 2>&1; then
    powerprofilesctl set "${saved_profile}"
  fi
  while IFS='|' read -r kind first second; do
    case "${kind}" in
      governor|epp) write_cpu_value "${first}" "${second}" ;;
    esac
  done <"${state_file}"
  rm -f -- "${state_file}"
  log_info "restored the saved CPU power policy"
  show_status
}

case "${action}" in
  status) show_status ;;
  enable) enable_performance ;;
  restore) restore_state ;;
esac
