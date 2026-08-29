#!/usr/bin/env bash
set -Eeuo pipefail

module3_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  echo "usage: $0 static|dynamic|appearance [dim_warm|dim_cool|bright_warm|bright_cool]"
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if (( $# < 1 || $# > 2 )); then
  usage >&2
  exit 2
fi

scenario="$1"
profile="${2:-}"
case "${scenario}" in
  static|dynamic)
    if [[ -n "${profile}" ]]; then
      echo "${scenario} does not accept an appearance profile" >&2
      exit 2
    fi
    profile="baseline"
    ;;
  appearance)
    profile="${profile:-bright_warm}"
    case "${profile}" in
      dim_warm|dim_cool|bright_warm|bright_cool) ;;
      *)
        echo "unknown Rivermark appearance profile: ${profile}" >&2
        exit 2
        ;;
    esac
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

export RIVERMARK_HEADLESS=0
export RIVERMARK_RVIZ=1
export RIVERMARK_AUTO_GOAL="${RIVERMARK_AUTO_GOAL:-1}"
export RIVERMARK_VISUAL_ROUTE="${RIVERMARK_VISUAL_ROUTE:-1}"
export RIVERMARK_APPEARANCE_PROFILE="${profile}"
export RIVERMARK_DYNAMIC_CASE="full_route_four_stage"
export RIVERMARK_DYNAMIC_VARIANT="v3"
visual_revision="${RIVERMARK_VISUAL_REVISION:-final}"
if [[ "${visual_revision}" == "final" ]]; then
  if [[ "${scenario}" == "static" ]]; then
    export RIVERMARK_OBSTACLE_CONFIG="${module3_root}/data/rivermark_demo/final_rivermark_static_obstacles.yaml"
    export RIVERMARK_PHYSICAL_OBSTACLES=1
  elif [[ "${scenario}" == "dynamic" ]]; then
    export RIVERMARK_OBSTACLE_CONFIG="${module3_root}/data/rivermark_demo/final_rivermark_dynamic.yaml"
    export RIVERMARK_PHYSICAL_OBSTACLES=1
    export RIVERMARK_DYNAMIC_CASE="crossing"
  else
    export RIVERMARK_OBSTACLE_CONFIG="${module3_root}/data/rivermark_demo/final_rivermark_static_obstacles.yaml"
    export RIVERMARK_PHYSICAL_OBSTACLES=1
  fi
elif [[ "${visual_revision}" != "attempt31" ]]; then
  echo "RIVERMARK_VISUAL_REVISION must be final or attempt31" >&2
  exit 2
fi

if [[ "${RIVERMARK_AUTO_GOAL}" == "0" ]]; then
  echo "Starting one-terminal Rivermark ${scenario} manual-goal navigation"
  echo "No waypoint will be published automatically; use RViz 2D Goal Pose"
else
  echo "Starting one-terminal Rivermark ${scenario} navigation"
fi
echo "Module2 + Module3 + Isaac GUI + dedicated outdoor RViz"
echo "appearance_profile=${profile}; visual_revision=${visual_revision}; Ctrl+C stops the complete stack"

exec "${module3_root}/scripts/run_rivermark_demo.sh" \
  module2 "${scenario}" "${profile}"
