#!/usr/bin/env bash
set -Eeuo pipefail

module3_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export BIO_NAV_INTEGRATION_ROOT="${BIO_NAV_INTEGRATION_ROOT:-/home/lyb/Workspace/Bio_Nav/worktrees/integration/final-indoor-outdoor-navigation}"
export RIVERMARK_SCENARIO_REVISION=final_rivermark
export RIVERMARK_FAIL_STOP=1

has_output=0
for argument in "$@"; do
  if [[ "${argument}" == "-h" || "${argument}" == "--help" ]]; then
    exec "${module3_root}/scripts/run_rivermark_campaign.sh" "$@"
  fi
  [[ "${argument}" == "--output" ]] && has_output=1
done
if [[ "${has_output}" != "1" ]]; then
  echo "Final Rivermark requires an explicit --output campaign directory; pilot and formal evidence must never share a root" >&2
  exit 2
fi

exec "${module3_root}/scripts/run_rivermark_campaign.sh" "$@"
