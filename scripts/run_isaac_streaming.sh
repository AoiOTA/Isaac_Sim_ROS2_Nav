#!/usr/bin/env bash

# Start the project's current Kujiale navigation scene as a persistent
# WebRTC-streamed Isaac Sim server.  Extra arguments are forwarded so callers
# can select another named spawn pose or camera profile when needed.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec "${SCRIPT_DIR}/run_isaac.sh" \
  --streaming \
  --environment-usd kujiale_0026_A_to_B_door_open.usd \
  --navigation-mode localization \
  --mode ideal \
  --spawn-pose long_route_start_g1 \
  "$@"
