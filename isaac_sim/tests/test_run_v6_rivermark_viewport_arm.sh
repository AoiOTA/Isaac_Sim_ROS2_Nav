#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
fixture="$(mktemp -d)"
trap 'rm -rf -- "${fixture}"' EXIT
mkdir -p "${fixture}/scripts/lib" "${fixture}/data/rivermark_demo"
cp "${ROOT}/scripts/run_v6_rivermark.sh" "${fixture}/scripts/"

cat >"${fixture}/scripts/lib/common.sh" <<'EOF'
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)"
die() { printf '%s\n' "$*" >&2; exit 2; }
require_file() { [[ -f "$1" ]] || die "missing file: $1"; }
EOF
cat >"${fixture}/scripts/import_assets.sh" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF
cat >"${fixture}/scripts/run_isaac.sh" <<'EOF'
#!/usr/bin/env bash
printf '%s\0' "$@" >"${VIEWPORT_CAPTURE:?}"
EOF
chmod +x "${fixture}/scripts/"*.sh

demo="${fixture}/data/rivermark_demo"
for name in \
  rivermark.usd \
  rivermark.spawn.yaml \
  rivermark_selected.yaml \
  rivermark_selected.geojson \
  rivermark_regions.yaml \
  rivermark_demo_goals.yaml \
  final_rivermark_static_obstacles.yaml \
  final_rivermark_dynamic.yaml \
  rivermark_appearance_profiles.yaml; do
  : >"${demo}/${name}"
done

wrapper="${fixture}/scripts/run_v6_rivermark.sh"
export RIVERMARK_DEMO_DIR="${demo}"
export RIVERMARK_USD="${demo}/rivermark.usd"

VIEWPORT_CAPTURE="${fixture}/default_a.argv" "${wrapper}" \
  isaac static --headless --max-frames 1
VIEWPORT_CAPTURE="${fixture}/explicit_a.argv" "${wrapper}" \
  isaac static --viewport-arm A --headless --max-frames 1
cmp "${fixture}/default_a.argv" "${fixture}/explicit_a.argv"

mkdir -p "${fixture}/b-run"
printf '%s\n' '{"schema":"startup-ab","winner":{"viewport_arm":"B"}}' \
  >"${fixture}/winner.json"
VIEWPORT_CAPTURE="${fixture}/b.argv" "${wrapper}" \
  isaac static --viewport-arm B \
  --viewport-runtime-attestation "${fixture}/b-run/viewport_runtime_attestation.json" \
  --viewport-winner-manifest "${fixture}/winner.json" \
  --viewport-run-root "${fixture}/b-run" \
  --headless --max-frames 1
python3 - \
  "${fixture}/explicit_a.argv" "${fixture}/b.argv" <<'PY'
from pathlib import Path
import sys

a = Path(sys.argv[1]).read_bytes().split(b"\0")[:-1]
b = Path(sys.argv[2]).read_bytes().split(b"\0")[:-1]
assert a.count(b"--no-disable-viewport-updates") == 1
assert b.count(b"--disable-viewport-updates") == 1
assert b[b.index(b"--viewport-arm-identity") + 1] == b"B"
assert b"--viewport-runtime-attestation" in b
assert b"--viewport-winner-manifest" in b
assert b"--viewport-winner-manifest-sha256" in b
assert b"--viewport-run-root" in b
assert b[b.index(b"--viewport-scene") + 1] == b"rivermark:static"
assert b"--rtx-descriptor-sets" in a and b"20000" in a
assert b"--disable-dlss" in a
assert b"rgbd_navigation" in a
PY

if VIEWPORT_CAPTURE="${fixture}/invalid.argv" "${wrapper}" \
    isaac static --viewport-arm B --no-headless; then
  echo "GUI viewport arm B unexpectedly succeeded" >&2
  exit 1
fi
if VIEWPORT_CAPTURE="${fixture}/invalid.argv" "${wrapper}" \
    isaac static --disable-viewport-updates --headless; then
  echo "caller low-level viewport override unexpectedly succeeded" >&2
  exit 1
fi
if VIEWPORT_CAPTURE="${fixture}/invalid.argv" "${wrapper}" \
    isaac static --viewport-arm C --headless; then
  echo "invalid viewport arm unexpectedly succeeded" >&2
  exit 1
fi

echo "run_v6_rivermark viewport-arm direct shell test: PASS"
