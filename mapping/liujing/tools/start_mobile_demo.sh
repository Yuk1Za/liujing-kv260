#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
seconds=${1:-900}
if (($#)); then shift; fi
# systemd has no interactive shell profile. PYNQ needs the installed XRT path.
export XILINX_XRT="${XILINX_XRT:-/usr}"
exec bash "$ROOT/tools/start_live_demo.sh" "$seconds" \
  --native-build "$ROOT/native_v2_preview_build" --demo-bridge --render-hz 10 "$@"
