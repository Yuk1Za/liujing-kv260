#!/usr/bin/env bash
# Bounded real-input static diagnostic. Calibration verification is not bypassed.
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
seconds=${1:-600}
if (($#)); then shift; fi
exec bash "$ROOT/tools/online.sh" run --engine native-v2 \
  --backend pl --mode static_diagnostic --render-size 320x240 \
  --calibration "${LIUJING_CALIBRATION:-$ROOT/configs/live_candidate_20261026.json}" \
  --native-build "$ROOT/native_v2_live_build" \
  --bit "$ROOT/artifacts/opaque_v1/liujing.bit" \
  --bit-manifest "$ROOT/artifacts/opaque_v1/manifest.json" \
  --seconds "$seconds" "$@"
