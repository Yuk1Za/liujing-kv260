#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
run=${1:?new output root required}
test ! -e "$run"
mkdir -p "$run"
export PYTHONPATH="$PWD/python" PYTHONPYCACHEPREFIX="$PWD/$run/pycache"
export LIUJING_LIBRARY="$PWD/build_guarded/libliujing.so" OMP_WAIT_POLICY=PASSIVE GOMP_SPINCOUNT=0 OPENBLAS_NUM_THREADS=1
py=/usr/local/share/pynq-venv/bin/python3
for schedule in sequential pipelined; do
 printf '%s\n' "$schedule" >"$run/phase"
 extra=()
 if [ "$schedule" = pipelined ]; then extra=(--encode-pipeline); fi
 "$py" python/bench_visual_chain.py --bit artifacts/stream_v5/liujing.bit --restore-bit /home/ubuntu/losam_live_20261021/artifacts/c16_feature.bit --out "$run/$schedule" --numeric-contract qrcp_v1 "${extra[@]}" >"$run/$schedule.log" 2>&1
 "$py" - "$run/$schedule/report.json" <<'PY'
import json,sys
assert json.load(open(sys.argv[1]))['correctness_passed']
PY
done
printf 'completed\n' >"$run/phase"
