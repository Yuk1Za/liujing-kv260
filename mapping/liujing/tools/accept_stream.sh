#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
bit=$(realpath "${1:?integrated bit required}")
run=${2:?new output directory required}
test ! -e "$run"
mkdir -p "$run"
run=$(realpath "$run")
trap 'code=$?; printf "%s\n" "$code" >"$run/exit_code"; cp "$run/phase" "$run/failed_stage"; printf "failed\n" >"$run/phase"' ERR
export PYTHONPATH="$PWD/python" LIUJING_LIBRARY="${LIUJING_LIBRARY:-$PWD/build_stream/libliujing.so}"
export PYTHONPYCACHEPREFIX="$run/pycache"
export OMP_WAIT_POLICY=PASSIVE GOMP_SPINCOUNT=0 OPENBLAS_NUM_THREADS=1 LIUJING_CPU_RENDER_MODE="${LIUJING_CPU_RENDER_MODE:-4}"
numeric_contract=mixed
if [ "$LIUJING_CPU_RENDER_MODE" = 5 ]; then numeric_contract=qrcp_v1; fi
py=/usr/local/share/pynq-venv/bin/python3
restore=/home/ubuntu/losam_live_20261021/artifacts/c16_feature.bit
printf 'c16\n' >"$run/phase"
"$py" tools/verify_integrated.py --bit "$bit" --out "$run/c16" --restore-bit "$restore" >"$run/c16.log" 2>&1
printf 'io_diagnostic\n' >"$run/phase"
"$py" python/profile_raster_io.py --bit "$bit" --out "$run/io_diagnostic" --restore-bit "$restore" --numeric-contract "$numeric_contract" >"$run/io_diagnostic.log" 2>&1
for scene in camera_preview_04 analytic_100k dense_real_depth_01; do
 printf '%s\n' "$scene" >"$run/phase"
 "$py" python/bench_raster.py "runs/$scene" --bit "$bit" --out "$run/$scene" --restore-bit "$restore" >"$run/$scene.log" 2>&1
 "$py" - "$run/$scene/report.json" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
assert r['correctness_passed'], 'Raster parity failure: do not promote to association/fusion'
assert all(x.get('fp32_quality',{}).get('psnr_db',0)>=40 for x in r['rows']), 'FP32 quality regression'
PY
done
printf 'visual_chain\n' >"$run/phase"
"$py" python/bench_visual_chain.py --bit "$bit" --out "$run/visual_chain" --restore-bit "$restore" --numeric-contract "$numeric_contract" >"$run/visual_chain.log" 2>&1
"$py" - "$run/visual_chain/report.json" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]));assert r['correctness_passed'],'Map or view mismatch'
print('Measured visual chain speedup:',r['speedup'])
PY
printf 'c16_alone\n' >"$run/phase"
"$py" python/bench_joint.py runs/camera_preview_04 --bit "$bit" --out "$run/c16_alone" --restore-bit "$restore" --numeric-contract "$numeric_contract" --seconds 60 --lidar-only >"$run/c16_alone.log" 2>&1
printf 'c16_concurrent\n' >"$run/phase"
"$py" python/bench_joint.py runs/camera_preview_04 --bit "$bit" --out "$run/c16_concurrent" --restore-bit "$restore" --numeric-contract "$numeric_contract" --seconds 60 >"$run/c16_concurrent.log" 2>&1
printf 'completed_compute_checks_not_room_acceptance\n' >"$run/phase"
printf '0\n' >"$run/exit_code"
