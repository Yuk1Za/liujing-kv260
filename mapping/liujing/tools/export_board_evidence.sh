#!/usr/bin/env bash
set -euo pipefail
cd /home/ubuntu/liujing
test "$(cat raster_real_02.exit)" = 0
test "$(cat raster_100k_01.exit)" = 0
test "$(cat joint_01.exit)" = 0
ctest --test-dir build --output-on-failure >tests_cpp.log 2>&1
python3 - <<'PY'
import json,hashlib,platform
from pathlib import Path
files=['build/libliujing.so','src/core.cpp','src/core.hpp','hardware/raster.cpp','hardware/raster.hpp','python/native.py','python/accelerator.py','python/bench_raster.py','python/bench_joint.py','artifacts/liujing.bit','artifacts/liujing.hwh']
report={'collection':'after matched tests; tested native library and driver were not rebuilt during the tests','machine':platform.machine(),'board_wall_clock_known_incorrect':True,'files':{f:hashlib.sha256(Path(f).read_bytes()).hexdigest() for f in files}}
Path('board_run_provenance.json').write_text(json.dumps(report,indent=2))
PY
tar czf /home/ubuntu/liujing_board_evidence_v2.tgz \
 board_run_provenance.json tests_cpp.log tests_python.log \
 raster_real_01.log raster_real_01.exit \
 runs/raster_real_01/restored.json \
 runs/raster_real_02/report.json runs/raster_real_02/restored.json runs/raster_real_02/cpu.png runs/raster_real_02/pl.png \
 runs/raster_100k_01/report.json runs/raster_100k_01/restored.json \
 runs/joint_01/report.json runs/joint_01/restored.json \
 runs/c16_integrated_01/c16_acceptance.json runs/c16_integrated_01/baseline_comparison.json runs/c16_integrated_01/verify_c16_snapshot.py \
 runs/camera_preview_04/surfels.npy runs/camera_preview_04/camera.json runs/camera_preview_04/report.json runs/camera_preview_04/workload.json \
 runs/analytic_100k/workload.json \
 runs/camera_02/0_depth.png runs/camera_02/0_rgb.png runs/camera_02/factory_calibration.json runs/camera_02/frames.jsonl \
 runs/alignment_01/report.json runs/alignment_01/timestamp_audit.json \
 runs/alignment_check_01/report.json runs/alignment_check_01/initialization_projection.jpg runs/alignment_check_01/refinement_projection.jpg
sha256sum /home/ubuntu/liujing_board_evidence_v2.tgz
ls -lh /home/ubuntu/liujing_board_evidence_v2.tgz
