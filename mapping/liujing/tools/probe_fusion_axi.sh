#!/usr/bin/env bash
# Run under a dedicated systemd service: a fault must keep DMA buffers pinned.
set -euo pipefail
if [[ $# -ne 4 ]]; then echo 'usage: probe_fusion_axi.sh ARTIFACT CAPTURE OUTPUT GROUP_COUNT' >&2; exit 2; fi
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export PYTHONPATH="$ROOT/python"
export XILINX_XRT=/usr LD_LIBRARY_PATH=/usr/local/lib
export OMP_NUM_THREADS=2 OMP_THREAD_LIMIT=2 OMP_WAIT_POLICY=PASSIVE GOMP_SPINCOUNT=0
export LJ3_PROBE_GROUP_COUNT="$4"
exec /usr/local/share/pynq-venv/bin/python3 "$ROOT/tools/fusion_pl_replay.py" "$1" "$2" "$3" --diagnostic-probe
