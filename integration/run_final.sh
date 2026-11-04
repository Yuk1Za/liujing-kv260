#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$ROOT/localization/final3/install/setup.bash"
source "$ROOT/sensors/lidar/install/setup.bash"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
exec python3 "$ROOT/integration/run_pipeline.py" "$@"
