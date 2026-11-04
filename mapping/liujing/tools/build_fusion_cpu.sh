#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
BUILD=${1:-"$ROOT/native_fusion_v3_build"}
# A build must not compete with a measured live/replay session on four cores.
exec 9>/home/ubuntu/losam_live_20261021/pipeline.lock
flock -n 9 || { echo 'A pipeline owns the board; finish it before building.' >&2; exit 1; }
source /opt/ros/humble/setup.bash
cmake -S "$ROOT/native_fusion_v3" -B "$BUILD" -DLJ2_BOARD=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" --target liujing_native fusion_replay fusion_test unproject_check lj2_test lj2_bounds_check -j2
ctest --test-dir "$BUILD" --output-on-failure
sha256sum "$BUILD/liujing_native" "$BUILD/fusion_replay"
