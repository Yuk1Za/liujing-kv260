#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
cd "$ROOT/localization/final3"
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cd "$ROOT/sensors/lidar"
colcon build --merge-install
source "$ROOT/localization/final3/install/setup.bash"
cmake -S "$ROOT/mapping/liujing/native_fusion_overflow" -B "$ROOT/mapping/liujing/native_fusion_overflow_build" -DLJ2_BOARD=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/mapping/liujing/native_fusion_overflow_build" --target liujing_native -j2
