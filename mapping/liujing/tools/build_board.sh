#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=${1:-build_stream}
# Normalized archive mtimes also require discarding generated Python bytecode.
find python tests -type f -name '*.pyc' -delete
# Deploy archives deliberately normalize mtimes for reproducibility. An
# incremental make would silently retain old objects after extraction.
# Always rebuild cleanly and record the actual sources and resulting library.
cmake -S . -B "$build_dir" -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build "$build_dir" --clean-first -j2
ctest --test-dir "$build_dir" --output-on-failure
export LIUJING_LIBRARY="$(realpath "$build_dir/libliujing.so")"
python3 -m unittest discover -s tests
python3 - "$build_dir" <<'PY'
import hashlib,json,sys
from pathlib import Path
b=Path(sys.argv[1]);paths=[Path('CMakeLists.txt'),*Path('src').glob('*'),Path('hardware/raster.cpp'),Path('hardware/raster.hpp'),b/'libliujing.so',b/'compile_commands.json']
(b/'build_manifest.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths if p.is_file()},indent=2))
PY
