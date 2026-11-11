#!/usr/bin/env bash
set -euo pipefail
project_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
stage=${1:?fresh build path required}
st_source=${2:?pinned ST driver path required}
idf_base=/home/jinhyeong/.cache/it-arena/idf-6.1-smoke
export IDF_PATH="$idf_base/esp-idf"
export IDF_TOOLS_PATH="$idf_base/tools"
export IDF_PYTHON_ENV_PATH="$idf_base/python-v3"
export PATH="$IDF_PYTHON_ENV_PATH/bin:/home/jinhyeong/.cache/it-arena/xtensa-esp-15.2.0_20251204/xtensa-esp-elf/bin:$PATH"
export IDF_COMPONENT_MANAGER=0
export IDF_PY_BUILD_JOBS=4
export ST_DRIVER_DIR="$st_source"
test ! -e "$stage"
test "$(git -C "$st_source" rev-parse HEAD)" = 2e86793125f7f46b6195fce93a2b3cb76a51e564
git -C "$st_source" diff --exit-code
mkdir -p "$stage"
cp -a "$project_root/src/arena_vehicle_interface/firmware_imu_probe" "$stage/project"
python "$IDF_PATH/tools/idf.py" -C "$stage/project" -B "$stage/build" -D "ST_DRIVER_DIR=$st_source" build
git -C "$IDF_PATH" rev-parse HEAD > "$stage/idf_commit.txt"
sha256sum "$stage/build/arena_imu_probe.bin" > "$stage/firmware.sha256"
