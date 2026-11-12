#!/usr/bin/env bash
# 실제 IMU + 출력 비활성 UART 계약의 빌드. flash는 수행하지 않는다.
set -euo pipefail
task_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
stage=${1:?fresh no-spaces build path required}
st_source=${2:?pinned ST source required}
evidence=${3:?fresh evidence path required}
base=/home/jinhyeong/.cache/it-arena/idf-6.1-smoke
export IDF_PATH="$base/esp-idf" IDF_TOOLS_PATH="$base/tools" IDF_PYTHON_ENV_PATH="$base/python-v3"
export PATH="$IDF_PYTHON_ENV_PATH/bin:/home/jinhyeong/.cache/it-arena/xtensa-esp-15.2.0_20251204/xtensa-esp-elf/bin:$PATH"
export IDF_COMPONENT_MANAGER=0 IDF_PY_BUILD_JOBS=4 ST_DRIVER_DIR="$st_source"
test ! -e "$stage"
test ! -e "$evidence/build_exit.txt"
test "$(git -C "$st_source" rev-parse HEAD)" = 2e86793125f7f46b6195fce93a2b3cb76a51e564
git -C "$st_source" diff --exit-code
mkdir -p "$stage" "$evidence"
cd "$task_root"
sources=(src/arena_vehicle_interface/portable/*.hpp src/arena_vehicle_interface/firmware_sensor_link/CMakeLists.txt
  src/arena_vehicle_interface/firmware_sensor_link/sdkconfig.defaults src/arena_vehicle_interface/firmware_sensor_link/main/*)
sha256sum "${sources[@]}" > "$evidence/sources_before.sha256"
tar -czf "$evidence/source_snapshot.tar.gz" "${sources[@]}"
finish() {
  result=$?
  sha256sum "${sources[@]}" > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/build_exit.txt"
}
trap finish EXIT
cp -a src/arena_vehicle_interface/portable "$stage/portable"
cp -a src/arena_vehicle_interface/firmware_sensor_link "$stage/firmware_sensor_link"
python "$IDF_PATH/tools/idf.py" -C "$stage/firmware_sensor_link" -B "$stage/build" build
python "$IDF_PATH/tools/idf.py" -C "$stage/firmware_sensor_link" -B "$stage/build" size > "$evidence/size.txt"
cp "$stage/build/arena_sensor_link.bin" "$evidence/"
sha256sum "$stage/build/arena_sensor_link.bin" > "$evidence/firmware.sha256"
git -C "$IDF_PATH" rev-parse HEAD > "$evidence/idf_commit.txt"
xtensa-esp32s3-elf-nm -S -C "$stage/build/arena_sensor_link.elf" > "$evidence/symbols.txt"
