#!/usr/bin/env bash
# PC ESP32-S3 빌드만. 어떤 장치 포트도 열거나 flash하지 않는다.
set -euo pipefail
cd "$(dirname "$0")/../../../.."
root=$PWD
a="$root/artifacts/validation/2026-11-12/runtime_hal_preview_r289_v1"
stage=/home/jinhyeong/.cache/it-arena/runtime-preview-r289
base=/home/jinhyeong/.cache/it-arena/idf-6.1-smoke
export IDF_PATH="$base/esp-idf" IDF_TOOLS_PATH="$base/tools" IDF_PYTHON_ENV_PATH="$base/python-v3"
export PATH="$IDF_PYTHON_ENV_PATH/bin:/home/jinhyeong/.cache/it-arena/xtensa-esp-15.2.0_20251204/xtensa-esp-elf/bin:$PATH"
export IDF_COMPONENT_MANAGER=0 IDF_PY_BUILD_JOBS=4
export ST_DRIVER_DIR=/home/jinhyeong/.cache/it-arena/lsm6dsox-pid_20261111
test "$(git -C "$ST_DRIVER_DIR" rev-parse HEAD)" = 2e86793125f7f46b6195fce93a2b3cb76a51e564
git -C "$ST_DRIVER_DIR" diff --exit-code
test ! -e "$stage"
mkdir -p "$stage"
cp -a "$a/firmware" "$stage/firmware"
cp -a src/arena_vehicle_interface/portable "$stage/portable"
cp artifacts/validation/2026-11-12/motor_endpoint_r283_v1/actuation_motor_endpoint.hpp "$stage/portable/"
cp artifacts/validation/2026-11-12/servo_nonblocking_r287_v1/st3215_transaction.hpp "$stage/portable/"
cp src/arena_vehicle_interface/firmware_sensor_link/main/imu_source.c "$stage/firmware/main/"
find "$stage/firmware" "$stage/portable" -type f -print0 | sort -z | xargs -0 sha256sum > "$a/sources.sha256"
tar -czf "$a/source_snapshot.tar.gz" -C "$stage" firmware portable
python "$IDF_PATH/tools/idf.py" -C "$stage/firmware" -B "$stage/build" build
python "$IDF_PATH/tools/idf.py" -C "$stage/firmware" -B "$stage/build" size > "$a/size.txt"
sha256sum "$stage/build/arena_runtime_preview.bin" > "$a/firmware.sha256"
xtensa-esp32s3-elf-nm -S -C "$stage/build/arena_runtime_preview.elf" > "$a/symbols.txt"
printf 'build_only_no_flash\n' > "$a/completed.txt"
