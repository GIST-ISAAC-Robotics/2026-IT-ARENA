#!/usr/bin/env bash
set -euo pipefail
task_root=$(cd -- "$(dirname -- "$0")/.." && pwd)
cd -- "$task_root"
evidence=$(realpath -m -- "${1:?Provide a fresh evidence directory}")
task_stage=$(realpath -m -- "${2:?Provide a fresh no-spaces build directory}")
task_base=${3:-/home/jinhyeong/.cache/it-arena/idf-6.1-smoke}
export IDF_PATH="$task_base/esp-idf"
export IDF_TOOLS_PATH="$task_base/tools"
export IDF_PYTHON_ENV_PATH="$task_base/python-v3"
export IDF_PY_BUILD_JOBS=4
export IDF_COMPONENT_MANAGER=0
export PATH="$IDF_PYTHON_ENV_PATH/bin:/home/jinhyeong/.cache/it-arena/xtensa-esp-15.2.0_20251204/xtensa-esp-elf/bin:$PATH"
test ! -e "$evidence/build_exit.txt"
test ! -e "$task_stage"
mkdir -p "$evidence"
task_sources=(src/arena_vehicle_interface/portable/*.hpp src/arena_vehicle_interface/firmware_smoke/CMakeLists.txt \
  src/arena_vehicle_interface/firmware_smoke/sdkconfig.defaults src/arena_vehicle_interface/firmware_smoke/main/*)
sha256sum "${task_sources[@]}" > "$evidence/build_sources_before.sha256"
tar -czf "$evidence/build_source_snapshot.tar.gz" "${task_sources[@]}"
finish() {
  result=$?
  sha256sum "${task_sources[@]}" > "$evidence/build_sources_after.sha256"
  diff -u "$evidence/build_sources_before.sha256" "$evidence/build_sources_after.sha256" > "$evidence/build_source_hash_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/build_exit.txt"
}
trap finish EXIT
test ! -e "$task_stage"
mkdir -p "$task_stage/portable"
cp src/arena_vehicle_interface/portable/*.hpp "$task_stage/portable/"
cp -a src/arena_vehicle_interface/firmware_smoke "$task_stage/firmware_smoke"
# IDF는 공백 있는 component 경로를 지원하지 않으므로 읽기 전용 소스 사본에서 빌드한다.
python "$IDF_PATH/tools/idf.py" -C "$task_stage/firmware_smoke" -B "$task_stage/build" -D IDF_TARGET=esp32s3 build
python "$IDF_PATH/tools/idf.py" -C "$task_stage/firmware_smoke" -B "$task_stage/build" size > "$evidence/idf_size.txt"
cp "$task_stage/build/arena_actuation_smoke.elf" "$evidence/"
cp "$task_stage/build/arena_actuation_smoke.bin" "$evidence/"
cp "$task_stage/build/arena_actuation_smoke.map" "$evidence/"
cp "$task_stage/firmware_smoke/sdkconfig" "$evidence/sdkconfig.txt"
cp "$task_stage/build/compile_commands.json" "$evidence/compile_commands.json"
xtensa-esp32s3-elf-readelf -h "$task_stage/build/arena_actuation_smoke.elf" > "$evidence/elf_header.txt"
xtensa-esp32s3-elf-nm -u -C "$task_stage/build/arena_actuation_smoke.elf" > "$evidence/undefined_symbols.txt"
xtensa-esp32s3-elf-nm -S -C "$task_stage/build/arena_actuation_smoke.elf" > "$evidence/symbols.txt"
sha256sum "$evidence/arena_actuation_smoke.elf" "$evidence/arena_actuation_smoke.bin" > "$evidence/binaries.sha256"
printf 'BUILD_ONLY_COMPLETE; hardware_executed=false; firmware_verified=false; ready_to_drive=false\n'
