#!/usr/bin/env bash
# 출력 비활성 UART bring-up 검사 앱(firmware_bench)의 대상 빌드만 수행한다. flash/monitor/장치 조회 없음.
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
task_sources=(src/arena_vehicle_interface/portable/*.hpp src/arena_vehicle_interface/firmware_smoke/main/*.hpp \
  src/arena_vehicle_interface/firmware_bench/CMakeLists.txt src/arena_vehicle_interface/firmware_bench/sdkconfig.defaults \
  src/arena_vehicle_interface/firmware_bench/main/*)
sha256sum "${task_sources[@]}" > "$evidence/build_sources_before.sha256"
tar -czf "$evidence/build_source_snapshot.tar.gz" "${task_sources[@]}"
finish() {
  result=$?
  sha256sum "${task_sources[@]}" > "$evidence/build_sources_after.sha256"
  diff -u "$evidence/build_sources_before.sha256" "$evidence/build_sources_after.sha256" > "$evidence/build_source_hash_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/build_exit.txt"
}
trap finish EXIT
mkdir -p "$task_stage/portable" "$task_stage/firmware_smoke/main"
cp src/arena_vehicle_interface/portable/*.hpp "$task_stage/portable/"
cp src/arena_vehicle_interface/firmware_smoke/main/*.hpp "$task_stage/firmware_smoke/main/"
cp -a src/arena_vehicle_interface/firmware_bench "$task_stage/firmware_bench"
# IDF는 공백 있는 component 경로를 지원하지 않으므로 공백 없는 소스 사본에서 빌드한다.
python "$IDF_PATH/tools/idf.py" -C "$task_stage/firmware_bench" -B "$task_stage/build" -D IDF_TARGET=esp32s3 build
python "$IDF_PATH/tools/idf.py" -C "$task_stage/firmware_bench" -B "$task_stage/build" size > "$evidence/idf_size.txt"
app=arena_actuation_uart_bench
cp "$task_stage/build/$app.elf" "$task_stage/build/$app.bin" "$task_stage/build/$app.map" "$evidence/"
cp "$task_stage/firmware_bench/sdkconfig" "$evidence/sdkconfig.txt"
cp "$task_stage/build/compile_commands.json" "$evidence/compile_commands.json"
cp "$task_stage/build/esp-idf/main/CMakeFiles/__idf_main.dir/main.cpp.su" "$evidence/main_cpp_stack_usage.su"
xtensa-esp32s3-elf-readelf -h "$task_stage/build/$app.elf" > "$evidence/elf_header.txt"
xtensa-esp32s3-elf-nm -u -C "$task_stage/build/$app.elf" > "$evidence/undefined_symbols.txt"
xtensa-esp32s3-elf-nm -S -C "$task_stage/build/$app.elf" > "$evidence/symbols.txt"
# 비활성 기본값에서도 실제 SDK UART 경로가 링크됐는지(컴파일러가 지우지 않았는지) 확인한다. 실행 증거는 아니다.
# v6.1의 uart_set_pin은 인수 개수로 고르는 매크로이며 4핀 호출은 _uart_set_pin6으로 링크된다.
grep -E ' (uart_driver_install|uart_param_config|_uart_set_pin6|uart_read_bytes|uart_tx_chars|uart_flush_input)$' \
  "$evidence/symbols.txt" > "$evidence/linked_uart_symbols.txt"
test "$(wc -l < "$evidence/linked_uart_symbols.txt")" -eq 6
# 함수별 정적 프레임 2 KiB 초과를 거절한다(소유 작업 6 KiB·main 8 KiB stack 검사값 대비). 실제 stack 사용량 증거는 아니다.
awk -F'	' '$2 > 2048' "$evidence/main_cpp_stack_usage.su" > "$evidence/large_stack_frames.txt"
test ! -s "$evidence/large_stack_frames.txt"
sha256sum "$evidence/$app.elf" "$evidence/$app.bin" > "$evidence/binaries.sha256"
printf 'BUILD_ONLY_COMPLETE; hardware_executed=false; external_io_default=disabled; firmware_verified=false; ready_to_drive=false\n'
