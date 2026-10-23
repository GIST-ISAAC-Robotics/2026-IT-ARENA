#!/usr/bin/env bash
set -euo pipefail
cd -- '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
evidence=artifacts/validation/2026-10-22/actuation_xtensa_probe_v1
task_bin=/home/jinhyeong/.cache/it-arena/xtensa-esp-15.2.0_20251204/xtensa-esp-elf/bin
task_cxx="$task_bin/xtensa-esp32s3-elf-g++"
test ! -e "$evidence/probe_target.o"
sha256sum src/arena_vehicle_interface/portable/*.hpp scripts/actuation_target_probe.cpp \
  scripts/actuation_receiver_production_api_check.cpp > "$evidence/sources_before.sha256"
finish() {
  result=$?
  sha256sum src/arena_vehicle_interface/portable/*.hpp scripts/actuation_target_probe.cpp \
    scripts/actuation_receiver_production_api_check.cpp > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/probe_exit.txt"
}
trap finish EXIT
task_flags=(-std=c++17 -Os -Wall -Wextra -Werror -pedantic -fno-exceptions -fno-rtti -fstack-usage \
  -Isrc/arena_vehicle_interface/portable)
"$task_cxx" --version > "$evidence/compiler_version.txt"
"$task_cxx" -dumpmachine >> "$evidence/compiler_version.txt"
"$task_cxx" -print-multi-lib > "$evidence/multilib.txt"
"$task_cxx" "${task_flags[@]}" -c scripts/actuation_target_probe.cpp -o "$evidence/probe_target.o"
"$task_cxx" "${task_flags[@]}" -c scripts/actuation_receiver_production_api_check.cpp -o "$evidence/production_api_target.o"
g++ "${task_flags[@]}" -c scripts/actuation_target_probe.cpp -o "$evidence/probe_host.o"
g++ "${task_flags[@]}" scripts/actuation_receiver_production_api_check.cpp -o "$evidence/production_api_host"
"$evidence/production_api_host"
"$task_bin/xtensa-esp32s3-elf-nm" -S -C "$evidence/probe_target.o" > "$evidence/target_symbols.txt"
"$task_bin/xtensa-esp32s3-elf-nm" -u -C "$evidence/probe_target.o" > "$evidence/target_undefined.txt"
nm -S -C "$evidence/probe_host.o" > "$evidence/host_symbols.txt"
"$task_bin/xtensa-esp32s3-elf-size" "$evidence/probe_target.o" > "$evidence/target_size.txt"
"$task_bin/xtensa-esp32s3-elf-readelf" -h "$evidence/probe_target.o" > "$evidence/target_elf_header.txt"
task_stdlib=$("$task_cxx" -print-file-name=libstdc++.a)
test -f "$task_stdlib"
printf '%s\n' "$task_stdlib" > "$evidence/stdlib_path.txt"
sha256sum "$task_stdlib" > "$evidence/stdlib.sha256"
"$task_bin/xtensa-esp32s3-elf-nm" -A -C "$task_stdlib" 2>/dev/null | \
  grep -E ' [TW] std::from_chars\(char const\*, char const\*, double&' > "$evidence/from_chars_definition.txt"
printf 'COMPILE_PROBE_COMPLETE (not firmware link or target execution)\n'
