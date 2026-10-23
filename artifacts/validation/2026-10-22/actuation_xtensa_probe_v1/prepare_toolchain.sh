#!/usr/bin/env bash
# 공식 ESP-IDF v6.1의 권장 Xtensa 도구만 별도 캐시에 준비한다. SDK/펌웨어 설치·flash 없음.
set -euo pipefail
cd -- '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
evidence=artifacts/validation/2026-10-22/actuation_xtensa_probe_v1
task_cache=/home/jinhyeong/.cache/it-arena/xtensa-esp-15.2.0_20251204
task_archive="$task_cache/toolchain.tar.xz"
task_digest=3d50f5cd5f173acfd524e07c1cd69bc99585731a415ca2e5bce879997fe602b8
task_url=https://github.com/espressif/crosstool-NG/releases/download/esp-15.2.0_20251204/xtensa-esp-elf-15.2.0_20251204-x86_64-linux-gnu.tar.xz
mkdir -p "$task_cache"
curl --fail --location --retry 2 --max-time 600 "$task_url" -o "$task_archive"
printf '%s  %s\n' "$task_digest" "$task_archive" | sha256sum -c -
test "$(stat -c '%s' "$task_archive")" = 180530004
if [[ ! -d "$task_cache/xtensa-esp-elf" ]]; then
  tar -xJf "$task_archive" -C "$task_cache"
fi
"$task_cache/xtensa-esp-elf/bin/xtensa-esp32s3-elf-g++" --version
"$task_cache/xtensa-esp-elf/bin/xtensa-esp32s3-elf-g++" -dumpmachine
du -sh "$task_cache"
