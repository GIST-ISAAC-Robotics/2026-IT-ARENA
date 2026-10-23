#!/usr/bin/env bash
set -euo pipefail
cd -- '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
evidence=artifacts/validation/2026-10-22/actuation_idf_smoke_v2
sha256sum src/arena_vehicle_interface/portable/*.hpp src/arena_vehicle_interface/firmware_smoke/main/* \
  scripts/actuation_smoke_host.cpp > "$evidence/host_sources_before.sha256"
g++ -std=c++17 -O1 -Wall -Wextra -Werror -pedantic -fno-exceptions -fno-rtti \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  -Isrc/arena_vehicle_interface/portable -Isrc/arena_vehicle_interface/firmware_smoke/main \
  scripts/actuation_smoke_host.cpp -o "$evidence/smoke_host"
"$evidence/smoke_host"
