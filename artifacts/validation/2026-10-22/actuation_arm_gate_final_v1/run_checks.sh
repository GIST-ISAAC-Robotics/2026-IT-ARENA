#!/usr/bin/env bash
# ARM 관문 후 전체 portable 명시 빌드 + 회귀. 운영/보드/ROS 그래프 변경 없음.
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-22/actuation_arm_gate_final_v1
test ! -e "$evidence/checks_exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
tar --exclude='__pycache__' -czf "$evidence/source_snapshot.tar.gz" src scripts tests config
finish() {
  result=$?
  find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || true
  printf '%s\n' "$result" > "$evidence/checks_exit.txt"
}
trap finish EXIT
flags=(-std=c++17 -Wall -Wextra -Werror -pedantic -Isrc/arena_vehicle_interface/portable -O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
{
  g++ "${flags[@]}" scripts/actuation_receiver_api_checks.cpp -o "$evidence/receiver_api_driver"
  g++ "${flags[@]}" scripts/actuation_receiver_production_api_check.cpp -o "$evidence/receiver_production_api_driver"
  g++ "${flags[@]}" scripts/actuation_receiver_cpp_driver.cpp -o "$evidence/receiver_driver"
  g++ "${flags[@]}" scripts/actuation_command_cpp_driver.cpp -o "$evidence/command_driver"
  g++ "${flags[@]}" scripts/actuation_link_cpp_driver.cpp -o "$evidence/link_driver"
  g++ "${flags[@]}" scripts/actuation_wire_cpp_driver.cpp -o "$evidence/wire_driver"
} > "$evidence/compiler.log" 2>&1
sha256sum "$evidence"/*_driver > "$evidence/binaries.sha256"
export ARENA_RECEIVER_API_BINARY="$PWD/$evidence/receiver_api_driver"
export ARENA_RECEIVER_PRODUCTION_API_BINARY="$PWD/$evidence/receiver_production_api_driver"
export ARENA_RECEIVER_CPP_BINARY="$PWD/$evidence/receiver_driver"
export ARENA_COMMAND_CPP_BINARY="$PWD/$evidence/command_driver"
export ARENA_LINK_CPP_BINARY="$PWD/$evidence/link_driver"
export ARENA_WIRE_CPP_BINARY="$PWD/$evidence/wire_driver"
python3 -B -m pytest -q -p no:cacheprovider tests > "$evidence/full_tests.log" 2>&1
