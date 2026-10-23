#!/usr/bin/env bash
# 사용자 마감 요청 당시 작성 중인 프레임 코어만 컴파일·대조하고 종료한다.
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-21/actuation_wire_cpp_v1
if [[ -e "$evidence/tests.log" || -e "$evidence/compiler.log" ]]; then
    printf 'Fresh evidence path required\n' >&2
    exit 2
fi
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
tar -czf "$evidence/source_snapshot_before.tar.gz" src scripts tests config
finish() {
    result=$?
    find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
    diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || true
    printf 'EXIT_STATUS=%s\n' "$result" > "$evidence/runner_status.txt"
}
trap finish EXIT
g++ --version > "$evidence/compiler_version.txt"
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -O1 -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -Isrc/arena_vehicle_interface/portable scripts/actuation_wire_cpp_driver.cpp \
    -o "$evidence/wire_driver" > "$evidence/compiler.log" 2>&1
export ARENA_WIRE_CPP_BINARY="$PWD/$evidence/wire_driver"
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_wire_cpp.py \
    tests/test_actuation_transport_conformance.py tests/test_actuation_conformance.py \
    tests/test_actuation_contract.py tests/test_actuation_ros_nodes.py tests/test_actuation_link_budget.py \
    > "$evidence/tests.log" 2>&1
python3 -B scripts/validate_actuation_wire_cpp.py --binary "$ARENA_WIRE_CPP_BINARY" \
    --output "$evidence/export" > "$evidence/export.log" 2>&1
