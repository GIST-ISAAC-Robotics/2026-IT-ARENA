#!/usr/bin/env bash
# 10/22 C++ 수신 상태기계 API 검토 수정본: 이전 실패 증거와 분리해 검사한다.
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-22/actuation_receiver_cpp_v2
if [[ -e "$evidence/tests.log" || -e "$evidence/compiler.log" || -e "$evidence/export" ]]; then
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
flags=(-std=c++17 -Wall -Wextra -Werror -pedantic -Isrc/arena_vehicle_interface/portable)
sanitize=(-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
g++ --version > "$evidence/compiler_version.txt"
{
    g++ "${flags[@]}" "${sanitize[@]}" scripts/actuation_receiver_api_checks.cpp -o "$evidence/receiver_api_driver"
    g++ "${flags[@]}" -O2 scripts/actuation_receiver_api_checks.cpp -o "$evidence/receiver_api_driver_o2"
    g++ "${flags[@]}" "${sanitize[@]}" scripts/actuation_receiver_production_api_check.cpp -o "$evidence/receiver_production_api_driver"
    g++ "${flags[@]}" "${sanitize[@]}" scripts/actuation_receiver_cpp_driver.cpp -o "$evidence/receiver_driver"
    g++ "${flags[@]}" -O2 scripts/actuation_receiver_cpp_driver.cpp -o "$evidence/receiver_driver_o2"
    g++ "${flags[@]}" "${sanitize[@]}" scripts/actuation_command_cpp_driver.cpp -o "$evidence/command_driver"
    g++ "${flags[@]}" "${sanitize[@]}" scripts/actuation_link_cpp_driver.cpp -o "$evidence/link_driver"
    g++ "${flags[@]}" "${sanitize[@]}" scripts/actuation_wire_cpp_driver.cpp -o "$evidence/wire_driver"
} > "$evidence/compiler.log" 2>&1
sha256sum "$evidence"/*_driver "$evidence"/*_driver_o2 > "$evidence/binaries.sha256"
export ARENA_RECEIVER_API_BINARY="$PWD/$evidence/receiver_api_driver"
export ARENA_RECEIVER_PRODUCTION_API_BINARY="$PWD/$evidence/receiver_production_api_driver"
export ARENA_RECEIVER_CPP_BINARY="$PWD/$evidence/receiver_driver"
export ARENA_COMMAND_CPP_BINARY="$PWD/$evidence/command_driver"
export ARENA_LINK_CPP_BINARY="$PWD/$evidence/link_driver"
export ARENA_WIRE_CPP_BINARY="$PWD/$evidence/wire_driver"
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_receiver_api.py tests/test_actuation_receiver_cpp.py tests/test_actuation_command_cpp.py \
    tests/test_actuation_link_cpp.py tests/test_actuation_wire_cpp.py tests/test_actuation_transport_conformance.py \
    tests/test_actuation_conformance.py tests/test_actuation_contract.py tests/test_actuation_ros_nodes.py \
    tests/test_actuation_link_budget.py > "$evidence/tests.log" 2>&1
python3 -B scripts/validate_actuation_receiver_cpp.py --binary "$ARENA_RECEIVER_CPP_BINARY" \
    --output "$evidence/export" > "$evidence/export.log" 2>&1
python3 -B scripts/validate_actuation_receiver_cpp.py --binary "$PWD/$evidence/receiver_driver_o2" \
    --output "$evidence/export_o2" > "$evidence/export_o2.log" 2>&1
python3 -B scripts/check_actuation_receiver_mutants.py --output "$evidence/mutants" > "$evidence/mutants.log" 2>&1
ARENA_RECEIVER_API_BINARY="$PWD/$evidence/receiver_api_driver_o2" python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_receiver_api.py > "$evidence/api_o2_tests.log" 2>&1
python3 -B -m pytest -q -p no:cacheprovider tests > "$evidence/full_tests.log" 2>&1
