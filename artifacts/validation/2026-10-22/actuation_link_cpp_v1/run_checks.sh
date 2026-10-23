#!/usr/bin/env bash
# 10/22 FrameLink 후보(부분 프레임 절대 기한·연결/오류 정책)와 기존 프레임 코어 회귀를 PC에서만 검사한다.
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-22/actuation_link_cpp_v1
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
    g++ "${flags[@]}" "${sanitize[@]}" scripts/actuation_link_cpp_driver.cpp -o "$evidence/link_driver"
    g++ "${flags[@]}" "${sanitize[@]}" scripts/actuation_wire_cpp_driver.cpp -o "$evidence/wire_driver"
    g++ "${flags[@]}" -O2 scripts/actuation_link_cpp_driver.cpp -o "$evidence/link_driver_o2"
} > "$evidence/compiler.log" 2>&1
sha256sum "$evidence/link_driver" "$evidence/wire_driver" "$evidence/link_driver_o2" > "$evidence/binaries.sha256"
export ARENA_LINK_CPP_BINARY="$PWD/$evidence/link_driver"
export ARENA_WIRE_CPP_BINARY="$PWD/$evidence/wire_driver"
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_link_cpp.py tests/test_actuation_wire_cpp.py \
    tests/test_actuation_transport_conformance.py tests/test_actuation_conformance.py \
    tests/test_actuation_contract.py tests/test_actuation_ros_nodes.py tests/test_actuation_link_budget.py \
    > "$evidence/tests.log" 2>&1
python3 -B scripts/validate_actuation_link_cpp.py --binary "$ARENA_LINK_CPP_BINARY" \
    --output "$evidence/export" > "$evidence/export.log" 2>&1
python3 -B scripts/validate_actuation_link_cpp.py --binary "$PWD/$evidence/link_driver_o2" \
    --output "$evidence/export_o2" > "$evidence/export_o2.log" 2>&1
python3 -B scripts/validate_actuation_wire_cpp.py --binary "$ARENA_WIRE_CPP_BINARY" \
    --output "$evidence/wire_export" > "$evidence/wire_export.log" 2>&1
python3 -B scripts/check_actuation_link_mutants.py --output "$evidence/mutants" > "$evidence/mutants.log" 2>&1
python3 -B -m pytest -q -p no:cacheprovider tests > "$evidence/full_tests.log" 2>&1
