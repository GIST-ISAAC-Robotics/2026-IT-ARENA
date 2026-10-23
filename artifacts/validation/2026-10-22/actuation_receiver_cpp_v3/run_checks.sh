#!/usr/bin/env bash
# v2의 core/API 실행 파일은 그대로 사용하고, 잘못된 변이 주입 위치를 고친 뒤 미완료 검사를 마친다.
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-22/actuation_receiver_cpp_v3
previous=artifacts/validation/2026-10-22/actuation_receiver_cpp_v2
if [[ -e "$evidence/source_snapshot_before.tar.gz" || -e "$evidence/mutants" ]]; then
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
# 실행 파일 해시 및 그것을 만든 모든 portable 헤더/driver 소스가 v2와 같음을 먼저 확인한다.
sha256sum -c "$previous/binaries.sha256" > "$evidence/reused_binaries_check.log" 2>&1
awk '$2 ~ /^src\/arena_vehicle_interface\/portable\// || $2 ~ /^scripts\/actuation_.*(cpp_driver|api_checks|production_api_check)\.cpp$/' \
    "$previous/sources_before.sha256" > "$evidence/reused_sources.sha256"
test -s "$evidence/reused_sources.sha256"
sha256sum -c "$evidence/reused_sources.sha256" > "$evidence/reused_sources_check.log" 2>&1
export ARENA_RECEIVER_API_BINARY="$PWD/$previous/receiver_api_driver"
export ARENA_RECEIVER_PRODUCTION_API_BINARY="$PWD/$previous/receiver_production_api_driver"
export ARENA_RECEIVER_CPP_BINARY="$PWD/$previous/receiver_driver"
export ARENA_COMMAND_CPP_BINARY="$PWD/$previous/command_driver"
export ARENA_LINK_CPP_BINARY="$PWD/$previous/link_driver"
export ARENA_WIRE_CPP_BINARY="$PWD/$previous/wire_driver"
python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_receiver_mutants.py > "$evidence/mutation_anchors.log" 2>&1
python3 -B scripts/check_actuation_receiver_mutants.py --output "$evidence/mutants" > "$evidence/mutants.log" 2>&1
ARENA_RECEIVER_API_BINARY="$PWD/$previous/receiver_api_driver_o2" python3 -B -m pytest -q -p no:cacheprovider tests/test_actuation_receiver_api.py > "$evidence/api_o2_tests.log" 2>&1
python3 -B -m pytest -q -p no:cacheprovider tests > "$evidence/full_tests.log" 2>&1
