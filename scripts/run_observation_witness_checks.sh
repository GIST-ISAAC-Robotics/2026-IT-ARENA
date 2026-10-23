#!/usr/bin/env bash
# 인계 진단 검사를 재현한다. 소스 수정/설치/ROS 실행 없이 pytest 종료값을 보존한다.
set -u
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." || exit 2
if [[ $# -ne 1 || -e "$1" ]]; then
    printf 'Usage: bash scripts/run_observation_witness_checks.sh NEW_LOG_PATH\n' >&2
    exit 2
fi
export PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts
python3 -m pytest tests/test_observation_witness.py tests/test_observation_lateral.py \
    tests/test_observation_envelope.py -q > "$1" 2>&1
test_rc=$?
printf '\nPYTEST_EXIT_CODE=%s\n' "$test_rc" >> "$1"
tail -n 35 -- "$1"
exit "$test_rc"
