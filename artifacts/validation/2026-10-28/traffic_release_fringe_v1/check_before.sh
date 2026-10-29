#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
base=artifacts/validation/2026-10-28/traffic_release_fringe_v1
test ! -e "$base/before_tests.log"
set +e
python3 -B -m pytest -q -p no:cacheprovider "$base/test_release_fringe_candidate.py" > "$base/before_tests.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/before_tests.exit"
tail -25 "$base/before_tests.log"
exit "$result"
