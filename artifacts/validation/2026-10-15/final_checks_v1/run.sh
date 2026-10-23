#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/final_checks_v1
test ! -e "$base/full_tests.log"
git rev-parse HEAD > "$base/base_commit.txt"
date -u --iso-8601=seconds > "$base/started_utc.txt"
set +e
python3 -B -m pytest -q tests > "$base/full_tests.log" 2>&1
test_exit=$?
set -e
printf 'full_tests=%s\n' "$test_exit" > "$base/exit.txt"
tail -8 "$base/full_tests.log"
exit "$test_exit"
