#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_relative_yield_connection_v1
test ! -e "$base/run2"
test ! -e "$base/run2_tests.log"
set +e
python3 -B -m pytest -q -p no:cacheprovider tests/test_bypass*.py > "$base/run2_tests.log" 2>&1
tests_exit=$?
if [ "$tests_exit" != 0 ]; then printf 'tests=%s\n' "$tests_exit" > "$base/run2_exit.txt"; exit "$tests_exit"; fi
python3 -B "$base/closed_loop.py" --output "$base/run2" > "$base/run2.log" 2>&1
result=$?
printf 'tests=%s\nsynthetic_replay=%s\n' "$tests_exit" "$result" > "$base/run2_exit.txt"
exit "$result"
