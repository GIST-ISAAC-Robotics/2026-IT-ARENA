#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-16/g1_final_checks_v1
test ! -e "$base/tests.log"
python3 -B "$base/check.py" before > "$base/preflight.log" 2>&1
test_result=0
python3 -B -m pytest -q tests > "$base/tests.log" 2>&1 || test_result=$?
python3 -B "$base/check.py" after > "$base/integrity.log" 2>&1
tail -3 "$base/tests.log"
tail -1 "$base/integrity.log"
exit "$test_result"
