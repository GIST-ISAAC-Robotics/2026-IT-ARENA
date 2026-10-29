#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_entry_gazebo_v1
test ! -e "$base/contracts1.log" || exit 90
/usr/bin/python3 -B -m pytest -q -p no:cacheprovider tests/test_bypass*.py tests/test_follow_trace.py > "$base/contracts1.log" 2>&1
result=$?
printf '{"tests":%s}\n' "$result" > "$base/contracts1_exit.json"
exit "$result"
