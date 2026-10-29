#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/bump_traffic_v1
test ! -e "$base/rolling_failure_v1"
test ! -e "$base/rolling_failure_v1.log"
set +e
/usr/bin/python3 -B "$base/compare_rolling_clearance.py" "$base/run2/follow_decisions.jsonl" --output "$base/rolling_failure_v1" > "$base/rolling_failure_v1.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/rolling_failure_v1.exit"
exit "$result"
