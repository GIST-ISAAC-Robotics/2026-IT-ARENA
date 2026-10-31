#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_relative_yield_observer_v1
test ! -e "$base/closed_v3_run1"
test ! -e "$base/closed_v3_run1.log"
test ! -e "$base/test_v3_1.log"
set +e
python3 -B -m pytest -q -p no:cacheprovider "$base/test_observer_v2.py" "$base/test_observer_v3.py" > "$base/test_v3_1.log" 2>&1
test_exit=$?
if [ "$test_exit" != 0 ]; then printf 'tests=%s\n' "$test_exit" > "$base/closed_v3_run1_exit.txt"; exit "$test_exit"; fi
python3 -B "$base/closed_loop_v3.py" --output "$base/closed_v3_run1" > "$base/closed_v3_run1.log" 2>&1
result=$?
printf 'tests=%s\nsynthetic=%s\n' "$test_exit" "$result" > "$base/closed_v3_run1_exit.txt"
exit "$result"
