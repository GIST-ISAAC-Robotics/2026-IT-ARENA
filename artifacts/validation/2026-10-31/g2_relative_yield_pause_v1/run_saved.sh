#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_relative_yield_pause_v1
test ! -e "$base/saved1.log"
set +e
python3 -B "$base/compare_saved.py" > "$base/saved1.log" 2>&1
code=$?
printf 'saved_input=%s\n' "$code" > "$base/saved1_exit.txt"
exit "$code"
