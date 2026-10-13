#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/clock_bounds_v1
test ! -e "$base/full_regression_retry.log"
python3 -B -c 'import sys, xacro; print(sys.executable); print(xacro.__file__); print(sys.path)' > "$base/full_regression_retry.log" 2>&1
python3 -B -m pytest -q tests >> "$base/full_regression_retry.log" 2>&1
tail -10 "$base/full_regression_retry.log"
