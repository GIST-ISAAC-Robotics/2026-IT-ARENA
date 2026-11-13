#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
a='artifacts/validation/2026-11-14/traffic_current_geometry_r496_v1'
test ! -e "$a/native_v1/receipt.json"
mkdir -p "$a/native_v1"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
set +e
timeout --kill-after=5s 180s python3 -B "$a/run_native_regression.py" > "$a/native_v1/pytest.log" 2>&1
rc=$?
set -e
printf '%s\n' "$rc" > "$a/native_v1/run.exit"
exit "$rc"
