#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
a='artifacts/validation/2026-11-14/traffic_return_recovery_r490_v1'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
set +e
python3 "$a/stop_owned.py" > "$a/execution_v1/stop_helper.log" 2>&1
rc=$?
set -e
printf '%s\n' "$rc" > "$a/execution_v1/stop_helper.exit"
cat "$a/execution_v1/stop_helper.log"
exit "$rc"
