#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact='artifacts/validation/2026-11-14/solo_guard_recovery_r492_v1'
test ! -e "$artifact/execution_prepared.json"
test ! -e "$artifact/runtime_preflight/result.json"
mkdir "$artifact/preflight_v2"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$artifact/runtime_preflight.py" > "$artifact/preflight_v2/import.log" 2>&1
python3 -B "$artifact/prepare_execution.py" --freeze > "$artifact/preflight_v2/freeze.json"
python3 -B "$artifact/prepare_execution.py" --check > "$artifact/preflight_v2/final_check.json"
printf '%s\n' 'r492 import/freeze complete; no build/ROS init/Gazebo run'
