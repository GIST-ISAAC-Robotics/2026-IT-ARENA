#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-30/g2_merge_gazebo_v1
python3 -B -m pytest -q "$base/test_saved_alias.py" > "$base/alias_tests1.log" 2>&1
python3 -B "$base/reevaluate_saved.py" > "$base/reevaluation1.log" 2>&1
