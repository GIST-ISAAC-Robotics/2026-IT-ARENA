#!/usr/bin/env bash
set -eo pipefail
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
work='/home/jinhyeong/it-arena-r466-ros-a'
source /opt/ros/jazzy/setup.bash
source "$repo/install/setup.bash"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$work/src/arena_autonomy:$work/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$repo/artifacts/validation/2026-11-13/vehicle_marker_r466_v1/ros_validation/diagnose_duplicates.py"
