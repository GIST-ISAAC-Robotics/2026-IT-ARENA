#!/usr/bin/env bash
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
cd "$HOME/arena_scratch/right_angle_v1/ws2" || exit 2
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
python3 -B "$D/trial_patch/dbg_polyline.py" "$D/$1" "$2"
