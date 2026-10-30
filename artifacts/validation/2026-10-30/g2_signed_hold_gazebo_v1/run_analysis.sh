#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
set -e
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_signed_hold_gazebo_v1
/usr/bin/python3 -B "$base/analyze_run.py" > "$base/analyze_run.log" 2>&1
/usr/bin/python3 -B "$base/analyze_recovery.py" > "$base/analyze_recovery.log" 2>&1
