#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_handoff_corridor_gazebo_v1
test ! -e "$base/run1" || exit 93
timeout 1200 /usr/bin/python3 -B "$base/run.py" --output "$PWD/$base/run1" > "$base/run1_trial.log" 2>&1
code=$?
printf '%s\n' "$code" > "$base/run1_exit.txt"
exit "$code"
