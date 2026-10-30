#!/usr/bin/env bash
# ROS 환경 누락으로 중단된 run.sh의 미실행 합성만 수행한다.
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
base=artifacts/validation/2026-10-30/fresh_confirmation_body_v2
test ! -e "$base/resume_exit.txt"
trap 'printf "%s\n" "$?" > "$base/resume_exit.txt"' EXIT
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/$base:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$base/probe_memory.py" --policy memory --sensor-profile combined --output "$base/probe_combined" --trace --cases B_gazebo_left B_gazebo_right C_lead_stop_left > "$base/probe_combined_r2.log" 2>&1
python3 -B "$base/probe_memory.py" --policy memory --sensor-profile ideal --output "$base/probe_ideal" --trace --cases B_gazebo_left B_gazebo_right > "$base/probe_ideal.log" 2>&1
