#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_handoff_corridor_gazebo_v1
test ! -e "$base/reevaluation1.log"
python3 -B -m pytest -q "$base/test_evaluation_modes.py" > "$base/evaluation_tests1.log" 2>&1
python3 -B "$base/reevaluate.py" > "$base/reevaluation1.log" 2>&1
