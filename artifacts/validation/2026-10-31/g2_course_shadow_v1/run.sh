#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
python3 -B artifacts/validation/2026-10-31/g2_course_shadow_v1/compare.py > artifacts/validation/2026-10-31/g2_course_shadow_v1/run1.log 2>&1
