#!/usr/bin/env bash
# r41 기록 1회 기준 재생 + 1회 cProfile 재생. ROS·Gazebo·Jetson 미사용. PYTHON/ROS 환경 제거.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-11-08/r41_cost_breakdown_v1
env -u PYTHONPATH -u PYTHONHOME -u ROS_DISTRO -u AMENT_PREFIX_PATH python3 -I -B $A/profile_r41.py > $A/raw/profile_run.log 2>&1
code=$?
echo "ACTUAL_EXIT_CODE=$code" > $A/profile_run.exit
exit "$code"
