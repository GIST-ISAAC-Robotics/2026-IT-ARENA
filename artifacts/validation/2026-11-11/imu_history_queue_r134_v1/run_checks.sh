#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
/usr/bin/python3 -B -m pytest -q artifacts/validation/2026-11-11/imu_history_queue_r134_v1/test_history_contract.py tests/test_lidar_motion_ros.py tests/test_lidar_motion.py tests/test_autonomy_subscription_audit.py > build/r134_history_tests.log 2>&1
