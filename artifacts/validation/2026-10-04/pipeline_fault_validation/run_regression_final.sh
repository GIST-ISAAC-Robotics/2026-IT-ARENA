#!/usr/bin/env bash
set -euo pipefail

cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"

set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u

export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"

python3 -m pytest -q \
  tests/test_pipeline_fault_validation.py tests/test_lidar_shutdown.py tests/test_vision_process.py \
  tests/test_actuation_contract.py tests/test_actuation_ros_nodes.py tests/test_drive_feedback.py \
  tests/test_lidar_safety.py tests/test_lidar_motion_ros.py \
  2>&1 | tee artifacts/validation/2026-10-04/pipeline_fault_validation/regression_final.log
