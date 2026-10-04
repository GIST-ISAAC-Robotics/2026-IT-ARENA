#!/usr/bin/env bash
# WSL 전용 실행 기록. PowerShell에서 $?를 보간하지 않는다.
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
python3 -m pytest tests/test_lidar_observation.py tests/test_lidar_safety.py tests/test_lidar_motion.py tests/test_lidar_motion_ros.py tests/test_drivetrain_model.py -q
python3 scripts/validate_sensor_input_ros.py --output artifacts/validation/2026-10-04/sensor_input_validation/ros_fault_v1
python3 artifacts/validation/2026-10-04/sensor_input_validation/probe_inputs.py
