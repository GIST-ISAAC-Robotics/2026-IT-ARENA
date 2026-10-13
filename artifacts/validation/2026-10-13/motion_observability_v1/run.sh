#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/motion_observability_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_motor_observability.py tests/test_motor_encoder_validation.py tests/test_drive_feedback.py tests/test_actuation_contract.py tests/test_actuation_ros_nodes.py tests/test_local_follow.py tests/test_local_follow_validation.py tests/test_lidar_motion.py tests/test_lidar_motion_ros.py > "$base/tests.log" 2>&1
tail -5 "$base/tests.log"
test ! -e "$base/analysis.log"
python3 -B scripts/analyze_motor_observability.py --output "$base/analysis.json" > "$base/analysis.log" 2>&1
tail -70 "$base/analysis.log"
