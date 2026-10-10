#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base='artifacts/validation/2026-10-08/braking_ab_v1'
test ! -e "$base/pytest_preflight.log"
python3 -B -m pytest -q -p no:cacheprovider tests/test_braking_command_profile.py tests/test_wheel_contact_probe.py tests/test_wheel_contact_analysis.py tests/test_drive_feedback.py tests/test_motor_encoder_validation.py > "$base/pytest_preflight.log" 2>&1
