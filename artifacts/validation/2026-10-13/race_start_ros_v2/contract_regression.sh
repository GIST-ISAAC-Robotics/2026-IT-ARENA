#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/race_start_ros_v2
test ! -e "$base/contract_regression.log"
python3 -B -m pytest -q tests/test_physics_step_option.py tests/test_follow_trace.py tests/test_r0_safety_contract.py > "$base/contract_regression.log" 2>&1
tail -15 "$base/contract_regression.log"
