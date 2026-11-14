#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/tests:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID=223
export ROS_LOCALHOST_ONLY=1
art=artifacts/validation/2026-11-14/ray_order_followup_r554_v1/common_transfer
python3 -B -m pytest -q tests/test_bypass_ray_bearings.py tests/test_bypass_entry_trial.py tests/test_bypass_entry_guide.py tests/test_bypass_wall_fit.py tests/test_bypass_handoff.py tests/test_moving_pass.py tests/test_moving_pass_current_return.py --junitxml="$art/junit.xml" > "$art/pytest.log" 2>&1
