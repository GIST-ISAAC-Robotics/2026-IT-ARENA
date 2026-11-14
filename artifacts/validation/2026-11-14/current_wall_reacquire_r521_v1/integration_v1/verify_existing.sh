#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q tests/test_local_follow.py tests/test_local_follow_validation.py tests/test_moving_pass.py tests/test_moving_pass_current_return.py tests/test_moving_pass_wall_padding.py tests/test_moving_path_preview.py tests/test_current_route_proof.py tests/test_runtime_dual_contract.py
