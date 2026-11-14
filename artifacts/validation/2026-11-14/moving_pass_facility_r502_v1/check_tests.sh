#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m pytest -q tests/test_moving_path_preview.py tests/test_moving_pass.py tests/test_moving_pass_medium.py tests/test_moving_pass_faster.py tests/test_moving_pass_current_return.py tests/test_adaptive_entry_preview.py
