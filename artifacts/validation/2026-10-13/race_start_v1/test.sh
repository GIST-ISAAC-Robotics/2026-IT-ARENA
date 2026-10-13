#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/race_start_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_official_start_signal.py tests/test_race_permission.py tests/test_basic_autonomy.py tests/test_vision_process.py tests/test_signal_mask.py > "$base/tests.log" 2>&1
tail -15 "$base/tests.log"
