#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out='artifacts/validation/2026-10-07/g1b_extensions_v1'
test ! -e "$out/pytest_preflight.log"
python3 -B -m pytest -q -p no:cacheprovider tests/test_g1_traffic_extensions.py tests/test_g1_traffic_eval.py tests/test_local_follow.py > "$out/pytest_preflight.log" 2>&1
