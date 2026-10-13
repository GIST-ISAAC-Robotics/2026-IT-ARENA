#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_lidar_stress_v1
test ! -e "$base/unit_tests_v3.log"
python3 -B -m pytest -q tests/test_g1_lidar_stress.py tests/test_lidar_impairment.py > "$base/unit_tests_v3.log" 2>&1
tail -3 "$base/unit_tests_v3.log"
