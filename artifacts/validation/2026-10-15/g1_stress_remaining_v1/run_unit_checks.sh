#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_stress_remaining_v1
test ! -e "$base/preparation_tests.log"
python3 -B -m pytest -q tests/test_occupancy_timing.py tests/test_follow_profile.py tests/test_follow_profile_bundle.py tests/test_g1_lidar_stress.py > "$base/preparation_tests.log" 2>&1
tail -4 "$base/preparation_tests.log"
