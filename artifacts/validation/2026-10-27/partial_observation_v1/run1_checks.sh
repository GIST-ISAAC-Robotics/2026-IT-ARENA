#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/partial_observation_v1
test ! -e "$base/run1_tests.log"
python3 -B -m pytest -q tests/test_partial_observation.py tests/test_lidar_observation.py tests/test_lidar_safety.py tests/test_recent_observation_safety.py tests/test_g1_solo_candidate_validation.py > "$base/run1_tests.log" 2>&1
tail -5 "$base/run1_tests.log"
