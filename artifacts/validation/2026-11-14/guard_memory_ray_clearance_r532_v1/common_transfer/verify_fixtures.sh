#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q tests/test_independent_guard_route.py tests/test_partial_observation.py tests/test_lidar_safety.py tests/test_guard_memory_command.py tests/test_guard_memory_ray_clearance.py --tb=short --junitxml=artifacts/validation/2026-11-14/guard_memory_ray_clearance_r532_v1/common_transfer/fixtures_regression.xml
