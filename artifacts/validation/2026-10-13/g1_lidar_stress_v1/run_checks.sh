#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_lidar_stress_v1
test ! -e "$base/integration_tests.log"
python3 -B -m pytest -q tests/test_g1_lidar_stress.py tests/test_lidar_impairment.py tests/test_g1_dense_stop.py tests/test_g1_traffic_eval.py tests/test_g1_traffic_extensions.py tests/test_g1_side_occlusion.py tests/test_g1_case_evidence.py tests/test_physics_step_option.py > "$base/integration_tests.log" 2>&1
tail -4 "$base/integration_tests.log"
test ! -e "$base/full_tests.log"
python3 -B -m pytest -q tests > "$base/full_tests.log" 2>&1
tail -5 "$base/full_tests.log"
