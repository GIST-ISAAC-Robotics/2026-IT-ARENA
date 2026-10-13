#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_dense_stop_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_g1_dense_stop.py tests/test_g1_stop_pose_coverage.py tests/test_g1_traffic_coverage_audit.py tests/test_g1_traffic_eval.py tests/test_g1_trace_timing.py tests/test_physics_step_option.py > "$base/tests.log" 2>&1
tail -8 "$base/tests.log"
test ! -e "$base/full_tests.log"
python3 -B -m pytest -q tests > "$base/full_tests.log" 2>&1
tail -8 "$base/full_tests.log"
