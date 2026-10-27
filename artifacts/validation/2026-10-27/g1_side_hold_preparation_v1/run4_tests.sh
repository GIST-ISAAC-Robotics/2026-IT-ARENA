#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/g1_side_hold_preparation_v1
test ! -e "$base/run4_tests.log"
/usr/bin/python3 -B -m pytest -q tests/test_parallel_lane_trial.py tests/test_g1_partial_policy.py tests/test_g1_partial_traffic.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_source_provenance_r0_plus_b3.py > "$base/run4_tests.log" 2>&1
tail -5 "$base/run4_tests.log"
