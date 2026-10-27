#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
/usr/bin/python3 -B -m pytest -q -x tests/test_g1_partial_policy.py tests/test_local_follow.py tests/test_follow_trace.py tests/test_corner_follow.py tests/test_source_provenance_r0_plus_b3.py > artifacts/validation/2026-10-27/g1_partial_integration_v1/run2_tests.log 2>&1
