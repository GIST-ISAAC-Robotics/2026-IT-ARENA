#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_stop_coverage_v1
test ! -e "$base/after_tests.log"
python3 -B -m pytest -q tests/test_g1_stop_pose_coverage.py tests/test_g1_traffic_coverage_audit.py tests/test_g1_trace_timing.py > "$base/after_tests.log" 2>&1
tail -8 "$base/after_tests.log"
python3 -B "$base/recheck.py" > "$base/recheck.log" 2>&1
tail -1 "$base/recheck.log"
