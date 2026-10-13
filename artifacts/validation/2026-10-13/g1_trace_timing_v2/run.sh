#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/g1_trace_timing_v2
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_g1_trace_timing.py tests/test_g1_traffic_coverage_audit.py > "$base/tests.log" 2>&1
python3 -B scripts/audit_g1_trace_timing.py artifacts/validation/2026-10-13/g1_position_matrix_v1/side_left_s1 --window 6.059 6.161 --output "$base/report.json" > "$base/audit.log" 2>&1
tail -8 "$base/tests.log"
tail -1 "$base/audit.log"
