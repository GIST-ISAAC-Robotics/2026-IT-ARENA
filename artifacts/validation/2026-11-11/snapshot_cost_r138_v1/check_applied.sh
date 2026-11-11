#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
/usr/bin/python3 -B -m pytest -q tests/test_snapshot_copy.py tests/test_follow_trace.py tests/test_decision_process.py tests/test_decision_delivery.py tests/test_decision_publication.py tests/test_native_scan_pipeline.py > build/r138_applied_tests.log 2>&1
