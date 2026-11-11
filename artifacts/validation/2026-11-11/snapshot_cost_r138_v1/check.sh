#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="artifacts/validation/2026-11-11/snapshot_cost_r138_v1:src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
/usr/bin/python3 -B -m pytest -q artifacts/validation/2026-11-11/snapshot_cost_r138_v1/test_snapshot.py tests/test_follow_trace.py > build/r138_tests.log 2>&1
