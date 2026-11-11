#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
/usr/bin/python3 -m pytest -q tests/test_recent_boundary_confirmation.py tests/test_junction_priority.py tests/test_boundary_bridge.py tests/test_local_follow.py tests/test_traffic_boundary_refresh.py tests/test_gapped_boundary_evidence.py > build/r130_tests_v3.log 2>&1
