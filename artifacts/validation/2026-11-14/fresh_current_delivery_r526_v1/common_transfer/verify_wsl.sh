#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -q tests/test_fresh_current_delivery.py tests/test_runtime_dual_contract.py tests/test_runtime_dual_slots.py tests/test_runtime_plan_trace.py tests/test_runtime_node_backend.py tests/test_compact_follow_trace.py --tb=short
