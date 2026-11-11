#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="artifacts/validation/2026-11-11/follow_rebase_r126_v1:src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
/usr/bin/python3 -B -m pytest -q artifacts/validation/2026-11-11/follow_rebase_r126_v1/test_rebase.py artifacts/validation/2026-11-11/follow_rebase_r126_v1/test_connection.py tests/test_decision_delivery.py tests/test_decision_publication.py tests/test_decision_node_boundary.py > build/r126_boundary_tests_v2.log 2>&1
