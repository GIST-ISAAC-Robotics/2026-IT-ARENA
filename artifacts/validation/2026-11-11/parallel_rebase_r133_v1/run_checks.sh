#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="artifacts/validation/2026-11-11/parallel_rebase_r133_v1:src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
/usr/bin/python3 -B -m pytest -q --import-mode=importlib artifacts/validation/2026-11-11/parallel_rebase_r133_v1/test_parallel.py artifacts/validation/2026-11-11/follow_rebase_r126_v1/test_rebase.py artifacts/validation/2026-11-11/parallel_rebase_r133_v1/test_connection.py tests/test_decision_delivery.py tests/test_parallel_lane_trial.py > build/r133_tests.log 2>&1
