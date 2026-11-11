#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="artifacts/validation/2026-11-11/guard_recheck_r139_v1:src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
/usr/bin/python3 -B -m pytest -q artifacts/validation/2026-11-11/guard_recheck_r139_v1/test_recheck.py tests/test_guard_curve_trial.py tests/test_guard_masked_speed.py tests/test_rear_fringe_trial.py tests/test_released_rear_sweep.py > build/r139_tests.log 2>&1
