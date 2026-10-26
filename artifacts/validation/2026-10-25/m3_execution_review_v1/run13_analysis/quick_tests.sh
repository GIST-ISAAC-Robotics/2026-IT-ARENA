#!/usr/bin/env bash
set -uo pipefail
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
export PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts:tests
python3 -B -m pytest -q -p no:cacheprovider tests/test_m3_baseline_release.py tests/test_m3_execution_trial.py tests/test_m3_command_selection.py 2>&1 | tail -30
