#!/usr/bin/env bash
set -uo pipefail
repo="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
cd "$repo" || exit 90
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface:$repo/scripts:$repo/tests"
python3 "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-26/m3_return_command_guard_v1/run7_guarded_return_regression/preflight.py"
python3 -c 'import numpy,pytest,arena_autonomy; print("numpy",numpy.__version__); print("pytest",pytest.__version__); print("arena_autonomy.__path__",list(arena_autonomy.__path__))'
python3 -m pytest -q tests/test_m3_guarded_return_trial.py tests/test_m3_return_guard.py
