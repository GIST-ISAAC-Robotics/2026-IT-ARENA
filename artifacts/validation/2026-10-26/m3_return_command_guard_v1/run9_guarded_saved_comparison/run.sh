#!/usr/bin/env bash
set -uo pipefail
repo="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
cd "$repo" || exit 90
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface:$repo/scripts:$repo/tests"
python3 "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-26/m3_return_command_guard_v1/run9_guarded_saved_comparison/preflight.py"
python3 -c 'import numpy,pytest,arena_autonomy; print("numpy",numpy.__version__); print("pytest",pytest.__version__); print("arena_autonomy.__path__",list(arena_autonomy.__path__))'
python3 -I artifacts/validation/2026-10-26/m3_return_command_guard_v1/compare_guarded_trial.py --output artifacts/validation/2026-10-26/m3_return_command_guard_v1/run9_guarded_saved_comparison/data --workers 5
