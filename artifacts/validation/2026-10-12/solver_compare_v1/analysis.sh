#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-12/solver_compare_v1
test ! -e "$base/analysis.log"
python3 -B -m pytest -q tests/test_solver_compare.py tests/test_solver_compare_analysis.py tests/test_scheduled_surface_analysis.py tests/test_contact_model_analysis.py > "$base/analysis_tests.log" 2>&1
tail -5 "$base/analysis_tests.log"
python3 -B scripts/analyze_solver_compare.py --base "$base" --output "$base/analysis.json" > "$base/analysis.log" 2>&1
tail -80 "$base/analysis.log"
