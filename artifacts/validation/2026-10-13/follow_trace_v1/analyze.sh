#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/follow_trace_v1
test ! -e "$base/analysis_tests.log"
python3 -B -m pytest -q tests/test_follow_trace_analysis.py > "$base/analysis_tests.log" 2>&1
tail -6 "$base/analysis_tests.log"
test ! -e "$base/analysis.log"
python3 -B scripts/analyze_follow_trace.py "$base/side_left_s0/follow_decisions.jsonl" --output "$base/analysis.json" > "$base/analysis.log" 2>&1
tail -6 "$base/analysis.log"
