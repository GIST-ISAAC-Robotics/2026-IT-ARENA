#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/g1_solo_lap_v1
python3 -B scripts/analyze_follow_trace.py "$base/run/follow_decisions.jsonl" --output "$base/boundary_analysis.json" > "$base/boundary_analysis.log" 2>&1
cat "$base/boundary_analysis.log"
