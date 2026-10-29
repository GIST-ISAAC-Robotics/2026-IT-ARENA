#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_side_scope_v1
test ! -e "$base/complete_set_comparison.json"
python3 -B "$base/compare_complete_set.py" > "$base/complete_set_comparison.log" 2>&1
python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course/follow_decisions.jsonl --output "$base/recovery_off_replay.json" > "$base/recovery_off_replay.log" 2>&1
