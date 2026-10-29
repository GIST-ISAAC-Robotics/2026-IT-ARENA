#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_anchor_set_v1
test ! -e "$base/replay_fixed.json"
python3 -B -m pytest -q -p no:cacheprovider tests/test_replay_anchor_cost.py tests/test_follow_trace.py tests/test_g1_partial_policy.py tests/test_follow_trace_analysis.py > "$base/replay_regression.log" 2>&1
python3 -B scripts/replay_follow_trace.py artifacts/validation/2026-10-28/traffic_path_recovery_connection_v1/run1_course/follow_decisions.jsonl --output "$base/replay_fixed.json" > "$base/replay_fixed.log" 2>&1
