#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/motion_uncertainty_v1
trial=artifacts/validation/2026-10-28/official_traffic_v1
test ! -e "$base/official_run2_replay_v2"
test ! -e "$base/official_run2_replay_v2.log"
/usr/bin/python3 -B "$base/compare_trace.py" --trace "$trial/run2_course/follow_decisions.jsonl" --output "$base/official_run2_replay_v2" --cases baseline angle combined > "$base/official_run2_replay_v2.log" 2>&1
cat "$base/official_run2_replay_v2.log"
