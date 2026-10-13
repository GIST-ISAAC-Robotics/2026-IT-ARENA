#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-13/follow_trace_v1
test ! -e "$base/plot.log"
python3 -B scripts/plot_follow_snapshot.py "$base/side_left_s0/follow_decisions.jsonl" --output "$base/boundary_snapshots.png" > "$base/plot.log" 2>&1
