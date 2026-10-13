#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/follow_trace_v1
test ! -e "$base/overlap_tests.log"
python3 -B -m pytest -q tests/test_follow_overlap.py > "$base/overlap_tests.log" 2>&1
tail -6 "$base/overlap_tests.log"
test ! -e "$base/overlap_probe.log"
python3 -B scripts/probe_follow_overlap.py "$base/side_left_s0/follow_decisions.jsonl" --output "$base/overlap_probe.json" > "$base/overlap_probe.log" 2>&1
tail -6 "$base/overlap_probe.log"
