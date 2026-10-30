#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-30/fresh_confirmation_review_v1
old=artifacts/validation/2026-10-30/fresh_confirmation_memory_v1
test ! -e "$base/exit.txt"
trap 'printf "%s\n" "$?" > "$base/exit.txt"' EXIT
export PYTHONDONTWRITEBYTECODE=1
python3 -B "$base/analyze.py" artifacts/validation/2026-10-30/rear_lane_measured_right_gazebo_v1/drive_right_on/follow_decisions.jsonl --output "$base/gazebo_right" > "$base/gazebo_right.log" 2>&1
python3 -B "$base/analyze.py" artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4/comparison_combined/B_gazebo_right_trace.jsonl --output "$base/combined_right_saved" > "$base/combined_right_saved.log" 2>&1
python3 -B "$base/analyze.py" "$old/probe_memory_combined/B_gazebo_right_trace.jsonl" --output "$base/combined_right_candidate" > "$base/combined_right_candidate.log" 2>&1
