#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
base=artifacts/validation/2026-10-30/fresh_confirmation_body_v2
test ! -e "$base/exit.txt"
trap 'printf "%s\n" "$?" > "$base/exit.txt"' EXIT
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/$base:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
python3 -B -m pytest -q -p no:cacheprovider "$base/test_body_memory.py" > "$base/tests.log" 2>&1
python3 -B "$base/shadow.py" artifacts/validation/2026-10-30/rear_lane_measured_right_gazebo_v1/drive_right_on/follow_decisions.jsonl "$base/saved_gazebo_right.json" > "$base/saved_gazebo_right.log" 2>&1
python3 -B "$base/shadow.py" artifacts/validation/2026-10-30/rear_lane_measured_gazebo_v1/drive_on/follow_decisions.jsonl "$base/saved_gazebo_left.json" > "$base/saved_gazebo_left.log" 2>&1
python3 -B "$base/shadow.py" artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4/comparison_combined/B_gazebo_right_trace.jsonl "$base/saved_combined_right.json" > "$base/saved_combined_right.log" 2>&1
python3 -B "$base/probe_memory.py" --policy memory --sensor-profile combined --output "$base/probe_combined" --trace --cases B_gazebo_left B_gazebo_right C_lead_stop_left > "$base/probe_combined.log" 2>&1
python3 -B "$base/probe_memory.py" --policy memory --sensor-profile ideal --output "$base/probe_ideal" --trace --cases B_gazebo_left B_gazebo_right > "$base/probe_ideal.log" 2>&1
