#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONPATH='src/arena_autonomy:src/arena_vehicle_interface:scripts:tests'
D=artifacts/validation/2026-10-25/m3_execution_review_v1/run32_corner_after_threat
if [ -e "$D/straight_commitment.json" ]; then exit 93; fi
python3 -B artifacts/validation/2026-10-25/m3_execution_review_v1/probe_straight_commitment.py "$D/run/follow_decisions.jsonl" "$D/straight_commitment.json" > "$D/commitment_stdout.txt" 2> "$D/commitment_stderr.txt"
result_code=$?
printf '%s\n' "$result_code" > "$D/commitment_exit_code.txt"
exit "$result_code"
