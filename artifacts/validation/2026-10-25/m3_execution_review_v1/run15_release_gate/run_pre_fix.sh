#!/usr/bin/env bash
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
D=artifacts/validation/2026-10-25/m3_execution_review_v1/run15_release_gate
export PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts:tests
python3 -B "$D/release_scenarios.py" "$D/pre_fix_scenarios.json" > "$D/pre_fix_stdout.txt" 2> "$D/pre_fix_stderr.txt"
echo $? > "$D/pre_fix_exit_code.txt"
