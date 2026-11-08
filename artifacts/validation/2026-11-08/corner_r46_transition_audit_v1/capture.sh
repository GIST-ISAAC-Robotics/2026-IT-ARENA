#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
A=artifacts/validation/2026-11-08/corner_r46_transition_audit_v1
python3 -B "$A/capture.py" > "$A/capture_v2.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/capture_v2.exit"
exit "$rc"
