#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests"
A=artifacts/validation/2026-11-08/faster_entry_consistency_r44_v1
python3 -B "$A/legacy_compare.py" > "$A/legacy_compare_v1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/legacy_compare_v1.exit"
exit "$rc"
