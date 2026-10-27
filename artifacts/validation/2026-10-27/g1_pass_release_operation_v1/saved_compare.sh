#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-27/g1_pass_release_operation_v1
set +e
/usr/bin/python3 -B "$base/compare_saved_selector.py" artifacts/validation/2026-10-27/g1_partial_traffic_v1/run1_stop_restart/follow_decisions.jsonl --output "$base/saved_stop_restart_on.json" > "$base/saved_stop_restart_on.log" 2>&1
a=$?
/usr/bin/python3 -B "$base/compare_saved_selector.py" artifacts/validation/2026-10-27/g1_partial_integration_v1/run11_lap/follow_decisions.jsonl --output "$base/saved_run11_lap_on.json" > "$base/saved_run11_lap_on.log" 2>&1
b=$?
set -e
printf 'stop_restart=%s\nrun11=%s\n' "$a" "$b" > "$base/saved_compare_exit.txt"
