#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-27/g1_pass_release_operation_v1
cfg=artifacts/validation/2026-10-27/g1_pass_release_integration_v1/run4_left/follow_decisions.jsonl
m3=artifacts/validation/2026-10-25/m3_execution_review_v1
set +e
/usr/bin/python3 -B "$base/compare_cross_inputs.py" "$m3/run51_curve_stop_restart/run/follow_decisions.jsonl" --config-from "$cfg" --output "$base/cross_run51_curve_stop_restart.json" > "$base/cross_run51_curve_stop_restart.log" 2>&1
a=$?
/usr/bin/python3 -B "$base/compare_cross_inputs.py" "$m3/run32_corner_after_threat/run/follow_decisions.jsonl" --config-from "$cfg" --output "$base/cross_run32_corner_after_threat.json" > "$base/cross_run32_corner_after_threat.log" 2>&1
b=$?
set -e
printf 'run51=%s\nrun32=%s\n' "$a" "$b" > "$base/cross_compare_exit.txt"
