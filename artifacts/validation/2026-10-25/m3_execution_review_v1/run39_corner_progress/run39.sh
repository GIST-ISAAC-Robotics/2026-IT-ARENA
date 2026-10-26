#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
D=artifacts/validation/2026-10-25/m3_execution_review_v1/run39_corner_progress
[ ! -e "$D/result.json" ] || exit 91
cp artifacts/validation/2026-10-25/m3_execution_review_v1/analyze_corner_progress.py "$D/probe_source.py"
python3 -B "$D/probe_source.py" "$D/result.json" > "$D/stdout.txt" 2> "$D/stderr.txt"
result_code=$?
printf '%s\n' "$result_code" > "$D/exit_code.txt"
exit "$result_code"
