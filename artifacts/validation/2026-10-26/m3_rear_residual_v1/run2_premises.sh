#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
D=artifacts/validation/2026-10-26/m3_rear_residual_v1
O="$D/run2_premises"
[ ! -e "$O" ] || exit 91
python3 -I -B "$D/compare_release_premises.py" --plan "$D/run2_plan.json" --workers 8 --output "$O" > "$D/run2_stdout.txt" 2> "$D/run2_stderr.txt"
code=$?
printf '%s\n' "$code" > "$D/run2_exit_code.txt"
exit "$code"
