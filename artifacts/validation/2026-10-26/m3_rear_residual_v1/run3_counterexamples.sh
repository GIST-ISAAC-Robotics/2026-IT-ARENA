#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
D=artifacts/validation/2026-10-26/m3_rear_residual_v1
O="$D/run3_counterexamples"
[ ! -e "$O" ] || exit 91
python3 -I -B "$D/counterexamples.py" --workers 12 --output "$O" > "$D/run3_stdout.txt" 2> "$D/run3_stderr.txt"
code=$?
printf '%s\n' "$code" > "$D/run3_exit_code.txt"
exit "$code"
