#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/right_side_newcourse_v1
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/review.py" > "$A/review.log" 2>&1
status=$?
printf '%s\n' "$status" > "$A/review.exit.txt"
exit "$status"
