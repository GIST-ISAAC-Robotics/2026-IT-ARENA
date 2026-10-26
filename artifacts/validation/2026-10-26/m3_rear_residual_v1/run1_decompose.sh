#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
A=artifacts/validation/2026-10-25/m3_execution_review_v1
D=artifacts/validation/2026-10-26/m3_rear_residual_v1/run1_decompose
[ ! -e "$D" ] || exit 91
mkdir -p "$D"
sha256sum src/arena_autonomy/arena_autonomy/m3_execution_trial.py artifacts/validation/2026-10-26/m3_rear_residual_v1/decompose_release_residual.py > "$D/before.sha256"
python3 -I -B artifacts/validation/2026-10-26/m3_rear_residual_v1/decompose_release_residual.py --output "$D/result.json" \
  "$A/run17_observed_release_closed_loop/run/follow_decisions.jsonl" \
  "$A/run21_right_encounter/run/follow_decisions.jsonl" \
  "$A/run19_stop_restart/run/follow_decisions.jsonl" \
  "$A/run32_corner_after_threat/run/follow_decisions.jsonl" \
  "$A/run35_committed_surface_corner/run/follow_decisions.jsonl" > "$D/stdout.txt" 2> "$D/stderr.txt"
code=$?
printf '%s\n' "$code" > "$D/exit_code.txt"
sha256sum -c "$D/before.sha256" > "$D/source_check.txt" 2>&1
exit "$code"
