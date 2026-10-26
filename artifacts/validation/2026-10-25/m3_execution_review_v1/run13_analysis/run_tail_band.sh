#!/usr/bin/env bash
set -uo pipefail
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
A=artifacts/validation/2026-10-25/m3_execution_review_v1
export PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts:tests
timeout 1800 python3 -B "$A/run13_analysis/tail_band_counterexample.py" "$A/run13_selection_closed_loop/run/follow_decisions.jsonl" --output "$A/run13_analysis/tail_band_counterexample.json" > "$A/run13_analysis/tail_band_stdout.txt" 2> "$A/run13_analysis/tail_band_stderr.txt"
echo $? > "$A/run13_analysis/tail_band_exit_code.txt"
