#!/usr/bin/env bash
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
A=artifacts/validation/2026-10-25/m3_execution_review_v1
PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts python3 -B "$A/run13_analysis/release_margin.py" "$A/run13_selection_closed_loop/run/follow_decisions.jsonl" "$A/run9_closed_loop_left/run/follow_decisions.jsonl" "$A/run13_analysis/release_margin.json"
echo "EXIT=$?"
