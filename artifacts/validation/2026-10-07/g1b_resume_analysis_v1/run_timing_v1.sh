#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-10-07/g1b_resume_analysis_v1
[ -s "$A/command_timing_old.json" ] || python3 -B "$A/command_timing.py" "$A/source_before" > "$A/command_timing_old.json"
python3 -B "$A/command_timing.py" . > "$A/command_timing_${TAG:-new}.json"
cat "$A/command_timing_old.json" "$A/command_timing_${TAG:-new}.json"
