#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
a='artifacts/validation/2026-11-14/continuous_pass_preview_drive_r513_v1'
export PYTHONDONTWRITEBYTECODE=1
python3 -B "$a/execution_v1/attribute_occupied.py" > "$a/execution_v1/attribute.log" 2>&1
