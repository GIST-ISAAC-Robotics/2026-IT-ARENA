#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
trial_dir='artifacts/validation/2026-10-29/g2_motion_envelope_v1'
if [ -e "$trial_dir/saved_dense_comparison.json" ] || [ -e "$trial_dir/tests_v3.log" ]; then
  exit 73
fi
/usr/bin/python3 -B "$trial_dir/test_rounded_sweep.py" > "$trial_dir/tests_v3.log" 2>&1
/usr/bin/python3 -B "$trial_dir/compare_dense.py" > "$trial_dir/compare_dense.log" 2>&1
