#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/solo_current_mask_r463_v1'
export PYTHONDONTWRITEBYTECODE=1
python3 -B compare_missing_flank.py base
python3 -B compare_missing_flank.py stage
