#!/bin/bash
set -e
bundle='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/crop_current_crawl_r440_v1'
root=/home/jinhyeong/it-arena-r440-verify-a
test ! -e "$root"
mkdir "$root"
python3 -m zipfile -e "$bundle/validation_sources_v1.zip" "$root"
cd "$root"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$root/src/arena_autonomy:$root/src/arena_vehicle_interface"
python3 -B run_validation.py > "$root/run_v1.log" 2>&1
