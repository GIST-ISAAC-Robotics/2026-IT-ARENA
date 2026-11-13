#!/bin/bash
set -e
bundle='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/crop_masked_recovery_r438_v1'
root=/home/jinhyeong/it-arena-r438-crop-a
test ! -e "$root"
mkdir "$root"
python3 -m zipfile -e "$bundle/frozen_packages.zip" "$root"
cp -r "$bundle/inputs" "$bundle/stage" "$root/"
cp "$bundle/preparation.json" "$bundle/analyze_saved.py" "$root/"
cd "$root"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$root/src/arena_autonomy:$root/src/arena_vehicle_interface:$root/stage"
python3 -B analyze_saved.py --root "$root" --output "$root/result_v1" > "$root/run_v1.log" 2>&1
