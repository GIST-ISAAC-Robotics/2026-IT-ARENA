#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/race_start_v1
test ! -e "$base/layout_audit.log"
python3 -B scripts/audit_start_signal_layout.py --output "$base/layout_audit.json" > "$base/layout_audit.log" 2>&1
tail -6 "$base/layout_audit.log"
