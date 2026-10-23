#!/usr/bin/env bash
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$ROOT"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 scripts/probe_recent_straight_precision.py --output artifacts/validation/2026-10-17/g1_recent_straight_precision_v1 > artifacts/validation/2026-10-17/g1_recent_straight_precision_v1.stdout.log 2>&1
