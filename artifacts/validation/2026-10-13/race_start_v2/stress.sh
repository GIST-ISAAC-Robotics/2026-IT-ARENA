#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/race_start_v2
test ! -e "$base/stress_tests.log"
python3 -B -m pytest -q tests/test_official_start_stress.py > "$base/stress_tests.log" 2>&1
tail -10 "$base/stress_tests.log"
test ! -e "$base/stress.log"
python3 -B scripts/validate_official_start.py --output "$base/stress.json" > "$base/stress.log" 2>&1
tail -10 "$base/stress.log"
