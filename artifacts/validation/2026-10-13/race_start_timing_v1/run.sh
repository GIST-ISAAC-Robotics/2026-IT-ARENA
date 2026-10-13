#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/race_start_timing_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_start_signal_timing.py tests/test_official_start_signal.py tests/test_official_vision_pipeline.py tests/test_vision_process.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B scripts/validate_start_signal_timing.py --output "$base/run" > "$base/run.log" 2>&1
tail -10 "$base/run.log"
