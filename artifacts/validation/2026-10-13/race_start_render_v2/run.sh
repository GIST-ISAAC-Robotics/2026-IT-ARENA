#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/race_start_render_v2
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_start_signal_render_fixture.py tests/test_start_signal_geometry.py tests/test_official_start_signal.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B scripts/validate_start_signal_render.py --output "$base/run" --timeout 180 > "$base/run.log" 2>&1
tail -10 "$base/run.log"
