#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/marker_roles_v2
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_marker_roles.py tests/test_vision_process.py tests/test_official_vision_pipeline.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B scripts/validate_marker_roles.py --output "$base/pixels" > "$base/pixels.log" 2>&1
tail -1 "$base/pixels.log"
