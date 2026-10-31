#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_directional_reference_v1
test ! -e "$base/run1.log"
PYTHONPATH="$PWD/$base:$PYTHONPATH" python3 -B -m pytest -q "$base/test_candidate.py" tests/test_bypass_initial_reference.py > "$base/tests1.log" 2>&1
python3 -B "$base/probe.py" > "$base/run1.log" 2>&1
