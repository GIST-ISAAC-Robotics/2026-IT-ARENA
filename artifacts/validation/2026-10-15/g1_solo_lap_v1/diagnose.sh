#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 -B artifacts/validation/2026-10-15/g1_solo_lap_v1/diagnose.py > artifacts/validation/2026-10-15/g1_solo_lap_v1/diagnosis.log 2>&1
cat artifacts/validation/2026-10-15/g1_solo_lap_v1/diagnosis.log
