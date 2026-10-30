#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
set -e
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-30/g2_signed_hold_v1
stress=artifacts/validation/2026-10-30/g2_center_stress_v1
/usr/bin/python3 -B "$stress/timeline.py" > "$stress/timeline.log" 2>&1
/usr/bin/python3 -B "$stress/analyze_run.py" > "$stress/analyze_run.log" 2>&1
/usr/bin/python3 -B -m pytest -q "$base/test_candidate.py" > "$base/tests1.log" 2>&1
/usr/bin/python3 -B "$base/shadow.py" > "$base/shadow1.log" 2>&1
