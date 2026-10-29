#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/rotation_reobservation_review_v1
/usr/bin/python3 -B "$base/probe_candidate.py" "$base/run3/fixture.json" --output "$base/run3/candidate_diagnostics_v2.json" > "$base/run3/candidate_diagnostics_v2.log" 2>&1
/usr/bin/python3 -B "$base/probe_candidate.py" "$base/run2/fixture.json" --output "$base/run2/candidate_v2.json" > "$base/run2/candidate_v2.log" 2>&1
