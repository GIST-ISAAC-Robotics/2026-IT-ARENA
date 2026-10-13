#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/gapped_boundary_noise_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_fragmented_observation.py tests/test_fragmented_follow.py tests/test_gapped_boundary_evidence.py tests/test_local_follow.py tests/test_follow_overlap.py > "$base/tests.log" 2>&1
tail -8 "$base/tests.log"
python3 -B scripts/validate_fragmented_observation.py --output "$base/results" > "$base/results.log" 2>&1
tail -5 "$base/results.log"
