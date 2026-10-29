#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_side_scope_v1
python3 -B -m pytest -q -p no:cacheprovider tests/test_same_path_side_scope.py tests/test_same_path_gap_probe.py tests/test_same_path_speed_recheck.py > "$base/static_v1.log" 2>&1
python3 -B artifacts/validation/2026-10-28/traffic_positive_clearance_v1/counterfactual_boundaries.py > "$base/counterfactual_boundaries_v2.log" 2>&1
