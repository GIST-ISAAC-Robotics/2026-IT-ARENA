#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/rotation_stress_v1
test ! -e "$base/surface_packaging_v1.log"
/usr/bin/python3 -B -m pytest -q tests/test_follow_profile_bundle.py tests/test_jetson_bundle_dependencies.py tests/test_source_provenance_r0_plus_b3.py tests/test_anchor_set_provenance.py "$base/test_contract.py" "$base/test_sensor_stress.py" > "$base/surface_packaging_v1.log" 2>&1
