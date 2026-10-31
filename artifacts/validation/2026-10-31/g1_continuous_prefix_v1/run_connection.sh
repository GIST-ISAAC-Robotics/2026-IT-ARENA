#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g1_continuous_prefix_v1
test ! -e "$base/connection_tests1.log"
python3 -B -m pytest -q -p no:cacheprovider tests/test_continuous_prefix.py tests/test_short_prefix_connection.py tests/test_short_prefix_origin.py tests/test_short_corner_support.py tests/test_corner_corridor_module.py tests/test_corridor_observer.py tests/test_corridor_follow.py tests/test_corridor_roles.py tests/test_corridor_input_recovery.py tests/test_follow_profile_bundle.py tests/test_jetson_bundle_dependencies.py tests/test_source_provenance_r0_plus_b3.py > "$base/connection_tests1.log" 2>&1
python3 -B "$base/verify_connection.py" > "$base/connection_replay1.log" 2>&1
