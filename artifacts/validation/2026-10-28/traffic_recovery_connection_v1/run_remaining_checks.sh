#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
BASE="$PWD/artifacts/validation/2026-10-28/traffic_recovery_connection_v1"
OLD="$PWD/artifacts/validation/2026-10-28/traffic_residual_v1"
export PYTHONPATH="$BASE:$OLD:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
set +e
/usr/bin/python3 -B -m pytest -q tests/test_source_provenance_r0_plus_b3.py tests/test_corner_follow.py tests/test_follow_profile.py tests/test_follow_profile_bundle.py tests/test_g1_dense_stop.py tests/test_multi_vehicle_g0.py tests/test_official_start_ros_fixture.py tests/test_physics_step_option.py tests/test_pipeline_fault_validation.py tests/test_r0_safety_contract.py tests/test_scheduled_apply.py tests/test_scheduled_command.py tests/test_traffic_recovery_contract.py > "$BASE/provenance_dependencies_v2.log" 2>&1
code=$?
printf '%s\n' "$code" > "$BASE/provenance_dependencies_v2.exit"
exit "$code"
