#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_integration_regression_v1
test ! -e "$base/manifest_fix_v1.log"
set +e
python3 -B -m pytest -q -rs -p no:cacheprovider tests/test_source_contract_helpers.py tests/test_official_start_ros_fixture.py tests/test_pipeline_fault_validation.py tests/test_source_provenance_r0_plus_b3.py tests/test_anchor_set_provenance.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_corner_follow.py tests/test_r0_safety_contract.py tests/test_multi_vehicle_g0.py tests/test_scheduled_apply.py tests/test_scheduled_command.py > "$base/manifest_fix_v1.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/manifest_fix_v1.exit"
exit "$result"
