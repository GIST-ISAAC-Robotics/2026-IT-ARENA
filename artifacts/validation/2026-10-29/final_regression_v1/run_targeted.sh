#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/final_regression_v1
test ! -e "$base/final_targeted.log"
set +e
/usr/bin/python3 -B -m pytest -q -rs -p no:cacheprovider tests/test_actor_tail_provenance.py tests/test_g2_handoff_provenance.py tests/test_source_provenance_r0_plus_b3.py tests/test_source_contract_helpers.py tests/test_anchor_set_provenance.py tests/test_corner_follow.py tests/test_follow_profile.py tests/test_follow_profile_bundle.py tests/test_official_start_ros_fixture.py tests/test_pipeline_fault_validation.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py > "$base/final_targeted.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/final_targeted.exit"
exit "$result"