#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/official_v2026_10_24_update/run8_display_regression
mkdir "$O"
trap 'rc=$?; echo "$rc" > "$O/exit_code.txt"' EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_corner_follow.py tests/test_follow_profile.py tests/test_follow_profile_bundle.py tests/test_g1_dense_stop.py tests/test_multi_vehicle_g0.py tests/test_official_start_ros_fixture.py tests/test_physics_step_option.py tests/test_pipeline_fault_validation.py tests/test_r0_safety_contract.py tests/test_scheduled_apply.py tests/test_source_provenance_r0_plus_b3.py --junitxml="$O/junit.xml" > "$O/tests.log" 2>&1
