#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/partial_observation_v1
test ! -e "$base/run5_tests.log"
python3 -B -m pytest -q tests/test_partial_observation.py tests/test_lidar_observation.py tests/test_lidar_safety.py tests/test_recent_observation_safety.py tests/test_g1_solo_candidate_validation.py tests/test_r0_safety_contract.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_source_provenance_r0_plus_b3.py tests/test_source_contract_helpers.py > "$base/run5_tests.log" 2>&1
python3 -B scripts/validate_sensor_input_ros.py --partial-observation-policy bounded_side_v1 --output "$base/run5_ros" > "$base/run5_ros.log" 2>&1
colcon build --symlink-install --packages-select arena_autonomy arena_bringup arena_vehicle_interface > "$base/run5_build.log" 2>&1
tail -4 "$base/run5_tests.log"
cat "$base/run5_ros.log"
tail -2 "$base/run5_build.log"
