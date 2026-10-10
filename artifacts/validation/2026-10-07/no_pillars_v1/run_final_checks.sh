#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
out='artifacts/validation/2026-10-07/no_pillars_v1'
test ! -e "$out/pytest_final.log"
python3 -B scripts/audit_g1_traffic_coverage.py "$out/after_g1_stopped_s0" --output "$out/after_g1_stopped_s0/postcoverage.json" > "$out/stopped_audit.log"
python3 -B "$out/final_audit.py"
set +e
python3 -B -m pytest -q -p no:cacheprovider tests/test_local_follow.py tests/test_g1_traffic_eval.py tests/test_local_follow_validation.py tests/test_local_scene.py tests/test_g0_geometry.py tests/test_multi_vehicle_g0.py tests/test_r0_safety_contract.py tests/test_local_pursuit.py tests/test_lidar_safety.py tests/test_lidar_observation.py tests/test_lidar_motion.py tests/test_rotating_lidar.py tests/test_lidar_motion_ros.py tests/test_vision_process.py tests/test_g1_traffic_coverage_audit.py tests/test_local_path_range_noise.py tests/test_official_track.py tests/test_runtime_world_options.py tests/test_experimental_facilities.py tests/test_track_variants.py tests/test_g1b_braking_analysis.py > "$out/pytest_final.log" 2>&1
code=$?
set -e
printf '%s\n' "$code" > "$out/pytest_final.exitcode"
tail -30 "$out/pytest_final.log"
exit "$code"
