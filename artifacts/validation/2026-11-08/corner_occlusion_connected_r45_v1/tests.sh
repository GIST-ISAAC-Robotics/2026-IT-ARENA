#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
A=artifacts/validation/2026-11-08/corner_occlusion_connected_r45_v1
python3 -B -m pytest -q -p no:cacheprovider --tb=short tests/test_corner_occlusion_support.py tests/test_corner_boundary_support.py tests/test_local_follow.py tests/test_local_follow_validation.py tests/test_piecewise_wall.py tests/test_piecewise_noise.py tests/test_traffic_piecewise.py tests/test_pass_release_trial.py tests/test_source_provenance_r0_plus_b3.py > "$A/tests_v1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$A/tests_v1.exit"
exit "$rc"
