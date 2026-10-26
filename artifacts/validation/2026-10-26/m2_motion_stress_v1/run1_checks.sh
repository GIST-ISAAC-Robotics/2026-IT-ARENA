#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m2_motion_stress_v1/run1_checks
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
python3 -B -m pytest -q tests/test_g1_motion_stress.py tests/test_motion_impairment.py tests/test_g1_lidar_stress.py tests/test_m3_lidar_stress_cli.py tests/test_source_provenance_r0_plus_b3.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_corner_follow.py > "$O/stdout.txt" 2> "$O/stderr.txt"
code=$?
printf '%s\n' "$code" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$code"
