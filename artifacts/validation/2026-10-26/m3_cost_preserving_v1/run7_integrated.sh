#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
BASE=artifacts/validation/2026-10-26/m3_cost_preserving_v1
O="$BASE/run7_integrated"
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
python3 -B -m pytest -q tests/test_path_projection_exact.py tests/test_returned_ray_window.py tests/test_m3*.py tests/test_lateral_intent.py tests/test_observed_lateral_trial.py tests/test_scan_topology.py tests/test_source_provenance_r0_plus_b3.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py > "$O/tests_stdout.txt" 2> "$O/tests_stderr.txt"
code=$?
printf '%s\n' "$code" > "$O/tests_exit_code.txt"
if [ "$code" -ne 0 ]; then exit "$code"; fi
python3 -B "$BASE/verify_integrated.py" --full --output "$O/data" > "$O/stdout.txt" 2> "$O/stderr.txt"
code=$?
printf '%s\n' "$code" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$code"
