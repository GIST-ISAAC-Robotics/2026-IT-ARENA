#!/usr/bin/env bash
set -o pipefail
repo="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface:$repo/scripts:$repo/tests:${PYTHONPATH:-}"
python3 "$repo/artifacts/validation/2026-10-26/m4_second_start_v1/run3_environment_preflight/preflight.py" "$repo/artifacts/validation/2026-10-26/m4_second_start_v1/run3_environment_preflight"
pre=$?
printf "%s\n" "$pre" > "$repo/artifacts/validation/2026-10-26/m4_second_start_v1/run3_environment_preflight/preflight_exit_code.txt"
if [ "$pre" -ne 0 ]; then exit "$pre"; fi
python3 -m pytest -q tests/test_m4_second_start.py tests/test_m4_operation_trial.py tests/test_m4_operation_cli.py tests/test_source_provenance_r0_plus_b3.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_g1_traffic_eval.py tests/test_g1_traffic_extensions.py
