#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-28/traffic_release_fringe_v1
test ! -e "$base/regression.log"
set +e
python3 -B -m pytest -q -p no:cacheprovider tests/test_traffic_release_fringe.py tests/test_pass_release_trial.py tests/test_guard_curve_trial.py tests/test_rear_fringe_trial.py tests/test_g1_guard_partial_coupling.py tests/test_traffic_recovery_contract.py tests/test_source_provenance_r0_plus_b3.py tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_r0_safety_contract.py tests/test_scheduled_apply.py tests/test_scheduled_command.py > "$base/regression.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/regression.exit"
tail -45 "$base/regression.log"
exit "$result"
