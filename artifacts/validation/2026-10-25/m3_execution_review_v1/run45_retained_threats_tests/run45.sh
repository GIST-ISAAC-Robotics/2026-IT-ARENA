#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
D=artifacts/validation/2026-10-25/m3_execution_review_v1/run45_retained_threats_tests
[ ! -e "$D/exit_code.txt" ] || exit 91
TESTS=(tests/test_m3_retained_threats.py tests/test_m3_surface_commitment.py tests/test_m3_small_center_straight.py
tests/test_m3_release_gate_review.py tests/test_m3_baseline_release.py tests/test_m3_execution_trial.py
tests/test_m3_command_selection.py tests/test_m3_execution_review.py tests/test_m3_execution_integration.py
tests/test_m3_observed_follow.py tests/test_lateral_intent.py tests/test_source_provenance_r0_plus_b3.py
tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_g1_traffic_eval.py)
sha256sum src/arena_autonomy/arena_autonomy/*.py scripts/validate_g1_traffic.py config/tests/m3_*json "${TESTS[@]}" > "$D/before.sha256"
tar -cf "$D/source_tested.tar" src/arena_autonomy/arena_autonomy/*.py scripts/validate_g1_traffic.py config/tests/m3_*json "${TESTS[@]}"
python3 -B -m pytest -p no:cacheprovider "${TESTS[@]}" -q --junitxml="$D/pytest.xml" > "$D/stdout.txt" 2> "$D/stderr.txt"
result_code=$?
printf '%s\n' "$result_code" > "$D/exit_code.txt"
sha256sum -c "$D/before.sha256" > "$D/source_check.txt"
exit "$result_code"
