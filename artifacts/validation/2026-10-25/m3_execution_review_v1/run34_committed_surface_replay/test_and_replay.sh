#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
D=artifacts/validation/2026-10-25/m3_execution_review_v1/run34_committed_surface_replay
[ ! -e "$D/exit_code.txt" ] || exit 91
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
TESTS=(tests/test_m3_surface_commitment.py tests/test_m3_small_center_straight.py
tests/test_m3_release_gate_review.py tests/test_m3_baseline_release.py tests/test_m3_execution_trial.py
tests/test_m3_command_selection.py tests/test_m3_execution_review.py tests/test_m3_execution_integration.py
tests/test_m3_observed_follow.py tests/test_lateral_intent.py tests/test_source_provenance_r0_plus_b3.py
tests/test_corner_follow.py tests/test_g1_dense_stop.py tests/test_physics_step_option.py tests/test_g1_traffic_eval.py)
sha256sum src/arena_autonomy/arena_autonomy/*.py src/arena_bringup/launch/g1_follow.launch.py scripts/validate_g1_traffic.py config/tests/m3_*json "${TESTS[@]}" > "$D/before.sha256"
tar -cf "$D/source_tested.tar" src/arena_autonomy/arena_autonomy/*.py src/arena_bringup/launch/g1_follow.launch.py scripts/validate_g1_traffic.py config/tests/m3_*json "${TESTS[@]}"
printf '%q ' python3 -B -m pytest -p no:cacheprovider "${TESTS[@]}" -q > "$D/command.txt"
python3 -B -m pytest -p no:cacheprovider "${TESTS[@]}" -q --junitxml="$D/pytest.xml" > "$D/stdout.txt" 2> "$D/stderr.txt"
result_code=$?
printf '%s\n' "$result_code" > "$D/exit_code.txt"
sha256sum -c "$D/before.sha256" > "$D/source_check.txt"
if [ "$result_code" != 0 ]; then exit "$result_code"; fi
A=artifacts/validation/2026-10-25/m3_execution_review_v1
python3 -B "$A/compare_surface_commitment.py" --output "$D/replay" "$A/run32_corner_after_threat/run/follow_decisions.jsonl" "$A/run27_small_center_stop_restart/run/follow_decisions.jsonl" "$A/run24_first_curve/run/follow_decisions.jsonl" "$A/run17_observed_release_closed_loop/run/follow_decisions.jsonl" "$A/run21_right_encounter/run/follow_decisions.jsonl" > "$D/replay_stdout.txt" 2> "$D/replay_stderr.txt"
replay_code=$?
printf '%s\n' "$replay_code" > "$D/replay_exit_code.txt"
sha256sum -c "$D/before.sha256" > "$D/source_check_after_replay.txt"
exit "$replay_code"
