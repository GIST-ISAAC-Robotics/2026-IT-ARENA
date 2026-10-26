#!/usr/bin/env bash
# run26: 기본 false small_center_straight 연결의 관련 회귀·빌드·설치본 확인·기존 기록 OFF 재생 동일성.
# Gazebo/ROS 주행 없음. 결과 파일이 있으면 다시 실행하지 않는다.
set -uo pipefail
ROOT="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
A="artifacts/validation/2026-10-25/m3_execution_review_v1"
D="$A/run26_small_center_integration"
cd "$ROOT" || exit 90
if [ -e "$D/test_exit_code.txt" ]; then echo "run26 already executed; refusing duplicate" | tee -a "$D/duplicate_refused.txt"; exit 91; fi
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
TESTS=(tests/test_m3_small_center_straight.py tests/test_m3_release_gate_review.py tests/test_m3_baseline_release.py
       tests/test_m3_execution_trial.py tests/test_m3_command_selection.py tests/test_m3_execution_review.py
       tests/test_m3_execution_integration.py tests/test_m3_observed_follow.py tests/test_lateral_intent.py
       tests/test_source_provenance_r0_plus_b3.py tests/test_corner_follow.py tests/test_g1_dense_stop.py
       tests/test_physics_step_option.py tests/test_g1_traffic_eval.py)
FILES=(src/arena_autonomy/arena_autonomy/*.py src/arena_bringup/launch/g1_follow.launch.py
       scripts/validate_g1_traffic.py scripts/g1_traffic_eval.py scripts/replay_follow_trace.py
       config/tests/m3_*_r0_sha256.json "${TESTS[@]}" "$A/compare_small_center.py" "$A/prepare_small_center_contract.py"
       "$A/run13_selection_closed_loop/run/follow_decisions.jsonl" "$A/run9_closed_loop_left/run/follow_decisions.jsonl")
hash_all() { sha256sum "${FILES[@]}"; }
proc() { ps -eo pid,etime,cmd | grep -E 'gz sim|ros2|parameter_bridge|ruby|validate_g1|pytest|compare_small' | grep -v grep || true; }
{ echo "ROS_DISTRO=${ROS_DISTRO:-unset}"; echo "PYTHONPATH=$PYTHONPATH"; echo "started_utc=$(date -u +%FT%TZ)"; } > "$D/environment.txt"
proc > "$D/preexisting_processes.txt"
hash_all > "$D/source_sha256_before.txt"
mkdir -p "$D/source_snapshot" && tar -cf "$D/source_snapshot/sources.tar" "${FILES[@]:0:$((${#FILES[@]}-2))}"

# 1) 관련 회귀
printf '%q ' python3 -B -m pytest -p no:cacheprovider "${TESTS[@]}" -q --junitxml="$D/pytest.xml" > "$D/test_command.txt"; echo >> "$D/test_command.txt"
python3 -B -m pytest -p no:cacheprovider "${TESTS[@]}" -q --junitxml="$D/pytest.xml" > "$D/test_stdout.txt" 2> "$D/test_stderr.txt"
echo $? > "$D/test_exit_code.txt"
hash_all > "$D/source_sha256_after_tests.txt"

# 2) 두 패키지 빌드와 새 셸 설치본 확인
BUILD=(colcon build --symlink-install --packages-select arena_autonomy arena_bringup --executor sequential --event-handlers console_direct+)
printf '%q ' "${BUILD[@]}" > "$D/build_command.txt"; echo >> "$D/build_command.txt"
"${BUILD[@]}" > "$D/build_stdout.txt" 2> "$D/build_stderr.txt"
echo $? > "$D/build_exit_code.txt"
env -i HOME="$HOME" PATH=/usr/bin:/bin bash -c "cd \"$ROOT\" && set +u && source /opt/ros/jazzy/setup.bash && source install/local_setup.bash && set -u && python3 -B $A/run12_selection_integration/install_import_check.py $D/install_import_results.json" > "$D/install_import_stdout.txt" 2> "$D/install_import_stderr.txt"
echo $? > "$D/install_import_exit_code.txt"
hash_all > "$D/source_sha256_after_build.txt"

# 3) 기본 OFF 기록(run13, run9) 원래 판단 재생 동일성
for r in run13_selection_closed_loop run9_closed_loop_left; do
  python3 -B scripts/replay_follow_trace.py "$A/$r/run/follow_decisions.jsonl" --output "$D/replay_$r.json" > "$D/replay_${r}_stdout.txt" 2> "$D/replay_${r}_stderr.txt"
  echo $? > "$D/replay_${r}_exit_code.txt"
done

hash_all > "$D/source_sha256_after.txt"
diff "$D/source_sha256_before.txt" "$D/source_sha256_after.txt" > "$D/hash_diff.txt"; echo "diff_exit=$?" >> "$D/hash_diff.txt"
sleep 2
proc > "$D/residual_processes.txt"
echo "finished_utc=$(date -u +%FT%TZ)" >> "$D/environment.txt"
echo "RUN26_DONE tests=$(cat "$D/test_exit_code.txt") build=$(cat "$D/build_exit_code.txt") install=$(cat "$D/install_import_exit_code.txt") run13=$(cat "$D/replay_run13_selection_closed_loop_exit_code.txt") run9=$(cat "$D/replay_run9_closed_loop_left_exit_code.txt")" | tee "$D/DONE.txt"
