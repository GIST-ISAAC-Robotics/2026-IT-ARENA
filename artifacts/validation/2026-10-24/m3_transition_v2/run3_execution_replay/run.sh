#!/usr/bin/env bash
# run3: M3 유한 실행 후보의 동결 소스 관련 회귀 + 저장 4기록 개루프 대조. ROS/Gazebo 주행 없음.
# 사용: run.sh pre | recorded | self | post   (각 단계는 10분 안 실행을 목표로 나눈다)
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_transition_v2/run3_execution_replay'
tool='artifacts/validation/2026-10-24/m3_transition_v2'
# run2 실패 수정: ROS setup은 미정의 변수를 참조하므로 source 동안만 nounset을 끈다.
set +u
source /opt/ros/jazzy/setup.bash
set -u
export PYTHONPATH="scripts:src/arena_autonomy:src/arena_vehicle_interface:${PYTHONPATH:-}"
V='full no_history no_straight strict frozen_run17'
case "${1:-}" in
pre)
  date -u +%FT%TZ > "$out/started_utc.txt"
  python3 -B "$tool/hash_manifest.py" "$out/sha256_before.json"
  mkdir -p "$out/source_snapshot"
  cp src/arena_autonomy/arena_autonomy/{m3_execution_trial,m3_observed_follow,observed_lateral_trial,lateral_intent,local_follow,observation_support,scene_motion,local_path,local_scene,occupancy_timing,linear_observation}.py \
     tests/test_m3_execution_trial.py "$tool/replay_execution.py" "$tool/analyze_run17.py" "$tool/hash_manifest.py" "$out/source_snapshot/"
  python3 -B -m pytest -q -p no:cacheprovider --junitxml="$out/regression.xml" \
    tests/test_m3_execution_trial.py tests/test_m3_observed_follow.py tests/test_observed_lateral_trial.py \
    tests/test_lateral_intent.py tests/test_lateral_sweep_review.py tests/test_observation_support.py \
    tests/test_local_follow.py tests/test_surface_contact_eval.py >"$out/regression.log" 2>&1
  rc=$?
  printf 'pytest_exit_code=%s\n' "$rc" >>"$out/regression.log"
  tail -n 3 "$out/regression.log"
  exit "$rc";;
recorded)
  grep -q '^pytest_exit_code=0$' "$out/regression.log" || { echo replay_skipped_due_to_regression_failure; exit 3; }
  python3 -B "$tool/replay_execution.py" "$out/recorded" --variants $V --solo-variants full strict frozen_run17 >"$out/replay_recorded.log" 2>&1
  rc=$?
  printf 'replay_exit_code=%s\n' "$rc" >>"$out/replay_recorded.log"
  tail -n 2 "$out/replay_recorded.log"
  exit "$rc";;
self)
  grep -q '^pytest_exit_code=0$' "$out/regression.log" || { echo replay_skipped_due_to_regression_failure; exit 3; }
  python3 -B "$tool/replay_execution.py" "$out/self_steering" --self-steering --variants $V --solo-variants full strict frozen_run17 >"$out/replay_self.log" 2>&1
  rc=$?
  printf 'replay_exit_code=%s\n' "$rc" >>"$out/replay_self.log"
  tail -n 2 "$out/replay_self.log"
  exit "$rc";;
post)
  python3 -B "$tool/hash_manifest.py" "$out/sha256_after.json"
  if cmp -s "$out/sha256_before.json" "$out/sha256_after.json"; then echo hashes_unchanged > "$out/hash_check.txt"; else echo HASHES_CHANGED > "$out/hash_check.txt"; fi
  ps -eo pid,etimes,cmd | grep -E 'replay_execution|pytest' | grep -v grep > "$out/leftover_processes.txt" || true
  printf 'leftover_count=%s\n' "$(wc -l < "$out/leftover_processes.txt")" >> "$out/hash_check.txt"
  date -u +%FT%TZ > "$out/finished_utc.txt"
  cat "$out/hash_check.txt";;
*) echo 'usage: run.sh pre|recorded|self|post'; exit 2;;
esac
