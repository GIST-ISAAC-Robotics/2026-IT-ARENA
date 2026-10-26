#!/usr/bin/env bash
# run25: 작은 중앙 보정 생략(선택) 저장 입력 개루프 비교. Gazebo/ROS 주행 없음. 결과 폴더가 있으면 거절.
set -uo pipefail
ROOT="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
A="artifacts/validation/2026-10-25/m3_execution_review_v1"
D="$A/run25_center_hold_compare"
cd "$ROOT" || exit 90
test ! -e "$D/replay" || exit 91
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
FILES=(src/arena_autonomy/arena_autonomy/*.py tests/test_m3_small_center_straight.py "$A/compare_small_center.py"
       "$A/run17_observed_release_closed_loop/run/follow_decisions.jsonl" "$A/run19_stop_restart/run/follow_decisions.jsonl"
       "$A/run21_right_encounter/run/follow_decisions.jsonl" "$A/run24_first_curve/run/follow_decisions.jsonl")
sha256sum "${FILES[@]}" > "$D/replay_sha256_before.txt"
cp "$A/compare_small_center.py" "$D/"
date -u +%FT%TZ > "$D/replay_start.txt"
python3 -B "$A/compare_small_center.py" "$A/run19_stop_restart/run/follow_decisions.jsonl" \
  "$A/run17_observed_release_closed_loop/run/follow_decisions.jsonl" "$A/run21_right_encounter/run/follow_decisions.jsonl" \
  "$A/run24_first_curve/run/follow_decisions.jsonl" --output "$D/replay" > "$D/replay_stdout.txt" 2> "$D/replay_stderr.txt"
rc=$?
echo "$rc" > "$D/replay_exitcode.txt"
date -u +%FT%TZ > "$D/replay_end.txt"
sha256sum "${FILES[@]}" > "$D/replay_sha256_after.txt"
cmp -s "$D/replay_sha256_before.txt" "$D/replay_sha256_after.txt"; echo "sha_unchanged_rc=$?" >> "$D/replay_exitcode.txt"
ps -eo pid,etime,cmd | grep -E 'compare_small_center|gz sim|ros2' | grep -v grep > "$D/replay_residual.txt" || true
exit "$rc"
