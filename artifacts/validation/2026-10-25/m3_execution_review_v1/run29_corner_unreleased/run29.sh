#!/usr/bin/env bash
# run29: 해제되지 않은 M3 상태로 run24 첫 코너 저장 입력을 재생(개루프 주입). Gazebo/ROS 주행 없음.
set -uo pipefail
ROOT="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
A="artifacts/validation/2026-10-25/m3_execution_review_v1"
D="$A/run29_corner_unreleased"
cd "$ROOT" || exit 90
test ! -e "$D/replay" || exit 91
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
FILES=(src/arena_autonomy/arena_autonomy/*.py "$A/compare_corner_active.py" "$A/run24_first_curve/run/follow_decisions.jsonl")
sha256sum "${FILES[@]}" > "$D/sha256_before.txt"
CMD=(python3 -B "$A/compare_corner_active.py" "$A/run24_first_curve/run/follow_decisions.jsonl" --inject-s 20.0 --curve-s 23.0 35.5 --output "$D/replay")
printf '%q ' "${CMD[@]}" > "$D/command.txt"; echo >> "$D/command.txt"
"${CMD[@]}" > "$D/stdout.txt" 2> "$D/stderr.txt"
echo $? > "$D/exitcode.txt"
sha256sum "${FILES[@]}" > "$D/sha256_after.txt"
cmp -s "$D/sha256_before.txt" "$D/sha256_after.txt"; echo "sha_unchanged_rc=$?" >> "$D/exitcode.txt"
ps -eo pid,etime,cmd | grep -E 'compare_corner|gz sim|ros2' | grep -v grep > "$D/residual.txt" || true
