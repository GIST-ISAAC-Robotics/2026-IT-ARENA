#!/usr/bin/env bash
# 저장 판단 재생 일치만 확인한다. ROS/Gazebo 실행 없음.
set -u
REPO="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
OUT="$REPO/artifacts/validation/2026-10-24/m3_lateral_hold_screen_v1"
cd "$REPO" || exit 90
python3 -I "$OUT/replay_check.py" > "$OUT/replay_stdout.json" 2> "$OUT/replay_stderr.log"
code=$?
printf '%s\n' "$code" > "$OUT/replay_exit.txt"
exit "$code"
