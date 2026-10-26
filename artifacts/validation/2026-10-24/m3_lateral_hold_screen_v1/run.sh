#!/usr/bin/env bash
# 순수 오프라인 계산. ROS/Gazebo 실행 없음. 종료 코드를 파일에 남긴다.
set -u
REPO="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
OUT="$REPO/artifacts/validation/2026-10-24/m3_lateral_hold_screen_v1"
cd "$REPO" || exit 90
python3 -I "$OUT/analyze.py" > "$OUT/stdout.json" 2> "$OUT/stderr.log"
code=$?
printf '%s\n' "$code" > "$OUT/exit.txt"
exit "$code"
