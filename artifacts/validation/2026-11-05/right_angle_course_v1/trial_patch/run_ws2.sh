#!/usr/bin/env bash
# ws2(격리 시험 후보)로 run.sh 실행. 사용: run_ws2.sh <이름> [월드 디렉터리 이름]
D="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-05/right_angle_course_v1"
N="${1:?name}"
WS_DIR="$HOME/arena_scratch/right_angle_v1/ws2" WORLD="${2:-course_world}" bash "$D/run.sh" "$N"
